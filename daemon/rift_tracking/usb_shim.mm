// libusb's API (the part declared in libusb.h beside this file) on IOUSBHost.
//
// Transfers complete on one serial dispatch queue, and their callbacks run there, one at a time, as libusb's run one
// at a time on the thread that handles its events. A callback resubmits its transfer at once from where it
// completed: handing it to an event thread first left the camera's next microframes without a request often
// enough to cost whole frames. libusb_handle_events_timeout_completed only waits.
#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOUSBHost/IOUSBHost.h>

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <sys/time.h>
#include <unistd.h>

#include "libusb.h"

namespace {

constexpr long kOculusVendor = 0x2833;

std::atomic<uint64_t> g_iso_bytes{0};

long number(io_registry_entry_t entry, CFStringRef key) {
  long value = -1;
  if (CFTypeRef ref = IORegistryEntryCreateCFProperty(entry, key, kCFAllocatorDefault, 0)) {
    if (CFGetTypeID(ref) == CFNumberGetTypeID())
      CFNumberGetValue(static_cast<CFNumberRef>(ref), kCFNumberLongType, &value);
    CFRelease(ref);
  }
  return value;
}

std::chrono::microseconds duration(const struct timeval *tv, long fallback_us) {
  if (!tv)
    return std::chrono::microseconds(fallback_us);
  return std::chrono::microseconds(static_cast<long long>(tv->tv_sec) * 1000000 + tv->tv_usec);
}

struct TransferState {
  NSMutableData *data = nil;
  std::vector<IOUSBHostIsochronousTransaction> transactions;
  unsigned int packet_length = 0;
};

} // namespace

struct libusb_context {
  std::mutex mutex;
  std::condition_variable arrived; // only waited on, with a timeout: nothing is handed over
  // for threads waiting for the event thread to have served callbacks
  std::mutex waiters;
  std::condition_variable served;
  dispatch_queue_t queue = nil;
};

struct libusb_device {
  libusb_context *context = nullptr;
  io_service_t service = 0;
  int references = 1;
  libusb_device_descriptor descriptor{};
  std::string serial;
};

struct libusb_device_handle {
  libusb_device *device = nullptr;
  IOUSBHostInterface *interfaces[2] = {nil, nil};
  IOUSBHostPipe *pipe = nil;
  uint8_t pipe_address = 0;
};

namespace {

void unref(libusb_device *device) {
  if (--device->references > 0)
    return;
  if (device->service)
    IOObjectRelease(device->service);
  delete device;
}

IOUSBHostInterface *interface(libusb_device_handle *handle, int index) {
  if (index < 0 || index > 1)
    return nil;
  if (handle->interfaces[index])
    return handle->interfaces[index];
  io_iterator_t it = 0;
  if (IORegistryEntryGetChildIterator(handle->device->service, kIOServicePlane, &it) != KERN_SUCCESS)
    return nil;
  io_service_t found = 0;
  while (io_service_t child = IOIteratorNext(it)) {
    if (!found && IOObjectConformsTo(child, "IOUSBHostInterface") &&
        number(child, CFSTR("bInterfaceNumber")) == index)
      found = child;
    else
      IOObjectRelease(child);
  }
  IOObjectRelease(it);
  if (!found)
    return nil;
  NSError *error = nil;
  IOUSBHostInterface *object = [[IOUSBHostInterface alloc] initWithIOService:found
                                                                     options:IOUSBHostObjectInitOptionsNone
                                                                       queue:handle->device->context->queue
                                                                       error:&error
                                                             interestHandler:nil];
  IOObjectRelease(found);
  if (!object)
    std::fprintf(stderr, "rift sensor %s: interface %d cannot be opened: %s\n", handle->device->serial.c_str(), index,
                 error.localizedDescription.UTF8String);
  handle->interfaces[index] = object;
  return object;
}

void dropPipe(libusb_device_handle *handle) {
  if (!handle->pipe)
    return;
  // synchronous: every queued request has been handed to its completion handler when this returns
  [handle->pipe abortWithOption:IOUSBHostAbortOptionSynchronous error:nil];
  handle->pipe = nil;
}

} // namespace

extern "C" {

int libusb_init(libusb_context **ctx) {
  auto *context = new libusb_context;
  context->queue = dispatch_queue_create("metalvr.rift-sensor.usb", DISPATCH_QUEUE_SERIAL);
  *ctx = context;
  return 0;
}

void libusb_exit(libusb_context *ctx) { delete ctx; }

ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list) {
  std::vector<libusb_device *> devices;
  io_iterator_t it = 0;
  if (IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceMatching("IOUSBHostDevice"), &it) == KERN_SUCCESS) {
    while (io_service_t service = IOIteratorNext(it)) {
      if (number(service, CFSTR("idVendor")) != kOculusVendor) {
        IOObjectRelease(service);
        continue;
      }
      auto *device = new libusb_device;
      device->context = ctx;
      device->service = service;
      libusb_device_descriptor &d = device->descriptor;
      d.bLength = 18;
      d.bDescriptorType = 1;
      // IOKit's "Device Speed": 2 is high speed, 3 and up SuperSpeed
      d.bcdUSB = number(service, CFSTR("Device Speed")) >= 3 ? 0x0300 : 0x0200;
      d.idVendor = static_cast<uint16_t>(number(service, CFSTR("idVendor")));
      d.idProduct = static_cast<uint16_t>(number(service, CFSTR("idProduct")));
      d.bcdDevice = static_cast<uint16_t>(number(service, CFSTR("bcdDevice")));
      d.bNumConfigurations = 1;
      if (CFTypeRef serial =
              IORegistryEntryCreateCFProperty(service, CFSTR("kUSBSerialNumberString"), kCFAllocatorDefault, 0)) {
        if (CFGetTypeID(serial) == CFStringGetTypeID())
          device->serial = [(__bridge NSString *)serial UTF8String];
        CFRelease(serial);
      }
      d.iSerialNumber = device->serial.empty() ? 0 : 1;
      devices.push_back(device);
    }
    IOObjectRelease(it);
  }
  auto **out = static_cast<libusb_device **>(std::calloc(devices.size() + 1, sizeof(libusb_device *)));
  if (!out) {
    for (libusb_device *device : devices)
      unref(device);
    return LIBUSB_ERROR_NO_MEM;
  }
  for (size_t i = 0; i < devices.size(); ++i)
    out[i] = devices[i];
  *list = out;
  return static_cast<ssize_t>(devices.size());
}

void libusb_free_device_list(libusb_device **list, int unref_devices) {
  if (!list)
    return;
  if (unref_devices)
    for (libusb_device **device = list; *device; ++device)
      unref(*device);
  std::free(list);
}

int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc) {
  *desc = dev->descriptor;
  return 0;
}

int libusb_open(libusb_device *dev, libusb_device_handle **handle) {
  auto *opened = new libusb_device_handle;
  opened->device = dev;
  ++dev->references;
  // the control interface carries every request; a sensor another process holds fails here
  if (!interface(opened, 0)) {
    libusb_close(opened);
    return LIBUSB_ERROR_IO;
  }
  *handle = opened;
  return 0;
}

void libusb_close(libusb_device_handle *handle) {
  if (!handle)
    return;
  dropPipe(handle);
  for (IOUSBHostInterface *__strong &object : handle->interfaces) {
    [object destroy];
    object = nil;
  }
  unref(handle->device);
  delete handle;
}

libusb_device *libusb_get_device(libusb_device_handle *handle) { return handle->device; }

int libusb_get_string_descriptor_ascii(libusb_device_handle *handle, uint8_t index, unsigned char *data, int length) {
  const libusb_device *device = handle->device;
  if (index == 0 || index != device->descriptor.iSerialNumber || length < 1)
    return LIBUSB_ERROR_NOT_FOUND;
  const int n = std::min<int>(static_cast<int>(device->serial.size()), length - 1);
  std::memcpy(data, device->serial.data(), static_cast<size_t>(n));
  data[n] = 0;
  return n;
}

// nothing on macOS claims these sensors
int libusb_set_auto_detach_kernel_driver(libusb_device_handle *, int) { return 0; }

int libusb_claim_interface(libusb_device_handle *handle, int interface_number) {
  IOUSBHostInterface *object = interface(handle, interface_number);
  if (!object)
    return LIBUSB_ERROR_NOT_FOUND;
  // A sensor whose last user died is still streaming, and refuses to be set up again until it is stopped.
  if (interface_number == 1)
    [object selectAlternateSetting:0 error:nil];
  return 0;
}

int libusb_set_interface_alt_setting(libusb_device_handle *handle, int interface_number, int alternate_setting) {
  IOUSBHostInterface *object = interface(handle, interface_number);
  if (!object)
    return LIBUSB_ERROR_NOT_FOUND;
  dropPipe(handle);
  NSError *error = nil;
  if (![object selectAlternateSetting:static_cast<NSUInteger>(alternate_setting) error:&error]) {
    std::fprintf(stderr, "rift sensor %s: alternate setting %d refused: %s\n", handle->device->serial.c_str(),
                 alternate_setting, error.localizedDescription.UTF8String);
    return LIBUSB_ERROR_IO;
  }
  return 0;
}

int libusb_control_transfer(libusb_device_handle *handle, uint8_t request_type, uint8_t request, uint16_t value,
                            uint16_t index, unsigned char *data, uint16_t length, unsigned int timeout) {
  IOUSBHostInterface *via = interface(handle, 0);
  if (!via)
    return LIBUSB_ERROR_NO_DEVICE;
  IOUSBDeviceRequest r{};
  r.bmRequestType = request_type;
  r.bRequest = request;
  r.wValue = value;
  r.wIndex = index;
  r.wLength = length;
  const bool in = (request_type & 0x80) != 0;
  NSMutableData *buffer = nil;
  if (length)
    buffer = in ? [NSMutableData dataWithLength:length] : [NSMutableData dataWithBytes:data length:length];
  // A sensor now and then refuses a request while its video is starting and takes the same one a moment later.
  // The requests made of it set or read a register, so asking again is safe.
  NSError *error = nil;
  NSUInteger done = 0;
  BOOL sent = NO;
  for (int attempt = 0; attempt < 3 && !sent; ++attempt) {
    if (attempt)
      usleep(20000);
    if (!in && length)
      buffer = [NSMutableData dataWithBytes:data length:length];
    sent = [via sendDeviceRequest:r
                             data:buffer
                 bytesTransferred:&done
                completionTimeout:timeout ? timeout / 1000.0 : 0
                            error:&error];
  }
  if (!sent) {
    std::fprintf(stderr, "rift sensor %s: request %02x %02x value %04x index %04x failed three times: %s\n",
                 handle->device->serial.c_str(), request_type, request, value, index,
                 error.localizedDescription.UTF8String);
    return LIBUSB_ERROR_IO;
  }
  if (in && done)
    std::memcpy(data, buffer.bytes, std::min<NSUInteger>(done, length));
  return static_cast<int>(done);
}

struct libusb_transfer *libusb_alloc_transfer(int iso_packets) {
  auto *transfer = static_cast<libusb_transfer *>(std::calloc(
      1, sizeof(libusb_transfer) + static_cast<size_t>(iso_packets) * sizeof(libusb_iso_packet_descriptor)));
  if (!transfer)
    return nullptr;
  transfer->num_iso_packets = iso_packets;
  transfer->shim = new TransferState;
  return transfer;
}

void libusb_free_transfer(struct libusb_transfer *transfer) {
  if (!transfer)
    return;
  if (transfer->flags & LIBUSB_TRANSFER_FREE_BUFFER)
    std::free(transfer->buffer);
  delete static_cast<TransferState *>(transfer->shim);
  std::free(transfer);
}

void libusb_fill_iso_transfer(struct libusb_transfer *transfer, libusb_device_handle *handle, unsigned char endpoint,
                              unsigned char *buffer, int length, int num_iso_packets, libusb_transfer_cb_fn callback,
                              void *user_data, unsigned int timeout) {
  transfer->dev_handle = handle;
  transfer->endpoint = endpoint;
  transfer->type = 1; // isochronous
  transfer->timeout = timeout;
  transfer->buffer = buffer;
  transfer->length = length;
  transfer->num_iso_packets = num_iso_packets;
  transfer->callback = callback;
  transfer->user_data = user_data;
}

void libusb_set_iso_packet_lengths(struct libusb_transfer *transfer, unsigned int length) {
  static_cast<TransferState *>(transfer->shim)->packet_length = length;
  for (int i = 0; i < transfer->num_iso_packets; ++i)
    transfer->iso_packet_desc[i].length = length;
}

// The packets are read where IOUSBHost put them, in memory it allocated for I/O; the caller's own buffer is
// never filled.
unsigned char *libusb_get_iso_packet_buffer_simple(struct libusb_transfer *transfer, unsigned int packet) {
  auto *state = static_cast<TransferState *>(transfer->shim);
  if (!state->data || packet >= static_cast<unsigned int>(transfer->num_iso_packets))
    return nullptr;
  return static_cast<unsigned char *>(state->data.mutableBytes) + static_cast<size_t>(packet) * state->packet_length;
}

int libusb_submit_transfer(struct libusb_transfer *transfer) {
  libusb_device_handle *handle = transfer->dev_handle;
  auto *state = static_cast<TransferState *>(transfer->shim);
  IOUSBHostInterface *streaming = handle->interfaces[1];
  if (!streaming || !state->packet_length)
    return LIBUSB_ERROR_NOT_FOUND;
  NSError *error = nil;
  if (!handle->pipe || handle->pipe_address != transfer->endpoint) {
    handle->pipe = [streaming copyPipeWithAddress:transfer->endpoint error:&error];
    handle->pipe_address = transfer->endpoint;
    if (!handle->pipe)
      return LIBUSB_ERROR_NOT_FOUND;
  }
  const size_t count = static_cast<size_t>(transfer->num_iso_packets);
  const size_t bytes = count * state->packet_length;
  if (!state->data) {
    state->data = [streaming ioDataWithCapacity:bytes error:&error];
    if (!state->data)
      return LIBUSB_ERROR_NO_MEM;
  }
  state->transactions.assign(count, IOUSBHostIsochronousTransaction{});
  for (size_t i = 0; i < count; ++i) {
    state->transactions[i].requestCount = state->packet_length;
    state->transactions[i].offset = static_cast<uint32_t>(i * state->packet_length);
  }
  libusb_context *context = handle->device->context;
  const BOOL queued =
      [handle->pipe enqueueIORequestWithData:state->data
                             transactionList:state->transactions.data()
                        transactionListCount:count
                            firstFrameNumber:0
                                     options:IOUSBHostIsochronousTransferOptionsNone
                                       error:&error
                           completionHandler:^(IOReturn status, IOUSBHostIsochronousTransaction *list) {
                             int total = 0;
                             for (int i = 0; i < transfer->num_iso_packets; ++i) {
                               libusb_iso_packet_descriptor &packet = transfer->iso_packet_desc[i];
                               const bool good = list[i].status == kIOReturnSuccess ||
                                                 list[i].status == kIOReturnUnderrun;
                               packet.actual_length =
                                   good ? std::min<uint32_t>(list[i].completeCount, packet.length) : 0;
                               packet.status = good ? LIBUSB_TRANSFER_COMPLETED : LIBUSB_TRANSFER_ERROR;
                               total += static_cast<int>(packet.actual_length);
                             }
                             transfer->actual_length = total;
                             g_iso_bytes += static_cast<uint64_t>(total);
                             if (status == kIOReturnAborted)
                               transfer->status = LIBUSB_TRANSFER_CANCELLED;
                             else if (status == kIOReturnNoDevice || status == kIOReturnNotResponding ||
                                      status == kIOReturnNotOpen)
                               transfer->status = LIBUSB_TRANSFER_NO_DEVICE;
                             else
                               transfer->status = LIBUSB_TRANSFER_COMPLETED; // packets carry their own status
                             if (transfer->callback)
                               transfer->callback(transfer);
                             std::lock_guard<std::mutex> lock(context->waiters);
                             context->served.notify_all();
                           }];
  return queued ? 0 : LIBUSB_ERROR_IO;
}

// callbacks run on the completion queue (above): there is nothing to handle here but the wait
int libusb_handle_events_timeout_completed(libusb_context *ctx, struct timeval *tv, int *completed) {
  std::unique_lock<std::mutex> lock(ctx->mutex);
  if (!(completed && *completed))
    ctx->arrived.wait_for(lock, duration(tv, 100000), [&] { return completed && *completed; });
  return 0;
}

// not libusb's: every isochronous byte received since the process started, for telling that video is arriving
uint64_t metalvr_usb_shim_iso_bytes(void) { return g_iso_bytes.load(); }

void libusb_lock_event_waiters(libusb_context *ctx) { ctx->waiters.lock(); }

void libusb_unlock_event_waiters(libusb_context *ctx) { ctx->waiters.unlock(); }

// Called with the waiters lock held. Returns after callbacks were served or, with no timeout given, after a
// fifth of a second, so that a caller polling a condition cannot wait forever on a wakeup it missed.
int libusb_wait_for_event(libusb_context *ctx, struct timeval *tv) {
  std::unique_lock<std::mutex> lock(ctx->waiters, std::adopt_lock);
  const bool served = ctx->served.wait_for(lock, duration(tv, 200000)) == std::cv_status::no_timeout;
  lock.release();
  return (tv && !served) ? 1 : 0;
}

} // extern "C"
