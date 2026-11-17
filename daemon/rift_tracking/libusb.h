// The part of libusb's API that OpenHMD's Rift sensor code uses, implemented on macOS's IOUSBHost in
// usb_shim.mm. It is not libusb: only what those sources call is here, with libusb's names and meanings so that
// they compile unchanged.
#pragma once

#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct libusb_context libusb_context;
typedef struct libusb_device libusb_device;
typedef struct libusb_device_handle libusb_device_handle;

struct libusb_device_descriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
};

enum libusb_transfer_status {
  LIBUSB_TRANSFER_COMPLETED = 0,
  LIBUSB_TRANSFER_ERROR,
  LIBUSB_TRANSFER_TIMED_OUT,
  LIBUSB_TRANSFER_CANCELLED,
  LIBUSB_TRANSFER_STALL,
  LIBUSB_TRANSFER_NO_DEVICE,
  LIBUSB_TRANSFER_OVERFLOW,
};

enum {
  LIBUSB_REQUEST_TYPE_CLASS = 0x20,
  LIBUSB_RECIPIENT_INTERFACE = 0x01,
  LIBUSB_TRANSFER_FREE_BUFFER = 1 << 1,
  LIBUSB_ERROR_IO = -1,
  LIBUSB_ERROR_NO_DEVICE = -4,
  LIBUSB_ERROR_NOT_FOUND = -5,
  LIBUSB_ERROR_NO_MEM = -11,
  LIBUSB_ERROR_OTHER = -99,
};

struct libusb_iso_packet_descriptor {
  unsigned int length;
  unsigned int actual_length;
  enum libusb_transfer_status status;
};

struct libusb_transfer;
typedef void (*libusb_transfer_cb_fn)(struct libusb_transfer *transfer);

struct libusb_transfer {
  libusb_device_handle *dev_handle;
  uint8_t flags;
  unsigned char endpoint;
  unsigned char type;
  unsigned int timeout;
  enum libusb_transfer_status status;
  int length;
  int actual_length;
  libusb_transfer_cb_fn callback;
  void *user_data;
  unsigned char *buffer;
  int num_iso_packets;
  void *shim; // the shim's own state for this transfer
  struct libusb_iso_packet_descriptor iso_packet_desc[];
};

int libusb_init(libusb_context **ctx);
void libusb_exit(libusb_context *ctx);
ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list);
void libusb_free_device_list(libusb_device **list, int unref_devices);
int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc);
int libusb_open(libusb_device *dev, libusb_device_handle **handle);
void libusb_close(libusb_device_handle *handle);
libusb_device *libusb_get_device(libusb_device_handle *handle);
int libusb_get_string_descriptor_ascii(libusb_device_handle *handle, uint8_t index, unsigned char *data, int length);
int libusb_set_auto_detach_kernel_driver(libusb_device_handle *handle, int enable);
int libusb_claim_interface(libusb_device_handle *handle, int interface_number);
int libusb_set_interface_alt_setting(libusb_device_handle *handle, int interface_number, int alternate_setting);
int libusb_control_transfer(libusb_device_handle *handle, uint8_t request_type, uint8_t request, uint16_t value,
                            uint16_t index, unsigned char *data, uint16_t length, unsigned int timeout);

struct libusb_transfer *libusb_alloc_transfer(int iso_packets);
void libusb_free_transfer(struct libusb_transfer *transfer);
int libusb_submit_transfer(struct libusb_transfer *transfer);
void libusb_fill_iso_transfer(struct libusb_transfer *transfer, libusb_device_handle *handle, unsigned char endpoint,
                              unsigned char *buffer, int length, int num_iso_packets, libusb_transfer_cb_fn callback,
                              void *user_data, unsigned int timeout);
void libusb_set_iso_packet_lengths(struct libusb_transfer *transfer, unsigned int length);
unsigned char *libusb_get_iso_packet_buffer_simple(struct libusb_transfer *transfer, unsigned int packet);

int libusb_handle_events_timeout_completed(libusb_context *ctx, struct timeval *tv, int *completed);
void libusb_lock_event_waiters(libusb_context *ctx);
void libusb_unlock_event_waiters(libusb_context *ctx);
int libusb_wait_for_event(libusb_context *ctx, struct timeval *tv);

#ifdef __cplusplus
}
#endif
