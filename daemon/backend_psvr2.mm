// The PlayStation VR2's USB side for the direct-display driver (docs/HEADSETS.md, phase J), through Sony's PC
// adapter: finds the headset (054c:0cde) with IOUSBHost, reads its lens calibration and serial number, and hands
// the driver its IMU (2 kHz), whether it is worn, its IPD dial, and its position and heading, which the headset
// tracks itself from its cameras and sends finished. Its Sense controllers (054c:0e45 left, 0e46 right), paired with
// the Mac over Bluetooth, are read here too: buttons, stick, trigger, touches, battery, rotation from their IMU,
// and vibration. The report layouts are in psvr2_protocol.h.
//
// What it does not do: track the controllers' positions (the headset's cameras see their LEDs; no open driver
// solves that yet), eye tracking, passthrough, the headset's own rumble, or the adaptive triggers.
#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOKit/hid/IOHIDManager.h>
#import <IOUSBHost/IOUSBHost.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "common.h"
#include "direct_backend.h"
#include "hid_util.h"
#include "psvr2_protocol.h"

namespace metalvr {
namespace {

using namespace psvr2;

constexpr int64_t kImuPeriodNs = static_cast<int64_t>(1e9 / kImuHz);
constexpr double kSlamFreshSeconds = 0.5;

long registryNumber(io_registry_entry_t entry, CFStringRef key) {
  long value = -1;
  if (CFTypeRef ref = IORegistryEntryCreateCFProperty(entry, key, kCFAllocatorDefault, 0)) {
    if (CFGetTypeID(ref) == CFNumberGetTypeID())
      CFNumberGetValue(static_cast<CFNumberRef>(ref), kCFNumberLongType, &value);
    CFRelease(ref);
  }
  return value;
}

// one endpoint read over and over, each completed transfer handed to `handle`
struct Reader {
  IOUSBHostPipe *pipe = nil;
  NSMutableData *data = nil;
  std::function<void(const uint8_t *, size_t)> handle;
  const char *name = "";
};

class Psvr2Backend final : public DirectBackend {
public:
  const char *name() const override { return "psvr2"; }

  ~Psvr2Backend() override { closeControllers(); }

  void run(DirectHost &host, const std::atomic<bool> &running) override {
    host_ = &host;
    openControllers();
    bool told = false;
    while (running) {
      if (!open()) {
        if (!told) {
          told = true;
          logInfo("psvr2: waiting for a PlayStation VR2 on USB (054c:0cde), through Sony's PC adapter");
        }
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, false);
        continue;
      }
      drive(running);
      close();
    }
  }

  bool trackedPose(int device, TrackedPose &out) override {
    if (device != LighthouseTracking::kHead)
      return false;
    std::lock_guard<std::mutex> guard(slam_mutex_);
    if (!have_slam_)
      return false;
    out.orientation = head_.orientation;
    out.position = head_.position;
    out.velocity = velocity_;
    out.position_tracked = (nowNs() - slam_received_ns_) / 1e9 < kSlamFreshSeconds;
    return true;
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) override {
    std::lock_guard<std::mutex> guard(controllers_mutex_);
    for (auto &entry : controllers_) {
      Controller &c = *entry.second;
      if (c.hand != static_cast<int>(hand))
        continue;
      c.vibration_until_ns = duration_s > 0 ? nowNs() + static_cast<int64_t>(std::max(duration_s, 0.02f) * 1e9) : 0;
      c.vibration = duration_s > 0 ? std::clamp(amplitude, 0.0f, 1.0f) : 0.0f;
      sendVibration(c);
    }
  }

private:
  // ---- the headset ----

  bool open() {
    CFMutableDictionaryRef matching = [IOUSBHostDevice createMatchingDictionaryWithVendorID:@(kVendor)
                                                                                 productID:@(kProduct)
                                                                                 bcdDevice:nil
                                                                               deviceClass:nil
                                                                            deviceSubclass:nil
                                                                            deviceProtocol:nil
                                                                                     speed:nil
                                                                            productIDArray:nil];
    io_service_t service = IOServiceGetMatchingService(kIOMainPortDefault, matching); // takes `matching`
    if (!service)
      return false;
    queue_ = dispatch_queue_create("metalvr.psvr2.usb", DISPATCH_QUEUE_SERIAL);
    NSError *error = nil;
    // the vendor requests go to the device; if macOS's composite driver keeps it, through an interface instead
    device_ = [[IOUSBHostDevice alloc] initWithIOService:service
                                                 options:IOUSBHostObjectInitOptionsNone
                                                   queue:queue_
                                                   error:&error
                                         interestHandler:nil];
    if (!device_)
      logInfo("psvr2: the device itself did not open (%s); requests go through an interface",
              error.localizedDescription.UTF8String);
    for (int number : {kSlamInterface, kCameraInterface, kStatusInterface, kLedDetectorInterface,
                       kRelocalizerInterface, kVdInterface})
      if (!openInterface(service, number)) {
        IOObjectRelease(service);
        logInfo("psvr2: interface %d of the PlayStation VR2 did not open; is another program using it?", number);
        close();
        return false;
      }
    IOObjectRelease(service);
    return true;
  }

  bool openInterface(io_service_t device, int number) {
    io_iterator_t it = 0;
    if (IORegistryEntryGetChildIterator(device, kIOServicePlane, &it) != KERN_SUCCESS)
      return false;
    io_service_t found = 0;
    while (io_service_t child = IOIteratorNext(it)) {
      if (!found && IOObjectConformsTo(child, "IOUSBHostInterface") &&
          registryNumber(child, CFSTR("bInterfaceNumber")) == number)
        found = child;
      else
        IOObjectRelease(child);
    }
    IOObjectRelease(it);
    if (!found)
      return false;
    NSError *error = nil;
    IOUSBHostInterface *object = [[IOUSBHostInterface alloc] initWithIOService:found
                                                                       options:IOUSBHostObjectInitOptionsNone
                                                                         queue:queue_
                                                                         error:&error
                                                               interestHandler:nil];
    IOObjectRelease(found);
    if (!object)
      return false;
    const NSUInteger alternate = number == kStatusInterface ? kStatusAltSetting : 0;
    if (![object selectAlternateSetting:alternate error:&error]) {
      logInfo("psvr2: interface %d refused alternate setting %lu: %s", number, static_cast<unsigned long>(alternate),
              error.localizedDescription.UTF8String);
      return false;
    }
    interfaces_[number] = object;
    return true;
  }

  void close() {
    for (Reader &r : readers_)
      [r.pipe abortWithOption:IOUSBHostAbortOptionSynchronous error:nil];
    readers_.clear();
    for (auto &entry : interfaces_)
      [entry.second destroy];
    interfaces_.clear();
    [device_ destroy];
    device_ = nil;
    queue_ = nullptr;
    std::lock_guard<std::mutex> guard(slam_mutex_);
    have_slam_ = false;
  }

  // a vendor request (psvr2::ControlRequest) on the default pipe; how many bytes came back for a read, -1 if it
  // failed
  long control(const ControlRequest &r, std::vector<uint8_t> *reply = nullptr) {
    IOUSBDeviceRequest request{};
    request.bmRequestType = r.request_type;
    request.bRequest = r.request;
    request.wValue = r.value;
    request.wIndex = r.index;
    request.wLength = static_cast<uint16_t>(r.data.size());
    const bool in = (r.request_type & 0x80) != 0;
    NSMutableData *buffer = in ? [NSMutableData dataWithLength:r.data.size()]
                               : [NSMutableData dataWithBytes:r.data.data() length:r.data.size()];
    IOUSBHostObject *via = device_ ? static_cast<IOUSBHostObject *>(device_) : interfaces_[kStatusInterface];
    NSUInteger done = 0;
    NSError *error = nil;
    if (![via sendDeviceRequest:request data:buffer bytesTransferred:&done completionTimeout:0.2 error:&error]) {
      logInfo("psvr2: request %02x for report %02x failed: %s", r.request, r.value,
              error.localizedDescription.UTF8String);
      return -1;
    }
    if (reply)
      reply->assign(static_cast<const uint8_t *>(buffer.bytes), static_cast<const uint8_t *>(buffer.bytes) + done);
    return static_cast<long>(done);
  }

  bool startReader(int interface, int endpoint, size_t size, const char *label,
                   std::function<void(const uint8_t *, size_t)> handle) {
    IOUSBHostInterface *object = interfaces_[interface];
    NSError *error = nil;
    Reader r;
    r.pipe = [object copyPipeWithAddress:endpoint error:&error];
    r.data = r.pipe ? [object ioDataWithCapacity:size error:&error] : nil;
    if (!r.data) {
      logInfo("psvr2: the %s endpoint (%02x) did not open: %s", label, endpoint, error.localizedDescription.UTF8String);
      return false;
    }
    r.data.length = size;
    r.handle = std::move(handle);
    r.name = label;
    readers_.push_back(r);
    return true;
  }

  void submit(size_t index) {
    Reader &r = readers_[index];
    IOUSBHostPipe *pipe = r.pipe;
    NSMutableData *data = r.data;
    NSError *error = nil;
    const BOOL queued =
        [pipe enqueueIORequestWithData:data
                     completionTimeout:0
                                 error:&error
                     completionHandler:^(IOReturn status, NSUInteger bytes) {
                       if (status == kIOReturnAborted)
                         return;
                       if (status == kIOReturnNoDevice || status == kIOReturnNotResponding ||
                           status == kIOReturnNotOpen) {
                         gone_ = true;
                         return;
                       }
                       if (status == kIOReturnSuccess || status == kIOReturnUnderrun)
                         if (index < readers_.size())
                           readers_[index].handle(static_cast<const uint8_t *>(data.bytes), bytes);
                       if (!gone_ && index < readers_.size())
                         submit(index);
                     }];
    if (!queued) {
      logInfo("psvr2: reading the %s endpoint failed: %s", r.name, error.localizedDescription.UTF8String);
      gone_ = true;
    }
  }

  void drive(const std::atomic<bool> &running) {
    gone_ = false;
    std::vector<uint8_t> reply;
    const ControlRequest info = firmwareInfoRequest();
    if (control(info, &reply) > 0) {
      const auto payload = readPayload(reply.data(), reply.size(), 84);
      if (const auto firmware = parseFirmwareInfo(payload.data(), payload.size()))
        logInfo("psvr2: PlayStation VR2 %s, firmware %08x", firmware->pcb_id.c_str(), firmware->version);
    }
    std::array<float, 8> lens = defaultLensCalibration();
    if (control(lensCalibrationRequest(), &reply) > 0) {
      const auto payload = readPayload(reply.data(), reply.size(), 0x100);
      lens = lensCalibration(payload.data(), payload.size());
    } else {
      logInfo("psvr2: no lens calibration from the headset; using Monado's default");
    }
    host_->directModel(psvr2Model(lens));
    control(brightnessRequest(1.0f));
    // the headset tracks itself with its cameras: they run as Monado runs them, and what they send is read and
    // dropped, as are the other streams Monado drains
    control(cameraModeRequest(kCameraTracking));

    last_imu_ns_ = 0;
    const bool started =
        startReader(kStatusInterface, kStatusEndpoint, kStatusTransferSize, "status",
                    [this](const uint8_t *p, size_t n) { status(p, n); }) &&
        startReader(kSlamInterface, kSlamEndpoint, kSlamTransferSize, "tracking",
                    [this](const uint8_t *p, size_t n) { slam(p, n); }) &&
        startReader(kCameraInterface, kCameraEndpoint, kCameraTransferSize, "camera", [](const uint8_t *, size_t) {}) &&
        startReader(kLedDetectorInterface, kLedDetectorEndpoint, kLedDetectorTransferSize, "LED detector",
                    [](const uint8_t *, size_t) {}) &&
        startReader(kRelocalizerInterface, kRelocalizerEndpoint, kRelocalizerTransferSize, "relocalizer",
                    [](const uint8_t *, size_t) {}) &&
        startReader(kVdInterface, kVdEndpoint, kVdTransferSize, "VD", [](const uint8_t *, size_t) {});
    if (!started) {
      logInfo("psvr2: the PlayStation VR2's endpoints did not all open; looking for it again");
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, false);
      return;
    }
    for (size_t i = 0; i < readers_.size(); i++)
      submit(i);
    host_->directOn(true);
    for (int hand = 0; hand < 2; hand++)
      host_->directControllerGrip(hand, senseGrip(hand), senseAim(hand));

    while (running && !gone_) {
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
      stopFinishedVibrations();
    }
    if (gone_)
      logInfo("psvr2: the PlayStation VR2 went away; looking for it again");
    else
      control(cameraModeRequest(kCameraOff));
    host_->directOn(false);
  }

  // on the USB queue
  void status(const uint8_t *p, size_t n) {
    const auto s = parseStatus(p, n);
    if (!s)
      return;
    const int64_t received = nowNs();
    const size_t count = s->imu.size();
    for (size_t i = 0; i < count; i++) {
      // the samples are half a millisecond apart, and the last arrived just now
      const int64_t t = received - static_cast<int64_t>(count - 1 - i) * kImuPeriodNs;
      if (t <= last_imu_ns_)
        continue;
      last_imu_ns_ = t;
      host_->directImu(t, s->imu[i].accel, s->imu[i].gyro);
    }
    if (s->worn != worn_ || !told_worn_) {
      worn_ = s->worn;
      told_worn_ = true;
      host_->directWorn(worn_);
      logInfo(worn_ ? "psvr2: the PlayStation VR2 is worn" : "psvr2: the PlayStation VR2 is off the face");
    }
    if (s->ipd_mm >= 55 && s->ipd_mm <= 75 && s->ipd_mm != ipd_mm_) {
      ipd_mm_ = s->ipd_mm;
      host_->directIpd(ipd_mm_ / 1000.0f);
      logInfo("psvr2: IPD %u mm", ipd_mm_);
    }
    if (s->display == 0 && display_ready_) {
      display_ready_ = false;
      logInfo("psvr2: the headset's DisplayPort input lost its signal");
    } else if (s->display != 0 && !display_ready_) {
      display_ready_ = true;
      logInfo("psvr2: the headset's DisplayPort input has a signal");
    }
  }

  // on the USB queue
  void slam(const uint8_t *p, size_t n) {
    std::lock_guard<std::mutex> guard(slam_mutex_);
    const auto s = parseSlam(p, n, slam_.orientation);
    if (!s)
      return;
    const int64_t now = nowNs();
    const mvr_pose head = headFromSlam(s->pose);
    if (have_slam_) {
      const float dt = static_cast<float>(static_cast<uint32_t>(s->vts_us - slam_vts_us_)) / 1e6f;
      if (dt > 0 && dt < 0.2f)
        velocity_ = {(head.position.x - head_.position.x) / dt, (head.position.y - head_.position.y) / dt,
                     (head.position.z - head_.position.z) / dt};
    } else {
      logInfo("psvr2: the headset is tracking itself");
    }
    slam_ = s->pose;
    slam_vts_us_ = s->vts_us;
    head_ = head;
    slam_received_ns_ = now;
    have_slam_ = true;
  }

  // ---- the Sense controllers ----

  struct Controller {
    IOHIDDeviceRef device = nullptr;
    int hand = -1;
    bool bluetooth = true;
    std::optional<SenseCalibration> calibration;
    uint8_t buffer[128];
    uint8_t sequence = 0, counter = 0;
    float vibration = 0;
    int64_t vibration_until_ns = 0;
    uint32_t last_ticks = 0;
    bool have_ticks = false;
    int64_t imu_ns = 0;
    Psvr2Backend *owner = nullptr;
  };

  void openControllers() {
    manager_ = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    NSArray *matches = @[
      @{@kIOHIDVendorIDKey : @(kVendor), @kIOHIDProductIDKey : @(kSenseLeft)},
      @{@kIOHIDVendorIDKey : @(kVendor), @kIOHIDProductIDKey : @(kSenseRight)}
    ];
    IOHIDManagerSetDeviceMatchingMultiple(manager_, (__bridge CFArrayRef)matches);
    IOHIDManagerRegisterDeviceMatchingCallback(manager_, onController, this);
    IOHIDManagerRegisterDeviceRemovalCallback(manager_, onControllerGone, this);
    IOHIDManagerScheduleWithRunLoop(manager_, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    IOHIDManagerOpen(manager_, kIOHIDOptionsTypeNone);
  }

  void closeControllers() {
    if (!manager_)
      return;
    std::lock_guard<std::mutex> guard(controllers_mutex_);
    for (auto &entry : controllers_)
      IOHIDDeviceRegisterInputReportCallback(entry.second->device, entry.second->buffer, sizeof(entry.second->buffer),
                                             nullptr, nullptr);
    controllers_.clear();
    IOHIDManagerClose(manager_, kIOHIDOptionsTypeNone);
    CFRelease(manager_);
    manager_ = nullptr;
  }

  static void onController(void *context, IOReturn, void *, IOHIDDeviceRef device) {
    static_cast<Psvr2Backend *>(context)->addController(device);
  }
  static void onControllerGone(void *context, IOReturn, void *, IOHIDDeviceRef device) {
    auto *self = static_cast<Psvr2Backend *>(context);
    std::lock_guard<std::mutex> guard(self->controllers_mutex_);
    if (self->controllers_.erase(device))
      logInfo("psvr2: a Sense controller went away");
  }

  void addController(IOHIDDeviceRef device) {
    auto c = std::make_unique<Controller>();
    c->device = device;
    c->owner = this;
    c->hand = hid::product(device) == kSenseLeft ? 0 : 1;
    NSString *transport = (__bridge NSString *)IOHIDDeviceGetProperty(device, CFSTR(kIOHIDTransportKey));
    c->bluetooth = !transport || ![transport isEqualToString:@"USB"];
    // the calibration, in two parts; reading it also switches the controller into the reports read here
    uint8_t a[64] = {}, b[64] = {};
    if (hid::getFeature(device, kSenseCalibrationReport, a, sizeof(a)) == sizeof(a) &&
        hid::getFeature(device, kSenseCalibrationReport, b, sizeof(b)) == sizeof(b))
      c->calibration = parseSenseCalibration(a, b);
    logInfo("psvr2: %s Sense controller over %s%s", c->hand == 0 ? "left" : "right",
            c->bluetooth ? "Bluetooth" : "USB", c->calibration ? "" : " (no IMU calibration: rotation off)");
    IOHIDDeviceRegisterInputReportCallback(device, c->buffer, sizeof(c->buffer), onReport, c.get());
    std::lock_guard<std::mutex> guard(controllers_mutex_);
    controllers_[device] = std::move(c);
  }

  static void onReport(void *context, IOReturn result, void *, IOHIDReportType, uint32_t id, uint8_t *report,
                       CFIndex length) {
    if (result != kIOReturnSuccess)
      return;
    auto *c = static_cast<Controller *>(context);
    c->owner->controllerReport(*c, id, report, length);
  }

  void controllerReport(Controller &c, uint32_t id, const uint8_t *raw, CFIndex length) {
    uint8_t report[128];
    const size_t n = hid::withId(id, raw, length, report, sizeof(report));
    const auto s = parseSenseReport(report, n, c.hand == 0);
    if (!s)
      return;
    const int64_t received = nowNs();
    DirectControllerInput in;
    in.buttons = s->buttons;
    in.touches = s->touches;
    in.trigger = s->trigger;
    in.squeeze = s->squeeze;
    in.thumbstick_x = s->stick_x;
    in.thumbstick_y = s->stick_y;
    in.has_battery = s->battery_valid;
    in.battery = s->battery;
    in.charging = s->charging;
    host_->directControllerInput(c.hand, in, received);
    if (!c.calibration)
      return;
    // the IMU's own clock, thirds of a microsecond, laid on the Mac's from the first report
    if (!c.have_ticks) {
      c.have_ticks = true;
      c.imu_ns = received;
    } else {
      c.imu_ns += static_cast<int64_t>(static_cast<uint32_t>(s->imu_ticks - c.last_ticks)) * 1000 / 3;
      // never ahead of now, nor far behind it
      c.imu_ns = std::clamp(c.imu_ns, received - 20'000'000, received);
    }
    c.last_ticks = s->imu_ticks;
    float accel[3], gyro[3];
    senseImu(*s, *c.calibration, accel, gyro);
    host_->directControllerImu(c.hand, c.imu_ns, accel, gyro);
  }

  // controllers_mutex_ held
  void sendVibration(Controller &c) {
    if (!c.bluetooth)
      return; // over USB the controller takes another report, which is not sent here
    const auto report = senseOutputReport(c.vibration, c.sequence, c.counter++,
                                          static_cast<uint32_t>(nowNs() / 1000));
    c.sequence = (c.sequence + 1) % 16;
    IOHIDDeviceSetReport(c.device, kIOHIDReportTypeOutput, report[0], report.data(),
                         static_cast<CFIndex>(report.size()));
  }

  void stopFinishedVibrations() {
    std::lock_guard<std::mutex> guard(controllers_mutex_);
    const int64_t now = nowNs();
    for (auto &entry : controllers_) {
      Controller &c = *entry.second;
      if (c.vibration > 0 && now >= c.vibration_until_ns) {
        c.vibration = 0;
        sendVibration(c);
      }
    }
  }

  DirectHost *host_ = nullptr;
  dispatch_queue_t queue_ = nullptr;
  IOUSBHostDevice *device_ = nil;
  std::map<int, IOUSBHostInterface *> interfaces_;
  std::vector<Reader> readers_;
  std::atomic<bool> gone_{false};
  int64_t last_imu_ns_ = 0;
  bool worn_ = false, told_worn_ = false, display_ready_ = false;
  uint8_t ipd_mm_ = 0;

  std::mutex slam_mutex_;
  bool have_slam_ = false;
  mvr_pose slam_{{0, 0, 0, 1}, {0, 0, 0}}, head_{{0, 0, 0, 1}, {0, 0, 0}};
  mvr_vec3 velocity_{};
  uint32_t slam_vts_us_ = 0;
  int64_t slam_received_ns_ = 0;

  IOHIDManagerRef manager_ = nullptr;
  std::mutex controllers_mutex_;
  std::map<IOHIDDeviceRef, std::unique_ptr<Controller>> controllers_;
};

} // namespace

std::unique_ptr<DirectBackend> makePsvr2Backend() { return std::make_unique<Psvr2Backend>(); }

} // namespace metalvr
