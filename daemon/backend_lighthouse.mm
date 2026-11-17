// The Lighthouse headsets' USB side for the direct-display driver (docs/HEADSETS.md, phases F, G and H): finds an
// HTC Vive, Vive Pro, Vive Pro 2 or Valve Index, reads the calibration it keeps in its firmware (its lenses and its
// IMU), turns it on, switches a Vive Pro 2 to the mode asked for, and hands the driver its IMU samples, its IPD and
// whether it is worn, and its controllers' inputs and rotation through their Watchman receivers. The report layouts
// are in lighthouse_protocol.h.
//
// What it does not do yet: track position with the base stations (the Lighthouse pulses are not read; libsurvive
// would do it), so the head and the controllers have their rotation only, as a Rift CV1 without its sensors does.
#import <Foundation/Foundation.h>
#import <IOKit/hid/IOHIDManager.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "common.h"
#include "direct_backend.h"
#include "hid_util.h"
#include "lighthouse_protocol.h"
#include "lighthouse_tracking.h"

namespace metalvr {
namespace {

using namespace lighthouse;

// The device's 48 MHz clock onto the server's: anchored at a report's arrival, carried by the ticks between samples,
// and anchored again when the two drift apart.
class TickClock {
public:
  int64_t toNs(uint32_t ticks, int64_t received_ns) {
    if (!anchored_) {
      anchored_ = true;
      base_ticks_ = ticks;
      base_ns_ = received_ns;
      return received_ns;
    }
    const uint32_t delta = ticks - base_ticks_; // wraps
    int64_t t = base_ns_ + static_cast<int64_t>(delta / kTicksPerSecond * 1e9);
    if (t > received_ns || received_ns - t > 50'000'000) {
      base_ticks_ = ticks;
      base_ns_ = received_ns;
      t = received_ns;
    }
    return t;
  }
  void reset() { anchored_ = false; }

private:
  bool anchored_ = false;
  uint32_t base_ticks_ = 0;
  int64_t base_ns_ = 0;
};

struct Controller {
  IOHIDDeviceRef device = nullptr;
  bool gen2 = false;
  bool configured = false;
  ControllerConfig config;
  ImuRanges ranges;
  ControllerState state;
  int hand = -1;
  TickClock clock;
  std::chrono::steady_clock::time_point retry{};
  uint8_t buffer[64];
  class LighthouseBackend *owner = nullptr;
};

class LighthouseBackend final : public DirectBackend {
public:
  explicit LighthouseBackend(const LighthouseOptions &options) : options_(options) {}
  const char *name() const override { return "lighthouse"; }

  void run(DirectHost &host, const std::atomic<bool> &running) override {
    host_ = &host;
    bool told = false;
    while (running) {
      if (!find()) {
        if (!told) {
          told = true;
          logInfo("lighthouse: waiting for an HTC Vive, Vive Pro, Vive Pro 2 or Valve Index on USB");
        }
        close();
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, false);
        continue;
      }
      if (!configure()) {
        close();
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 3.0, false);
        continue;
      }
      drive(running);
      close();
    }
  }

  bool trackedPose(int device, TrackedPose &out) override {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    return tracking_ && tracking_->pose(device, out);
  }

  uint32_t trackingReferences(mvr_tracking_reference *out) override {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    return tracking_ ? tracking_->references(out) : 0;
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) override {
    std::lock_guard<std::mutex> lock(controllers_mutex_);
    for (const auto &c : controllers_)
      if (c->configured && c->hand == static_cast<int>(hand)) {
        const auto pulse = hapticReport(duration_s <= 0 ? 0 : std::max(duration_s, 0.05f), frequency_hz, amplitude);
        hid::setFeature(c->device, pulse.data(), pulse.size());
      }
  }

  void controllerInput(Controller &c, uint32_t id, const uint8_t *raw, CFIndex length) {
    uint8_t report[64];
    const size_t n = hid::withId(id, raw, length, report, sizeof(report));
    const int64_t received = nowNs();
    if (report[0] == kControllerDisconnect) {
      logInfo("lighthouse: a controller went off");
      return;
    }
    ControllerImu imu[4];
    const size_t samples = decodeControllerReport(report, n, c.gen2, c.config, c.ranges, c.state, imu);
    if (c.hand < 0)
      return;
    host_->directControllerInput(c.hand, inputOf(c), received);
    for (size_t i = 0; i < samples; ++i)
      host_->directControllerImu(c.hand, c.clock.toNs(imu[i].ticks, received), imu[i].accel, imu[i].gyro);
  }

private:
  bool find() {
    manager_ = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    NSArray *matching = @[ @{@kIOHIDVendorIDKey : @(kHtcVendor)}, @{@kIOHIDVendorIDKey : @(kValveVendor)} ];
    IOHIDManagerSetDeviceMatchingMultiple(manager_, (__bridge CFArrayRef)matching);
    IOHIDManagerOpen(manager_, kIOHIDOptionsTypeNone);
    std::vector<IOHIDDeviceRef> list = hid::devices(manager_);
    std::vector<UsbId> ids;
    for (IOHIDDeviceRef d : list)
      ids.push_back({hid::vendor(d), hid::product(d)});
    model_ = identify(ids);
    if (model_ == Model::None) {
      hid::release(list);
      return false;
    }
    for (IOHIDDeviceRef d : list) {
      const uint16_t vendor = hid::vendor(d), product = hid::product(d);
      const int interface = hid::interfaceNumber(d);
      if (vendor == kValveVendor && (product == kViveSensors || product == kLhrSensors) && interface == 0 && !sensors_)
        sensors_ = take(d);
      else if (vendor == kHtcVendor && (product == kViveMainboard || product == kViveProMainboard ||
                                        product == kViveProMainboardB || product == kVivePro2Mainboard) &&
               interface == 0 && !mainboard_)
        mainboard_ = take(d);
      else if (vendor == kValveVendor && (product == kWatchmanGen1 || product == kWatchmanGen2) && interface == 0) {
        if (IOHIDDeviceRef opened = take(d)) {
          auto c = std::make_unique<Controller>();
          c->device = opened;
          c->gen2 = product == kWatchmanGen2;
          c->owner = this;
          controllers_.push_back(std::move(c));
        }
      }
    }
    hid::release(list);
    if (!sensors_) {
      logInfo("lighthouse: %s on USB, but its IMU interface did not open", nameOf(model_));
      return false;
    }
    return true;
  }

  IOHIDDeviceRef take(IOHIDDeviceRef d) {
    if (IOHIDDeviceOpen(d, kIOHIDOptionsTypeNone) != kIOReturnSuccess)
      return nullptr;
    CFRetain(d);
    return d;
  }

  void close() {
    {
      std::lock_guard<std::mutex> lock(controllers_mutex_);
      for (auto &c : controllers_) {
        IOHIDDeviceUnscheduleFromRunLoop(c->device, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
        IOHIDDeviceClose(c->device, kIOHIDOptionsTypeNone);
        CFRelease(c->device);
      }
      controllers_.clear();
    }
    for (IOHIDDeviceRef *d : {&sensors_, &mainboard_}) {
      if (!*d)
        continue;
      IOHIDDeviceUnscheduleFromRunLoop(*d, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
      IOHIDDeviceClose(*d, kIOHIDOptionsTypeNone);
      CFRelease(*d);
      *d = nullptr;
    }
    if (manager_) {
      IOHIDManagerClose(manager_, kIOHIDOptionsTypeNone);
      CFRelease(manager_);
      manager_ = nullptr;
    }
    have_sequence_ = false;
    clock_.reset();
  }

  // a device's configuration: feature report 0x10 starts it, 0x11 reports carry it until an empty one
  static std::optional<std::string> readConfig(IOHIDDeviceRef device) {
    uint8_t report[64];
    if (!hid::getFeature(device, kConfigStartReport, report, sizeof(report)))
      return std::nullopt;
    ConfigAssembler assembler;
    for (int i = 0; i < 512 && !assembler.done(); ++i) {
      const size_t n = hid::getFeature(device, kConfigReadReport, report, sizeof(report));
      if (!n || !assembler.add(report, n))
        return std::nullopt;
    }
    return assembler.json();
  }

  bool configure() {
    const auto json = readConfig(sensors_);
    const auto config = json ? parseHeadsetConfig(*json) : std::nullopt;
    if (!config) {
      logInfo("lighthouse: %s on USB, but its configuration (lenses, IMU) %s; it is not driven", nameOf(model_),
              json ? "is not one MetalVR reads" : "could not be read");
      return false;
    }
    if (config->model != model_)
      logInfo("lighthouse: USB said %s, its configuration says %s (\"%s\"); going by the configuration",
              nameOf(model_), nameOf(config->model), config->model_number.c_str());
    model_ = config->model;
    config_ = *config;
    uint8_t report[64];
    const size_t n = hid::getFeature(sensors_, kImuRangeReport, report, sizeof(report));
    if (const auto ranges = parseImuRanges(report, n))
      ranges_ = *ranges;
    else
      logInfo("lighthouse: no IMU range report; taking 500 degrees/s and 4 g");

    if (model_ == Model::VivePro2 && options_.pro2_mode >= 0 && !pro2_switched_) {
      if (!mainboard_) {
        logInfo("lighthouse: a Vive Pro 2's mode is switched through its HTC mainboard, which is not on USB");
      } else {
        const Pro2Mode &mode = pro2Modes()[std::min<size_t>(options_.pro2_mode, pro2Modes().size() - 1)];
        logInfo("lighthouse: switching the Vive Pro 2 to %dx%d at %.0f Hz%s; it restarts", mode.width, mode.height,
                mode.refresh_hz, mode.needs_dsc ? " (needs Display Stream Compression, which macOS may not do)" : "");
        char dtd[16];
        std::snprintf(dtd, sizeof(dtd), "dtd,%d", mode.index);
        for (const char *command : {"wireless,0", static_cast<const char *>(dtd), "chipreset"}) {
          const auto r = pro2Command(command);
          hid::setFeature(mainboard_, r.data(), r.size());
        }
        pro2_switched_ = true;
        return false; // it goes away and comes back; look for it again
      }
    }
    pro2_mode_ = pro2_switched_ ? options_.pro2_mode : 2;
    host_->directModel(lighthouseModel(config_, pro2_mode_));
    logInfo("lighthouse: %s (%s), panel %dx%d per eye", nameOf(model_), config_.serial.c_str(), config_.eye_width,
            config_.eye_height);
    return true;
  }

  void drive(const std::atomic<bool> &running) {
    if (mainboard_ && model_ != Model::Index) {
      const auto on = powerReport(true);
      if (!hid::setFeature(mainboard_, on.data(), on.size()))
        logInfo("lighthouse: the mainboard did not take the power-on report");
    }
    host_->directOn(true);
    // positions from the base stations, when libsurvive is built in; it opens the same devices alongside
    if (lighthouseTrackingBuilt()) {
      auto tracking = startLighthouseTracking();
      std::lock_guard<std::mutex> lock(tracking_mutex_);
      tracking_ = std::move(tracking);
    } else {
      logInfo("lighthouse: built without libsurvive (scripts/build-libsurvive.sh); rotation only");
    }
    IOHIDDeviceRegisterInputReportCallback(sensors_, sensors_buffer_, sizeof(sensors_buffer_), onSensors, this);
    IOHIDDeviceScheduleWithRunLoop(sensors_, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    if (mainboard_) {
      IOHIDDeviceRegisterInputReportCallback(mainboard_, mainboard_buffer_, sizeof(mainboard_buffer_), onMainboard, this);
      IOHIDDeviceScheduleWithRunLoop(mainboard_, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    }
    gone_ = false;
    IOHIDManagerRegisterDeviceRemovalCallback(manager_, onRemoved, this);
    IOHIDManagerScheduleWithRunLoop(manager_, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    while (running && !gone_) {
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, false);
      serviceControllers();
    }
    IOHIDManagerUnscheduleFromRunLoop(manager_, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    {
      std::unique_ptr<LighthouseTracking> tracking;
      {
        std::lock_guard<std::mutex> lock(tracking_mutex_);
        tracking = std::move(tracking_);
      }
      tracking.reset(); // libsurvive stops and lets the devices go
    }
    if (mainboard_ && model_ != Model::Index) {
      const auto off = powerReport(false);
      hid::setFeature(mainboard_, off.data(), off.size());
    }
    host_->directOn(false);
  }

  // A receiver's controller answers once it is on: its configuration says which it is (a wand, a left or right
  // Index controller). Until then it is asked again every two seconds.
  void serviceControllers() {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(controllers_mutex_);
    for (auto &c : controllers_) {
      if (c->configured || now < c->retry)
        continue;
      c->retry = now + std::chrono::seconds(2);
      const auto json = readConfig(c->device);
      const auto config = json ? parseControllerConfig(*json) : std::nullopt;
      if (!config || config->kind == ControllerKind::Unknown) {
        if (json)
          logInfo("lighthouse: a controller MetalVR does not know (\"%s\")", config ? config->model_number.c_str() : "?");
        continue;
      }
      c->config = *config;
      uint8_t report[64];
      if (const auto ranges = parseImuRanges(report, hid::getFeature(c->device, kImuRangeReport, report, sizeof(report))))
        c->ranges = *ranges;
      c->hand = config->kind == ControllerKind::IndexLeft    ? 0
                : config->kind == ControllerKind::IndexRight ? 1
                                                             : freeHand();
      c->configured = true;
      IOHIDDeviceRegisterInputReportCallback(c->device, c->buffer, sizeof(c->buffer), onController, c.get());
      IOHIDDeviceScheduleWithRunLoop(c->device, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
      host_->directControllerGrip(c->hand, mvr_pose{{0, 0, 0, 1}, {0, 0, 0}}, mvr_pose{{0, 0, 0, 1}, {0, 0, 0}});
      logInfo("lighthouse: %s controller: %s", c->hand == 0 ? "left" : "right", config->model_number.c_str());
    }
  }

  // a Vive wand has no side: the first one on is the left hand, the next the right
  int freeHand() const {
    bool used[2] = {false, false};
    for (const auto &c : controllers_)
      if (c->configured && c->hand >= 0)
        used[c->hand] = true;
    return used[0] ? 1 : 0;
  }

  static DirectControllerInput inputOf(const Controller &c) {
    const ControllerState &s = c.state;
    DirectControllerInput in;
    const auto bit = [&](int n) { return (s.buttons >> n) & 1; };
    in.trigger = s.trigger;
    if (c.config.kind == ControllerKind::Wand) {
      // trigger 0, trackpad touch 1, trackpad click 2, system 3, grip 4, menu 5
      in.buttons = (bit(0) ? MVR_BUTTON_TRIGGER : 0u) | (bit(2) ? MVR_BUTTON_TRACKPAD : 0u) |
                   (bit(3) ? MVR_BUTTON_SYSTEM : 0u) | (bit(4) ? MVR_BUTTON_SQUEEZE : 0u) |
                   (bit(5) ? MVR_BUTTON_MENU : 0u);
      in.touches = bit(1) ? MVR_BUTTON_TRACKPAD : 0u;
      in.squeeze = bit(4) ? 1.0f : 0.0f;
      in.trackpad_x = s.pad_x;
      in.trackpad_y = s.pad_y;
    } else {
      // trigger 0, trackpad touch 1, thumbstick click 2, system 3, A 4, B 5; touched: system 3, A 4, B 5, stick 6
      const auto touched = [&](int n) { return (s.touch >> n) & 1; };
      in.buttons = (bit(0) ? MVR_BUTTON_TRIGGER : 0u) | (bit(2) ? MVR_BUTTON_THUMBSTICK : 0u) |
                   (bit(3) ? MVR_BUTTON_SYSTEM : 0u) | (bit(4) ? MVR_BUTTON_PRIMARY : 0u) |
                   (bit(5) ? MVR_BUTTON_SECONDARY : 0u);
      in.touches = (bit(1) ? MVR_BUTTON_TRACKPAD : 0u) | (touched(3) ? MVR_BUTTON_SYSTEM : 0u) |
                   (touched(4) ? MVR_BUTTON_PRIMARY : 0u) | (touched(5) ? MVR_BUTTON_SECONDARY : 0u) |
                   (touched(6) ? MVR_BUTTON_THUMBSTICK : 0u);
      // the trackpad and the stick report on the same axes: the trackpad's while it is touched (Monado's choice)
      if (bit(1)) {
        in.trackpad_x = s.pad_x;
        in.trackpad_y = s.pad_y;
      } else {
        in.thumbstick_x = s.pad_x;
        in.thumbstick_y = s.pad_y;
      }
      if (s.has_fingers) {
        in.has_fingers = true;
        for (int f = 0; f < 4; ++f)
          in.fingers[f] = s.fingers[f] / 255.0f;
        in.squeeze = (s.fingers[1] + s.fingers[2] + s.fingers[3]) / (3.0f * 255.0f);
        in.squeeze_force = s.squeeze_force / 255.0f;
        in.trackpad_force = s.trackpad_force / 255.0f;
      }
    }
    if (s.has_battery) {
      in.has_battery = true;
      in.battery = s.battery / 100.0f;
      in.charging = s.charging;
    }
    return in;
  }

  static void onSensors(void *context, IOReturn result, void *, IOHIDReportType, uint32_t id, uint8_t *report,
                        CFIndex length) {
    if (result != kIOReturnSuccess)
      return;
    auto *self = static_cast<LighthouseBackend *>(context);
    uint8_t buffer[64];
    const size_t n = hid::withId(id, report, length, buffer, sizeof(buffer));
    if (buffer[0] != kImuReport)
      return; // Lighthouse pulses (0x25, 0x27, 0x28): not used yet
    ImuSample samples[3];
    const int64_t received = nowNs();
    const size_t count = parseImuReport(buffer, n, self->config_, self->ranges_, self->sequence_, self->have_sequence_,
                                        samples);
    for (size_t i = 0; i < count; ++i)
      self->host_->directImu(self->clock_.toNs(samples[i].ticks, received), samples[i].accel, samples[i].gyro);
  }

  static void onMainboard(void *context, IOReturn result, void *, IOHIDReportType, uint32_t id, uint8_t *report,
                          CFIndex length) {
    if (result != kIOReturnSuccess)
      return;
    auto *self = static_cast<LighthouseBackend *>(context);
    uint8_t buffer[64];
    const size_t n = hid::withId(id, report, length, buffer, sizeof(buffer));
    const auto status = parseMainboardStatus(buffer, n);
    if (!status)
      return;
    if (status->ipd > 0.05f && status->ipd < 0.08f)
      self->host_->directIpd(status->ipd);
    self->host_->directWorn(status->proximity > kWornProximity);
  }

  static void onController(void *context, IOReturn result, void *, IOHIDReportType, uint32_t id, uint8_t *report,
                           CFIndex length) {
    if (result != kIOReturnSuccess)
      return;
    auto *c = static_cast<Controller *>(context);
    c->owner->controllerInput(*c, id, report, length);
  }

  static void onRemoved(void *context, IOReturn, void *, IOHIDDeviceRef device) {
    auto *self = static_cast<LighthouseBackend *>(context);
    if (device == self->sensors_ || device == self->mainboard_) {
      logInfo("lighthouse: the headset went away from USB");
      self->gone_ = true;
    }
  }

  const LighthouseOptions options_;
  DirectHost *host_ = nullptr;
  IOHIDManagerRef manager_ = nullptr;
  IOHIDDeviceRef sensors_ = nullptr, mainboard_ = nullptr;
  std::mutex controllers_mutex_;
  std::mutex tracking_mutex_;
  std::unique_ptr<LighthouseTracking> tracking_;
  std::vector<std::unique_ptr<Controller>> controllers_;
  Model model_ = Model::None;
  HeadsetConfig config_;
  ImuRanges ranges_;
  uint8_t sequence_ = 0;
  bool have_sequence_ = false;
  TickClock clock_;
  bool gone_ = false;
  bool pro2_switched_ = false;
  int pro2_mode_ = 2;
  uint8_t sensors_buffer_[64];
  uint8_t mainboard_buffer_[64];
};

} // namespace

std::unique_ptr<DirectBackend> makeLighthouseBackend(const LighthouseOptions &options) {
  return std::make_unique<LighthouseBackend>(options);
}

} // namespace metalvr
