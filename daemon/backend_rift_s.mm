// The Oculus Rift S's USB side for the direct-display driver (docs/HEADSETS.md, phase I): finds the headset
// (2833:0051) and opens its three HID interfaces, reads its IMU's scales and calibration, turns it on and keeps it
// on, and hands the driver its IMU samples, whether it is worn, and its Touch controllers' buttons and rotation.
// The report layouts are in rift_s_protocol.h.
//
// The headset's position comes from its cameras (rift_s_tracking.h): Basalt, when scripts/build-basalt.sh has built
// it and macOS lets metalvrd read the cameras; otherwise the headset tracks rotation only. What it does not do:
// track the controllers' positions (no open driver does), or play haptics on them (the command is not known).
#import <Foundation/Foundation.h>
#import <IOKit/hid/IOHIDManager.h>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "common.h"
#include "direct_backend.h"
#include "hid_util.h"
#include "rift_s_cameras.h"
#include "rift_s_protocol.h"
#include "rift_s_tracking.h"

namespace metalvr {
namespace {

using namespace rift_s;

constexpr float kPi = 3.14159265358979323846f;

class RiftSBackend final : public DirectBackend {
public:
  const char *name() const override { return "rift-s"; }

  bool trackedPose(int device, TrackedPose &out) override {
    if (device != LighthouseTracking::kHead)
      return false;
    std::lock_guard<std::mutex> guard(tracking_mutex_);
    return tracking_ && tracking_->pose(out);
  }

  void run(DirectHost &host, const std::atomic<bool> &running) override {
    host_ = &host;
    bool told = false;
    while (running) {
      if (!open()) {
        if (!told) {
          told = true;
          logInfo("rift: waiting for a Rift S on USB (2833:0051, interfaces 6, 7 and 8)");
        }
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, false);
        continue;
      }
      drive(running);
      close();
    }
  }

private:
  bool open() {
    IOHIDManagerRef manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    IOHIDManagerSetDeviceMatching(manager,
                                  (__bridge CFDictionaryRef) @{@kIOHIDVendorIDKey : @(kVendor), @kIOHIDProductIDKey : @(kProduct)});
    IOHIDManagerOpen(manager, kIOHIDOptionsTypeNone);
    std::vector<IOHIDDeviceRef> list = hid::devices(manager);
    for (IOHIDDeviceRef d : list) {
      const int interface = hid::interfaceNumber(d);
      IOHIDDeviceRef *slot = interface == kInterfaceHmd ? &hmd_ : interface == kInterfaceStatus ? &status_
                             : interface == kInterfaceControllers ? &controllers_ : nullptr;
      if (!slot || *slot || IOHIDDeviceOpen(d, kIOHIDOptionsTypeNone) != kIOReturnSuccess)
        continue;
      CFRetain(d);
      *slot = d;
    }
    hid::release(list);
    manager_ = manager;
    if (!hmd_) {
      close();
      return false;
    }
    if (!status_ || !controllers_)
      logInfo("rift: the Rift S's %s interface did not open; %s", !status_ ? "status" : "controller",
              !status_ ? "the screen follows nothing and stays on" : "its Touch controllers are off");
    return true;
  }

  void close() {
    for (IOHIDDeviceRef *d : {&hmd_, &status_, &controllers_}) {
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
    controller_state_.clear();
    devices_.clear();
    imu_configs_.clear();
  }

  void drive(const std::atomic<bool> &running) {
    uint8_t buffer[256];
    size_t n = hid::getFeature(hmd_, kReportDeviceInfo, buffer, sizeof(buffer));
    if (const auto info = parseDeviceInfo(buffer, n))
      logInfo("rift: Rift S panel %ux%u at %u Hz", info->width, info->height, info->refresh_hz);
    else
      logInfo("rift: the Rift S did not report its panel; taking 1440x2560 at 80 Hz");
    n = hid::getFeature(hmd_, kReportImuConfig, buffer, sizeof(buffer));
    if (const auto config = parseImuConfig(buffer, n))
      imu_config_ = *config;
    else
      logInfo("rift: the Rift S did not report its IMU's scales; using defaults");
    const std::string json = readFirmwareBlock(kFirmwareImuCalibration);
    if (const auto calibration = parseImuCalibration(json))
      calibration_ = *calibration;
    else
      logInfo("rift: no IMU calibration from the Rift S (%zu bytes read); its rotation may drift more", json.size());
    startTracking();

    host_->directModel(riftSModel());
    for (const std::vector<uint8_t> &report : enableReports(true))
      if (!hid::setFeature(hmd_, report.data(), report.size()))
        logInfo("rift: the Rift S did not take report 0x%02x", report[0]);
    if (!status_) {
      // no proximity reports: the screen stays on
      const auto on = screenReport(true);
      hid::setFeature(hmd_, on.data(), on.size());
    }
    host_->directOn(true);
    for (int hand = 0; hand < 2; ++hand) {
      // Touch's grip and aim are its own frame turned 40 degrees up about x (Monado's rift_s_controller.c)
      const float half = 20.0f * kPi / 180.0f;
      const mvr_pose up{{std::sin(half), 0, 0, std::cos(half)}, {0, 0, 0}};
      host_->directControllerGrip(hand, up, up);
    }

    IOHIDDeviceRef interfaces[3] = {hmd_, status_, controllers_};
    for (int i = 0; i < 3; ++i) {
      if (!interfaces[i])
        continue;
      // a buffer each: IOHID fills it before the callback
      IOHIDDeviceRegisterInputReportCallback(interfaces[i], input_buffers_[i], sizeof(input_buffers_[i]), onInput, this);
      IOHIDDeviceScheduleWithRunLoop(interfaces[i], CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    }
    last_imu_ns_ = 0;
    auto last_camera_look = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    bool told_cameras = false;
    auto last_keep_alive = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    gone_ = false;
    while (running && !gone_) {
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);
      const auto now = std::chrono::steady_clock::now();
      if (now - last_keep_alive >= std::chrono::milliseconds(kKeepAliveMs)) {
        last_keep_alive = now;
        const auto keep = keepAliveReport();
        if (!hid::setFeature(hmd_, keep.data(), keep.size()) && ++keep_alive_failures_ > 5) {
          logInfo("rift: the Rift S stopped answering; looking for it again");
          gone_ = true;
        }
      }
      serviceControllers();
      if (tracking() && !cameras_ && now - last_camera_look >= std::chrono::seconds(2)) {
        // the camera device appears once the headset is on
        last_camera_look = now;
        startCameras();
        if (!cameras_ && !told_cameras) {
          told_cameras = true;
          logInfo("rift: no \"Rift S Sensor\" camera among macOS's yet; looking again every 2 s");
        }
      }
      uint16_t exposure;
      uint8_t gain;
      if (tracking() && cameras_ && tracking()->exposureChange(exposure, gain)) {
        const auto report = cameraReport(true, exposure, gain, true);
        hid::setFeature(hmd_, report.data(), report.size());
      }
    }
    cameras_.reset(); // no frame arrives after this
    {
      std::lock_guard<std::mutex> guard(tracking_mutex_);
      tracking_.reset();
    }
    for (const std::vector<uint8_t> &report : enableReports(false))
      hid::setFeature(hmd_, report.data(), report.size());
    host_->directOn(false);
  }

  RiftSTracking *tracking() {
    std::lock_guard<std::mutex> guard(tracking_mutex_);
    return tracking_.get();
  }

  // Basalt with this headset's cameras' calibration (firmware block 0x0f) and its IMU's
  void startTracking() {
    const std::string json = readFirmwareBlock(kFirmwareCameraCalibration);
    const auto cameras = parseCameraCalibration(json);
    if (!cameras) {
      logInfo("rift: no camera calibration from the Rift S (%zu bytes read); it tracks rotation only", json.size());
      return;
    }
    std::string why;
    auto tracking = RiftSTracking::start(imu_config_, calibration_, *cameras, &why);
    if (!tracking) {
      logInfo("rift: the Rift S tracks rotation only: %s", why.c_str());
      return;
    }
    std::lock_guard<std::mutex> guard(tracking_mutex_);
    tracking_ = std::move(tracking);
  }

  void startCameras() {
    RiftSTracking *tracking = this->tracking();
    cameras_ = startRiftSCameras([tracking](const uint8_t *grey, uint32_t width, uint32_t height, size_t stride) {
      tracking->frame(grey, width, height, stride);
    });
  }

  // A firmware block, read 56 bytes at a time after its 12-byte header (OpenHMD's rift_s_read_firmware_block).
  std::string readFirmwareBlock(uint8_t block) {
    uint8_t reply[64];
    if (!readFirmware(block, 0, 0x0c, reply))
      return {};
    const uint32_t length = firmwareBlockLength(reply, sizeof(reply));
    std::string out;
    for (uint32_t at = 0; at < length; at += 56) {
      const uint8_t chunk = static_cast<uint8_t>(std::min<uint32_t>(56, length - at));
      if (!readFirmware(block, at + 0x0c, chunk, reply))
        return {};
      out.append(reinterpret_cast<const char *>(reply + 8), chunk);
    }
    return out;
  }

  bool readFirmware(uint8_t block, uint32_t offset, uint8_t length, uint8_t reply[64]) {
    const auto request = firmwareRequest(block, offset, length);
    bool send = true;
    for (int attempt = 0; attempt < 20; ++attempt) {
      if (send && !hid::setFeature(hmd_, request.data(), request.size()))
        return false;
      const size_t n = hid::getFeature(hmd_, kReportFirmwareBlock, reply, 64);
      if (n && firmwareReplyMatches(reply, n, request))
        return true;
      // 1: the read is being done, ask again for its result; 0: the request was not taken, send it again
      send = n < 2 || reply[1] == 0x00;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }

  static void onInput(void *context, IOReturn result, void *, IOHIDReportType, uint32_t id, uint8_t *report,
                      CFIndex length) {
    if (result == kIOReturnSuccess)
      static_cast<RiftSBackend *>(context)->input(id, report, length);
  }

  void input(uint32_t id, const uint8_t *raw, CFIndex length) {
    uint8_t report[256];
    const size_t n = hid::withId(id, raw, length, report, sizeof(report));
    const int64_t received = nowNs();
    keep_alive_failures_ = 0;
    if (report[0] == kInHmd) {
      const auto parsed = parseHmdReport(report, n, imu_config_, calibration_);
      if (!parsed || parsed->count == 0)
        return;
      // the samples are a tick apart and the last arrived just now
      if (RiftSTracking *tracking = this->tracking())
        tracking->imu(*parsed);
      const int64_t tick = 1'000'000'000 / std::max<uint32_t>(imu_config_.hz, 1);
      for (int i = 0; i < parsed->count; ++i) {
        const int64_t t = received - (parsed->count - 1 - i) * tick;
        if (t <= last_imu_ns_)
          continue;
        last_imu_ns_ = t;
        host_->directImu(t, parsed->samples[i].accel, parsed->samples[i].gyro);
      }
    } else if (report[0] == kInStatus) {
      const auto worn = parseStatus(report, n);
      if (!worn || *worn == worn_)
        return;
      worn_ = *worn;
      // the screen follows the proximity sensor, as Oculus's runtime has it
      const auto screen = screenReport(worn_);
      hid::setFeature(hmd_, screen.data(), screen.size());
      host_->directWorn(worn_);
      logInfo(worn_ ? "rift: the Rift S is worn; its screen is on" : "rift: the Rift S is off the face; screen off");
    } else if (report[0] == kInController) {
      ControllerState probe;
      const uint64_t device = parseControllerReport(report, n, probe);
      if (device == 0)
        return;
      ControllerState &state = controller_state_[device];
      parseControllerReport(report, n, state);
      const int hand = handOf(device);
      if (hand < 0)
        return;
      const TouchValues v = touchValues(state);
      DirectControllerInput in;
      in.buttons = (v.primary ? MVR_BUTTON_PRIMARY : 0u) | (v.secondary ? MVR_BUTTON_SECONDARY : 0u) |
                   (v.stick ? MVR_BUTTON_THUMBSTICK : 0u) |
                   (v.menu ? (hand == 0 ? MVR_BUTTON_MENU : MVR_BUTTON_SYSTEM) : 0u);
      in.touches = (v.primary_touch ? MVR_BUTTON_PRIMARY : 0u) | (v.secondary_touch ? MVR_BUTTON_SECONDARY : 0u) |
                   (v.stick_touch ? MVR_BUTTON_THUMBSTICK : 0u) | (v.trigger_touch ? MVR_BUTTON_TRIGGER : 0u);
      in.trigger = std::clamp(v.trigger, 0.0f, 1.0f);
      in.squeeze = std::clamp(v.grip, 0.0f, 1.0f);
      in.thumbstick_x = std::clamp(v.stick_x, -1.0f, 1.0f);
      in.thumbstick_y = std::clamp(v.stick_y, -1.0f, 1.0f);
      host_->directControllerInput(hand, in, received);
      const auto config = imu_configs_.find(device);
      if (probe.has_imu && config != imu_configs_.end()) {
        float accel[3], gyro[3];
        controllerImu(state, config->second, accel, gyro);
        host_->directControllerImu(hand, received, accel, gyro);
      }
    }
  }

  int handOf(uint64_t device) {
    for (const Device &d : devices_)
      if (d.id == device)
        return d.type == kLeftController ? 0 : d.type == kRightController ? 1 : -1;
    wanted_devices_ = true;
    return -1;
  }

  // the radio's slow work, between reports: which controller is which hand, and each one's IMU scales
  void serviceControllers() {
    if (!controllers_ && !hmd_)
      return;
    const auto now = std::chrono::steady_clock::now();
    if (wanted_devices_ && now - devices_read_ > std::chrono::milliseconds(500)) {
      devices_read_ = now;
      uint8_t buffer[200];
      const size_t n = hid::getFeature(hmd_, kReportDevices, buffer, sizeof(buffer));
      const std::vector<Device> list = parseDevices(buffer, n);
      if (!list.empty()) {
        devices_ = list;
        wanted_devices_ = false;
        for (const Device &d : devices_)
          if (d.type == kLeftController || d.type == kRightController)
            logInfo("rift: %s Touch controller %016llx", d.type == kLeftController ? "left" : "right",
                    static_cast<unsigned long long>(d.id));
      }
    }
    for (const Device &d : devices_) {
      if (imu_configs_.count(d.id) || (d.type != kLeftController && d.type != kRightController))
        continue;
      if (now < radio_retry_[d.id])
        continue;
      if (const auto config = readControllerImuConfig(d.id)) {
        imu_configs_[d.id] = *config;
        logInfo("rift: the %s Touch controller's IMU is on", d.type == kLeftController ? "left" : "right");
      } else {
        radio_retry_[d.id] = now + std::chrono::seconds(2); // asleep, or not answering yet
      }
    }
  }

  // one radio command and its reply (OpenHMD's rift_s_radio_update): sent with 0x12, its reply polled on 0x0b
  // until the radio is no longer busy and its sequence number has moved
  std::optional<ControllerImuConfig> readControllerImuConfig(uint64_t device) {
    const auto command = radioCommand(device, kControllerConfigRequest, sizeof(kControllerConfigRequest));
    if (!hid::setFeature(hmd_, command.data(), 61))
      return std::nullopt;
    uint8_t reply[200];
    for (int attempt = 0; attempt < 50; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      const size_t n = hid::getFeature(hmd_, kReportRadioResponse, reply, sizeof(reply));
      if (n < 4)
        return std::nullopt;
      const int seqnum = reply[1];
      const bool busy = reply[2] != 0;
      if (busy || seqnum == last_radio_seqnum_) {
        if (busy)
          last_radio_seqnum_ = seqnum;
        continue;
      }
      last_radio_seqnum_ = seqnum;
      return parseControllerImuConfig(reply + 3, n - 3);
    }
    return std::nullopt;
  }

  DirectHost *host_ = nullptr;
  IOHIDManagerRef manager_ = nullptr;
  IOHIDDeviceRef hmd_ = nullptr, status_ = nullptr, controllers_ = nullptr;
  uint8_t input_buffers_[3][256];
  ImuConfig imu_config_;
  ImuCalibration calibration_;
  int64_t last_imu_ns_ = 0;
  bool worn_ = false;
  bool gone_ = false;
  int keep_alive_failures_ = 0;
  std::map<uint64_t, ControllerState> controller_state_;
  std::vector<Device> devices_;
  bool wanted_devices_ = true;
  std::chrono::steady_clock::time_point devices_read_{};
  std::map<uint64_t, ControllerImuConfig> imu_configs_;
  std::map<uint64_t, std::chrono::steady_clock::time_point> radio_retry_;
  int last_radio_seqnum_ = -1;
  std::mutex tracking_mutex_;
  std::unique_ptr<RiftSTracking> tracking_;
  std::unique_ptr<RiftSCameras> cameras_;
};

} // namespace

std::unique_ptr<DirectBackend> makeRiftSBackend() { return std::make_unique<RiftSBackend>(); }

} // namespace metalvr
