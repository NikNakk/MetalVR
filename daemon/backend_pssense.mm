// Monado owns PSVR2 USB and Sense HID when optical tracking is enabled.
#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOUSBHost/IOUSBHost.h>

#include "common.h"
#include "direct_backend.h"
#include "pssense/bridge.h"
#include "psvr2_protocol.h"

#include <chrono>
#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>
#include <mach-o/dyld.h>
#include <mutex>
#include <string_view>
#include <thread>

namespace metalvr {
namespace {

uint64_t headsetRegistryId() {
  CFMutableDictionaryRef match = [IOUSBHostDevice createMatchingDictionaryWithVendorID:@(psvr2::kVendor)
                                                                               productID:@(psvr2::kProduct)
                                                                               bcdDevice:nil
                                                                             deviceClass:nil
                                                                          deviceSubclass:nil
                                                                          deviceProtocol:nil
                                                                                   speed:nil
                                                                          productIDArray:nil];
  io_service_t service = IOServiceGetMatchingService(kIOMainPortDefault, match);
  if (!service)
    return 0;
  uint64_t id = 0;
  IORegistryEntryGetRegistryEntryID(service, &id);
  IOObjectRelease(service);
  return id;
}

class SenseBackend final : public DirectBackend {
public:
  SenseBackend(void *library, const mvr_pssense_bridge &bridge) : library_(library), bridge_(bridge) {}
  ~SenseBackend() override { dlclose(library_); }

  const char *name() const override { return "psvr2"; }
  bool usesDevicePoses() const override { return true; }

  void run(DirectHost &host, const std::atomic<bool> &running) override {
    bool waiting = false;
    bool saw_display = false;
    while (running) {
      if (const uint64_t headset = headsetRegistryId()) {
        std::array<float, 8> calibration;
        if (auto *system = bridge_.open(calibration.data())) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            system_ = system;
          }
          waiting = false;
          host.directModel(psvr2Model(calibration));
          host.directOn(true);
          logInfo("psvr2: using Monado for headset and Sense tracking");
          while (running && headsetRegistryId() == headset) {
            mvr_psvr2_status status{};
            bridge_.status(system, &status);
            host.directWorn(status.worn);
            host.directDevicePoseUpdate(nowNs());
            if (status.ipd_m >= 0.055f && status.ipd_m <= 0.075f)
              host.directIpd(status.ipd_m);
            if (status.display_signal != saw_display) {
              saw_display = status.display_signal;
              logInfo(saw_display ? "psvr2: the headset's DisplayPort input has a signal"
                                  : "psvr2: the headset's DisplayPort input lost its signal");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          host.directWorn(false);
          host.directOn(false);
          {
            std::lock_guard<std::mutex> lock(mutex_);
            system_ = nullptr;
          }
          bridge_.close(system);
          saw_display = false;
          continue;
        }
      }
      if (!waiting) {
        logInfo("psvr2: waiting for Monado to open the headset; check USB and close other PSVR2 drivers");
        waiting = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  bool deviceHead(int64_t time_ns, mvr_psvr2_head &out) override {
    std::lock_guard<std::mutex> lock(mutex_);
    return system_ && bridge_.head(system_, time_ns, &out);
  }

  void deviceControllers(int64_t time_ns, mvr_controller_state out[2]) override {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int hand = 0; hand < 2; ++hand) {
      out[hand] = {};
      if (system_)
        bridge_.controller(system_, hand, time_ns, &out[hand]);
    }
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (system_)
      bridge_.haptic(system_, hand, duration_s, frequency_hz, amplitude);
  }

private:
  void *library_;
  const mvr_pssense_bridge &bridge_;
  std::mutex mutex_;
  mvr_pssense_system *system_ = nullptr;
};

} // namespace

std::unique_ptr<DirectBackend> makeSenseBackend() {
  const char *enabled = std::getenv("PSVR2_SENSE_6DOF");
  if (!enabled || std::string_view(enabled) != "1")
    return {};

  const char *calibration = std::getenv("PSVR2_SENSE_6DOF_CALIBRATION");
  std::error_code error;
  if (!calibration || !std::filesystem::path(calibration).is_absolute() ||
      !std::filesystem::is_regular_file(calibration, error)) {
    logInfo("psvr2: set PSVR2_SENSE_6DOF_CALIBRATION to an absolute calibration file path for Sense 6DoF");
    return {};
  }

  std::vector<std::filesystem::path> candidates;
  if (const char *path = std::getenv("METALVR_PSSENSE_LIBRARY"); path && *path) {
    candidates.emplace_back(path);
  } else {
    char executable[4096];
    uint32_t size = sizeof(executable);
    if (_NSGetExecutablePath(executable, &size) == 0)
      candidates.push_back(std::filesystem::path(executable).parent_path() / "../lib/libmetalvr_pssense.dylib");
    candidates.emplace_back(METALVR_PSSENSE_LIBRARY);
  }
  for (const auto &path : candidates) {
    if (!std::filesystem::is_regular_file(path, error))
      continue;
    void *library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!library) {
      logInfo("psvr2: cannot load Sense tracking library: %s", dlerror());
      continue;
    }
    const auto get_bridge = reinterpret_cast<mvr_pssense_get_bridge_fn>(dlsym(library, "mvr_pssense_get_bridge"));
    if (const auto *bridge = get_bridge ? get_bridge(MVR_PSSENSE_BRIDGE_VERSION) : nullptr)
      return std::make_unique<SenseBackend>(library, *bridge);
    logInfo("psvr2: incompatible Sense tracking library at %s", path.c_str());
    dlclose(library);
  }
  logInfo("psvr2: Sense tracking library unavailable; run scripts/build-pssense.sh");
  return {};
}

} // namespace metalvr
