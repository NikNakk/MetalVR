// The Rift S's position from its cameras (docs/HEADSETS.md, phase I), as Monado's rift_s_tracker.c does it: the
// headset's IMU and its tracking frames on the headset's clock, given to Basalt (camera_tracking.h) with the
// cameras' calibration from the headset's firmware. Nothing here touches the device: the backend
// (backend_rift_s.mm) gives it the reports and frames it receives, and sends the exposure it asks for, so that the
// same path runs on synthetic input (tests/rift_s_tracking_check.cpp).
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "camera_tracking.h"
#include "rift_s_protocol.h"

namespace metalvr {

// The cameras as Basalt is given them, in rift_s::kTrackerCameraOrder: each lens fitted to KB4 and placed from the
// IMU, whose frame is the headset's axes at the IMU's position.
std::vector<TrackerCamera> riftSTrackerCameras(const std::array<rift_s::CameraCalibration, rift_s::kCameraCount> &cameras,
                                               const rift_s::ImuCalibration &imu);

class RiftSTracking {
public:
  // Basalt, started for this headset; null with `why` set if it cannot be
  static std::unique_ptr<RiftSTracking> start(const rift_s::ImuConfig &config, const rift_s::ImuCalibration &imu,
                                              const std::array<rift_s::CameraCalibration, rift_s::kCameraCount> &cameras,
                                              std::string *why);
  // The same with a tracker given (for checking the timing and frame handling without Basalt)
  RiftSTracking(std::unique_ptr<CameraTracking> tracker, const rift_s::ImuConfig &config,
                const std::array<rift_s::CameraCalibration, rift_s::kCameraCount> &cameras);

  // one IMU report, as parsed (its samples in the headset's frame)
  void imu(const rift_s::HmdReport &report);
  // one frame from the camera device: 8-bit grey, rift_s::kCameraFrameWidth wide. Controller frames and frames
  // before the IMU's first are skipped. Returns whether it was given to the tracker.
  bool frame(const uint8_t *grey, uint32_t width, uint32_t height, size_t stride);
  // where the headset is, in the tracker's room (OpenXR's axes)
  bool pose(TrackedPose &out, int64_t *time_ns = nullptr);
  // The exposure and gain the cameras should take, when they changed since last asked (an auto exposure aiming the
  // front-left picture at a third of full brightness, as tracking prefers dark pictures with little blur)
  bool exposureChange(uint16_t &exposure, uint8_t &gain);

  uint64_t framesTracked() const { return frames_tracked_; }

private:
  void adjustExposure(const std::vector<uint8_t> &picture);

  std::unique_ptr<CameraTracking> tracker_;
  rift_s::ImuConfig config_;
  std::array<rift_s::CameraCalibration, rift_s::kCameraCount> cameras_;
  std::mutex mutex_;
  // the IMU's 32-bit microsecond clock, extended
  bool have_imu_ = false;
  uint32_t last_imu_us_ = 0;
  int64_t imu_high_ = 0; // multiples of 2^32 us
  int64_t latest_imu_us_ = 0;
  uint64_t frames_tracked_ = 0;
  std::array<std::vector<uint8_t>, rift_s::kCameraCount> pictures_;
  uint16_t exposure_ = 6000, sent_exposure_ = 0;
  uint8_t gain_ = 127, sent_gain_ = 0;
  int frames_since_exposure_ = 0;
};

} // namespace metalvr
