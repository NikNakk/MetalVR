#include "rift_s_tracking.h"

#include <algorithm>
#include <cmath>

namespace metalvr {

using namespace rift_s;

std::vector<TrackerCamera> riftSTrackerCameras(const std::array<CameraCalibration, kCameraCount> &cameras,
                                               const ImuCalibration &imu) {
  std::vector<TrackerCamera> out;
  for (CameraId id : kTrackerCameraOrder) {
    const CameraCalibration &c = cameras[id];
    TrackerCamera t;
    t.width = c.width;
    t.height = c.height;
    t.fx = t.fy = c.f;
    t.cx = c.cx;
    t.cy = c.cy;
    t.kb4 = kb4FromFisheye62(c);
    t.hz = 30; // every other frame of the 60 Hz feed is for tracking (Monado's CAMERA_FREQUENCY)
    // the IMU's frame has the headset's axes, at the IMU: the camera's rotation is the headset's, its position
    // moved by where the IMU is
    std::copy(c.device_from_camera, c.device_from_camera + 16, t.imu_from_camera);
    for (int a = 0; a < 3; a++) t.imu_from_camera[4 * a + 3] -= imu.imu_position[a];
    out.push_back(t);
  }
  return out;
}

std::unique_ptr<RiftSTracking> RiftSTracking::start(const ImuConfig &config, const ImuCalibration &imu,
                                                    const std::array<CameraCalibration, kCameraCount> &cameras,
                                                    std::string *why) {
  TrackerImu tracker_imu;
  tracker_imu.hz = config.hz;
  std::copy(imu.imu_position, imu.imu_position + 3, tracker_imu.position);
  auto tracker = startCameraTracking(riftSTrackerCameras(cameras, imu), tracker_imu, why);
  if (!tracker) return nullptr;
  return std::make_unique<RiftSTracking>(std::move(tracker), config, cameras);
}

RiftSTracking::RiftSTracking(std::unique_ptr<CameraTracking> tracker, const ImuConfig &config,
                             const std::array<CameraCalibration, kCameraCount> &cameras)
    : tracker_(std::move(tracker)), config_(config), cameras_(cameras) {}

void RiftSTracking::imu(const HmdReport &report) {
  int64_t first_us;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (have_imu_ && report.timestamp_us < last_imu_us_ && last_imu_us_ - report.timestamp_us > 0x80000000u)
      imu_high_++; // the 32-bit clock wrapped (every 71 minutes)
    have_imu_ = true;
    last_imu_us_ = report.timestamp_us;
    first_us = (imu_high_ << 32) + report.timestamp_us;
  }
  const int64_t tick_ns = 1'000'000'000 / std::max<uint32_t>(config_.hz, 1);
  int64_t last_us = first_us;
  for (int i = 0; i < report.count; i++) {
    const int64_t t_ns = first_us * 1000 + i * tick_ns;
    tracker_->pushImu(t_ns, report.samples[i].accel, report.samples[i].gyro);
    last_us = t_ns / 1000;
  }
  std::lock_guard<std::mutex> guard(mutex_);
  latest_imu_us_ = last_us;
}

bool RiftSTracking::frame(const uint8_t *grey, uint32_t width, uint32_t height, size_t stride) {
  const auto info = parseCameraFrameInfo(grey, width, height);
  if (!info || !info->tracking) return false;
  int64_t imu_us;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!have_imu_) return false;
    imu_us = latest_imu_us_;
  }
  // The frame's time is 64-bit and the IMU's 32-bit on the same clock: the frame's low 32 bits, placed in the
  // 2^32 window nearest the IMU's time (Monado's camera_ts_offset)
  const int64_t low = static_cast<int64_t>(info->time_us & 0xffffffffu);
  int64_t frame_us = (imu_us & ~static_cast<int64_t>(0xffffffff)) + low;
  if (frame_us - imu_us > 0x80000000LL) frame_us -= 0x100000000LL;
  if (imu_us - frame_us > 0x80000000LL) frame_us += 0x100000000LL;
  if (!splitTrackingFrame(grey, width, height, stride, *info, cameras_, pictures_)) return false;
  std::vector<const uint8_t *> pointers;
  for (const auto &picture : pictures_) pointers.push_back(picture.data());
  tracker_->pushFrames(frame_us * 1000, pointers);
  frames_tracked_++;
  adjustExposure(pictures_[0]);
  return true;
}

bool RiftSTracking::pose(TrackedPose &out, int64_t *time_ns) { return tracker_->pose(out, time_ns); }

void RiftSTracking::adjustExposure(const std::vector<uint8_t> &picture) {
  // an exposure takes effect two frames after it is sent: change it every fourth tracking frame
  if (++frames_since_exposure_ < 4 || picture.empty()) return;
  frames_since_exposure_ = 0;
  uint64_t sum = 0;
  for (size_t i = 0; i < picture.size(); i += 7) sum += picture[i];
  const double mean = static_cast<double>(sum) / static_cast<double>((picture.size() + 6) / 7);
  const double target = 255.0 / 3.0;
  if (std::fabs(mean - target) < 12) return;
  const double factor = std::clamp(target / std::max(mean, 1.0), 0.5, 2.0);
  std::lock_guard<std::mutex> guard(mutex_);
  const double wanted = exposure_ * factor;
  if (wanted > kMaxExposure && gain_ < 255) {
    exposure_ = kMaxExposure;
    gain_ = static_cast<uint8_t>(std::min(255.0, gain_ * factor));
  } else if (factor < 1 && gain_ > 127) {
    gain_ = static_cast<uint8_t>(std::max(127.0, gain_ * factor));
  } else {
    exposure_ = static_cast<uint16_t>(std::clamp(wanted, double(kMinExposure), double(kMaxExposure)));
  }
}

bool RiftSTracking::exposureChange(uint16_t &exposure, uint8_t &gain) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (exposure_ == sent_exposure_ && gain_ == sent_gain_) return false;
  sent_exposure_ = exposure = exposure_;
  sent_gain_ = gain = gain_;
  return true;
}

} // namespace metalvr
