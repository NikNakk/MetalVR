// Position tracking from a headset's own cameras and IMU (inside-out; the Rift S, docs/HEADSETS.md phase I): Basalt,
// Monado's fork (BSD-3-Clause), built by scripts/build-basalt.sh as libbasalt.dylib and loaded when tracking starts
// through Monado's VIT interface (daemon/vit/vit_interface.h, BSL-1.0), as Monado loads it. metalvrd neither links
// it nor needs it: without it a headset tracks rotation only.
//
// The tracker is given the IMU's samples in the headset's frame (OpenXR's axes, +y up, +z back), the cameras'
// pictures with their lenses as Kannala-Brandt (KB4) and where each sits from the IMU, all on the headset's clock.
// It hands back where the headset is in a room it builds as it goes, turned into OpenXR's axes.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "lighthouse_tracking.h"

namespace metalvr {

struct TrackerCamera {
  uint32_t width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  std::array<double, 4> kb4{};
  double imu_from_camera[16] = {}; // row-major isometry
  double hz = 30;
};

struct TrackerImu {
  double hz = 1000;
  // the noise and bias walk Monado gives the Rift S's IMU (rift_s_fill_slam_imu_calibration)
  double accel_noise = 0.016, accel_bias = 0.001;
  double gyro_noise = 0.000282, gyro_bias = 0.0001;
  float position[3] = {}; // where the IMU sits in the headset, meters: the pose handed back is the headset's origin
};

class CameraTracking {
public:
  virtual ~CameraTracking() = default;
  // One IMU sample: specific force (m/s^2) and angular velocity (rad/s), in the headset's frame. Times are the
  // headset's, in nanoseconds, increasing.
  virtual void pushImu(int64_t time_ns, const float accel[3], const float gyro[3]) = 0;
  // One picture from every camera, taken together; each `width * height` bytes of grey, rows packed
  virtual void pushFrames(int64_t time_ns, const std::vector<const uint8_t *> &pictures) = 0;
  // The newest pose of the headset's origin: rotation and position in the tracker's room (OpenXR's axes, +y up);
  // false until it has one. `time_ns` is the headset time it is for.
  virtual bool pose(TrackedPose &out, int64_t *time_ns = nullptr) = 0;
};

// Basalt, loaded and started with these sensors; null (with `why` set) if libbasalt.dylib is not found or refuses
// them. It is looked for in $METALVR_BASALT_LIBRARY, next to metalvrd, and in the build's build/basalt/prefix/lib.
std::unique_ptr<CameraTracking> startCameraTracking(const std::vector<TrackerCamera> &cameras, const TrackerImu &imu,
                                                    std::string *why);

} // namespace metalvr
