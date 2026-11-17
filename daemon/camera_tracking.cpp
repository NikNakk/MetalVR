#include "camera_tracking.h"

#include <dlfcn.h>
#include <mach-o/dyld.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <mutex>

#include "common.h"
#include "vit/vit_interface.h"

#ifndef METALVR_BASALT_DIR
#define METALVR_BASALT_DIR ""
#endif

namespace metalvr {
namespace {

// the VIT functions, from the loaded library
struct Vit {
  void *library = nullptr;
#define VIT_FUNCTION(name) PFN_vit_##name name = nullptr;
  VIT_FUNCTION(api_get_version)
  VIT_FUNCTION(tracker_create)
  VIT_FUNCTION(tracker_destroy)
  VIT_FUNCTION(tracker_start)
  VIT_FUNCTION(tracker_stop)
  VIT_FUNCTION(tracker_push_imu_sample)
  VIT_FUNCTION(tracker_push_img_sample)
  VIT_FUNCTION(tracker_add_imu_calibration)
  VIT_FUNCTION(tracker_add_camera_calibration)
  VIT_FUNCTION(tracker_pop_pose)
  VIT_FUNCTION(pose_destroy)
  VIT_FUNCTION(pose_get_data)
#undef VIT_FUNCTION

  bool load(const std::string &path, std::string &why) {
    library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!library) {
      why = dlerror();
      return false;
    }
    bool all = true;
#define VIT_FUNCTION(name)                                                                                             \
  name = reinterpret_cast<PFN_vit_##name>(dlsym(library, "vit_" #name));                                              \
  all = all && name;
    VIT_FUNCTION(api_get_version)
    VIT_FUNCTION(tracker_create)
    VIT_FUNCTION(tracker_destroy)
      VIT_FUNCTION(tracker_start)
    VIT_FUNCTION(tracker_stop)
    VIT_FUNCTION(tracker_push_imu_sample)
    VIT_FUNCTION(tracker_push_img_sample)
    VIT_FUNCTION(tracker_add_imu_calibration)
    VIT_FUNCTION(tracker_add_camera_calibration)
    VIT_FUNCTION(tracker_pop_pose)
    VIT_FUNCTION(pose_destroy)
    VIT_FUNCTION(pose_get_data)
#undef VIT_FUNCTION
    if (!all) {
      why = path + " is not a VIT tracker";
      return false;
    }
    uint32_t major = 0, minor = 0, patch = 0;
    if (api_get_version(&major, &minor, &patch) != VIT_SUCCESS || major != VIT_HEADER_VERSION_MAJOR) {
      why = path + ": VIT " + std::to_string(major) + "." + std::to_string(minor) + ", not " +
            std::to_string(VIT_HEADER_VERSION_MAJOR);
      return false;
    }
    return true;
  }
};

std::vector<std::string> libraryCandidates() {
  std::vector<std::string> out;
  if (const char *path = std::getenv("METALVR_BASALT_LIBRARY"); path && *path) out.emplace_back(path);
  char executable[4096];
  uint32_t size = sizeof(executable);
  if (_NSGetExecutablePath(executable, &size) == 0) {
    const std::filesystem::path dir = std::filesystem::path(executable).parent_path();
    out.push_back((dir / "libbasalt.dylib").string());
    out.push_back((dir / "../lib/libbasalt.dylib").string());
  }
  if (*METALVR_BASALT_DIR) out.push_back(std::string(METALVR_BASALT_DIR) + "/libbasalt.dylib");
  return out;
}

// Basalt's room has gravity down its -z; OpenXR's has +y up: a quarter turn about x (as libsurvive's, in
// lighthouse_tracking.cpp)
constexpr double kHalf = 0.70710678118654752440;

class Basalt final : public CameraTracking {
public:
  bool start(const std::vector<TrackerCamera> &cameras, const TrackerImu &imu, std::string &why) {
    for (const std::string &path : libraryCandidates()) {
      std::error_code error;
      if (!std::filesystem::exists(path, error)) continue;
      if (vit_.load(path, why)) {
        path_ = path;
        break;
      }
      logInfo("camera tracking: %s", why.c_str());
    }
    if (!vit_.library) {
      if (why.empty()) why = "libbasalt.dylib not found (scripts/build-basalt.sh)";
      return false;
    }
    if (cameras.size() < 2) {
      why = "Basalt needs two cameras or more";
      return false;
    }
    imu_position_[0] = imu.position[0];
    imu_position_[1] = imu.position[1];
    imu_position_[2] = imu.position[2];
    vit_config_t config{};
    config.file = nullptr; // the calibration below and Basalt's default pipeline, as Monado does with a driver's
    config.cam_count = static_cast<uint32_t>(cameras.size());
    config.imu_count = 1;
    if (vit_.tracker_create(&config, &tracker_) != VIT_SUCCESS || !tracker_) {
      why = "Basalt did not create a tracker";
      return false;
    }
    // 8-bit grey is what Basalt's push_frame takes; its has_image_format answers no to everything (it overwrites
    // its own answer), so it is not asked, as Monado does not ask it
    for (size_t i = 0; i < cameras.size(); i++) {
      const TrackerCamera &c = cameras[i];
      vit_camera_calibration_t calibration{};
      calibration.camera_index = static_cast<uint32_t>(i);
      calibration.width = static_cast<int>(c.width);
      calibration.height = static_cast<int>(c.height);
      calibration.frequency = c.hz;
      calibration.fx = c.fx;
      calibration.fy = c.fy;
      calibration.cx = c.cx;
      calibration.cy = c.cy;
      calibration.model = VIT_CAMERA_DISTORTION_KB4;
      calibration.distortion_count = 4;
      for (int k = 0; k < 4; k++) calibration.distortion[k] = c.kb4[k];
      for (int k = 0; k < 16; k++) calibration.transform[k] = c.imu_from_camera[k];
      if (vit_.tracker_add_camera_calibration(tracker_, &calibration) != VIT_SUCCESS) {
        why = "Basalt refused camera " + std::to_string(i) + "'s calibration";
        return false;
      }
    }
    // the samples are given already rectified: identity transforms, no offsets (as Monado gives the Rift S's)
    vit_imu_calibration_t calibration{};
    calibration.imu_index = 0;
    calibration.frequency = imu.hz;
    for (vit_inertial_calibration *inertial : {&calibration.accel, &calibration.gyro}) {
      inertial->transform[0] = inertial->transform[4] = inertial->transform[8] = 1;
      const bool accel = inertial == &calibration.accel;
      for (int a = 0; a < 3; a++) {
        inertial->noise_std[a] = accel ? imu.accel_noise : imu.gyro_noise;
        inertial->bias_std[a] = accel ? imu.accel_bias : imu.gyro_bias;
      }
    }
    if (vit_.tracker_add_imu_calibration(tracker_, &calibration) != VIT_SUCCESS) {
      why = "Basalt refused the IMU's calibration";
      return false;
    }
    if (vit_.tracker_start(tracker_) != VIT_SUCCESS) {
      why = "Basalt did not start";
      return false;
    }
    running_ = true;
    logInfo("camera tracking: Basalt (%s) with %zu cameras", path_.c_str(), cameras.size());
    return true;
  }

  ~Basalt() override {
    if (tracker_) {
      if (running_) vit_.tracker_stop(tracker_);
      vit_.tracker_destroy(tracker_);
    }
    // libbasalt stays loaded: its threads' static state is not made to be unloaded
  }

  void pushImu(int64_t time_ns, const float accel[3], const float gyro[3]) override {
    std::lock_guard<std::mutex> guard(push_mutex_);
    if (time_ns <= last_imu_ns_) return; // Basalt takes only increasing times
    last_imu_ns_ = time_ns;
    vit_imu_sample_t sample{};
    sample.timestamp = time_ns;
    sample.ax = accel[0], sample.ay = accel[1], sample.az = accel[2];
    sample.wx = gyro[0], sample.wy = gyro[1], sample.wz = gyro[2];
    vit_.tracker_push_imu_sample(tracker_, &sample);
  }

  void pushFrames(int64_t time_ns, const std::vector<const uint8_t *> &pictures) override {
    {
      std::lock_guard<std::mutex> guard(push_mutex_);
      if (time_ns <= last_frame_ns_) return;
      last_frame_ns_ = time_ns;
      for (size_t i = 0; i < pictures.size(); i++) {
        vit_img_sample_t sample{};
        sample.cam_index = static_cast<uint32_t>(i);
        sample.timestamp = time_ns;
        sample.data = const_cast<uint8_t *>(pictures[i]);
        sample.format = VIT_IMAGE_FORMAT_L8;
        sizes(i, sample);
        vit_.tracker_push_img_sample(tracker_, &sample);
      }
    }
    drainPoses();
  }

  bool pose(TrackedPose &out, int64_t *time_ns) override {
    drainPoses();
    std::lock_guard<std::mutex> guard(pose_mutex_);
    if (!have_pose_) return false;
    out = pose_;
    if (time_ns) *time_ns = pose_time_ns_;
    return true;
  }

  void setSizes(const std::vector<TrackerCamera> &cameras) {
    for (const TrackerCamera &c : cameras) {
      widths_.push_back(c.width);
      heights_.push_back(c.height);
    }
  }

private:
  void sizes(size_t i, vit_img_sample_t &sample) const {
    sample.width = widths_[i];
    sample.height = heights_[i];
    sample.stride = widths_[i];
    sample.size = widths_[i] * heights_[i];
  }

  void drainPoses() {
    vit_pose_t *pose = nullptr;
    while (vit_.tracker_pop_pose(tracker_, &pose) == VIT_SUCCESS && pose) {
      vit_pose_data_t data{};
      if (vit_.pose_get_data(pose, &data) == VIT_SUCCESS) take(data);
      vit_.pose_destroy(pose);
      pose = nullptr;
    }
  }

  // Basalt's pose of the IMU (its body: the headset's axes, at the IMU) in its room, to the headset's origin in
  // OpenXR's axes
  void take(const vit_pose_data_t &d) {
    const double qx = d.ox, qy = d.oy, qz = d.oz, qw = d.ow;
    // the headset's origin is -imu_position in the body's frame: rotate it into the room
    const double v[3] = {-imu_position_[0], -imu_position_[1], -imu_position_[2]};
    double r[3];
    rotate(qx, qy, qz, qw, v, r);
    const double p[3] = {d.px + r[0], d.py + r[1], d.pz + r[2]};
    // c = a quarter turn about x, -90 degrees: (x, y, z) -> (x, z, -y); q' = c q
    const double cx = -kHalf, cw = kHalf;
    TrackedPose out;
    out.orientation = {static_cast<float>(cw * qx + cx * qw), static_cast<float>(cw * qy - cx * qz),
                       static_cast<float>(cw * qz + cx * qy), static_cast<float>(cw * qw - cx * qx)};
    out.position = {static_cast<float>(p[0]), static_cast<float>(p[2]), static_cast<float>(-p[1])};
    out.velocity = {d.vx, d.vz, -d.vy};
    out.position_tracked = std::isfinite(out.position.x) && std::isfinite(out.position.y) &&
                           std::isfinite(out.position.z);
    std::lock_guard<std::mutex> guard(pose_mutex_);
    pose_ = out;
    pose_time_ns_ = d.timestamp;
    have_pose_ = true;
  }

  static void rotate(double x, double y, double z, double w, const double v[3], double out[3]) {
    // v + 2 w (q x v) + 2 q x (q x v)
    const double tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
  }

  Vit vit_;
  std::string path_;
  vit_tracker_t *tracker_ = nullptr;
  bool running_ = false;
  double imu_position_[3] = {};
  std::vector<uint32_t> widths_, heights_;
  std::mutex push_mutex_, pose_mutex_;
  int64_t last_imu_ns_ = 0, last_frame_ns_ = 0;
  TrackedPose pose_;
  int64_t pose_time_ns_ = 0;
  bool have_pose_ = false;
};

} // namespace

std::unique_ptr<CameraTracking> startCameraTracking(const std::vector<TrackerCamera> &cameras, const TrackerImu &imu,
                                                    std::string *why) {
  auto basalt = std::make_unique<Basalt>();
  basalt->setSizes(cameras);
  std::string reason;
  if (!basalt->start(cameras, imu, reason)) {
    if (why) *why = reason;
    return nullptr;
  }
  return basalt;
}

} // namespace metalvr
