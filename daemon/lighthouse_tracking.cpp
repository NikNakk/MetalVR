#include "lighthouse_tracking.h"

#ifdef METALVR_LIBSURVIVE

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>

#include <hidapi.h>
#include <hidapi_darwin.h>
#include <survive_api.h>

#include "common.h"

namespace metalvr {
namespace {

constexpr double kSeenSeconds = 0.5;

// Monado's survive_driver.c pose_to_relation: libsurvive's world has +z up; its objects look down their -z, which
// is turned forward here
mvr_quat rotationToOpenXr(const LinmathQuat q) {
  const double s = std::sqrt(0.5);
  // down = (x s, y 0, z 0, w -s), applied on the left: down * q
  const double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  const double dw = -s, dx = s;
  mvr_quat out{static_cast<float>(dw * qx + dx * qw), static_cast<float>(dw * qy - dx * qz),
               static_cast<float>(dw * qz + dx * qy), static_cast<float>(dw * qw - dx * qx)};
  const float n = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z + out.w * out.w);
  if (!(n > 0)) return {0, 0, 0, 1};
  return {out.x / n, out.y / n, out.z / n, out.w / n};
}

mvr_vec3 pointToOpenXr(const LinmathVec3d p) {
  return {static_cast<float>(p[0]), static_cast<float>(p[2]), static_cast<float>(-p[1])};
}

void logFromSurvive(SurviveSimpleContext *, SurviveLogLevel level, const char *message) {
  if (level == SURVIVE_LOG_LEVEL_ERROR || level == SURVIVE_LOG_LEVEL_WARNING)
    logInfo("libsurvive: %s", message);
  else
    logDebug("libsurvive: %s", message);
}

class Survive final : public LighthouseTracking {
public:
  bool start(bool simulated) {
    // The Lighthouse backend opens the same HID devices: hidapi must not take them for itself. hid_init sets
    // exclusive opening once, at its first call; libsurvive's own call after this one leaves it as set here.
    if (hid_init() != 0)
      return false;
    hid_darwin_set_open_exclusive(0);
    // libsurvive keeps where the base stations stand (its calibration) beside MetalVR's other settings, so that a
    // room is solved once
    // (the simulated room of a check is kept apart, in a file of its own that starts empty)
    const char *home = std::getenv("HOME");
    config_path_ = simulated ? std::filesystem::temp_directory_path().string()
                             : std::string(home ? home : ".") + "/Library/Application Support/MetalVR";
    std::error_code error;
    std::filesystem::create_directories(config_path_, error);
    config_path_ += simulated ? "/metalvr-libsurvive-simulated.json" : "/libsurvive.json";
    if (simulated)
      std::filesystem::remove(config_path_, error);
    char *args[] = {const_cast<char *>("metalvrd"),
                    const_cast<char *>("--lighthouse-gen"),
                    const_cast<char *>("0"), // 1.0 or 2.0 base stations, whichever are there
                    const_cast<char *>("--globalscenesolver"),
                    const_cast<char *>("1"),
                    const_cast<char *>("--configfile"),
                    config_path_.data(),
                    const_cast<char *>("--simulator"),
                    const_cast<char *>("1")};
    const int count = sizeof(args) / sizeof(args[0]) - (simulated ? 0 : 2);
    context_ = survive_simple_init_with_logger(count, args, logFromSurvive);
    if (!context_)
      return false;
    survive_simple_start_thread(context_);
    logInfo("lighthouse: libsurvive tracks positions with the base stations (its calibration in %s)",
            config_path_.c_str());
    return true;
  }

  ~Survive() override {
    if (context_)
      survive_simple_close(context_);
  }

  bool pose(int device, TrackedPose &out) override {
    std::lock_guard<std::mutex> guard(mutex_);
    const SurviveSimpleObject *object = find(device);
    if (!object)
      return false;
    SurvivePose pose{};
    const FLT time = survive_simple_object_get_latest_pose(object, &pose);
    if (time <= 0 || (pose.Rot[0] == 0 && pose.Rot[1] == 0 && pose.Rot[2] == 0 && pose.Rot[3] == 0))
      return false;
    SurviveVelocity velocity{};
    survive_simple_object_get_latest_velocity(object, &velocity);
    out.orientation = rotationToOpenXr(pose.Rot);
    out.position = pointToOpenXr(pose.Pos);
    out.velocity = pointToOpenXr(velocity.Pos);
    out.position_tracked = survive_simple_run_time(context_) - time < kSeenSeconds;
    return true;
  }

  uint32_t references(mvr_tracking_reference *out) override {
    std::lock_guard<std::mutex> guard(mutex_);
    uint32_t count = 0;
    for (const SurviveSimpleObject *o = survive_simple_get_first_object(context_);
         o && count < MVR_MAX_TRACKING_REFERENCES; o = survive_simple_get_next_object(context_, o)) {
      if (survive_simple_object_get_type(o) != SurviveSimpleObject_LIGHTHOUSE)
        continue;
      mvr_tracking_reference &r = out[count++];
      std::memset(&r, 0, sizeof(r));
      const char *serial = survive_simple_serial_number(o);
      std::snprintf(r.serial, sizeof(r.serial), "%s", serial && *serial ? serial : survive_simple_object_name(o));
      SurvivePose pose{};
      if (survive_simple_object_get_latest_pose(o, &pose) > 0 &&
          (pose.Rot[0] != 0 || pose.Rot[1] != 0 || pose.Rot[2] != 0 || pose.Rot[3] != 0)) {
        r.flags = MVR_REFERENCE_POSE_VALID;
        r.pose.orientation = rotationToOpenXr(pose.Rot);
        r.pose.position = pointToOpenXr(pose.Pos);
      } else {
        r.pose.orientation = {0, 0, 0, 1};
      }
    }
    return count;
  }

  std::vector<std::pair<std::string, TrackedPose>> objects() override {
    std::lock_guard<std::mutex> guard(mutex_);
    std::vector<std::pair<std::string, TrackedPose>> out;
    for (const SurviveSimpleObject *o = survive_simple_get_first_object(context_); o;
         o = survive_simple_get_next_object(context_, o)) {
      SurvivePose pose{};
      const FLT time = survive_simple_object_get_latest_pose(o, &pose);
      if (time <= 0 || (pose.Rot[0] == 0 && pose.Rot[1] == 0 && pose.Rot[2] == 0 && pose.Rot[3] == 0))
        continue;
      TrackedPose p;
      p.orientation = rotationToOpenXr(pose.Rot);
      p.position = pointToOpenXr(pose.Pos);
      p.position_tracked = survive_simple_run_time(context_) - time < kSeenSeconds;
      out.emplace_back(survive_simple_object_name(o), p);
    }
    return out;
  }

private:
  // the head is libsurvive's HMD; a controller is an Index controller of its side, or a Vive wand, the first one
  // seen the left hand
  const SurviveSimpleObject *find(int device) {
    const SurviveSimpleObject *wands[2] = {nullptr, nullptr};
    int wand_count = 0;
    for (const SurviveSimpleObject *o = survive_simple_get_first_object(context_); o;
         o = survive_simple_get_next_object(context_, o)) {
      const auto type = survive_simple_object_get_type(o);
      const auto subtype = survive_simple_object_get_subtype(o);
      if (device == kHead && type == SurviveSimpleObject_HMD)
        return o;
      if (type != SurviveSimpleObject_OBJECT)
        continue;
      if (subtype == SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L && device == kLeft)
        return o;
      if (subtype == SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R && device == kRight)
        return o;
      if (subtype == SURVIVE_OBJECT_SUBTYPE_WAND && wand_count < 2)
        wands[wand_count++] = o;
    }
    return device == kLeft ? wands[0] : device == kRight ? wands[1] : nullptr;
  }

  SurviveSimpleContext *context_ = nullptr;
  std::string config_path_;
  std::mutex mutex_;
};

} // namespace

bool lighthouseTrackingBuilt() { return true; }

std::unique_ptr<LighthouseTracking> startLighthouseTracking(bool simulated) {
  auto survive = std::make_unique<Survive>();
  if (!survive->start(simulated)) {
    logInfo("lighthouse: libsurvive did not start; rotation only");
    return nullptr;
  }
  return survive;
}

} // namespace metalvr

#else

namespace metalvr {
bool lighthouseTrackingBuilt() { return false; }
std::unique_ptr<LighthouseTracking> startLighthouseTracking(bool) { return nullptr; }
} // namespace metalvr

#endif
