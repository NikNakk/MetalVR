// The simulated headset: deterministic poses and inputs, and an optional dump of the encoded stream. It exists so
// the whole path from an application's swapchain to encoded video can be exercised with no headset attached.
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

#include "common.h"
#include "driver.h"
#include "headsets.h"
#include "metalvr_math.h"

namespace metalvr {

namespace {

constexpr float kPi = 3.14159265358979f;

class SimDriver final : public Driver {
public:
  explicit SimDriver(const SimOptions &options) : options_(options) {}
  ~SimDriver() override { stop(); }

  const char *name() const override { return "sim"; }

  bool start() override {
    start_ns_ = nowNs();
    if (!options_.dump_path.empty()) {
      dump_ = std::fopen(options_.dump_path.c_str(), "wb");
      if (!dump_) {
        logInfo("sim: cannot write %s", options_.dump_path.c_str());
        return false;
      }
    }
    return true;
  }

  void stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dump_) {
      std::fclose(dump_);
      dump_ = nullptr;
    }
    if (frames_ && !reported_) {
      reported_ = true;
      logInfo("sim: %llu frames (%llu keyframes), %.2f MB encoded", static_cast<unsigned long long>(frames_),
              static_cast<unsigned long long>(keyframes_), bytes_ / 1e6);
    }
  }

  bool connected() override { return true; }

  mvr_system_info systemInfo() override {
    mvr_system_info info{};
    info.flags = MVR_SYSTEM_HMD_CONNECTED | MVR_SYSTEM_SIMULATED | MVR_SYSTEM_HAND_TRACKING;
    info.eye_width = options_.eye_width;
    info.eye_height = options_.eye_height;
    info.refresh_hz = options_.refresh_hz;
    info.ipd = kIpd;
    info.fov[0] = {-0.942f, 0.698f, 0.768f, -0.960f};
    info.fov[1] = {-0.698f, 0.942f, 0.768f, -0.960f};
    const mvr_headset_description *description = mvr_describe_headset(options_.headset);
    info.headset = options_.headset;
    info.controllers = description->controllers;
    if (options_.headset == MVR_HEADSET_SIMULATED)
      std::snprintf(info.system_name, sizeof(info.system_name), "MetalVR Simulated Headset");
    else
      std::snprintf(info.system_name, sizeof(info.system_name), "%s (simulated)", description->name);
    const HeadsetSpec &spec = headsetSpec(options_.headset);
    info.refresh_rate_count = 0;
    bool listed = false;
    for (uint32_t i = 0; i < spec.refresh_rate_count; ++i) {
      info.refresh_rates[info.refresh_rate_count++] = spec.refresh_rates[i];
      listed = listed || std::fabs(spec.refresh_rates[i] - options_.refresh_hz) < 0.5f;
    }
    if (!listed) {
      info.refresh_rate_count = 1;
      info.refresh_rates[0] = options_.refresh_hz;
    }
    info.play_area_width = 2.0f;
    info.play_area_depth = 1.5f;
    return info;
  }

  uint64_t generation() override { return 1; }

  void tracking(int64_t target_time_ns, mvr_tracking_response &out) override {
    const int64_t now = nowNs();
    const int64_t time = target_time_ns > 0 ? target_time_ns : now;
    const float t = options_.animate ? static_cast<float>((time - start_ns_) / 1e9) : 0.0f;

    out.flags = MVR_TRACKING_HMD_CONNECTED | MVR_TRACKING_HEAD_VALID | MVR_TRACKING_PRESENTING;
    out.sample_id = ++sample_;
    out.time_ns = time;

    // a seated head that looks slowly left and right
    const float yaw = 0.35f * std::sin(t * 2.0f * kPi * 0.2f);
    const float yaw_rate = 0.35f * 2.0f * kPi * 0.2f * std::cos(t * 2.0f * kPi * 0.2f);
    out.head.orientation = quatAxisAngle({0, 1, 0}, yaw);
    out.head.position = {0.0f, 1.6f + 0.01f * std::sin(t * 2.0f * kPi * 0.5f), 0.0f};
    out.head_linear_velocity = {0, options_.animate ? 0.01f * kPi * std::cos(t * kPi) : 0.0f, 0};
    out.head_angular_velocity = {0, options_.animate ? yaw_rate : 0.0f, 0};

    const mvr_system_info info = systemInfo();
    for (int eye = 0; eye < 2; ++eye) {
      const float offset = (eye == 0 ? -0.5f : 0.5f) * kIpd;
      out.views[eye].pose = poseMul(out.head, mvr_pose{quatIdentity(), {offset, 0, 0}});
      out.views[eye].fov = info.fov[eye];
    }

    for (int hand = 0; hand < 2; ++hand) {
      mvr_controller_state &c = out.controllers[hand];
      std::memset(&c, 0, sizeof(c));
      c.flags = MVR_CONTROLLER_CONNECTED | MVR_CONTROLLER_POSE_VALID;
      const float side = hand == 0 ? -1.0f : 1.0f;
      // held in front of the chest, pointing forward, drifting a little
      const mvr_pose in_head{quatAxisAngle({1, 0, 0}, -0.3f),
                             {side * 0.2f, -0.35f + 0.03f * std::sin(t * 1.3f + hand), -0.4f}};
      // controllers follow the body's heading, not the head's pitch
      const mvr_pose body{quatYawOnly(out.head.orientation), out.head.position};
      c.grip = poseMul(body, in_head);
      c.aim = c.grip;
      c.flags |= MVR_CONTROLLER_BATTERY_VALID;
      c.battery = hand == 0 ? 0.8f : 0.6f;
      if (options_.animate) {
        // the right trigger ramps over four seconds; the left primary button is down every other second
        if (hand == 1) {
          c.trigger = std::fmod(t, 4.0f) / 4.0f;
          if (c.trigger > 0.8f)
            c.buttons |= MVR_BUTTON_TRIGGER;
          if (c.trigger > 0.05f)
            c.touches |= MVR_BUTTON_TRIGGER;
          c.thumbstick_x = std::sin(t);
          c.thumbstick_y = std::cos(t);
        } else if (static_cast<int64_t>(t) % 2 == 1) {
          c.buttons |= MVR_BUTTON_PRIMARY;
          c.touches |= MVR_BUTTON_PRIMARY;
        }
        // what the controllers of the headset stood in for have besides
        const uint32_t kind = mvr_describe_headset(options_.headset)->controllers;
        if (kind == MVR_CONTROLLERS_VIVE_WAND) {
          // no stick: the thumb's circle is on the trackpad
          c.trackpad_x = c.thumbstick_x;
          c.trackpad_y = c.thumbstick_y;
          c.thumbstick_x = c.thumbstick_y = 0;
          if (c.trackpad_x != 0 || c.trackpad_y != 0)
            c.touches |= MVR_BUTTON_TRACKPAD;
        } else if (kind == MVR_CONTROLLERS_INDEX) {
          c.flags |= MVR_CONTROLLER_FINGERS_VALID;
          for (int finger = 0; finger < 4; ++finger)
            c.fingers[finger] = 0.5f + 0.5f * std::sin(t + finger * 0.5f);
          c.squeeze = c.fingers[MVR_FINGER_MIDDLE];
          c.trackpad_x = 0.5f * std::cos(t);
          c.trackpad_y = 0.5f * std::sin(t);
          c.touches |= MVR_BUTTON_TRACKPAD;
        } else if (kind == MVR_CONTROLLERS_STEAM_FRAME && static_cast<int64_t>(t) % 3 == 2) {
          c.buttons |= hand == 0 ? MVR_BUTTON_DPAD_UP : MVR_BUTTON_BUMPER;
        }
      }
    }
  }

  // the hands where the controllers are, open
  void hands(int64_t target_time_ns, mvr_hand out[2]) override {
    mvr_tracking_response tracking{};
    this->tracking(target_time_ns, tracking);
    for (int hand = 0; hand < 2; ++hand)
      openHand(tracking.controllers[hand].grip, hand == 0, out[hand]);
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) override {
    ++haptics_;
    logDebug("sim: haptic hand %u %.3fs %.0fHz amplitude %.2f", hand, duration_s, frequency_hz, amplitude);
  }

  VideoParams videoParams() override {
    VideoParams params;
    params.codec = options_.codec;
    params.frame_width = options_.eye_width * 2;
    params.frame_height = options_.eye_height;
    if (options_.foveated_encoding) {
      // ALVR's default foveated_encoding settings
      params.foveation = computeFoveation(options_.eye_width, options_.eye_height, 0.45f, 0.4f, 0.4f, 0.1f, 4.0f, 5.0f);
      params.frame_width = params.foveation.optimized_eye_width * 2;
      params.frame_height = params.foveation.optimized_eye_height;
    }
    params.fps = options_.refresh_hz;
    params.bitrate_bps = options_.bitrate_bps;
    return params;
  }

  bool consumeKeyframeRequest() override { return false; }

  void sendVideoConfig(Codec, const uint8_t *annexb, size_t size) override {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.assign(annexb, annexb + size);
  }

  void sendFrame(const FrameInfo &frame, const uint8_t *annexb, size_t size) override {
    const bool keyframe = frame.keyframe;
    std::lock_guard<std::mutex> lock(mutex_);
    ++frames_;
    bytes_ += size;
    if (keyframe)
      ++keyframes_;
    if (!dump_)
      return;
    // a file that starts at any keyframe decodes: the parameter sets go in front of each one
    if (keyframe && !config_.empty())
      std::fwrite(config_.data(), 1, config_.size(), dump_);
    std::fwrite(annexb, 1, size, dump_);
    std::fflush(dump_);
  }

private:
  static constexpr float kIpd = 0.063f;

  SimOptions options_;
  int64_t start_ns_ = 0;
  std::atomic<uint64_t> sample_{0};
  std::atomic<uint64_t> haptics_{0};
  std::mutex mutex_;
  std::FILE *dump_ = nullptr;
  std::vector<uint8_t> config_;
  uint64_t frames_ = 0, keyframes_ = 0, bytes_ = 0;
  bool reported_ = false;
};

} // namespace

std::unique_ptr<Driver> makeSimDriver(const SimOptions &options) { return std::make_unique<SimDriver>(options); }

} // namespace metalvr
