// A headset on the Mac's own video output and USB, other than the Rift CV1 (whose code is in driver_rift.mm): the
// device side of the direct-display driver (docs/HEADSETS.md). The driver owns the window on the headset's display,
// the lens correction, the home, frame pacing and the head's rotation; a backend finds the headset on USB, turns it
// on, and hands the driver what the headset says: which model it is, its IMU samples, whether it is worn, and its
// controllers.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "lighthouse_tracking.h"
#include "metalvr_protocol.h"

namespace metalvr {

// how a headset's panel is laid out and its lenses corrected
struct DirectLens {
  enum Model : uint32_t { CatmullRomCV1 = 0, Panotools = 1, Vive = 2, Psvr2 = 3 };
  uint32_t model = CatmullRomCV1;
  // panotools (Rift S)
  float pano_k[5] = {};
  float pano_aberration[3] = {1, 1, 1};
  float pano_scale = 0;
  float pano_center[2] = {};
  float pano_viewport[2] = {};
  // Vive (Monado's u_vive_values)
  float vive_aspect = 0.9f;
  float vive_grow = 0.5f;
  float vive_center[3][2] = {};
  float vive_coefficients[3][4] = {};
  bool flip_y = false; // the Vive Pro 2's image is upside down
  // PlayStation VR2 (psvr2::lensCalibration): this eye's x and y offset, then cos and sin of its tilt
  float psvr2[4] = {0, 0, 1, 0};
};

struct DirectModel {
  uint32_t headset = MVR_HEADSET_UNKNOWN;
  uint32_t controllers = MVR_CONTROLLERS_UNKNOWN;
  std::string name;
  uint32_t display_vendor = 0;
  std::vector<uint32_t> display_products; // any of these is the headset's display
  float refresh_hz = 90;
  std::vector<float> refresh_rates;
  uint32_t eye_width = 0, eye_height = 0; // the render size applications are asked for
  mvr_fov fov[2] = {};
  mvr_quat eye_rotation[2] = {{0, 0, 0, 1}, {0, 0, 0, 1}}; // canted lenses (Index): each eye's view in the head
  float ipd = 0.0635f;
  bool portrait_eyes = false; // the eyes are the panel's top and bottom halves, each turned a quarter right (Rift S)
  DirectLens lens[2];
};

// a controller's report, in the controller's own frame (+x right, +y up, +z back)
struct DirectControllerInput {
  uint32_t buttons = 0; // mvr_button
  uint32_t touches = 0;
  float trigger = 0, squeeze = 0;
  float thumbstick_x = 0, thumbstick_y = 0;
  float trackpad_x = 0, trackpad_y = 0, trackpad_force = 0, squeeze_force = 0;
  bool has_fingers = false;
  float fingers[4] = {};
  bool has_battery = false;
  float battery = 0;
  bool charging = false;
};

class DirectHost {
public:
  virtual ~DirectHost() = default;
  // the headset is identified: which it is and how to draw for it
  virtual void directModel(const DirectModel &model) = 0;
  // the headset is on (its display and IMU running) or off
  virtual void directOn(bool on) = 0;
  // one IMU sample of the head, in the headset's frame as OpenXR has it; time on the server clock
  virtual void directImu(int64_t time_ns, const float accel[3], const float gyro[3]) = 0;
  virtual void directWorn(bool worn) = 0;
  virtual void directIpd(float meters) = 0;
  // a controller's IMU sample and its inputs; hand 0 left, 1 right. `grip` is the grip pose in the controller's
  // own frame, for its kind (Monado's offsets)
  virtual void directControllerImu(int hand, int64_t time_ns, const float accel[3], const float gyro[3]) = 0;
  virtual void directControllerInput(int hand, const DirectControllerInput &input, int64_t time_ns) = 0;
  virtual void directControllerGrip(int hand, const mvr_pose &grip, const mvr_pose &aim) = 0;
};

class DirectBackend {
public:
  virtual ~DirectBackend() = default;
  virtual const char *name() const = 0;
  // Runs on the driver's USB thread until `running` turns false: finds the headset, turns it on, reports. Returns
  // having turned it off again.
  virtual void run(DirectHost &host, const std::atomic<bool> &running) = 0;
  virtual void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) {}
  // Where a tracker outside the headset (the base stations) has the head (LighthouseTracking::kHead) or a hand, in
  // the tracker's space; false while it has nothing. The driver keeps its own rotation from the IMU and takes the
  // position from here, as it does with a Rift's sensors. Any thread.
  virtual bool trackedPose(int device, TrackedPose &out) { return false; }
  // that tracker's base stations, in its space
  virtual uint32_t trackingReferences(mvr_tracking_reference *out) { return 0; }
};

struct LighthouseOptions {
  int pro2_mode = -1; // the Vive Pro 2's mode to switch to (0..5); -1 leaves it as it is
};

std::unique_ptr<DirectBackend> makeRiftSBackend();
std::unique_ptr<DirectBackend> makeLighthouseBackend(const LighthouseOptions &options);
std::unique_ptr<DirectBackend> makePsvr2Backend();

namespace lighthouse {
struct HeadsetConfig;
}

// the model the Rift S backend reports, and a Lighthouse headset's from its configuration (direct_models.cpp), so
// that what is drawn can be checked without a headset. `pro2_mode` is the Vive Pro 2's mode (pro2Modes()).
DirectModel riftSModel();
DirectModel lighthouseModel(const lighthouse::HeadsetConfig &config, int pro2_mode);
DirectModel riftCv1Model();
// the PlayStation VR2 with its lens calibration (psvr2::lensCalibration)
DirectModel psvr2Model(const std::array<float, 8> &calibration);

// the lens-check grid for `model` with the head straight ahead, drawn as the headset is drawn, into a panel-sized
// BGRA picture (driver_rift.mm): the lens models checked without a headset (metalvrd --panel-check)
// With `passthrough`, the home instead, showing a stereo camera picture red on the left and blue on the right.
bool drawTestPanel(const DirectModel &model, uint32_t width, uint32_t height, std::vector<uint8_t> &bgra,
                   bool passthrough = false);

} // namespace metalvr
