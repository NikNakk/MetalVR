// The USB side of Valve's Lighthouse headsets and controllers (the HTC Vive, Vive Pro and Vive Pro 2, and the
// Valve Index): which one is plugged in, the calibration each keeps in its own firmware, its IMU reports, and the
// Watchman radio its controllers report through (docs/HEADSETS.md).
//
// Nothing here touches a device: these are the reports' layouts, taken in and made as bytes, so that each can be
// checked without a headset (tests/headset_protocol_check.cpp). The layouts and the arithmetic follow Monado's Vive
// driver (src/xrt/drivers/vive, src/xrt/auxiliary/vive, src/xrt/drivers/vp2 and u_distortion_mesh.c; Copyright
// 2016-2019 Philipp Zabel, 2019-2023 Collabora, Ltd. and others; BSL-1.0, https://gitlab.freedesktop.org/monado/monado).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "metalvr_protocol.h"

namespace metalvr::lighthouse {

constexpr uint16_t kHtcVendor = 0x0bb4;
constexpr uint16_t kValveVendor = 0x28de;
constexpr uint16_t kViveMainboard = 0x2c87;      // HTC Vive
constexpr uint16_t kViveProMainboard = 0x0309;   // HTC Vive Pro (Monado, OpenHMD)
constexpr uint16_t kViveProMainboardB = 0x030e;  // HTC Vive Pro (libsurvive)
constexpr uint16_t kVivePro2Mainboard = 0x0342;  // HTC Vive Pro 2
constexpr uint16_t kViveSensors = 0x2000;        // the Vive's IMU and Lighthouse receiver
constexpr uint16_t kLhrSensors = 0x2300;         // the Vive Pro's, Vive Pro 2's and Index's
constexpr uint16_t kWatchmanGen1 = 0x2101;       // a receiver for Vive wands
constexpr uint16_t kWatchmanGen2 = 0x2102;       // a receiver for Index controllers

enum class Model { None, Vive, VivePro, VivePro2, Index };

// mvr_headset of a model
uint32_t headsetOf(Model model);
const char *nameOf(Model model);

struct UsbId {
  uint16_t vendor = 0;
  uint16_t product = 0;
};

// Which headset the devices on USB are, or None. An HTC mainboard names its headset; Valve's IMU device 2300 with no
// HTC mainboard is an Index only once its own configuration says so (modelFromConfig), since a Vive Pro whose
// mainboard is not seen, or a Tracker, shows the same device: here it is Index, to be confirmed.
Model identify(const std::vector<UsbId> &devices);

// the model a headset's configuration names (its "model_number"), or None
Model modelFromConfig(std::string_view model_number);

// ---- the configuration in a device's firmware: feature report 0x10 starts it, 0x11 reports carry it zlib-compressed

constexpr uint8_t kConfigStartReport = 0x10;
constexpr uint8_t kConfigReadReport = 0x11;

// Gathers the 0x11 reports' payloads. add() takes one report (with its id); false when it is not one or the data
// is too much. done() once a report with no payload has come.
class ConfigAssembler {
public:
  bool add(const uint8_t *report, size_t size);
  bool done() const { return done_; }
  // the JSON, inflated; nothing if it does not inflate
  std::optional<std::string> json() const;

private:
  std::vector<uint8_t> compressed_;
  bool done_ = false;
};

std::optional<std::string> inflateConfig(const std::vector<uint8_t> &compressed);

struct Vec3 {
  float x = 0, y = 0, z = 0;
};

// one eye's lens, as the configuration gives it (Monado's u_vive_values)
struct Distortion {
  float aspect_x_over_y = 0.9f;
  float grow_for_undistort = 0.5f;
  float intrinsic_focus[2] = {};
  float intrinsic_center[2] = {};
  float undistort_r2_cutoff = 1.0f;
  float center[3][2] = {}; // red, green, blue
  float coefficients[3][4] = {};
};

struct HeadsetConfig {
  std::string model_number;
  std::string serial;
  Model model = Model::None;
  Vec3 acc_bias, acc_scale{1, 1, 1}, gyro_bias, gyro_scale{1, 1, 1};
  int eye_width = 1080; // the panel's pixels for one eye
  int eye_height = 1200;
  float lens_separation = 0; // meters; 0 if the configuration has none (the IPD is the headset's to report)
  Distortion distortion[2];
  mvr_fov fov[2] = {};
  // each eye's view in the head ("eye_to_head"): the Index's lenses are canted
  mvr_quat eye_rotation[2] = {{0, 0, 0, 1}, {0, 0, 0, 1}};
};

std::optional<HeadsetConfig> parseHeadsetConfig(std::string_view json);

// Where each colour of the point (u, v) of an eye's view on the panel (0..1 across and down) is in the rendered
// eye image (0..1 across and down): Monado's u_compute_distortion_vive. The Vive Pro 2's image is upside down
// against the others' (flip_y).
void computeDistortion(const Distortion &d, float u, float v, bool flip_y, float out[3][2]);

// ---- the IMU: report 0x20, three samples of an MPU-6500 in round robin; its ranges from feature report 0x01

constexpr uint8_t kImuReport = 0x20;
constexpr size_t kImuReportSize = 52;
constexpr uint8_t kImuRangeReport = 0x01;

struct ImuRanges {
  float gyro = 8.726646f;  // rad/s at full scale (500 degrees/s)
  float accel = 39.2266f;  // m/s^2 at full scale (4 g)
};

std::optional<ImuRanges> parseImuRanges(const uint8_t *report, size_t size);

struct ImuSample {
  uint32_t ticks = 0; // the device's 48 MHz clock
  float accel[3] = {}; // m/s^2, in the headset's frame as OpenXR has it (+x right, +y up, +z back)
  float gyro[3] = {};  // rad/s
};

constexpr double kTicksPerSecond = 48e6;

// The samples of an IMU report not seen before, oldest first. `sequence` is the newest sequence number taken,
// `have_sequence` whether there is one yet; both are updated.
size_t parseImuReport(const uint8_t *report, size_t size, const HeadsetConfig &config, const ImuRanges &ranges,
                      uint8_t &sequence, bool &have_sequence, ImuSample out[3]);

// ---- the HTC mainboard (Vive Pro, Vive Pro 2): status report 0x03, and feature report 0x04 commands

constexpr uint8_t kMainboardStatusReport = 0x03;

struct MainboardStatus {
  float ipd = 0;             // meters
  float lens_separation = 0; // meters
  uint16_t proximity = 0;
};

std::optional<MainboardStatus> parseMainboardStatus(const uint8_t *report, size_t size);

// whether a proximity reading means the headset is on a face (a guess from the community Vive Pro 2 driver)
constexpr uint16_t kWornProximity = 150;

// feature report 0x04 of type 0x2978: the headset's power on and off (Vive, Vive Pro)
std::array<uint8_t, 64> powerReport(bool on);

// the Vive Pro 2's modes (docs/HEADSETS.md), which `dtd,N` selects
struct Pro2Mode {
  int index;
  int width, height; // both eyes
  float refresh_hz;
  bool needs_dsc;    // only with Display Stream Compression
};
const std::array<Pro2Mode, 6> &pro2Modes();
// feature report 0x04 of type 0x2970 carrying a text command ("wireless,0", "dtd,2", "chipreset")
std::array<uint8_t, 64> pro2Command(std::string_view text);

// ---- controllers, through a Watchman receiver: reports 0x23 (one message) and 0x24 (two)

constexpr uint8_t kControllerReport1 = 0x23;
constexpr uint8_t kControllerReport2 = 0x24;
constexpr uint8_t kControllerDisconnect = 0x26;

enum class ControllerKind { Unknown, Wand, IndexLeft, IndexRight };

struct ControllerConfig {
  std::string model_number;
  ControllerKind kind = ControllerKind::Unknown;
  Vec3 acc_bias, acc_scale{1, 1, 1}, gyro_bias, gyro_scale{1, 1, 1};
};

std::optional<ControllerConfig> parseControllerConfig(std::string_view json);

// what a controller has reported so far
struct ControllerState {
  // Vive wand bits: trigger click 0, trackpad touch 1, trackpad click 2, system 3, grip 4, menu 5.
  // Index bits: trigger click 0, trackpad touch 1, thumbstick click 2, system 3, A 4, B 5.
  uint8_t buttons = 0;
  // Index: touched, as buttons' bits 3..6 (system 3, A 4, B 5, thumbstick 6)
  uint8_t touch = 0;
  float trigger = 0;
  float pad_x = 0, pad_y = 0; // the trackpad, or on an Index the thumbstick when the pad is not touched
  uint8_t battery = 0;        // percent
  bool charging = false;
  bool has_battery = false;
  uint8_t fingers[4] = {};    // index, middle, ring, pinky: 0 open .. 255 around the handle
  uint8_t squeeze_force = 0, trackpad_force = 0;
  bool has_fingers = false;
};

// One message's IMU sample, already in the controller's frame (+x right, +y up, +z back); `ticks` from the message.
struct ControllerImu {
  uint32_t ticks = 0;
  float accel[3] = {}, gyro[3] = {};
};

// Decodes the messages of a report into `state`, and its IMU samples into `imu` (at most four, two a message);
// returns how many IMU samples there were. `gen2` for an Index controller's receiver.
size_t decodeControllerReport(const uint8_t *report, size_t size, bool gen2, const ControllerConfig &config,
                              const ImuRanges &ranges, ControllerState &state, ControllerImu imu[4]);

// the haptic pulse command for a controller, sent as a feature report to its receiver
std::array<uint8_t, 10> hapticReport(float duration_s, float frequency_hz, float amplitude);

} // namespace metalvr::lighthouse
