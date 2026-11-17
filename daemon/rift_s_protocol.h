// The Oculus Rift S's USB side (docs/HEADSETS.md, phase I): its three HID interfaces (6 the headset and its IMU,
// 7 its status, 8 the Touch controllers' radio), the reports on them and the commands that turn it on.
//
// Nothing here touches a device: the reports are taken in and made as bytes, so that each can be checked without a
// headset (tests/headset_protocol_check.cpp). The layouts follow OpenHMD's Rift S driver (external/OpenHMD-rift/
// src/drv_oculus_rift_s; Copyright 2019 Lucas Teske, 2019-2020 Jan Schmidt; BSL-1.0) and Monado's
// (src/xrt/drivers/rift_s; Copyright 2020-2022 Jan Schmidt; BSL-1.0).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace metalvr::rift_s {

constexpr uint16_t kVendor = 0x2833;
constexpr uint16_t kProduct = 0x0051;
constexpr uint32_t kDisplayVendor = 0x3ed2; // "OVR"
constexpr uint32_t kDisplayProduct = 0x0012;
constexpr int kInterfaceHmd = 6, kInterfaceStatus = 7, kInterfaceControllers = 8;

constexpr uint8_t kReportDeviceInfo = 0x06;
constexpr uint8_t kReportImuConfig = 0x09;
constexpr uint8_t kReportDevices = 0x0c;
constexpr uint8_t kReportFirmwareBlock = 0x4a;
constexpr uint8_t kReportRadioCommand = 0x12;
constexpr uint8_t kReportRadioResponse = 0x0b;
constexpr uint8_t kInHmd = 0x65, kInStatus = 0x66, kInController = 0x67;
constexpr uint8_t kFirmwareImuCalibration = 0x0e;

// ---- the headset ----

struct DeviceInfo {
  uint16_t width = 1440, height = 2560; // the panel as the computer sees it: portrait
  uint8_t refresh_hz = 80;
};
std::optional<DeviceInfo> parseDeviceInfo(const uint8_t *report, size_t size); // report 6, its id first

struct ImuConfig {
  uint32_t hz = 1000;
  float gyro_scale = 16.4f;    // raw / scale = degrees/s
  float accel_scale = 2048.0f; // raw * g / scale = m/s^2
};
std::optional<ImuConfig> parseImuConfig(const uint8_t *report, size_t size); // report 9, its id first

// The IMU's calibration in firmware block 0x0e (JSON): offsets and rectifications, and where the IMU sits in the
// headset (its rotation into the headset's frame, which is OpenXR's: +x right, +y up, +z back).
struct ImuCalibration {
  float device_from_imu[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // row-major rotation
  float gyro_rectification[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float gyro_offset[3] = {};
  float accel_rectification[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float accel_offset[3] = {};
  float imu_position[3] = {}; // where the IMU sits in the headset, meters
};
std::optional<ImuCalibration> parseImuCalibration(std::string_view json);

struct ImuSample {
  float accel[3] = {}; // m/s^2, in the headset's frame
  float gyro[3] = {};  // rad/s
};

struct HmdReport {
  uint32_t timestamp_us = 0; // the first sample's
  ImuSample samples[3];
  int count = 0; // the valid ones, in order
};
// IN report 0x65: three IMU samples of the headset, rectified and turned into its frame
std::optional<HmdReport> parseHmdReport(const uint8_t *report, size_t size, const ImuConfig &config,
                                        const ImuCalibration &calibration);

// IN report 0x66: whether the proximity sensor sees a face
std::optional<bool> parseStatus(const uint8_t *report, size_t size);

// what turns the headset on and keeps it on (OpenHMD's rift_s_hmd_enable), as feature reports, in order
std::vector<std::vector<uint8_t>> enableReports(bool on);
std::vector<uint8_t> keepAliveReport(); // every second: the headset stops after 3 without one
std::vector<uint8_t> screenReport(bool on);
constexpr int kKeepAliveMs = 1000;

// a request for part of a firmware block (report 0x4a); reading block `id` starts with its 12-byte header
std::array<uint8_t, 64> firmwareRequest(uint8_t block, uint32_t offset, uint8_t length);
// whether a 0x4a reply answers `request`, and its payload (bytes 8 on)
bool firmwareReplyMatches(const uint8_t *reply, size_t size, const std::array<uint8_t, 64> &request);
// the block's length from its header reply (bytes 16..19), 0 if invalid
uint32_t firmwareBlockLength(const uint8_t *header_reply, size_t size);

// ---- the Touch controllers, through interface 8 ----

constexpr uint32_t kLeftController = 0x13001101, kRightController = 0x13011101;

// report 0x0c: the radio's devices, each an id and a type
struct Device {
  uint64_t id = 0;
  uint32_t type = 0;
};
std::vector<Device> parseDevices(const uint8_t *report, size_t size);

struct ControllerState {
  uint8_t buttons = 0; // 1 A/X, 2 B/Y, 4 stick, 8 menu/Oculus
  uint8_t fingers = 0; // capacitive touches, strong and weak
  uint16_t trigger = 0, grip = 0; // raw: 4095 released .. 0 pressed
  int16_t stick_x = 0, stick_y = 0;
  bool has_imu = false;
  uint32_t imu_timestamp = 0;
  int16_t accel[3] = {}, gyro[3] = {}; // raw; scaled by the controller's config, read over the radio
};
// IN report 0x67: one controller's blocks, into `state`; returns its device id, or 0 for an empty report
uint64_t parseControllerReport(const uint8_t *report, size_t size, ControllerState &state);

// Touch values from a state: 0..1 trigger and grip, -1..1 stick, and the touches (Monado's mapping)
struct TouchValues {
  bool primary = false, secondary = false, stick = false, menu = false; // pressed
  bool primary_touch = false, secondary_touch = false, stick_touch = false, trigger_touch = false;
  float trigger = 0, grip = 0, stick_x = 0, stick_y = 0;
};
TouchValues touchValues(const ControllerState &state);

// The controller's IMU config, read over the radio: {0x32, 0x20, 0xe8, 0x03, 0...} to it as a radio command
std::array<uint8_t, 64> radioCommand(uint64_t device, const uint8_t *bytes, size_t size);
constexpr uint8_t kControllerConfigRequest[10] = {0x32, 0x20, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
struct ControllerImuConfig {
  float accel_scale = 0, gyro_scale = 0; // raw * accel_scale * g, raw * gyro_scale degrees/s
};
// the response bytes of the config command (after the radio reply's 3-byte header)
std::optional<ControllerImuConfig> parseControllerImuConfig(const uint8_t *bytes, size_t size);
// a controller's raw IMU sample in its frame, scaled; Touch's frame is its own, the grip pose 40 degrees up from it
void controllerImu(const ControllerState &state, const ControllerImuConfig &config, float accel[3], float gyro[3]);

// ---- the lenses and the panel ----
//
// No correct Rift S lens model has been published: OpenHMD's and Monado's drivers both use the CV1's panotools
// coefficients with the Rift S's panel size, marked as wrong. MetalVR starts from Monado's numbers
// (rift_s_hmd.c) and leaves them to be tuned on a headset (docs/HEADSETS.md, phase I).
//
// The panel is one portrait 1440x2560 display: the left eye's view is its top half, the right eye's its bottom half,
// each turned a quarter right (Monado's u_device_rotation_right): an eye's view is 1280 wide and 1440 high.

struct Panotools {
  float distortion_k[5] = {0.819f, -0.241f, 0.324f, 0.098f, 0.0f};
  float aberration_k[3] = {0.9952420f, 1.0f, 1.0008074f};
  float scale = 0;           // meters
  float lens_center[2] = {}; // meters, in the eye's view
  float viewport[2] = {};    // meters
};

// Monado's values for each eye: the panel's 149.76 x 93.6 mm, lenses 74 mm apart, 105 degrees across
Panotools panotools(int eye);
// each eye's field of view from those (Monado's math_compute_fovs); radians, OpenXR's signs
void fieldOfView(int eye, float &left, float &right, float &up, float &down);
// where each colour of the view's point (u, v) (0..1 across and down, as the application renders it) is in the
// rendered image (Monado's u_compute_distortion_panotools)
void computeDistortion(const Panotools &p, float u, float v, float out[3][2]);

// ---- the cameras (camera tracking, docs/HEADSETS.md phase I) ----
//
// The five cameras reach the computer as one UVC video device, "Rift S Sensor", whose frames are the five pictures
// side by side, 8-bit grey, though the device calls them YUYV at half the width (Monado's rift_s_camera.c). Every
// other frame is exposed for tracking the room, the others for the controllers' LEDs; a row of 8x8 blocks at the top
// of each frame says which, with its time on the headset's clock and each camera's exposure.

constexpr int kCameraCount = 5;
enum CameraId { kCameraTop = 0, kCameraSideLeft = 1, kCameraFrontRight = 2, kCameraFrontLeft = 3, kCameraSideRight = 4 };
// the order the tracker is given them in (Monado's CAM_IDX_TO_ID): the front pair first, as its stereo pair
constexpr CameraId kTrackerCameraOrder[kCameraCount] = {kCameraFrontLeft, kCameraFrontRight, kCameraSideLeft,
                                                        kCameraSideRight, kCameraTop};
constexpr uint8_t kFirmwareCameraCalibration = 0x0f;
constexpr uint32_t kCameraFrameWidth = 50 * 8 * 8; // the frame's width as 8-bit grey: five 640-wide pictures

// one camera's calibration from firmware block 0x0f (JSON)
struct CameraCalibration {
  uint32_t width = 640, height = 480;
  uint32_t x_offset = 0;                  // where its picture starts in the frame
  double device_from_camera[16] = {};     // row-major isometry, in the headset's frame (meters)
  double f = 0, cx = 0, cy = 0;           // PinholeSymmetric, pixels
  double k[6] = {}, p1 = 0, p2 = 0;       // Fisheye62
};
std::optional<std::array<CameraCalibration, kCameraCount>> parseCameraCalibration(std::string_view json);

// The tracker takes a Kannala-Brandt lens (KB4) and the headset keeps Fisheye62: fitted as Monado's
// rift_s_get_cam_calib does, over a 21x21 grid of the picture.
void fisheye62Project(const CameraCalibration &c, const double point[2], double pixel[2]);
// the point on the plane z = 1 that Fisheye62 puts at `pixel`; false if it does not converge
bool fisheye62Unproject(const CameraCalibration &c, const double pixel[2], double point[2]);
std::array<double, 4> kb4FromFisheye62(const CameraCalibration &c);
void kb4Project(const CameraCalibration &c, const std::array<double, 4> &kb4, const double point[2], double pixel[2]);

// the 8x8-block row at the top of a frame (8-bit grey, kCameraFrameWidth wide)
struct CameraFrameInfo {
  bool tracking = false; // exposed for the room (else for the controllers)
  uint16_t counter = 0;
  uint64_t time_us = 0; // the headset's clock, as its IMU's
  uint16_t exposure[kCameraCount] = {};
  uint8_t gain[kCameraCount] = {};
};
std::optional<CameraFrameInfo> parseCameraFrameInfo(const uint8_t *grey, uint32_t width, uint32_t height);
// how far down the frame a camera's picture starts, from its exposure (Monado's get_y_offset)
uint32_t cameraYOffset(uint16_t exposure);
// report 5: the cameras on or off, with every camera's exposure (38..14022) and gain (16..255); `radio_sync` as
// Monado sends it while it reads the cameras
std::vector<uint8_t> cameraReport(bool on, uint16_t exposure = 0x36b3, uint8_t gain = 0xf0, bool radio_sync = false);
constexpr uint16_t kMinExposure = 38, kMaxExposure = 14022;
constexpr uint8_t kMinGain = 16;

// a tracking frame's five pictures, each `width * height` bytes packed, in kTrackerCameraOrder; false if the frame
// is too small for where the calibration and exposures put them (Monado's rift_s_camera_extract_frame)
bool splitTrackingFrame(const uint8_t *grey, uint32_t width, uint32_t height, size_t stride,
                        const CameraFrameInfo &info, const std::array<CameraCalibration, kCameraCount> &calibration,
                        std::array<std::vector<uint8_t>, kCameraCount> &pictures);

} // namespace metalvr::rift_s
