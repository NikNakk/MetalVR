// The PlayStation VR2's USB side and its Sense controllers' reports (docs/HEADSETS.md, phase J).
//
// Nothing here touches a device: reports are taken in and made as bytes, so that each can be checked without a
// headset (tests/headset_protocol_check.cpp). The layouts and the axis conventions follow Monado's drivers
// (src/xrt/drivers/psvr2 and pssense; Copyright 2023 Jan Schmidt, 2023 Collabora, 2023 Jarett Millard, 2024 Joel
// Valenciano, 2025-2026 Beyley Cardellio; BSL-1.0).
//
// The headset is a USB device of vendor-specific interfaces, not HID: its status and IMU arrive on an interrupt
// endpoint, and the headset tracks itself (inside-out, on its own processor) and sends its pose on a bulk endpoint.
// It is controlled with vendor requests on the default pipe. The Sense controllers pair with the Mac over Bluetooth
// as HID devices.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "metalvr_protocol.h"

namespace metalvr::psvr2 {

constexpr uint16_t kVendor = 0x054c; // Sony
constexpr uint16_t kProduct = 0x0cde;
constexpr uint32_t kDisplayVendor = 0x4dd9; // "SNY" in the EDID

// interfaces and their IN endpoints (Monado's psvr2_protocol.h)
constexpr int kSlamInterface = 3, kSlamEndpoint = 0x83;
constexpr int kGazeInterface = 5, kGazeEndpoint = 0x85;
constexpr int kCameraInterface = 6, kCameraEndpoint = 0x87;
constexpr int kStatusInterface = 7, kStatusEndpoint = 0x88, kStatusAltSetting = 1;
constexpr int kLedDetectorInterface = 8, kLedDetectorEndpoint = 0x89;
constexpr int kRelocalizerInterface = 9, kRelocalizerEndpoint = 0x8a;
constexpr int kVdInterface = 10, kVdEndpoint = 0x8b;
constexpr size_t kSlamTransferSize = 1024, kStatusTransferSize = 1024, kCameraTransferSize = 1040640,
                 kLedDetectorTransferSize = 36944, kRelocalizerTransferSize = 821120, kVdTransferSize = 32768;

constexpr float kImuHz = 2000.0f;

// ---- the headset's status and IMU (interrupt endpoint 8) ----

struct ImuSample {
  uint32_t vts_us = 0; // the headset's clock
  float accel[3] = {}; // m/s^2, in the headset's frame (OpenXR's: +x right, +y up, +z back)
  float gyro[3] = {};  // rad/s
};

struct Status {
  uint8_t display = 0;    // the DisplayPort receiver: 0 not ready
  bool worn = false;      // the proximity sensor
  bool function = false;  // the button under the headset
  uint8_t ipd_mm = 0;     // the IPD dial, 59..72
  std::vector<ImuSample> imu;
};
std::optional<Status> parseStatus(const uint8_t *report, size_t size);

// ---- the headset's own tracking (bulk endpoint 3) ----

struct SlamPose {
  uint32_t vts_us = 0;
  mvr_pose pose{{0, 0, 0, 1}, {0, 0, 0}}; // the headset's tracker, in its room (OpenXR's axes)
};
// A 512-byte "SLA" record, its axes turned as Monado turns them. `previous` keeps the quaternion on the same side
// as the last one: the headset's tracker does not, and a flip would read as a full turn.
std::optional<SlamPose> parseSlam(const uint8_t *record, size_t size, const mvr_quat &previous);
// From the tracker's pose to the head's: Monado's SLAM correction (a quarter turn about z) and the tracker's place
// in the head (T_imu_head)
mvr_pose headFromSlam(const mvr_pose &slam);

// ---- vendor requests (Monado's send_psvr2_control and get_psvr2_control) ----

struct ControlRequest {
  uint8_t request_type = 0; // vendor, to an endpoint; 0x80 set for a read
  uint8_t request = 0;      // 9 to send, 1 to read
  uint16_t value = 0;       // the report id
  uint16_t index = 0;
  std::vector<uint8_t> data; // {report id u16, subcommand u16, length u32, payload}
};
ControlRequest sendRequest(uint16_t report_id, uint16_t subcommand, const uint8_t *payload, uint32_t length);
ControlRequest readRequest(uint16_t report_id, uint16_t subcommand, uint32_t length);
// what a read returned: the payload after the 8-byte header
std::vector<uint8_t> readPayload(const uint8_t *reply, size_t size, uint32_t length);

enum CameraMode : uint32_t { kCameraOff = 0, kCameraTracking = 0x0a }; // 0x0a is the mode Monado runs
ControlRequest cameraModeRequest(uint32_t mode);
ControlRequest brightnessRequest(float brightness); // 0..1
ControlRequest firmwareInfoRequest();
ControlRequest lensCalibrationRequest();

struct FirmwareInfo {
  uint32_t version = 0;
  std::string pcb_id; // the headset's serial number
};
std::optional<FirmwareInfo> parseFirmwareInfo(const uint8_t *payload, size_t size);

// ---- the lenses ----

// The eye's lens offsets and tilt from the headset's calibration (report 0x8f), as Monado's
// psvr2_setup_distortion_and_fovs works them out: x and y offsets, then cos and sin of the tilt, left then right.
// Before calibration version 4, or without one, Monado's default.
std::array<float, 8> lensCalibration(const uint8_t *payload, size_t size);
std::array<float, 8> defaultLensCalibration();
// Monado's psvr2_compute_distortion_asymmetric: where each colour of the eye's panel point (u, v) (0..1 across and
// down) is in the rendered image
void computeDistortion(const std::array<float, 8> &calibration, int eye, float u, float v, float out[3][2]);
// each eye's field of view, OpenXR's signs, radians (Monado's)
mvr_fov fieldOfView(int eye);

// ---- the Sense controllers ----

constexpr uint16_t kSenseLeft = 0x0e45, kSenseRight = 0x0e46;
constexpr uint8_t kSenseBluetoothReport = 0x31, kSenseUsbReport = 0x01, kSenseCalibrationReport = 0x05;
constexpr size_t kSenseBluetoothLength = 78, kSenseUsbLength = 64, kSenseFeatureLength = 64;

uint32_t crc32(uint32_t crc, const uint8_t *data, size_t size); // the controllers' CRC-32 (reflected, 0xedb88320)

struct SenseState {
  uint32_t buttons = 0; // mvr_button: A/X (cross, square), B/Y (circle, triangle), menu, system, stick, squeeze, trigger
  uint32_t touches = 0;
  float trigger = 0, squeeze = 0;
  float stick_x = 0, stick_y = 0; // -1..1, +y up
  int16_t gyro[3] = {}, accel[3] = {}; // raw
  uint32_t imu_ticks = 0;              // thirds of a microsecond
  float battery = 0;                   // 0..1
  bool charging = false, battery_valid = false;
};
// An input report, Bluetooth (0x31, its CRC checked) or USB (0x01), of the `left` or right controller
std::optional<SenseState> parseSenseReport(const uint8_t *report, size_t size, bool left);

struct SenseCalibration {
  float gyro_bias[3] = {}, gyro_scale[3] = {}; // (raw - bias) * scale = rad/s
  float accel_bias[3] = {}, accel_scale[3] = {}; // (raw - bias) * scale = m/s^2
};
// Feature report 5, read twice: its two parts (0x00 and 0x81) in either order. Reading it switches the controller
// from its compatibility mode into the one with these reports.
std::optional<SenseCalibration> parseSenseCalibration(const uint8_t part_a[64], const uint8_t part_b[64]);
void senseImu(const SenseState &state, const SenseCalibration &calibration, float accel[3], float gyro[3]);

// The Bluetooth output report (78 bytes) with a vibration of `amplitude` (0..1, 0 stops it), its tracking LEDs off
// (MetalVR does not track them), stamped with `sequence` (0..15) and `counter`, its CRC filled in
std::array<uint8_t, 78> senseOutputReport(float amplitude, uint8_t sequence, uint8_t counter, uint32_t host_time_us);

// the grip and aim poses in the controller's own frame (Monado's T_steamvrroot_* offsets, for each hand)
mvr_pose senseGrip(int hand);
mvr_pose senseAim(int hand);

} // namespace metalvr::psvr2
