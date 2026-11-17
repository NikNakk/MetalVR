// Checks, without a headset, what MetalVR reads from and sends to the headsets it drives directly (docs/HEADSETS.md):
// the Lighthouse headsets' and controllers' reports and configuration, the Rift S's reports, the JSON reader both
// use, and the table of headsets the server describes.
//
//   headset_protocol_check        prints each check; exits 1 if any fails
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "foveation.h"
#include "headsets.h"
#include "json_value.h"
#include "lighthouse_protocol.h"
#include "psvr2_protocol.h"
#include "rift_s_protocol.h"

using namespace metalvr;

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool ok, const char *what) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("FAIL: %s\n", what);
  }
}

bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }
uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24); }

void put16(std::vector<uint8_t> &v, size_t at, int value) {
  v[at] = static_cast<uint8_t>(value & 0xff);
  v[at + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}
void put32(std::vector<uint8_t> &v, size_t at, uint32_t value) {
  for (int i = 0; i < 4; i++) v[at + i] = static_cast<uint8_t>(value >> (8 * i));
}
void putf(std::vector<uint8_t> &v, size_t at, float f) {
  uint32_t bits;
  std::memcpy(&bits, &f, 4);
  put32(v, at, bits);
}

// ---- JSON ----

void json() {
  const auto doc = JsonValue::parse(R"( {"a": [1, 2.5, -3e2], "b": {"c": "x\"yé"}, "d": true, "e": null} )");
  check(doc && doc->isObject(), "json: an object parses");
  if (!doc) return;
  float a[3];
  check((*doc)["a"].numbers(a, 3) && a[0] == 1 && a[1] == 2.5f && a[2] == -300.0f, "json: numbers");
  check((*doc)["b"]["c"].string() == "x\"y\xc3\xa9", "json: escapes");
  check((*doc)["d"].boolean() && (*doc)["e"].type() == JsonValue::Type::Null, "json: true and null");
  check((*doc)["missing"]["deeper"].type() == JsonValue::Type::Null, "json: a missing member is null");
  check(!JsonValue::parse("{\"a\": 1,}"), "json: a trailing comma is refused");
  check(!JsonValue::parse("{\"a\": 1} x"), "json: text after the document is refused");
  check(!JsonValue::parse("[1, 2"), "json: an unclosed array is refused");
  std::string deep(200, '[');
  check(!JsonValue::parse(deep), "json: nesting is bounded");
  check(JsonValue::parse(std::string("{}\0\0", 4)).has_value(), "json: a firmware block's trailing NULs are allowed");
}

// ---- Lighthouse ----

// a headset configuration as the Index's firmware has it, cut down to what MetalVR reads
const char *kIndexConfig = R"({
  "model_number": "Index",
  "device_serial_number": "LHR-TEST0001",
  "imu": {"acc_bias": [0.1, -0.2, 0.3], "acc_scale": [1.0, 1.01, 0.99], "gyro_bias": [0.001, 0.002, -0.003],
          "gyro_scale": [1, 1, 1], "plus_x": [1,0,0], "plus_z": [0,0,1], "position": [0,0,0]},
  "device": {"eye_target_width_in_pixels": 1440, "eye_target_height_in_pixels": 1600,
             "physical_aspect_x_over_y": 0.9},
  "tracking_to_eye_transform": [
    {"grow_for_undistort": 0.6, "undistort_r2_cutoff": 1.1,
     "intrinsics": [[0.92, 0, -0.08], [0, 1.02, 0.0], [0, 0, -1]],
     "distortion": {"center_x": -0.04, "center_y": 0.01, "coeffs": [0.15, 0.1, 0.02, 0]},
     "distortion_red": {"center_x": -0.04, "center_y": 0.01, "coeffs": [0.14, 0.1, 0.02, 0]},
     "distortion_blue": {"center_x": -0.04, "center_y": 0.01, "coeffs": [0.17, 0.1, 0.02, 0]}},
    {"grow_for_undistort": 0.6, "undistort_r2_cutoff": 1.1,
     "intrinsics": [[0.92, 0, 0.08], [0, 1.02, 0.0], [0, 0, -1]],
     "distortion": {"center_x": 0.04, "center_y": 0.01, "coeffs": [0.15, 0.1, 0.02, 0]},
     "distortion_red": {"center_x": 0.04, "center_y": 0.01, "coeffs": [0.14, 0.1, 0.02, 0]},
     "distortion_blue": {"center_x": 0.04, "center_y": 0.01, "coeffs": [0.17, 0.1, 0.02, 0]}}
  ]
})";

std::vector<uint8_t> deflate(const std::string &text) {
  uLongf size = compressBound(text.size());
  std::vector<uint8_t> out(size);
  compress2(out.data(), &size, reinterpret_cast<const Bytef *>(text.data()), text.size(), 9);
  out.resize(size);
  return out;
}

void lighthouseIdentify() {
  using namespace lighthouse;
  const UsbId vive_board{kHtcVendor, kViveMainboard}, vive_imu{kValveVendor, kViveSensors};
  const UsbId pro_board{kHtcVendor, kViveProMainboard}, pro_board_b{kHtcVendor, kViveProMainboardB};
  const UsbId pro2_board{kHtcVendor, kVivePro2Mainboard}, lhr{kValveVendor, kLhrSensors};
  const UsbId rift{0x2833, 0x0031}, rift_s{0x2833, 0x0051}, dongle{kValveVendor, kWatchmanGen2};
  check(identify({vive_board, vive_imu}) == Model::Vive, "identify: an HTC Vive");
  check(identify({vive_board}) == Model::None, "identify: a Vive mainboard without its IMU device is nothing");
  check(identify({pro_board, lhr}) == Model::VivePro, "identify: a Vive Pro (0309)");
  check(identify({pro_board_b, lhr}) == Model::VivePro, "identify: a Vive Pro (030e)");
  check(identify({pro2_board, lhr, dongle}) == Model::VivePro2, "identify: a Vive Pro 2");
  check(identify({lhr, dongle, dongle}) == Model::Index, "identify: 2300 alone is an Index, to be confirmed");
  check(identify({rift}) == Model::None && identify({rift_s}) == Model::None, "identify: a Rift is never a Lighthouse");
  check(identify({}) == Model::None, "identify: nothing is nothing");
  check(modelFromConfig("Index") == Model::Index && modelFromConfig("Utah MP") == Model::Index, "config: Index");
  check(modelFromConfig("Vive MV") == Model::Vive && modelFromConfig("Vive_Pro MV") == Model::VivePro &&
            modelFromConfig("VIVE_Pro 2 MV") == Model::VivePro2,
        "config: the Vive models");
  check(modelFromConfig("Knuckles Right") == Model::None, "config: a controller is no headset");
  check(headsetOf(Model::Index) == MVR_HEADSET_VALVE_INDEX && headsetOf(Model::None) == MVR_HEADSET_UNKNOWN,
        "headsetOf");
}

void lighthouseConfig() {
  using namespace lighthouse;
  // the configuration arrives as 0x11 reports of up to 62 bytes, the last one empty
  const std::vector<uint8_t> compressed = deflate(kIndexConfig);
  ConfigAssembler assembler;
  bool ok = true;
  for (size_t at = 0; at < compressed.size(); at += 62) {
    const size_t n = std::min<size_t>(62, compressed.size() - at);
    uint8_t report[64] = {kConfigReadReport, static_cast<uint8_t>(n)};
    std::memcpy(report + 2, compressed.data() + at, n);
    ok = ok && assembler.add(report, sizeof(report));
  }
  check(ok && !assembler.done(), "config: the chunks are taken");
  const uint8_t last[64] = {kConfigReadReport, 0};
  check(assembler.add(last, sizeof(last)) && assembler.done(), "config: an empty report ends it");
  const auto json = assembler.json();
  check(json && *json == kIndexConfig, "config: it inflates to the JSON that was deflated");
  const uint8_t bad[64] = {kConfigReadReport, 63};
  ConfigAssembler other;
  check(!other.add(bad, sizeof(bad)), "config: a chunk longer than 62 bytes is refused");

  const auto config = parseHeadsetConfig(kIndexConfig);
  check(config.has_value(), "config: an Index configuration parses");
  if (!config) return;
  check(config->model == Model::Index && config->serial == "LHR-TEST0001", "config: model and serial");
  check(config->eye_width == 1440 && config->eye_height == 1600, "config: the panel's size");
  check(near(config->acc_bias.y, -0.2f) && near(config->acc_scale.y, 1.01f), "config: IMU calibration");
  check(near(config->distortion[0].coefficients[2][0], 0.17f) && near(config->distortion[1].center[1][0], 0.04f),
        "config: distortion coefficients and centers, per colour");
  // Monado's _calculate_fov with these intrinsics
  const float scale = 1.6f;
  const float fx = 0.92f / scale * 720.0f, fy = 1.02f / scale * 800.0f;
  const float cx = (-0.08f / scale - 1.0f) * -720.0f, cy = (0.0f / scale + 1.0f) * 800.0f;
  check(near(config->fov[0].left, -std::atan(cx / fx)) && near(config->fov[0].right, std::atan((1440 - cx) / fx)) &&
            near(config->fov[0].up, std::atan(cy / fy)) && near(config->fov[0].down, -std::atan((1600 - cy) / fy)),
        "config: the field of view from the intrinsics");
  check(config->fov[0].left < 0 && config->fov[0].right > 0 && near(config->fov[0].left, -config->fov[1].right),
        "config: the eyes' fields of view mirror each other");
  check(!parseHeadsetConfig(R"({"model_number": "Knuckles Left"})"), "config: a controller's is no headset's");
  check(!parseHeadsetConfig(R"({"model_number": "Index"})"), "config: a headset without lenses is refused");

  // the lens: a point on the lens axis stays there, and further out it maps further out (barrel)
  float out[3][2];
  Distortion flat;
  computeDistortion(flat, 0.5f, 0.5f, false, out);
  check(near(out[1][0], 0.5f) && near(out[1][1], 0.5f), "distortion: the center maps to the center");
  computeDistortion(config->distortion[1], 0.5f, 0.5f, false, out);
  float edge[3][2];
  computeDistortion(config->distortion[1], 0.95f, 0.5f, false, edge);
  check(edge[1][0] > out[1][0], "distortion: further out on the panel is further out in the image");
  check(edge[0][0] > edge[2][0], "distortion: red and blue are bent apart (blue more)");
  float flipped[3][2];
  computeDistortion(config->distortion[1], 0.3f, 0.2f, true, flipped);
  computeDistortion(config->distortion[1], 0.3f, 0.2f, false, out);
  check(near(flipped[1][1], 1.0f - out[1][1]), "distortion: the Vive Pro 2's image is upside down");
}

void lighthouseImu() {
  using namespace lighthouse;
  check(!parseImuRanges((const uint8_t[]){0x01, 0, 0}, 3), "ranges: zero codes are refused");
  const auto ranges = parseImuRanges((const uint8_t[]){0x01, 1, 1}, 3);
  check(ranges && near(ranges->gyro, 500.0f * 3.14159265f / 180.0f) && near(ranges->accel, 4 * 9.80665f, 1e-3f),
        "ranges: 500 degrees/s and 4 g");
  HeadsetConfig config;
  config.model = Model::Index;
  // a report whose samples have sequence numbers 11, 12, 10 (round robin); x on the accelerometer only
  std::vector<uint8_t> report(kImuReportSize, 0);
  report[0] = kImuReport;
  const uint8_t seq[3] = {11, 12, 10};
  for (int i = 0; i < 3; i++) {
    const size_t at = 1 + 17 * i;
    put16(report, at, 8192);          // accel x: a quarter of full scale
    put16(report, at + 6 + 4, -16384); // gyro z: minus half
    put32(report, at + 12, 1000 * (seq[i] - 9));
    report[at + 16] = seq[i];
  }
  ImuRanges r;
  ImuSample out[3];
  uint8_t sequence = 0;
  bool have = false;
  size_t n = parseImuReport(report.data(), report.size(), config, r, sequence, have, out);
  check(n == 3 && sequence == 12 && out[0].ticks == 1000 && out[2].ticks == 3000,
        "imu: all three samples, oldest first");
  // the Index's IMU: x' = -y, y' = -x, z' = -z
  check(near(out[0].accel[1], -0.25f * r.accel, 1e-3f) && near(out[0].accel[0], 0) &&
            near(out[0].gyro[2], 0.5f * r.gyro, 1e-4f),
        "imu: the Index's axes into OpenXR's");
  report[1 + 16] = 13; // the next report: one new sample
  n = parseImuReport(report.data(), report.size(), config, r, sequence, have, out);
  check(n == 1 && sequence == 13, "imu: samples already seen are skipped");
  config.model = Model::Vive;
  have = false;
  n = parseImuReport(report.data(), report.size(), config, r, sequence, have, out);
  check(n == 3 && near(out[0].accel[0], 0.25f * r.accel, 1e-3f) && near(out[0].gyro[2], 0.5f * r.gyro, 1e-4f),
        "imu: the Vive's axes into OpenXR's");
  check(parseImuReport(report.data(), 40, config, r, sequence, have, out) == 0, "imu: a short report is refused");
}

void lighthouseCommands() {
  using namespace lighthouse;
  const auto on = powerReport(true);
  const uint8_t expected[13] = {0x04, 0x78, 0x29, 0x38, 0x01, 0, 0, 0, 0, 0, 0x02, 0x00, 0x01};
  check(std::memcmp(on.data(), expected, sizeof(expected)) == 0 && on[45] == 0x7a,
        "power: the power-on report libsurvive and Monado send");
  check(powerReport(false)[4] == 0 && powerReport(false)[45] == 0x7c, "power: off");
  const auto mode = pro2Command("dtd,2");
  check(mode[0] == 0x04 && mode[1] == 0x70 && mode[2] == 0x29 && mode[3] == 5 && std::memcmp(mode.data() + 4, "dtd,2", 5) == 0,
        "pro 2: a mode command");
  check(pro2Modes()[2].width == 3264 && !pro2Modes()[2].needs_dsc && pro2Modes()[4].needs_dsc,
        "pro 2: its modes, the 4896-wide ones with DSC");
  std::vector<uint8_t> status(64, 0);
  status[0] = kMainboardStatusReport;
  put16(status, 4, 6350);  // lens separation 63.50 mm
  put16(status, 14, 210);  // proximity
  put16(status, 16, 6420); // IPD 64.20 mm
  const auto s = parseMainboardStatus(status.data(), status.size());
  check(s && near(s->ipd, 0.0642f, 1e-6f) && near(s->lens_separation, 0.0635f, 1e-6f) && s->proximity == 210,
        "mainboard: IPD, lens separation, proximity");
  const auto pulse = hapticReport(0.1f, 200.0f, 0.5f);
  // 1e6 / 200 = 5000 us a period; half amplitude: low 1250, high 3750; 20 repeats
  check(pulse[0] == 0xff && pulse[1] == 0x8f && pulse[2] == 7 && (pulse[4] | pulse[5] << 8) == 3750 &&
            (pulse[6] | pulse[7] << 8) == 1250 && (pulse[8] | pulse[9] << 8) == 20,
        "haptic: a 200 Hz pulse of half amplitude for 0.1 s");
  check((hapticReport(0.1f, 200.0f, 0.0f)[6]) == 1, "haptic: amplitude 0 is the weakest pulse, not none");
}

void lighthouseControllers() {
  using namespace lighthouse;
  const auto wand = parseControllerConfig(R"({"model_number": "Vive. Controller MV", "acc_scale": [1,1,1]})");
  check(wand && wand->kind == ControllerKind::Wand, "controller config: a Vive wand");
  const auto left = parseControllerConfig(R"({"model_number": "Knuckles EV3.0 Left", "imu": {}})");
  check(left && left->kind == ControllerKind::IndexLeft, "controller config: a left Index controller");
  ImuRanges ranges;

  // a wand: flags 1111 0111 (trigger, trackpad, buttons), buttons, trigger, trackpad x and y
  {
    uint8_t report[30] = {kControllerReport1, 0x12, 9, 0x34, 0xF7, 0x05, 0x80, 0x00, 0x40, 0x00, 0xC0};
    ControllerState state;
    ControllerImu imu[4];
    const size_t n = decodeControllerReport(report, sizeof(report), false, *wand, ranges, state, imu);
    check(n == 0 && state.buttons == 0x05 && near(state.trigger, 128 / 255.0f) && near(state.pad_x, 0.5f, 1e-3f) &&
              near(state.pad_y, -0.5f, 1e-3f),
          "watchman 1: buttons, trigger, trackpad");
  }
  // a wand's battery then its IMU: 1110 1001 = battery and IMU
  {
    uint8_t report[30] = {kControllerReport1, 0x12, 16, 0x34, 0xE9, 0x80 | 77, 0x56};
    std::vector<uint8_t> v(report, report + 30);
    put16(v, 7, 0);       // accel x
    put16(v, 9, 16384);   // accel y: half of full scale
    put16(v, 11, 0);
    std::memcpy(report, v.data(), 30);
    ControllerState state;
    ControllerImu imu[4];
    const size_t n = decodeControllerReport(report, sizeof(report), false, *wand, ranges, state, imu);
    check(state.has_battery && state.battery == 77 && state.charging, "watchman 1: battery and charging");
    // the wand's IMU: x' = -x, y' = -z, z' = -y
    check(n == 1 && near(imu[0].accel[2], -0.5f * ranges.accel, 1e-3f) && imu[0].ticks == (0x12u << 24 | 0x34u << 16 | 0x56u << 8),
          "watchman 1: an IMU sample, in the wand's frame, with its time");
  }
  // an Index controller: 0xf0 then touch and force
  {
    uint8_t report[30] = {kControllerReport1, 0x01, 11, 0x02, 0xF0, 161, 0x30, 10, 20, 30, 40, 50, 60};
    ControllerState state;
    ControllerImu imu[4];
    decodeControllerReport(report, sizeof(report), true, *left, ranges, state, imu);
    check(state.has_fingers && state.touch == 0x30 && state.fingers[0] == 40 && state.fingers[1] == 10 &&
              state.fingers[3] == 30 && state.squeeze_force == 50 && state.trackpad_force == 60,
          "watchman 2: touches, fingers and forces");
  }
  // two messages in one report
  {
    uint8_t report[59] = {kControllerReport2, 0, 3, 0, 0xF1, 0x09};
    report[30] = 0;
    report[31] = 3;
    report[33] = 0xF1;
    report[34] = 0x21;
    ControllerState state;
    ControllerImu imu[4];
    decodeControllerReport(report, sizeof(report), true, *left, ranges, state, imu);
    check(state.buttons == 0x21, "watchman 2: both messages of a report, the second last");
  }
  ControllerState state;
  ControllerImu imu[4];
  uint8_t junk[30] = {kControllerReport1, 0, 200};
  check(decodeControllerReport(junk, sizeof(junk), true, *left, ranges, state, imu) == 0,
        "watchman: a message longer than its report is ignored");
}

// ---- Rift S ----

void riftS() {
  using namespace rift_s;
  std::vector<uint8_t> info(32, 0);
  info[0] = kReportDeviceInfo;
  put16(info, 1, 2560);
  put16(info, 3, 1440);
  info[7] = 80;
  const auto device = parseDeviceInfo(info.data(), info.size());
  check(device && device->width == 1440 && device->height == 2560 && device->refresh_hz == 80,
        "rift s: the panel, portrait, at 80 Hz");

  std::vector<uint8_t> imu_config(32, 0);
  imu_config[0] = kReportImuConfig;
  put32(imu_config, 1, 1000);
  putf(imu_config, 5, 16.4f);
  putf(imu_config, 9, 2048.0f);
  const auto config = parseImuConfig(imu_config.data(), imu_config.size());
  check(config && config->hz == 1000 && near(config->gyro_scale, 16.4f) && near(config->accel_scale, 2048.0f),
        "rift s: the IMU's scales");

  const char *calibration_json = R"({"FileFormat": {"Version": "1.0"}, "ImuCalibration": {
      "DeviceFromImu": [0, -1, 0, 0.01, 1, 0, 0, 0.02, 0, 0, 1, 0.03, 0, 0, 0, 1],
      "Gyroscope": {"RectificationMatrix": [1, 0, 0, 0, 1, 0, 0, 0, 1], "Offset": {"ConstantOffset": [0.01, 0, 0]}},
      "Accelerometer": {"RectificationMatrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                        "Offset": {"OffsetAtZeroDegC": [0, 0.1, 0], "OffsetTemperatureCoefficient": [0, 0, 0]}}}})";
  const auto calibration = parseImuCalibration(calibration_json);
  check(calibration && near(calibration->device_from_imu[1], -1) && near(calibration->device_from_imu[3], 1) &&
            near(calibration->gyro_offset[0], 0.01f),
        "rift s: the IMU calibration, its rotation from the 4x4");
  check(calibration && near(calibration->imu_position[0], 0.01f) && near(calibration->imu_position[2], 0.03f),
        "rift s: the IMU calibration, where the IMU sits");
  check(!parseImuCalibration(R"({"FileFormat": {"Version": "2.0"}, "ImuCalibration": {}})"),
        "rift s: a calibration of another version is refused");
  if (!config || !calibration) return;

  std::vector<uint8_t> hmd(64, 0);
  hmd[0] = kInHmd;
  put32(hmd, 3, 123456);
  // sample 0: accel y = 1 g (2048), gyro x = 16.4 (1 degree/s); sample 2 invalid
  put16(hmd, 7 + 1 + 2, 2048);
  put16(hmd, 7 + 7, 164);
  hmd[7 + 30] = 0x80;
  const auto report = parseHmdReport(hmd.data(), hmd.size(), *config, *calibration);
  check(report && report->count == 2 && report->timestamp_us == 123456, "rift s: valid samples up to the first invalid");
  if (report) {
    // the IMU's y is the device's -x (DeviceFromImu row 0 is [0 -1 0]); accel offset 0.1 on y first
    check(near(report->samples[0].accel[0], -(9.80665f - 0.1f), 1e-3f), "rift s: accel into the headset's frame");
    const float gyro_x = 10.0f * 3.14159265f / 180.0f - 0.01f;
    check(near(report->samples[0].gyro[1], gyro_x, 1e-4f), "rift s: gyro into the headset's frame");
  }
  check(!parseHmdReport(hmd.data(), 63, *config, *calibration), "rift s: a short IMU report is refused");
  check(parseStatus((const uint8_t[]){kInStatus, 1}, 2).value_or(false), "rift s: worn");
  const auto enable = enableReports(true);
  check(enable.size() == 5 && enable[0] == std::vector<uint8_t>({0x07, 0xa3, 0x01}) && enable[2][0] == 0x0a &&
            enable[2][1] == 2 && enable[4].size() == 41 && enable[4][1] == 1,
        "rift s: the power-on sequence");
  check(keepAliveReport() == std::vector<uint8_t>({0x93, 0x01, 0xb8, 0x0b, 0x00, 0x00}), "rift s: keep-alive");

  const auto request = firmwareRequest(kFirmwareImuCalibration, 0x0c, 56);
  check(request[0] == 0x4a && request[2] == 0x0e && request[3] == 0x0c && request[7] == 56, "rift s: firmware request");
  std::vector<uint8_t> reply(64, 0);
  std::memcpy(reply.data(), request.data(), 8);
  check(firmwareReplyMatches(reply.data(), reply.size(), request), "rift s: a reply that answers the request");
  reply[3] = 0;
  check(!firmwareReplyMatches(reply.data(), reply.size(), request), "rift s: a reply for another offset");
  put32(reply, 16, 2400);
  check(firmwareBlockLength(reply.data(), reply.size()) == 2400, "rift s: a block's length");

  std::vector<uint8_t> devices(200, 0);
  devices[0] = kReportDevices;
  devices[2] = 2;
  put32(devices, 3, 0xAAAA);
  put32(devices, 3 + 8, kLeftController);
  put32(devices, 31, 0xBBBB);
  put32(devices, 31 + 8, kRightController);
  const auto list = parseDevices(devices.data(), devices.size());
  check(list.size() == 2 && list[0].type == kLeftController && list[1].id == 0xBBBB, "rift s: the radio's devices");

  // a controller report: buttons 0x05, trigger/grip, stick, IMU
  std::vector<uint8_t> c(62, 0);
  c[0] = kInController;
  put32(c, 1, 0xAAAA);
  size_t at = 14;
  const uint8_t blocks[] = {0x0c, 0x05, 0x0d, 0x11, 0x1b, 0x00, 0x08, 0x40, 0x22};
  std::memcpy(c.data() + at, blocks, sizeof(blocks));
  at += sizeof(blocks);
  put16(c, at, 16384);
  put16(c, at + 2, static_cast<uint16_t>(-16384));
  at += 4;
  c[at] = 0x91;
  put32(c, at + 1, 777);
  put16(c, at + 7, 1024);
  at += 19;
  c[9] = static_cast<uint8_t>(at - 10);
  ControllerState state;
  check(parseControllerReport(c.data(), c.size(), state) == 0xAAAA, "rift s: a controller report names its device");
  const TouchValues v = touchValues(state);
  check(v.primary && v.stick && !v.secondary && v.primary_touch, "rift s: buttons and touches");
  // trigger: 0x800 of 4096 travel left; grip: 0x400
  check(near(v.trigger, 0.5f, 1e-3f) && near(v.grip, 0.75f, 1e-3f), "rift s: trigger and grip");
  check(near(v.stick_x, 0.5f) && near(v.stick_y, -0.5f), "rift s: the stick");
  check(state.has_imu && state.imu_timestamp == 777 && state.accel[0] == 1024, "rift s: the controller's IMU block");

  std::vector<uint8_t> response(32, 0);
  response[4] = 16;
  putf(response, 5 + 8, 1.0f / 1024.0f);
  putf(response, 5 + 12, 0.1220703f);
  const auto imu = parseControllerImuConfig(response.data(), response.size());
  check(imu && near(imu->accel_scale, 1.0f / 1024.0f, 1e-7f), "rift s: the controller's IMU config");
  if (imu) {
    float accel[3], gyro[3];
    controllerImu(state, *imu, accel, gyro);
    check(near(accel[0], 9.80665f, 1e-3f), "rift s: the controller's accel, scaled");
  }
  // the lenses: each eye sees about 105 degrees across, the left eye's mirrored from the right's
  float fov[2][4];
  fieldOfView(0, fov[0][0], fov[0][1], fov[0][2], fov[0][3]);
  fieldOfView(1, fov[1][0], fov[1][1], fov[1][2], fov[1][3]);
  check(near(fov[0][1] - fov[0][0], 105.0f * 3.14159265f / 180.0f, 1e-4f) && near(fov[0][0], -fov[1][1]) &&
            fov[0][2] > 0 && fov[0][3] < 0 && near(fov[0][2], -fov[0][3]),
        "rift s: the field of view, 105 degrees across, the eyes mirrored");
  const Panotools lens = panotools(1);
  float center[3][2], outer[3][2];
  computeDistortion(lens, lens.lens_center[0] / lens.viewport[0], 0.5f, center);
  check(near(center[1][0], lens.lens_center[0] / lens.viewport[0]) && near(center[1][1], 0.5f),
        "rift s: the lens center stays put");
  computeDistortion(lens, 0.95f, 0.5f, outer);
  float mid[3][2];
  computeDistortion(lens, 0.75f, 0.5f, mid);
  check(outer[1][0] > mid[1][0] && mid[1][0] > center[1][0] && outer[2][0] > outer[0][0],
        "rift s: further out on the panel is further out in the image, blue furthest");

  const auto command = radioCommand(0xAAAA, kControllerConfigRequest, sizeof(kControllerConfigRequest));
  check(command[0] == 0x12 && command[1] == 0xAA && command[2] == 0xAA && command[9] == 0x32, "rift s: a radio command");
}

// ---- the Rift S's cameras ----

void riftSCameras() {
  using namespace rift_s;
  std::string json = R"({"FileFormat": {"Version": "1.0"}, "CameraCalibration": [)";
  for (int i = 0; i < kCameraCount; i++) {
    char camera[512];
    std::snprintf(camera, sizeof(camera),
                  R"(%s{"Id": "%d", "ImageSize": [640, 480], "DeviceFromCamera": [1, 0, 0, %d, 0, 1, 0, 0, 0, 0, 1,
                  0, 0, 0, 0, 1], "Projection": {"Model": "PinholeSymmetric", "Coefficients": [300, 320, 240]},
                  "Distortion": {"Model": "Fisheye62", "Coefficients": [-0.03, 0.012, -0.006, 0.0013, 0, 0, 0.0004,
                  -0.0003]}})",
                  i ? "," : "", i, i);
    json += camera;
  }
  json += "]}";
  const auto cameras = parseCameraCalibration(json);
  check(cameras && (*cameras)[3].x_offset == 1920 && near((*cameras)[2].device_from_camera[3], 2) &&
            near((*cameras)[0].f, 300) && near((*cameras)[0].k[0], -0.03f) && near((*cameras)[0].p2, -0.0003f),
        "rift s cameras: the calibration's five cameras, side by side in the frame");
  std::string wrong = json;
  wrong.replace(wrong.find("Fisheye62"), 9, "Fisheye64");
  check(!parseCameraCalibration(wrong), "rift s cameras: another lens model is refused");
  if (!cameras) return;

  // the KB4 lens fitted to Fisheye62 puts every point of the picture within a pixel of where it was (KB4 has no
  // tangential terms; the rest of the difference is a few hundredths)
  const CameraCalibration &c = (*cameras)[0];
  const auto kb4 = kb4FromFisheye62(c);
  double worst = 0;
  bool inverted = true;
  for (int y = 0; y < 480; y += 30)
    for (int x = 0; x < 640; x += 30) {
      const double pixel[2] = {double(x), double(y)};
      double point[2], fisheye[2], fitted[2];
      inverted = inverted && fisheye62Unproject(c, pixel, point);
      fisheye62Project(c, point, fisheye);
      kb4Project(c, kb4, point, fitted);
      worst = std::max({worst, std::hypot(fisheye[0] - pixel[0], fisheye[1] - pixel[1]),
                        std::hypot(fitted[0] - pixel[0], fitted[1] - pixel[1])});
    }
  std::printf("rift s cameras: KB4 %.5f %.5f %.5f %.5f, %.3f px at worst\n", kb4[0], kb4[1], kb4[2], kb4[3], worst);
  check(inverted && worst < 1.0, "rift s cameras: the KB4 fit matches the Fisheye62 lens within a pixel");

  // a frame: its header row, and the pictures where the exposure puts them
  std::vector<uint8_t> frame(kCameraFrameWidth * 700, 0);
  uint8_t raw[50] = {0x86, 0xcd, 0xab, 0x34, 0x12};
  const uint64_t time = 0x0000000512345678ULL;
  for (int i = 0; i < 8; i++) raw[16 + i] = static_cast<uint8_t>(time >> (8 * i));
  for (int i = 0; i < kCameraCount; i++) raw[28 + 2 * i] = static_cast<uint8_t>(1000 + 1000 * i), // exposures
                                         raw[29 + 2 * i] = static_cast<uint8_t>((1000 + 1000 * i) >> 8),
                                         raw[40 + i] = static_cast<uint8_t>(0x40 + i);
  raw[48] = 0xce, raw[49] = 0xfa;
  for (int block = 0; block < 400; block++)
    if (raw[block / 8] & (0x80 >> (block % 8)))
      for (int y = 0; y < 8; y++) std::memset(&frame[y * kCameraFrameWidth + block * 8], 200, 8);
  const auto info = parseCameraFrameInfo(frame.data(), kCameraFrameWidth, 700);
  check(info && info->tracking && info->counter == 0x1234 && info->time_us == time && info->exposure[4] == 5000 &&
            info->gain[2] == 0x42,
        "rift s cameras: a frame's header row: tracking, its counter, time, exposures and gains");
  check(!parseCameraFrameInfo(frame.data(), 1600, 700), "rift s cameras: a frame of another width is refused");
  check(cameraYOffset(6000) == 165 && cameraYOffset(0) == 8 && cameraYOffset(20000) == 375,
        "rift s cameras: the picture's offset from the exposure, within 8..375");
  // each picture marked with its camera id at its corner
  for (int id = 0; id < kCameraCount; id++)
    frame[cameraYOffset(info->exposure[id]) * kCameraFrameWidth + (*cameras)[id].x_offset] = static_cast<uint8_t>(10 + id);
  std::array<std::vector<uint8_t>, kCameraCount> pictures;
  const bool split = splitTrackingFrame(frame.data(), kCameraFrameWidth, 700, kCameraFrameWidth, *info, *cameras, pictures);
  check(split && pictures[0][0] == 10 + kCameraFrontLeft && pictures[1][0] == 10 + kCameraFrontRight &&
            pictures[4][0] == 10 + kCameraTop && pictures[2].size() == 640 * 480,
        "rift s cameras: the pictures, cut out in the tracker's order (front left, front right, sides, top)");
  check(!splitTrackingFrame(frame.data(), kCameraFrameWidth, 400, kCameraFrameWidth, *info, *cameras, pictures),
        "rift s cameras: a frame too short for its pictures is refused");

  const auto report = cameraReport(true, 0x1234, 0x56, true);
  const auto off = cameraReport(false);
  check(report.size() == 41 && report[0] == 5 && report[1] == 1 && report[2] == 1 && report[3] == 0x34 &&
            report[4] == 0x12 && report[11] == 0x34 && report[17] == 0x56 && report[18] == 0x26 && off[1] == 0 &&
            off[3] == 0xb3 && off[13] == 0xf0,
        "rift s cameras: the camera report with every camera's exposure and gain");
}

// ---- the PlayStation VR2 ----

void psvr2Headset() {
  using namespace psvr2;
  // status: worn, function button, IPD 64, then two IMU records
  std::vector<uint8_t> status(32 + 2 * 24, 0);
  status[0] = 2, status[1] = 1, status[2] = 1, status[5] = 64;
  put32(status, 32, 1000);
  put16(status, 32 + 4, 0);      // accel x (the IMU's)
  put16(status, 32 + 6, 0);      // accel y
  put16(status, 32 + 8, 8192);   // accel z: 1 g up the IMU's z, which is the headset's +y
  put16(status, 32 + 10, 16384); // gyro x: 1000 degrees/s about the IMU's x, the headset's -z
  put32(status, 56, 1500);
  const auto st = parseStatus(status.data(), status.size());
  check(st && st->worn && st->function && st->ipd_mm == 64 && st->display == 2 && st->imu.size() == 2 &&
            st->imu[1].vts_us == 1500,
        "psvr2: status: worn, button, IPD and its IMU records");
  check(st && near(st->imu[0].accel[1], 9.80665f, 0.01f) && near(st->imu[0].accel[0], 0) &&
            near(st->imu[0].gyro[2], -1000.0f * 3.14159265f / 180.0f * 16384.0f * 2000.0f / 32767.0f / 1000.0f, 0.01f),
        "psvr2: the IMU's axes turned into the headset's (Monado's), 1 g up");
  check(!parseStatus(status.data(), 20), "psvr2: a short status report is refused");

  // tracking: the headset's position (1, 2, 3) and orientation w=1
  std::vector<uint8_t> record(512, 0);
  std::memcpy(record.data(), "SLA", 3);
  put32(record, 8, 777);
  putf(record, 16, 1), putf(record, 20, 2), putf(record, 24, 3);
  putf(record, 28, -1); // w = -1: the same rotation as +1, on the far side
  const auto slam = parseSlam(record.data(), record.size(), mvr_quat{0, 0, 0, 1});
  check(slam && slam->vts_us == 777 && near(slam->pose.position.x, 3) && near(slam->pose.position.y, 2) &&
            near(slam->pose.position.z, -1) && near(slam->pose.orientation.w, 1),
        "psvr2: the headset's own pose, its axes turned, kept on the last quaternion's side");
  record[0] = 'X';
  check(!parseSlam(record.data(), record.size(), mvr_quat{0, 0, 0, 1}), "psvr2: a record without SLA is refused");
  const mvr_pose head = headFromSlam(mvr_pose{{0, 0, 0, 1}, {0, 0, 0}});
  check(near(head.orientation.z, std::sqrt(0.5f)) && near(head.orientation.w, std::sqrt(0.5f)) &&
            near(std::sqrt(head.position.x * head.position.x + head.position.y * head.position.y +
                           head.position.z * head.position.z),
                 0.104827f, 0.0005f),
        "psvr2: the head from the tracker: Monado's correction and the tracker's 10.5 cm offset");

  // requests
  const ControlRequest mode = cameraModeRequest(kCameraTracking);
  check(mode.request_type == 0x42 && mode.request == 9 && mode.value == 0x0b && mode.data.size() == 16 &&
            mode.data[0] == 0x0b && mode.data[2] == 1 && mode.data[4] == 8 && mode.data[8] == 1 && mode.data[12] == 0x0a,
        "psvr2: the camera mode request");
  const ControlRequest read = lensCalibrationRequest();
  check(read.request_type == 0xc2 && read.request == 1 && read.value == 0x8f && read.data.size() == 8 + 0x100 &&
            brightnessRequest(1.0f).data[8] == 31,
        "psvr2: a read request, and brightness");
  std::vector<uint8_t> fw(8 + 84, 0);
  put32(fw, 8, 0x06000102);
  std::memcpy(fw.data() + 8 + 56, "SERIAL123", 9);
  const auto payload = readPayload(fw.data(), fw.size(), 84);
  const auto firmware = parseFirmwareInfo(payload.data(), payload.size());
  check(firmware && firmware->version == 0x06000102 && firmware->pcb_id == "SERIAL123", "psvr2: firmware and serial");

  // lenses: an old calibration is Monado's default; a version 4 one is worked out
  std::vector<uint8_t> lens(0x100, 0);
  lens[0] = 3;
  const auto old = lensCalibration(lens.data(), lens.size());
  lens[0] = 4;
  putf(lens, 8 + 4 * 4, 10.0f); // the left eye tilted 10 degrees
  const auto cal = lensCalibration(lens.data(), lens.size());
  check(near(old[0], -0.09919293f) && near(old[2], 0.09919293f) && near(cal[4], std::cos(10 * 3.14159265f / 180)) &&
            near(cal[5], -std::sin(10 * 3.14159265f / 180)) && near(cal[1], 14.95f / 1000 / 32.46199f),
        "psvr2: the lens calibration, its offsets and tilt");
  float center[3][2], edge[3][2], mid[3][2];
  computeDistortion(old, 1, 0.5f, 0.5f, center);
  computeDistortion(old, 1, 0.95f, 0.5f, edge);
  computeDistortion(old, 1, 0.75f, 0.5f, mid);
  check(center[1][0] > 0.3f && center[1][0] < 0.4f && near(center[1][1], 0.5f, 0.01f) && edge[1][0] > mid[1][0] &&
            mid[1][0] > center[1][0] && edge[2][0] > edge[0][0],
        "psvr2: the lens: further out on the panel is further out in the image, blue furthest");
  float left_center[3][2];
  computeDistortion(old, 0, 0.5f, 0.5f, left_center);
  check(near(left_center[1][0], 1.0f - center[1][0], 0.001f) && near(fieldOfView(0).left, -fieldOfView(1).right),
        "psvr2: the eyes mirror each other");
}

void psvr2Sense() {
  using namespace psvr2;
  const uint8_t check_value[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  check(psvr2::crc32(0, check_value, 9) == 0xcbf43926u, "sense: CRC-32 (the standard check value)");

  // a Bluetooth report from the right controller: cross pressed and touched, trigger half, stick right
  std::vector<uint8_t> r(78, 0);
  r[0] = kSenseBluetoothReport;
  uint8_t *c = r.data() + 2;
  c[0] = 255, c[1] = 128, c[2] = 128;
  c[7] = 2;     // cross
  c[9] = 2 | 4; // cross touched, stick touched
  c[8] = 16;    // PS
  put16(r, 2 + 15, 100); // gyro x
  put16(r, 2 + 21, 200); // accel x
  put32(r, 2 + 27, 3000);
  c[41] = 0x15; // charging, level 5
  const uint8_t seed = 0xa1;
  uint32_t crc = psvr2::crc32(psvr2::crc32(0, &seed, 1), r.data(), 74);
  put32(r, 74, crc);
  const auto right = parseSenseReport(r.data(), r.size(), false);
  check(right && (right->buttons & MVR_BUTTON_PRIMARY) && (right->buttons & MVR_BUTTON_SYSTEM) &&
            (right->touches & MVR_BUTTON_PRIMARY) && (right->touches & MVR_BUTTON_THUMBSTICK) &&
            near(right->trigger, 128 / 255.0f) && near(right->stick_x, 127 / 128.0f) && near(right->stick_y, 0) &&
            right->gyro[0] == 100 && right->accel[0] == 200 && right->imu_ticks == 3000 && right->charging &&
            near(right->battery, 0.55f),
        "sense: a right controller's Bluetooth report: cross is A, the PS button, touches, trigger, stick, battery");
  const auto as_left = parseSenseReport(r.data(), r.size(), true);
  check(as_left && !(as_left->buttons & MVR_BUTTON_PRIMARY), "sense: the left controller's buttons are its own");
  r[10] ^= 1;
  check(!parseSenseReport(r.data(), r.size(), false), "sense: a report that fails its CRC is refused");

  // calibration: accel +-8192 about 0, gyro bias 10, +-1010 at 2000 degrees/s
  uint8_t a[64] = {kSenseCalibrationReport, 0x00}, b[64] = {kSenseCalibrationReport, 0x81};
  uint8_t d[116] = {};
  const auto put = [&](size_t at, int16_t v) { d[at] = static_cast<uint8_t>(v), d[at + 1] = static_cast<uint8_t>(v >> 8); };
  put(0x00, 8192), put(0x06, -8192), put(0x0e, 8192), put(0x14, -8192), put(0x1c, 8192), put(0x22, -8192);
  put(0x24, 10), put(0x26, 10), put(0x28, 10);
  put(0x2a, 1010), put(0x30, -990), put(0x38, 1010), put(0x3e, -990), put(0x46, 1010), put(0x4c, -990);
  put(0x4e, 2000), put(0x50, 2000);
  std::memcpy(a + 2, d, 58);
  std::memcpy(b + 2, d + 58, 58);
  const auto calibration = parseSenseCalibration(b, a); // either order
  check(calibration && near(calibration->accel_scale[0], 9.80665f / 8192) && near(calibration->gyro_bias[1], 10) &&
            near(calibration->gyro_scale[2], 2000 * 3.14159265f / 180 / 1000, 1e-4f),
        "sense: the IMU calibration from its two parts");
  if (calibration) {
    SenseState state;
    state.gyro[0] = 1010, state.accel[1] = 8192;
    float accel[3], gyro[3];
    senseImu(state, *calibration, accel, gyro);
    check(near(gyro[0], 2000 * 3.14159265f / 180, 0.01f) && near(accel[1], 9.80665f, 0.01f),
          "sense: a raw sample scaled with it");
  }

  const auto out = senseOutputReport(0.5f, 3, 9, 1234);
  const uint8_t out_seed = 0xa2;
  check(out[0] == 0x31 && out[1] == 0x30 && out[2] == 0x10 && out[3] == 0x03 && out[5] == 128 && out[22] == 5 &&
            out[41] == 9 && le32(out.data() + 74) == psvr2::crc32(psvr2::crc32(0, &out_seed, 1), out.data(), 74),
        "sense: the vibration report, LEDs off, its CRC");
  const mvr_pose grip = senseGrip(0), grip_right = senseGrip(1);
  check(near(grip.position.x, -grip_right.position.x) && std::fabs(grip.orientation.x) > 0.4f,
        "sense: grip poses mirror between the hands, tilted from the controller");
}

// ---- the headsets table ----

void headsetTable() {
  check(headsetFromAlvrName("Quest 3") == MVR_HEADSET_QUEST_3 && headsetFromAlvrName("Quest 3S") == MVR_HEADSET_QUEST_3S,
        "alvr names: Quest 3 and Quest 3S are told apart");
  check(headsetFromAlvrName("Steam Frame") == MVR_HEADSET_STEAM_FRAME, "alvr names: the patched client on a Steam Frame");
  check(headsetFromAlvrName("quest 3") == MVR_HEADSET_UNKNOWN && headsetFromAlvrName("Android (unknown)") ==
                                                                     MVR_HEADSET_UNKNOWN,
        "alvr names: anything else is unknown");
  check(headsetFromKey("index") == MVR_HEADSET_VALVE_INDEX && headsetFromKey("rift") == MVR_HEADSET_UNKNOWN,
        "keys");
  for (uint32_t h = 1; h < MVR_HEADSET_COUNT; h++) {
    const HeadsetSpec &spec = headsetSpec(h);
    char what[96];
    std::snprintf(what, sizeof(what), "spec of %s: sizes, rates and field of view", mvr_describe_headset(h)->key);
    bool ok = spec.eye_width > 0 && spec.eye_height > 0 && spec.refresh_rate_count > 0 && spec.default_refresh > 0;
    bool listed = false;
    for (uint32_t i = 0; i < spec.refresh_rate_count; i++) listed = listed || spec.refresh_rates[i] == spec.default_refresh;
    ok = ok && listed && spec.fov[0].left < 0 && spec.fov[0].right > 0 && spec.fov[0].up > 0 && spec.fov[0].down < 0;
    check(ok, what);
    check(std::string(mvr_describe_headset(h)->key) != "unknown", "each headset has a key");
  }
  check(!withinDecoder(headsetSpec(MVR_HEADSET_QUEST_1), 4288, 2368, 72.0f),
        "decoder: ALVR's default frame is too much for a Quest 1");
  check(withinDecoder(headsetSpec(MVR_HEADSET_QUEST_1), 3328, 1856, 72.0f), "decoder: a smaller one is not");
  check(withinDecoder(headsetSpec(MVR_HEADSET_QUEST_3), 4288, 2240, 90.0f), "decoder: a Quest 3 at 90 Hz");
  check(withinDecoder(headsetSpec(MVR_HEADSET_UNKNOWN), 9999, 9999, 120.0f), "decoder: no limit known, none applied");
  mvr_system_info info{};
  applyHeadsetSpec(headsetSpec(MVR_HEADSET_VIVE_PRO_2), info);
  check(info.headset == MVR_HEADSET_VIVE_PRO_2 && info.controllers == MVR_CONTROLLERS_VIVE_WAND &&
            std::string(info.system_name) == "HTC Vive Pro 2" && info.refresh_rate_count == 2,
        "applyHeadsetSpec: identity and rates");
}

// ---- foveated encoding: the server's squeeze and ALVR's client's stretch undo each other ----

void foveation() {
  // ALVR's default shape, for a Quest 2-sized eye
  const Foveation f = computeFoveation(2144, 2336, 0.45f, 0.4f, 0.4f, 0.1f, 4.0f, 5.0f);
  check(f.enabled && f.optimized_eye_width % 32 == 0 && f.optimized_eye_height % 32 == 0 &&
            f.optimized_eye_width < 2144 && f.optimized_eye_height < 2336,
        "foveation: a smaller frame, in multiples of 32");
  check(f.eye_size_ratio[0] > 0.9f && f.eye_size_ratio[0] <= 1.0f, "foveation: the squeezed eye nearly fills it");
  float worst = 0;
  for (int eye = 0; eye < 2; eye++)
    for (int i = 0; i <= 20; i++)
      for (int j = 0; j <= 20; j++) {
        const float p[2] = {0.02f + 0.96f * i / 20.0f, 0.02f + 0.96f * j / 20.0f};
        float q[2], back[2];
        foveationClientSample(f, eye, p, q); // where the client reads this point in the encoded eye
        const float frame[2] = {(eye + q[0]) * 0.5f, q[1]};
        foveationSource(f, frame, back); // what the server put there, in the composed frame
        const float expected[2] = {(eye + p[0]) * 0.5f, p[1]};
        worst = std::max(worst, std::max(std::fabs(back[0] - expected[0]), std::fabs(back[1] - expected[1])));
      }
  std::printf("foveation: worst round trip %.6f of the frame\n", worst);
  check(worst < 1e-3f, "foveation: the client's stretch undoes the server's squeeze everywhere");
  check(!computeFoveation(2144, 2336, 0.45f, 0.4f, 0.4f, 0.1f, 0.5f, 5.0f).enabled,
        "foveation: an edge ratio under 1 is refused");
}

} // namespace

int main() {
  json();
  lighthouseIdentify();
  lighthouseConfig();
  lighthouseImu();
  lighthouseCommands();
  lighthouseControllers();
  riftS();
  riftSCameras();
  psvr2Headset();
  psvr2Sense();
  headsetTable();
  foveation();
  std::printf("%s: %d checks, %d failed\n", g_failures ? "FAILED" : "PASSED", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
