#include "lighthouse_protocol.h"

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "json_value.h"

namespace metalvr::lighthouse {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravity = 9.80665f;
constexpr size_t kMaxCompressed = 16 * 1024;
constexpr size_t kMaxJson = 64 * 1024;

uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24); }

bool vec3(const JsonValue &v, Vec3 &out) {
  float f[3];
  if (!v.numbers(f, 3)) return false;
  out = {f[0], f[1], f[2]};
  return true;
}

// a row-major 3x3 matrix from an array of three rows
bool matrix3(const JsonValue &v, float out[9]) {
  if (!v.isArray() || v.size() != 3) return false;
  for (size_t row = 0; row < 3; row++)
    if (!v[row].numbers(out + 3 * row, 3)) return false;
  return true;
}

void readImu(const JsonValue &root, Vec3 &acc_bias, Vec3 &acc_scale, Vec3 &gyro_bias, Vec3 &gyro_scale) {
  // early devices (the original Vive) keep the IMU's numbers at the root
  const JsonValue &imu = root["imu"].isObject() ? root["imu"] : root;
  vec3(imu["acc_bias"], acc_bias);
  vec3(imu["acc_scale"], acc_scale);
  vec3(imu["gyro_bias"], gyro_bias);
  vec3(imu["gyro_scale"], gyro_scale); // not on every device
}

std::string modelNumber(const JsonValue &root) {
  if (root["model_number"].isString()) return root["model_number"].string();
  if (root["model_name"].isString()) return root["model_name"].string();
  return {};
}

// a rotation matrix (row-major) as a quaternion
mvr_quat quatFromMatrix(const float m[9]) {
  const float trace = m[0] + m[4] + m[8];
  mvr_quat q{};
  if (trace > 0) {
    const float s = 0.5f / std::sqrt(trace + 1.0f);
    q = {(m[7] - m[5]) * s, (m[2] - m[6]) * s, (m[3] - m[1]) * s, 0.25f / s};
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const float s = 2.0f * std::sqrt(1.0f + m[0] - m[4] - m[8]);
    q = {0.25f * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s, (m[7] - m[5]) / s};
  } else if (m[4] > m[8]) {
    const float s = 2.0f * std::sqrt(1.0f + m[4] - m[0] - m[8]);
    q = {(m[1] + m[3]) / s, 0.25f * s, (m[5] + m[7]) / s, (m[2] - m[6]) / s};
  } else {
    const float s = 2.0f * std::sqrt(1.0f + m[8] - m[0] - m[4]);
    q = {(m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25f * s, (m[3] - m[1]) / s};
  }
  const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(n > 0)) return {0, 0, 0, 1};
  return {q.x / n, q.y / n, q.z / n, q.w / n};
}

// Monado's _calculate_fov: the angles from the view's center to the panel's edges, from the lens intrinsics
void computeFov(HeadsetConfig &c) {
  const float width = static_cast<float>(c.eye_width), height = static_cast<float>(c.eye_height);
  for (int eye = 0; eye < 2; eye++) {
    const Distortion &d = c.distortion[eye];
    const float scale = d.grow_for_undistort + 1.0f;
    const float focus_x = d.intrinsic_focus[0] / scale * width / 2.0f;
    const float focus_y = d.intrinsic_focus[1] / scale * height / 2.0f;
    const float center_x = (d.intrinsic_center[0] / scale - 1.0f) * -1.0f * width / 2.0f;
    const float center_y = (d.intrinsic_center[1] / scale + 1.0f) * height / 2.0f;
    if (!(focus_x > 0) || !(focus_y > 0)) {
      c.fov[eye] = {};
      continue;
    }
    c.fov[eye].up = std::atan(center_y / focus_y);
    c.fov[eye].down = -std::atan((height - center_y) / focus_y);
    c.fov[eye].left = -std::atan(center_x / focus_x);
    c.fov[eye].right = std::atan((width - center_x) / focus_x);
  }
}

// MPU-6500 full-scale codes to rad/s and m/s^2
void rangesFromCodes(uint8_t gyro, uint8_t accel, ImuRanges &out) {
  out.gyro = kPi / 180.0f * static_cast<float>(250 << gyro);
  out.accel = kGravity * static_cast<float>(2 << accel);
}

void scaled(const int16_t raw[3], float range, const Vec3 &scale, const Vec3 &bias, float out[3]) {
  const float k = range / 32768.0f;
  out[0] = k * scale.x * raw[0] - bias.x;
  out[1] = k * scale.y * raw[1] - bias.y;
  out[2] = k * scale.z * raw[2] - bias.z;
}

// Monado's convert_imu_to_openxr: each model's IMU is mounted its own way
void headsetToOpenXr(Model model, float v[3]) {
  const float x = v[0], y = v[1], z = v[2];
  switch (model) {
  case Model::Vive:
    v[0] = x, v[1] = -y, v[2] = -z;
    break;
  case Model::VivePro:
  case Model::VivePro2:
    v[0] = -x, v[1] = y, v[2] = -z;
    break;
  case Model::Index:
    v[0] = -y, v[1] = -x, v[2] = -z;
    break;
  case Model::None:
    break;
  }
}

void controllerToOpenXr(ControllerKind kind, float v[3]) {
  const float x = v[0], y = v[1], z = v[2];
  switch (kind) {
  case ControllerKind::Wand:
    v[0] = -x, v[1] = -z, v[2] = -y;
    break;
  case ControllerKind::IndexRight:
    v[0] = z, v[1] = -y, v[2] = x;
    break;
  case ControllerKind::IndexLeft:
    v[0] = -z, v[1] = x, v[2] = -y;
    break;
  case ControllerKind::Unknown:
    break;
  }
}

} // namespace

uint32_t headsetOf(Model model) {
  switch (model) {
  case Model::Vive: return MVR_HEADSET_VIVE;
  case Model::VivePro: return MVR_HEADSET_VIVE_PRO;
  case Model::VivePro2: return MVR_HEADSET_VIVE_PRO_2;
  case Model::Index: return MVR_HEADSET_VALVE_INDEX;
  case Model::None: break;
  }
  return MVR_HEADSET_UNKNOWN;
}

const char *nameOf(Model model) {
  switch (model) {
  case Model::Vive: return "HTC Vive";
  case Model::VivePro: return "HTC Vive Pro";
  case Model::VivePro2: return "HTC Vive Pro 2";
  case Model::Index: return "Valve Index";
  case Model::None: break;
  }
  return "no Lighthouse headset";
}

Model identify(const std::vector<UsbId> &devices) {
  const auto has = [&](uint16_t vendor, uint16_t product) {
    return std::any_of(devices.begin(), devices.end(),
                       [&](const UsbId &d) { return d.vendor == vendor && d.product == product; });
  };
  // the HTC mainboard names an HTC headset; its IMU device has to be there too for it to be of any use
  if (has(kHtcVendor, kViveMainboard)) return has(kValveVendor, kViveSensors) ? Model::Vive : Model::None;
  if (has(kHtcVendor, kVivePro2Mainboard)) return has(kValveVendor, kLhrSensors) ? Model::VivePro2 : Model::None;
  if (has(kHtcVendor, kViveProMainboard) || has(kHtcVendor, kViveProMainboardB))
    return has(kValveVendor, kLhrSensors) ? Model::VivePro : Model::None;
  if (has(kValveVendor, kLhrSensors)) return Model::Index; // until its configuration confirms it
  return Model::None;
}

Model modelFromConfig(std::string_view n) {
  if (n == "Utah MP" || n == "Index") return Model::Index;
  if (n == "Vive MV" || n == "Vive MV." || n == "Vive. MV") return Model::Vive;
  if (n == "Vive_Pro MV" || n == "VIVE_Pro MV") return Model::VivePro;
  if (n == "Vive_Pro 2 MV" || n == "VIVE_Pro 2 MV" || n == "VIVE_Pro 2 PV" || n == "Vive_Pro 2 PV")
    return Model::VivePro2;
  return Model::None;
}

bool ConfigAssembler::add(const uint8_t *report, size_t size) {
  if (done_ || size < 2 || report[0] != kConfigReadReport) return false;
  const size_t length = report[1];
  if (length > 62 || length + 2 > size || compressed_.size() + length > kMaxCompressed) return false;
  if (length == 0) {
    done_ = true;
    return true;
  }
  compressed_.insert(compressed_.end(), report + 2, report + 2 + length);
  return true;
}

std::optional<std::string> ConfigAssembler::json() const {
  if (!done_) return std::nullopt;
  return inflateConfig(compressed_);
}

std::optional<std::string> inflateConfig(const std::vector<uint8_t> &compressed) {
  if (compressed.empty()) return std::nullopt;
  std::string out(kMaxJson, '\0');
  z_stream stream{};
  stream.next_in = const_cast<Bytef *>(compressed.data());
  stream.avail_in = static_cast<uInt>(compressed.size());
  stream.next_out = reinterpret_cast<Bytef *>(out.data());
  stream.avail_out = static_cast<uInt>(out.size());
  if (inflateInit(&stream) != Z_OK) return std::nullopt;
  const int result = inflate(&stream, Z_FINISH);
  const size_t produced = stream.total_out;
  inflateEnd(&stream);
  if (result != Z_STREAM_END) return std::nullopt;
  out.resize(produced);
  return out;
}

std::optional<HeadsetConfig> parseHeadsetConfig(std::string_view text) {
  const auto root = JsonValue::parse(text);
  if (!root || !root->isObject()) return std::nullopt;
  HeadsetConfig c;
  c.model_number = modelNumber(*root);
  c.model = modelFromConfig(c.model_number);
  if (c.model == Model::None) return std::nullopt;
  readImu(*root, c.acc_bias, c.acc_scale, c.gyro_bias, c.gyro_scale);
  if ((*root)["device_serial_number"].isString()) c.serial = (*root)["device_serial_number"].string();
  if ((*root)["lens_separation"].isNumber())
    c.lens_separation = static_cast<float>((*root)["lens_separation"].number());
  else if ((*root)["ipd"]["default_mm"].isNumber())
    c.lens_separation = static_cast<float>((*root)["ipd"]["default_mm"].number() / 1000.0);
  const JsonValue &device = (*root)["device"];
  if (device["eye_target_width_in_pixels"].isNumber())
    c.eye_width = static_cast<int>(device["eye_target_width_in_pixels"].number());
  if (device["eye_target_height_in_pixels"].isNumber())
    c.eye_height = static_cast<int>(device["eye_target_height_in_pixels"].number());
  for (Distortion &d : c.distortion)
    d.aspect_x_over_y = static_cast<float>(device["physical_aspect_x_over_y"].number(d.aspect_x_over_y));

  const JsonValue &eyes = (*root)["tracking_to_eye_transform"];
  for (int eye = 0; eye < 2; eye++) {
    const JsonValue &e = eyes[static_cast<size_t>(eye)];
    if (!e.isObject()) return std::nullopt; // no lens, nothing to draw with
    Distortion &d = c.distortion[eye];
    float eye_to_head[9];
    if (matrix3(e["eye_to_head"], eye_to_head)) c.eye_rotation[eye] = quatFromMatrix(eye_to_head);
    d.grow_for_undistort = static_cast<float>(e["grow_for_undistort"].number(d.grow_for_undistort));
    d.undistort_r2_cutoff = static_cast<float>(e["undistort_r2_cutoff"].number(d.undistort_r2_cutoff));
    float k[9];
    if (!matrix3(e["intrinsics"], k)) return std::nullopt;
    // a pinhole camera's matrix: [fx 0 cx; 0 fy cy; 0 0 -1]
    d.intrinsic_focus[0] = k[0];
    d.intrinsic_focus[1] = k[4];
    d.intrinsic_center[0] = k[2];
    d.intrinsic_center[1] = k[5];
    static const char *const names[3] = {"distortion_red", "distortion", "distortion_blue"};
    for (int channel = 0; channel < 3; channel++) {
      const JsonValue &dist = e[names[channel]];
      if (!dist.isObject()) continue;
      d.center[channel][0] = static_cast<float>(dist["center_x"].number());
      d.center[channel][1] = static_cast<float>(dist["center_y"].number());
      // a Vive has 8 coefficients of which 3 are used, an Index 4
      const JsonValue &coeffs = dist["coeffs"];
      for (size_t i = 0; i < 4 && i < coeffs.size(); i++)
        d.coefficients[channel][i] = static_cast<float>(coeffs[i].number());
    }
  }
  computeFov(c);
  for (const mvr_fov &f : c.fov)
    if (!(f.left < 0 && f.right > 0 && f.up > 0 && f.down < 0)) return std::nullopt;
  return c;
}

void computeDistortion(const Distortion &d, float u, float v, bool flip_y, float out[3][2]) {
  const float factor = 0.5f / (1.0f + d.grow_for_undistort);
  for (int i = 0; i < 3; i++) {
    float x = 2.0f * u - 1.0f;
    float y = (2.0f * v - 1.0f) / d.aspect_x_over_y;
    x -= d.center[i][0];
    y -= d.center[i][1];
    const float r2 = x * x + y * y;
    const float k1 = d.coefficients[i][0], k2 = d.coefficients[i][1], k3 = d.coefficients[i][2];
    const float k4 = d.coefficients[i][3];
    const float scale = 1.0f / (1.0f + r2 * (k1 + r2 * (k2 + r2 * k3))) + k4;
    out[i][0] = 0.5f + (x * scale + d.center[i][0]) * factor;
    out[i][1] = 0.5f + (y * scale + d.center[i][1]) * factor * d.aspect_x_over_y;
    if (flip_y) out[i][1] = 1.0f - out[i][1];
  }
}

std::optional<ImuRanges> parseImuRanges(const uint8_t *report, size_t size) {
  if (size < 3 || report[0] != kImuRangeReport) return std::nullopt;
  const uint8_t gyro = report[1], accel = report[2];
  if (gyro == 0 || accel == 0 || gyro > 4 || accel > 4) return std::nullopt;
  ImuRanges ranges;
  rangesFromCodes(gyro, accel, ranges);
  return ranges;
}

size_t parseImuReport(const uint8_t *report, size_t size, const HeadsetConfig &config, const ImuRanges &ranges,
                      uint8_t &sequence, bool &have_sequence, ImuSample out[3]) {
  if (size < kImuReportSize || report[0] != kImuReport) return 0;
  // {accel 3 x int16, gyro 3 x int16, time u32, sequence u8} x 3, updated round robin
  const auto at = [&](int i) { return report + 1 + 17 * i; };
  const uint8_t s0 = at(0)[16], s1 = at(1)[16], s2 = at(2)[16];
  // Monado's oldest_sequence_index: start at the oldest
  int i = 0;
  if (s0 == static_cast<uint8_t>(s1 + 2)) i = 1;
  else if (s1 == static_cast<uint8_t>(s2 + 2)) i = 2;
  size_t n = 0;
  // what was seen is judged against the newest sequence number before this report, as Monado does
  const uint8_t last = sequence;
  const bool had = have_sequence;
  for (int j = 0; j < 3; j++, i = (i + 1) % 3) {
    const uint8_t *s = at(i);
    const uint8_t seq = s[16];
    if (had && (seq == last || seq == static_cast<uint8_t>(last - 1) || seq == static_cast<uint8_t>(last - 2)))
      continue;
    const int16_t acc[3] = {static_cast<int16_t>(le16(s)), static_cast<int16_t>(le16(s + 2)),
                            static_cast<int16_t>(le16(s + 4))};
    const int16_t gyro[3] = {static_cast<int16_t>(le16(s + 6)), static_cast<int16_t>(le16(s + 8)),
                             static_cast<int16_t>(le16(s + 10))};
    ImuSample &sample = out[n++];
    sample.ticks = le32(s + 12);
    scaled(acc, ranges.accel, config.acc_scale, config.acc_bias, sample.accel);
    scaled(gyro, ranges.gyro, config.gyro_scale, config.gyro_bias, sample.gyro);
    headsetToOpenXr(config.model, sample.accel);
    headsetToOpenXr(config.model, sample.gyro);
    sequence = seq;
    have_sequence = true;
  }
  return n;
}

std::optional<MainboardStatus> parseMainboardStatus(const uint8_t *report, size_t size) {
  // {id, u16, len, lens separation u16, u16, button, audio button, 2, proximity change, 1, proximity u16, ipd u16}
  if (size < 18 || report[0] != kMainboardStatusReport) return std::nullopt;
  MainboardStatus status;
  status.lens_separation = le16(report + 4) * 1e-5f; // hundredths of a millimeter
  status.proximity = le16(report + 14);
  status.ipd = le16(report + 16) * 1e-5f;
  return status;
}

std::array<uint8_t, 64> powerReport(bool on) {
  // {id 4, type 0x2978, len 56, 9 bytes, 32 reserved, 1 byte, 18 reserved}: Monado's power_on_report
  std::array<uint8_t, 64> r{};
  r[0] = 0x04;
  r[1] = 0x78;
  r[2] = 0x29;
  r[3] = 56;
  const uint8_t on_bytes[9] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01};
  const uint8_t off_bytes[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00};
  std::memcpy(r.data() + 4, on ? on_bytes : off_bytes, 9);
  r[4 + 9 + 32] = on ? 0x7a : 0x7c;
  return r;
}

const std::array<Pro2Mode, 6> &pro2Modes() {
  static const std::array<Pro2Mode, 6> modes = {{
      {0, 2448, 1224, 90.03f, false},
      {1, 2448, 1224, 120.05f, false},
      {2, 3264, 1632, 90.0f, false},
      {3, 3680, 1836, 90.02f, false},
      {4, 4896, 2448, 90.02f, true},
      {5, 4896, 2448, 120.02f, true},
  }};
  return modes;
}

std::array<uint8_t, 64> pro2Command(std::string_view text) {
  std::array<uint8_t, 64> r{};
  r[0] = 0x04;
  r[1] = 0x70; // 0x2970, little-endian
  r[2] = 0x29;
  const size_t n = std::min(text.size(), r.size() - 4);
  r[3] = static_cast<uint8_t>(n);
  std::memcpy(r.data() + 4, text.data(), n);
  return r;
}

std::optional<ControllerConfig> parseControllerConfig(std::string_view text) {
  const auto root = JsonValue::parse(text);
  if (!root || !root->isObject()) return std::nullopt;
  ControllerConfig c;
  c.model_number = modelNumber(*root);
  const std::string &n = c.model_number;
  if (n == "Vive. Controller MV" || n == "Vive Controller MV")
    c.kind = ControllerKind::Wand;
  else if (n == "Knuckles Right" || n == "Knuckles EV3.0 Right")
    c.kind = ControllerKind::IndexRight;
  else if (n == "Knuckles Left" || n == "Knuckles EV3.0 Left")
    c.kind = ControllerKind::IndexLeft;
  readImu(*root, c.acc_bias, c.acc_scale, c.gyro_bias, c.gyro_scale);
  return c;
}

namespace {

void controllerImu(const uint8_t *p, uint32_t message_ticks, const ControllerConfig &config, const ImuRanges &ranges,
                   ControllerImu &out) {
  // {timestamp high byte, accel 3 x int16, gyro 3 x int16}; the time is in 48 MHz ticks, its low byte missing
  out.ticks = message_ticks | (static_cast<uint32_t>(p[0]) << 8);
  const int16_t acc[3] = {static_cast<int16_t>(le16(p + 1)), static_cast<int16_t>(le16(p + 3)),
                          static_cast<int16_t>(le16(p + 5))};
  const int16_t gyro[3] = {static_cast<int16_t>(le16(p + 7)), static_cast<int16_t>(le16(p + 9)),
                           static_cast<int16_t>(le16(p + 11))};
  scaled(acc, ranges.accel, config.acc_scale, config.acc_bias, out.accel);
  scaled(gyro, ranges.gyro, config.gyro_scale, config.gyro_bias, out.gyro);
  controllerToOpenXr(config.kind, out.accel);
  controllerToOpenXr(config.kind, out.gyro);
}

constexpr size_t kImuSampleSize = 13;
constexpr size_t kTouchForceSize = 8;
constexpr uint8_t kTypeFlagTouchForce = 161;

void battery(uint8_t b, ControllerState &s) {
  s.battery = b & 0x7f;
  s.charging = (b & 0x80) != 0;
  s.has_battery = true;
}

void touchForce(const uint8_t *p, ControllerState &s) {
  // {type flag, touched, middle, ring, pinky, index on the trigger, squeeze force, trackpad force}
  s.touch = p[1];
  s.fingers[1] = p[2];
  s.fingers[2] = p[3];
  s.fingers[3] = p[4];
  s.fingers[0] = p[5];
  s.squeeze_force = p[6];
  s.trackpad_force = p[7];
  s.has_fingers = true;
}

void pad(const uint8_t *p, ControllerState &s) {
  s.pad_x = static_cast<float>(static_cast<int16_t>(le16(p))) / 32767.0f;
  s.pad_y = static_cast<float>(static_cast<int16_t>(le16(p + 2))) / 32767.0f;
}

// Monado's vive_controller_decode_watchmanv1
size_t decodeV1(const uint8_t *buf, const uint8_t *end, uint32_t ticks, const ControllerConfig &config,
                const ImuRanges &ranges, ControllerState &s, ControllerImu *imu) {
  size_t samples = 0;
  if ((*buf & 0xe0) != 0xe0 || buf >= end) return 0;
  const uint8_t flags = *buf++;
  const bool has_battery = (flags & 0x10) != 0x10 && (flags & 0x1) == 0x1;
  const bool has_trigger = (flags & 0x10) == 0x10 && (flags & 0x4) == 0x4;
  const bool has_trackpad = (flags & 0x10) == 0x10 && (flags & 0x2) == 0x2;
  const bool has_buttons = (flags & 0x10) == 0x10 && (flags & 0x1) == 0x1;
  const bool has_imu = (flags & 0x08) == 0x8;
  const auto room = [&](size_t n) { return buf + n <= end + 1; };
  if (has_battery && room(1)) battery(*buf++, s);
  if (has_buttons && room(1)) s.buttons = *buf++;
  if (has_trigger && room(1)) s.trigger = *buf++ / 255.0f;
  if (has_trackpad && room(4)) {
    pad(buf, s);
    buf += 4;
  }
  if (has_imu && room(kImuSampleSize)) {
    controllerImu(buf, ticks, config, ranges, imu[samples++]);
    buf += kImuSampleSize;
  }
  return samples;
}

// Monado's vive_controller_decode_watchmanv2
size_t decodeV2(const uint8_t *buf, const uint8_t *end, uint32_t ticks, const ControllerConfig &config,
                const ImuRanges &ranges, ControllerState &s, ControllerImu *imu) {
  size_t samples = 0;
  const auto room = [&](size_t n) { return buf + n <= end + 1; };
  if (buf < end && *buf == 0xe1) {
    buf++;
    if (room(1)) battery(*buf++, s);
  }
  if (buf < end && *buf == 0xf0) {
    buf++;
    if (room(kTouchForceSize)) {
      touchForce(buf, s);
      buf += kTouchForceSize;
    }
  }
  if (buf < end && *buf == 0xe8) {
    buf++;
    if (room(kImuSampleSize)) {
      controllerImu(buf, ticks, config, ranges, imu[samples++]);
      buf += kImuSampleSize;
    }
  }
  if (buf < end - 1 && (*buf & 0xf0) == 0xf0) {
    const uint8_t flags = *buf++;
    if ((flags & 0x1) && room(1)) s.buttons = *buf++;
    if ((flags & 0x4) && room(1)) s.trigger = *buf++ / 255.0f;
    if ((flags & 0x2) && room(4)) {
      pad(buf, s);
      buf += 4;
    }
    if ((flags & 0x8) && room(kTouchForceSize) && *buf == kTypeFlagTouchForce) {
      touchForce(buf, s);
      buf += kTouchForceSize;
    }
    if (buf < end && end - buf >= static_cast<long>(kImuSampleSize)) {
      controllerImu(buf, ticks, config, ranges, imu[samples++]);
      buf += kImuSampleSize;
    }
  }
  return samples;
}

} // namespace

size_t decodeControllerReport(const uint8_t *report, size_t size, bool gen2, const ControllerConfig &config,
                              const ImuRanges &ranges, ControllerState &state, ControllerImu imu[4]) {
  if (size < 1) return 0;
  int messages = 0;
  if (report[0] == kControllerReport1 && size >= 30) messages = 1;
  if (report[0] == kControllerReport2 && size >= 59) messages = 2;
  size_t samples = 0;
  for (int m = 0; m < messages; m++) {
    // {timestamp high, length, timestamp low, payload 26}
    const uint8_t *message = report + 1 + 29 * m;
    const uint8_t length = message[1];
    if (length < 1 || length > 26) continue;
    const uint8_t *payload = message + 3;
    const uint8_t *end = payload + length - 1;
    const uint32_t ticks = (static_cast<uint32_t>(message[0]) << 24) | (static_cast<uint32_t>(message[2]) << 16);
    samples += gen2 ? decodeV2(payload, end, ticks, config, ranges, state, imu + samples)
                    : decodeV1(payload, end, ticks, config, ranges, state, imu + samples);
  }
  return samples;
}

std::array<uint8_t, 10> hapticReport(float duration_s, float frequency_hz, float amplitude) {
  // a pulse train: frequency = 1e6 / (high + low) microseconds, amplitude the low part's share, Monado's mapping
  if (!(frequency_hz > 0)) frequency_hz = 150.0f;
  frequency_hz = std::clamp(frequency_hz, 1.0f, 1000.0f);
  amplitude = std::clamp(std::isfinite(amplitude) ? amplitude : 0.0f, 0.0f, 1.0f);
  duration_s = std::clamp(std::isfinite(duration_s) ? duration_s : 0.0f, 0.0f, 10.0f);
  const float high_plus_low = 1e6f / frequency_hz;
  uint16_t low = static_cast<uint16_t>(amplitude * high_plus_low / 2.0f);
  if (low == 0) low = 1; // a Vive wand does not move with 0
  const uint16_t high = static_cast<uint16_t>(high_plus_low - low);
  const uint16_t repeat = duration_s > 0 ? static_cast<uint16_t>(std::max(1.0f, duration_s * frequency_hz)) : 0;
  return {0xff, 0x8f, 7, 0, static_cast<uint8_t>(high & 0xff), static_cast<uint8_t>(high >> 8),
          static_cast<uint8_t>(low & 0xff), static_cast<uint8_t>(low >> 8), static_cast<uint8_t>(repeat & 0xff),
          static_cast<uint8_t>(repeat >> 8)};
}

} // namespace metalvr::lighthouse
