#include "psvr2_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "psvr2_lens_table.h"

namespace metalvr::psvr2 {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravity = 9.80665f;
constexpr float kGyroScale = 2000.0f / 32767.0f;        // degrees/s per count
constexpr float kAccelScale = 4.0f * kGravity / 32767.0f; // m/s^2 per count

uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24); }
float lef32(const uint8_t *p) {
  const uint32_t bits = le32(p);
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}
void put16(uint8_t *p, uint32_t v) { p[0] = static_cast<uint8_t>(v), p[1] = static_cast<uint8_t>(v >> 8); }
void put32(uint8_t *p, uint32_t v) {
  for (int i = 0; i < 4; i++) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

mvr_quat mul(const mvr_quat &a, const mvr_quat &b) {
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
mvr_quat conj(const mvr_quat &q) { return {-q.x, -q.y, -q.z, q.w}; }
mvr_vec3 rotate(const mvr_quat &q, const mvr_vec3 &v) {
  const float tx = 2 * (q.y * v.z - q.z * v.y), ty = 2 * (q.z * v.x - q.x * v.z), tz = 2 * (q.x * v.y - q.y * v.x);
  return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
          v.z + q.w * tz + (q.x * ty - q.y * tx)};
}
mvr_quat normalize(mvr_quat q) {
  const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(n > 0)) return {0, 0, 0, 1};
  return {q.x / n, q.y / n, q.z / n, q.w / n};
}
mvr_quat aboutX(float degrees) {
  const float half = degrees * kPi / 360.0f;
  return {std::sin(half), 0, 0, std::cos(half)};
}
// a's inverse composed with b: b in a's frame
mvr_pose relative(const mvr_pose &a, const mvr_pose &b) {
  const mvr_quat inv = conj(a.orientation);
  const mvr_vec3 d{b.position.x - a.position.x, b.position.y - a.position.y, b.position.z - a.position.z};
  return {normalize(mul(inv, b.orientation)), rotate(inv, d)};
}

ControlRequest request(bool read, uint16_t report_id, uint16_t subcommand, const uint8_t *payload, uint32_t length) {
  ControlRequest r;
  r.request_type = static_cast<uint8_t>(0x40 | 0x02 | (read ? 0x80 : 0)); // vendor, to an endpoint
  r.request = read ? 0x01 : 0x09;
  r.value = report_id;
  r.data.assign(8 + length, 0);
  put16(r.data.data(), report_id);
  put16(r.data.data() + 2, subcommand);
  put32(r.data.data() + 4, length);
  if (payload && length) std::memcpy(r.data.data() + 8, payload, length);
  return r;
}

} // namespace

// ---- status ----

std::optional<Status> parseStatus(const uint8_t *r, size_t size) {
  // {DisplayPort receiver, proximity, function button, 2 empty, IPD mm, 26 more} then 24-byte IMU records:
  // {vts u32, accel i16 x 3, gyro i16 x 3, DisplayPort frame u16, line u16, IMU time u16, status u16}
  constexpr size_t kHeader = 32, kRecord = 24;
  if (size < kHeader) return std::nullopt;
  Status s;
  s.display = r[0];
  s.worn = r[1] != 0;
  s.function = r[2] != 0;
  s.ipd_mm = r[5];
  const float degrees = kPi / 180.0f;
  for (size_t at = kHeader; at + kRecord <= size; at += kRecord) {
    const uint8_t *p = r + at;
    int16_t a[3], g[3];
    for (int i = 0; i < 3; i++) {
      a[i] = static_cast<int16_t>(le16(p + 4 + 2 * i));
      g[i] = static_cast<int16_t>(le16(p + 10 + 2 * i));
    }
    ImuSample sample;
    sample.vts_us = le32(p);
    // the IMU's axes into the headset's (Monado's process_imu_record)
    sample.gyro[0] = -g[1] * kGyroScale * degrees;
    sample.gyro[1] = g[2] * kGyroScale * degrees;
    sample.gyro[2] = -g[0] * kGyroScale * degrees;
    sample.accel[0] = -a[1] * kAccelScale;
    sample.accel[1] = a[2] * kAccelScale;
    sample.accel[2] = -a[0] * kAccelScale;
    s.imu.push_back(sample);
  }
  return s;
}

// ---- tracking ----

std::optional<SlamPose> parseSlam(const uint8_t *r, size_t size, const mvr_quat &previous) {
  // {"SLA", 1, size u32, vts u32, u32, position f32 x 3, orientation f32 x 4 (w first), ...}: 512 bytes
  if (size != 512 || std::memcmp(r, "SLA", 3) != 0) return std::nullopt;
  SlamPose s;
  s.vts_us = le32(r + 8);
  const float p[3] = {lef32(r + 16), lef32(r + 20), lef32(r + 24)};
  const float o[4] = {lef32(r + 28), lef32(r + 32), lef32(r + 36), lef32(r + 40)};
  for (float v : p)
    if (!std::isfinite(v)) return std::nullopt;
  for (float v : o)
    if (!std::isfinite(v)) return std::nullopt;
  s.pose.position = {p[2], p[1], -p[0]};
  mvr_quat q{-o[2], -o[1], o[3], o[0]};
  if (q.x * previous.x + q.y * previous.y + q.z * previous.z + q.w * previous.w < 0) q = {-q.x, -q.y, -q.z, -q.w};
  s.pose.orientation = q;
  return s;
}

mvr_pose headFromSlam(const mvr_pose &slam) {
  const float h = std::sqrt(0.5f);
  const mvr_quat correction{0, 0, h, h};
  const mvr_quat q = normalize(mul(correction, slam.orientation));
  const mvr_vec3 offset = rotate(q, mvr_vec3{0.000247f, -0.000273f, 0.104826f});
  return {q, {slam.position.x + offset.x, slam.position.y + offset.y, slam.position.z + offset.z}};
}

// ---- requests ----

ControlRequest sendRequest(uint16_t report_id, uint16_t subcommand, const uint8_t *payload, uint32_t length) {
  return request(false, report_id, subcommand, payload, length);
}
ControlRequest readRequest(uint16_t report_id, uint16_t subcommand, uint32_t length) {
  return request(true, report_id, subcommand, nullptr, length);
}
std::vector<uint8_t> readPayload(const uint8_t *reply, size_t size, uint32_t length) {
  if (size < 8) return {};
  const size_t n = std::min<size_t>(length, size - 8);
  return std::vector<uint8_t>(reply + 8, reply + 8 + n);
}

ControlRequest cameraModeRequest(uint32_t mode) {
  uint8_t payload[8];
  put32(payload, 1);
  put32(payload + 4, mode);
  return sendRequest(0x0b, 1, payload, sizeof(payload));
}

ControlRequest brightnessRequest(float brightness) {
  const uint8_t level = static_cast<uint8_t>(std::clamp(brightness * 31.0f, 0.0f, 31.0f));
  return sendRequest(0x12, 1, &level, 1);
}

ControlRequest firmwareInfoRequest() { return readRequest(0x81, 1, 84); }
ControlRequest lensCalibrationRequest() { return readRequest(0x8f, 1, 0x100); }

std::optional<FirmwareInfo> parseFirmwareInfo(const uint8_t *p, size_t size) {
  // {version u32, u32, commit 40, u32, u32, PCB id 16, ...}
  if (size < 72) return std::nullopt;
  FirmwareInfo f;
  f.version = le32(p);
  const char *id = reinterpret_cast<const char *>(p + 56);
  f.pcb_id.assign(id, strnlen(id, 16));
  return f;
}

// ---- lenses ----

std::array<float, 8> defaultLensCalibration() { return {-0.09919293f, 0, 0.09919293f, 0, 1, 0, 1, 0}; }

std::array<float, 8> lensCalibration(const uint8_t *payload, size_t size) {
  // {version u8, 7 bytes, f32 x 32}
  if (size < 8 + 7 * 4 || payload[0] < 4) return defaultLensCalibration();
  float p[7];
  for (int i = 0; i < 7; i++) p[i] = lef32(payload + 8 + 4 * i);
  for (float v : p)
    if (!std::isfinite(v)) return defaultLensCalibration();
  std::array<float, 8> c{};
  c[0] = (((-p[0] - p[6]) * 29.9f + 14.95f) / 1000.0f - 3.22f) / 32.46199f;
  c[1] = (((-p[1] * 29.9f) + 14.95f) / 1000.0f) / 32.46199f;
  c[2] = (((p[6] - p[2]) * 29.9f + 14.95f) / 1000.0f + 3.22f) / 32.46199f;
  c[3] = (((-p[3] * 29.9f) + 14.95f) / 1000.0f) / 32.46199f;
  const float left = -p[4] * kPi / 180.0f, right = -p[5] * kPi / 180.0f;
  c[4] = std::cos(left), c[5] = std::sin(left);
  c[6] = std::cos(right), c[7] = std::sin(right);
  return c;
}

void computeDistortion(const std::array<float, 8> &cal, int eye, float u, float v, float out[3][2]) {
  const float fx = 0.3585564f, fy = 0.3762281f, cy = 0.5f;
  const float k1 = cal[eye ? 2 : 0], k2 = cal[eye ? 3 : 1], k3 = cal[eye ? 6 : 4], k4 = cal[eye ? 7 : 5];
  const float cx = eye ? 0.3396213f : 0.6603788f;
  const float x = k1 + ((u + u) - 1.0f) * 0.9803922f * 0.9394987f;
  const float y = k2 + ((v + v) - 1.0f) * 0.9394987f;
  const float r = (x * x + y * y) * 1023.0f;
  const int i = static_cast<int>(r);
  float scale[3];
  if (i > -1 && i + 1 < kLensTableSize) {
    const float f = r - static_cast<float>(i);
    for (int c = 0; c < 3; c++) scale[c] = f * kLensTable[i + 1][c] + (1.0f - f) * kLensTable[i][c];
  } else {
    const float d = r - 1023.0f;
    scale[0] = d * 0.003648639f + 1.592586f;
    scale[1] = d * 0.004862547f + 1.692979f;
    scale[2] = d * 0.007425308f + 1.841771f;
  }
  for (int c = 0; c < 3; c++) {
    const float a = x * scale[c];
    const float b = (c == 1 ? y - 0.0002302693f : y) * scale[c];
    out[c][0] = (k3 * a - k4 * b) * fx + cx;
    out[c][1] = (k4 * a + k3 * b) * fy + cy;
  }
}

mvr_fov fieldOfView(int eye) {
  const float d = kPi / 180.0f;
  return eye == 0 ? mvr_fov{-61.5f * d, 43.5f * d, 53.0f * d, -53.0f * d}
                  : mvr_fov{-43.5f * d, 61.5f * d, 53.0f * d, -53.0f * d};
}

// ---- Sense ----

uint32_t crc32(uint32_t crc, const uint8_t *p, size_t size) {
  crc ^= 0xffffffffu;
  while (size--) {
    crc ^= *p++;
    for (int i = 0; i < 8; i++) crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0);
  }
  return crc ^ 0xffffffffu;
}

std::optional<SenseState> parseSenseReport(const uint8_t *r, size_t size, bool left) {
  const uint8_t *c = nullptr; // the part both carry (Monado's pssense_input_report_common)
  if (size == kSenseBluetoothLength && r[0] == kSenseBluetoothReport) {
    const uint8_t seed = 0xa1;
    uint32_t crc = crc32(0, &seed, 1);
    crc = crc32(crc, r, kSenseBluetoothLength - 4);
    if (crc != le32(r + kSenseBluetoothLength - 4)) return std::nullopt;
    c = r + 2;
  } else if (size == kSenseUsbLength && r[0] == kSenseUsbReport) {
    c = r + 1;
  } else {
    return std::nullopt;
  }
  // {stick x, stick y, trigger, trigger proximity, squeeze proximity, 2, buttons x 3, 1, sequence u32,
  //  gyro i16 x 3, accel i16 x 3, IMU ticks u32, temperature, 7, trigger feedback x 2, battery, plug, ...}
  const uint8_t *b = c + 7;
  SenseState s;
  s.stick_x = (c[0] - 128) / 128.0f;
  s.stick_y = (c[1] - 128) / -128.0f;
  s.trigger = c[2] / 255.0f;
  const bool ps = b[1] & 16, squeeze_touch = b[2] & 8, trigger_touch = b[1] & 128, stick_touch = b[2] & 4;
  bool primary, primary_touch, secondary, secondary_touch, squeeze, trigger_click, stick, menu;
  if (left) {
    // square is X, triangle Y; Create is the menu
    primary = b[0] & 1, primary_touch = b[2] & 2, secondary = b[0] & 8, secondary_touch = b[2] & 1;
    squeeze = b[0] & 16, trigger_click = b[0] & 64, stick = b[1] & 4, menu = b[1] & 1;
  } else {
    // cross is A, circle B; Options is the menu
    primary = b[0] & 2, primary_touch = b[2] & 2, secondary = b[0] & 4, secondary_touch = b[2] & 1;
    squeeze = b[0] & 32, trigger_click = b[0] & 128, stick = b[1] & 8, menu = b[1] & 2;
  }
  s.squeeze = squeeze ? 1.0f : 0.0f;
  s.buttons = (primary ? MVR_BUTTON_PRIMARY : 0u) | (secondary ? MVR_BUTTON_SECONDARY : 0u) |
              (menu ? MVR_BUTTON_MENU : 0u) | (ps ? MVR_BUTTON_SYSTEM : 0u) | (stick ? MVR_BUTTON_THUMBSTICK : 0u) |
              (squeeze ? MVR_BUTTON_SQUEEZE : 0u) | (trigger_click ? MVR_BUTTON_TRIGGER : 0u);
  s.touches = (primary_touch ? MVR_BUTTON_PRIMARY : 0u) | (secondary_touch ? MVR_BUTTON_SECONDARY : 0u) |
              (stick_touch ? MVR_BUTTON_THUMBSTICK : 0u) | (trigger_touch ? MVR_BUTTON_TRIGGER : 0u) |
              (squeeze_touch ? MVR_BUTTON_SQUEEZE : 0u);
  for (int i = 0; i < 3; i++) {
    s.gyro[i] = static_cast<int16_t>(le16(c + 15 + 2 * i));
    s.accel[i] = static_cast<int16_t>(le16(c + 21 + 2 * i));
  }
  s.imu_ticks = le32(c + 27);
  const uint8_t battery = c[41];
  const uint8_t state = battery >> 4;
  s.battery = std::min(1.0f, (battery & 0x0f) * 0.1f + 0.05f);
  s.battery_valid = state <= 2;
  s.charging = state == 1 || state == 2;
  if (state == 2) s.battery = 1.0f;
  return s;
}

std::optional<SenseCalibration> parseSenseCalibration(const uint8_t a[64], const uint8_t b[64]) {
  // each part {5, part id, 58 bytes, crc}: part 0x00 first in the calibration, 0x81 second
  if (a[0] != kSenseCalibrationReport || b[0] != kSenseCalibrationReport) return std::nullopt;
  const uint8_t *first = a[1] == 0x00 ? a : b[1] == 0x00 ? b : nullptr;
  const uint8_t *second = a[1] == 0x81 ? a : b[1] == 0x81 ? b : nullptr;
  if (!first || !second) return std::nullopt;
  uint8_t d[116];
  std::memcpy(d, first + 2, 58);
  std::memcpy(d + 58, second + 2, 58);
  const auto i16 = [&](size_t at) { return static_cast<float>(static_cast<int16_t>(le16(d + at))); };
  const float accel_plus[3] = {i16(0x00), i16(0x0e), i16(0x1c)}, accel_minus[3] = {i16(0x06), i16(0x14), i16(0x22)};
  const float gyro_bias[3] = {i16(0x24), i16(0x26), i16(0x28)};
  const float gyro_plus[3] = {i16(0x2a), i16(0x38), i16(0x46)}, gyro_minus[3] = {i16(0x30), i16(0x3e), i16(0x4c)};
  float reference = (i16(0x4e) + i16(0x50)) * 0.5f * kPi / 180.0f;
  if (!(reference > 0)) reference = kPi * 3.0f; // Monado's fallback
  SenseCalibration cal;
  for (int i = 0; i < 3; i++) {
    const float span = (std::fabs(gyro_plus[i] - gyro_bias[i]) + std::fabs(gyro_minus[i] - gyro_bias[i])) / 2.0f;
    const float bias = (accel_plus[i] + accel_minus[i]) / 2.0f;
    if (!(span > 0) || !(accel_plus[i] - bias != 0)) return std::nullopt;
    cal.gyro_bias[i] = gyro_bias[i];
    cal.gyro_scale[i] = reference / span;
    cal.accel_bias[i] = bias;
    cal.accel_scale[i] = kGravity / (accel_plus[i] - bias);
  }
  return cal;
}

void senseImu(const SenseState &s, const SenseCalibration &c, float accel[3], float gyro[3]) {
  for (int i = 0; i < 3; i++) {
    gyro[i] = (s.gyro[i] - c.gyro_bias[i]) * c.gyro_scale[i];
    accel[i] = (s.accel[i] - c.accel_bias[i]) * c.accel_scale[i];
  }
}

std::array<uint8_t, 78> senseOutputReport(float amplitude, uint8_t sequence, uint8_t counter, uint32_t host_time_us) {
  // {0x31, sequence << 4, 0x10, settings (38), counter, PCM haptics (32), crc}; settings: {flag1, flag2, amplitude,
  //  0, trigger (11), host time u32, LEDs {phase, sequence, period, position u32, length u32, blink x 4}, 4}
  std::array<uint8_t, 78> r{};
  r[0] = kSenseBluetoothReport;
  r[1] = static_cast<uint8_t>((sequence & 0x0f) << 4);
  r[2] = 0x10;
  uint8_t *settings = r.data() + 3;
  const uint8_t level = static_cast<uint8_t>(std::clamp(amplitude, 0.0f, 1.0f) * 255.0f + 0.5f);
  settings[0] = 0x01 | 0x02; // vibration on (with rumble emulation), at 120 Hz
  settings[2] = level;
  put32(settings + 15, host_time_us);
  settings[19] = 5; // tracking LEDs all off
  r[41] = counter;
  const uint8_t seed = 0xa2;
  uint32_t crc = crc32(0, &seed, 1);
  crc = crc32(crc, r.data(), r.size() - 4);
  put32(r.data() + 74, crc);
  return r;
}

namespace {
// Monado's T_steamvrroot_model, _xrgrip and _xraim, for a hand
mvr_pose senseRoot(int hand, int which) {
  const float sx = hand == 0 ? 1.0f : -1.0f;
  if (which == 0) return {aboutX(-39.0f), {-sx * 0.01432f, 0.007713f, 0.10399f}};
  if (which == 1) return {aboutX(20.6f), {sx * 0.007f, -0.00182941f, 0.1019482f}};
  return {aboutX(-39.4f), {sx * 0.007f, -0.03894766f, 0.00949694f}};
}
} // namespace

mvr_pose senseGrip(int hand) { return relative(senseRoot(hand, 0), senseRoot(hand, 1)); }
mvr_pose senseAim(int hand) { return relative(senseRoot(hand, 0), senseRoot(hand, 2)); }

} // namespace metalvr::psvr2
