#include "rift_s_protocol.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <utility>

#include "json_value.h"

namespace metalvr::rift_s {

namespace {

constexpr float kGravity = 9.80665f;
constexpr float kDegrees = 3.14159265358979323846f / 180.0f;

uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24); }
uint64_t le64(const uint8_t *p) { return le32(p) | (static_cast<uint64_t>(le32(p + 4)) << 32); }
float lef32(const uint8_t *p) {
  const uint32_t bits = le32(p);
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

void apply(const float m[9], const float in[3], float out[3]) {
  const float x = in[0], y = in[1], z = in[2];
  out[0] = m[0] * x + m[1] * y + m[2] * z;
  out[1] = m[3] * x + m[4] * y + m[5] * z;
  out[2] = m[6] * x + m[7] * y + m[8] * z;
}

} // namespace

std::optional<DeviceInfo> parseDeviceInfo(const uint8_t *r, size_t size) {
  // {id, height u16, width u16, u16, refresh u8, ...}: the panel's resolution as the computer sees it
  if (size < 8 || r[0] != kReportDeviceInfo) return std::nullopt;
  DeviceInfo info;
  info.height = le16(r + 1);
  info.width = le16(r + 3);
  info.refresh_hz = r[7];
  if (!info.width || !info.height || !info.refresh_hz) return std::nullopt;
  return info;
}

std::optional<ImuConfig> parseImuConfig(const uint8_t *r, size_t size) {
  // {id, imu hz u32, gyro scale f32, accel scale f32, temperature scale f32, temperature offset f32}
  if (size < 21 || r[0] != kReportImuConfig) return std::nullopt;
  ImuConfig c;
  c.hz = le32(r + 1);
  c.gyro_scale = lef32(r + 5);
  c.accel_scale = lef32(r + 9);
  if (!c.hz || !(c.gyro_scale > 0) || !(c.accel_scale > 0) || !std::isfinite(c.gyro_scale) ||
      !std::isfinite(c.accel_scale))
    return std::nullopt;
  return c;
}

std::optional<ImuCalibration> parseImuCalibration(std::string_view text) {
  const auto root = JsonValue::parse(text);
  if (!root || !root->isObject()) return std::nullopt;
  const JsonValue &version = (*root)["FileFormat"]["Version"];
  if (!version.isString() || std::strtof(version.string().c_str(), nullptr) != 1.0f) return std::nullopt;
  const JsonValue &imu = (*root)["ImuCalibration"];
  ImuCalibration c;
  // a row-major 4x4 (or 3x4) isometry: the rotation is its upper left 3x3
  float m[12];
  if (!imu["DeviceFromImu"].numbers(m, 12)) return std::nullopt;
  const float rotation[9] = {m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]};
  std::memcpy(c.device_from_imu, rotation, sizeof(rotation));
  c.imu_position[0] = m[3];
  c.imu_position[1] = m[7];
  c.imu_position[2] = m[11];
  const JsonValue &gyro = imu["Gyroscope"];
  const JsonValue &accel = imu["Accelerometer"];
  if (!gyro["RectificationMatrix"].numbers(c.gyro_rectification, 9) ||
      !gyro["Offset"]["ConstantOffset"].numbers(c.gyro_offset, 3) ||
      !accel["RectificationMatrix"].numbers(c.accel_rectification, 9) ||
      !accel["Offset"]["OffsetAtZeroDegC"].numbers(c.accel_offset, 3))
    return std::nullopt;
  return c;
}

std::optional<HmdReport> parseHmdReport(const uint8_t *r, size_t size, const ImuConfig &config,
                                        const ImuCalibration &calibration) {
  // {id, u16, timestamp u32, three {marker, accel 3 x i16, gyro 3 x i16, temperature i16}, ...}: 64 bytes
  if (size != 64 || r[0] != kInHmd) return std::nullopt;
  HmdReport report;
  report.timestamp_us = le32(r + 3);
  const float gyro_scale = kDegrees / config.gyro_scale;
  const float accel_scale = kGravity / config.accel_scale;
  for (int i = 0; i < 3; i++) {
    const uint8_t *s = r + 7 + 15 * i;
    if (s[0] & 0x80) break; // this sample and the ones after it are not valid
    float accel[3], gyro[3];
    for (int a = 0; a < 3; a++) {
      accel[a] = accel_scale * static_cast<int16_t>(le16(s + 1 + 2 * a)) - calibration.accel_offset[a];
      gyro[a] = gyro_scale * static_cast<int16_t>(le16(s + 7 + 2 * a)) - calibration.gyro_offset[a];
    }
    float rectified_accel[3], rectified_gyro[3];
    apply(calibration.accel_rectification, accel, rectified_accel);
    apply(calibration.gyro_rectification, gyro, rectified_gyro);
    ImuSample &out = report.samples[report.count++];
    apply(calibration.device_from_imu, rectified_accel, out.accel);
    apply(calibration.device_from_imu, rectified_gyro, out.gyro);
  }
  return report;
}

std::optional<bool> parseStatus(const uint8_t *r, size_t size) {
  if (size < 2 || r[0] != kInStatus) return std::nullopt;
  return r[1] != 0;
}

std::vector<std::vector<uint8_t>> enableReports(bool on) {
  std::vector<std::vector<uint8_t>> reports;
  if (on) reports.push_back({0x07, 0xa3, 0x01});            // unknown, sent by Oculus's runtime first
  reports.push_back({0x14, static_cast<uint8_t>(on ? 1 : 0)}); // unknown
  reports.push_back({0x0a, static_cast<uint8_t>(on ? 2 : 0)}); // the radio to the controllers
  if (!on) reports.push_back(screenReport(false));
  reports.push_back({0x02, static_cast<uint8_t>(on ? 1 : 0)}); // the proximity sensor and the IMU
  reports.push_back(cameraReport(on));
  return reports;
}

std::vector<uint8_t> cameraReport(bool on, uint16_t exposure, uint8_t gain, bool radio_sync) {
  // {id, uvc enable, radio sync, exposure u16 x 5, gain x 5, 26 00 40, u32 x 5} (OpenHMD's rift_s_send_camera_report)
  std::vector<uint8_t> camera = {0x05, 0x01, 0x00, 0xb3, 0x36, 0xb3, 0x36, 0xb3, 0x36, 0xb3, 0x36, 0xb3, 0x36, 0xf0,
                                 0xf0, 0xf0, 0xf0, 0xf0, 0x26, 0x00, 0x40, 0x7a, 0x04, 0x00, 0x00, 0xa7, 0x04, 0x00,
                                 0x00, 0xa7, 0x04, 0x00, 0x00, 0xa5, 0x04, 0x00, 0x00, 0xa8, 0x04, 0x00, 0x00};
  camera[1] = on ? 1 : 0;
  camera[2] = radio_sync ? 1 : 0;
  for (int i = 0; i < kCameraCount; i++) {
    camera[3 + 2 * i] = static_cast<uint8_t>(exposure);
    camera[4 + 2 * i] = static_cast<uint8_t>(exposure >> 8);
    camera[13 + i] = gain;
  }
  return camera;
}

std::vector<uint8_t> keepAliveReport() {
  // report 0x93: a 3000 ms timeout, renewed every second
  return {0x93, 0x01, 0xb8, 0x0b, 0x00, 0x00};
}

std::vector<uint8_t> screenReport(bool on) { return {0x08, static_cast<uint8_t>(on ? 1 : 0)}; }

std::array<uint8_t, 64> firmwareRequest(uint8_t block, uint32_t offset, uint8_t length) {
  std::array<uint8_t, 64> r{};
  r[0] = kReportFirmwareBlock;
  r[2] = block;
  r[3] = static_cast<uint8_t>(offset);
  r[4] = static_cast<uint8_t>(offset >> 8);
  r[5] = static_cast<uint8_t>(offset >> 16);
  r[6] = static_cast<uint8_t>(offset >> 24);
  r[7] = length;
  return r;
}

bool firmwareReplyMatches(const uint8_t *reply, size_t size, const std::array<uint8_t, 64> &request) {
  return size >= 8 + static_cast<size_t>(request[7]) && std::memcmp(reply, request.data(), 7) == 0;
}

uint32_t firmwareBlockLength(const uint8_t *reply, size_t size) {
  // the header: 8 bytes of checksum, then the length
  if (size < 20) return 0;
  const uint32_t length = le32(reply + 16);
  if (length < 0xC || length == 0xFFFFFFFFu || length > 1u << 20) return 0;
  return length;
}

std::vector<Device> parseDevices(const uint8_t *r, size_t size) {
  std::vector<Device> devices;
  if (size < 3 || r[0] != kReportDevices) return devices;
  size_t count = (size - 3) / 28;
  if (count > r[2]) count = r[2];
  for (size_t i = 0; i < count && i < 7; i++) {
    const uint8_t *d = r + 3 + 28 * i;
    devices.push_back({le64(d), le32(d + 8)});
  }
  return devices;
}

uint64_t parseControllerReport(const uint8_t *r, size_t size, ControllerState &state) {
  // {id, device u64, data length, flags, log 3, blocks...}
  if (size < 10 || r[0] != kInController) return 0;
  const uint64_t device = le64(r + 1);
  size_t avail = r[9];
  if (device == 0 || avail < 4) return device;
  if (avail > size - 10) avail = size - 10;
  const uint8_t *p = r + 14;
  avail -= 4;
  while (avail > 1) {
    size_t length = 0;
    switch (p[0]) {
    case 0x08: case 0x0c: case 0x0d: case 0x0e: length = 2; break; // one byte
    case 0x1b: length = 4; break;                                   // trigger and grip
    case 0x22: length = 5; break;                                   // stick
    case 0x27: length = 5; break;                                   // capacitive sensors
    case 0x91: length = 19; break;                                  // IMU
    default: break;
    }
    if (length == 0 || avail < length) break;
    switch (p[0]) {
    case 0x0c:
      state.buttons = p[1];
      break;
    case 0x0d:
      state.fingers = p[1];
      break;
    case 0x1b:
      state.trigger = static_cast<uint16_t>((p[2] & 0x0f) << 8 | p[1]);
      state.grip = static_cast<uint16_t>((p[2] & 0xf0) >> 4 | (p[3] << 4));
      break;
    case 0x22:
      state.stick_x = static_cast<int16_t>(le16(p + 1));
      state.stick_y = static_cast<int16_t>(le16(p + 3));
      break;
    case 0x91:
      // {id, timestamp u32, u16, accel 3 x i16, gyro 3 x i16}
      state.has_imu = true;
      state.imu_timestamp = le32(p + 1);
      for (int a = 0; a < 3; a++) {
        state.accel[a] = static_cast<int16_t>(le16(p + 7 + 2 * a));
        state.gyro[a] = static_cast<int16_t>(le16(p + 13 + 2 * a));
      }
      break;
    default:
      break;
    }
    p += length;
    avail -= length;
  }
  return device;
}

TouchValues touchValues(const ControllerState &s) {
  constexpr uint8_t kAxStrong = 0x01, kByStrong = 0x02, kStickStrong = 0x04, kTriggerStrong = 0x08;
  constexpr uint8_t kAxWeak = 0x10, kByWeak = 0x20, kStickWeak = 0x40, kTriggerWeak = 0x80;
  TouchValues v;
  v.primary = s.buttons & 0x01;
  v.secondary = s.buttons & 0x02;
  v.stick = s.buttons & 0x04;
  v.menu = s.buttons & 0x08;
  const uint8_t f = s.fingers;
  v.primary_touch = (f & kAxStrong) || ((f & kAxWeak) && !(f & (kByStrong | kStickStrong)));
  v.secondary_touch = (f & kByStrong) || ((f & kByWeak) && !(f & (kAxStrong | kStickStrong)));
  v.stick_touch = (f & kStickStrong) || ((f & kStickWeak) && !(f & (kAxStrong | kByStrong)));
  v.trigger_touch = f & (kTriggerWeak | kTriggerStrong);
  v.trigger = 1.0f - s.trigger / 4096.0f;
  v.grip = 1.0f - s.grip / 4096.0f;
  if (s.trigger == 0 && s.grip == 0 && s.buttons == 0 && f == 0) v.trigger = v.grip = 0; // nothing reported yet
  v.stick_x = s.stick_x / 32768.0f;
  v.stick_y = s.stick_y / 32768.0f;
  return v;
}

std::array<uint8_t, 64> radioCommand(uint64_t device, const uint8_t *bytes, size_t size) {
  // {0x12, device u64, command 52}
  std::array<uint8_t, 64> r{};
  r[0] = kReportRadioCommand;
  for (int i = 0; i < 8; i++) r[1 + i] = static_cast<uint8_t>(device >> (8 * i));
  std::memcpy(r.data() + 9, bytes, size < 52 ? size : 52);
  return r;
}

std::optional<ControllerImuConfig> parseControllerImuConfig(const uint8_t *bytes, size_t size) {
  // {u32, length, accel limit u16, gyro limit u16, accel hz u16, gyro hz u16, accel scale f32, gyro scale f32}
  if (size < 5 + 16 || bytes[4] < 16) return std::nullopt;
  ControllerImuConfig c;
  c.accel_scale = lef32(bytes + 5 + 8);
  c.gyro_scale = lef32(bytes + 5 + 12);
  if (!(c.accel_scale > 0) || !(c.gyro_scale > 0) || !std::isfinite(c.accel_scale) || !std::isfinite(c.gyro_scale))
    return std::nullopt;
  return c;
}

void controllerImu(const ControllerState &s, const ControllerImuConfig &config, float accel[3], float gyro[3]) {
  for (int a = 0; a < 3; a++) {
    accel[a] = kGravity * config.accel_scale * s.accel[a];
    gyro[a] = kDegrees * config.gyro_scale * s.gyro[a];
  }
}

namespace {

constexpr double kPanelWidth = 0.149760 / 2.0; // one eye's, in the view
constexpr double kPanelHeight = 0.093600;
constexpr double kLensSeparation = 0.074;
constexpr double kHorizontalFov = 105.0 * 3.14159265358979323846 / 180.0;

// Monado's math_solve_triangle: a width split at w1, seen across theta_total from a point: the angles to either end
bool solveTriangle(double w_total, double w_1, double theta_total, double &theta_1, double &theta_2, double &d) {
  const double w_2 = w_total - w_1;
  const double u = w_2 / w_1;
  const double v = std::tan(theta_total);
  const double b = u + 1.0;
  const double root = std::sqrt(b * b + 4 * u * v * v);
  const double plus = std::atan((-b + root) / (2 * v)), minus = std::atan((-b - root) / (2 * v));
  if (plus > 0 && plus < theta_total)
    theta_2 = plus;
  else if (minus > 0 && minus < theta_total)
    theta_2 = minus;
  else
    return false;
  theta_1 = theta_total - theta_2;
  d = w_2 / std::tan(theta_2);
  return true;
}

} // namespace

Panotools panotools(int eye) {
  Panotools p;
  const double center_x = kLensSeparation / 2.0;
  p.scale = static_cast<float>(kPanelWidth - kLensSeparation / 2.0);
  // the left eye's lens is the right eye's moved the other way
  p.lens_center[0] = static_cast<float>(eye == 0 ? kPanelWidth - center_x : center_x);
  p.lens_center[1] = static_cast<float>(kPanelHeight / 2.0);
  p.viewport[0] = static_cast<float>(kPanelWidth);
  p.viewport[1] = static_cast<float>(kPanelHeight);
  return p;
}

void fieldOfView(int eye, float &left, float &right, float &up, float &down) {
  const double center_x = kLensSeparation / 2.0, center_y = kPanelHeight / 2.0;
  double theta_1 = 0, theta_2 = 0, d = 1;
  // Monado's: the right eye's horizontal center of projection is center_x, the left eye's the panel's width less it
  solveTriangle(kPanelWidth, eye == 0 ? kPanelWidth - center_x : center_x, kHorizontalFov, theta_1, theta_2, d);
  left = static_cast<float>(-theta_1);
  right = static_cast<float>(theta_2);
  down = static_cast<float>(-std::atan(center_y / d));
  up = static_cast<float>(std::atan((kPanelHeight - center_y) / d));
}

void computeDistortion(const Panotools &p, float u, float v, float out[3][2]) {
  const float rx = (u * p.viewport[0] - p.lens_center[0]) / p.scale;
  const float ry = (v * p.viewport[1] - p.lens_center[1]) / p.scale;
  const float r = std::sqrt(rx * rx + ry * ry);
  const float k = p.distortion_k[0] + p.distortion_k[1] * r + p.distortion_k[2] * r * r +
                  p.distortion_k[3] * r * r * r + p.distortion_k[4] * r * r * r * r;
  for (int c = 0; c < 3; c++) {
    const float dx = rx * k * p.scale * p.aberration_k[c], dy = ry * k * p.scale * p.aberration_k[c];
    out[c][0] = (dx + p.lens_center[0]) / p.viewport[0];
    out[c][1] = (dy + p.lens_center[1]) / p.viewport[1];
  }
}

std::optional<std::array<CameraCalibration, kCameraCount>> parseCameraCalibration(std::string_view text) {
  const auto root = JsonValue::parse(text);
  if (!root || !root->isObject()) return std::nullopt;
  const JsonValue &version = (*root)["FileFormat"]["Version"];
  if (!version.isString() || std::strtof(version.string().c_str(), nullptr) != 1.0f) return std::nullopt;
  const JsonValue &cameras = (*root)["CameraCalibration"];
  if (!cameras.isArray() || cameras.size() != kCameraCount) return std::nullopt;
  std::array<CameraCalibration, kCameraCount> out;
  for (int i = 0; i < kCameraCount; i++) {
    const JsonValue &camera = cameras[static_cast<size_t>(i)];
    CameraCalibration &c = out[i];
    if (!camera["Id"].isString() || std::strtol(camera["Id"].string().c_str(), nullptr, 10) != i) return std::nullopt;
    double size[2], projection[3], distortion[8];
    if (!camera["ImageSize"].numbers(size, 2) || !camera["DeviceFromCamera"].numbers(c.device_from_camera, 16) ||
        camera["Projection"]["Model"].string() != "PinholeSymmetric" ||
        !camera["Projection"]["Coefficients"].numbers(projection, 3) ||
        camera["Distortion"]["Model"].string() != "Fisheye62" ||
        !camera["Distortion"]["Coefficients"].numbers(distortion, 8))
      return std::nullopt;
    if (!(size[0] > 0 && size[0] <= 640 && size[1] > 0 && size[1] <= 480) || !(projection[0] > 0)) return std::nullopt;
    c.width = static_cast<uint32_t>(size[0]);
    c.height = static_cast<uint32_t>(size[1]);
    c.x_offset = static_cast<uint32_t>(i) * c.width; // the pictures sit side by side in their ids' order
    c.f = projection[0];
    c.cx = projection[1];
    c.cy = projection[2];
    std::memcpy(c.k, distortion, sizeof(c.k));
    c.p1 = distortion[6];
    c.p2 = distortion[7];
  }
  return out;
}

void fisheye62Project(const CameraCalibration &c, const double point[2], double pixel[2]) {
  // Monado's fisheye62_undistort_func: a point on the plane z = 1 to its pixel
  const double x = point[0], y = point[1];
  const double r = std::sqrt(x * x + y * y);
  if (r < 1e-8) {
    pixel[0] = c.f * x + c.cx;
    pixel[1] = c.f * y + c.cy;
    return;
  }
  const double theta = std::atan(r), theta2 = theta * theta;
  const double xp = x * theta / r, yp = y * theta / r;
  double radial = c.k[5];
  for (int i = 4; i >= 0; i--) radial = radial * theta2 + c.k[i];
  radial = radial * theta2 + 1;
  const double dx = 2 * c.p1 * xp * yp + c.p2 * (theta2 + 2 * xp * xp);
  const double dy = 2 * c.p2 * xp * yp + c.p1 * (theta2 + 2 * yp * yp);
  pixel[0] = c.f * (xp * radial + dx) + c.cx;
  pixel[1] = c.f * (yp * radial + dy) + c.cy;
}

void kb4Project(const CameraCalibration &c, const std::array<double, 4> &k, const double point[2], double pixel[2]) {
  const double x = point[0], y = point[1];
  const double r = std::sqrt(x * x + y * y);
  if (r < 1e-8) {
    pixel[0] = c.f * x + c.cx;
    pixel[1] = c.f * y + c.cy;
    return;
  }
  const double theta = std::atan(r), theta2 = theta * theta;
  const double d = theta * (1 + theta2 * (k[0] + theta2 * (k[1] + theta2 * (k[2] + theta2 * k[3]))));
  pixel[0] = c.f * x * d / r + c.cx;
  pixel[1] = c.f * y * d / r + c.cy;
}

// Newton's method from the pinhole guess
bool fisheye62Unproject(const CameraCalibration &c, const double pixel[2], double point[2]) {
  point[0] = (pixel[0] - c.cx) / c.f;
  point[1] = (pixel[1] - c.cy) / c.f;
  for (int iteration = 0; iteration < 50; iteration++) {
    double at[2];
    fisheye62Project(c, point, at);
    const double ex = at[0] - pixel[0], ey = at[1] - pixel[1];
    if (ex * ex + ey * ey < 1e-14) return true;
    const double h = 1e-7;
    double ax[2], ay[2];
    const double px[2] = {point[0] + h, point[1]}, py[2] = {point[0], point[1] + h};
    fisheye62Project(c, px, ax);
    fisheye62Project(c, py, ay);
    const double j00 = (ax[0] - at[0]) / h, j10 = (ax[1] - at[1]) / h;
    const double j01 = (ay[0] - at[0]) / h, j11 = (ay[1] - at[1]) / h;
    const double det = j00 * j11 - j01 * j10;
    if (!(std::fabs(det) > 1e-18)) return false;
    point[0] -= (j11 * ex - j01 * ey) / det;
    point[1] -= (j00 * ey - j10 * ex) / det;
  }
  double at[2];
  fisheye62Project(c, point, at);
  return std::hypot(at[0] - pixel[0], at[1] - pixel[1]) < 1e-3;
}

std::array<double, 4> kb4FromFisheye62(const CameraCalibration &c) {
  // KB4 puts (x, y) at f (x, y) / r theta (1 + k1 theta^2 + .. + k4 theta^8) + c: linear in k, so the fit over the
  // grid is a 4x4 least-squares solve
  constexpr int kSteps = 21;
  double ata[4][4] = {}, atb[4] = {};
  for (int yi = 0; yi < kSteps; yi++)
    for (int xi = 0; xi < kSteps; xi++) {
      const double pixel[2] = {static_cast<double>(xi * (static_cast<int>(c.width) - 1) / (kSteps - 1)),
                               static_cast<double>(yi * (static_cast<int>(c.height) - 1) / (kSteps - 1))};
      double point[2];
      if (!fisheye62Unproject(c, pixel, point)) continue;
      const double r = std::sqrt(point[0] * point[0] + point[1] * point[1]);
      if (r < 1e-8) continue;
      const double theta = std::atan(r), theta2 = theta * theta;
      for (int axis = 0; axis < 2; axis++) {
        const double base = c.f * point[axis] / r * theta; // the pixel's offset with k = 0
        const double target = pixel[axis] - (axis ? c.cy : c.cx) - base;
        double row[4];
        double power = theta2;
        for (int i = 0; i < 4; i++, power *= theta2) row[i] = base * power;
        for (int i = 0; i < 4; i++) {
          atb[i] += row[i] * target;
          for (int j = 0; j < 4; j++) ata[i][j] += row[i] * row[j];
        }
      }
    }
  // Gaussian elimination with partial pivoting
  double m[4][5];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) m[i][j] = ata[i][j];
    m[i][4] = atb[i];
  }
  for (int col = 0; col < 4; col++) {
    int pivot = col;
    for (int r = col + 1; r < 4; r++)
      if (std::fabs(m[r][col]) > std::fabs(m[pivot][col])) pivot = r;
    if (!(std::fabs(m[pivot][col]) > 1e-30)) return {0, 0, 0, 0};
    for (int j = 0; j < 5; j++) std::swap(m[col][j], m[pivot][j]);
    for (int r = 0; r < 4; r++) {
      if (r == col) continue;
      const double factor = m[r][col] / m[col][col];
      for (int j = col; j < 5; j++) m[r][j] -= factor * m[col][j];
    }
  }
  return {m[0][4] / m[0][0], m[1][4] / m[1][1], m[2][4] / m[2][2], m[3][4] / m[3][3]};
}

std::optional<CameraFrameInfo> parseCameraFrameInfo(const uint8_t *grey, uint32_t width, uint32_t height) {
  // Monado's parse_frame_data: 50 bytes, a bit per 8x8 block, read across the middle (row 4) of the top blocks
  if (width != kCameraFrameWidth || height < 8 || !grey) return std::nullopt;
  const uint8_t *row = grey + width * 4;
  uint8_t raw[50] = {};
  for (uint32_t block = 0; block < 400; block++)
    if (row[block * 8 + 4] > 128) raw[block / 8] |= static_cast<uint8_t>(0x80 >> (block % 8));
  // {type, abcd, counter u16, QHWH u32, 7 zero, time u64 us, pair counter u32, exposure u16 x 5, 2 zero, gain x 5,
  //  zero, u16, face}
  if (le16(raw + 1) != 0xabcd || le16(raw + 48) != 0xface) return std::nullopt;
  CameraFrameInfo info;
  info.tracking = (raw[0] & 0x80) != 0;
  info.counter = le16(raw + 3);
  info.time_us = le64(raw + 16);
  for (int i = 0; i < kCameraCount; i++) {
    info.exposure[i] = le16(raw + 28 + 2 * i);
    info.gain[i] = raw[40 + i];
  }
  return info;
}

bool splitTrackingFrame(const uint8_t *grey, uint32_t width, uint32_t height, size_t stride,
                        const CameraFrameInfo &info, const std::array<CameraCalibration, kCameraCount> &calibration,
                        std::array<std::vector<uint8_t>, kCameraCount> &pictures) {
  for (int i = 0; i < kCameraCount; i++) {
    const CameraId id = kTrackerCameraOrder[i];
    const CameraCalibration &c = calibration[id];
    const uint32_t top = cameraYOffset(info.exposure[id]);
    if (c.x_offset + c.width > width || top + c.height > height) return false;
    std::vector<uint8_t> &out = pictures[i];
    out.resize(static_cast<size_t>(c.width) * c.height);
    for (uint32_t y = 0; y < c.height; y++)
      std::memcpy(out.data() + static_cast<size_t>(y) * c.width, grey + (top + y) * stride + c.x_offset, c.width);
  }
  return true;
}

uint32_t cameraYOffset(uint16_t exposure) {
  const int offset = (static_cast<int>(exposure) + 275) / 38;
  return static_cast<uint32_t>(offset < 8 ? 8 : offset > 375 ? 375 : offset);
}

} // namespace metalvr::rift_s
