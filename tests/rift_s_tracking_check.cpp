// The Rift S's camera tracking end to end, without a headset (docs/HEADSETS.md, phase I): a synthetic Rift S moves
// through a textured room along a known path. Its five Fisheye62 cameras' pictures are rendered and packed into the
// camera device's frames as the headset sends them (the 8x8-block header row, each picture lowered by its exposure,
// controller frames between the tracking ones), and its IMU's reports are made from the same path, with the IMU's
// 32-bit clock wrapping part way. All of it goes through RiftSTracking (the backend's path) into Basalt, and the
// path Basalt reports, in OpenXR's axes, is compared with the true one.
//
//   rift_s_tracking_check [seconds]      (needs scripts/build-basalt.sh)
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "rift_s_protocol.h"
#include "rift_s_tracking.h"

using namespace metalvr;
using namespace metalvr::rift_s;

namespace {

struct Vec {
  double x = 0, y = 0, z = 0;
};
Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec operator*(double s, Vec a) { return {s * a.x, s * a.y, s * a.z}; }
double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec normalize(Vec a) { return (1.0 / std::sqrt(dot(a, a))) * a; }

struct Mat {
  double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  Vec operator*(Vec v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }
  Mat operator*(const Mat &o) const {
    Mat r;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
    return r;
  }
  Mat t() const {
    Mat r;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) r.m[i][j] = m[j][i];
    return r;
  }
};
Mat rotX(double a) {
  Mat r;
  r.m[1][1] = r.m[2][2] = std::cos(a);
  r.m[1][2] = -std::sin(a);
  r.m[2][1] = std::sin(a);
  return r;
}
Mat rotY(double a) {
  Mat r;
  r.m[0][0] = r.m[2][2] = std::cos(a);
  r.m[0][2] = std::sin(a);
  r.m[2][0] = -std::sin(a);
  return r;
}
Mat rotZ(double a) {
  Mat r;
  r.m[0][0] = r.m[1][1] = std::cos(a);
  r.m[0][1] = -std::sin(a);
  r.m[1][0] = std::sin(a);
  return r;
}
Mat fromQuat(double x, double y, double z, double w) {
  Mat r;
  r.m[0][0] = 1 - 2 * (y * y + z * z), r.m[0][1] = 2 * (x * y - z * w), r.m[0][2] = 2 * (x * z + y * w);
  r.m[1][0] = 2 * (x * y + z * w), r.m[1][1] = 1 - 2 * (x * x + z * z), r.m[1][2] = 2 * (y * z - x * w);
  r.m[2][0] = 2 * (x * z - y * w), r.m[2][1] = 2 * (y * z + x * w), r.m[2][2] = 1 - 2 * (x * x + y * y);
  return r;
}
// the angle of a rotation matrix
double angleOf(const Mat &r) {
  return std::acos(std::clamp((r.m[0][0] + r.m[1][1] + r.m[2][2] - 1) / 2, -1.0, 1.0));
}

// ---- the path: at rest for a second, then moving about in all six axes ----

double ramp(double t) {
  const double s = std::clamp((t - 1.0) / 2.0, 0.0, 1.0);
  return s * s * (3 - 2 * s);
}
Vec truePosition(double t) {
  const double r = ramp(t);
  return {r * 0.45 * std::sin(0.55 * t), 1.6 + r * 0.12 * std::sin(0.8 * t + 0.3), r * 0.4 * std::sin(0.37 * t + 1.0)};
}
Mat trueRotation(double t) {
  const double r = ramp(t);
  return rotY(r * 0.7 * std::sin(0.31 * t)) * rotX(r * 0.25 * std::sin(0.47 * t)) * rotZ(r * 0.12 * std::sin(0.6 * t));
}

// ---- the room: a box textured with smooth noise at two scales ----

double hash(int a, int b, int c) {
  uint32_t h = static_cast<uint32_t>(a) * 374761393u + static_cast<uint32_t>(b) * 668265263u +
               static_cast<uint32_t>(c) * 2246822519u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return ((h ^ (h >> 16)) & 0xffff) / 65535.0;
}
double noise(double u, double v, double scale, int seed) {
  const double x = u / scale, y = v / scale;
  const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
  double fx = x - ix, fy = y - iy;
  fx = fx * fx * (3 - 2 * fx);
  fy = fy * fy * (3 - 2 * fy);
  const double a = hash(ix, iy, seed), b = hash(ix + 1, iy, seed), c = hash(ix, iy + 1, seed),
               d = hash(ix + 1, iy + 1, seed);
  return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
}
const Vec kRoomMin{-3.5, 0.0, -3.5}, kRoomMax{3.5, 2.8, 3.5};
uint8_t shade(Vec origin, Vec direction) {
  double t = 1e9;
  int face = 0;
  const double o[3] = {origin.x, origin.y, origin.z}, d[3] = {direction.x, direction.y, direction.z};
  const double lo[3] = {kRoomMin.x, kRoomMin.y, kRoomMin.z}, hi[3] = {kRoomMax.x, kRoomMax.y, kRoomMax.z};
  for (int a = 0; a < 3; a++) {
    if (std::fabs(d[a]) < 1e-12) continue;
    const double ta = ((d[a] > 0 ? hi[a] : lo[a]) - o[a]) / d[a];
    if (ta > 0 && ta < t) t = ta, face = a * 2 + (d[a] > 0);
  }
  const Vec p = origin + t * direction;
  double u, v;
  if (face / 2 == 0) u = p.z, v = p.y;
  else if (face / 2 == 1) u = p.x, v = p.z;
  else u = p.x, v = p.y;
  const double value = 0.5 * noise(u, v, 0.21, face) + 0.35 * noise(u, v, 0.07, face + 10) +
                       0.15 * noise(u, v, 0.6, face + 20);
  return static_cast<uint8_t>(std::clamp(20.0 + 230.0 * (value - 0.15) / 0.7, 0.0, 255.0));
}

// ---- the headset: five cameras, as a Rift S's firmware would describe them ----

struct Placement {
  Vec position;
  double yaw, pitch; // where it looks: yaw left of straight ahead, pitch up
};
// in the headset's frame (+x right, +y up, +z back), by camera id: top, side left, front right, front left, side right
const Placement kPlacements[kCameraCount] = {
    {{0.0, 0.05, -0.06}, 0.0, 0.75},  {{-0.09, 0.0, -0.03}, 1.25, -0.35}, {{0.035, 0.0, -0.08}, -0.15, -0.2},
    {{-0.035, 0.0, -0.08}, 0.15, -0.2}, {{0.09, 0.0, -0.03}, -1.25, -0.35}};

std::string calibrationJson(Mat device_from_camera[kCameraCount]) {
  std::string json = "{\"FileFormat\":{\"Version\":\"1.0\"},\"CameraCalibration\":[";
  for (int i = 0; i < kCameraCount; i++) {
    const Placement &p = kPlacements[i];
    // the camera looks down its +z with +y down (OpenCV's camera); turned to look along `yaw` and `pitch`
    Mat look;
    look.m[0][0] = 1, look.m[1][1] = -1, look.m[2][2] = -1; // camera (x, y, z) -> headset (x, -y, -z): ahead
    const Mat r = rotY(p.yaw) * rotX(p.pitch) * look;
    device_from_camera[i] = r;
    char camera[1024];
    std::snprintf(camera, sizeof(camera),
                  "%s{\"Id\":\"%d\",\"ImageSize\":[640,480],\"DeviceFromCamera\":[%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                  "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,0,0,0,1],\"Projection\":{\"Model\":\"PinholeSymmetric\","
                  "\"Coefficients\":[%.3f,%.3f,%.3f]},\"Distortion\":{\"Model\":\"Fisheye62\",\"Coefficients\":"
                  "[-0.028,0.011,-0.006,0.0013,0,0,0.00042,-0.00031]}}",
                  i ? "," : "", i, r.m[0][0], r.m[0][1], r.m[0][2], p.position.x, r.m[1][0], r.m[1][1], r.m[1][2],
                  p.position.y, r.m[2][0], r.m[2][1], r.m[2][2], p.position.z, 290.0 + i, 321.5 - i, 239.0 + i * 0.5);
    json += camera;
  }
  return json + "]}";
}

std::string imuJson(const float imu_position[3]) {
  char json[1024];
  std::snprintf(json, sizeof(json),
                "{\"FileFormat\":{\"Version\":\"1.0\"},\"ImuCalibration\":{\"DeviceFromImu\":[1,0,0,%f,0,1,0,%f,0,0,"
                "1,%f,0,0,0,1],\"Gyroscope\":{\"RectificationMatrix\":[1,0,0,0,1,0,0,0,1],\"Offset\":"
                "{\"ConstantOffset\":[0,0,0]}},\"Accelerometer\":{\"RectificationMatrix\":[1,0,0,0,1,0,0,0,1],"
                "\"Offset\":{\"OffsetAtZeroDegC\":[0,0,0]}}}}",
                imu_position[0], imu_position[1], imu_position[2]);
  return json;
}

// a camera device frame: the header row's 50 bytes as 8x8 blocks, then each camera's picture where it belongs
void packFrame(std::vector<uint8_t> &frame, uint32_t height, bool tracking, uint16_t counter, uint64_t time_us,
               uint16_t exposure, const std::array<std::vector<uint8_t>, kCameraCount> *pictures,
               const std::array<CameraCalibration, kCameraCount> &calibration) {
  frame.assign(static_cast<size_t>(kCameraFrameWidth) * height, 0);
  uint8_t raw[50] = {};
  raw[0] = tracking ? 0x86 : 0x06;
  raw[1] = 0xcd, raw[2] = 0xab;
  raw[3] = static_cast<uint8_t>(counter), raw[4] = static_cast<uint8_t>(counter >> 8);
  for (int i = 0; i < 8; i++) raw[16 + i] = static_cast<uint8_t>(time_us >> (8 * i));
  for (int c = 0; c < kCameraCount; c++) {
    raw[28 + 2 * c] = static_cast<uint8_t>(exposure);
    raw[29 + 2 * c] = static_cast<uint8_t>(exposure >> 8);
    raw[40 + c] = 0xf0;
  }
  raw[48] = 0xce, raw[49] = 0xfa;
  for (int block = 0; block < 400; block++)
    if (raw[block / 8] & (0x80 >> (block % 8)))
      for (int y = 0; y < 8; y++) std::memset(&frame[y * kCameraFrameWidth + block * 8], 255, 8);
  if (!pictures) return;
  const uint32_t top = cameraYOffset(exposure);
  for (int id = 0; id < kCameraCount; id++) {
    const CameraCalibration &c = calibration[id];
    const std::vector<uint8_t> &picture = (*pictures)[id];
    for (uint32_t y = 0; y < c.height; y++)
      std::memcpy(&frame[(top + y) * kCameraFrameWidth + c.x_offset], &picture[y * c.width], c.width);
  }
}

} // namespace

int main(int argc, char **argv) {
  const double seconds = argc > 1 ? std::atof(argv[1]) : 24.0;
  int failures = 0;
  auto check = [&](bool ok, const char *what) {
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) failures++;
  };

  // the headset's description, as its firmware would hand it over, through the parsers the backend uses
  const float imu_position[3] = {0.012f, -0.008f, -0.045f};
  Mat device_from_camera[kCameraCount];
  const auto calibration = parseCameraCalibration(calibrationJson(device_from_camera));
  const auto imu_calibration = parseImuCalibration(imuJson(imu_position));
  check(calibration.has_value() && imu_calibration.has_value(), "the synthetic firmware's calibration parses");
  if (!calibration || !imu_calibration) return 1;
  ImuConfig config;
  config.hz = 1000;

  std::string why;
  auto tracking = RiftSTracking::start(config, *imu_calibration, *calibration, &why);
  if (!tracking) {
    std::printf("FAIL  Basalt did not start: %s\n", why.c_str());
    return 1;
  }

  // each camera's rays, once: the true lens (Fisheye62) undone at every pixel
  std::array<std::vector<Vec>, kCameraCount> rays;
  for (int id = 0; id < kCameraCount; id++) {
    const CameraCalibration &c = (*calibration)[id];
    rays[id].resize(static_cast<size_t>(c.width) * c.height);
    for (uint32_t y = 0; y < c.height; y++)
      for (uint32_t x = 0; x < c.width; x++) {
        double point[2] = {0, 0};
        const double pixel[2] = {static_cast<double>(x), static_cast<double>(y)};
        fisheye62Unproject(c, pixel, point);
        rays[id][y * c.width + x] = device_from_camera[id] * normalize(Vec{point[0], point[1], 1.0});
      }
  }

  // the IMU's clock starts 6 s before its 32-bit wrap; the camera's 64-bit clock has other high bits
  const uint64_t imu_start_us = 0x100000000ULL - 6'000'000ULL;
  const uint64_t camera_high = 0x500000000ULL;
  const Vec gravity{0, -9.80665, 0};
  std::mt19937 random(7);
  std::normal_distribution<double> accel_noise(0, 0.02), gyro_noise(0, 0.002);
  const double h = 1e-4;
  auto specificForce = [&](double t) {
    const Vec a = (1 / (h * h)) * (truePosition(t + h) - 2.0 * truePosition(t) + truePosition(t - h));
    return trueRotation(t).t() * (a - gravity);
  };
  auto angularVelocity = [&](double t) {
    // R(t)^T R(t + h) = I + h [w]x
    const Mat d = trueRotation(t - h).t() * trueRotation(t + h);
    return (1 / (4 * h)) * Vec{d.m[2][1] - d.m[1][2], d.m[0][2] - d.m[2][0], d.m[1][0] - d.m[0][1]};
  };

  const uint32_t frame_height = cameraYOffset(6000) + 480 + 8;
  std::array<std::vector<uint8_t>, kCameraCount> pictures;
  std::vector<uint8_t> frame;
  struct Sample {
    double t;
    TrackedPose pose;
  };
  std::vector<Sample> estimates;
  int64_t last_pose_ns = 0;
  const int imu_steps = static_cast<int>(seconds * 1000);
  uint16_t counter = 0;
  int skipped_controller_frames = 0;
  const auto started = std::chrono::steady_clock::now();
  for (int step = 0; step < imu_steps; step += 3) {
    // an IMU report: three samples a millisecond apart, the first's time in it
    HmdReport report;
    report.timestamp_us = static_cast<uint32_t>(imu_start_us + static_cast<uint64_t>(step) * 1000);
    report.count = 3;
    for (int i = 0; i < 3; i++) {
      const double t = (step + i) / 1000.0;
      const Vec f = specificForce(t), w = angularVelocity(t);
      ImuSample &s = report.samples[i];
      s.accel[0] = static_cast<float>(f.x + accel_noise(random));
      s.accel[1] = static_cast<float>(f.y + accel_noise(random));
      s.accel[2] = static_cast<float>(f.z + accel_noise(random));
      s.gyro[0] = static_cast<float>(w.x + gyro_noise(random));
      s.gyro[1] = static_cast<float>(w.y + gyro_noise(random));
      s.gyro[2] = static_cast<float>(w.z + gyro_noise(random));
    }
    tracking->imu(report);
    // the camera device's frames at 60 Hz, tracking and controller frames in turn, once the IMU has passed them
    const int frame_index_now = (step + 2) * 60 / 1000, frame_index_before = (step - 1) * 60 / 1000;
    if (step == 0 || frame_index_now == frame_index_before) continue;
    const int index = frame_index_now;
    const double t = index / 60.0;
    const bool is_tracking = index % 2 == 0;
    const uint64_t time_us = camera_high + ((imu_start_us + static_cast<uint64_t>(t * 1e6)) & 0xffffffffULL);
    if (!is_tracking) {
      packFrame(frame, frame_height, false, counter++, time_us, 6000, nullptr, *calibration);
      if (!tracking->frame(frame.data(), kCameraFrameWidth, frame_height, kCameraFrameWidth)) skipped_controller_frames++;
      continue;
    }
    // render the five pictures from the true pose at this time
    const Mat world_from_device = trueRotation(t);
    const Vec device_position = truePosition(t);
    std::vector<std::thread> workers;
    for (int id = 0; id < kCameraCount; id++)
      workers.emplace_back([&, id] {
        const CameraCalibration &c = (*calibration)[id];
        pictures[id].resize(static_cast<size_t>(c.width) * c.height);
        const Vec origin = device_position + world_from_device * kPlacements[id].position;
        for (size_t p = 0; p < pictures[id].size(); p++)
          pictures[id][p] = shade(origin, world_from_device * rays[id][p]);
      });
    for (std::thread &w : workers) w.join();
    packFrame(frame, frame_height, true, counter++, time_us, 6000, &pictures, *calibration);
    tracking->frame(frame.data(), kCameraFrameWidth, frame_height, kCameraFrameWidth);
    TrackedPose pose;
    int64_t pose_ns = 0;
    if (tracking->pose(pose, &pose_ns) && pose_ns != last_pose_ns) {
      last_pose_ns = pose_ns;
      // the pose's time back to the path's: the headset's extended microseconds since the IMU's first
      estimates.push_back({(pose_ns / 1000 - static_cast<int64_t>(imu_start_us)) / 1e6, pose});
    }
  }
  // the last poses, as Basalt finishes
  for (int wait = 0; wait < 40; wait++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    TrackedPose pose;
    int64_t pose_ns = 0;
    if (tracking->pose(pose, &pose_ns) && pose_ns != last_pose_ns) {
      last_pose_ns = pose_ns;
      estimates.push_back({(pose_ns / 1000 - static_cast<int64_t>(imu_start_us)) / 1e6, pose});
      wait = 0;
    }
  }
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  std::printf("      %.0f s of headset time in %.1f s; %llu tracking frames given, %zu poses back\n", seconds, elapsed,
              static_cast<unsigned long long>(tracking->framesTracked()), estimates.size());
  check(skipped_controller_frames > 0 && tracking->framesTracked() > 0,
        "controller frames are skipped and tracking frames are given to the tracker");
  check(estimates.size() > tracking->framesTracked() / 2, "Basalt reports a pose for most tracking frames");
  if (estimates.size() < 10) return 1;

  // The tracker's room has its own origin and heading: align it to the true one by the first pose, by heading only
  // (both have +y up, which this checks), then compare the whole path.
  const Sample &first = estimates.front();
  const Mat est0 = fromQuat(first.pose.orientation.x, first.pose.orientation.y, first.pose.orientation.z,
                            first.pose.orientation.w);
  const Mat true0 = trueRotation(first.t);
  const Mat align_full = true0 * est0.t();
  const double heading = std::atan2(align_full.m[0][2] - align_full.m[2][0], align_full.m[0][0] + align_full.m[2][2]);
  const Mat align = rotY(heading);
  const Vec est_p0{first.pose.position.x, first.pose.position.y, first.pose.position.z};
  const double tilt = angleOf(align.t() * align_full);
  std::printf("      the tracker's up is %.2f degrees from the true up\n", tilt * 180 / M_PI);
  check(tilt < 2 * M_PI / 180, "the tracker's room has +y up, as OpenXR (within 2 degrees)");

  double sum_sq = 0, worst = 0, worst_angle = 0, path = 0;
  Vec previous = truePosition(first.t);
  for (const Sample &s : estimates) {
    const Vec est{s.pose.position.x, s.pose.position.y, s.pose.position.z};
    const Vec mapped = truePosition(first.t) + align * (est - est_p0);
    const Vec truth = truePosition(s.t);
    const Vec error = mapped - truth;
    const double e = std::sqrt(dot(error, error));
    sum_sq += e * e;
    worst = std::max(worst, e);
    path += std::sqrt(dot(truth - previous, truth - previous));
    previous = truth;
    const Mat r = align * fromQuat(s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z, s.pose.orientation.w);
    worst_angle = std::max(worst_angle, angleOf(r.t() * trueRotation(s.t)));
  }
  const double rms = std::sqrt(sum_sq / estimates.size());
  std::printf("      path %.2f m: position error %.1f cm RMS, %.1f cm at worst; rotation %.2f degrees at worst\n", path,
              rms * 100, worst * 100, worst_angle * 180 / M_PI);
  check(path > 2.0, "the headset moved more than 2 m");
  check(rms < 0.05, "the tracked path follows the true one (under 5 cm RMS)");
  check(worst_angle < 3 * M_PI / 180, "the tracked rotation follows the true one (under 3 degrees)");
  std::printf(failures ? "FAILED: %d\n" : "PASSED\n", failures);
  return failures ? 1 : 0;
}
