// The Oculus Rift CV1 on the Mac's own video output and USB (docs/RIFT.md).
//
// The headset's HID interface (2833:0031) turns its display, audio and LEDs on and reports its IMU at 1000 Hz; the
// head's rotation comes from those samples, and composed frames are drawn on the headset's display, which macOS
// shows as an ordinary display once its head-mounted-display override is lifted, with the CV1's lens distortion.
// With Rift sensors the headset and the Touch controllers are tracked in position too (rift_tracking/);
// without them, rotation only: the head stays where a seated user's would be.
//
// The report formats, the IMU sample packing and the lens distortion follow Monado's Rift driver
// (src/xrt/drivers/rift and src/xrt/auxiliary/util/u_distortion_mesh_rift.c; Copyright 2025-2026 Beyley Cardellio,
// Jan Schmidt and Philipp Zabel; BSL-1.0, https://gitlab.freedesktop.org/monado/monado).
#import <AppKit/AppKit.h>
#import <CoreGraphics/CoreGraphics.h>
#import <IOKit/hid/IOHIDManager.h>
#import <IOSurface/IOSurface.h>
#import <ImageIO/ImageIO.h>
#include <mach-o/dyld.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "common.h"
#include "camera_passthrough.h"
#include "direct_backend.h"
#include "driver.h"
#include "headsets.h"
#include "metalvr_headsets.h"
#include "rift_s_protocol.h"
#include "window_capture.h"
#include "metalvr_math.h"

#ifdef METALVR_RIFT_TRACKING
#include "rift_tracking.h"
#endif

namespace metalvr {
namespace {

constexpr int kVendor = 0x2833;
constexpr int kProductCV1 = 0x0031;
constexpr uint32_t kDisplayVendor = 0x3ed2; // "OVR" in the EDID
constexpr uint32_t kDisplayProductCV1 = 0x0004;

// feature reports
constexpr uint8_t kReportConfig = 2;
constexpr uint8_t kReportDisplayInfo = 9;
constexpr uint8_t kReportTrackingConfig = 12;
constexpr uint8_t kReportPositionInfo = 15;
constexpr uint8_t kReportKeepAliveMux = 17;
constexpr uint8_t kReportEnableComponents = 29;
// IN reports
constexpr uint8_t kInReportImu = 11;

constexpr uint8_t kConfigUseCalibration = 1 << 2;
constexpr uint8_t kConfigAutoCalibration = 1 << 3;
constexpr uint8_t kConfigMotionKeepAlive = 1 << 4;
constexpr uint8_t kConfigCommandKeepAlive = 1 << 5;

constexpr uint8_t kComponentDisplay = 1 << 0;
constexpr uint8_t kComponentAudio = 1 << 1;
constexpr uint8_t kComponentLeds = 1 << 2;

constexpr uint16_t kKeepAliveIntervalMs = 10000;
constexpr double kKeepAliveSendS = 9.5; // before the headset's 10 s run out
constexpr int64_t kUsbLatencyNs = 200'000;
constexpr int64_t kSampleNs = 1'000'000; // 1000 Hz

constexpr float kRefreshHz = 90.0f;
// the headset cameras' half field of view, taken as 55 degrees until their calibration is read (tan 55)
constexpr float kPassthroughHalfFovTan = 1.428f;

// Monado's CV1 distortion (rift_driver.c CV1_DISTORTIONS): the default eye relief's Catmull-Rom magnification
// curve, colour fringing correction, and lens diameter
constexpr int kCurveKnots = 11;
constexpr float kCurve[kCurveKnots] = {1.000f, 1.0312999f, 1.0698f, 1.1155f, 1.173f,     1.2460001f,
                                       1.336f, 1.457f,     1.630f,  1.900f,  2.3599999f};
constexpr float kChromatic[4] = {-0.008f, -0.005f, 0.015f, 0.005f};
constexpr float kMaxR = 1.0f;
constexpr float kMetersPerTanAngle = 0.0438f;

// the CV1's field of view, as Monado has it from Oculus's own runtime (rift_driver.c): left, right, up, down
constexpr mvr_fov kFov[2] = {{-0.76754993f, 0.6208969f, 0.7269826f, -0.8378981f},
                             {-0.6208969f, 0.76754993f, 0.7269826f, -0.8378981f}};

void unpackSample(const uint8_t *in, float scale, float out[3]) {
  const int32_t x = (in[0] << 24) | (in[1] << 16) | ((in[2] & 0xF8) << 8);
  const int32_t y = ((in[2] & 0x07) << 29) | (in[3] << 21) | (in[4] << 13) | ((in[5] & 0xC0) << 5);
  const int32_t z = ((in[5] & 0x3F) << 26) | (in[6] << 18) | (in[7] << 10);
  out[0] = scale * static_cast<float>(x >> 11);
  out[1] = scale * static_cast<float>(y >> 11);
  out[2] = scale * static_cast<float>(z >> 11);
}

uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24); }

mvr_quat quatFromRotationVector(const float w[3], float dt) {
  const float angle = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * dt;
  if (angle < 1e-9f)
    return quatIdentity();
  const float s = std::sin(angle * 0.5f) / (angle / dt);
  return quatNormalize(mvr_quat{w[0] * s, w[1] * s, w[2] * s, std::cos(angle * 0.5f)});
}

// Head rotation from the IMU: the gyroscope integrated, with the accelerometer pulling the tilt toward gravity
// while the head is near rest. The heading drifts slowly, as a CV1's does without its sensors.
class Fusion {
public:
  void update(int64_t time_ns, const float accel[3], const float gyro[3]) {
    if (!last_ns_) {
      last_ns_ = time_ns;
      std::memcpy(gyro_, gyro, sizeof(gyro_));
      return;
    }
    const float dt = std::clamp(static_cast<float>(time_ns - last_ns_) / 1e9f, 0.0f, 0.05f);
    last_ns_ = time_ns;
    std::memcpy(gyro_, gyro, sizeof(gyro_));
    rot_ = quatNormalize(quatMul(rot_, quatFromRotationVector(gyro, dt)));

    // tilt correction: rotate the measured up (the accelerometer, in head space) toward world up
    const float norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (norm > 9.0f && norm < 10.6f) {
      const mvr_vec3 up = quatRotate(rot_, mvr_vec3{accel[0] / norm, accel[1] / norm, accel[2] / norm});
      // axis = up x (0,1,0), angle from up.y
      const mvr_vec3 axis{-up.z, 0.0f, up.x};
      const float axis_len = std::sqrt(axis.x * axis.x + axis.z * axis.z);
      if (axis_len > 1e-6f) {
        const float angle = std::atan2(axis_len, up.y);
        // a fast pull at start-up, then the error's half-life is about 1.4 s
        const float gain = settled_ < 500 ? 0.05f : 0.5f * dt;
        const mvr_quat fix = quatAxisAngle(mvr_vec3{axis.x / axis_len, 0.0f, axis.z / axis_len}, angle * gain);
        rot_ = quatNormalize(quatMul(fix, rot_));
      }
      ++settled_;
    }
  }

  // the rotation at `time_ns`, predicted forward from the newest sample with its angular velocity
  mvr_quat predict(int64_t time_ns, mvr_vec3 *angular_velocity) const {
    const float dt = std::clamp(static_cast<float>(time_ns - last_ns_) / 1e9f, 0.0f, 0.1f);
    if (angular_velocity)
      *angular_velocity = quatRotate(rot_, mvr_vec3{gyro_[0], gyro_[1], gyro_[2]});
    return quatNormalize(quatMul(rot_, quatFromRotationVector(gyro_, dt)));
  }

  // the turn from the newest sample to `time_ns` in the device's own frame, and how long that is
  mvr_quat ahead(int64_t time_ns, float *seconds) const {
    const float dt = std::clamp(static_cast<float>(time_ns - last_ns_) / 1e9f, 0.0f, 0.1f);
    if (seconds)
      *seconds = dt;
    return quatFromRotationVector(gyro_, dt);
  }
  mvr_quat rotation() const { return rot_; }
  mvr_vec3 gyro() const { return {gyro_[0], gyro_[1], gyro_[2]}; }

  bool ready() const { return settled_ > 50; }

private:
  mvr_quat rot_ = quatIdentity();
  float gyro_[3] = {};
  int64_t last_ns_ = 0;
  uint64_t settled_ = 0;
};

// The panel's geometry, from the headset's display info report.
struct Panel {
  uint32_t width_px = 2160, height_px = 1200;
  float width_m = 0.1296f, height_m = 0.072f; // reported in micrometers; these are only used when no report comes
  float center_from_top_m = 0.036f;
  float lens_separation_m = 0.0635f;
};

NSString *const kShaderSource = @R"(
#include <metal_stdlib>
using namespace metal;

struct Distortion {
  float2 lens_center; // in the eye's NDC, for the right eye; the left eye is its mirror
  float2 tan_scale;
  float4 fov_tan;     // left, right, up, down, each a positive tangent
  float4 chromatic;
  float max_r2;
  uint eye;
  float2 viewport_origin; // pixels
  float2 viewport_size;
  float k[11];
};

// The lens of a headset other than the CV1 (direct_backend.h DirectLens), and how its eye sits on the panel
struct Lens {
  uint model;             // 0: the CV1's curve (Distortion); 1: panotools (Rift S); 2: Vive, Vive Pro, Index
  uint portrait;          // the eye's view is turned a quarter right on the panel (Rift S)
  uint flip_y;            // the Vive Pro 2's image is upside down
  uint pad;
  float4 eye_rotation;    // the eye's view in the head, a quaternion (x, y, z, w): the Index's lenses are canted
  float4 pano_k;          // panotools k0..k3
  float4 pano_k4_ab;      // k4, then the aberration of red, green, blue
  float4 pano;            // scale, lens center x, y (meters)
  float4 pano_viewport;   // the view's size (meters)
  float4 vive;            // aspect x over y, grow for undistort
  float4 vive_center[3];  // per colour
  float4 vive_coeff[3];   // per colour, k1..k4
};

static float3 qrot(float4 q, float3 v) {
  float3 t = 2.0 * cross(q.xyz, v);
  return v + q.w * t + cross(q.xyz, t);
}

struct VOut {
  float4 position [[position]];
};

vertex VOut fullscreen(uint vid [[vertex_id]]) {
  // one triangle that covers the viewport
  float2 p = float2((vid << 1) & 2, vid & 2);
  VOut out;
  out.position = float4(p * 2.0 - 1.0, 0.0, 1.0);
  return out;
}

static float value_at(constant float *k, int j) { return j == 0 ? 1.0 : k[j]; }

static float slope_after(constant float *k, int n, int j) {
  if (j == 0) return k[1] - k[0];
  if (j <= n - 3) return 0.5 * (k[j + 1] - k[j - 1]);
  if (j == n - 2) return 0.5 * (k[n - 1] - k[n - 2]);
  return k[n - 1] - k[n - 2];
}

static float slope_before(constant float *k, int n, int j) {
  if (j <= n - 3) return slope_after(k, n, j);
  if (j == n - 2) return 0.5 * (k[n - 1] - k[n - 3]);
  return k[n - 1] - k[n - 2];
}

static float hermite(float p0, float m0, float p1, float m1, float t) {
  float t2 = t * t, t3 = t2 * t;
  return (2 * t3 - 3 * t2 + 1) * p0 + (t3 - 2 * t2 + t) * m0 + (-2 * t3 + 3 * t2) * p1 + (t3 - t2) * m1;
}

static float magnification(constant Distortion &d, float rho) {
  const int n = 11;
  float s = (n - 1) * rho / d.max_r2;
  int j = clamp(int(floor(s)), 0, n - 1);
  float t = s - j;
  float p0 = value_at(d.k, j), m0 = slope_after(d.k, n, j), p1, m1;
  if (j == n - 1) {
    p1 = p0 + m0;
    m1 = m0;
  } else {
    p1 = value_at(d.k, j + 1);
    m1 = slope_before(d.k, n, j + 1);
  }
  return hermite(p0, m0, p1, m1, t);
}

// Where each colour of this pixel is in the rendered eye image (0..1 across and down), for the lens models other
// than the CV1's: Monado's u_compute_distortion_panotools and u_compute_distortion_vive.
static void lens_uv(constant Distortion &d, constant Lens &lens, float2 position, thread float2 src[3]) {
  float2 uv = (position - d.viewport_origin) / d.viewport_size;
  // the panel's view of the eye turned back to the eye's own (Monado's u_device_rotation_right)
  if (lens.portrait != 0) {
    float2 c = uv - 0.5;
    uv = float2(c.y, -c.x) + 0.5;
  }
  if (lens.model == 1) {
    float2 r = (uv * lens.pano_viewport.xy - lens.pano.yz) / lens.pano.x;
    float m = length(r);
    float k = lens.pano_k.x + lens.pano_k.y * m + lens.pano_k.z * m * m + lens.pano_k.w * m * m * m +
              lens.pano_k4_ab.x * m * m * m * m;
    for (int c = 0; c < 3; ++c)
      src[c] = (r * k * lens.pano.x * lens.pano_k4_ab[1 + c] + lens.pano.yz) / lens.pano_viewport.xy;
    return;
  }
  float aspect = lens.vive.x;
  float factor = 0.5 / (1.0 + lens.vive.y);
  for (int c = 0; c < 3; ++c) {
    float2 t = float2(2.0 * uv.x - 1.0, (2.0 * uv.y - 1.0) / aspect) - lens.vive_center[c].xy;
    float r2 = dot(t, t);
    float4 k = lens.vive_coeff[c];
    float s = 1.0 / (1.0 + r2 * (k.x + r2 * (k.y + r2 * k.z))) + k.w;
    src[c] = 0.5 + (t * s + lens.vive_center[c].xy) * float2(factor, factor * aspect);
    if (lens.flip_y != 0) src[c].y = 1.0 - src[c].y;
  }
}

// The direction each colour channel of this pixel shows, as a tangent from the eye's axis (x right, y down): the
// lens bends red, green and blue apart, so each is looked up where the lens takes it.
static void channel_tangents(constant Distortion &d, constant Lens &lens, float2 position, thread float2 out[3]) {
  if (lens.model != 0) {
    // a position in the rendered image is a direction: the image spans the eye's field of view
    float2 src[3];
    lens_uv(d, lens, position, src);
    for (int c = 0; c < 3; ++c)
      out[c] = float2(src[c].x * (d.fov_tan.x + d.fov_tan.y) - d.fov_tan.x,
                      src[c].y * (d.fov_tan.z + d.fov_tan.w) - d.fov_tan.z);
    return;
  }
  float2 uv = (position - d.viewport_origin) / d.viewport_size;
  float2 ndc = uv * 2.0 - 1.0;
  // the halves of the panel are mirror images. The lens geometry (lens_center) is the right eye's, where the lens
  // sits toward the nose, left of its half's centre; the left eye is worked out as its mirror, as Monado does
  if (d.eye == 0) ndc.x = -ndc.x;
  float2 tan_angle = (ndc - d.lens_center) * d.tan_scale;
  float rho = dot(tan_angle, tan_angle);
  float r = magnification(d, rho);
  float3 scale = float3(r * (1.0 + d.chromatic.x + rho * d.chromatic.y), r, r * (1.0 + d.chromatic.z + rho * d.chromatic.w));
  for (int c = 0; c < 3; ++c) {
    float2 t = tan_angle * scale[c];
    if (d.eye == 0) t.x = -t.x;
    out[c] = t;
  }
}

struct SceneRotation {
  float4 columns[3]; // the composed frame: the head now to the head the frame was rendered for; the scene: head to world
};

fragment float4 distort(VOut in [[stage_in]], constant Distortion &d [[buffer(0)]],
                        constant SceneRotation &rotation [[buffer(1)]], constant Lens &lens [[buffer(2)]],
                        texture2d<float> frame [[texture(0)]], sampler s [[sampler(0)]]) {
  float2 t[3];
  channel_tangents(d, lens, in.position.xy, t);
  float3x3 now_to_rendered = float3x3(rotation.columns[0].xyz, rotation.columns[1].xyz, rotation.columns[2].xyz);
  float4 eye = lens.eye_rotation;
  float4 eye_inverse = float4(-eye.xyz, eye.w);
  float3 color;
  for (int c = 0; c < 3; ++c) {
    // The direction this pixel shows, as the head is now, seen from the head the frame was rendered for: the frame
    // is turned by how far the head has turned since. The eye looks down -z with +y up; tangent y runs down. A
    // canted eye's direction goes into the head's frame for the turn, and back.
    float3 v = qrot(eye_inverse, now_to_rendered * qrot(eye, float3(t[c].x, -t[c].y, -1.0)));
    if (v.z > -1e-4) {
      color[c] = 0.0;
      continue;
    }
    float2 tr = float2(v.x / -v.z, -v.y / -v.z);
    // where that direction is in the eye's image: x from the left edge of its field of view, y from the top
    float2 src = float2((tr.x + d.fov_tan.x) / (d.fov_tan.x + d.fov_tan.y), (tr.y + d.fov_tan.z) / (d.fov_tan.z + d.fov_tan.w));
    if (any(src < 0.0) || any(src > 1.0)) {
      color[c] = 0.0;
      continue;
    }
    color[c] = frame.sample(s, float2((d.eye + src.x) * 0.5, src.y))[c];
  }
  return float4(color, 1.0);
}

// The test scene shown while no application draws: a sphere around the viewer, fixed in the world, with a line every
// The test scene (--test-scene): 15 degrees of latitude and longitude, the horizon, and a red mark straight ahead, so
// that turning the head and the lens correction can be judged without an application.
static float3 grid_color(float3 w) {
  const float deg = 57.2957795;
  float lat = asin(clamp(w.y, -1.0, 1.0)) * deg;
  float lon = atan2(w.x, -w.z) * deg;
  float3 color = w.y > 0.0 ? float3(0.03, 0.04, 0.10) : float3(0.03, 0.07, 0.03);
  float d_lat = abs(lat - 15.0 * round(lat / 15.0));
  float d_lon = abs(lon - 15.0 * round(lon / 15.0)) * cos(lat / deg);
  if (d_lat < 0.15 || d_lon < 0.15) color = float3(0.55);
  if (abs(lon) < 0.3 && lat > -60.0 && lat < 60.0) color = float3(0.9, 0.3, 0.3); // the meridian straight ahead
  if (abs(lat) < 0.35) color = float3(0.2, 0.9, 1.0);                             // the horizon
  float ahead = acos(clamp(-w.z, -1.0, 1.0)) * deg;
  if (ahead < 1.5) color = float3(1.0, 0.1, 0.1);
  return color;
}

// ---- the home: deep space all around, the logo ahead, and below it the game's window while it loads ----

struct Home {
  float4 columns[3]; // head to world
  float4 eye[2];     // each eye's position in the world, meters (the head is at the origin)
  float4 window;     // x: 1 when a window is shown; y: its height over its width
  float4 logo;       // x: 1 when there is a logo; y: its height over its width
  float4 passthrough; // x: 1 when the headset's cameras are shown; y: the tangent of their half field of view;
                      // z: 1 when the picture holds both eyes side by side
  // each Touch controller, in the world with the head at the origin: where it is, its three axes, and its inputs.
  // hand_in0 is the thumbstick's x and y, the trigger and the grip; hand_in1 is the buttons pressed and touched
  // (mvr_button bits), 1 when the controller is on, and -1 for the left hand or 1 for the right
  float4 hand_pos[2];
  float4 hand_x[2];
  float4 hand_y[2];
  float4 hand_z[2];
  float4 hand_in0[2];
  float4 hand_in1[2];
  uint test_scene;
};

static float hash13(float3 p) {
  p = fract(p * 0.1031);
  p += dot(p, p.zyx + 31.32);
  return fract((p.x + p.y) * p.z);
}

static float3 hash33(float3 p) {
  p = fract(p * float3(0.1031, 0.1030, 0.0973));
  p += dot(p, p.yxz + 33.33);
  return fract((p.xxy + p.yxx) * p.zyx);
}

static float noise3(float3 p) {
  float3 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  float n000 = hash13(i), n100 = hash13(i + float3(1, 0, 0)), n010 = hash13(i + float3(0, 1, 0));
  float n110 = hash13(i + float3(1, 1, 0)), n001 = hash13(i + float3(0, 0, 1)), n101 = hash13(i + float3(1, 0, 1));
  float n011 = hash13(i + float3(0, 1, 1)), n111 = hash13(i + float3(1, 1, 1));
  return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y), mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y),
             f.z);
}

static float fbm(float3 p) {
  float v = 0.0, a = 0.5;
  for (int i = 0; i < 5; ++i) {
    v += a * noise3(p);
    p = p * 2.03 + 17.1;
    a *= 0.5;
  }
  return v;
}

// one star per lit cell of a grid of directions; `size` is the star's radius in cells
static float3 star_layer(float3 w, float scale, float threshold, float size) {
  float3 p = w * scale;
  float3 c = floor(p);
  float h = hash13(c);
  if (h < threshold)
    return float3(0.0);
  float3 pos = c + 0.3 + 0.4 * hash33(c);
  float d = length(p - pos);
  float brightness = exp(-d * d / (size * size)) * (0.35 + 0.65 * (h - threshold) / (1.0 - threshold));
  float3 tint = mix(float3(0.70, 0.80, 1.00), float3(1.00, 0.86, 0.72), hash13(c + 7.0));
  return tint * brightness;
}

// a spiral galaxy seen from `center`, `radius` its size in radians, `tilt` how flat it looks
static float3 galaxy(float3 w, float3 center, float radius, float tilt, float twist, float3 tint) {
  center = normalize(center);
  float cosang = dot(w, center);
  if (cosang < cos(radius * 3.0))
    return float3(0.0);
  float3 u = normalize(cross(center, abs(center.y) < 0.9 ? float3(0, 1, 0) : float3(1, 0, 0)));
  float3 v = cross(center, u);
  float2 q = float2(dot(w, u), dot(w, v)) / radius;
  q.y /= tilt;
  float r = length(q);
  float theta = atan2(q.y, q.x);
  float arms = 0.5 + 0.5 * cos(2.0 * theta - log(r + 0.05) * twist);
  float disk = exp(-r * 2.2) * (0.35 + 0.65 * arms * smoothstep(0.05, 0.4, r));
  float core = exp(-r * r * 40.0) * 1.6;
  return tint * disk * 0.55 + float3(1.0, 0.92, 0.85) * core;
}

static float3 space_color(float3 w) {
  float3 color = float3(0.004, 0.005, 0.012);
  // a band of nebula across the sky, purple and blue like the logo
  float3 band_normal = normalize(float3(0.35, 1.0, 0.25));
  float band = exp(-pow(dot(w, band_normal) / 0.32, 2.0));
  float n = fbm(w * 2.4 + 3.0);
  float m = fbm(w * 5.1 - 11.0);
  float cloud = smoothstep(0.35, 0.85, n) * band;
  float3 nebula = mix(float3(0.22, 0.05, 0.42), float3(0.03, 0.20, 0.55), smoothstep(0.3, 0.7, m));
  color += nebula * cloud * 0.55 + float3(0.10, 0.08, 0.16) * band * smoothstep(0.45, 0.9, m) * 0.25;
  // dust lanes darken it
  color *= 1.0 - 0.55 * smoothstep(0.55, 0.8, fbm(w * 9.0 + 5.0)) * band;
  color += galaxy(w, float3(-0.75, 0.45, -0.55), 0.10, 0.45, 5.0, float3(0.55, 0.45, 1.0));
  color += galaxy(w, float3(0.85, 0.25, 0.30), 0.06, 0.75, 6.0, float3(0.40, 0.70, 1.0));
  color += galaxy(w, float3(0.20, -0.55, 0.80), 0.08, 0.30, 4.0, float3(0.85, 0.50, 0.95));
  color += galaxy(w, float3(-0.30, 0.85, 0.45), 0.045, 0.6, 5.5, float3(0.60, 0.60, 1.0));
  color += star_layer(w, 340.0, 0.965, 0.22) * 0.9;
  color += star_layer(w, 150.0, 0.975, 0.18) * 1.2;
  color += star_layer(w, 60.0, 0.985, 0.10) * 1.8 * (1.0 + band);
  return color;
}

// Where a ray from `origin` along `dir` meets a panel facing the viewer (+z) at `center`, as UV in [0, 1] with v
// down; false when it misses
static bool panel_hit(float3 origin, float3 dir, float3 center, float2 size, thread float2 &uv) {
  if (dir.z >= -1e-4)
    return false;
  float t = (center.z - origin.z) / dir.z;
  if (t <= 0.0)
    return false;
  float3 hit = origin + dir * t;
  uv = float2((hit.x - center.x) / size.x + 0.5, 0.5 - (hit.y - center.y) / size.y);
  return all(uv >= 0.0) && all(uv <= 1.0);
}

// the panels in front of the space, as a colour and how much it covers
static float4 panels(float3 origin, float3 dir, constant Home &home, texture2d<float> logo, texture2d<float> window,
                     sampler s) {
  float2 uv;
  const bool has_window = home.window.x > 0.5;
  if (has_window) {
    const float width = 2.4;
    const float3 center = float3(0.0, -0.15, -2.4);
    float2 size = float2(width, width * home.window.y);
    // a faint frame around the screen
    float2 frame_uv;
    if (panel_hit(origin, dir, center, size + 0.04, frame_uv)) {
      if (panel_hit(origin, dir, center, size, uv))
        return float4(window.sample(s, uv).rgb, 1.0);
      return float4(0.10, 0.10, 0.16, 1.0);
    }
  }
  if (home.logo.x > 0.5) {
    const float width = has_window ? 1.5 : 2.0;
    const float3 center = has_window ? float3(0.0, 1.2, -2.6) : float3(0.0, 0.05, -2.6);
    if (panel_hit(origin, dir, center, float2(width, width * home.logo.y), uv)) {
      float3 c = logo.sample(s, uv).rgb;
      // the logo is drawn on black: black is see-through
      float a = smoothstep(0.03, 0.22, max(c.r, max(c.g, c.b)));
      return float4(c, a);
    }
  }
  return float4(0.0);
}

// Where a ray meets a capsule from `pa` to `pb` of radius `r`, or a negative number; `n` is the surface normal there
static float capsule_hit(float3 ro, float3 rd, float3 pa, float3 pb, float r, thread float3 &n) {
  float3 ba = pb - pa, oa = ro - pa;
  float baba = dot(ba, ba), bard = dot(ba, rd), baoa = dot(ba, oa), rdoa = dot(rd, oa), oaoa = dot(oa, oa);
  float a = baba - bard * bard, b = baba * rdoa - baoa * bard, c = baba * oaoa - baoa * baoa - r * r * baba;
  float h = b * b - a * c;
  if (h < 0.0 || a < 1e-9)
    return -1.0;
  float t = (-b - sqrt(h)) / a;
  float y = baoa + t * bard;
  if (y <= 0.0 || y >= baba) {
    // one of the round ends
    float3 oc = y <= 0.0 ? oa : ro - pb;
    b = dot(rd, oc);
    c = dot(oc, oc) - r * r;
    h = b * b - c;
    if (h <= 0.0)
      return -1.0;
    t = -b - sqrt(h);
  }
  if (t <= 0.0)
    return -1.0;
  float3 pos = ro + rd * t;
  float along = clamp(dot(pos - pa, ba) / baba, 0.0, 1.0);
  n = normalize(pos - pa - ba * along);
  return t;
}

// ---- a Touch controller, as a distance field in its own frame (meters; -z is the way it points, +y is up) ----
// It is modelled for the right hand and mirrored for the left: a round face plate tilted toward the thumb, with the
// thumbstick, two buttons and the small menu button on it; the handle running back and down into the palm; the
// tracking ring around the knuckles; the trigger under the index finger and the grip button under the middle one.
// Each moving part is where the controller's own inputs say it is.

struct TouchInputs {
  float2 stick;   // mirrored for the left hand, as the model is
  float trigger;
  float grip;
  float primary;  // 1 pressed, 0 released
  float secondary;
  float menu;
  float click;    // the thumbstick's click
};

static float sd_capsule(float3 p, float3 a, float3 b, float r) {
  float3 pa = p - a, ba = b - a;
  return length(pa - ba * clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0)) - r;
}

// a disc of radius `r` and half height `h` lying in the xy plane, with rounded edges
static float sd_disc(float3 p, float r, float h, float round) {
  float2 d = float2(length(p.xy) - r + round, abs(p.z) - h + round);
  return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0) - round;
}

static float sd_box(float3 p, float3 half_size, float round) {
  float3 d = abs(p) - half_size + round;
  return length(max(d, 0.0)) + min(max(d.x, max(d.y, d.z)), 0.0) - round;
}

static float smooth_min(float a, float b, float k) {
  float h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
  return mix(b, a, h) - k * h * (1.0 - h);
}

// materials: 0 body, 1 primary button, 2 secondary button, 3 menu button, 4 thumbstick, 5 trigger, 6 grip
static float2 nearer(float2 a, float d, float material) { return d < a.x ? float2(d, material) : a; }

static float2 touch_map(float3 p, TouchInputs in) {
  // the face plate's own axes: across it, up it (away from the palm), and out of it
  const float3 face_center = float3(0.0, 0.016, -0.010);
  const float3 face_up = float3(0.0, 0.574, -0.819);
  const float3 face_out = float3(0.0, 0.819, 0.574);
  float3 q = p - face_center;
  float3 f = float3(q.x, dot(q, face_up), dot(q, face_out));

  const float3 handle_top = float3(0.0, -0.004, 0.018), handle_end = float3(0.0, -0.078, 0.092);
  float body = sd_disc(f, 0.0275, 0.0065, 0.004);
  body = smooth_min(body, sd_capsule(p, handle_top, handle_end, 0.0165), 0.014);
  // the ring: around the outside of the hand, hung from the plate
  float3 r = f - float3(0.031, -0.004, -0.013);
  float ring = length(float2(length(r.xy) - 0.048, r.z)) - 0.0055;
  float2 result = float2(smooth_min(body, ring, 0.006), 0.0);

  // the buttons sit proud of the plate and sink when pressed
  result = nearer(result, sd_disc(f - float3(-0.0045, -0.0115, 0.0078 - 0.0016 * in.primary), 0.0058, 0.0020, 0.0012), 1.0);
  result = nearer(result, sd_disc(f - float3(-0.0135, 0.0035, 0.0078 - 0.0016 * in.secondary), 0.0058, 0.0020, 0.0012), 2.0);
  result = nearer(result, sd_disc(f - float3(0.0165, -0.0075, 0.0072 - 0.0012 * in.menu), 0.0036, 0.0016, 0.0010), 3.0);

  // the thumbstick leans the way it is pushed and sinks when clicked
  float3 stick_base = float3(0.0065, 0.0095, 0.0055 - 0.0015 * in.click);
  float3 lean = normalize(float3(in.stick.x * 0.55, in.stick.y * 0.55, 1.0));
  float3 stick_top = stick_base + lean * 0.0135;
  float stick = min(sd_capsule(f, stick_base, stick_top, 0.0034),
                    sd_disc(f - stick_top, 0.0078, 0.0026, 0.0022));
  result = nearer(result, stick, 4.0);

  // the trigger swings back into the body, the grip button into the handle
  float3 trigger = p - float3(0.0, -0.012 - 0.002 * in.trigger, -0.034 + 0.009 * in.trigger);
  result = nearer(result, sd_box(trigger, float3(0.0062, 0.0115, 0.0085), 0.0035), 5.0);
  float3 grip = p - (mix(handle_top, handle_end, 0.34) + float3(-0.0025, -0.011, -0.012) * (1.0 - 0.3 * in.grip));
  result = nearer(result, sd_box(grip, float3(0.0055, 0.0125, 0.0050), 0.0030), 6.0);
  return result;
}

// The Touch controllers, as a colour and how much it covers: each one ray-marched in its own frame, shaded as
// dark plastic, with the part being touched or pressed lit in the hand's colour.
static float4 hands(float3 origin, float3 dir, constant Home &home) {
  float nearest = 1e9;
  float4 result = float4(0.0);
  for (int i = 0; i < 2; ++i) {
    if (home.hand_in1[i].z < 0.5)
      continue;
    const float side = home.hand_in1[i].w;
    const float3 tint = i == 0 ? float3(0.25, 0.55, 1.00) : float3(0.72, 0.38, 1.00);
    const float3 ax = home.hand_x[i].xyz, ay = home.hand_y[i].xyz, az = home.hand_z[i].xyz;
    // the ray in the controller's frame, mirrored for the left hand
    float3 rel = origin - home.hand_pos[i].xyz;
    float3 ro = float3(dot(rel, ax) * side, dot(rel, ay), dot(rel, az));
    float3 rd = float3(dot(dir, ax) * side, dot(dir, ay), dot(dir, az));

    // the faint line the way it points
    float3 line_n;
    float line_t = capsule_hit(ro, rd, float3(0.0, 0.0, -0.05), float3(0.0, 0.0, -0.65), 0.0014, line_n);
    if (line_t > 0.0 && line_t < nearest) {
      nearest = line_t;
      result = float4(tint, 0.45);
    }

    // everything of the controller is inside this sphere: most rays miss it and stop here
    const float3 bound_center = float3(0.012, -0.022, 0.028);
    const float bound_radius = 0.105;
    float3 oc = ro - bound_center;
    float b = dot(oc, rd), c = dot(oc, oc) - bound_radius * bound_radius;
    float h = b * b - c;
    if (h <= 0.0)
      continue;
    float root = sqrt(h);
    float t = max(-b - root, 0.0);
    const float t_end = -b + root;
    if (t_end <= 0.0 || t > nearest)
      continue;

    const uint pressed = uint(home.hand_in1[i].x), touched = uint(home.hand_in1[i].y);
    TouchInputs in;
    in.stick = float2(home.hand_in0[i].x * side, home.hand_in0[i].y);
    in.trigger = home.hand_in0[i].z;
    in.grip = home.hand_in0[i].w;
    in.primary = (pressed & 1u) ? 1.0 : 0.0;
    in.secondary = (pressed & 2u) ? 1.0 : 0.0;
    in.menu = (pressed & 12u) ? 1.0 : 0.0;
    in.click = (pressed & 64u) ? 1.0 : 0.0;

    float2 hit = float2(1.0, 0.0);
    bool found = false;
    for (int step = 0; step < 56 && t < t_end; ++step) {
      hit = touch_map(ro + rd * t, in);
      if (hit.x < 0.00025) {
        found = true;
        break;
      }
      t += hit.x;
    }
    if (!found || t >= nearest)
      continue;
    nearest = t;

    float3 pos = ro + rd * t;
    const float2 e = float2(0.0004, 0.0);
    float3 n = normalize(float3(touch_map(pos + e.xyy, in).x - touch_map(pos - e.xyy, in).x,
                                touch_map(pos + e.yxy, in).x - touch_map(pos - e.yxy, in).x,
                                touch_map(pos + e.yyx, in).x - touch_map(pos - e.yyx, in).x));
    // how lit up the part is: touched a little, pressed fully, and by how far for the trigger, grip and stick
    const int material = int(hit.y + 0.5);
    float glow = 0.0;
    if (material == 1)
      glow = max(in.primary, (touched & 1u) ? 0.30 : 0.0);
    else if (material == 2)
      glow = max(in.secondary, (touched & 2u) ? 0.30 : 0.0);
    else if (material == 3)
      glow = in.menu;
    else if (material == 4)
      glow = max(max(in.click, min(length(in.stick), 1.0)), (touched & 64u) ? 0.30 : 0.0);
    else if (material == 5)
      glow = max(in.trigger, (touched & 16u) ? 0.25 : 0.0);
    else if (material == 6)
      glow = in.grip;

    // lit from above and behind the viewer, in the controller's frame turned back to the world
    float3 world_n = ax * (n.x * side) + ay * n.y + az * n.z;
    float light = 0.22 + 0.78 * max(dot(world_n, normalize(float3(0.25, 0.85, 0.45))), 0.0);
    float facing = max(dot(world_n, -dir), 0.0);
    float rim = pow(1.0 - facing, 3.0);
    float3 base = material == 0 ? float3(0.055, 0.058, 0.066) : float3(0.11, 0.115, 0.13);
    float3 shaded = base * light + float3(0.20) * pow(facing, 24.0) * light + tint * 0.22 * rim;
    shaded += tint * glow * (0.55 + 0.45 * facing);
    result = float4(shaded, 1.0);
  }
  return result;
}

// The headset's cameras, as straight-ahead views of a fixed field of view: what the eye looks at in the head's frame,
// found in the camera's picture (its own eye's half of an Index's side-by-side picture)
static float3 passthrough_color(float3 dir_head, uint eye, constant Home &home, texture2d<float> camera, sampler s) {
  if (dir_head.z > -1e-3) return float3(0.0);
  float2 t = float2(dir_head.x / -dir_head.z, -dir_head.y / -dir_head.z) / home.passthrough.y;
  float2 uv = t * 0.5 + 0.5;
  if (any(uv < 0.0) || any(uv > 1.0)) return float3(0.0);
  if (home.passthrough.z > 0.5) uv.x = (float(eye) + uv.x) * 0.5;
  return camera.sample(s, uv).rgb;
}

fragment float4 scene(VOut in [[stage_in]], constant Distortion &d [[buffer(0)]], constant Home &home [[buffer(1)]],
                      constant Lens &lens [[buffer(2)]], texture2d<float> logo [[texture(0)]],
                      texture2d<float> window [[texture(1)]], texture2d<float> camera [[texture(2)]],
                      sampler s [[sampler(0)]]) {
  float2 t[3];
  channel_tangents(d, lens, in.position.xy, t);
  // a canted eye looks a little aside from the head: its view into the head's frame first
  float3x3 eye_to_head = float3x3(qrot(lens.eye_rotation, float3(1, 0, 0)), qrot(lens.eye_rotation, float3(0, 1, 0)),
                                  qrot(lens.eye_rotation, float3(0, 0, 1)));
  float3x3 head_to_world = float3x3(home.columns[0].xyz, home.columns[1].xyz, home.columns[2].xyz) * eye_to_head;
  float3 color;
  if (home.test_scene) {
    for (int c = 0; c < 3; ++c) {
      // the eye looks down -z, with +y up: tangent y runs down
      float3 view = normalize(float3(t[c].x, -t[c].y, -1.0));
      color[c] = grid_color(head_to_world * view)[c];
    }
    return float4(color, 1.0);
  }
  // the space is far away and soft: one look-up for all three colours; the panels are near and sharp: one per colour
  const float3 origin = home.eye[d.eye].xyz;
  const float3 background = home.passthrough.x > 0.5
      ? passthrough_color(normalize(eye_to_head * float3(t[1].x, -t[1].y, -1.0)), d.eye, home, camera, s)
      : space_color(normalize(head_to_world * float3(t[1].x, -t[1].y, -1.0)));
  for (int c = 0; c < 3; ++c) {
    float3 dir = normalize(head_to_world * float3(t[c].x, -t[c].y, -1.0));
    float4 panel = panels(origin, dir, home, logo, window, s);
    color[c] = mix(background[c], panel[c], panel.a);
  }
  // the controllers are within arm's reach, in front of everything else. They are marched once, along the green
  // channel's ray: three marches a pixel would cost more than the colour fringe at their edges is worth.
  float4 hand = hands(origin, normalize(head_to_world * float3(t[1].x, -t[1].y, -1.0)), home);
  color = mix(color, hand.rgb, hand.a);
  return float4(color, 1.0);
}
)";

struct DistortionUniforms {
  float lens_center[2];
  float tan_scale[2];
  float fov_tan[4];
  float chromatic[4];
  float max_r2;
  uint32_t eye;
  float viewport_origin[2];
  float viewport_size[2];
  float k[kCurveKnots];
  float pad[3]; // Metal's struct is 128 bytes: its float4 members align it to 16
};
static_assert(sizeof(DistortionUniforms) == 128, "the shader's Distortion struct");

struct LensUniforms {
  uint32_t model;
  uint32_t portrait;
  uint32_t flip_y;
  uint32_t pad;
  float eye_rotation[4];
  float pano_k[4];
  float pano_k4_ab[4];
  float pano[4];
  float pano_viewport[4];
  float vive[4];
  float vive_center[3][4];
  float vive_coeff[3][4];
};
static_assert(sizeof(LensUniforms) == 208, "the shader's Lens struct");

struct HomeUniforms {
  float columns[3][4]; // head to world
  float eye[2][4];     // each eye's position in the world, meters
  float window[4];     // 1 when a window is shown, its height over its width
  float logo[4];       // 1 when there is a logo, its height over its width
  float passthrough[4]; // 1 when the cameras are shown, the tangent of their half field of view, 1 for stereo
  float hand_pos[2][4]; // each controller: where it is, seen from the head
  float hand_x[2][4];   // its axes
  float hand_y[2][4];
  float hand_z[2][4];
  float hand_in0[2][4]; // thumbstick x and y, trigger, grip
  float hand_in1[2][4]; // buttons pressed, buttons touched, 1 when it is on, -1 left or 1 right
  uint32_t test_scene;
  uint32_t pad[3];
};
static_assert(sizeof(HomeUniforms) == 336, "the shader's Home struct");

// ---- the Touch controllers and the remote, through the headset's radio ----
//
// The radio is the headset's USB interface 1. Each controller sends IN report 12 at about 500 Hz (two 28-byte
// messages a report); commands go out as feature reports 26 (the command, then polled until done) and 27 (its data).
// A controller's calibration (its IMU matrices, stick, trigger and grip ranges, touch thresholds) is JSON in its own
// flash, read over the radio once and cached by the flash's hash. The layout and the decoding follow RiftLinux's
// rift_radio.c (Monado's Rift driver, BSL-1.0; Beyley Cardellio, Jan Schmidt, Aditya Mehra and others).

// ---- positional tracking with the Rift's sensors ----

struct TrackedLed {
  float position[3]; // meters, in the device's frame
  float direction[3];
};

// a device as the sensors' tracker has it, in the tracker's own space
struct SensorPose {
  mvr_quat orientation = quatIdentity();
  mvr_vec3 position{}, velocity{}; // smoothed (OneEuro below); the velocity is the smoothed position's
  bool position_tracked = false;   // a sensor saw it in the last half second
};

// The yaw of a rotation about +y alone, and the rotation `from` turned toward `to` by `fraction` of their yaw
// difference (both about +y).
float yawAngle(mvr_quat q) {
  const mvr_quat y = quatYawOnly(q);
  return 2.0f * std::atan2(y.y, y.w);
}

mvr_quat yawToward(mvr_quat from, mvr_quat to, float fraction) {
  float a = yawAngle(from), d = yawAngle(to) - a;
  while (d > 3.14159265f)
    d -= 6.2831853f;
  while (d < -3.14159265f)
    d += 6.2831853f;
  return quatAxisAngle(mvr_vec3{0, 1, 0}, a + d * fraction);
}

// The 1 Euro filter (Casiez, Roussel and Vogel, CHI 2012) on a position: smoothed hard while it barely moves, where
// the cameras' noise shows, and less the faster it moves, where lag would. Run once per new tracker sample.
class OneEuro {
public:
  OneEuro(float min_cutoff_hz, float beta) : min_cutoff_(min_cutoff_hz), beta_(beta) {}

  void update(uint64_t time_ns, mvr_vec3 x) {
    const double dt = static_cast<double>(time_ns - time_ns_) / 1e9;
    const mvr_vec3 jump = x - x_;
    // the first sample, a gap, or a jump the device cannot have made: start again from here
    if (!init_ || time_ns <= time_ns_ || dt > 0.25 ||
        jump.x * jump.x + jump.y * jump.y + jump.z * jump.z > 0.3f * 0.3f) {
      if (!init_ || time_ns != time_ns_) {
        x_ = x;
        dx_ = {};
        time_ns_ = time_ns;
        init_ = true;
      }
      return;
    }
    time_ns_ = time_ns;
    const mvr_vec3 v = jump * static_cast<float>(1.0 / dt);
    dx_ = dx_ + (v - dx_) * alpha(kDerivativeCutoffHz, dt);
    const float speed = std::sqrt(dx_.x * dx_.x + dx_.y * dx_.y + dx_.z * dx_.z);
    x_ = x_ + jump * alpha(min_cutoff_ + beta_ * speed, dt);
  }

  mvr_vec3 value() const { return x_; }
  mvr_vec3 velocity() const { return dx_; }
  uint64_t time() const { return time_ns_; }

private:
  static constexpr float kDerivativeCutoffHz = 1.0f;
  static float alpha(float cutoff_hz, double dt) {
    const double tau = 1.0 / (2.0 * 3.14159265358979 * cutoff_hz);
    return static_cast<float>(1.0 / (1.0 + tau / dt));
  }

  float min_cutoff_, beta_;
  bool init_ = false;
  uint64_t time_ns_ = 0;
  mvr_vec3 x_{}, dx_{};
};

// Where the tracker's space sits in the application's. Straight ahead and the origin are set on each re-center.
struct TrackingSpace {
  mvr_quat heading = quatIdentity();
  mvr_vec3 origin{}; // in the tracker's space
  mvr_vec3 offset{}; // where that point is in the application's
  mvr_vec3 toApp(mvr_vec3 p) const { return quatRotate(heading, p - origin) + offset; }
};

// The sensors and the tracker behind them. Every call is safe from any thread; while the tracker is closed, or in
// a build without it, a device has no pose and the driver tracks rotation alone.
class RiftSensors {
public:
  enum Device { kHmd = 0, kRightTouch = 1, kLeftTouch = 2, kDevices = 3 };

  static bool built() {
#ifdef METALVR_RIFT_TRACKING
    return true;
#else
    return false;
#endif
  }

#ifdef METALVR_RIFT_TRACKING
  // the tracker's own messages: its warnings and errors always, the rest with --verbose
  static void trackerSays(const char *line) {
    if (std::strncmp(line, "[WW]", 4) == 0 || std::strncmp(line, "[EE]", 4) == 0)
      logInfo("rift tracker: %s", line);
    else
      logDebug("rift tracker: %s", line);
  }
#endif

  // Opens the sensors and starts their cameras, which takes a few seconds. The number of sensors watching.
  int open(const uint8_t radio[5]) {
#ifdef METALVR_RIFT_TRACKING
    mvr_rift_tracker_set_log(trackerSays);
    mvr_rift_tracker *tracker = mvr_rift_tracker_open(radio);
    if (!tracker)
      return 0;
    const int count = mvr_rift_tracker_sensor_count(tracker);
    std::unique_lock<std::shared_mutex> lock(mutex_);
    tracker_ = tracker;
    return count;
#else
    (void)radio;
    return 0;
#endif
  }

  void close() {
#ifdef METALVR_RIFT_TRACKING
    std::unique_lock<std::shared_mutex> lock(mutex_);
    for (int i = 0; i < kDevices; ++i)
      added_[i] = seen_[i] = false;
    if (tracker_)
      mvr_rift_tracker_close(tracker_);
    tracker_ = nullptr;
#endif
  }

  bool running() const {
#ifdef METALVR_RIFT_TRACKING
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return tracker_ != nullptr;
#else
    return false;
#endif
  }

  bool has(int device) const { return added_[device]; }
  // whether a sensor has ever seen the device: before that its position means nothing
  bool seen(int device) const { return seen_[device]; }

  bool add(int device, const std::vector<TrackedLed> &leds, const float imu_position[3], bool flipped) {
#ifdef METALVR_RIFT_TRACKING
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (!tracker_ || added_[device])
      return false;
    std::vector<mvr_rift_led> model(leds.size());
    for (size_t i = 0; i < leds.size(); ++i) {
      std::memcpy(model[i].position, leds[i].position, sizeof(model[i].position));
      std::memcpy(model[i].direction, leds[i].direction, sizeof(model[i].direction));
    }
    if (!mvr_rift_tracker_add_device(tracker_, device, model.data(), static_cast<int>(model.size()), imu_position,
                                     flipped ? 1 : 0))
      return false;
    added_[device] = true;
    return true;
#else
    (void)device, (void)leds, (void)imu_position, (void)flipped;
    return false;
#endif
  }

  // the headset's IMU report with its id in front, 64 bytes
  void hmdReport(const uint8_t *report, int size) {
#ifdef METALVR_RIFT_TRACKING
    if (!added_[kHmd])
      return;
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (tracker_)
      mvr_rift_tracker_hmd_report(tracker_, report, size);
#else
    (void)report, (void)size;
#endif
  }

  void touchImu(int device, uint32_t timestamp_us, const float gyro[3], const float accel[3]) {
#ifdef METALVR_RIFT_TRACKING
    if (!added_[device])
      return;
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (tracker_)
      mvr_rift_tracker_touch_imu(tracker_, device, timestamp_us, gyro, accel);
#else
    (void)device, (void)timestamp_us, (void)gyro, (void)accel;
#endif
  }

  bool pose(int device, SensorPose &out) {
#ifdef METALVR_RIFT_TRACKING
    if (!added_[device])
      return false;
    std::shared_lock<std::shared_mutex> lock(mutex_);
    mvr_rift_pose p;
    if (!tracker_ || !mvr_rift_tracker_pose(tracker_, device, &p))
      return false;
    out.orientation = quatNormalize(mvr_quat{p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]});
    out.position_tracked = p.position_tracked != 0;
    if (out.position_tracked)
      seen_[device] = true;
    // The tracker's position moves with every camera fix, so it is smoothed here, once for each new IMU sample of
    // the device (a pose read twice between samples is the same); its velocity, which the tracker estimates noisily
    // and an application predicts ahead with, becomes the smoothed position's.
    const mvr_vec3 raw{p.position[0], p.position[1], p.position[2]};
    std::lock_guard<std::mutex> smooth(smooth_mutex_);
    OneEuro &filter = smoothed_[device];
    if (p.device_time_ns != filter.time()) {
      filter.update(p.device_time_ns, raw);
      if (trace_)
        std::fprintf(trace_, "%d %llu %d %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", device,
                     static_cast<unsigned long long>(p.device_time_ns), out.position_tracked ? 1 : 0, raw.x, raw.y,
                     raw.z, filter.value().x, filter.value().y, filter.value().z, out.orientation.x,
                     out.orientation.y, out.orientation.z, out.orientation.w);
    }
    out.position = filter.value();
    out.velocity = filter.velocity();
    return true;
#else
    (void)device, (void)out;
    return false;
#endif
  }

  // the sensors as tracking references, in the application's space; how many were written
  uint32_t references(mvr_tracking_reference *out, const TrackingSpace &space) {
    uint32_t count = 0;
#ifdef METALVR_RIFT_TRACKING
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (!tracker_)
      return 0;
    for (int index = 0; count < MVR_MAX_TRACKING_REFERENCES; ++index) {
      mvr_tracking_reference &r = out[count];
      float q[4] = {0, 0, 0, 1}, p[3] = {};
      const int state = mvr_rift_tracker_sensor(tracker_, index, r.serial, q, p);
      if (!state)
        break;
      r.flags = 0;
      r.pose = poseIdentity();
      if (state == 2) {
        r.flags = MVR_REFERENCE_POSE_VALID;
        r.pose.orientation = quatNormalize(quatMul(space.heading, mvr_quat{q[0], q[1], q[2], q[3]}));
        r.pose.position = space.toApp(mvr_vec3{p[0], p[1], p[2]});
      }
      ++count;
    }
#else
    (void)out, (void)space;
#endif
    return count;
  }

  // "12 and 0": the bright spots each camera sees now, which are LEDs when a device is in its view
  std::string spots() {
    std::string text;
#ifdef METALVR_RIFT_TRACKING
    const int count = mvr_rift_camera_count();
    for (int camera = 0; camera < count; ++camera) {
      if (camera)
        text += camera + 1 == count ? " and " : ", ";
      text += std::to_string(mvr_rift_camera_blobs(camera, nullptr));
    }
#endif
    return text;
  }

  // each camera's next picture as <dir>/metalvr-sensor-<n>.png, to see what the sensors see
  void writePictures(const std::string &dir) {
#ifdef METALVR_RIFT_TRACKING
    std::vector<uint8_t> pixels(1280 * 960);
    const int count = mvr_rift_camera_count();
    for (int camera = 0; camera < count; ++camera) {
      int width = 0, height = 0;
      const std::string path = dir + "/metalvr-sensor-" + std::to_string(camera) + ".png";
      if (!mvr_rift_camera_picture(camera, pixels.data(), static_cast<int>(pixels.size()), &width, &height)) {
        logInfo("rift: sensor camera %d gave no picture", camera);
        continue;
      }
      CGColorSpaceRef gray = CGColorSpaceCreateDeviceGray();
      CGContextRef context = CGBitmapContextCreate(pixels.data(), static_cast<size_t>(width),
                                                   static_cast<size_t>(height), 8, static_cast<size_t>(width), gray,
                                                   static_cast<uint32_t>(kCGImageAlphaNone));
      CGImageRef image = context ? CGBitmapContextCreateImage(context) : nullptr;
      CGImageDestinationRef out = image ? CGImageDestinationCreateWithURL(
                                              (__bridge CFURLRef)[NSURL fileURLWithPath:@(path.c_str())],
                                              CFSTR("public.png"), 1, nullptr)
                                        : nullptr;
      bool written = false;
      if (out) {
        CGImageDestinationAddImage(out, image, nullptr);
        written = CGImageDestinationFinalize(out);
        CFRelease(out);
      }
      CGImageRelease(image);
      if (context)
        CGContextRelease(context);
      CGColorSpaceRelease(gray);
      logInfo(written ? "rift: wrote what sensor camera %d sees (%d bright spots) to %s"
                      : "rift: could not write sensor camera %d's picture (%d bright spots) to %s",
              camera, mvr_rift_camera_blobs(camera, nullptr), path.c_str());
    }
#else
    (void)dir;
#endif
  }

  // says when a device comes into the sensors' view or leaves it; called a couple of times a second
  void logChanges() {
    static const char *const names[kDevices] = {"headset", "right Touch controller", "left Touch controller"};
    for (int device = 0; device < kDevices; ++device) {
      SensorPose p;
      if (!pose(device, p) || p.position_tracked == tracked_[device])
        continue;
      tracked_[device] = p.position_tracked;
      if (p.position_tracked)
        logInfo("rift: the sensors see the %s; its position is tracked", names[device]);
      else
        logInfo("rift: the sensors lost sight of the %s; it stays where it was last seen and still turns",
                names[device]);
    }
  }

  // METALVR_RIFT_TRACE names a file that gets one line per new tracker sample of each device: device, its time in
  // ns, whether a sensor sees it, the tracker's position, the smoothed position, and the tracker's orientation
  RiftSensors() {
    if (const char *path = std::getenv("METALVR_RIFT_TRACE"); path && *path) {
      trace_ = std::fopen(path, "w");
      if (trace_)
        std::setvbuf(trace_, nullptr, _IOFBF, 1 << 16);
    }
  }
  ~RiftSensors() {
    if (trace_)
      std::fclose(trace_);
  }

private:
#ifdef METALVR_RIFT_TRACKING
  mutable std::shared_mutex mutex_;
  mvr_rift_tracker *tracker_ = nullptr;
#endif
  std::mutex smooth_mutex_;
  OneEuro smoothed_[kDevices] = {OneEuro(1.5f, 20.0f), OneEuro(2.0f, 20.0f), OneEuro(2.0f, 20.0f)};
  FILE *trace_ = nullptr;
  std::atomic<bool> added_[kDevices] = {};
  std::atomic<bool> seen_[kDevices] = {};
  bool tracked_[kDevices] = {}; // logChanges alone
};

constexpr uint8_t kReportRadioCommand = 26;
constexpr uint8_t kReportRadioData = 27;
constexpr uint8_t kInReportRadio = 12;
constexpr uint8_t kRadioRemote = 1, kRadioLeftTouch = 2, kRadioRightTouch = 3;
constexpr int64_t kTouchGoneNs = 5'000'000'000;
constexpr int64_t kCalibrationRetryNs = 2'000'000'000;
// how fast a device's IMU heading is pulled to the sensors' tracker's: the time constant of the pull
constexpr float kYawPullSeconds = 2.0f;

struct TouchCalibration {
  float gyro_matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, gyro_offset[3] = {};
  float accel_matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, accel_offset[3] = {};
  float joy_x_range[2] = {0, 1023}, joy_x_dead[2] = {500, 524};
  float joy_y_range[2] = {0, 1023}, joy_y_dead[2] = {500, 524};
  float trigger[3] = {0, 512, 1023}, grip[3] = {0, 512, 1023};
  float cap_min[8] = {}, cap_touch[8] = {1, 1, 1, 1, 1, 1, 1, 1};
  // for the sensors: the controller's infrared LEDs and where its IMU sits, in the controller's frame
  std::vector<TrackedLed> leds;
  float imu_position[3] = {};
};

float rangeToUnit(const float r[3], float v) {
  // the raw value goes down as the trigger or grip is pulled
  if (v < r[1])
    return std::clamp(1.0f - (v - r[0]) / std::max(1.0f, r[1] - r[0]) * 0.5f, 0.0f, 1.0f);
  return std::clamp(0.5f - (v - r[1]) / std::max(1.0f, r[2] - r[1]) * 0.5f, 0.0f, 1.0f);
}

float stickAxis(float raw, const float range[2], const float dead[2]) {
  if (raw > dead[1])
    return std::min(1.0f, (raw - dead[1]) / std::max(1.0f, range[1] - dead[1]));
  if (raw < dead[0])
    return std::max(-1.0f, -(dead[0] - raw) / std::max(1.0f, dead[0] - range[0]));
  return 0.0f;
}

void mat3Apply(const float m[9], const float v[3], float out[3]) {
  for (int r = 0; r < 3; ++r)
    out[r] = m[r * 3] * v[0] + m[r * 3 + 1] * v[1] + m[r * 3 + 2] * v[2];
}

class TouchRadio {
public:
  ~TouchRadio() { detach(); }

  // the headset's interface (the radio's address lives there) and the radio's; the radio device is retained
  void attach(IOHIDDeviceRef headset, IOHIDDeviceRef radio) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    headset_ = headset;
    radio_ = radio;
    CFRetain(radio_);
    // asking for the radio's address turns the radio on
    uint8_t reply[64];
    if (command(headset_, 0x05, 0x03, 0x05) && get(headset_, kReportRadioData, reply, sizeof(reply)) >= 8) {
      logInfo("rift: radio %02x:%02x:%02x:%02x:%02x on", reply[3], reply[4], reply[5], reply[6], reply[7]);
      std::memcpy(address_, reply + 3, sizeof(address_));
    } else
      logInfo("rift: the radio did not answer; Touch controllers and the remote are off");
  }

  // the radio's address, which the sensors use to expose in step with the LEDs; zeros when it is not known
  void address(uint8_t out[5]) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    std::memcpy(out, address_, sizeof(address_));
  }

  void setSensors(RiftSensors *sensors) { sensors_ = sensors; }

  void detach() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (radio_)
      CFRelease(radio_);
    radio_ = nullptr;
    headset_ = nullptr;
  }

  // IN report 12, its payload after the id
  void input(const uint8_t *p, size_t length, int64_t now_ns) {
    for (int k = 0; k < 2; ++k) {
      const size_t b = 2 + 28 * k;
      if (b + 28 > length)
        break;
      const uint16_t flags = le16(p + b);
      if (flags != 0x001c && flags != 0x0005)
        continue;
      const uint8_t type = p[b + 2];
      if (type == kRadioRemote)
        remoteInput(le16(p + b + 3));
      else if (type == kRadioLeftTouch || type == kRadioRightTouch)
        touchInput(type == kRadioLeftTouch ? 0 : 1, p + b, now_ns);
    }
  }

  // the radio's slow work: a controller's calibration when it turns up, and haptics; called every few ms
  void service(int64_t now_ns) {
    for (int hand = 0; hand < 2; ++hand) {
      bool need_calibration;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        Hand &h = hands_[hand];
        // A read that failed is tried again every couple of seconds for as long as the controller is on: one
        // that is asleep (put down, not held) sends its reports and does not answer commands until it is picked up.
        const bool may_try = !h.calibration_failed || now_ns - h.calibration_failed_ns > kCalibrationRetryNs;
        need_calibration = h.last_ns && now_ns - h.last_ns < kTouchGoneNs && !h.calibrated && may_try;
      }
      if (need_calibration)
        readCalibration(hand);
      trackWithSensors(hand);
      serviceHaptics(hand, now_ns);
    }
  }

  static int sensorDevice(int hand) { return hand == 0 ? RiftSensors::kLeftTouch : RiftSensors::kRightTouch; }

  // a calibrated controller is given to the sensors' tracker once, when both are there
  void trackWithSensors(int hand) {
    const int device = sensorDevice(hand);
    if (!sensors_ || !sensors_->running() || sensors_->has(device))
      return;
    std::vector<TrackedLed> leds;
    float imu_position[3];
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const Hand &h = hands_[hand];
      if (!h.calibrated || h.sensors_refused)
        return;
      leds = h.cal.leds;
      std::memcpy(imu_position, h.cal.imu_position, sizeof(imu_position));
    }
    const char *name = hand == 0 ? "left" : "right";
    if (leds.size() >= 4 && sensors_->add(device, leds, imu_position, false)) {
      logInfo("rift: the sensors look for the %s Touch controller (%zu LEDs)", name, leds.size());
      return;
    }
    logInfo("rift: the %s Touch controller cannot be tracked by the sensors (%zu LEDs in its calibration); it "
            "keeps its rotation and a place beside the body",
            name, leds.size());
    std::lock_guard<std::mutex> lock(mutex_);
    hands_[hand].sensors_refused = true;
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) {
    if (hand > 1)
      return;
    std::lock_guard<std::mutex> lock(mutex_);
    Hand &h = hands_[hand];
    // shorter or weaker than this cannot be felt
    h.haptic_amplitude = amplitude > 0 ? std::max(amplitude, 0.25f) : 0.0f;
    h.haptic_high = frequency_hz > 240.0f;
    h.haptic_until_ns = nowNs() + static_cast<int64_t>(std::max(duration_s, 0.04f) * 1e9);
  }

  // the controllers' state for the server: body is the head's position with its heading alone
  void fill(mvr_controller_state out[2], const mvr_pose &body, const TrackingSpace &space, int64_t time_ns) {
    const mvr_quat heading = space.heading;
    std::lock_guard<std::mutex> lock(mutex_);
    for (int hand = 0; hand < 2; ++hand) {
      Hand &h = hands_[hand];
      mvr_controller_state &c = out[hand];
      std::memset(&c, 0, sizeof(c));
      if (!h.last_ns || time_ns - h.last_ns > kTouchGoneNs)
        continue;
      c.flags = MVR_CONTROLLER_CONNECTED;
      c.buttons = h.buttons;
      c.touches = h.touches;
      c.trigger = h.trigger;
      c.squeeze = h.grip;
      c.thumbstick_x = h.stick_x;
      c.thumbstick_y = h.stick_y;
      if (h.trigger > 0.75f)
        c.buttons |= MVR_BUTTON_TRIGGER;
      if (h.grip > 0.75f)
        c.buttons |= MVR_BUTTON_SQUEEZE;
      if (!h.calibrated || !h.fusion.ready())
        continue;
      mvr_pose hand_pose;
      SensorPose tracked;
      if (sensors_ && sensors_->pose(sensorDevice(hand), tracked) && sensors_->seen(sensorDevice(hand))) {
        // The sensors have seen it: it is where they put it. Its rotation is its own IMU's, which is smooth, turned
        // to the tracker's heading; the tracker's own rotation takes in every camera fix and shakes as the hand
        // moves, so it only pulls that heading along, slowly, while a sensor sees the controller. Out of their view
        // its position holds and it goes on turning.
        const mvr_quat own = h.fusion.rotation();
        const mvr_quat target = quatYawOnly(quatMul(tracked.orientation, quatConj(own)));
        const int64_t now = nowNs();
        if (!h.on_sensors) {
          h.yaw_align = target;
          h.on_sensors = true;
        } else if (tracked.position_tracked && now > h.align_ns) {
          const float dt = std::min(static_cast<float>(now - h.align_ns) / 1e9f, 0.1f);
          h.yaw_align = yawToward(h.yaw_align, target, 1.0f - std::exp(-dt / kYawPullSeconds));
        }
        h.align_ns = now;
        const mvr_quat turn = h.fusion.ahead(time_ns, nullptr);
        hand_pose.orientation = quatNormalize(quatMul(heading, quatMul(h.yaw_align, quatMul(own, turn))));
        hand_pose.position = space.toApp(tracked.position);
        c.linear_velocity = quatRotate(heading, tracked.velocity);
        c.angular_velocity = quatRotate(hand_pose.orientation, h.fusion.gyro());
        h.aligned = false; // should the sensors go away, the stand-in below lines up with the head afresh
      } else {
        h.on_sensors = false;
        mvr_vec3 angular{};
        mvr_quat q = h.fusion.predict(time_ns, &angular);
        // the controller's heading is set to the head's when it is first tracked (it is held pointing ahead then),
        // and again on each re-center
        if (!h.aligned) {
          const mvr_quat controller_yaw = quatYawOnly(quatMul(heading, q));
          h.yaw_fix = quatMul(quatYawOnly(body.orientation), quatConj(controller_yaw));
          h.aligned = true;
        }
        q = quatNormalize(quatMul(h.yaw_fix, quatMul(heading, q)));
        // Rotation only, no position tracking: the hand is put where an arm would hold a controller that points
        // this way. The elbow rests at the side, a forearm's length behind the controller, and comes up and forward
        // as the controller is raised. Held level it is 40 degrees below straight ahead, inside the headset's
        // view, which ends 48 degrees down; a hand hanging at the side would be out of it.
        const float side = hand == 0 ? -1.0f : 1.0f;
        const mvr_vec3 pointing = quatRotate(quatConj(body.orientation), quatRotate(q, mvr_vec3{0, 0, -1}));
        const float raised = pointing.y > 0 ? pointing.y : 0;
        const mvr_vec3 elbow{side * 0.20f, -0.32f + 0.10f * raised, -0.10f - 0.06f * raised};
        const float forearm = 0.28f;
        const mvr_vec3 in_body{elbow.x + pointing.x * forearm, elbow.y + pointing.y * forearm,
                               elbow.z + pointing.z * forearm};
        hand_pose = mvr_pose{q, quatRotate(body.orientation, in_body) + body.position};
        c.angular_velocity = quatRotate(h.yaw_fix, quatRotate(heading, angular));
      }
      // the grip and the aim from the controller's own frame (RiftLinux rift_radio.c)
      c.grip = poseMul(hand_pose, mvr_pose{mvr_quat{0.5f, 0, 0, 0.8660254f}, {0, -0.03207f, 0.04045f}});
      c.aim = poseMul(hand_pose, mvr_pose{quatIdentity(), {0, -0.00207f, -0.05455f}});
      c.flags |= MVR_CONTROLLER_POSE_VALID;
    }
  }

  // a re-center (the headset put on, or the remote's Oculus button): the controllers' headings follow the head's
  void recenter() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (Hand &h : hands_)
      h.aligned = false;
  }

  bool takeRecenterRequest() { return recenter_requested_.exchange(false); }

private:
  struct Hand {
    int64_t last_ns = 0;
    bool calibrated = false, calibration_failed = false;
    int calibration_attempts = 0;
    int64_t calibration_failed_ns = 0;
    TouchCalibration cal;
    Fusion fusion;
    bool aligned = false;
    bool sensors_refused = false; // its LED model was not one the tracker takes
    // with the sensors: the turn about +y from its own IMU's heading to the tracker's, and when it last moved
    bool on_sensors = false;
    mvr_quat yaw_align = quatIdentity();
    int64_t align_ns = 0;
    mvr_quat yaw_fix = quatIdentity();
    uint32_t buttons = 0, touches = 0;
    float trigger = 0, grip = 0, stick_x = 0, stick_y = 0;
    float cap[8] = {};
    bool stick_touched = false;
    // haptics: what is asked for, and what the controller has
    float haptic_amplitude = 0;
    bool haptic_high = false;
    int64_t haptic_until_ns = 0;
    uint8_t sent_amplitude = 0;
    bool sent_high = false;
  };

  void remoteInput(uint16_t buttons) {
    // the remote's Oculus button re-centers, as it does on Windows
    const bool oculus = buttons & 0x080;
    if (oculus && !remote_oculus_)
      recenter_requested_ = true;
    remote_oculus_ = oculus;
  }

  void touchInput(int hand, const uint8_t *m, int64_t now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    Hand &h = hands_[hand];
    if (!h.last_ns || now_ns - h.last_ns > kTouchGoneNs) {
      logInfo("rift: %s Touch controller is on", hand == 0 ? "left" : "right");
      h.calibration_failed = false;
      h.calibration_attempts = 0;
      h.aligned = false;
    }
    h.last_ns = now_ns;
    const uint8_t buttons = m[19] & 0x0f;
    h.buttons = 0;
    if (buttons & 0x01)
      h.buttons |= MVR_BUTTON_PRIMARY;
    if (buttons & 0x02)
      h.buttons |= MVR_BUTTON_SECONDARY;
    if (buttons & 0x04)
      h.buttons |= hand == 0 ? MVR_BUTTON_MENU : MVR_BUTTON_SYSTEM;
    if (buttons & 0x08)
      h.buttons |= MVR_BUTTON_THUMBSTICK;
    if (!h.calibrated)
      return;
    const TouchCalibration &c = h.cal;
    const uint8_t *t = m + 20;
    const float trigger = static_cast<float>(t[0] | ((t[1] & 0x03) << 8));
    const float grip = static_cast<float>(((t[1] & 0xfc) >> 2) | ((t[2] & 0x0f) << 6));
    const float sx = static_cast<float>(((t[2] & 0xf0) >> 4) | ((t[3] & 0x3f) << 4));
    const float sy = static_cast<float>(((t[3] & 0xc0) >> 6) | (t[4] << 2));
    h.trigger = rangeToUnit(c.trigger, trigger);
    h.grip = rangeToUnit(c.grip, grip);
    float x = stickAxis(sx, c.joy_x_range, c.joy_x_dead), y = stickAxis(sy, c.joy_y_range, c.joy_y_dead);
    // a round dead zone, and the outer edge counted as fully pushed
    const float r = std::sqrt(x * x + y * y);
    if (r < 0.10f) {
      x = y = 0;
    } else {
      const float scaled = std::min(1.0f, (r - 0.10f) / (0.92f - 0.10f));
      x = x / r * scaled;
      y = y / r * scaled;
    }
    h.stick_x = x;
    h.stick_y = y;
    // one touch sensor (or the battery) a message, in turn
    const uint8_t channel = m[25];
    const float adc = static_cast<float>(le16(m + 26));
    const int index = channel == 0x01 ? 0 : channel == 0x02 ? 1 : channel == 0x03 ? 2 : channel == 0x04 ? 3
                                                                                  : channel == 0x08 ? 7
                                                                                                    : -1;
    if (index >= 0)
      h.cap[index] = (adc - c.cap_min[index]) / std::max(1.0f, c.cap_touch[index] - c.cap_min[index]);
    h.touches = 0;
    h.stick_touched = h.cap[0] >= (h.stick_touched ? 0.35f : 0.50f);
    if (h.stick_touched)
      h.touches |= MVR_BUTTON_THUMBSTICK;
    if (h.cap[1] >= 1.0f)
      h.touches |= MVR_BUTTON_SECONDARY;
    if (h.cap[2] >= 1.0f)
      h.touches |= MVR_BUTTON_TRIGGER;
    if (h.cap[3] >= 1.0f)
      h.touches |= MVR_BUTTON_PRIMARY;
    if (h.cap[7] >= 1.0f)
      h.touches |= MVR_BUTTON_THUMBREST;
    // a controller at rest sends its buttons with the timestamp and IMU all zero: no measurement
    const int16_t *accel_raw = nullptr;
    (void)accel_raw;
    int16_t a[3], g[3];
    for (int i = 0; i < 3; ++i) {
      a[i] = static_cast<int16_t>(le16(m + 7 + 2 * i));
      g[i] = static_cast<int16_t>(le16(m + 13 + 2 * i));
    }
    if (!le32(m + 3) && !a[0] && !a[1] && !a[2] && !g[0] && !g[1] && !g[2])
      return;
    float raw_accel[3], raw_gyro[3], accel[3], gyro[3];
    for (int i = 0; i < 3; ++i) {
      raw_accel[i] = 9.80665f / 2048.0f * a[i] - c.accel_offset[i];
      raw_gyro[i] = g[i] / (16.384f * 180.0f) * 3.14159265f - c.gyro_offset[i];
    }
    mat3Apply(c.accel_matrix, raw_accel, accel);
    mat3Apply(c.gyro_matrix, raw_gyro, gyro);
    h.fusion.update(now_ns, accel, gyro);
    if (sensors_)
      sensors_->touchImu(sensorDevice(hand), le32(m + 3), gyro, accel);
  }

  // ---- radio commands (one at a time) ----

  bool set(IOHIDDeviceRef device, uint8_t id, const uint8_t *payload, size_t size) {
    uint8_t buf[69] = {id};
    std::memcpy(buf + 1, payload, std::min(size, sizeof(buf) - 1));
    return device && IOHIDDeviceSetReport(device, kIOHIDReportTypeFeature, id, buf,
                                          static_cast<CFIndex>(std::min(size + 1, sizeof(buf)))) == kIOReturnSuccess;
  }

  // the report with its id at byte 0; its length, or 0
  size_t get(IOHIDDeviceRef device, uint8_t id, uint8_t *out, size_t size) {
    CFIndex length = static_cast<CFIndex>(size);
    out[0] = id;
    if (!device || IOHIDDeviceGetReport(device, kIOHIDReportTypeFeature, id, out, &length) != kIOReturnSuccess)
      return 0;
    return static_cast<size_t>(length);
  }

  // a radio command, waited for: false when the radio or the device did not answer
  bool command(IOHIDDeviceRef device, uint8_t a, uint8_t b, uint8_t c) {
    const uint8_t cmd[5] = {0, 0, a, b, c};
    if (!set(device, kReportRadioCommand, cmd, sizeof(cmd))) {
      last_command_status_ = -2; // the command could not be written
      return false;
    }
    uint8_t reply[64];
    const int64_t deadline = nowNs() + 300'000'000;
    for (;;) {
      if (get(device, kReportRadioCommand, reply, sizeof(reply)) < 4) {
        last_command_status_ = -1; // the report could not be read
        return false;
      }
      last_command_status_ = reply[3];
      if (!(reply[3] & 0x80))
        return !(reply[3] & 0x08);
      if (nowNs() > deadline)
        return false;
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }

  // `length` (at most 20) bytes of a controller's flash at `offset`
  bool readFlash(uint8_t type, uint16_t offset, uint16_t length, uint8_t *out) {
    uint8_t request[34] = {0, 0, static_cast<uint8_t>(offset), static_cast<uint8_t>(offset >> 8),
                           static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8)};
    uint8_t reply[64];
    if (!set(radio_, kReportRadioData, request, sizeof(request)) || !command(radio_, 0x03, 0x0a, type) ||
        get(radio_, kReportRadioData, reply, sizeof(reply)) < 7u + length || le16(reply + 5) != length)
      return false;
    std::memcpy(out, reply + 7, length);
    return true;
  }

  int last_command_status_ = 0; // the radio's answer to the last command, under command_mutex_
  uint8_t address_[5] = {};     // under command_mutex_
  RiftSensors *sensors_ = nullptr;

  static std::string configDir() {
    const char *dir = std::getenv("METALVR_CONFIG_DIR");
    if (dir && *dir)
      return dir;
    return std::string(NSHomeDirectory().UTF8String) + "/.metalsharp/metalvr";
  }

  void readCalibration(int hand) {
    std::lock_guard<std::mutex> radio_lock(command_mutex_);
    if (!radio_)
      return;
    const uint8_t type = hand == 0 ? kRadioLeftTouch : kRadioRightTouch;
    const char *name = hand == 0 ? "left" : "right";
    uint8_t reply[64];
    std::string serial;
    if (command(radio_, 0x03, 0x88, type) && get(radio_, kReportRadioData, reply, sizeof(reply)) >= 26)
      serial.assign(reinterpret_cast<const char *>(reply + 12), strnlen(reinterpret_cast<const char *>(reply + 12), 14));
    const int serial_status = last_command_status_;
    uint8_t hash[16];
    if (serial.empty() || !readFlash(type, 0x1bf0, 16, hash)) {
      // the radio's status byte says why: bit 7 still busy, bit 3 the controller did not answer; -1 and -2 are
      // the USB report itself failing
      // said once for each time the controller comes on, not at every try
      bool first;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        first = hands_[hand].calibration_attempts == 0;
      }
      if (first)
        logInfo("rift: the %s Touch controller does not answer yet (%s, radio status %d then %d): it has buttons "
                "and no pose until it is picked up and its calibration is read",
                name, serial.empty() ? "no serial" : "no hash", serial_status, last_command_status_);
      failCalibration(hand);
      return;
    }
    char hex[33];
    for (int i = 0; i < 16; ++i)
      std::snprintf(hex + 2 * i, 3, "%02x", hash[i]);
    const std::string cache = configDir() + "/touch" + std::to_string(type) + "_" + serial + "_" + hex + ".json";
    NSData *json = [NSData dataWithContentsOfFile:@(cache.c_str())];
    if (!json) {
      uint8_t header[4];
      if (!readFlash(type, 0, 4, header) || header[0] != 1 || header[1] != 0) {
        logInfo("rift: the %s Touch controller's calibration header is not one this driver knows", name);
        failCalibration(hand);
        return;
      }
      const uint16_t size = le16(header + 2);
      std::vector<uint8_t> body(size);
      for (uint16_t off = 0; off < size; off += 20) {
        const uint16_t n = std::min<uint16_t>(20, size - off);
        if (!readFlash(type, static_cast<uint16_t>(4 + off), n, body.data() + off)) {
          logInfo("rift: reading the %s Touch controller's calibration failed at %u of %u bytes", name, off, size);
          failCalibration(hand);
          return;
        }
      }
      json = [NSData dataWithBytes:body.data() length:body.size()];
      [[NSFileManager defaultManager] createDirectoryAtPath:@(configDir().c_str())
                                withIntermediateDirectories:YES
                                                 attributes:nil
                                                      error:nil];
      [json writeToFile:@(cache.c_str()) atomically:YES];
    }
    TouchCalibration cal;
    if (!parseCalibration(json, cal)) {
      logInfo("rift: the %s Touch controller's calibration could not be read as JSON", name);
      failCalibration(hand);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    hands_[hand].cal = cal;
    hands_[hand].calibrated = true;
    logInfo("rift: %s Touch controller %s calibrated", name, serial.c_str());
  }

  void failCalibration(int hand) {
    std::lock_guard<std::mutex> lock(mutex_);
    Hand &h = hands_[hand];
    h.calibration_failed = true;
    h.calibration_failed_ns = nowNs();
    ++h.calibration_attempts;
  }

  static bool numbers(NSDictionary *d, NSString *key, float *out, size_t n) {
    NSArray *a = [d[key] isKindOfClass:[NSArray class]] ? d[key] : nil;
    if (a.count < n)
      return false;
    for (size_t i = 0; i < n; ++i)
      out[i] = [a[i] floatValue];
    return true;
  }

  static bool number(NSDictionary *d, NSString *key, float &out) {
    id v = d[key];
    if (![v isKindOfClass:[NSNumber class]])
      return false;
    out = [v floatValue];
    return true;
  }

  static bool parseCalibration(NSData *json, TouchCalibration &c) {
    NSDictionary *root = [NSJSONSerialization JSONObjectWithData:json options:0 error:nil];
    NSDictionary *o = [root isKindOfClass:[NSDictionary class]] ? root[@"TrackedObject"] : nil;
    if (![o isKindOfClass:[NSDictionary class]])
      return false;
    float g[12], a[12];
    if (numbers(o, @"GyroCalibration", g, 12)) {
      std::memcpy(c.gyro_matrix, g, sizeof(c.gyro_matrix));
      std::memcpy(c.gyro_offset, g + 9, sizeof(c.gyro_offset));
    }
    if (numbers(o, @"AccCalibration", a, 12)) {
      std::memcpy(c.accel_matrix, a, sizeof(c.accel_matrix));
      std::memcpy(c.accel_offset, a + 9, sizeof(c.accel_offset));
    }
    number(o, @"JoyXRangeMin", c.joy_x_range[0]);
    number(o, @"JoyXRangeMax", c.joy_x_range[1]);
    number(o, @"JoyXDeadMin", c.joy_x_dead[0]);
    number(o, @"JoyXDeadMax", c.joy_x_dead[1]);
    number(o, @"JoyYRangeMin", c.joy_y_range[0]);
    number(o, @"JoyYRangeMax", c.joy_y_range[1]);
    number(o, @"JoyYDeadMin", c.joy_y_dead[0]);
    number(o, @"JoyYDeadMax", c.joy_y_dead[1]);
    number(o, @"TriggerMinRange", c.trigger[0]);
    number(o, @"TriggerMidRange", c.trigger[1]);
    number(o, @"TriggerMaxRange", c.trigger[2]);
    number(o, @"MiddleMinRange", c.grip[0]);
    number(o, @"MiddleMidRange", c.grip[1]);
    number(o, @"MiddleMaxRange", c.grip[2]);
    numbers(o, @"CapSenseMin", c.cap_min, 8);
    numbers(o, @"CapSenseTouch", c.cap_touch, 8);
    numbers(o, @"ImuPosition", c.imu_position, 3);
    // {"Point0": [x, y, z, direction x, y, z, ...], ...}
    NSDictionary *points = [o[@"ModelPoints"] isKindOfClass:[NSDictionary class]] ? o[@"ModelPoints"] : nil;
    for (NSUInteger i = 0; i < points.count; ++i) {
      float v[6];
      if (!numbers(points, [NSString stringWithFormat:@"Point%lu", static_cast<unsigned long>(i)], v, 6))
        break;
      c.leds.push_back(TrackedLed{{v[0], v[1], v[2]}, {v[3], v[4], v[5]}});
    }
    return true;
  }

  void serviceHaptics(int hand, int64_t now_ns) {
    uint8_t amplitude;
    bool high;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Hand &h = hands_[hand];
      if (!h.last_ns || now_ns - h.last_ns > kTouchGoneNs)
        return;
      const bool on = h.haptic_amplitude > 0 && now_ns < h.haptic_until_ns;
      amplitude = on ? static_cast<uint8_t>(std::clamp(h.haptic_amplitude, 0.0f, 1.0f) * 255.0f) : 0;
      high = h.haptic_high;
      if (amplitude == h.sent_amplitude && (amplitude == 0 || high == h.sent_high))
        return;
      h.sent_amplitude = amplitude;
      h.sent_high = high;
    }
    std::lock_guard<std::mutex> radio_lock(command_mutex_);
    uint8_t data[30] = {};
    if (amplitude) {
      data[high ? 3 : 2] = 0xa0; // 320 or 160 Hz
      data[6] = amplitude;
    }
    if (!set(radio_, kReportRadioData, data, sizeof(data)) ||
        !command(radio_, 0x02, 0x03, hand == 0 ? kRadioLeftTouch : kRadioRightTouch)) {
      std::lock_guard<std::mutex> lock(mutex_);
      hands_[hand].sent_amplitude = 0xff; // sent again on the next pass
    }
  }

  std::mutex mutex_;         // the hands' state
  std::mutex command_mutex_; // one radio command at a time
  IOHIDDeviceRef headset_ = nullptr;
  IOHIDDeviceRef radio_ = nullptr;
  Hand hands_[2];
  bool remote_oculus_ = false;
  std::atomic<bool> recenter_requested_{false};
};

// ---- the controllers of the headsets other than the CV1: rotation from each one's IMU, placed beside the body ----

class DirectControllers {
public:
  void imu(int hand, int64_t time_ns, const float accel[3], const float gyro[3]) {
    if (hand < 0 || hand > 1)
      return;
    std::lock_guard<std::mutex> lock(mutex_);
    Hand &h = hands_[hand];
    h.fusion.update(time_ns, accel, gyro);
    h.last_ns = time_ns;
    h.has_imu = true;
  }

  void input(int hand, const DirectControllerInput &input, int64_t time_ns) {
    if (hand < 0 || hand > 1)
      return;
    std::lock_guard<std::mutex> lock(mutex_);
    Hand &h = hands_[hand];
    if (!h.last_ns || time_ns - h.last_ns > kTouchGoneNs)
      logInfo("direct: %s controller is on", hand == 0 ? "left" : "right");
    h.input = input;
    h.last_ns = time_ns;
  }

  void grip(int hand, const mvr_pose &grip, const mvr_pose &aim) {
    if (hand < 0 || hand > 1)
      return;
    std::lock_guard<std::mutex> lock(mutex_);
    hands_[hand].grip = grip;
    hands_[hand].aim = aim;
  }

  void recenter() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (Hand &h : hands_)
      h.aligned = false;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (Hand &h : hands_)
      h = Hand{};
  }

  // As TouchRadio::fill: a controller's rotation is its own IMU's. With a tracker's pose (the base stations) it is
  // where the tracker puts it, its heading pulled toward the tracker's; without, its heading is lined up with the
  // head's when it is first seen, and it is placed where an arm would hold a controller pointing that way.
  void fill(mvr_controller_state out[2], const mvr_pose &body, const TrackingSpace &space, int64_t time_ns,
            DirectBackend *tracker = nullptr) {
    const mvr_quat heading = space.heading;
    std::lock_guard<std::mutex> lock(mutex_);
    for (int hand = 0; hand < 2; ++hand) {
      Hand &h = hands_[hand];
      mvr_controller_state &c = out[hand];
      std::memset(&c, 0, sizeof(c));
      if (!h.last_ns || time_ns - h.last_ns > kTouchGoneNs)
        continue;
      const DirectControllerInput &in = h.input;
      c.flags = MVR_CONTROLLER_CONNECTED;
      c.buttons = in.buttons;
      c.touches = in.touches;
      c.trigger = in.trigger;
      c.squeeze = in.squeeze;
      c.thumbstick_x = in.thumbstick_x;
      c.thumbstick_y = in.thumbstick_y;
      c.trackpad_x = in.trackpad_x;
      c.trackpad_y = in.trackpad_y;
      c.trackpad_force = in.trackpad_force;
      c.squeeze_force = in.squeeze_force;
      if (in.has_fingers) {
        c.flags |= MVR_CONTROLLER_FINGERS_VALID;
        std::memcpy(c.fingers, in.fingers, sizeof(c.fingers));
      }
      if (in.has_battery) {
        c.flags |= MVR_CONTROLLER_BATTERY_VALID | (in.charging ? MVR_CONTROLLER_CHARGING : 0u);
        c.battery = in.battery;
      }
      if (c.trigger > 0.75f)
        c.buttons |= MVR_BUTTON_TRIGGER;
      if (c.squeeze > 0.75f)
        c.buttons |= MVR_BUTTON_SQUEEZE;
      if (!h.has_imu || !h.fusion.ready())
        continue;
      TrackedPose tracked;
      if (tracker && tracker->trackedPose(hand == 0 ? LighthouseTracking::kLeft : LighthouseTracking::kRight,
                                          tracked) &&
          tracked.position_tracked) {
        const mvr_quat own = h.fusion.rotation();
        const mvr_quat target = quatYawOnly(quatMul(tracked.orientation, quatConj(own)));
        const int64_t now = nowNs();
        if (!h.on_tracker) {
          h.yaw_align = target;
          h.on_tracker = true;
        } else if (now > h.align_ns) {
          const float dt = std::min(static_cast<float>(now - h.align_ns) / 1e9f, 0.1f);
          h.yaw_align = yawToward(h.yaw_align, target, 1.0f - std::exp(-dt / kYawPullSeconds));
        }
        h.align_ns = now;
        const mvr_quat turn = h.fusion.ahead(time_ns, nullptr);
        const mvr_pose hand_pose{quatNormalize(quatMul(heading, quatMul(h.yaw_align, quatMul(own, turn)))),
                                 space.toApp(tracked.position)};
        c.linear_velocity = quatRotate(heading, tracked.velocity);
        c.angular_velocity = quatRotate(hand_pose.orientation, h.fusion.gyro());
        c.grip = poseMul(hand_pose, h.grip);
        c.aim = poseMul(hand_pose, h.aim);
        c.flags |= MVR_CONTROLLER_POSE_VALID;
        h.aligned = false;
        continue;
      }
      h.on_tracker = false;
      mvr_vec3 angular{};
      mvr_quat q = h.fusion.predict(time_ns, &angular);
      if (!h.aligned) {
        const mvr_quat controller_yaw = quatYawOnly(quatMul(heading, q));
        h.yaw_fix = quatMul(quatYawOnly(body.orientation), quatConj(controller_yaw));
        h.aligned = true;
      }
      q = quatNormalize(quatMul(h.yaw_fix, quatMul(heading, q)));
      const float side = hand == 0 ? -1.0f : 1.0f;
      const mvr_vec3 pointing = quatRotate(quatConj(body.orientation), quatRotate(q, mvr_vec3{0, 0, -1}));
      const float raised = pointing.y > 0 ? pointing.y : 0;
      const mvr_vec3 elbow{side * 0.20f, -0.32f + 0.10f * raised, -0.10f - 0.06f * raised};
      const float forearm = 0.28f;
      const mvr_vec3 in_body{elbow.x + pointing.x * forearm, elbow.y + pointing.y * forearm,
                             elbow.z + pointing.z * forearm};
      const mvr_pose hand_pose{q, quatRotate(body.orientation, in_body) + body.position};
      c.angular_velocity = quatRotate(h.yaw_fix, quatRotate(heading, angular));
      c.grip = poseMul(hand_pose, h.grip);
      c.aim = poseMul(hand_pose, h.aim);
      c.flags |= MVR_CONTROLLER_POSE_VALID;
    }
  }

private:
  struct Hand {
    int64_t last_ns = 0;
    bool has_imu = false;
    Fusion fusion;
    bool aligned = false;
    mvr_quat yaw_fix = quatIdentity();
    bool on_tracker = false;
    mvr_quat yaw_align = quatIdentity();
    int64_t align_ns = 0;
    DirectControllerInput input;
    mvr_pose grip{quatIdentity(), {0, 0, 0}};
    mvr_pose aim{quatIdentity(), {0, 0, 0}};
  };
  std::mutex mutex_;
  Hand hands_[2];
};

// what the driver draws for, before and after the headset is identified
DirectModel cv1Model() {
  DirectModel m;
  m.headset = MVR_HEADSET_RIFT_CV1;
  m.controllers = MVR_CONTROLLERS_TOUCH;
  m.name = "Oculus Rift CV1";
  m.display_vendor = kDisplayVendor;
  m.display_products = {kDisplayProductCV1};
  m.refresh_hz = kRefreshHz;
  m.refresh_rates = {kRefreshHz};
  m.eye_width = 1344;
  m.eye_height = 1600;
  m.fov[0] = kFov[0];
  m.fov[1] = kFov[1];
  return m;
}

// a family whose headset is not identified yet: nothing is assumed of it but a size to render at
DirectModel unidentifiedModel(bool lighthouse) {
  DirectModel m;
  m.name = lighthouse ? "Lighthouse headset (not identified)" : "Rift (not identified)";
  m.controllers = MVR_CONTROLLERS_UNKNOWN;
  m.refresh_hz = 90;
  m.refresh_rates = {90};
  m.eye_width = lighthouse ? 1728 : 1344;
  m.eye_height = lighthouse ? 1920 : 1600;
  m.fov[0] = kFov[0];
  m.fov[1] = kFov[1];
  return m;
}

class RiftDriver final : public Driver, public DirectHost {
public:
  explicit RiftDriver(const RiftOptions &options) : options_(options), model_(unidentifiedModel(options.lighthouse)) {
    space_.offset = {0.0f, options_.head_height, 0.0f};
    touch_.setSensors(&sensors_);
    refresh_hz_ = model_.refresh_hz;
  }
  ~RiftDriver() override { stop(); }

  const char *name() const override { return options_.lighthouse ? "lighthouse" : "rift"; }
  bool displaysFrames() const override { return true; }
  bool needsMainRunLoop() const override { return true; }

  bool start() override {
    if (!setUpGpu())
      return false;
    if (!options_.test_scene) {
      logo_ = loadLogo();
      window_capture_ = makeWindowCapture();
    }

    running_ = true;
    hid_thread_ = std::thread([this] { hidThread(); });
    scene_thread_ = std::thread([this] { sceneThread(); });
    return true;
  }

  // Draws the lens-check grid for `model`, with the head straight ahead, through the same pipeline and uniforms the
  // headset is drawn with, into a panel-sized BGRA picture. For checking the lens models without a headset.
  bool drawTestPanel(const DirectModel &model, uint32_t width, uint32_t height, std::vector<uint8_t> &bgra,
                     bool passthrough = false) {
    if (!device_ && !setUpGpu())
      return false;
    // for passthrough: a stereo camera picture whose left half is red and right half blue
    id<MTLTexture> camera = nil;
    if (passthrough) {
      MTLTextureDescriptor *cd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                                                    width:64
                                                                                   height:32
                                                                                mipmapped:NO];
      camera = [device_ newTextureWithDescriptor:cd];
      std::vector<uint32_t> pixels(64 * 32);
      for (int y = 0; y < 32; ++y)
        for (int x = 0; x < 64; ++x)
          pixels[y * 64 + x] = x < 32 ? 0xffff0000u : 0xff0000ffu; // BGRA in memory: red, blue
      [camera replaceRegion:MTLRegionMake2D(0, 0, 64, 32) mipmapLevel:0 withBytes:pixels.data() bytesPerRow:64 * 4];
    }
    setModel(model);
    MTLTextureDescriptor *td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                                                  width:width
                                                                                 height:height
                                                                              mipmapped:NO];
    td.usage = MTLTextureUsageRenderTarget;
    td.storageMode = MTLStorageModeShared;
    id<MTLTexture> target = [device_ newTextureWithDescriptor:td];
    MTLRenderPassDescriptor *pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = target;
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLCommandBuffer> cmd = [queue_ commandBuffer];
    id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:pass];
    [enc setRenderPipelineState:scene_pipeline_];
    [enc setFragmentTexture:blank_ atIndex:0];
    [enc setFragmentTexture:blank_ atIndex:1];
    [enc setFragmentTexture:(camera ?: blank_) atIndex:2];
    [enc setFragmentSamplerState:sampler_ atIndex:0];
    HomeUniforms home{};
    const float columns[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
    std::memcpy(home.columns, columns, sizeof(columns));
    home.test_scene = passthrough ? 0 : 1;
    if (passthrough) {
      home.passthrough[0] = 1;
      home.passthrough[1] = kPassthroughHalfFovTan;
      home.passthrough[2] = 1;
    }
    [enc setFragmentBytes:&home length:sizeof(home) atIndex:1];
    for (uint32_t eye = 0; eye < 2; ++eye) {
      const MTLViewport viewport = eyeViewport(model, eye, width, height);
      DistortionUniforms u = uniforms(model, eye, viewport);
      LensUniforms lens = lensUniforms(model, eye);
      [enc setViewport:viewport];
      [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
      [enc setFragmentBytes:&lens length:sizeof(lens) atIndex:2];
      [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    }
    [enc endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];
    if (cmd.status != MTLCommandBufferStatusCompleted)
      return false;
    bgra.resize(static_cast<size_t>(width) * height * 4);
    [target getBytes:bgra.data() bytesPerRow:width * 4 fromRegion:MTLRegionMake2D(0, 0, width, height) mipmapLevel:0];
    return true;
  }

private:
  bool setUpGpu() {
    device_ = MTLCreateSystemDefaultDevice();
    if (!device_) {
      logInfo("rift: no Metal device");
      return false;
    }
    queue_ = [device_ newCommandQueue];
    queue_.label = @"metalvr-rift";
    NSError *error = nil;
    id<MTLLibrary> library = [device_ newLibraryWithSource:kShaderSource options:nil error:&error];
    if (!library) {
      logInfo("rift: shader compile failed: %s", error.localizedDescription.UTF8String);
      return false;
    }
    MTLRenderPipelineDescriptor *desc = [MTLRenderPipelineDescriptor new];
    desc.vertexFunction = [library newFunctionWithName:@"fullscreen"];
    desc.fragmentFunction = [library newFunctionWithName:@"distort"];
    desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
    pipeline_ = [device_ newRenderPipelineStateWithDescriptor:desc error:&error];
    desc.fragmentFunction = [library newFunctionWithName:@"scene"];
    scene_pipeline_ = pipeline_ ? [device_ newRenderPipelineStateWithDescriptor:desc error:&error] : nil;
    if (!pipeline_ || !scene_pipeline_) {
      logInfo("rift: pipeline failed: %s", error.localizedDescription.UTF8String);
      return false;
    }
    MTLSamplerDescriptor *sampler = [MTLSamplerDescriptor new];
    sampler.minFilter = MTLSamplerMinMagFilterLinear;
    sampler.magFilter = MTLSamplerMinMagFilterLinear;
    sampler.sAddressMode = MTLSamplerAddressModeClampToEdge;
    sampler.tAddressMode = MTLSamplerAddressModeClampToEdge;
    sampler_ = [device_ newSamplerStateWithDescriptor:sampler];
    {
      MTLTextureDescriptor *td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                    width:1
                                                                                   height:1
                                                                                mipmapped:NO];
      blank_ = [device_ newTextureWithDescriptor:td];
      const uint32_t black = 0;
      [blank_ replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&black bytesPerRow:4];
    }
    return true;
  }

public:

  void stop() override {
    if (!running_.exchange(false))
      return;
    if (hid_loop_)
      CFRunLoopStop(hid_loop_);
    if (hid_thread_.joinable())
      hid_thread_.join();
    if (scene_thread_.joinable())
      scene_thread_.join();
    window_capture_.reset();
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      frame_texture_ = nil;
      if (frame_buffer_)
        CVPixelBufferRelease(frame_buffer_);
      frame_buffer_ = nullptr;
      shown_texture_ = nil;
      if (shown_buffer_)
        CVPixelBufferRelease(shown_buffer_);
      shown_buffer_ = nullptr;
    }
    dispatch_sync_main([this] {
      closeWindow();
      removeScreenObserver();
    });
  }

  bool connected() override { return hid_open_ && fusion_ready_ && layer_ready_; }

  mvr_system_info systemInfo() override {
    const DirectModel m = model();
    mvr_system_info info{};
    info.flags = hid_open_ ? MVR_SYSTEM_HMD_CONNECTED : 0;
    info.eye_width = options_.eye_width ? options_.eye_width : m.eye_width;
    info.eye_height = options_.eye_height ? options_.eye_height : m.eye_height;
    info.refresh_hz = m.refresh_hz;
    info.ipd = ipd();
    info.fov[0] = m.fov[0];
    info.fov[1] = m.fov[1];
    std::snprintf(info.system_name, sizeof(info.system_name), "%s", m.name.c_str());
    info.headset = m.headset;
    info.controllers = m.controllers;
    if (passthrough_.load())
      info.flags |= MVR_SYSTEM_PASSTHROUGH;
    for (float rate : m.refresh_rates)
      if (info.refresh_rate_count < MVR_MAX_REFRESH_RATES)
        info.refresh_rates[info.refresh_rate_count++] = rate;
    return info;
  }

  uint64_t generation() override { return generation_; }

  void tracking(int64_t target_time_ns, mvr_tracking_response &out) override {
    const int64_t time = target_time_ns > 0 ? target_time_ns : nowNs();
    Head head;
    TrackingSpace space;
    {
      std::lock_guard<std::mutex> lock(fusion_mutex_);
      head = headLocked(time);
      space = space_;
    }
    out.flags = hid_open_ ? MVR_TRACKING_HMD_CONNECTED | MVR_TRACKING_HEAD_VALID | MVR_TRACKING_PRESENTING : 0;
    out.sample_id = ++sample_;
    out.time_ns = time;
    out.head.orientation = head.orientation;
    out.head.position = head.position;
    out.head_linear_velocity = head.linear_velocity;
    out.head_angular_velocity = head.angular_velocity;
    const float ipd_m = ipd();
    const DirectModel m = model();
    for (int eye = 0; eye < 2; ++eye) {
      out.views[eye].pose =
          poseMul(out.head, mvr_pose{m.eye_rotation[eye], {(eye == 0 ? -0.5f : 0.5f) * ipd_m, 0, 0}});
      out.views[eye].fov = m.fov[eye];
    }
    // the controllers are carried along with the head's heading, as a body turns with it
    const mvr_pose body{quatYawOnly(out.head.orientation), out.head.position};
    if (DirectBackend *backend = backend_.load())
      direct_controllers_.fill(out.controllers, body, space, time, backend);
    else
      touch_.fill(out.controllers, body, space, time);
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) override {
    if (DirectBackend *backend = backend_.load())
      backend->haptic(hand, duration_s, frequency_hz, amplitude);
    else
      touch_.haptic(hand, duration_s, frequency_hz, amplitude);
  }

  uint32_t trackingReferences(mvr_tracking_reference *out) override {
    TrackingSpace space;
    {
      std::lock_guard<std::mutex> lock(fusion_mutex_);
      space = space_;
    }
    DirectBackend *backend = backend_.load();
    if (!backend)
      return sensors_.references(out, space);
    // the base stations, from the tracker's space to the application's, as the Rift's sensors are
    const uint32_t count = backend->trackingReferences(out);
    for (uint32_t i = 0; i < count && i < MVR_MAX_TRACKING_REFERENCES; ++i)
      if (out[i].flags & MVR_REFERENCE_POSE_VALID) {
        out[i].pose.orientation = quatNormalize(quatMul(space.heading, out[i].pose.orientation));
        out[i].pose.position = space.toApp(out[i].pose.position);
      }
    return count;
  }

  VideoParams videoParams() override {
    const mvr_system_info info = systemInfo();
    VideoParams params;
    params.frame_width = info.eye_width * 2;
    params.frame_height = info.eye_height;
    params.fps = info.refresh_hz;
    return params;
  }

  bool consumeKeyframeRequest() override { return false; }
  void sendVideoConfig(Codec, const uint8_t *, size_t) override {}
  void sendFrame(const FrameInfo &, const uint8_t *, size_t) override {}

  void displayFrame(const FrameInfo &frame, CVPixelBufferRef composed) override {
    if (!composed)
      return;
    IOSurfaceRef surface = CVPixelBufferGetIOSurface(composed);
    if (!surface)
      return;
    MTLTextureDescriptor *td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                                                  width:IOSurfaceGetWidth(surface)
                                                                                 height:IOSurfaceGetHeight(surface)
                                                                              mipmapped:NO];
    td.usage = MTLTextureUsageShaderRead;
    td.storageMode = MTLStorageModeShared;
    id<MTLTexture> texture = [device_ newTextureWithDescriptor:td iosurface:surface plane:0];
    if (!texture)
      return;
    // the display thread shows it from its next refresh on, and again at every refresh until the next one comes,
    // each time turned to where the head points then
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      CVPixelBufferRetain(composed);
      if (frame_buffer_)
        CVPixelBufferRelease(frame_buffer_);
      frame_buffer_ = composed;
      frame_texture_ = texture;
      frame_orientation_ = frame.has_orientation ? frame.head_orientation : mvr_quat{0, 0, 0, 0};
      ++frame_seq_;
      // the application's frame interval, smoothed, for pacing (sceneThread)
      const int64_t arrived = nowNs();
      if (last_app_frame_ns_ && arrived - last_app_frame_ns_ < 250'000'000) {
        const double interval = static_cast<double>(arrived - last_app_frame_ns_);
        frame_interval_ns_ = frame_interval_ns_ > 0 ? frame_interval_ns_ + 0.1 * (interval - frame_interval_ns_) : interval;
      }
    }
    last_app_frame_ns_ = nowNs();
    ++app_frames_;
    const int64_t now = nowNs();
    if (g_verbose && now - last_report_ns_ > 5'000'000'000) {
      if (last_report_ns_) {
        const double seconds = (now - last_report_ns_) / 1e9;
        int pace;
        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          pace = pace_;
        }
        logDebug("rift: %.1f frames/s from the application, each shown for %d refresh%s; %.1f refreshes/s on the "
                 "headset, %.1f ms from the frame's end to its composition",
                 (app_frames_ - last_app_frames_) / seconds, pace, pace == 1 ? "" : "es",
                 (presented_ - last_presented_) / seconds, (frame.composed_ns - frame.submitted_ns) / 1e6);
      }
      last_report_ns_ = now;
      last_app_frames_ = app_frames_;
      last_presented_ = presented_;
    }
  }

private:
  // Draws one frame on the headset: the composed frame through the lens correction, or with no frame the test scene
  // seen with the head's rotation. `keep` is held until the GPU is done with it.
  // `home_head` and `home_space` place the head and the controllers in the home, whose floor is the head's height
  // below its origin.
  void present(CAMetalLayer *layer, id<MTLTexture> texture, const mvr_quat *rotation, CVPixelBufferRef keep,
               id<MTLTexture> window = nil, mvr_vec3 home_head = {}, const TrackingSpace &home_space = {},
               id<MTLTexture> camera = nil, bool camera_stereo = false) {
    std::lock_guard<std::mutex> draw(draw_mutex_);
    id<CAMetalDrawable> drawable = [layer nextDrawable];
    if (!drawable)
      return;

    id<MTLCommandBuffer> cmd = [queue_ commandBuffer];
    MTLRenderPassDescriptor *pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = drawable.texture;
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:pass];
    if (texture) {
      [enc setRenderPipelineState:pipeline_];
      [enc setFragmentTexture:texture atIndex:0];
      [enc setFragmentSamplerState:sampler_ atIndex:0];
    } else {
      [enc setRenderPipelineState:scene_pipeline_];
      [enc setFragmentTexture:(logo_ ?: blank_) atIndex:0];
      [enc setFragmentTexture:(window ?: blank_) atIndex:1];
      [enc setFragmentTexture:(camera ?: blank_) atIndex:2];
      [enc setFragmentSamplerState:sampler_ atIndex:0];
    }
    // the rotation's columns: for the frame, the head now to the head it was rendered for; for the home, head to world
    const mvr_quat q = rotation ? *rotation : quatIdentity();
    const mvr_vec3 x = quatRotate(q, {1, 0, 0}), y = quatRotate(q, {0, 1, 0}), z = quatRotate(q, {0, 0, 1});
    HomeUniforms home{};
    const float columns[3][4] = {{x.x, x.y, x.z, 0}, {y.x, y.y, y.z, 0}, {z.x, z.y, z.z, 0}};
    std::memcpy(home.columns, columns, sizeof(columns));
    if (!texture) {
      const float half_ipd = ipd() * 0.5f;
      for (int eye = 0; eye < 2; ++eye) {
        const mvr_vec3 e = quatRotate(q, {eye == 0 ? -half_ipd : half_ipd, 0, 0});
        home.eye[eye][0] = home_head.x + e.x;
        home.eye[eye][1] = home_head.y + e.y;
        home.eye[eye][2] = home_head.z + e.z;
      }
      if (window && window.width) {
        home.window[0] = 1;
        home.window[1] = static_cast<float>(window.height) / window.width;
      }
      if (logo_) {
        home.logo[0] = 1;
        home.logo[1] = static_cast<float>(logo_.height) / logo_.width;
      }
      home.test_scene = options_.test_scene ? 1 : 0;
      if (camera) {
        home.passthrough[0] = 1;
        home.passthrough[1] = kPassthroughHalfFovTan;
        home.passthrough[2] = camera_stereo ? 1 : 0;
      }
      // the controllers, where the server places them, seen from the head
      mvr_controller_state hands[2];
      const mvr_pose body{quatYawOnly(q), home_head};
      if (DirectBackend *backend = backend_.load())
        direct_controllers_.fill(hands, body, home_space, nowNs(), backend);
      else
        touch_.fill(hands, body, home_space, nowNs());
      for (int hand = 0; hand < 2; ++hand) {
        if (!(hands[hand].flags & MVR_CONTROLLER_POSE_VALID))
          continue;
        // the aim pose is the controller's own frame, which the model is built in
        const mvr_controller_state &c = hands[hand];
        const mvr_pose &aim = c.aim;
        const mvr_vec3 ax = quatRotate(aim.orientation, {1, 0, 0}), ay = quatRotate(aim.orientation, {0, 1, 0});
        const mvr_vec3 az = quatRotate(aim.orientation, {0, 0, 1});
        const float values[6][4] = {{aim.position.x, aim.position.y, aim.position.z, 0},
                                    {ax.x, ax.y, ax.z, 0},
                                    {ay.x, ay.y, ay.z, 0},
                                    {az.x, az.y, az.z, 0},
                                    {c.thumbstick_x, c.thumbstick_y, c.trigger, c.squeeze},
                                    {static_cast<float>(c.buttons), static_cast<float>(c.touches), 1,
                                     hand == 0 ? -1.0f : 1.0f}};
        std::memcpy(home.hand_pos[hand], values[0], sizeof(values[0]));
        std::memcpy(home.hand_x[hand], values[1], sizeof(values[1]));
        std::memcpy(home.hand_y[hand], values[2], sizeof(values[2]));
        std::memcpy(home.hand_z[hand], values[3], sizeof(values[3]));
        std::memcpy(home.hand_in0[hand], values[4], sizeof(values[4]));
        std::memcpy(home.hand_in1[hand], values[5], sizeof(values[5]));
      }
    }
    [enc setFragmentBytes:&home length:sizeof(home) atIndex:1];
    const DirectModel m = model();
    for (uint32_t eye = 0; eye < 2; ++eye) {
      const MTLViewport viewport = eyeViewport(m, eye, drawable.texture.width, drawable.texture.height);
      DistortionUniforms u = uniforms(m, eye, viewport);
      LensUniforms lens = lensUniforms(m, eye);
      [enc setViewport:viewport];
      [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
      [enc setFragmentBytes:&lens length:sizeof(lens) atIndex:2];
      [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    }
    [enc endEncoding];
    if (keep) {
      CVPixelBufferRetain(keep);
      [cmd addCompletedHandler:^(id<MTLCommandBuffer>) { CVPixelBufferRelease(keep); }];
    }
    [cmd addCompletedHandler:^(id<MTLCommandBuffer> done) {
      if (done.status == MTLCommandBufferStatusError) {
        if (!gpu_error_logged_.exchange(true))
          logInfo("rift: drawing a frame failed on the GPU: %s", done.error.localizedDescription.UTF8String);
        return;
      }
      gpu_ns_total_ += static_cast<uint64_t>((done.GPUEndTime - done.GPUStartTime) * 1e9);
      ++gpu_frames_;
    }];
    [cmd presentDrawable:drawable];
    [cmd commit];
    ++presented_;
    // SIGUSR1 while the home shows: the same picture once more into a texture, written as a PNG, to see what the
    // headset shows without wearing it (the picture is as the panel gets it, before the lenses)
    if (!texture && g_snapshot_requested) {
      g_snapshot_requested = 0;
      MTLTextureDescriptor *td =
          [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                             width:drawable.texture.width
                                                            height:drawable.texture.height
                                                         mipmapped:NO];
      td.usage = MTLTextureUsageRenderTarget;
      td.storageMode = MTLStorageModeShared;
      id<MTLTexture> copy = [device_ newTextureWithDescriptor:td];
      MTLRenderPassDescriptor *copy_pass = [MTLRenderPassDescriptor renderPassDescriptor];
      copy_pass.colorAttachments[0].texture = copy;
      copy_pass.colorAttachments[0].loadAction = MTLLoadActionClear;
      copy_pass.colorAttachments[0].storeAction = MTLStoreActionStore;
      id<MTLCommandBuffer> copy_cmd = [queue_ commandBuffer];
      id<MTLRenderCommandEncoder> ce = [copy_cmd renderCommandEncoderWithDescriptor:copy_pass];
      [ce setRenderPipelineState:scene_pipeline_];
      [ce setFragmentTexture:(logo_ ?: blank_) atIndex:0];
      [ce setFragmentTexture:(window ?: blank_) atIndex:1];
      [ce setFragmentTexture:(camera ?: blank_) atIndex:2];
      [ce setFragmentSamplerState:sampler_ atIndex:0];
      [ce setFragmentBytes:&home length:sizeof(home) atIndex:1];
      for (uint32_t eye = 0; eye < 2; ++eye) {
        const MTLViewport viewport = eyeViewport(m, eye, drawable.texture.width, drawable.texture.height);
        DistortionUniforms u = uniforms(m, eye, viewport);
        LensUniforms lens = lensUniforms(m, eye);
        [ce setViewport:viewport];
        [ce setFragmentBytes:&u length:sizeof(u) atIndex:0];
        [ce setFragmentBytes:&lens length:sizeof(lens) atIndex:2];
        [ce drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
      }
      [ce endEncoding];
      [copy_cmd commit];
      [copy_cmd waitUntilCompleted];
      const size_t width = copy.width, height = copy.height;
      std::vector<uint8_t> pixels(width * height * 4);
      [copy getBytes:pixels.data() bytesPerRow:width * 4 fromRegion:MTLRegionMake2D(0, 0, width, height) mipmapLevel:0];
      const std::string path = std::string(NSHomeDirectory().UTF8String) + "/.metalsharp/logs/metalvr-home.png";
      CGColorSpaceRef space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
      CGContextRef context =
          CGBitmapContextCreate(pixels.data(), width, height, 8, width * 4, space,
                                static_cast<uint32_t>(kCGImageAlphaNoneSkipFirst) |
                                    static_cast<uint32_t>(kCGBitmapByteOrder32Little));
      CGImageRef image = context ? CGBitmapContextCreateImage(context) : nullptr;
      CGImageDestinationRef out = image ? CGImageDestinationCreateWithURL(
                                              (__bridge CFURLRef)[NSURL fileURLWithPath:@(path.c_str())],
                                              CFSTR("public.png"), 1, nullptr)
                                        : nullptr;
      bool written = false;
      if (out) {
        CGImageDestinationAddImage(out, image, nullptr);
        written = CGImageDestinationFinalize(out);
        CFRelease(out);
      }
      CGImageRelease(image);
      if (context)
        CGContextRelease(context);
      CGColorSpaceRelease(space);
      logInfo(written ? "rift: wrote the home as the panel shows it to %s" : "rift: could not write %s", path.c_str());
      sensors_.writePictures(std::string(NSHomeDirectory().UTF8String) + "/.metalsharp/logs");
    }
    const int64_t now = nowNs();
    if (g_verbose && now - last_gpu_report_ns_ > 5'000'000'000) {
      if (last_gpu_report_ns_) {
        const uint64_t frames = gpu_frames_.exchange(0), ns = gpu_ns_total_.exchange(0);
        logDebug("rift: %.1f refreshes/s drawn, %.2f ms of GPU each (%s)",
                 (presented_ - last_gpu_presented_) / ((now - last_gpu_report_ns_) / 1e9),
                 frames ? ns / 1e6 / frames : 0.0, texture ? "the application's frame" : "the home");
      }
      last_gpu_report_ns_ = now;
      last_gpu_presented_ = presented_;
    }
  }

  // The headset shows something at every refresh: nextDrawable waits for a drawable the display has let go of,
  // which paces the loop. While an application draws, that is its newest frame turned by how far the head has turned
  // since the frame was rendered, so turning the head stays smooth however slowly the application draws; otherwise
  // the test scene.
  void sceneThread() {
    while (running_) {
      CAMetalLayer *layer = nil;
      {
        std::lock_guard<std::mutex> lock(layer_mutex_);
        layer = layer_;
      }
      if (!layer || !fusion_ready_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      @autoreleasepool {
        // shown about two refreshes from now
        Head head;
        TrackingSpace home_space;
        {
          std::lock_guard<std::mutex> lock(fusion_mutex_);
          head = headLocked(nowNs() + static_cast<int64_t>(2e9 / refresh_hz_.load()));
          home_space = space_;
        }
        const mvr_quat q = head.orientation;
        // the home's origin is where the head is after a re-center
        const mvr_vec3 home_head = head.position - home_space.offset;
        home_space.offset = {0, 0, 0};
        id<MTLTexture> texture = nil;
        CVPixelBufferRef buffer = nullptr;
        mvr_quat rendered{0, 0, 0, 0};
        if (nowNs() - last_app_frame_ns_ < kShowAppFrameNs) {
          // A frame is shown for a whole number of refreshes, the same for every frame: an application that does not
          // keep up with the headset's refresh rate would otherwise have its frames shown for two refreshes, then
          // three, then two, and what moves in them, hands above all, would move unevenly. A new frame waits until
          // the one on show has had its refreshes; the newest is the one taken.
          std::lock_guard<std::mutex> lock(frame_mutex_);
          const int pace = pacing();
          if (!shown_texture_ || (frame_seq_ != shown_seq_ && shown_count_ >= pace)) {
            if (shown_buffer_)
              CVPixelBufferRelease(shown_buffer_);
            shown_texture_ = frame_texture_;
            shown_buffer_ = frame_buffer_;
            if (shown_buffer_)
              CVPixelBufferRetain(shown_buffer_);
            shown_orientation_ = frame_orientation_;
            shown_seq_ = frame_seq_;
            shown_count_ = 0;
          }
          ++shown_count_;
          texture = shown_texture_;
          buffer = shown_buffer_;
          if (buffer)
            CVPixelBufferRetain(buffer);
          rendered = shown_orientation_;
        } else {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          if (shown_buffer_)
            CVPixelBufferRelease(shown_buffer_);
          shown_buffer_ = nullptr;
          shown_texture_ = nil;
          shown_count_ = 0;
          frame_interval_ns_ = 0;
          pace_ = 1;
        }
        if (texture) {
          // a frame without an orientation (quads alone) is shown as it is
          const bool turn = rendered.x != 0 || rendered.y != 0 || rendered.z != 0 || rendered.w != 0;
          const mvr_quat now_to_rendered = turn ? quatNormalize(quatMul(quatConj(rendered), q)) : quatIdentity();
          present(layer, texture, &now_to_rendered, buffer);
        } else {
          // the home, with the window of the game that is starting
          CVPixelBufferRef captured = window_capture_ ? window_capture_->latest() : nullptr;
          id<MTLTexture> window = captured ? textureFor(captured) : nil;
          // the room through the headset's cameras, with --passthrough
          CameraPassthrough *cameras = passthrough_.load();
          CVPixelBufferRef room = cameras ? cameras->latest() : nullptr;
          id<MTLTexture> camera = room ? textureFor(room) : nil;
          present(layer, nil, &q, captured, window, home_head, home_space, camera, cameras && cameras->stereo());
          if (room)
            CVPixelBufferRelease(room);
          if (captured)
            CVPixelBufferRelease(captured);
        }
        if (buffer)
          CVPixelBufferRelease(buffer);
      }
    }
  }

  // How many refreshes each application frame is shown for: the fewest the application keeps up with, from its
  // smoothed frame interval. Going to a faster pace needs the application a little faster than it, so that a rate at
  // the edge does not switch back and forth. METALVR_PACING=off shows every frame as it comes. frame_mutex_ is held.
  int pacing() {
    static const bool off = [] {
      const char *v = std::getenv("METALVR_PACING");
      return v && !std::strcmp(v, "off");
    }();
    if (off || frame_interval_ns_ <= 0)
      return 1;
    const double rate = 1e9 / frame_interval_ns_;
    const double refresh = refresh_hz_.load();
    int target = 4;
    for (int n = 1; n <= 3; ++n)
      if (rate >= refresh / n * 0.92) {
        target = n;
        break;
      }
    if (target < pace_ && rate < refresh / target * 0.97)
      target = pace_;
    pace_ = target;
    return pace_;
  }

  // a captured window's picture as a texture, without a copy
  id<MTLTexture> textureFor(CVPixelBufferRef buffer) {
    IOSurfaceRef surface = CVPixelBufferGetIOSurface(buffer);
    if (!surface)
      return nil;
    MTLTextureDescriptor *td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                                                  width:IOSurfaceGetWidth(surface)
                                                                                 height:IOSurfaceGetHeight(surface)
                                                                              mipmapped:NO];
    td.usage = MTLTextureUsageShaderRead;
    td.storageMode = MTLStorageModeShared;
    return [device_ newTextureWithDescriptor:td iosurface:surface plane:0];
  }

  // the home's logo: --home-logo, or home-logo.png in share/metalvr beside the directory metalvrd is in
  id<MTLTexture> loadLogo() {
    std::string path = options_.home_logo;
    if (path.empty()) {
      char exe[4096];
      uint32_t size = sizeof(exe);
      if (_NSGetExecutablePath(exe, &size) == 0) {
        NSString *dir = [[[NSString stringWithUTF8String:exe] stringByResolvingSymlinksInPath]
            stringByDeletingLastPathComponent];
        path = [[dir stringByAppendingPathComponent:@"../share/metalvr/home-logo.png"] stringByStandardizingPath]
                   .UTF8String;
      }
    }
    if (path.empty() || ![[NSFileManager defaultManager] fileExistsAtPath:@(path.c_str())])
      return nil;
    CGImageSourceRef source =
        CGImageSourceCreateWithURL((__bridge CFURLRef)[NSURL fileURLWithPath:@(path.c_str())], nullptr);
    CGImageRef image = source ? CGImageSourceCreateImageAtIndex(source, 0, nullptr) : nullptr;
    if (source)
      CFRelease(source);
    if (!image) {
      logInfo("rift: cannot read the home logo %s", path.c_str());
      return nil;
    }
    const size_t w = CGImageGetWidth(image), h = CGImageGetHeight(image);
    std::vector<uint8_t> pixels(w * h * 4);
    CGColorSpaceRef space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef context = CGBitmapContextCreate(pixels.data(), w, h, 8, w * 4, space,
                                                 kCGImageAlphaPremultipliedLast);
    CGContextDrawImage(context, CGRectMake(0, 0, w, h), image);
    CGContextRelease(context);
    CGColorSpaceRelease(space);
    CGImageRelease(image);
    MTLTextureDescriptor *td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm_sRGB
                                                                                  width:w
                                                                                 height:h
                                                                              mipmapped:NO];
    id<MTLTexture> texture = [device_ newTextureWithDescriptor:td];
    [texture replaceRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:0 withBytes:pixels.data() bytesPerRow:w * 4];
    logInfo("rift: home logo %s (%zux%zu)", path.c_str(), w, h);
    return texture;
  }

  float ipd() const {
    if (options_.ipd > 0.0f)
      return options_.ipd;
    const float measured = ipd_m_.load();
    if (measured > 0.05f && measured < 0.08f)
      return measured;
    return backend_.load() ? model().ipd : panel_.lens_separation_m;
  }

  DirectModel model() const {
    std::lock_guard<std::mutex> lock(model_mutex_);
    return model_;
  }

  void setModel(const DirectModel &m) {
    {
      std::lock_guard<std::mutex> lock(model_mutex_);
      model_ = m;
    }
    refresh_hz_ = m.refresh_hz > 1 ? m.refresh_hz : 90.0f;
    ++generation_;
  }

  // --- what a backend reports (DirectHost) ---

  void directModel(const DirectModel &m) override {
    setModel(m);
    if (options_.passthrough && !passthrough_owner_) {
      passthrough_owner_ = startCameraPassthrough(m.headset);
      passthrough_ = passthrough_owner_.get();
    }
    logInfo("%s: %s, %ux%u per eye at %.0f Hz", name(), m.name.c_str(), m.eye_width, m.eye_height, m.refresh_hz);
  }

  void directOn(bool on) override {
    if (hid_open_.exchange(on) == on)
      return;
    if (!on)
      direct_controllers_.clear();
    ++generation_;
    // the scripts wait for these lines (scripts/metalsharp/metalvr-rift)
    logInfo(on ? "rift: headset on" : "rift: headset off");
  }

  void directImu(int64_t time_ns, const float accel[3], const float gyro[3]) override {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    fusion_.update(time_ns, accel, gyro);
    settleLocked(time_ns, pending_put_on_.exchange(false));
    ++imu_reports_;
  }

  void directWorn(bool worn) override {
    if (worn && !worn_)
      pending_put_on_ = true;
    worn_ = worn;
  }

  void directIpd(float meters) override { ipd_m_.store(meters); }

  void directControllerImu(int hand, int64_t time_ns, const float accel[3], const float gyro[3]) override {
    direct_controllers_.imu(hand, time_ns, accel, gyro);
  }

  void directControllerInput(int hand, const DirectControllerInput &input, int64_t time_ns) override {
    direct_controllers_.input(hand, input, time_ns);
  }

  void directControllerGrip(int hand, const mvr_pose &grip, const mvr_pose &aim) override {
    direct_controllers_.grip(hand, grip, aim);
  }

  // where an eye is on the panel: the left and right halves, or for a portrait panel (Rift S) the top and bottom
  static MTLViewport eyeViewport(const DirectModel &m, uint32_t eye, double width, double height) {
    if (m.portrait_eyes)
      return MTLViewport{0, eye * height / 2.0, width, height / 2.0, 0, 1};
    return MTLViewport{eye * width / 2.0, 0, width / 2.0, height, 0, 1};
  }

  static LensUniforms lensUniforms(const DirectModel &m, uint32_t eye) {
    const DirectLens &l = m.lens[eye];
    LensUniforms u{};
    u.model = l.model;
    u.portrait = m.portrait_eyes ? 1 : 0;
    u.flip_y = l.flip_y ? 1 : 0;
    const mvr_quat &q = m.eye_rotation[eye];
    const float rotation[4] = {q.x, q.y, q.z, q.w};
    std::memcpy(u.eye_rotation, rotation, sizeof(rotation));
    std::memcpy(u.pano_k, l.pano_k, sizeof(u.pano_k));
    u.pano_k4_ab[0] = l.pano_k[4];
    std::memcpy(u.pano_k4_ab + 1, l.pano_aberration, sizeof(l.pano_aberration));
    u.pano[0] = l.pano_scale;
    u.pano[1] = l.pano_center[0];
    u.pano[2] = l.pano_center[1];
    u.pano_viewport[0] = l.pano_viewport[0];
    u.pano_viewport[1] = l.pano_viewport[1];
    u.vive[0] = l.vive_aspect;
    u.vive[1] = l.vive_grow;
    for (int c = 0; c < 3; ++c) {
      u.vive_center[c][0] = l.vive_center[c][0];
      u.vive_center[c][1] = l.vive_center[c][1];
      std::memcpy(u.vive_coeff[c], l.vive_coefficients[c], sizeof(u.vive_coeff[c]));
    }
    return u;
  }

  DistortionUniforms uniforms(const DirectModel &m, uint32_t eye, const MTLViewport &viewport) const {
    if (m.lens[eye].model != DirectLens::CatmullRomCV1) {
      // the other lens models need the eye's field of view and where it is on the panel (Lens in the shader)
      DistortionUniforms u{};
      const mvr_fov &fov = m.fov[eye];
      u.fov_tan[0] = std::tan(-fov.left);
      u.fov_tan[1] = std::tan(fov.right);
      u.fov_tan[2] = std::tan(fov.up);
      u.fov_tan[3] = std::tan(-fov.down);
      u.eye = eye;
      u.viewport_origin[0] = static_cast<float>(viewport.originX);
      u.viewport_origin[1] = static_cast<float>(viewport.originY);
      u.viewport_size[0] = static_cast<float>(viewport.width);
      u.viewport_size[1] = static_cast<float>(viewport.height);
      return u;
    }
    return uniforms(eye, static_cast<float>(viewport.width), static_cast<float>(viewport.height));
  }

  DistortionUniforms uniforms(uint32_t eye, float viewport_w, float viewport_h) const {
    DistortionUniforms u{};
    // Monado's compute_render_info: the right eye's lens, toward the nose; the shader mirrors it for the left eye
    const float visible_width = panel_.width_m / 2.0f;
    u.tan_scale[0] = panel_.width_m / (4.0f * kMetersPerTanAngle);
    u.tan_scale[1] = panel_.height_m / (2.0f * kMetersPerTanAngle);
    const float from_left = (panel_.width_m - panel_.lens_separation_m) / 2.0f;
    u.lens_center[0] = 2.0f * (from_left / visible_width) - 1.0f;
    u.lens_center[1] = 2.0f * (panel_.center_from_top_m / panel_.height_m) - 1.0f;
    const mvr_fov &fov = kFov[eye];
    u.fov_tan[0] = std::tan(-fov.left);
    u.fov_tan[1] = std::tan(fov.right);
    u.fov_tan[2] = std::tan(fov.up);
    u.fov_tan[3] = std::tan(-fov.down);
    std::memcpy(u.chromatic, kChromatic, sizeof(u.chromatic));
    u.max_r2 = kMaxR * kMaxR;
    u.eye = eye;
    u.viewport_origin[0] = eye * viewport_w;
    u.viewport_origin[1] = 0;
    u.viewport_size[0] = viewport_w;
    u.viewport_size[1] = viewport_h;
    std::memcpy(u.k, kCurve, sizeof(u.k));
    return u;
  }

  // --- the headset's HID interface ---

  struct Head {
    mvr_quat orientation = quatIdentity();
    mvr_vec3 position{}, linear_velocity{}, angular_velocity{};
  };

  // The head at `time_ns` in the application's space; fusion_mutex_ is held. Its rotation and position are the
  // sensors' tracker's once the headset is in it, carried forward to `time_ns` with the newest gyroscope sample;
  // until then, and without sensors, the rotation is this driver's own and the head stays at its seated place.
  Head headLocked(int64_t time_ns) {
    const mvr_quat turn = fusion_.ahead(time_ns, nullptr);
    const mvr_quat own = fusion_.rotation();
    mvr_vec3 raw_position = space_.origin, raw_velocity{};
    SensorPose tracked;
    if (headTracked(tracked)) {
      // The head's rotation stays this driver's own, from the IMU, which is smooth; the tracker's takes in every
      // camera fix and shakes as the head turns. Its heading is turned to the tracker's, and pulled along slowly
      // while a sensor sees the headset, which takes out the IMU's drift.
      const mvr_quat target = quatYawOnly(quatMul(tracked.orientation, quatConj(own)));
      const int64_t now = nowNs();
      if (!head_on_sensors_) {
        // straight ahead stays where it was
        const mvr_quat before = quatYawOnly(quatMul(space_.heading, quatMul(yaw_align_, own)));
        yaw_align_ = target;
        space_.heading = quatNormalize(quatMul(before, quatConj(quatYawOnly(quatMul(yaw_align_, own)))));
        head_on_sensors_ = true;
        touch_.recenter();
        logInfo("rift: the sensors' tracker places the head; its heading follows the tracker's");
      } else if (tracked.position_tracked && now > align_ns_) {
        const float dt = std::min(static_cast<float>(now - align_ns_) / 1e9f, 0.1f);
        yaw_align_ = yawToward(yaw_align_, target, 1.0f - std::exp(-dt / kYawPullSeconds));
      }
      align_ns_ = now;
      if (tracked.position_tracked && !head_seen_) {
        // where a sensor first sees the headset is where the seated head was taken to be
        head_seen_ = true;
        space_.origin = tracked.position;
      }
      if (head_seen_) {
        raw_position = tracked.position;
        raw_velocity = tracked.velocity;
      }
    } else if (head_on_sensors_) {
      // the tracker went away: back to this driver's own heading, facing the same way
      space_.heading = quatConj(quatYawOnly(own));
      yaw_align_ = quatIdentity();
      head_on_sensors_ = head_seen_ = false;
      touch_.recenter();
      logInfo("rift: the sensors' tracker is gone; rotation only, facing ahead");
    }
    Head head;
    head.orientation = quatNormalize(quatMul(space_.heading, quatMul(yaw_align_, quatMul(own, turn))));
    head.position = space_.toApp(raw_position);
    head.linear_velocity = quatRotate(space_.heading, raw_velocity);
    head.angular_velocity = quatRotate(head.orientation, fusion_.gyro());
    return head;
  }

  // where a position tracker has the head: the Rift's sensors, or a backend's (the base stations)
  bool headTracked(SensorPose &out) {
    if (DirectBackend *backend = backend_.load()) {
      TrackedPose p;
      if (!backend->trackedPose(LighthouseTracking::kHead, p))
        return false;
      out.orientation = p.orientation;
      out.position = p.position;
      out.velocity = p.velocity;
      out.position_tracked = p.position_tracked;
      return true;
    }
    return sensors_.pose(RiftSensors::kHmd, out);
  }

  // Straight ahead becomes where the headset faces, and where it is becomes the seated head's place;
  // fusion_mutex_ is held.
  void recenterLocked(int64_t now_ns) {
    SensorPose tracked;
    if (head_on_sensors_ && headTracked(tracked)) {
      space_.heading = quatConj(quatYawOnly(quatMul(yaw_align_, fusion_.rotation())));
      if (head_seen_)
        space_.origin = tracked.position;
    } else {
      space_.heading = quatConj(quatYawOnly(fusion_.predict(now_ns, nullptr)));
    }
    ++generation_;
    touch_.recenter();
    direct_controllers_.recenter();
  }

  // The headset's infrared LEDs, and where its IMU sits, from its position reports: one LED (or the IMU) a read,
  // in turn, until the first comes round again. The LEDs at the back of the strap are left out, as OpenHMD does:
  // the strap moves against the front, and a sensor that sees only them would place the head wrong.
  bool readLeds(std::vector<TrackedLed> &leds, float imu_position[3]) {
    int first = -1;
    for (int i = 0; i < 128; ++i) {
      uint8_t p[40] = {};
      if (getFeature(kReportPositionInfo, p, sizeof(p)) < 26)
        break;
      // {command 2, flags 1, position um 4+4+4, direction 2+2+2, 2, index 1, 1, count 1, ...}
      const int index = p[23];
      if (index == first)
        break;
      if (first < 0)
        first = index;
      float position[3], direction[3];
      for (int a = 0; a < 3; ++a) {
        position[a] = static_cast<float>(static_cast<int32_t>(le32(p + 3 + 4 * a))) / 1e6f;
        direction[a] = static_cast<float>(static_cast<int16_t>(le16(p + 15 + 2 * a)));
      }
      if (p[2] == 1)
        std::memcpy(imu_position, position, sizeof(position));
      else if (p[2] == 2 && position[2] >= -0.1f)
        leds.push_back(TrackedLed{{position[0], position[1], position[2]}, {direction[0], direction[1], direction[2]}});
    }
    return leds.size() >= 4;
  }

  // Lights the headset's LEDs for the sensors and starts the sensors' tracker, which opens the cameras on a
  // thread of its own because that takes seconds.
  void startSensors() {
    if (!RiftSensors::built()) {
      logInfo("rift: built without the sensors' tracker; rotation only");
      return;
    }
    // {command 2, pattern, flags: on | carrier, 0, exposure us 2, period us 2, vsync offset 2, duty cycle}: every
    // LED lit at each exposure of the cameras
    const uint8_t tracking[12] = {0, 0, 0xff, 0x01 | 0x04, 0, 399 & 0xff, 399 >> 8, 19200 & 0xff, 19200 >> 8, 0, 0, 0x7f};
    if (!feature(kReportTrackingConfig, tracking, sizeof(tracking)))
      logInfo("rift: the headset did not take the LED configuration; the sensors may not see it");
    std::vector<TrackedLed> leds;
    float imu_position[3] = {};
    if (!readLeds(leds, imu_position)) {
      logInfo("rift: the headset did not report its LEDs (%zu read); rotation only", leds.size());
      return;
    }
    if (g_verbose) {
      logDebug("rift: the headset's IMU is at %.4f %.4f %.4f m; its LEDs (position m, direction):", imu_position[0],
               imu_position[1], imu_position[2]);
      for (const TrackedLed &led : leds)
        logDebug("rift:   %7.4f %7.4f %7.4f   %6.0f %6.0f %6.0f", led.position[0], led.position[1], led.position[2],
                 led.direction[0], led.direction[1], led.direction[2]);
    }
    uint8_t radio[5] = {};
    touch_.address(radio);
    sensor_thread_ = std::thread([this, leds, radio, imu = std::array<float, 3>{imu_position[0], imu_position[1],
                                                                                   imu_position[2]}] {
      const int count = sensors_.open(radio);
      if (!count) {
        logInfo("rift: no Rift sensor could be opened; rotation only");
        return;
      }
      if (!sensors_.add(RiftSensors::kHmd, leds, imu.data(), true)) {
        logInfo("rift: the sensors' tracker did not take the headset's %zu LEDs; rotation only", leds.size());
        return;
      }
      logInfo("rift: %d sensor%s watching for the headset's %zu LEDs", count, count == 1 ? "" : "s", leds.size());
    });
  }

  bool feature(uint8_t id, const uint8_t *payload, size_t size) {
    uint8_t buf[69] = {id};
    std::memcpy(buf + 1, payload, size);
    return IOHIDDeviceSetReport(hid_, kIOHIDReportTypeFeature, id, buf, static_cast<CFIndex>(size + 1)) ==
           kIOReturnSuccess;
  }

  // the report's payload after its id, or 0 bytes
  size_t getFeature(uint8_t id, uint8_t *out, size_t size) {
    uint8_t buf[69] = {id};
    CFIndex length = sizeof(buf);
    if (IOHIDDeviceGetReport(hid_, kIOHIDReportTypeFeature, id, buf, &length) != kIOReturnSuccess || length < 2)
      return 0;
    const uint8_t *data = buf[0] == id ? buf + 1 : buf;
    const size_t n = std::min(size, static_cast<size_t>(length - (data - buf)));
    std::memcpy(out, data, n);
    return n;
  }

  void keepAlive() {
    // keep-alive mux: {command 0, IN report 11, interval in ms}
    const uint8_t report[5] = {0, 0, kInReportImu, kKeepAliveIntervalMs & 0xff, kKeepAliveIntervalMs >> 8};
    if (!feature(kReportKeepAliveMux, report, sizeof(report)))
      logInfo("rift: keep-alive failed");
  }

  void readDisplayInfo() {
    uint8_t info[54] = {};
    if (getFeature(kReportDisplayInfo, info, sizeof(info)) < 23) {
      logInfo("rift: no display info; using the CV1's defaults");
      return;
    }
    // {command 2, distortion type 1, resolution 2+2, size um 4+4, center_v um 4, lens separation um 4, ...}
    Panel p;
    p.width_px = le16(info + 3);
    p.height_px = le16(info + 5);
    p.width_m = le32(info + 7) / 1e6f;
    p.height_m = le32(info + 11) / 1e6f;
    p.center_from_top_m = le32(info + 15) / 1e6f;
    p.lens_separation_m = le32(info + 19) / 1e6f;
    if (p.width_px && p.height_px && p.width_m > 0.05f && p.height_m > 0.03f && p.lens_separation_m > 0.04f)
      panel_ = p;
    logInfo("rift: panel %ux%u, %.1f x %.1f mm, lenses %.1f mm apart, %.1f mm from the top", panel_.width_px,
            panel_.height_px, panel_.width_m * 1e3f, panel_.height_m * 1e3f, panel_.lens_separation_m * 1e3f,
            panel_.center_from_top_m * 1e3f);
  }

  void configure() {
    uint8_t config[6] = {};
    getFeature(kReportConfig, config, sizeof(config));
    // {command 2, flags 1, interval 1, sample rate 2}: the headset calibrates its IMU, keeps alive on commands and
    // motion, and reports every sample
    config[0] = config[1] = 0;
    config[2] = kConfigUseCalibration | kConfigAutoCalibration | kConfigCommandKeepAlive | kConfigMotionKeepAlive;
    config[3] = 0;
    if (!feature(kReportConfig, config, sizeof(config)))
      logInfo("rift: config failed");
  }

  static void onInput(void *context, IOReturn result, void *, IOHIDReportType, uint32_t id, uint8_t *report,
                      CFIndex length) {
    if (result == kIOReturnSuccess)
      static_cast<RiftDriver *>(context)->input(id, report, length);
  }

  void input(uint32_t id, const uint8_t *report, CFIndex length) {
    const int64_t received = nowNs() - kUsbLatencyNs;
    ++input_reports_;
    last_input_id_ = id;
    last_input_length_ = static_cast<int>(length);
    if (id == kInReportImu) {
      // the sensors' tracker reads the report whole, with its id in front
      if (length == 64 && report[0] == id) {
        sensors_.hmdReport(report, 64);
      } else if (length == 63) {
        uint8_t whole[64] = {kInReportImu};
        std::memcpy(whole + 1, report, 63);
        sensors_.hmdReport(whole, 64);
      }
    }
    if (length > 0 && report[0] == id) {
      ++report;
      --length;
    }
    if (id == kInReportRadio) {
      touch_.input(report, static_cast<size_t>(length), received);
      return;
    }
    if (id != kInReportImu || length < 63)
      return;
    ++imu_reports_;
    // {command 2, samples 1, sample count 2, temperature 2, timestamp 4, two {accel 8, gyro 8}, presence 2, IAD 2, ...}
    const uint8_t samples = std::min<uint8_t>(report[2], 2);
    const uint16_t iad = le16(report + 45);
    // the proximity sensor inside the face cushion: the headset just went on a face
    const bool worn = le16(report + 43) > 3;
    const bool put_on = worn && !worn_;
    worn_ = worn;
    ipd_m_.store((std::round(((iad - 400) / 300.0f + 59.0f) * 2.0f) / 2.0f) / 1000.0f);
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    for (int i = 0; i < samples; ++i) {
      float accel[3], gyro[3];
      unpackSample(report + 11 + i * 16, 0.0001f, accel);
      unpackSample(report + 11 + i * 16 + 8, 0.0001f, gyro);
      fusion_.update(received - (samples - 1 - i) * kSampleNs, accel, gyro);
    }
    settleLocked(received, put_on);
  }

  // After new samples of the head: what the headset faces as it starts, and again each time it is put on, is
  // straight ahead. fusion_mutex_ is held.
  void settleLocked(int64_t now_ns, bool put_on) {
    if ((!fusion_ready_ || put_on) && fusion_.ready()) {
      recenterLocked(now_ns);
      if (!fusion_ready_)
        logInfo("%s: tracking head rotation", name());
      else
        logInfo("%s: the headset was put on; straight ahead is where it faces", name());
      fusion_ready_ = true;
    }
  }

  // Which headset of the family is on USB, then that headset's own loop until the driver stops. A Lighthouse
  // headset and a Rift S have a backend of their own (direct_backend.h); the CV1 is driven here.
  void hidThread() {
    hid_loop_ = CFRunLoopGetCurrent();
    if (options_.lighthouse) {
      LighthouseOptions lighthouse;
      lighthouse.pro2_mode = options_.pro2_mode;
      runBackend(makeLighthouseBackend(lighthouse));
      return;
    }
    bool told = false;
    while (running_) {
      // a Rift S (2833:0051) or a CV1 (2833:0031); nothing else of Oculus's is a headset here
      IOHIDManagerRef manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
      IOHIDManagerSetDeviceMatching(manager, (__bridge CFDictionaryRef) @{@kIOHIDVendorIDKey : @(kVendor)});
      IOHIDManagerOpen(manager, kIOHIDOptionsTypeNone);
      bool cv1 = false, rift_s = false;
      if (CFSetRef set = IOHIDManagerCopyDevices(manager)) {
        std::vector<const void *> values(CFSetGetCount(set));
        CFSetGetValues(set, values.data());
        for (const void *v : values) {
          const int product = intProperty(static_cast<IOHIDDeviceRef>(const_cast<void *>(v)), CFSTR(kIOHIDProductIDKey));
          cv1 = cv1 || product == kProductCV1;
          rift_s = rift_s || product == rift_s::kProduct;
        }
        CFRelease(set);
      }
      IOHIDManagerClose(manager, kIOHIDOptionsTypeNone);
      CFRelease(manager);
      if (cv1) {
        setModel(cv1Model());
        cv1Thread();
        return;
      }
      if (rift_s) {
        runBackend(makeRiftSBackend());
        return;
      }
      if (!told) {
        told = true;
        logInfo("rift: waiting for a Rift CV1 (2833:0031) or a Rift S (2833:0051) on USB");
      }
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, false);
    }
  }

  // A headset with a backend: the backend's loop on this thread, while this thread also watches its display.
  void runBackend(std::unique_ptr<DirectBackend> backend) {
    DirectBackend *b = backend.get();
    backend_owner_ = std::move(backend);
    backend_ = b;
    std::thread watcher([this] {
      while (running_) {
        watchDisplay();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    });
    b->run(*this, running_);
    watcher.join();
    hid_open_ = false;
    ++generation_;
  }

  static int intProperty(IOHIDDeviceRef device, CFStringRef key) {
    int value = 0;
    CFTypeRef p = IOHIDDeviceGetProperty(device, key);
    if (p && CFGetTypeID(p) == CFNumberGetTypeID())
      CFNumberGetValue(static_cast<CFNumberRef>(p), kCFNumberIntType, &value);
    return value;
  }

  // the Rift CV1: its HID interfaces, sensors and radio, driven by this driver itself
  void cv1Thread() {
    IOHIDManagerRef manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    NSDictionary *match = @{@kIOHIDVendorIDKey : @(kVendor), @kIOHIDProductIDKey : @(kProductCV1)};
    IOHIDManagerSetDeviceMatching(manager, (__bridge CFDictionaryRef)match);
    IOHIDManagerOpen(manager, kIOHIDOptionsTypeNone);
    while (running_ && !hid_) {
      CFSetRef set = IOHIDManagerCopyDevices(manager);
      if (set && CFSetGetCount(set)) {
        std::vector<const void *> values(CFSetGetCount(set));
        CFSetGetValues(set, values.data());
        std::vector<IOHIDDeviceRef> devices;
        for (const void *v : values)
          devices.push_back(static_cast<IOHIDDeviceRef>(const_cast<void *>(v)));
        // the headset's own interface is USB interface 0, which the IMU reports come in on; the radio's (the Touch
        // controllers and the remote) is interface 1
        std::sort(devices.begin(), devices.end(),
                  [](IOHIDDeviceRef a, IOHIDDeviceRef b) { return interfaceNumber(a) < interfaceNumber(b); });
        for (IOHIDDeviceRef dev : devices) {
          if (IOHIDDeviceOpen(dev, kIOHIDOptionsTypeNone) != kIOReturnSuccess)
            continue;
          hid_ = dev;
          uint8_t probe[54];
          if (getFeature(kReportDisplayInfo, probe, sizeof(probe)) >= 23) {
            CFRetain(dev);
            logDebug("rift: the headset's HID interface is USB interface %d", interfaceNumber(dev));
            break;
          }
          IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
          hid_ = nullptr;
        }
      }
      if (set)
        CFRelease(set);
      if (!hid_) {
        static bool told = false;
        if (!told) {
          told = true;
          logInfo("rift: waiting for a Rift CV1 on USB (2833:0031)");
        }
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0, false);
      }
    }
    if (!hid_) {
      IOHIDManagerClose(manager, kIOHIDOptionsTypeNone);
      CFRelease(manager);
      return;
    }

    // the radio: the same headset's other HID interface
    {
      CFSetRef set = IOHIDManagerCopyDevices(manager);
      if (set) {
        std::vector<const void *> values(CFSetGetCount(set));
        CFSetGetValues(set, values.data());
        for (const void *v : values) {
          IOHIDDeviceRef dev = static_cast<IOHIDDeviceRef>(const_cast<void *>(v));
          if (dev == hid_ || interfaceNumber(dev) != 1 || IOHIDDeviceOpen(dev, kIOHIDOptionsTypeNone) != kIOReturnSuccess)
            continue;
          radio_ = dev;
          CFRetain(radio_);
          break;
        }
        CFRelease(set);
      }
    }

    readDisplayInfo();
    configure();
    keepAlive();
    const uint8_t enable[3] = {0, 0, kComponentDisplay | kComponentAudio | kComponentLeds};
    if (!feature(kReportEnableComponents, enable, sizeof(enable)))
      logInfo("rift: turning the display on failed");
    hid_open_ = true;
    ++generation_;
    logInfo("rift: headset on");

    static uint8_t report_buffer[128];
    IOHIDDeviceRegisterInputReportCallback(hid_, report_buffer, sizeof(report_buffer), onInput, this);
    IOHIDDeviceScheduleWithRunLoop(hid_, hid_loop_, kCFRunLoopDefaultMode);
    static uint8_t radio_buffer[128];
    if (radio_) {
      touch_.attach(hid_, radio_);
      IOHIDDeviceRegisterInputReportCallback(radio_, radio_buffer, sizeof(radio_buffer), onInput, this);
      IOHIDDeviceScheduleWithRunLoop(radio_, hid_loop_, kCFRunLoopDefaultMode);
      radio_thread_ = std::thread([this] {
        while (running_ && hid_open_) {
          touch_.service(nowNs());
          if (touch_.takeRecenterRequest()) {
            // the remote's Oculus button: straight ahead is where the headset faces now
            std::lock_guard<std::mutex> lock(fusion_mutex_);
            recenterLocked(nowNs());
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
      });
    } else {
      logInfo("rift: no radio interface; Touch controllers and the remote are off");
    }
    startSensors();

    double last_keep_alive = CFAbsoluteTimeGetCurrent();
    while (running_) {
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.5, false);
      if (CFAbsoluteTimeGetCurrent() - last_keep_alive > kKeepAliveSendS) {
        keepAlive();
        last_keep_alive = CFAbsoluteTimeGetCurrent();
      }
      watchDisplay();
      sensors_.logChanges();
      if (g_verbose && CFAbsoluteTimeGetCurrent() - last_imu_log_ > 5.0) {
        last_imu_log_ = CFAbsoluteTimeGetCurrent();
        mvr_quat q;
        mvr_vec3 at;
        {
          std::lock_guard<std::mutex> lock(fusion_mutex_);
          const Head head = headLocked(nowNs());
          q = head.orientation;
          at = head.position;
        }
        // yaw about +y, pitch about +x, roll about +z, in degrees
        const float yaw = std::atan2(2 * (q.w * q.y + q.x * q.z), 1 - 2 * (q.y * q.y + q.x * q.x)) * 57.2958f;
        const float pitch = std::asin(std::clamp(2 * (q.w * q.x - q.y * q.z), -1.0f, 1.0f)) * 57.2958f;
        const float roll = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.x * q.x + q.z * q.z)) * 57.2958f;
        logDebug("rift: %llu input reports (%llu IMU) in 5 s, last id %u length %d; head yaw %.1f pitch %.1f roll %.1f "
                 "at %.2f %.2f %.2f m",
                 static_cast<unsigned long long>(input_reports_.exchange(0)),
                 static_cast<unsigned long long>(imu_reports_.exchange(0)), last_input_id_.load(),
                 last_input_length_.load(), yaw, pitch, roll, at.x, at.y, at.z);
        if (sensors_.running())
          logDebug("rift: the sensors' cameras see %s bright spots; the headset is %s", sensors_.spots().c_str(),
                   sensors_.seen(RiftSensors::kHmd) ? "tracked in position" : "not seen yet");
#ifdef METALVR_RIFT_TRACKING
        if (sensors_.running()) {
          char timing[200];
          mvr_rift_tracker_timing(timing, sizeof(timing));
          logDebug("rift: %s", timing);
        }
#endif
      }
    }

    IOHIDDeviceUnscheduleFromRunLoop(hid_, hid_loop_, kCFRunLoopDefaultMode);
    const uint8_t off[3] = {0, 0, 0};
    feature(kReportEnableComponents, off, sizeof(off));
    hid_open_ = false;
    if (radio_thread_.joinable())
      radio_thread_.join();
    // the cameras stop and the sensors are let go before the headset is
    if (sensor_thread_.joinable())
      sensor_thread_.join();
    sensors_.close();
    {
      std::lock_guard<std::mutex> lock(fusion_mutex_);
      head_on_sensors_ = head_seen_ = false;
    }
    if (radio_) {
      IOHIDDeviceUnscheduleFromRunLoop(radio_, hid_loop_, kCFRunLoopDefaultMode);
      touch_.detach();
      IOHIDDeviceClose(radio_, kIOHIDOptionsTypeNone);
      CFRelease(radio_);
      radio_ = nullptr;
    }
    IOHIDDeviceClose(hid_, kIOHIDOptionsTypeNone);
    CFRelease(hid_);
    hid_ = nullptr;
    IOHIDManagerClose(manager, kIOHIDOptionsTypeNone);
    CFRelease(manager);
  }

  // the USB interface a HID device is, from the registry above it; large when unknown
  static int interfaceNumber(IOHIDDeviceRef dev) {
    int number = 99;
    io_service_t service = IOHIDDeviceGetService(dev);
    CFTypeRef value = service ? IORegistryEntrySearchCFProperty(service, kIOServicePlane, CFSTR("bInterfaceNumber"),
                                                                kCFAllocatorDefault,
                                                                kIORegistryIterateRecursively | kIORegistryIterateParents)
                              : nullptr;
    if (value) {
      if (CFGetTypeID(value) == CFNumberGetTypeID())
        CFNumberGetValue(static_cast<CFNumberRef>(value), kCFNumberIntType, &number);
      CFRelease(value);
    }
    return number;
  }

  // --- the headset's display ---

  // the headset's display, by the vendor and product its EDID gives: the model's, once it is identified
  CGDirectDisplayID findDisplay() {
    const DirectModel m = model();
    if (!m.display_vendor)
      return kCGNullDirectDisplay;
    CGDirectDisplayID ids[16];
    uint32_t count = 0;
    CGGetOnlineDisplayList(16, ids, &count);
    for (uint32_t i = 0; i < count; ++i) {
      if (CGDisplayVendorNumber(ids[i]) != m.display_vendor)
        continue;
      for (uint32_t product : m.display_products)
        if (CGDisplayModelNumber(ids[i]) == product)
          return ids[i];
    }
    return kCGNullDirectDisplay;
  }

  void watchDisplay() {
    const CGDirectDisplayID display = findDisplay();
    if (display == display_) {
      // the headset's display can move on the desktop (another display made the main one, or the arrangement
      // changed): the window goes where it is now, so that it never covers another display
      if (display != kCGNullDirectDisplay)
        dispatch_async(dispatch_get_main_queue(), ^{ followScreen(); });
      return;
    }
    display_ = display;
    if (display == kCGNullDirectDisplay) {
      logInfo("rift: the headset's display went away");
      dispatch_async(dispatch_get_main_queue(), ^{ closeWindow(); });
      return;
    }
    static bool told = false;
    if (!told) {
      told = true;
      logInfo("rift: display %u is the headset's", display);
    }
    dispatch_async(dispatch_get_main_queue(), ^{ openWindow(display); });
  }

  static NSScreen *screenFor(CGDirectDisplayID display) {
    for (NSScreen *s in NSScreen.screens) {
      NSNumber *number = s.deviceDescription[@"NSScreenNumber"];
      if (number.unsignedIntValue == display)
        return s;
    }
    return nil;
  }

  // Keeps the window over the headset's display, wherever that is on the desktop now: macOS moves every display's
  // frame when another becomes the main one, and a window left where the headset's display was would cover
  // whatever display is there now. With no screen for the headset, the window goes until it has one again.
  void followScreen() {
    if (!window_)
      return;
    NSScreen *screen = screenFor(display_);
    if (!screen) {
      logInfo("rift: the headset's display has no screen; the window waits for it");
      closeWindow();
      display_ = kCGNullDirectDisplay; // watchDisplay opens it again
      return;
    }
    if (NSEqualRects(window_.frame, screen.frame))
      return;
    [window_ setFrame:screen.frame display:YES];
    CAMetalLayer *layer = static_cast<CAMetalLayer *>(window_.contentView.layer);
    layer.contentsScale = screen.backingScaleFactor;
    layer.drawableSize = CGSizeMake(screen.frame.size.width * screen.backingScaleFactor,
                                    screen.frame.size.height * screen.backingScaleFactor);
    logInfo("rift: the headset's display moved on the desktop; the window followed it");
  }

  void openWindow(CGDirectDisplayID display) {
    closeWindow();
    if (!screen_observer_)
      screen_observer_ = [NSNotificationCenter.defaultCenter
          addObserverForName:NSApplicationDidChangeScreenParametersNotification
                      object:nil
                       queue:NSOperationQueue.mainQueue
                  usingBlock:^(NSNotification *) { followScreen(); }];
    NSScreen *screen = screenFor(display);
    if (!screen) {
      logInfo("rift: display %u has no screen yet", display);
      display_ = kCGNullDirectDisplay; // look again
      return;
    }
    useDisplayMode(display);
    NSWindow *window = [[NSWindow alloc] initWithContentRect:screen.frame
                                                   styleMask:NSWindowStyleMaskBorderless
                                                     backing:NSBackingStoreBuffered
                                                       defer:NO
                                                      screen:screen];
    window.level = NSScreenSaverWindowLevel;
    window.opaque = YES;
    window.backgroundColor = NSColor.blackColor;
    window.ignoresMouseEvents = YES;
    window.hasShadow = NO;
    window.collectionBehavior = NSWindowCollectionBehaviorCanJoinAllSpaces | NSWindowCollectionBehaviorStationary |
                                NSWindowCollectionBehaviorFullScreenAuxiliary | NSWindowCollectionBehaviorIgnoresCycle;
    [window setFrame:screen.frame display:NO];
    CAMetalLayer *layer = [CAMetalLayer layer];
    layer.device = device_;
    layer.pixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
    layer.framebufferOnly = YES;
    layer.opaque = YES;
    layer.displaySyncEnabled = YES;
    layer.maximumDrawableCount = 3;
    layer.contentsScale = screen.backingScaleFactor;
    layer.drawableSize = CGSizeMake(screen.frame.size.width * screen.backingScaleFactor,
                                    screen.frame.size.height * screen.backingScaleFactor);
    NSView *view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, screen.frame.size.width, screen.frame.size.height)];
    view.wantsLayer = YES;
    view.layer = layer;
    window.contentView = view;
    [window orderFrontRegardless];
    window_ = window;
    {
      std::lock_guard<std::mutex> lock(layer_mutex_);
      layer_ = layer;
    }
    layer_ready_ = true;
    ++generation_;
    logInfo("rift: drawing on the headset's display, %.0fx%.0f", layer.drawableSize.width, layer.drawableSize.height);
  }

  // The refresh rate asked for (--refresh), if the display has a mode of its size at it; then the rate the display
  // really runs at becomes the headset's, which frames are paced to. The CV1 stays at its 90 Hz.
  void useDisplayMode(CGDirectDisplayID display) {
    if (!backend_.load())
      return;
    if (options_.refresh_hz > 0) {
      CGDisplayModeRef current = CGDisplayCopyDisplayMode(display);
      CFArrayRef modes = CGDisplayCopyAllDisplayModes(display, nullptr);
      CGDisplayModeRef chosen = nullptr;
      for (CFIndex i = 0; modes && current && i < CFArrayGetCount(modes); ++i) {
        CGDisplayModeRef mode = static_cast<CGDisplayModeRef>(const_cast<void *>(CFArrayGetValueAtIndex(modes, i)));
        if (CGDisplayModeGetPixelWidth(mode) == CGDisplayModeGetPixelWidth(current) &&
            CGDisplayModeGetPixelHeight(mode) == CGDisplayModeGetPixelHeight(current) &&
            std::fabs(CGDisplayModeGetRefreshRate(mode) - options_.refresh_hz) < 1.0)
          chosen = mode;
      }
      if (!chosen)
        logInfo("rift: the headset's display has no %.0f Hz mode at its size; it stays as it is", options_.refresh_hz);
      else if (CGDisplaySetDisplayMode(display, chosen, nullptr) != kCGErrorSuccess)
        logInfo("rift: macOS did not switch the headset's display to %.0f Hz", options_.refresh_hz);
      if (modes)
        CFRelease(modes);
      if (current)
        CGDisplayModeRelease(current);
    }
    CGDisplayModeRef mode = CGDisplayCopyDisplayMode(display);
    const double hz = mode ? CGDisplayModeGetRefreshRate(mode) : 0;
    if (mode)
      CGDisplayModeRelease(mode);
    DirectModel m = model();
    if (hz > 30 && std::fabs(hz - m.refresh_hz) > 0.5) {
      logInfo("rift: the headset's display runs at %.2f Hz", hz);
      m.refresh_hz = static_cast<float>(hz);
      if (std::none_of(m.refresh_rates.begin(), m.refresh_rates.end(),
                       [&](float r) { return std::fabs(r - hz) < 0.5; }))
        m.refresh_rates.push_back(m.refresh_hz);
      setModel(m);
    }
  }

  void closeWindow() {
    layer_ready_ = false;
    {
      std::lock_guard<std::mutex> lock(layer_mutex_);
      layer_ = nil;
    }
    [window_ orderOut:nil];
    window_ = nil;
  }

  void removeScreenObserver() {
    if (screen_observer_)
      [NSNotificationCenter.defaultCenter removeObserver:screen_observer_];
    screen_observer_ = nil;
  }

  template <typename F> static void dispatch_sync_main(F f) {
    if ([NSThread isMainThread])
      f();
    else
      dispatch_sync(dispatch_get_main_queue(), ^{ f(); });
  }

  RiftOptions options_;
  // the headset drawn for: the CV1's, a backend's (direct_backend.h), or nothing assumed until one is identified
  mutable std::mutex model_mutex_;
  DirectModel model_;
  std::atomic<float> refresh_hz_{90.0f};
  std::unique_ptr<DirectBackend> backend_owner_; // the USB thread's, set once
  std::atomic<DirectBackend *> backend_{nullptr};
  DirectControllers direct_controllers_;
  std::unique_ptr<CameraPassthrough> passthrough_owner_; // the USB thread's, set once
  std::atomic<CameraPassthrough *> passthrough_{nullptr};
  std::atomic<bool> pending_put_on_{false};
  Panel panel_;
  id<MTLDevice> device_ = nil;
  id<MTLCommandQueue> queue_ = nil;
  id<MTLRenderPipelineState> pipeline_ = nil;
  id<MTLRenderPipelineState> scene_pipeline_ = nil;
  std::mutex draw_mutex_;
  std::thread scene_thread_;
  std::atomic<int64_t> last_app_frame_ns_{0};
  id<MTLSamplerState> sampler_ = nil;
  id<MTLTexture> logo_ = nil;  // the home's logo, if there is one
  id<MTLTexture> blank_ = nil; // bound where there is nothing to show
  std::unique_ptr<WindowCapture> window_capture_;
  // the application's newest composed frame, shown at every refresh until the next comes; after a second without
  // one (a session that ended, or an application that stopped drawing) the headset goes back to the test scene
  static constexpr int64_t kShowAppFrameNs = 1'000'000'000;
  std::mutex frame_mutex_;
  id<MTLTexture> frame_texture_ = nil;
  CVPixelBufferRef frame_buffer_ = nullptr;
  mvr_quat frame_orientation_{0, 0, 0, 0}; // zero: the frame has none
  uint64_t frame_seq_ = 0;                 // counts the application's frames
  double frame_interval_ns_ = 0;           // their interval, smoothed; 0 until two have come
  // the frame on show and how many refreshes it has been shown for, and the pace (sceneThread, under frame_mutex_)
  id<MTLTexture> shown_texture_ = nil;
  CVPixelBufferRef shown_buffer_ = nullptr;
  mvr_quat shown_orientation_{0, 0, 0, 0};
  uint64_t shown_seq_ = 0;
  int shown_count_ = 0;
  int pace_ = 1;
  std::atomic<uint64_t> app_frames_{0};
  uint64_t last_app_frames_ = 0;

  std::atomic<bool> running_{false};
  std::thread hid_thread_;
  IOHIDDeviceRef radio_ = nullptr; // the headset's radio interface: Touch controllers and the remote
  TouchRadio touch_;
  std::thread radio_thread_;
  CFRunLoopRef hid_loop_ = nullptr;
  IOHIDDeviceRef hid_ = nullptr;
  std::atomic<bool> hid_open_{false};
  std::atomic<float> ipd_m_{0.0f};
  bool worn_ = false;
  std::atomic<uint64_t> input_reports_{0}, imu_reports_{0};
  std::atomic<uint32_t> last_input_id_{0};
  std::atomic<int> last_input_length_{0};
  double last_imu_log_ = 0;

  std::mutex fusion_mutex_;
  Fusion fusion_;
  std::atomic<bool> fusion_ready_{false};
  // fusion_mutex_: where the tracking space sits in the application's, and whether the head is the sensors'
  TrackingSpace space_;
  bool head_on_sensors_ = false; // the head's rotation comes from the sensors' tracker
  bool head_seen_ = false;       // a sensor has seen the headset, so its position means something
  mvr_quat yaw_align_ = quatIdentity(); // the turn about +y from the IMU's heading to the tracker's
  int64_t align_ns_ = 0;
  RiftSensors sensors_;
  std::thread sensor_thread_;
  std::atomic<uint64_t> generation_{1};
  std::atomic<uint64_t> sample_{0};

  CGDirectDisplayID display_ = kCGNullDirectDisplay;
  NSWindow *window_ = nil;
  id screen_observer_ = nil;
  std::mutex layer_mutex_;
  CAMetalLayer *layer_ = nil;
  std::atomic<bool> layer_ready_{false};

  std::atomic<uint64_t> presented_{0};
  std::atomic<bool> gpu_error_logged_{false};
  std::atomic<uint64_t> gpu_ns_total_{0}, gpu_frames_{0};
  int64_t last_gpu_report_ns_ = 0; // draw_mutex_
  uint64_t last_gpu_presented_ = 0;
  uint64_t last_presented_ = 0;
  int64_t last_report_ns_ = 0;
};

} // namespace

std::unique_ptr<Driver> makeRiftDriver(const RiftOptions &options) { return std::make_unique<RiftDriver>(options); }

bool drawTestPanel(const DirectModel &model, uint32_t width, uint32_t height, std::vector<uint8_t> &bgra,
                   bool passthrough) {
  RiftOptions options;
  options.test_scene = true;
  RiftDriver driver(options);
  return driver.drawTestPanel(model, width, height, bgra, passthrough);
}

DirectModel riftCv1Model() { return cv1Model(); }

} // namespace metalvr
