// The Oculus Rift CV1 on the Mac's own video output and USB (docs/RIFT.md).
//
// The headset's HID interface (2833:0031) turns its display, audio and LEDs on and reports its IMU at 1000 Hz; the
// head's rotation comes from those samples, and composed frames are drawn on the headset's display, which macOS
// shows as an ordinary display once its head-mounted-display override is lifted, with the CV1's lens distortion.
// Head rotation only: the head stays where a seated user's would be.
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
#include <chrono>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include "common.h"
#include "driver.h"
#include "window_capture.h"
#include "metalvr_math.h"

namespace metalvr {
namespace {

constexpr int kVendor = 0x2833;
constexpr int kProductCV1 = 0x0031;
constexpr uint32_t kDisplayVendor = 0x3ed2; // "OVR" in the EDID
constexpr uint32_t kDisplayProductCV1 = 0x0004;

// feature reports
constexpr uint8_t kReportConfig = 2;
constexpr uint8_t kReportDisplayInfo = 9;
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

// The direction each colour channel of this pixel shows, as a tangent from the eye's axis (x right, y down): the
// lens bends red, green and blue apart, so each is looked up where the lens takes it.
static void channel_tangents(constant Distortion &d, float2 position, thread float2 out[3]) {
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
                        constant SceneRotation &rotation [[buffer(1)]], texture2d<float> frame [[texture(0)]],
                        sampler s [[sampler(0)]]) {
  float2 t[3];
  channel_tangents(d, in.position.xy, t);
  float3x3 now_to_rendered = float3x3(rotation.columns[0].xyz, rotation.columns[1].xyz, rotation.columns[2].xyz);
  float3 color;
  for (int c = 0; c < 3; ++c) {
    // The direction this pixel shows, as the head is now, seen from the head the frame was rendered for: the frame
    // is turned by how far the head has turned since. The eye looks down -z with +y up; tangent y runs down.
    float3 v = now_to_rendered * float3(t[c].x, -t[c].y, -1.0);
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

fragment float4 scene(VOut in [[stage_in]], constant Distortion &d [[buffer(0)]], constant Home &home [[buffer(1)]],
                      texture2d<float> logo [[texture(0)]], texture2d<float> window [[texture(1)]],
                      sampler s [[sampler(0)]]) {
  float2 t[3];
  channel_tangents(d, in.position.xy, t);
  float3x3 head_to_world = float3x3(home.columns[0].xyz, home.columns[1].xyz, home.columns[2].xyz);
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
  const float3 background = space_color(normalize(head_to_world * float3(t[1].x, -t[1].y, -1.0)));
  for (int c = 0; c < 3; ++c) {
    float3 dir = normalize(head_to_world * float3(t[c].x, -t[c].y, -1.0));
    float4 panel = panels(origin, dir, home, logo, window, s);
    color[c] = mix(background[c], panel[c], panel.a);
  }
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

struct HomeUniforms {
  float columns[3][4]; // head to world
  float eye[2][4];     // each eye's position in the world, meters
  float window[4];     // 1 when a window is shown, its height over its width
  float logo[4];       // 1 when there is a logo, its height over its width
  uint32_t test_scene;
  uint32_t pad[3];
};
static_assert(sizeof(HomeUniforms) == 128, "the shader's Home struct");

// ---- the Touch controllers and the remote, through the headset's radio ----
//
// The radio is the headset's USB interface 1. Each controller sends IN report 12 at about 500 Hz (two 28-byte
// messages a report); commands go out as feature reports 26 (the command, then polled until done) and 27 (its data).
// A controller's calibration (its IMU matrices, stick, trigger and grip ranges, touch thresholds) is JSON in its own
// flash, read over the radio once and cached by the flash's hash. The layout and the decoding follow RiftLinux's
// rift_radio.c (Monado's Rift driver, BSL-1.0; Beyley Cardellio, Jan Schmidt, Aditya Mehra and others).

constexpr uint8_t kReportRadioCommand = 26;
constexpr uint8_t kReportRadioData = 27;
constexpr uint8_t kInReportRadio = 12;
constexpr uint8_t kRadioRemote = 1, kRadioLeftTouch = 2, kRadioRightTouch = 3;
constexpr int64_t kTouchGoneNs = 5'000'000'000;

struct TouchCalibration {
  float gyro_matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, gyro_offset[3] = {};
  float accel_matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, accel_offset[3] = {};
  float joy_x_range[2] = {0, 1023}, joy_x_dead[2] = {500, 524};
  float joy_y_range[2] = {0, 1023}, joy_y_dead[2] = {500, 524};
  float trigger[3] = {0, 512, 1023}, grip[3] = {0, 512, 1023};
  float cap_min[8] = {}, cap_touch[8] = {1, 1, 1, 1, 1, 1, 1, 1};
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
    if (command(headset_, 0x05, 0x03, 0x05) && get(headset_, kReportRadioData, reply, sizeof(reply)) >= 8)
      logInfo("rift: radio %02x:%02x:%02x:%02x:%02x on", reply[3], reply[4], reply[5], reply[6], reply[7]);
    else
      logInfo("rift: the radio did not answer; Touch controllers and the remote are off");
  }

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
        need_calibration = h.last_ns && now_ns - h.last_ns < kTouchGoneNs && !h.calibrated && !h.calibration_failed;
      }
      if (need_calibration)
        readCalibration(hand);
      serviceHaptics(hand, now_ns);
    }
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
  void fill(mvr_controller_state out[2], const mvr_pose &body, mvr_quat heading, int64_t time_ns) {
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
      // held in front of the chest, as an arm in a relaxed pose holds it: rotation only, no position tracking
      const float side = hand == 0 ? -1.0f : 1.0f;
      const mvr_pose hand_pose{q, quatRotate(body.orientation, mvr_vec3{side * 0.18f, -0.45f, -0.35f}) +
                                      body.position};
      // the grip and the aim from the controller's own frame (RiftLinux rift_radio.c)
      c.grip = poseMul(hand_pose, mvr_pose{mvr_quat{0.5f, 0, 0, 0.8660254f}, {0, -0.03207f, 0.04045f}});
      c.aim = poseMul(hand_pose, mvr_pose{quatIdentity(), {0, -0.00207f, -0.05455f}});
      c.angular_velocity = quatRotate(h.yaw_fix, quatRotate(heading, angular));
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
    TouchCalibration cal;
    Fusion fusion;
    bool aligned = false;
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
    if (!set(device, kReportRadioCommand, cmd, sizeof(cmd)))
      return false;
    uint8_t reply[64];
    const int64_t deadline = nowNs() + 300'000'000;
    for (;;) {
      if (get(device, kReportRadioCommand, reply, sizeof(reply)) < 4)
        return false;
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
    uint8_t hash[16];
    if (serial.empty() || !readFlash(type, 0x1bf0, 16, hash)) {
      logInfo("rift: the %s Touch controller's serial or calibration hash could not be read", name);
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
    hands_[hand].calibration_failed = true;
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

class RiftDriver final : public Driver {
public:
  explicit RiftDriver(const RiftOptions &options) : options_(options) {}
  ~RiftDriver() override { stop(); }

  const char *name() const override { return "rift"; }
  bool displaysFrames() const override { return true; }
  bool needsMainRunLoop() const override { return true; }

  bool start() override {
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
    if (!options_.test_scene) {
      logo_ = loadLogo();
      window_capture_ = makeWindowCapture();
    }

    running_ = true;
    hid_thread_ = std::thread([this] { hidThread(); });
    scene_thread_ = std::thread([this] { sceneThread(); });
    return true;
  }

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
    }
    dispatch_sync_main([this] {
      closeWindow();
      removeScreenObserver();
    });
  }

  bool connected() override { return hid_open_ && fusion_ready_ && layer_ready_; }

  mvr_system_info systemInfo() override {
    mvr_system_info info{};
    info.flags = hid_open_ ? MVR_SYSTEM_HMD_CONNECTED : 0;
    info.eye_width = options_.eye_width;
    info.eye_height = options_.eye_height;
    info.refresh_hz = kRefreshHz;
    info.ipd = ipd();
    info.fov[0] = kFov[0];
    info.fov[1] = kFov[1];
    std::snprintf(info.system_name, sizeof(info.system_name), "Oculus Rift CV1");
    return info;
  }

  uint64_t generation() override { return generation_; }

  void tracking(int64_t target_time_ns, mvr_tracking_response &out) override {
    const int64_t time = target_time_ns > 0 ? target_time_ns : nowNs();
    mvr_vec3 angular{};
    mvr_quat rot;
    {
      std::lock_guard<std::mutex> lock(fusion_mutex_);
      rot = fusion_.predict(time, &angular);
    }
    // the heading the headset had when it started is straight ahead
    rot = quatNormalize(quatMul(heading_, rot));
    out.flags = hid_open_ ? MVR_TRACKING_HMD_CONNECTED | MVR_TRACKING_HEAD_VALID | MVR_TRACKING_PRESENTING : 0;
    out.sample_id = ++sample_;
    out.time_ns = time;
    out.head.orientation = rot;
    out.head.position = {0.0f, options_.head_height, 0.0f};
    out.head_angular_velocity = quatRotate(heading_, angular);
    const float ipd_m = ipd();
    for (int eye = 0; eye < 2; ++eye) {
      out.views[eye].pose = poseMul(out.head, mvr_pose{quatIdentity(), {(eye == 0 ? -0.5f : 0.5f) * ipd_m, 0, 0}});
      out.views[eye].fov = kFov[eye];
    }
    // the controllers are carried along with the head's heading, as a body turns with it
    const mvr_pose body{quatYawOnly(out.head.orientation), out.head.position};
    touch_.fill(out.controllers, body, heading_, time);
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) override {
    touch_.haptic(hand, duration_s, frequency_hz, amplitude);
  }

  VideoParams videoParams() override {
    VideoParams params;
    params.frame_width = options_.eye_width * 2;
    params.frame_height = options_.eye_height;
    params.fps = kRefreshHz;
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
    }
    last_app_frame_ns_ = nowNs();
    ++app_frames_;
    const int64_t now = nowNs();
    if (g_verbose && now - last_report_ns_ > 5'000'000'000) {
      if (last_report_ns_) {
        const double seconds = (now - last_report_ns_) / 1e9;
        logDebug("rift: %.1f frames/s from the application, %.1f refreshes/s on the headset, %.1f ms from the "
                 "frame's end to its composition",
                 (app_frames_ - last_app_frames_) / seconds, (presented_ - last_presented_) / seconds,
                 (frame.composed_ns - frame.submitted_ns) / 1e6);
      }
      last_report_ns_ = now;
      last_app_frames_ = app_frames_;
      last_presented_ = presented_;
    }
  }

private:
  // Draws one frame on the headset: the composed frame through the lens correction, or with no frame the test scene
  // seen with the head's rotation. `keep` is held until the GPU is done with it.
  void present(CAMetalLayer *layer, id<MTLTexture> texture, const mvr_quat *rotation, CVPixelBufferRef keep,
               id<MTLTexture> window = nil) {
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
        home.eye[eye][0] = e.x;
        home.eye[eye][1] = e.y;
        home.eye[eye][2] = e.z;
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
    }
    [enc setFragmentBytes:&home length:sizeof(home) atIndex:1];
    const float half_w = drawable.texture.width / 2.0f, h = drawable.texture.height;
    for (uint32_t eye = 0; eye < 2; ++eye) {
      DistortionUniforms u = uniforms(eye, half_w, h);
      [enc setViewport:MTLViewport{eye * half_w, 0, half_w, h, 0, 1}];
      [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
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
        mvr_quat q;
        {
          std::lock_guard<std::mutex> lock(fusion_mutex_);
          q = fusion_.predict(nowNs() + static_cast<int64_t>(2e9 / kRefreshHz), nullptr);
        }
        q = quatNormalize(quatMul(heading_, q));
        id<MTLTexture> texture = nil;
        CVPixelBufferRef buffer = nullptr;
        mvr_quat rendered{0, 0, 0, 0};
        if (nowNs() - last_app_frame_ns_ < kShowAppFrameNs) {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          texture = frame_texture_;
          buffer = frame_buffer_;
          if (buffer)
            CVPixelBufferRetain(buffer);
          rendered = frame_orientation_;
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
          present(layer, nil, &q, captured, window);
          if (captured)
            CVPixelBufferRelease(captured);
        }
        if (buffer)
          CVPixelBufferRelease(buffer);
      }
    }
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
    return measured > 0.05f && measured < 0.08f ? measured : panel_.lens_separation_m;
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
    if ((!fusion_ready_ || put_on) && fusion_.ready()) {
      // what the headset faces as it starts, and again each time it is put on, is straight ahead
      heading_ = quatConj(quatYawOnly(fusion_.predict(received, nullptr)));
      ++generation_;
      if (!fusion_ready_)
        logInfo("rift: tracking head rotation");
      else
        logInfo("rift: the headset was put on; straight ahead is where it faces");
      touch_.recenter();
      fusion_ready_ = true;
    }
  }

  void hidThread() {
    hid_loop_ = CFRunLoopGetCurrent();
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
            heading_ = quatConj(quatYawOnly(fusion_.predict(nowNs(), nullptr)));
            ++generation_;
            touch_.recenter();
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
      });
    } else {
      logInfo("rift: no radio interface; Touch controllers and the remote are off");
    }

    double last_keep_alive = CFAbsoluteTimeGetCurrent();
    while (running_) {
      CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.5, false);
      if (CFAbsoluteTimeGetCurrent() - last_keep_alive > kKeepAliveSendS) {
        keepAlive();
        last_keep_alive = CFAbsoluteTimeGetCurrent();
      }
      watchDisplay();
      if (g_verbose && CFAbsoluteTimeGetCurrent() - last_imu_log_ > 5.0) {
        last_imu_log_ = CFAbsoluteTimeGetCurrent();
        mvr_quat q;
        {
          std::lock_guard<std::mutex> lock(fusion_mutex_);
          q = fusion_.predict(nowNs(), nullptr);
        }
        q = quatNormalize(quatMul(heading_, q));
        // yaw about +y, pitch about +x, roll about +z, in degrees
        const float yaw = std::atan2(2 * (q.w * q.y + q.x * q.z), 1 - 2 * (q.y * q.y + q.x * q.x)) * 57.2958f;
        const float pitch = std::asin(std::clamp(2 * (q.w * q.x - q.y * q.z), -1.0f, 1.0f)) * 57.2958f;
        const float roll = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.x * q.x + q.z * q.z)) * 57.2958f;
        logDebug("rift: %llu input reports (%llu IMU) in 5 s, last id %u length %d; head yaw %.1f pitch %.1f roll %.1f",
                 static_cast<unsigned long long>(input_reports_.exchange(0)),
                 static_cast<unsigned long long>(imu_reports_.exchange(0)), last_input_id_.load(),
                 last_input_length_.load(), yaw, pitch, roll);
      }
    }

    IOHIDDeviceUnscheduleFromRunLoop(hid_, hid_loop_, kCFRunLoopDefaultMode);
    const uint8_t off[3] = {0, 0, 0};
    feature(kReportEnableComponents, off, sizeof(off));
    hid_open_ = false;
    if (radio_thread_.joinable())
      radio_thread_.join();
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

  static CGDirectDisplayID findDisplay() {
    CGDirectDisplayID ids[16];
    uint32_t count = 0;
    CGGetOnlineDisplayList(16, ids, &count);
    for (uint32_t i = 0; i < count; ++i)
      if (CGDisplayVendorNumber(ids[i]) == kDisplayVendor && CGDisplayModelNumber(ids[i]) == kDisplayProductCV1)
        return ids[i];
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
  mvr_quat heading_ = quatIdentity();
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

} // namespace metalvr
