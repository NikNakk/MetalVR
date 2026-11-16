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
  float2 lens_center; // per eye, in the eye's NDC, for the left eye; the right eye is its mirror
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
  // the halves of the panel are mirror images: the right eye is worked out as the left one's mirror
  if (d.eye == 1) ndc.x = -ndc.x;
  float2 tan_angle = (ndc - d.lens_center) * d.tan_scale;
  float rho = dot(tan_angle, tan_angle);
  float r = magnification(d, rho);
  float3 scale = float3(r * (1.0 + d.chromatic.x + rho * d.chromatic.y), r, r * (1.0 + d.chromatic.z + rho * d.chromatic.w));
  for (int c = 0; c < 3; ++c) {
    float2 t = tan_angle * scale[c];
    if (d.eye == 1) t.x = -t.x;
    out[c] = t;
  }
}

fragment float4 distort(VOut in [[stage_in]], constant Distortion &d [[buffer(0)]],
                        texture2d<float> frame [[texture(0)]], sampler s [[sampler(0)]]) {
  float2 t[3];
  channel_tangents(d, in.position.xy, t);
  float3 color;
  for (int c = 0; c < 3; ++c) {
    // where that direction is in the eye's image: x from the left edge of its field of view, y from the top
    float2 src = float2((t[c].x + d.fov_tan.x) / (d.fov_tan.x + d.fov_tan.y), (t[c].y + d.fov_tan.z) / (d.fov_tan.z + d.fov_tan.w));
    if (any(src < 0.0) || any(src > 1.0)) {
      color[c] = 0.0;
      continue;
    }
    color[c] = frame.sample(s, float2((d.eye + src.x) * 0.5, src.y))[c];
  }
  return float4(color, 1.0);
}

// The test scene shown while no application draws: a sphere around the viewer, fixed in the world, with a line every
// 15 degrees of latitude and longitude, the horizon, and a red mark straight ahead, so that turning the head and the
// lens correction can be judged without an application.
struct SceneRotation {
  float4 columns[3]; // head to world
};

static float3 scene_color(float3 w) {
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

fragment float4 scene(VOut in [[stage_in]], constant Distortion &d [[buffer(0)]],
                      constant SceneRotation &rotation [[buffer(1)]]) {
  float2 t[3];
  channel_tangents(d, in.position.xy, t);
  float3x3 head_to_world = float3x3(rotation.columns[0].xyz, rotation.columns[1].xyz, rotation.columns[2].xyz);
  float3 color;
  for (int c = 0; c < 3; ++c) {
    // the eye looks down -z, with +y up: tangent y runs down
    float3 view = normalize(float3(t[c].x, -t[c].y, -1.0));
    color[c] = scene_color(head_to_world * view)[c];
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
    for (auto &c : out.controllers)
      std::memset(&c, 0, sizeof(c));
  }

  void haptic(uint32_t, float, float, float) override {}

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
    CAMetalLayer *layer = nil;
    {
      std::lock_guard<std::mutex> lock(layer_mutex_);
      layer = layer_;
    }
    if (!layer || !composed)
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
    last_app_frame_ns_ = nowNs();
    present(layer, texture, nullptr, composed);
    ++presented_;
    const int64_t now = nowNs();
    if (g_verbose && now - last_report_ns_ > 5'000'000'000) {
      if (last_report_ns_)
        logDebug("rift: %.1f frames/s on the headset, %.1f ms from the frame's end to its present",
                 (presented_ - last_presented_) / ((now - last_report_ns_) / 1e9), (now - frame.submitted_ns) / 1e6);
      last_report_ns_ = now;
      last_presented_ = presented_;
    }
  }

private:
  // Draws one frame on the headset: the composed frame through the lens correction, or with no frame the test scene
  // seen with the head's rotation. `keep` is held until the GPU is done with it.
  void present(CAMetalLayer *layer, id<MTLTexture> texture, const mvr_quat *rotation, CVPixelBufferRef keep) {
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
      // the head-to-world rotation's columns
      const mvr_quat q = rotation ? *rotation : quatIdentity();
      const mvr_vec3 x = quatRotate(q, {1, 0, 0}), y = quatRotate(q, {0, 1, 0}), z = quatRotate(q, {0, 0, 1});
      const float columns[12] = {x.x, x.y, x.z, 0, y.x, y.y, y.z, 0, z.x, z.y, z.z, 0};
      [enc setFragmentBytes:columns length:sizeof(columns) atIndex:1];
    }
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
    [cmd presentDrawable:drawable];
    [cmd commit];
  }

  // While no application draws, the headset shows the test scene at its refresh rate: nextDrawable waits for a
  // drawable the display has let go of, which paces the loop.
  void sceneThread() {
    while (running_) {
      CAMetalLayer *layer = nil;
      {
        std::lock_guard<std::mutex> lock(layer_mutex_);
        layer = layer_;
      }
      if (!layer || !fusion_ready_ || nowNs() - last_app_frame_ns_ < 250'000'000) {
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
        present(layer, nil, &q, nullptr);
      }
    }
  }

  float ipd() const {
    if (options_.ipd > 0.0f)
      return options_.ipd;
    const float measured = ipd_m_.load();
    return measured > 0.05f && measured < 0.08f ? measured : panel_.lens_separation_m;
  }

  DistortionUniforms uniforms(uint32_t eye, float viewport_w, float viewport_h) const {
    DistortionUniforms u{};
    // Monado's compute_render_info, for the left eye
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

  std::atomic<bool> running_{false};
  std::thread hid_thread_;
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

  uint64_t presented_ = 0, last_presented_ = 0;
  int64_t last_report_ns_ = 0;
};

} // namespace

std::unique_ptr<Driver> makeRiftDriver(const RiftOptions &options) { return std::make_unique<RiftDriver>(options); }

} // namespace metalvr
