// A headset driver: where tracking comes from and where encoded video goes.
//
// metalvrd owns the OpenXR-facing side (sessions, swapchains, composition, encoding). A Driver is the other end:
// the simulated headset used for tests, or a real headset reached through a streaming transport.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <CoreVideo/CVPixelBuffer.h>

#include "metalvr_protocol.h"

namespace metalvr {

enum class Codec : uint32_t { H264 = 0, HEVC = 1 };

// what the driver wants the encoder to produce; read before each encoder (re)configuration
struct VideoParams {
  Codec codec = Codec::H264;
  uint32_t frame_width = 0; // the whole frame: both eyes side by side, left eye in the left half
  uint32_t frame_height = 0;
  float fps = 72.0f;
  uint64_t bitrate_bps = 30'000'000;
};

// an encoded frame's identity and history; times are on the server clock
struct FrameInfo {
  uint64_t sample_id = 0;       // the tracking sample the frame was rendered with
  int64_t display_time_ns = 0;  // when the application expected it on the display
  Codec codec = Codec::H264;    // what the bytes are, fixed when the frame went into the encoder
  bool keyframe = false;
  int64_t submitted_ns = 0;     // the application ended the frame (FRAME_END reached the server)
  int64_t composed_ns = 0;      // the layers were composed and the frame went into the encoder
  int64_t encoded_ns = 0;       // the encoder put it out
  // the head's orientation the frame was rendered for, in tracking space; a driver that shows frames itself
  // turns them by how far the head has turned since (has_orientation is false for a frame of quads alone)
  mvr_quat head_orientation{0, 0, 0, 1};
  bool has_orientation = false;
};

class Driver {
public:
  virtual ~Driver() = default;

  virtual const char *name() const = 0;

  // starts the transport; false if it cannot run at all
  virtual bool start() = 0;
  virtual void stop() = 0;

  // true while a headset is connected and wants video
  virtual bool connected() = 0;

  // the headset as OpenXR should describe it. Valid before a headset connects (defaults), and may change when one
  // does; `generation` changes whenever the answer does.
  virtual mvr_system_info systemInfo() = 0;
  virtual uint64_t generation() = 0;

  // poses and inputs predicted for `target_time_ns` (server clock). Fills everything in `out` except `result`.
  // out.sample_id is the driver's own name for the sample and comes back in sendFrame.
  virtual void tracking(int64_t target_time_ns, mvr_tracking_response &out) = 0;

  virtual void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) = 0;
  // the tracking system's sensors, at most MVR_MAX_TRACKING_REFERENCES; how many were written
  virtual uint32_t trackingReferences(mvr_tracking_reference *out) { return 0; }

  virtual VideoParams videoParams() = 0;

  // true once when the headset needs a keyframe (a new connection, packet loss)
  virtual bool consumeKeyframeRequest() = 0;

  // the codec's parameter sets in Annex B (H.264: SPS, PPS; HEVC: VPS, SPS, PPS), sent before the first frame and
  // again whenever they change
  virtual void sendVideoConfig(Codec codec, const uint8_t *annexb, size_t size) = 0;

  // one encoded frame in Annex B, with where its time went
  virtual void sendFrame(const FrameInfo &frame, const uint8_t *annexb, size_t size) = 0;

  // A driver that shows frames on a display of the Mac's own (a headset on its video output) instead of streaming
  // them: the server then hands it each composed frame, an IOSurface-backed BGRA buffer the size of videoParams'
  // frame, in sRGB, and encodes nothing. The driver retains the buffer for as long as it reads it.
  virtual bool displaysFrames() const { return false; }
  virtual void displayFrame(const FrameInfo &frame, CVPixelBufferRef composed) {}

  // true for a driver that needs AppKit's main run loop (a window on the headset's display): metalvrd then runs the
  // server on a thread of its own and the main thread runs AppKit
  virtual bool needsMainRunLoop() const { return false; }
};

// the simulated headset: a slowly moving head, two controllers, and an optional Annex B dump of the stream
struct SimOptions {
  uint32_t eye_width = 1440;
  uint32_t eye_height = 1584;
  float refresh_hz = 72.0f;
  Codec codec = Codec::H264;
  uint64_t bitrate_bps = 30'000'000;
  std::string dump_path; // empty: encoded video is counted and dropped
  bool animate = true;
};
std::unique_ptr<Driver> makeSimDriver(const SimOptions &options);

// a headset running the stock ALVR client, through ALVR's server_core. Null if MetalVR was built without it.
struct AlvrOptions {
  std::string config_dir; // session.json and logs live here
  // The hostnames of the headsets allowed to connect, as their ALVR client shows them ("1234.client.local.").
  // ALVR streams only to clients marked trusted, and nothing is trusted that is not named here or already
  // trusted in the config directory: a headset that announces itself is logged with its name and left out.
  std::vector<std::string> trusted_clients;
};
std::unique_ptr<Driver> makeAlvrDriver(const AlvrOptions &options);

// An Oculus Rift CV1 on the Mac's own video output and USB (docs/RIFT.md): its HID interface turns the headset on
// and gives its rotation, and frames are drawn on its display with its lens distortion. Head rotation only.
struct RiftOptions {
  uint32_t eye_width = 1344; // the per-eye render size applications are asked for
  uint32_t eye_height = 1600;
  float ipd = 0.0f;          // meters; 0: the headset's own lens separation
  float head_height = 1.6f;  // meters above the floor, for a seated head that has no position tracking
  bool test_scene = false;   // between applications, the lens-check grid instead of the home
  std::string home_logo;     // the PNG shown in the home; empty: <metalvrd>/../share/metalvr/home-logo.png
};
std::unique_ptr<Driver> makeRiftDriver(const RiftOptions &options);

} // namespace metalvr
