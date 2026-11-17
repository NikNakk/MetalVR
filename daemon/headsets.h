// What metalvrd knows of each headset before the headset itself says (docs/HEADSETS.md): the render size,
// refresh rates and field of view OpenXR is told while none is connected, and what the stream to a standalone
// headset must stay within. A driver replaces each value with the headset's own as soon as it has it.
#pragma once

#include <cstdint>
#include <string_view>

#include "metalvr_headsets.h"
#include "metalvr_protocol.h"

namespace metalvr {

struct HeadsetSpec {
  uint32_t headset = MVR_HEADSET_UNKNOWN;
  uint32_t eye_width = 0; // the per-eye render size applications are asked for
  uint32_t eye_height = 0;
  float refresh_rates[MVR_MAX_REFRESH_RATES] = {}; // ascending; the first `default_refresh` is the default
  uint32_t refresh_rate_count = 0;
  float default_refresh = 0;
  // a streamed headset's rate for PC VR, which a new ALVR session asks for (0: ALVR's default)
  float stream_refresh = 0;
  mvr_fov fov[2] = {};
  float ipd = 0.063f;
  // a standalone headset's video decoder: the most pixels per second it decodes (0: no limit known), and AV1
  uint64_t max_decode_pixels_per_second = 0;
  bool av1_decode = false;
};

// the spec of a headset, or one with no values for MVR_HEADSET_UNKNOWN
const HeadsetSpec &headsetSpec(uint32_t headset);

// the headset an ALVR client's display_name stands for (alvr_system_info's Platform names: "Quest 2", "Quest 3S",
// ...), or MVR_HEADSET_UNKNOWN; matched whole, so "Quest 3" is never a "Quest 3S"
uint32_t headsetFromAlvrName(std::string_view display_name);

// the headset a command-line key names ("quest-3", "index"; mvr_headset_description.key), or MVR_HEADSET_UNKNOWN
uint32_t headsetFromKey(std::string_view key);

// Fills a system info's identity and defaults from a spec: name, headset, controllers, render size, refresh rates
// and field of view. Flags, IPD and the play area are left as they are.
void applyHeadsetSpec(const HeadsetSpec &spec, mvr_system_info &info);

// A hand joint's usual radius in meters (OpenXR's joint order), for headsets that send joints without one.
float handJointRadius(uint32_t joint);

// A relaxed open hand whose wrist is at `wrist` (the joints in OpenXR's order, x toward the thumb side for the
// right hand), for the simulated headset.
void openHand(const mvr_pose &wrist, bool left, mvr_hand &out);

// Whether a stream of frame_width x frame_height at fps is within what the headset decodes; true when unknown.
bool withinDecoder(const HeadsetSpec &spec, uint32_t frame_width, uint32_t frame_height, float fps);

} // namespace metalvr
