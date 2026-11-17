#include "headsets.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace metalvr {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float rad(float degrees) { return degrees * kPi / 180.0f; }

// a left eye's field of view and its mirror for the right
constexpr HeadsetSpec withFov(HeadsetSpec spec, float left, float right, float up, float down) {
  spec.fov[0] = {rad(left), rad(right), rad(up), rad(down)};
  spec.fov[1] = {rad(-right), rad(-left), rad(up), rad(down)};
  return spec;
}

constexpr uint64_t k4k60 = 3840ull * 2160ull * 60ull;   // Snapdragon 835
constexpr uint64_t k4k120 = 3840ull * 2160ull * 120ull; // Snapdragon XR2 Gen 1 and Gen 2

// Sources and what is a guess: docs/HEADSETS.md. Streamed headsets: the size is what ALVR negotiates for the
// headset's default view by default (2144 wide, the height from the headset's aspect, a multiple of 32), so that
// what OpenXR is told before a headset connects is what it will be told after. Headsets on the Mac's own display:
// about 1.2 times the panel, as the CV1's 1344x1600 is for its 1080x1200.
HeadsetSpec makeSpec(uint32_t headset) {
  HeadsetSpec s;
  s.headset = headset;
  const auto rates = [&](std::initializer_list<float> list, float preferred) {
    s.refresh_rate_count = 0;
    for (float rate : list)
      if (s.refresh_rate_count < MVR_MAX_REFRESH_RATES) s.refresh_rates[s.refresh_rate_count++] = rate;
    s.default_refresh = preferred;
  };
  switch (headset) {
  case MVR_HEADSET_SIMULATED:
    s.eye_width = 1440, s.eye_height = 1584, rates({72}, 72);
    return withFov(s, -54.0f, 40.0f, 44.0f, -55.0f);
  case MVR_HEADSET_RIFT_CV1:
    s.eye_width = 1344, s.eye_height = 1600, rates({90}, 90), s.ipd = 0.0635f;
    return withFov(s, -46.6f, 40.0f, 43.4f, -46.6f); // guess: the CV1 driver computes its own from the lens
  case MVR_HEADSET_RIFT_S:
    // Oculus's own default render size for the Rift S
    s.eye_width = 1648, s.eye_height = 1776, rates({80}, 80), s.ipd = 0.0635f;
    return withFov(s, -48.0f, 42.0f, 45.0f, -48.0f); // guess; open drivers assume about 105 degrees across
  case MVR_HEADSET_QUEST_1:
    s.eye_width = 2144, s.eye_height = 2368, rates({72}, 72), s.ipd = 0.0635f;
    s.max_decode_pixels_per_second = k4k60, s.stream_refresh = 72;
    return withFov(s, -52.0f, 42.0f, 47.0f, -53.0f);
  case MVR_HEADSET_QUEST_2:
    s.eye_width = 2144, s.eye_height = 2336, rates({72, 80, 90, 120}, 72);
    s.max_decode_pixels_per_second = k4k120, s.stream_refresh = 90;
    return withFov(s, -52.0f, 45.0f, 48.0f, -51.0f);
  case MVR_HEADSET_QUEST_3:
    s.eye_width = 2144, s.eye_height = 2240, rates({72, 80, 90, 120}, 72);
    s.max_decode_pixels_per_second = k4k120, s.av1_decode = true, s.stream_refresh = 90;
    return withFov(s, -55.0f, 55.0f, 48.0f, -48.0f);
  case MVR_HEADSET_QUEST_3S:
    s.eye_width = 2144, s.eye_height = 2240, rates({72, 80, 90, 120}, 72);
    s.max_decode_pixels_per_second = k4k120, s.av1_decode = true, s.stream_refresh = 90;
    return withFov(s, -50.0f, 46.0f, 45.0f, -45.0f);
  case MVR_HEADSET_QUEST_PRO:
    s.eye_width = 2144, s.eye_height = 2240, rates({72, 90}, 72);
    s.max_decode_pixels_per_second = k4k120, s.av1_decode = false, s.stream_refresh = 90;
    return withFov(s, -53.0f, 53.0f, 48.0f, -48.0f);
  case MVR_HEADSET_STEAM_FRAME:
    // its panels are 2160x2160; the clients that run on it decode HEVC 8-bit only
    s.eye_width = 2160, s.eye_height = 2160, rates({72, 80, 90, 96, 108, 120}, 90), s.stream_refresh = 90;
    return withFov(s, -52.0f, 52.0f, 52.0f, -52.0f); // guess: Valve says "up to 110 degrees"
  case MVR_HEADSET_VALVE_INDEX:
    s.eye_width = 1728, s.eye_height = 1920, rates({80, 90, 120, 144}, 90);
    return withFov(s, -54.0f, 50.0f, 55.0f, -55.0f); // guess: the driver reads its own from the headset
  case MVR_HEADSET_VIVE:
    s.eye_width = 1344, s.eye_height = 1600, rates({90}, 90);
    return withFov(s, -52.0f, 50.0f, 55.0f, -55.0f); // guess: the driver reads its own from the headset
  case MVR_HEADSET_VIVE_PRO:
    s.eye_width = 1728, s.eye_height = 1920, rates({90}, 90);
    return withFov(s, -52.0f, 50.0f, 55.0f, -55.0f);
  case MVR_HEADSET_VIVE_PRO_2:
    // in its 3264x1632 mode, the largest one an Apple silicon Mac is expected to drive (docs/HEADSETS.md)
    s.eye_width = 1952, s.eye_height = 1952, rates({90, 120}, 90);
    return withFov(s, -58.0f, 55.0f, 55.0f, -55.0f);
  case MVR_HEADSET_PSVR2:
    // 2000x2040 OLED panels at 90 or 120 Hz through Sony's PC adapter, rendered at the panel's own size (1.2 times
    // it would be nearly 6 million pixels an eye); Monado's field of view (psvr2.c), wider toward the temple
    s.eye_width = 2000, s.eye_height = 2040, rates({90, 120}, 90);
    return withFov(s, -61.5f, 43.5f, 53.0f, -53.0f);
  default:
    s.headset = MVR_HEADSET_UNKNOWN;
    return s;
  }
}

} // namespace

const HeadsetSpec &headsetSpec(uint32_t headset) {
  static const auto table = [] {
    struct Table {
      HeadsetSpec specs[MVR_HEADSET_COUNT];
    } t;
    for (uint32_t i = 0; i < MVR_HEADSET_COUNT; i++) t.specs[i] = makeSpec(i);
    return t;
  }();
  return table.specs[headset < MVR_HEADSET_COUNT ? headset : MVR_HEADSET_UNKNOWN];
}

uint32_t headsetFromAlvrName(std::string_view name) {
  if (name == "Quest 1") return MVR_HEADSET_QUEST_1;
  if (name == "Quest 2") return MVR_HEADSET_QUEST_2;
  if (name == "Quest 3") return MVR_HEADSET_QUEST_3;
  if (name == "Quest 3S") return MVR_HEADSET_QUEST_3S;
  if (name == "Quest Pro") return MVR_HEADSET_QUEST_PRO;
  // the name MetalVR's patched ALVR client gives itself on a Steam Frame (patches/alvr/0003); the stock client says
  // "Android (unknown)" there
  if (name == "Steam Frame") return MVR_HEADSET_STEAM_FRAME;
  return MVR_HEADSET_UNKNOWN;
}

uint32_t headsetFromKey(std::string_view key) {
  for (uint32_t i = 1; i < MVR_HEADSET_COUNT; i++)
    if (key == mvr_describe_headset(i)->key) return i;
  return MVR_HEADSET_UNKNOWN;
}

void applyHeadsetSpec(const HeadsetSpec &spec, mvr_system_info &info) {
  const mvr_headset_description *description = mvr_describe_headset(spec.headset);
  info.headset = spec.headset;
  info.controllers = description->controllers;
  std::snprintf(info.system_name, sizeof(info.system_name), "%s", description->name);
  if (spec.headset == MVR_HEADSET_UNKNOWN) return;
  info.eye_width = spec.eye_width;
  info.eye_height = spec.eye_height;
  info.refresh_hz = spec.default_refresh;
  info.refresh_rate_count = spec.refresh_rate_count;
  std::memcpy(info.refresh_rates, spec.refresh_rates, sizeof(info.refresh_rates));
  info.fov[0] = spec.fov[0];
  info.fov[1] = spec.fov[1];
}

float handJointRadius(uint32_t joint) {
  // palm, wrist; thumb metacarpal..tip; then index, middle, ring, little: metacarpal, proximal, intermediate,
  // distal, tip
  static const float radii[MVR_HAND_JOINTS] = {0.018f, 0.020f, 0.017f, 0.014f, 0.011f, 0.009f, 0.012f, 0.010f,
                                               0.009f, 0.008f, 0.007f, 0.012f, 0.010f, 0.009f, 0.008f, 0.007f,
                                               0.011f, 0.009f, 0.008f, 0.007f, 0.006f, 0.010f, 0.008f, 0.007f,
                                               0.006f, 0.005f};
  return joint < MVR_HAND_JOINTS ? radii[joint] : 0.008f;
}

void openHand(const mvr_pose &wrist, bool left, mvr_hand &out) {
  // where each joint is from the wrist, in the hand's frame (+x right, +y up, -z toward the fingertips), meters
  const float side = left ? -1.0f : 1.0f;
  float local[MVR_HAND_JOINTS][3];
  local[0][0] = 0, local[0][1] = 0, local[0][2] = -0.045f; // palm
  local[1][0] = 0, local[1][1] = 0, local[1][2] = 0;       // wrist
  for (int j = 0; j < 4; ++j) { // thumb, toward the inside
    local[2 + j][0] = -side * (0.020f + 0.018f * j);
    local[2 + j][1] = 0.005f;
    local[2 + j][2] = -0.020f - 0.020f * j;
  }
  for (int finger = 0; finger < 4; ++finger) {
    const float x = side * (0.025f - 0.016f * finger);
    const float length = finger == 1 ? 1.08f : finger == 3 ? 0.8f : 1.0f;
    for (int j = 0; j < 5; ++j) {
      float *p = local[6 + finger * 5 + j];
      p[0] = x;
      p[1] = 0;
      p[2] = -0.02f - (j == 0 ? 0.0f : 0.06f + 0.025f * (j - 1)) * length;
    }
  }
  for (uint32_t j = 0; j < MVR_HAND_JOINTS; ++j) {
    // the joint's position turned with the wrist; every joint faces as the wrist does
    const mvr_quat &q = wrist.orientation;
    const float vx = local[j][0], vy = local[j][1], vz = local[j][2];
    const float tx = 2 * (q.y * vz - q.z * vy), ty = 2 * (q.z * vx - q.x * vz), tz = 2 * (q.x * vy - q.y * vx);
    out.joints[j].orientation = q;
    out.joints[j].position = {wrist.position.x + vx + q.w * tx + (q.y * tz - q.z * ty),
                              wrist.position.y + vy + q.w * ty + (q.z * tx - q.x * tz),
                              wrist.position.z + vz + q.w * tz + (q.x * ty - q.y * tx)};
    out.radii[j] = handJointRadius(j);
  }
  out.flags = MVR_HAND_TRACKED;
}

bool withinDecoder(const HeadsetSpec &spec, uint32_t frame_width, uint32_t frame_height, float fps) {
  if (spec.max_decode_pixels_per_second == 0 || !(fps > 0.0f)) return true;
  const double rate = static_cast<double>(frame_width) * frame_height * fps;
  return rate <= static_cast<double>(spec.max_decode_pixels_per_second) * 1.02; // rounding of sizes and rates
}

} // namespace metalvr
