// ALVR's foveated encoding (docs/HEADSETS.md, phase E): each eye of the composed frame is squeezed toward its edges
// before it is encoded, the center kept whole, and the headset's ALVR client stretches it back. The server's half
// is ALVR's server_openvr FFR.cpp and CompressAxisAlignedPixelShader.hlsl; the client's, which this must match, is
// graphics/src/stream.rs (foveated_encoding_shader_constants) and graphics/resources/stream.wgsl (MIT, alvr-org).
#pragma once

#include <cstdint>

namespace metalvr {

struct Foveation {
  bool enabled = false;
  uint32_t eye_width = 0, eye_height = 0;                     // each eye as composed (the stream's view size)
  uint32_t optimized_eye_width = 0, optimized_eye_height = 0; // each eye as encoded, multiples of 32
  float eye_size_ratio[2] = {1, 1}; // the squeezed eye's share of its aligned size
  float center_size[2] = {};        // aligned, as ALVR's client aligns them
  float center_shift[2] = {};
  float edge_ratio[2] = {1, 1};
};

// ALVR's CalculateFoveationVars, from the eye's size and ALVR's foveated_encoding settings
Foveation computeFoveation(uint32_t eye_width, uint32_t eye_height, float center_size_x, float center_size_y,
                           float center_shift_x, float center_shift_y, float edge_ratio_x, float edge_ratio_y);

// The server's warp: for a point of the encoded frame (uv 0..1 over the whole side-by-side frame), the point of
// the composed frame it shows (uv over the whole composed frame). The compositor's shader does the same.
void foveationSource(const Foveation &f, const float frame_uv[2], float out[2]);

// The client's unwarp, for checking: for a point of an eye as the headset shows it (uv 0..1 over that eye), the
// point of the encoded eye it is read from (uv over that eye of the encoded frame).
void foveationClientSample(const Foveation &f, int eye, const float eye_uv[2], float out[2]);

} // namespace metalvr
