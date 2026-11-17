#include "foveation.h"

#include <cmath>

namespace metalvr {

Foveation computeFoveation(uint32_t eye_width, uint32_t eye_height, float center_size_x, float center_size_y,
                           float center_shift_x, float center_shift_y, float edge_ratio_x, float edge_ratio_y) {
  Foveation f;
  if (!eye_width || !eye_height || !(edge_ratio_x >= 1.0f) || !(edge_ratio_y >= 1.0f)) return f;
  const double size[2] = {static_cast<double>(eye_width), static_cast<double>(eye_height)};
  const double center[2] = {center_size_x, center_size_y};
  const double shift[2] = {center_shift_x, center_shift_y};
  const double edge[2] = {edge_ratio_x, edge_ratio_y};
  uint32_t optimized[2];
  for (int a = 0; a < 2; a++) {
    const double edge_size = size[a] - center[a] * size[a];
    const double center_aligned = 1.0 - std::ceil(edge_size / (edge[a] * 2.0)) * (edge[a] * 2.0) / size[a];
    const double edge_aligned = size[a] - center_aligned * size[a];
    const double shift_aligned =
        edge_aligned > 0 ? std::ceil(shift[a] * edge_aligned / (edge[a] * 2.0)) * (edge[a] * 2.0) / edge_aligned : 0;
    const double scale = center_aligned + (1.0 - center_aligned) / edge[a];
    const double squeezed = scale * size[a];
    optimized[a] = static_cast<uint32_t>(std::ceil(squeezed / 32.0) * 32.0);
    f.eye_size_ratio[a] = static_cast<float>(squeezed / optimized[a]);
    f.center_size[a] = static_cast<float>(center_aligned);
    f.center_shift[a] = static_cast<float>(shift_aligned);
    f.edge_ratio[a] = static_cast<float>(edge[a]);
  }
  f.enabled = true;
  f.eye_width = eye_width;
  f.eye_height = eye_height;
  f.optimized_eye_width = optimized[0];
  f.optimized_eye_height = optimized[1];
  return f;
}

void foveationSource(const Foveation &f, const float frame_uv[2], float out[2]) {
  // CompressAxisAlignedPixelShader.hlsl: the right eye is mirrored, so both are squeezed alike from the nose out
  const bool right = frame_uv[0] > 0.5f;
  float eye[2] = {(frame_uv[0] + (right ? 1.0f - 2.0f * frame_uv[0] : 0.0f)) * 2.0f, frame_uv[1]};
  float source[2];
  for (int a = 0; a < 2; a++) {
    const float uv = eye[a] / f.eye_size_ratio[a];
    const float size = f.center_size[a], shift = f.center_shift[a], edge = f.edge_ratio[a];
    const float c0 = (1.0f - size) / 2.0f;
    const float c1 = (edge - 1.0f) * c0 * (shift + 1.0f) / edge;
    const float c2 = (edge - 1.0f) * size + 1.0f;
    const float lo = c0 * (shift + 1.0f) / c2;
    const float hi = c0 * (shift - 1.0f) / c2 + 1.0f;
    const float center = uv * c2 / edge + c1;
    const float d2 = uv * c2;
    const float d3 = (uv - 1.0f) * c2 + 1.0f;
    const float g1 = uv / lo;
    const float g2 = (1.0f - uv) / (1.0f - hi);
    if (uv < lo)
      source[a] = g1 * center + (1.0f - g1) * d2;
    else if (uv > hi)
      source[a] = g2 * center + (1.0f - g2) * d3;
    else
      source[a] = center;
  }
  out[0] = source[0] * 0.5f + (right ? 1.0f - source[0] : 0.0f);
  out[1] = source[1];
}

void foveationClientSample(const Foveation &f, int eye, const float eye_uv[2], float out[2]) {
  // stream.rs's constants and stream.wgsl's fragment_main
  float uv[2] = {eye == 1 ? 1.0f - eye_uv[0] : eye_uv[0], eye_uv[1]};
  for (int a = 0; a < 2; a++) {
    const double size = f.center_size[a], shift = f.center_shift[a], edge = f.edge_ratio[a];
    const double c0 = (1.0 - size) * 0.5;
    const double c1 = (edge - 1.0) * c0 * (shift + 1.0) / edge;
    const double c2 = (edge - 1.0) * size + 1.0;
    const double lo_bound = c0 * (shift + 1.0);
    const double hi_bound = c0 * (shift - 1.0) + 1.0;
    const double lo_c = c0 * (shift + 1.0) / c2;
    const double hi_c = c0 * (shift - 1.0) / c2 + 1.0;
    const double a_left = c2 * (1.0 - edge) / (edge * lo_c);
    const double b_left = (c1 + c2 * lo_c) / lo_c;
    const double a_right = c2 * (edge - 1.0) / (edge * (1.0 - hi_c));
    const double b_right =
        (c2 - edge * c1 - 2.0 * edge * c2 + c2 * edge * (1.0 - hi_c) + edge) / (edge * (1.0 - hi_c));
    const double c_right = (c2 * edge - c2) * (c1 - hi_c + c2 * hi_c) / (edge * (1.0 - hi_c) * (1.0 - hi_c));
    const double v = uv[a];
    double r;
    if (v < lo_bound)
      r = (-b_left + std::sqrt(b_left * b_left + 4.0 * a_left * v)) / (2.0 * a_left);
    else if (v > hi_bound)
      r = (-b_right + std::sqrt(b_right * b_right - 4.0 * (c_right - a_right * v))) / (2.0 * a_right);
    else
      r = (v - c1) * edge / c2;
    uv[a] = static_cast<float>(r * f.eye_size_ratio[a]);
  }
  out[0] = eye == 1 ? 1.0f - uv[0] : uv[0];
  out[1] = uv[1];
}

} // namespace metalvr
