// Pose math shared by the runtime and the server. Header-only, no dependencies beyond <cmath>.
#pragma once

#include <cmath>

#include "metalvr_protocol.h"

namespace metalvr {

inline mvr_vec3 vec3(float x, float y, float z) { return mvr_vec3{x, y, z}; }
inline mvr_vec3 operator+(mvr_vec3 a, mvr_vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline mvr_vec3 operator-(mvr_vec3 a, mvr_vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline mvr_vec3 operator*(mvr_vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }

inline mvr_quat quatIdentity() { return mvr_quat{0, 0, 0, 1}; }
inline mvr_pose poseIdentity() { return mvr_pose{quatIdentity(), {0, 0, 0}}; }

inline mvr_quat quatMul(mvr_quat a, mvr_quat b) {
  return {
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

inline mvr_quat quatConj(mvr_quat q) { return {-q.x, -q.y, -q.z, q.w}; }

inline mvr_quat quatNormalize(mvr_quat q) {
  float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-12f)
    return quatIdentity();
  return {q.x / n, q.y / n, q.z / n, q.w / n};
}

inline mvr_quat quatAxisAngle(mvr_vec3 axis, float radians) {
  float s = std::sin(radians * 0.5f);
  return {axis.x * s, axis.y * s, axis.z * s, std::cos(radians * 0.5f)};
}

inline mvr_vec3 quatRotate(mvr_quat q, mvr_vec3 v) {
  // v + 2w(u x v) + 2u x (u x v)
  mvr_vec3 u{q.x, q.y, q.z};
  mvr_vec3 c1{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
  mvr_vec3 c2{u.y * c1.z - u.z * c1.y, u.z * c1.x - u.x * c1.z, u.x * c1.y - u.y * c1.x};
  return v + c1 * (2.0f * q.w) + c2 * 2.0f;
}

// the pose of `b`, given in the frame of `a`, in a's parent frame
inline mvr_pose poseMul(const mvr_pose &a, const mvr_pose &b) {
  return {quatNormalize(quatMul(a.orientation, b.orientation)), a.position + quatRotate(a.orientation, b.position)};
}

inline mvr_pose poseInverse(const mvr_pose &p) {
  mvr_quat inv = quatConj(p.orientation);
  return {inv, quatRotate(inv, p.position) * -1.0f};
}

// the rotation about +Y that keeps the heading of `q` and drops its pitch and roll
inline mvr_quat quatYawOnly(mvr_quat q) {
  mvr_vec3 forward = quatRotate(q, {0, 0, -1});
  float yaw = std::atan2(-forward.x, -forward.z);
  return quatAxisAngle({0, 1, 0}, yaw);
}

inline bool quatIsValid(mvr_quat q) {
  float n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(n) && std::fabs(n - 1.0f) < 0.01f;
}

} // namespace metalvr
