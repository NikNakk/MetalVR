// MetalVR's interface to the Monado device drivers. Monado types stay in bridge.c.
#pragma once

#include "metalvr_protocol.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct mvr_pssense_system;

struct mvr_psvr2_status {
  bool worn;
  bool display_signal;
  float ipd_m;
};

struct mvr_psvr2_head {
  mvr_pose pose;
  mvr_vec3 linear_velocity;
  mvr_vec3 angular_velocity;
  bool position_valid;
};

struct mvr_pssense_bridge {
  struct mvr_pssense_system *(*open)(float calibration[8]);
  void (*close)(struct mvr_pssense_system *system);
  void (*status)(struct mvr_pssense_system *system, struct mvr_psvr2_status *out);
  bool (*head)(struct mvr_pssense_system *system, int64_t time_ns, struct mvr_psvr2_head *out);
  void (*controller)(struct mvr_pssense_system *system, int hand, int64_t time_ns, mvr_controller_state *out);
  void (*haptic)(struct mvr_pssense_system *system, uint32_t hand, float duration_s, float frequency_hz,
                 float amplitude);
};

enum { MVR_PSSENSE_BRIDGE_VERSION = 1 };
typedef const struct mvr_pssense_bridge *(*mvr_pssense_get_bridge_fn)(int version);
const struct mvr_pssense_bridge *mvr_pssense_get_bridge(int version);

#ifdef __cplusplus
}
#endif
