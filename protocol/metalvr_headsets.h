/*
 * What each headset MetalVR can identify is called and what it comes with, shared by the server, the OpenXR
 * runtime and the SteamVR driver so that all three describe a headset the same way. Which headset is connected is
 * decided by the server's driver (mvr_system_info.headset); this table only describes it.
 */
#ifndef METALVR_HEADSETS_H
#define METALVR_HEADSETS_H

#include "metalvr_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mvr_headset_description {
  const char *name;         /* as a person would say it, e.g. "Meta Quest 3" */
  const char *manufacturer; /* as SteamVR shows it */
  const char *key;          /* for command lines, e.g. "quest-3" */
  uint32_t controllers;     /* mvr_controller_kind it ships with */
  uint32_t vendor_id;       /* the maker's USB vendor ID, which OpenXR reports as the system's vendor; 0 if none */
} mvr_headset_description;

static inline const mvr_headset_description *mvr_describe_headset(uint32_t headset) {
  static const mvr_headset_description table[MVR_HEADSET_COUNT] = {
      {"MetalVR Headset", "MetalVR", "unknown", MVR_CONTROLLERS_UNKNOWN, 0},
      {"MetalVR Simulated Headset", "MetalVR", "sim", MVR_CONTROLLERS_TOUCH, 0},
      {"Oculus Rift CV1", "Oculus", "rift-cv1", MVR_CONTROLLERS_TOUCH, 0x2833},
      {"Oculus Rift S", "Oculus", "rift-s", MVR_CONTROLLERS_TOUCH, 0x2833},
      {"Oculus Quest", "Oculus", "quest-1", MVR_CONTROLLERS_TOUCH, 0x2833},
      {"Meta Quest 2", "Oculus", "quest-2", MVR_CONTROLLERS_TOUCH, 0x2833},
      {"Meta Quest 3", "Meta", "quest-3", MVR_CONTROLLERS_TOUCH_PLUS, 0x2833},
      {"Meta Quest 3S", "Meta", "quest-3s", MVR_CONTROLLERS_TOUCH_PLUS, 0x2833},
      {"Meta Quest Pro", "Meta", "quest-pro", MVR_CONTROLLERS_TOUCH_PLUS, 0x2833},
      {"Valve Steam Frame", "Valve", "steam-frame", MVR_CONTROLLERS_STEAM_FRAME, 0x28de},
      {"Valve Index", "Valve", "index", MVR_CONTROLLERS_INDEX, 0x28de},
      {"HTC Vive", "HTC", "vive", MVR_CONTROLLERS_VIVE_WAND, 0x0bb4},
      {"HTC Vive Pro", "HTC", "vive-pro", MVR_CONTROLLERS_VIVE_WAND, 0x0bb4},
      {"HTC Vive Pro 2", "HTC", "vive-pro-2", MVR_CONTROLLERS_VIVE_WAND, 0x0bb4},
      {"PlayStation VR2", "Sony", "psvr2", MVR_CONTROLLERS_PS_SENSE, 0x054c},
  };
  return &table[headset < (uint32_t)MVR_HEADSET_COUNT ? headset : (uint32_t)MVR_HEADSET_UNKNOWN];
}

#ifdef __cplusplus
}
#endif

#endif /* METALVR_HEADSETS_H */
