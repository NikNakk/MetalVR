#include "bridge.h"

#include "os/os_threading.h"
#include "psvr2/psvr2.h"
#include "xrt/xrt_instance.h"
#include "xrt/xrt_space.h"
#include "xrt/xrt_system.h"

#include <stdlib.h>
#include <string.h>

struct mvr_pssense_system {
  struct xrt_instance *instance;
  struct xrt_system *system;
  struct xrt_system_devices *devices;
  struct xrt_space_overseer *spaces;
  struct xrt_device *head;
  struct xrt_device *hand[2];
};

static void close_system(struct mvr_pssense_system *system) {
  xrt_space_overseer_destroy(&system->spaces);
  xrt_system_devices_destroy(&system->devices);
  xrt_system_destroy(&system->system);
  xrt_instance_destroy(&system->instance);
  free(system);
}

static struct mvr_pssense_system *open_system(float calibration[8]) {
  struct mvr_pssense_system *system = calloc(1, sizeof(*system));
  if (!system)
    return NULL;
  if (xrt_instance_create(NULL, &system->instance) != XRT_SUCCESS ||
      xrt_instance_create_system(system->instance, &system->system, &system->devices, &system->spaces, NULL) !=
          XRT_SUCCESS ||
      !system->devices->static_roles.head || system->devices->static_roles.head->name != XRT_DEVICE_PSVR2) {
    close_system(system);
    return NULL;
  }

  system->head = system->devices->static_roles.head;
  for (size_t i = 0; i < system->devices->static_xdev_count; ++i) {
    struct xrt_device *device = system->devices->static_xdevs[i];
    if (device->name == XRT_DEVICE_PSSENSE)
      system->hand[device->device_type == XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER] = device;
  }
  memcpy(calibration, psvr2_hmd(system->head)->distortion_calibration, sizeof(float) * 8);
  return system;
}

static void status(struct mvr_pssense_system *system, struct mvr_psvr2_status *out) {
  struct psvr2_hmd *head = psvr2_hmd(system->head);
  os_mutex_lock(&head->data_lock);
  out->worn = xrt_atomic_s32_load(&head->proximity_sensor) != 0;
  out->display_signal = head->dprx_status != 0;
  out->ipd_m = head->ipd_mm / 1000.0f;
  os_mutex_unlock(&head->data_lock);
}

static mvr_vec3 vec3(struct xrt_vec3 v) {
  return (mvr_vec3){v.x, v.y, v.z};
}

static mvr_pose pose(struct xrt_pose p) {
  return (mvr_pose){{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}, vec3(p.position)};
}

static bool head_pose(struct mvr_pssense_system *system, int64_t time_ns, struct mvr_psvr2_head *out) {
  struct xrt_space_relation relation = {0};
  if (xrt_device_get_tracked_pose(system->head, XRT_INPUT_GENERIC_HEAD_POSE, time_ns, &relation) != XRT_SUCCESS ||
      !(relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT))
    return false;
  out->pose = pose(relation.pose);
  out->position_valid = (relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0;
  if (relation.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT)
    out->linear_velocity = vec3(relation.linear_velocity);
  if (relation.relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT)
    out->angular_velocity = vec3(relation.angular_velocity);
  return true;
}

static void controller(struct mvr_pssense_system *system, int hand, int64_t time_ns, mvr_controller_state *out) {
  struct xrt_device *device = system->hand[hand];
  if (!device || xrt_device_update_inputs(device) != XRT_SUCCESS)
    return;

  for (size_t i = 0; i < device->input_count; ++i) {
    const struct xrt_input *input = &device->inputs[i];
    if (!input->active)
      continue;
    out->flags |= MVR_CONTROLLER_CONNECTED;
    switch (input->name) {
    case XRT_INPUT_PSSENSE_SQUARE_CLICK:
    case XRT_INPUT_PSSENSE_CROSS_CLICK:
      if (input->value.boolean) out->buttons |= MVR_BUTTON_PRIMARY;
      break;
    case XRT_INPUT_PSSENSE_TRIANGLE_CLICK:
    case XRT_INPUT_PSSENSE_CIRCLE_CLICK:
      if (input->value.boolean) out->buttons |= MVR_BUTTON_SECONDARY;
      break;
    case XRT_INPUT_PSSENSE_SQUARE_TOUCH:
    case XRT_INPUT_PSSENSE_CROSS_TOUCH:
      if (input->value.boolean) out->touches |= MVR_BUTTON_PRIMARY;
      break;
    case XRT_INPUT_PSSENSE_TRIANGLE_TOUCH:
    case XRT_INPUT_PSSENSE_CIRCLE_TOUCH:
      if (input->value.boolean) out->touches |= MVR_BUTTON_SECONDARY;
      break;
    case XRT_INPUT_PSSENSE_SHARE_CLICK:
    case XRT_INPUT_PSSENSE_OPTIONS_CLICK:
      if (input->value.boolean) out->buttons |= MVR_BUTTON_MENU;
      break;
    case XRT_INPUT_PSSENSE_PS_CLICK:
      if (input->value.boolean) out->buttons |= MVR_BUTTON_SYSTEM;
      break;
    case XRT_INPUT_PSSENSE_TRIGGER_CLICK:
      if (input->value.boolean) out->buttons |= MVR_BUTTON_TRIGGER;
      break;
    case XRT_INPUT_PSSENSE_TRIGGER_TOUCH:
      if (input->value.boolean) out->touches |= MVR_BUTTON_TRIGGER;
      break;
    case XRT_INPUT_PSSENSE_TRIGGER_VALUE:
      out->trigger = input->value.vec1.x;
      break;
    case XRT_INPUT_PSSENSE_SQUEEZE_CLICK:
      if (input->value.boolean) {
        out->buttons |= MVR_BUTTON_SQUEEZE;
        out->squeeze = 1;
      }
      break;
    case XRT_INPUT_PSSENSE_SQUEEZE_TOUCH:
      if (input->value.boolean) out->touches |= MVR_BUTTON_SQUEEZE;
      break;
    case XRT_INPUT_PSSENSE_THUMBSTICK:
      out->thumbstick_x = input->value.vec2.x;
      out->thumbstick_y = input->value.vec2.y;
      break;
    case XRT_INPUT_PSSENSE_THUMBSTICK_CLICK:
      if (input->value.boolean) out->buttons |= MVR_BUTTON_THUMBSTICK;
      break;
    case XRT_INPUT_PSSENSE_THUMBSTICK_TOUCH:
      if (input->value.boolean) out->touches |= MVR_BUTTON_THUMBSTICK;
      break;
    default:
      break;
    }
  }

  if (!(out->flags & MVR_CONTROLLER_CONNECTED))
    return;

  bool present = false, charging = false;
  if (device->supported.battery_status &&
      xrt_device_get_battery_status(device, &present, &charging, &out->battery) == XRT_SUCCESS && present)
    out->flags |= MVR_CONTROLLER_BATTERY_VALID | (charging ? MVR_CONTROLLER_CHARGING : 0);

  struct xrt_space_relation grip = {0}, aim = {0};
  if (xrt_device_get_tracked_pose(device, XRT_INPUT_PSSENSE_GRIP_POSE, time_ns, &grip) != XRT_SUCCESS ||
      xrt_device_get_tracked_pose(device, XRT_INPUT_PSSENSE_AIM_POSE, time_ns, &aim) != XRT_SUCCESS)
    return;
  const uint32_t flags = grip.relation_flags & aim.relation_flags;
  out->flags |= MVR_CONTROLLER_SPLIT_POSE;
  if (flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) {
    out->flags |= MVR_CONTROLLER_ORIENTATION_VALID;
    out->grip = pose(grip.pose);
    out->aim = pose(aim.pose);
    if (flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT)
      out->flags |= MVR_CONTROLLER_ORIENTATION_TRACKED;
  }
  if ((flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) && (flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT))
    out->flags |= MVR_CONTROLLER_POSE_VALID;
  if (flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT)
    out->flags |= MVR_CONTROLLER_POSITION_TRACKED;
  if (flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) {
    out->linear_velocity = vec3(grip.linear_velocity);
    out->flags |= MVR_CONTROLLER_LINEAR_VELOCITY_VALID;
  }
  if (flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) {
    out->angular_velocity = vec3(grip.angular_velocity);
    out->flags |= MVR_CONTROLLER_ANGULAR_VELOCITY_VALID;
  }
}

static void haptic(struct mvr_pssense_system *system, uint32_t hand, float duration_s, float frequency_hz,
                   float amplitude) {
  if (hand >= 2 || !system->hand[hand])
    return;
  struct xrt_output_value value = {0};
  value.vibration.frequency = frequency_hz;
  value.vibration.amplitude = amplitude;
  value.vibration.duration_ns = (int64_t)(duration_s * 1e9);
  xrt_device_set_output(system->hand[hand], XRT_OUTPUT_NAME_PSSENSE_VIBRATION, &value);
}

const struct mvr_pssense_bridge *mvr_pssense_get_bridge(int version) {
  static const struct mvr_pssense_bridge bridge = {open_system, close_system, status, head_pose, controller, haptic};
  return version == MVR_PSSENSE_BRIDGE_VERSION ? &bridge : NULL;
}
