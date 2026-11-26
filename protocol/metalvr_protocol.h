/*
 * MetalVR wire protocol: OpenXR runtime (client) <-> metalvrd (server).
 *
 * One TCP connection on the loopback interface per runtime instance. Every request gets exactly one response whose
 * type is the request's type with MVR_RESPONSE_BIT set. No request blocks in the server for longer than it takes to
 * answer: frame pacing is a wake time the client sleeps to, so a slow frame never stalls another thread's request.
 *
 * All integers are little-endian, all structs are packed to 4 bytes and contain only fixed-width fields, so the
 * same header compiles for a Windows (PE) runtime under Wine and a native macOS runtime and server.
 *
 * Time is the server's monotonic clock in nanoseconds. A client learns the offset to its own clock with MVR_CLOCK.
 * Poses are in the server's tracking space: right-handed, +Y up, -Z forward, meters, origin on the floor (OpenXR
 * STAGE). Field-of-view angles are OpenXR's: radians, left and down negative.
 */
#ifndef METALVR_PROTOCOL_H
#define METALVR_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MVR_PROTOCOL_VERSION 2u
#define MVR_MAGIC 0x3152564du /* "MVR1" */
#define MVR_DEFAULT_PORT 37913
#define MVR_PORT_ENV "METALVR_PORT"
#define MVR_RESPONSE_BIT 0x8000u
#define MVR_MAX_PAYLOAD (1u << 20)

/* a swapchain's shares: image_count * array_size two-dimensional textures */
#define MVR_MAX_SWAPCHAIN_IMAGES 16u
#define MVR_MAX_LAYERS 16u
#define MVR_SHARE_NAME_LEN 64u

#pragma pack(push, 4)

typedef struct mvr_header {
  uint32_t magic;
  uint16_t type;
  uint16_t reserved;
  uint32_t size; /* payload bytes that follow */
} mvr_header;

enum mvr_message {
  MVR_MSG_HELLO = 1,
  MVR_MSG_CLOCK = 2,
  MVR_MSG_SESSION_BEGIN = 3,
  MVR_MSG_SESSION_END = 4,
  MVR_MSG_SWAPCHAIN_CREATE = 5,
  MVR_MSG_SWAPCHAIN_DESTROY = 6,
  MVR_MSG_FRAME_WAIT = 7,
  MVR_MSG_TRACKING = 8,
  MVR_MSG_FRAME_END = 9,
  MVR_MSG_HAPTIC = 10,
  MVR_MSG_SYSTEM = 11, /* the headset as it is now; it changes when one connects */
  MVR_MSG_REFERENCES = 12, /* the tracking sensors that watch the play space, where they stand */
  MVR_MSG_HANDS = 13,      /* the hands' joints, from a headset that tracks hands (MVR_SYSTEM_HAND_TRACKING) */
};

/* every response starts with one of these */
enum mvr_result {
  MVR_OK = 0,
  MVR_ERR_VERSION = 1,
  MVR_ERR_INVALID = 2,
  MVR_ERR_BUSY = 3, /* another client holds the session */
  MVR_ERR_SHARE = 4, /* a swapchain image could not be opened in the server */
  MVR_ERR_STATE = 5,
  MVR_ERR_INTERNAL = 6,
};

typedef struct mvr_vec3 {
  float x, y, z;
} mvr_vec3;

typedef struct mvr_quat {
  float x, y, z, w;
} mvr_quat;

typedef struct mvr_pose {
  mvr_quat orientation;
  mvr_vec3 position;
} mvr_pose;

typedef struct mvr_fov {
  float left, right, up, down;
} mvr_fov;

enum mvr_graphics_api {
  MVR_GFX_NONE = 0,
  MVR_GFX_D3D11 = 1, /* Mullion under Wine: images are shared D3D11 textures */
  MVR_GFX_METAL = 2, /* native: images are IOSurface-backed Metal textures */
  MVR_GFX_D3D12 = 3, /* Mullion under Wine: images are shared D3D12 textures */
};

enum mvr_system_flags {
  MVR_SYSTEM_HMD_CONNECTED = 1u << 0,
  MVR_SYSTEM_SIMULATED = 1u << 1,
  MVR_SYSTEM_HAND_TRACKING = 1u << 2, /* the headset can track bare hands (MVR_MSG_HANDS) */
  MVR_SYSTEM_PASSTHROUGH = 1u << 3,   /* the headset's home shows the room through its cameras */
};

/* Which headset the server drives. A driver names one only when it has identified it (metalvr_headsets.h lists
 * what each is); otherwise it says UNKNOWN, and nothing downstream assumes a model. */
enum mvr_headset {
  MVR_HEADSET_UNKNOWN = 0,
  MVR_HEADSET_SIMULATED = 1,
  MVR_HEADSET_RIFT_CV1 = 2,
  MVR_HEADSET_RIFT_S = 3,
  MVR_HEADSET_QUEST_1 = 4,
  MVR_HEADSET_QUEST_2 = 5,
  MVR_HEADSET_QUEST_3 = 6,
  MVR_HEADSET_QUEST_3S = 7,
  MVR_HEADSET_QUEST_PRO = 8,
  MVR_HEADSET_STEAM_FRAME = 9,
  MVR_HEADSET_VALVE_INDEX = 10,
  MVR_HEADSET_VIVE = 11,
  MVR_HEADSET_VIVE_PRO = 12,
  MVR_HEADSET_VIVE_PRO_2 = 13,
  MVR_HEADSET_PSVR2 = 14,
  MVR_HEADSET_COUNT
};

/* The physical controllers in the user's hands, which decides the interaction profile an application is given and
 * the controllers SteamVR shows. */
enum mvr_controller_kind {
  MVR_CONTROLLERS_UNKNOWN = 0,
  MVR_CONTROLLERS_TOUCH = 1,      /* Oculus Touch: Rift CV1, Rift S, Quest 1 and 2 */
  MVR_CONTROLLERS_TOUCH_PLUS = 2, /* Meta Touch Plus and Touch Pro: Quest 3, 3S and Pro */
  MVR_CONTROLLERS_INDEX = 3,      /* Valve Index controllers */
  MVR_CONTROLLERS_VIVE_WAND = 4,  /* HTC Vive controllers */
  MVR_CONTROLLERS_STEAM_FRAME = 5,
  MVR_CONTROLLERS_PS_SENSE = 6,   /* PlayStation VR2 Sense controllers */
  MVR_CONTROLLERS_COUNT
};

#define MVR_MAX_REFRESH_RATES 8u

typedef struct mvr_system_info {
  uint32_t flags;
  uint32_t eye_width;  /* recommended per-eye render size */
  uint32_t eye_height;
  float refresh_hz;    /* the rate the headset runs at now */
  float ipd;
  mvr_fov fov[2];
  char system_name[64];
  uint32_t headset;     /* mvr_headset */
  uint32_t controllers; /* mvr_controller_kind */
  /* the rates the headset can run at, refresh_hz among them; 0 entries when only refresh_hz is known */
  uint32_t refresh_rate_count;
  float refresh_rates[MVR_MAX_REFRESH_RATES];
  /* the play area the headset's own boundary encloses, meters; 0 when the headset has none or has not said */
  float play_area_width;
  float play_area_depth;
} mvr_system_info;

typedef struct mvr_hello_request {
  uint32_t version;
  uint32_t pid;
  uint32_t graphics_api;
  char application_name[128];
} mvr_hello_request;

typedef struct mvr_hello_response {
  uint32_t result;
  uint32_t version;
  int64_t server_time_ns;
  mvr_system_info system;
} mvr_hello_response;

typedef struct mvr_system_response {
  uint32_t result;
  mvr_system_info system;
} mvr_system_response;

typedef struct mvr_clock_response {
  uint32_t result;
  uint32_t reserved;
  int64_t server_time_ns;
} mvr_clock_response;

typedef struct mvr_result_response {
  uint32_t result;
} mvr_result_response;

/* pixel formats a swapchain may have; the server's Metal format is implied */
enum mvr_format {
  MVR_FORMAT_UNKNOWN = 0,
  MVR_FORMAT_RGBA8_UNORM = 1,
  MVR_FORMAT_RGBA8_SRGB = 2,
  MVR_FORMAT_BGRA8_UNORM = 3,
  MVR_FORMAT_BGRA8_SRGB = 4,
  MVR_FORMAT_RGBA16_FLOAT = 5,
  MVR_FORMAT_RGB10A2_UNORM = 6,
  MVR_FORMAT_RGBA16_UNORM = 7,
  /* BGRA in memory whose alpha channel holds nothing: a layer of it never blends */
  MVR_FORMAT_BGRX8_UNORM = 8,
  MVR_FORMAT_BGRX8_SRGB = 9,
  /* depth swapchains are accepted by the runtime and never sent to the server */
};

enum mvr_share_kind {
  /* name is the bootstrap name Mullion registered for the texture's Metal shared-texture port */
  MVR_SHARE_MULLION_BOOTSTRAP = 1,
  /* iosurface_id names a global IOSurface */
  MVR_SHARE_IOSURFACE_ID = 2,
};

typedef struct mvr_image_share {
  char name[MVR_SHARE_NAME_LEN];
  uint32_t iosurface_id;
  uint32_t reserved;
} mvr_image_share;

typedef struct mvr_swapchain_create_request {
  uint32_t swapchain_id; /* client-chosen, unique per connection */
  uint32_t width;
  uint32_t height;
  uint32_t format; /* mvr_format */
  uint32_t array_size;
  uint32_t reserved;
  uint32_t share_kind;
  uint32_t image_count;
  /* every share is a single-sampled two-dimensional texture: the share of image i, array slice s is
   * images[i * array_size + s], and image_count * array_size <= MVR_MAX_SWAPCHAIN_IMAGES. A runtime whose
   * application-facing images are arrays or multisampled copies or resolves into these when an image is released. */
  mvr_image_share images[MVR_MAX_SWAPCHAIN_IMAGES];
} mvr_swapchain_create_request;

typedef struct mvr_swapchain_destroy_request {
  uint32_t swapchain_id;
} mvr_swapchain_destroy_request;

typedef struct mvr_frame_wait_response {
  uint32_t result;
  uint32_t should_render;
  uint64_t frame_id;
  int64_t predicted_display_time_ns;
  int64_t period_ns;
  int64_t wake_time_ns; /* sleep until this server time, then begin the frame */
} mvr_frame_wait_response;

enum mvr_tracking_flags {
  MVR_TRACKING_HMD_CONNECTED = 1u << 0,
  MVR_TRACKING_HEAD_VALID = 1u << 1,
  /* the headset is worn and showing this session: the runtime may go VISIBLE and FOCUSED */
  MVR_TRACKING_PRESENTING = 1u << 2,
  /* head_battery holds the headset's charge */
  MVR_TRACKING_BATTERY_VALID = 1u << 3,
  MVR_TRACKING_CHARGING = 1u << 4,
};

enum mvr_controller_flags {
  MVR_CONTROLLER_CONNECTED = 1u << 0,
  MVR_CONTROLLER_POSE_VALID = 1u << 1,
  /* battery holds the controller's charge */
  MVR_CONTROLLER_BATTERY_VALID = 1u << 2,
  MVR_CONTROLLER_CHARGING = 1u << 3,
  /* fingers[] are measured (Index controllers), not derived from the buttons */
  MVR_CONTROLLER_FINGERS_VALID = 1u << 4,
  /* With split validity, POSE_VALID requires a valid position and orientation. */
  MVR_CONTROLLER_SPLIT_POSE = 1u << 5,
  MVR_CONTROLLER_ORIENTATION_VALID = 1u << 6,
  MVR_CONTROLLER_ORIENTATION_TRACKED = 1u << 7,
  MVR_CONTROLLER_POSITION_TRACKED = 1u << 8,
  MVR_CONTROLLER_LINEAR_VELOCITY_VALID = 1u << 9,
  MVR_CONTROLLER_ANGULAR_VELOCITY_VALID = 1u << 10,
};

/* one bit per digital input; which of them a controller has depends on its kind (mvr_controller_kind) */
enum mvr_button {
  MVR_BUTTON_PRIMARY = 1u << 0, /* A / X */
  MVR_BUTTON_SECONDARY = 1u << 1, /* B / Y */
  MVR_BUTTON_MENU = 1u << 2,
  MVR_BUTTON_SYSTEM = 1u << 3,
  MVR_BUTTON_TRIGGER = 1u << 4,
  MVR_BUTTON_SQUEEZE = 1u << 5,
  MVR_BUTTON_THUMBSTICK = 1u << 6,
  MVR_BUTTON_THUMBREST = 1u << 7,
  MVR_BUTTON_TRACKPAD = 1u << 8,  /* Index and Vive: the trackpad pressed in */
  MVR_BUTTON_BUMPER = 1u << 9,    /* Steam Frame */
  MVR_BUTTON_DPAD_UP = 1u << 10,  /* Steam Frame, left hand */
  MVR_BUTTON_DPAD_DOWN = 1u << 11,
  MVR_BUTTON_DPAD_LEFT = 1u << 12,
  MVR_BUTTON_DPAD_RIGHT = 1u << 13,
  MVR_BUTTON_VIEW = 1u << 14,     /* Steam Frame, left hand */
  /* Steam Frame, right hand: X and Y sit beside A and B */
  MVR_BUTTON_TERTIARY = 1u << 15,
  MVR_BUTTON_QUATERNARY = 1u << 16,
};

/* fingers[] of a controller that senses them, 0 open to 1 curled around the handle */
enum mvr_finger { MVR_FINGER_INDEX = 0, MVR_FINGER_MIDDLE = 1, MVR_FINGER_RING = 2, MVR_FINGER_PINKY = 3 };

typedef struct mvr_controller_state {
  uint32_t flags;
  uint32_t buttons; /* mvr_button, pressed */
  uint32_t touches; /* mvr_button, touched */
  float trigger;
  float squeeze;
  float thumbstick_x;
  float thumbstick_y;
  mvr_pose grip;
  mvr_pose aim;
  mvr_vec3 linear_velocity;
  mvr_vec3 angular_velocity;
  float trackpad_x; /* -1..1; a controller whose only pad is a trackpad (Vive) reports it here, not as a stick */
  float trackpad_y;
  float trackpad_force; /* 0..1, Index */
  float squeeze_force;  /* 0..1 beyond a full squeeze, Index */
  float fingers[4];     /* mvr_finger */
  float battery;        /* 0..1 when MVR_CONTROLLER_BATTERY_VALID */
} mvr_controller_state;

typedef struct mvr_view_state {
  mvr_pose pose;
  mvr_fov fov;
} mvr_view_state;

typedef struct mvr_tracking_request {
  int64_t time_ns; /* the server time to predict for; 0 for now */
} mvr_tracking_request;

typedef struct mvr_tracking_response {
  uint32_t result;
  uint32_t flags;
  /* names the tracking sample these poses came from; a frame rendered with them hands it back in FRAME_END so the
   * headset can reproject from the pose the frame was drawn with */
  uint64_t sample_id;
  int64_t time_ns;
  mvr_pose head;
  mvr_vec3 head_linear_velocity;
  mvr_vec3 head_angular_velocity;
  mvr_view_state views[2];
  mvr_controller_state controllers[2]; /* left, right */
  float head_battery; /* 0..1 when MVR_TRACKING_BATTERY_VALID */
  uint32_t reserved;
} mvr_tracking_response;

enum mvr_layer_type {
  MVR_LAYER_PROJECTION = 1,
  MVR_LAYER_QUAD = 2,
};

enum mvr_layer_flags {
  MVR_LAYER_BLEND_ALPHA = 1u << 0,
  MVR_LAYER_UNPREMULTIPLIED_ALPHA = 1u << 1,
  /* a quad's pose is relative to the head instead of the tracking space */
  MVR_LAYER_HEAD_LOCKED = 1u << 2,
};

/* A negative width or height mirrors the image along that axis: the region is the columns [x + width, x) or the
 * rows [y + height, y), read from x or y backwards. */
typedef struct mvr_rect {
  int32_t x, y, width, height;
} mvr_rect;

typedef struct mvr_layer_view {
  uint32_t swapchain_id;
  uint32_t image_index;
  uint32_t array_index;
  mvr_rect rect;
  mvr_pose pose; /* the pose the view was rendered with, tracking space */
  mvr_fov fov;
} mvr_layer_view;

typedef struct mvr_layer {
  uint32_t type;
  uint32_t flags;
  uint32_t eye_mask; /* quad: bit 0 left, bit 1 right */
  mvr_layer_view views[2]; /* projection: left, right; quad: views[0] is the image */
  mvr_pose quad_pose;
  float quad_width;
  float quad_height;
} mvr_layer;

typedef struct mvr_frame_end_request {
  uint64_t frame_id;
  uint64_t sample_id;
  int64_t display_time_ns;
  uint32_t layer_count; /* <= MVR_MAX_LAYERS; the layers follow */
  uint32_t reserved;
} mvr_frame_end_request;

/* A sensor of the tracking system (a Rift sensor, a base station): something that stands in the room. */
#define MVR_MAX_TRACKING_REFERENCES 4

enum mvr_reference_flags {
  MVR_REFERENCE_POSE_VALID = 1u << 0, /* it knows where it stands; before that only that it is there */
};

typedef struct mvr_tracking_reference {
  uint32_t flags;
  char serial[32];
  mvr_pose pose; /* in the tracking space, looking down its own -z with +y up */
} mvr_tracking_reference;

typedef struct mvr_references_response {
  uint32_t result;
  uint32_t count;
  mvr_tracking_reference references[MVR_MAX_TRACKING_REFERENCES];
} mvr_references_response;

/* A hand's joints in OpenXR's order (XR_HAND_JOINT_COUNT_EXT: palm, wrist, then each finger from its base), in the
 * tracking space, with each joint's radius in meters. */
#define MVR_HAND_JOINTS 26u

enum mvr_hand_flags {
  MVR_HAND_TRACKED = 1u << 0, /* the headset sees this hand now */
};

typedef struct mvr_hand {
  uint32_t flags;
  uint32_t reserved;
  mvr_pose joints[MVR_HAND_JOINTS];
  float radii[MVR_HAND_JOINTS];
} mvr_hand;

typedef struct mvr_hands_request {
  int64_t time_ns; /* the server time to predict for; 0 for now */
} mvr_hands_request;

typedef struct mvr_hands_response {
  uint32_t result;
  uint32_t reserved;
  int64_t time_ns;
  mvr_hand hands[2]; /* left, right */
} mvr_hands_response;

typedef struct mvr_haptic_request {
  uint32_t hand; /* 0 left, 1 right */
  float duration_s; /* <= 0 stops */
  float frequency_hz; /* 0 lets the device choose */
  float amplitude;
} mvr_haptic_request;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* METALVR_PROTOCOL_H */
