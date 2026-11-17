// Positional tracking for the Rift CV1 with its sensors: the Constellation cameras watch the infrared LEDs on the
// headset and the Touch controllers, and what they see is fused with each device's IMU.
//
// The tracking itself is OpenHMD's (external/OpenHMD-rift, Boost Software License 1.0), compiled as it is; this
// is the narrow interface the Rift driver uses, so that nothing else has to include OpenHMD's headers.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mvr_rift_tracker mvr_rift_tracker;

// the device numbers OpenHMD's tracker knows
enum { MVR_RIFT_HMD = 0, MVR_RIFT_RIGHT_TOUCH = 1, MVR_RIFT_LEFT_TOUCH = 2, MVR_RIFT_DEVICES = 3 };

typedef struct mvr_rift_led {
  float position[3];  // metres, in the device's own frame
  float direction[3]; // the way it faces
} mvr_rift_led;

typedef struct mvr_rift_pose {
  float orientation[4]; // x, y, z, w
  float position[3];    // metres; the origin is where the headset was when a sensor first saw it
  float linear_velocity[3];
  float angular_velocity[3];
  int position_tracked;    // a sensor saw the device within the last half second
  uint64_t device_time_ns; // the device's own clock at this pose
} mvr_rift_pose;

// Where the tracker's own messages go, a line at a time and from any of its threads; null: to stdout. They begin
// "[II] ", "[WW] " or "[EE] " for information, warnings and errors, or with neither.
void mvr_rift_tracker_set_log(void (*sink)(const char *line));

// Opens every Rift sensor on USB 3 and starts their cameras, synchronized to the headset whose radio this is.
// Null when no sensor can be opened.
mvr_rift_tracker *mvr_rift_tracker_open(const uint8_t radio_id[5]);
int mvr_rift_tracker_sensor_count(const mvr_rift_tracker *tracker);
// every byte of video received from the sensors since the process started (headers included)
uint64_t mvr_rift_tracker_video_bytes(void);
void mvr_rift_tracker_close(mvr_rift_tracker *tracker);

// The sensor the tracker numbers `index` (from 0, below the sensor count): its serial number, and where it stands
// once the tracker has worked that out from seeing the headset. The pose is in the same space as the devices';
// the sensor looks down its own -z with +y up. Returns 0 for no such sensor, 1 for a sensor not placed yet
// (the pose is untouched), 2 for one with a pose.
int mvr_rift_tracker_sensor(mvr_rift_tracker *tracker, int index, char serial[32], float orientation[4],
                            float position[3]);

// What each camera sees, for telling why a device is or is not tracked. Cameras are numbered from 0 in the order
// their first pictures arrived.
int mvr_rift_camera_count(void);
// the bright spots in the camera's newest picture (-1: no such camera), and how many pictures it has taken
int mvr_rift_camera_blobs(int camera, uint64_t *frames);
// The camera's next picture, 8 bits a pixel, waited for up to a second. Returns 0 when none came or it does not fit.
int mvr_rift_camera_picture(int camera, uint8_t *pixels, int capacity, int *width, int *height);

// A device to track, by its LED model. `flipped` is for the headset, whose LED model faces the other way.
// Returns 0 when the device cannot be added.
int mvr_rift_tracker_add_device(mvr_rift_tracker *tracker, int device, const mvr_rift_led *leds, int count,
                                const float imu_position[3], int flipped);

// The headset's IMU report (IN report 11, with its id): its samples, and when the LEDs were last lit.
void mvr_rift_tracker_hmd_report(mvr_rift_tracker *tracker, const uint8_t *report, int size);

// One calibrated IMU sample of a Touch controller: rad/s and m/s^2, with the controller's timestamp in microseconds.
void mvr_rift_tracker_touch_imu(mvr_rift_tracker *tracker, int device, uint32_t timestamp_us, const float gyro[3],
                                const float accel[3]);

// Returns 0 when the device was not added or has no IMU sample yet.
int mvr_rift_tracker_pose(mvr_rift_tracker *tracker, int device, mvr_rift_pose *out);

#ifdef __cplusplus
}
#endif
