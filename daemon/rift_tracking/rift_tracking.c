// See rift_tracking.h. OpenHMD's tracker is included here as source rather than linked, because whether a device
// is seen by a sensor is in its private state.
#include "rift-tracker.c"

#include "rift_tracking.h"

#include <math.h>
#include <string.h>

#define TICK_US 1000 // the headset samples its IMU at 1000 Hz

struct mvr_rift_tracker {
	ohmd_context ohmd;
	rift_tracker_ctx *tracker;
	rift_tracked_device *devices[MVR_RIFT_DEVICES];
	rift_leds leds[MVR_RIFT_DEVICES];

	bool have_hmd_timestamp;
	uint32_t last_hmd_timestamp;
	bool have_touch_timestamp[MVR_RIFT_DEVICES];
	uint32_t last_touch_timestamp[MVR_RIFT_DEVICES];
};

/* What the tracker sources need of OpenHMD's core (openhmd.c), which is not compiled */
void *ohmd_allocfn(ohmd_context *ctx, const char *e_msg, size_t size)
{
	void *ret = calloc(1, size);
	if (!ret)
		ohmd_set_error(ctx, "%s", e_msg);
	return ret;
}

uint64_t ohmd_monotonic_conv(uint64_t ticks, uint64_t srcTicksPerSecond, uint64_t dstTicksPerSecond)
{
	return ticks / srcTicksPerSecond * dstTicksPerSecond +
		ticks % srcTicksPerSecond * dstTicksPerSecond / srcTicksPerSecond;
}

void metalvr_camera_tap_reset(void);

/* Each device's accelerometer, smoothed over a few samples: at rest it points up in the IMU's frame, which is what a
 * camera's pose of the device is checked against (metalvr_model_pose_update). Written by the IMU feed, read by the
 * sensors' analysis threads. */
static pthread_mutex_t accel_lock = PTHREAD_MUTEX_INITIALIZER;
static vec3f accel_smoothed[MVR_RIFT_DEVICES];
static bool accel_known[MVR_RIFT_DEVICES];

static void note_accel(int device, const vec3f *accel)
{
	if (device < 0 || device >= MVR_RIFT_DEVICES)
		return;
	pthread_mutex_lock(&accel_lock);
	if (!accel_known[device]) {
		accel_smoothed[device] = *accel;
		accel_known[device] = true;
	} else {
		for (int i = 0; i < 3; i++)
			accel_smoothed[device].arr[i] += 0.1f * (accel->arr[i] - accel_smoothed[device].arr[i]);
	}
	pthread_mutex_unlock(&accel_lock);
}

/* How far from the exposure it is paired with each camera picture starts to arrive, in ms: the tracker pairs a
 * picture with the exposure within 10 ms of it, so a spread that reaches that would pair some with the wrong one.
 * rift-sensor.c's calls come here first (CMakeLists.txt). */
static pthread_mutex_t offsets_lock = PTHREAD_MUTEX_INITIALIZER;
static double offsets_sum, offsets_sq, offsets_min = 1e9, offsets_max = -1e9;
static uint64_t offsets_count, offsets_unmatched;

bool metalvr_frame_captured(rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts,
		rift_tracker_exposure_info *out_info, const char *source)
{
	const bool matched = rift_tracker_frame_captured(ctx, local_ts, frame_start_local_ts, out_info, source);
	const double paired = matched ? ((int64_t) frame_start_local_ts - (int64_t) out_info->local_ts) / 1e6 : 1e9;
	pthread_mutex_lock(&offsets_lock);
	/* METALVR_RIFT_TIMING names a file that gets a line per picture: the sensor, when it started to arrive, whether
	 * it was paired, the exposure it was paired with (its count, its local time), and the offset in ms */
	static FILE *timing;
	static bool timing_checked;
	if (!timing_checked) {
		timing_checked = true;
		const char *path = getenv("METALVR_RIFT_TIMING");
		if (path && *path)
			timing = fopen(path, "w");
	}
	if (timing)
		fprintf(timing, "%s %llu %d %u %llu %.3f\n", source, (unsigned long long) frame_start_local_ts, matched ? 1 : 0,
				matched ? out_info->count : 0, matched ? (unsigned long long) out_info->local_ts : 0ULL, paired);
	if (matched) {
		offsets_sum += paired;
		offsets_sq += paired * paired;
		if (paired < offsets_min)
			offsets_min = paired;
		if (paired > offsets_max)
			offsets_max = paired;
	} else {
		offsets_unmatched++;
	}
	offsets_count++;
	pthread_mutex_unlock(&offsets_lock);
	return matched;
}

/* METALVR_RIFT_OBSERVATIONS names a file that gets a line for each pose a sensor found for a device: the device, the
 * sensor, the device's time, whether the tracker took it, how many LEDs matched of those visible, and the pose in
 * the tracker's space. rift-sensor.c's calls come here first (CMakeLists.txt). */
/* A pose a camera found that cannot be the device's: the device tilted otherwise than its accelerometer says (checked
 * while it is near rest, when the accelerometer reads gravity alone), or a controller where the other controller
 * is. The two controllers' LEDs are mirror images, and without these checks the tracker took one controller's LEDs
 * for the other's, and a wrong first pose set its IMU's gravity the wrong way, so it slid off between fixes. */
static rift_tracker_ctx *checking_tracker;
static uint64_t rejected_tilt[MVR_RIFT_DEVICES], rejected_overlap[MVR_RIFT_DEVICES];

#define MAX_TILT_DEGREES 25.0f
#define MIN_CONTROLLER_SEPARATION_M 0.07f

static const char *implausible(rift_tracked_device *dev_base, const posef *model_pose)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) dev_base;
	const int device = dev_base->id;
	if (device < 0 || device >= MVR_RIFT_DEVICES)
		return NULL;

	/* the IMU's orientation in the world this pose gives, and where it puts the accelerometer's up */
	vec3f accel;
	bool known;
	pthread_mutex_lock(&accel_lock);
	accel = accel_smoothed[device];
	known = accel_known[device];
	pthread_mutex_unlock(&accel_lock);
	const float g = ovec3f_get_length(&accel);
	posef imu_pose;
	oposef_apply(&dev->fusion_from_model, model_pose, &imu_pose);
	if (known && g > 8.8f && g < 10.8f) {
		vec3f up;
		oquatf_get_rotated(&imu_pose.orient, &accel, &up);
		if (up.y / g < cosf(MAX_TILT_DEGREES * (float) M_PI / 180.0f)) {
			rejected_tilt[device]++;
			return "tilt";
		}
	}

	/* a controller where the other one is, while the other is seen */
	if (device != MVR_RIFT_HMD && checking_tracker) {
		const int other = device == MVR_RIFT_RIGHT_TOUCH ? MVR_RIFT_LEFT_TOUCH : MVR_RIFT_RIGHT_TOUCH;
		for (int i = 0; i < checking_tracker->n_devices; i++) {
			rift_tracked_device_priv *o = checking_tracker->devices + i;
			if (o->base.id != other)
				continue;
			ohmd_lock_mutex(o->device_lock);
			const bool seen = o->last_observed_pose_ts &&
					o->device_time_ns - o->last_observed_pose_ts < (uint64_t) POSE_LOST_THRESHOLD * 1000000;
			const vec3f there = o->reported_pose.pos;
			ohmd_unlock_mutex(o->device_lock);
			posef device_pose;
			oposef_apply(&dev->device_from_fusion, &imu_pose, &device_pose);
			vec3f apart;
			ovec3f_subtract(&device_pose.pos, &there, &apart);
			if (seen && ovec3f_get_length(&apart) < MIN_CONTROLLER_SEPARATION_M) {
				rejected_overlap[device]++;
				return "overlap";
			}
		}
	}
	return NULL;
}

bool metalvr_model_pose_update(rift_tracked_device *dev, uint64_t local_ts, uint64_t frame_start_local_ts,
		rift_tracker_exposure_info *exposure_info, rift_pose_metrics *score, posef *pose, const char *source)
{
	const char *refused = implausible(dev, pose);
	const bool taken = !refused && rift_tracked_device_model_pose_update(dev, local_ts, frame_start_local_ts,
			exposure_info, score, pose, source);
	static FILE *observations;
	static bool checked;
	pthread_mutex_lock(&offsets_lock);
	if (!checked) {
		checked = true;
		const char *path = getenv("METALVR_RIFT_OBSERVATIONS");
		if (path && *path)
			observations = fopen(path, "w");
	}
	if (observations)
		fprintf(observations, "%d %s %llu %d %d %d %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", dev->id, source,
				(unsigned long long) frame_start_local_ts, taken ? 1 : refused ? (refused[0] == 't' ? -1 : -2) : 0,
				score ? score->matched_blobs : -1,
				score ? score->visible_leds : -1, pose->pos.x, pose->pos.y, pose->pos.z, pose->orient.x,
				pose->orient.y, pose->orient.z, pose->orient.w);
	pthread_mutex_unlock(&offsets_lock);
	return taken;
}

void mvr_rift_tracker_timing(char *out, int size)
{
	pthread_mutex_lock(&offsets_lock);
	if (offsets_count) {
		const uint64_t paired_count = offsets_count - offsets_unmatched;
		const double mean = paired_count ? offsets_sum / paired_count : 0;
		snprintf(out, size, "pictures start %.1f ms (sd %.1f, %.1f to %.1f) from their exposure; %llu of %llu unpaired",
				mean, sqrt(fmax(0.0, paired_count ? offsets_sq / paired_count - mean * mean : 0)),
				paired_count ? offsets_min : 0, paired_count ? offsets_max : 0,
				(unsigned long long) offsets_unmatched, (unsigned long long) offsets_count);
		const size_t used = strlen(out);
		snprintf(out + used, size > (int) used ? size - used : 0,
				"; poses refused for tilt %llu/%llu/%llu, for overlap %llu/%llu/%llu (headset/right/left)",
				(unsigned long long) rejected_tilt[0], (unsigned long long) rejected_tilt[1],
				(unsigned long long) rejected_tilt[2], (unsigned long long) rejected_overlap[0],
				(unsigned long long) rejected_overlap[1], (unsigned long long) rejected_overlap[2]);
		memset(rejected_tilt, 0, sizeof(rejected_tilt));
		memset(rejected_overlap, 0, sizeof(rejected_overlap));
	} else {
		snprintf(out, size, "no pictures yet");
	}
	offsets_sum = offsets_sq = 0;
	offsets_min = 1e9;
	offsets_max = -1e9;
	offsets_count = offsets_unmatched = 0;
	pthread_mutex_unlock(&offsets_lock);
}
void metalvr_ohmd_set_log(void (*sink)(const char *line));

void mvr_rift_tracker_set_log(void (*sink)(const char *line))
{
	metalvr_ohmd_set_log(sink);
}

/* The Rift sensors, which the tracker can all use (one on USB 2 sends JPEG) */
static int usable_sensors(void)
{
	libusb_context *usb = NULL;
	libusb_device **devs = NULL;
	int count = 0;
	if (libusb_init(&usb) < 0)
		return 0;
	if (libusb_get_device_list(usb, &devs) >= 0) {
		for (int i = 0; devs[i]; i++) {
			struct libusb_device_descriptor desc;
			if (libusb_get_device_descriptor(devs[i], &desc) == 0 && desc.idVendor == 0x2833 &&
					desc.idProduct == CV1_PID)
				count++;
		}
		libusb_free_device_list(devs, 1);
	}
	libusb_exit(usb);
	return count;
}

mvr_rift_tracker *mvr_rift_tracker_open(const uint8_t radio_id[5])
{
	mvr_rift_tracker *t = calloc(1, sizeof(*t));
	if (!t)
		return NULL;
	ohmd_monotonic_init(&t->ohmd);

	/* A sensor now and then refuses its first commands, most often after its last user died while it was
	 * streaming, and takes them the next time. The tracker opens the sensors once, as it starts, so the whole
	 * of it is started again, at most twice more. */
	const int expected = usable_sensors();
	for (int attempt = 0; attempt < 3; attempt++) {
		t->tracker = rift_tracker_new(&t->ohmd, radio_id);
		if (t->tracker && t->tracker->n_sensors >= expected)
			break;
		if (attempt == 2)
			break;
		printf("%d of %d Rift sensors opened; trying again\n", t->tracker ? t->tracker->n_sensors : 0,
				expected);
		if (t->tracker)
			rift_tracker_free(t->tracker);
		t->tracker = NULL;
		metalvr_camera_tap_reset();
		ohmd_sleep(0.5);
	}
	if (!t->tracker || t->tracker->n_sensors == 0) {
		mvr_rift_tracker_close(t);
		return NULL;
	}
	checking_tracker = t->tracker;
	return t;
}

int mvr_rift_tracker_sensor_count(const mvr_rift_tracker *t)
{
	return t && t->tracker ? t->tracker->n_sensors : 0;
}

uint64_t metalvr_usb_shim_iso_bytes(void);

uint64_t mvr_rift_tracker_video_bytes(void)
{
	return metalvr_usb_shim_iso_bytes();
}

void mvr_rift_tracker_close(mvr_rift_tracker *t)
{
	if (!t)
		return;
	if (t->tracker && checking_tracker == t->tracker)
		checking_tracker = NULL;
	if (t->tracker)
		rift_tracker_free(t->tracker);
	metalvr_camera_tap_reset();
	for (int i = 0; i < MVR_RIFT_DEVICES; i++)
		free(t->leds[i].points);
	free(t);
}

struct add_device_call {
	mvr_rift_tracker *t;
	int device;
	const mvr_rift_led *leds;
	int count;
	const float *imu_position;
	int flipped;
	int result;
};

static int add_device(mvr_rift_tracker *t, int device, const mvr_rift_led *leds, int count,
		const float imu_position[3], int flipped);

static void *add_device_thread(void *arg)
{
	struct add_device_call *call = arg;
	call->result = add_device(call->t, call->device, call->leds, call->count, call->imu_position, call->flipped);
	return NULL;
}

/* Building a device's LED search model is done with the stack the tracker's own threads have (ohmd_compat.h) */
int mvr_rift_tracker_add_device(mvr_rift_tracker *t, int device, const mvr_rift_led *leds, int count,
		const float imu_position[3], int flipped)
{
	struct add_device_call call = { t, device, leds, count, imu_position, flipped, 0 };
	pthread_t thread;
	if (metalvr_pthread_create_large(&thread, NULL, add_device_thread, &call) != 0)
		return 0;
	pthread_join(thread, NULL);
	return call.result;
}

static int add_device(mvr_rift_tracker *t, int device, const mvr_rift_led *leds, int count,
		const float imu_position[3], int flipped)
{
	if (!t || device < 0 || device >= MVR_RIFT_DEVICES || t->devices[device] || count < 4 || count > 255)
		return 0;

	rift_leds *model = &t->leds[device];
	model->points = calloc((size_t) count, sizeof(rift_led));
	if (!model->points)
		return 0;
	model->num_points = (uint8_t) count;
	model->radius_mm = 4 / 1000.0;
	for (int i = 0; i < count; i++) {
		rift_led *led = model->points + i;
		led->id = i;
		memcpy(led->pos.arr, leds[i].position, sizeof(led->pos.arr));
		memcpy(led->dir.arr, leds[i].direction, sizeof(led->dir.arr));
		ovec3f_normalize_me(&led->dir);
		led->pattern = 0xff;
	}

	quatf identity = {{ 0.0, 0.0, 0.0, 1.0 }};
	quatf turned = {{ 0.0, 1.0, 0.0, 0.0 }}; /* 180 degrees about Y */
	vec3f imu_pos = {{ imu_position[0], imu_position[1], imu_position[2] }};
	vec3f model_pos = {{ 0.0, 0.0, 0.0 }};
	posef imu_pose, model_pose;
	oposef_init(&imu_pose, &imu_pos, &identity);
	oposef_init(&model_pose, &model_pos, flipped ? &turned : &identity);

	/* Only reported in the tracker's debug output: the samples it is given are already calibrated */
	rift_tracked_device_imu_calibration calibration = { 0 };
	calibration.accel_matrix[0] = calibration.accel_matrix[4] = calibration.accel_matrix[8] = 1.0;
	calibration.gyro_matrix[0] = calibration.gyro_matrix[4] = calibration.gyro_matrix[8] = 1.0;

	t->devices[device] = rift_tracker_add_device(t->tracker, device, &imu_pose, &model_pose, model, &calibration);
	return t->devices[device] != NULL;
}

void mvr_rift_tracker_hmd_report(mvr_rift_tracker *t, const uint8_t *report, int size)
{
	pkt_tracker_sensor s;
	if (!t || !t->devices[MVR_RIFT_HMD] || size != 64)
		return;
	memset(&s, 0, sizeof(s));
	if (!decode_tracker_sensor_msg_dk2(&s, report, size) || s.num_samples == 0)
		return;

	uint64_t local_ts = ohmd_monotonic_get(&t->ohmd);
	const vec3f mag = {{ s.mag[0] * 0.0001f, s.mag[1] * 0.0001f, s.mag[2] * 0.0001f }};

	/* After a gap, the first sample here carries the whole of it */
	uint32_t dt = TICK_US;
	if (t->have_hmd_timestamp) {
		dt = s.timestamp - t->last_hmd_timestamp;
		dt -= (s.num_samples - 1) * TICK_US;
	}
	uint32_t total_dt = (s.num_samples - 1) * TICK_US + dt;
	uint32_t device_ts = s.timestamp - total_dt;
	bool sent_exposure = false;
	local_ts -= (uint64_t) total_dt * 1000;

	for (int i = 0; i < s.num_samples; i++) {
		/* An exposure earlier than this sample is reported before it */
		if (!sent_exposure && (int32_t) (device_ts - s.exposure_timestamp) >= 0) {
			rift_tracker_on_new_exposure(t->tracker, device_ts, s.exposure_count, s.exposure_timestamp,
					s.led_pattern_phase);
			sent_exposure = true;
		}
		/* The headset applies its own calibration (it is configured to) */
		const vec3f accel = {{ s.samples[i].accel[0] * 0.0001f, s.samples[i].accel[1] * 0.0001f,
				s.samples[i].accel[2] * 0.0001f }};
		const vec3f gyro = {{ s.samples[i].gyro[0] * 0.0001f, s.samples[i].gyro[1] * 0.0001f,
				s.samples[i].gyro[2] * 0.0001f }};
		note_accel(MVR_RIFT_HMD, &accel);
		rift_tracked_device_imu_update(t->devices[MVR_RIFT_HMD], local_ts, device_ts, dt / 1000000.0f, &gyro,
				&accel, &mag);
		device_ts += dt;
		local_ts += (uint64_t) dt * 1000;
		dt = TICK_US;
	}
	t->last_hmd_timestamp = s.timestamp;
	t->have_hmd_timestamp = true;

	if (!sent_exposure)
		rift_tracker_on_new_exposure(t->tracker, s.timestamp, s.exposure_count, s.exposure_timestamp,
				s.led_pattern_phase);
}

void mvr_rift_tracker_touch_imu(mvr_rift_tracker *t, int device, uint32_t timestamp_us, const float gyro_in[3],
		const float accel_in[3])
{
	if (!t || device <= MVR_RIFT_HMD || device >= MVR_RIFT_DEVICES || !t->devices[device])
		return;

	int32_t dt = 2000; /* the controllers report at 500 Hz */
	if (t->have_touch_timestamp[device])
		dt = (int32_t) (timestamp_us - t->last_touch_timestamp[device]);
	if (dt <= 0)
		return; /* the same sample again */

	const uint32_t device_ts = timestamp_us - (uint32_t) dt;
	const uint64_t local_ts = ohmd_monotonic_get(&t->ohmd) - (uint64_t) dt * 1000;
	const vec3f gyro = {{ gyro_in[0], gyro_in[1], gyro_in[2] }};
	const vec3f accel = {{ accel_in[0], accel_in[1], accel_in[2] }};
	const vec3f mag = {{ 0.0f, 0.0f, 0.0f }};
	note_accel(device, &accel);
	rift_tracked_device_imu_update(t->devices[device], local_ts, device_ts, 1e-6f * dt, &gyro, &accel, &mag);
	t->last_touch_timestamp[device] = timestamp_us;
	t->have_touch_timestamp[device] = true;
}

bool metalvr_sensor_pose(int sensor_id, posef *pose);

int mvr_rift_tracker_sensor(mvr_rift_tracker *t, int index, char serial[32], float orientation[4], float position[3])
{
	if (!t || !t->tracker || index < 0 || index >= t->tracker->n_sensors)
		return 0;
	snprintf(serial, 32, "%s", rift_sensor_serial_no(t->tracker->sensors[index]));
	posef pose;
	if (!metalvr_sensor_pose(index, &pose))
		return 1;
	/* The tracker's cameras look down +z with +y down; turned half round about x they look down -z, +y up */
	const quatf flip = {{ 1.0, 0.0, 0.0, 0.0 }};
	quatf turned;
	oquatf_mult(&pose.orient, &flip, &turned);
	memcpy(orientation, turned.arr, sizeof(float) * 4);
	memcpy(position, pose.pos.arr, sizeof(float) * 3);
	return 2;
}

int mvr_rift_tracker_pose(mvr_rift_tracker *t, int device, mvr_rift_pose *out)
{
	if (!t || device < 0 || device >= MVR_RIFT_DEVICES || !t->devices[device])
		return 0;
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) t->devices[device];
	posef pose;
	vec3f vel, ang_vel;
	rift_tracked_device_get_view_pose(t->devices[device], &pose, &vel, NULL, &ang_vel);

	ohmd_lock_mutex(dev->device_lock);
	const uint64_t now = dev->device_time_ns, seen = dev->last_observed_pose_ts;
	ohmd_unlock_mutex(dev->device_lock);
	if (now == 0)
		return 0;

	memcpy(out->orientation, pose.orient.arr, sizeof(out->orientation));
	memcpy(out->position, pose.pos.arr, sizeof(out->position));
	memcpy(out->linear_velocity, vel.arr, sizeof(out->linear_velocity));
	memcpy(out->angular_velocity, ang_vel.arr, sizeof(out->angular_velocity));
	out->position_tracked = seen != 0 && now - seen < (uint64_t) POSE_LOST_THRESHOLD * 1000000;
	out->device_time_ns = now;
	return 1;
}
