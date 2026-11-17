// What the sensors' cameras see, for telling why a device is or is not tracked. rift-sensor.c is compiled with
// its calls to blobwatch_process sent here (see CMakeLists.txt); each is passed on unchanged, and the number of
// bright spots found, and on request the picture itself, are kept.
#include <pthread.h>
#include <string.h>
#include <sys/time.h>

#include "rift-sensor-blobwatch.h"
#include "rift-sensor-pose-search.h"

#include "rift_tracking.h"

#define MAX_CAMERAS 4

struct camera {
	blobwatch *bw; /* names the camera: each has its own */
	int blobs;
	uint64_t frames;
	bool wanted, taken;
	uint8_t *picture;
	int width, height;
};

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t taken = PTHREAD_COND_INITIALIZER;
static struct camera cameras[MAX_CAMERAS];
static int camera_count;

void metalvr_blobwatch_process(blobwatch *bw, uint8_t *frame, int width, int height, uint8_t led_pattern_phase,
		rift_led *leds, uint8_t num_leds, blobservation **output)
{
	blobwatch_process(bw, frame, width, height, led_pattern_phase, leds, num_leds, output);

	pthread_mutex_lock(&lock);
	struct camera *c = NULL;
	for (int i = 0; i < camera_count; i++)
		if (cameras[i].bw == bw)
			c = &cameras[i];
	if (!c && camera_count < MAX_CAMERAS) {
		c = &cameras[camera_count++];
		c->bw = bw;
	}
	if (c) {
		c->blobs = (output && *output) ? (*output)->num_blobs : 0;
		c->frames++;
		if (c->wanted) {
			uint8_t *picture = realloc(c->picture, (size_t) width * height);
			if (picture) {
				memcpy(picture, frame, (size_t) width * height);
				c->picture = picture;
				c->width = width;
				c->height = height;
				c->taken = true;
			}
			c->wanted = false;
			pthread_cond_broadcast(&taken);
		}
	}
	pthread_mutex_unlock(&lock);
}

/* Each sensor's pose finder, where the tracker keeps the sensor's place in the room once it has worked it out.
 * rift-sensor.c's call to set one up comes here first. */
static rift_pose_finder *finders[MAX_CAMERAS];
static int finder_count;

void metalvr_pose_finder_init(rift_pose_finder *pf, rift_sensor_camera_params *calib, rift_pose_finder_cb pose_cb,
		void *pose_cb_data)
{
	rift_pose_finder_init(pf, calib, pose_cb, pose_cb_data);
	pthread_mutex_lock(&lock);
	if (finder_count < MAX_CAMERAS)
		finders[finder_count++] = pf;
	pthread_mutex_unlock(&lock);
}

/* The camera-to-room pose of the sensor the tracker numbers `sensor_id`; false until it has one. The tracker
 * writes it once, from its own thread, and sets the flag after: a pose read as the flag turns is at worst the
 * one before. */
bool metalvr_sensor_pose(int sensor_id, posef *pose)
{
	bool have = false;
	pthread_mutex_lock(&lock);
	for (int i = 0; i < finder_count; i++)
		if (finders[i]->sensor_id == sensor_id && finders[i]->have_camera_pose) {
			*pose = finders[i]->camera_pose;
			have = true;
		}
	pthread_mutex_unlock(&lock);
	return have;
}

/* The cameras are the tracker's: when it closes they are forgotten */
void metalvr_camera_tap_reset(void)
{
	pthread_mutex_lock(&lock);
	finder_count = 0;
	for (int i = 0; i < camera_count; i++)
		free(cameras[i].picture);
	memset(cameras, 0, sizeof(cameras));
	camera_count = 0;
	pthread_mutex_unlock(&lock);
}

int mvr_rift_camera_count(void)
{
	pthread_mutex_lock(&lock);
	const int count = camera_count;
	pthread_mutex_unlock(&lock);
	return count;
}

int mvr_rift_camera_blobs(int camera, uint64_t *frames)
{
	int blobs = -1;
	pthread_mutex_lock(&lock);
	if (camera >= 0 && camera < camera_count) {
		blobs = cameras[camera].blobs;
		if (frames)
			*frames = cameras[camera].frames;
	}
	pthread_mutex_unlock(&lock);
	return blobs;
}

int mvr_rift_camera_picture(int camera, uint8_t *pixels, int capacity, int *width, int *height)
{
	int ok = 0;
	struct timeval now;
	struct timespec deadline;
	gettimeofday(&now, NULL);
	deadline.tv_sec = now.tv_sec + 1;
	deadline.tv_nsec = now.tv_usec * 1000;

	pthread_mutex_lock(&lock);
	if (camera >= 0 && camera < camera_count) {
		struct camera *c = &cameras[camera];
		c->taken = false;
		c->wanted = true;
		while (!c->taken && c->bw)
			if (pthread_cond_timedwait(&taken, &lock, &deadline) != 0)
				break;
		if (c->taken && c->width * c->height <= capacity) {
			memcpy(pixels, c->picture, (size_t) c->width * c->height);
			*width = c->width;
			*height = c->height;
			ok = 1;
		}
		c->wanted = false;
	}
	pthread_mutex_unlock(&lock);
	return ok;
}
