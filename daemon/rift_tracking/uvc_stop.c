// Stopping a sensor's video without pulling its pictures from under the tracker.
//
// OpenHMD's rift_sensor_uvc_stream_stop frees the stream's frames as it stops it, and rift_sensor_free calls it
// before it has stopped the sensor's analysis threads: a thread still looking at a frame then hands back freed
// memory, and the process aborts in free(). It happens whenever a tracker is closed while its cameras run,
// which opening one does when a sensor has to be tried again.
//
// rift-sensor-usb.c is compiled with its calls to the stream's stop and clear sent here (CMakeLists.txt). The
// stop does all the original does except free the frames; the clear, which rift_sensor_free reaches only after
// its threads are gone, frees them and then clears the stream as before.
#include <libusb.h>
#include <stdlib.h>

#include "openhmdi.h"
#include "sensor/uvc.h"

int metalvr_uvc_stream_stop(struct rift_sensor_uvc_stream *stream)
{
	if (libusb_set_interface_alt_setting(stream->devh, 1, 0))
		LOGW("Failed to clear USB alt setting to 0 for sensor");

	libusb_lock_event_waiters(stream->usb_ctx);
	stream->video_running = false;
	/* Wait for the transfers in flight to come back */
	while (stream->active_transfers > 0)
		if (libusb_wait_for_event(stream->usb_ctx, NULL))
			break;
	libusb_unlock_event_waiters(stream->usb_ctx);
	return 0;
}

int metalvr_uvc_stream_clear(struct rift_sensor_uvc_stream *stream)
{
	if (stream->alloced_frames) {
		for (int i = 0; i < stream->n_alloced_frames; i++) {
			ohmd_video_frame *frame = stream->alloced_frames[i];
			free(frame->data);
			free(frame);
		}
		free(stream->alloced_frames);
		free(stream->free_frames);
		stream->alloced_frames = stream->free_frames = NULL;
		stream->n_alloced_frames = stream->n_free_frames = 0;
	}
	return rift_sensor_uvc_stream_clear(stream);
}
