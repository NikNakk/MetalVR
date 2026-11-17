// Opens the Rift sensors through the tracker and lets their cameras run for a few seconds, without a headset
// to track: what the tracker prints shows which sensors opened, their lens calibration, and whether the video
// arrives whole (a stream with gaps prints "UVC Dropping short frame").
//
//   rift_tracking_smoke [seconds] [radio id as 10 hex digits]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rift_tracking.h"

int main(int argc, char **argv) {
  const int seconds = argc > 1 ? atoi(argv[1]) : 5;
  uint8_t radio[5] = {0};
  if (argc > 2 && strlen(argv[2]) == 10)
    for (int i = 0; i < 5; ++i) {
      unsigned int byte = 0;
      sscanf(argv[2] + 2 * i, "%2x", &byte);
      radio[i] = (uint8_t)byte;
    }
  setvbuf(stdout, NULL, _IOLBF, 0);
  mvr_rift_tracker *tracker = mvr_rift_tracker_open(radio);
  if (!tracker) {
    printf("no Rift sensor could be opened\n");
    return 1;
  }
  printf("%d sensor(s) streaming for %d s\n", mvr_rift_tracker_sensor_count(tracker), seconds);
  // a 1280x960 frame is 1.23 MB; the cameras run at about 52 frames a second
  const uint64_t before = mvr_rift_tracker_video_bytes();
  sleep((unsigned int)seconds);
  const double frames = (double)(mvr_rift_tracker_video_bytes() - before) / (1280.0 * 960.0);
  printf("%.1f frames a second from each sensor\n", frames / seconds / mvr_rift_tracker_sensor_count(tracker));
  mvr_rift_tracker_close(tracker);
  printf("closed\n");
  return 0;
}
