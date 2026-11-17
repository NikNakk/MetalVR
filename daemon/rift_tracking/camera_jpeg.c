// OpenHMD's JPEG decoder (its ohmd-jpeg.c, which needs libjpeg) on macOS's ImageIO, with the same names and
// meanings. A Rift sensor on USB 2 sends its pictures as JPEG, there not being the bandwidth for them raw; with this
// such a sensor is used like the others. HAVE_LIBJPEG is defined for OpenHMD's sources (CMakeLists.txt).
#include <CoreFoundation/CoreFoundation.h>
#include <CoreGraphics/CoreGraphics.h>
#include <ImageIO/ImageIO.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "ohmd-jpeg.h"

// a sensor's analysis holds a few frames at once (rift-sensor.c, NUM_CAPTURE_BUFFERS); more than this means one is
// never handed back
#define MAX_QUEUE_FRAMES 16

struct ohmd_jpeg_decoder {
  ohmd_context *ohmd_ctx;
  ohmd_mutex *frames_lock;
  ohmd_video_frame *free_frames[MAX_QUEUE_FRAMES];
  int n_free_frames;
  ohmd_video_frame *alloced_frames[MAX_QUEUE_FRAMES];
  int n_alloced_frames;
  CGColorSpaceRef gray;
  uint8_t *patched; // a frame with the Huffman tables put in, when it came without them
  size_t patched_size;
};

// Motion JPEG leaves out its Huffman tables and uses those of the JPEG standard's Annex K (K.3), which a decoder
// is expected to assume and ImageIO does not. A frame without a DHT segment gets this one before its scan.
static const uint8_t dc_bits[16] = {0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t dc_chroma_bits[16] = {0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
static const uint8_t dc_values[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const uint8_t ac_luma_bits[16] = {0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d};
static const uint8_t ac_luma_values[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14,
    0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09,
    0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a,
    0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65,
    0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
    0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
    0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca,
    0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};
static const uint8_t ac_chroma_bits[16] = {0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77};
static const uint8_t ac_chroma_values[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32,
    0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16,
    0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64,
    0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86,
    0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8,
    0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
    0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

static size_t put_table(uint8_t *out, uint8_t id, const uint8_t bits[16], const uint8_t *values, size_t count) {
  out[0] = id;
  memcpy(out + 1, bits, 16);
  memcpy(out + 17, values, count);
  return 17 + count;
}

// the standard tables as one DHT segment, marker included
static size_t standard_dht(uint8_t *out) {
  size_t n = 4;
  n += put_table(out + n, 0x00, dc_bits, dc_values, 12);
  n += put_table(out + n, 0x01, dc_chroma_bits, dc_values, 12);
  n += put_table(out + n, 0x10, ac_luma_bits, ac_luma_values, 162);
  n += put_table(out + n, 0x11, ac_chroma_bits, ac_chroma_values, 162);
  out[0] = 0xff;
  out[1] = 0xc4;
  out[2] = (uint8_t)((n - 2) >> 8);
  out[3] = (uint8_t)(n - 2);
  return n;
}

// Where the scan starts (its SOS marker) when no DHT segment comes before it; 0 when the frame has its tables or
// is not a JPEG this understands. Walks the segments before the scan, which carry their lengths.
static size_t scan_without_tables(const uint8_t *p, size_t size) {
  if (size < 4 || p[0] != 0xff || p[1] != 0xd8)
    return 0;
  size_t at = 2;
  while (at + 4 <= size) {
    if (p[at] != 0xff)
      return 0;
    const uint8_t marker = p[at + 1];
    if (marker == 0xc4)
      return 0;
    if (marker == 0xda)
      return at;
    at += 2 + (((size_t)p[at + 2] << 8) | p[at + 3]);
  }
  return 0;
}

static void release_frame(ohmd_video_frame *frame, ohmd_jpeg_decoder *jd) {
  assert(frame->owner == jd);
  ohmd_lock_mutex(jd->frames_lock);
  assert(jd->n_free_frames < jd->n_alloced_frames);
  jd->free_frames[jd->n_free_frames++] = frame;
  ohmd_unlock_mutex(jd->frames_lock);
}

static ohmd_video_frame *output_frame(ohmd_jpeg_decoder *jd, const ohmd_video_frame *in) {
  ohmd_video_frame *frame = NULL;
  ohmd_lock_mutex(jd->frames_lock);
  if (jd->n_free_frames > 0) {
    frame = jd->free_frames[--jd->n_free_frames];
  } else if (jd->n_alloced_frames < MAX_QUEUE_FRAMES) {
    frame = ohmd_alloc(jd->ohmd_ctx, sizeof(ohmd_video_frame));
    const size_t size = (size_t)in->width * in->height;
    frame->data = frame ? ohmd_alloc(jd->ohmd_ctx, size) : NULL;
    if (frame && frame->data) {
      frame->format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;
      frame->data_block_size = frame->data_size = size;
      frame->releasefn = (ohmd_video_frame_release_func)release_frame;
      frame->owner = jd;
      jd->alloced_frames[jd->n_alloced_frames++] = frame;
    } else {
      if (frame)
        free(frame);
      frame = NULL;
    }
  }
  ohmd_unlock_mutex(jd->frames_lock);
  return frame;
}

ohmd_jpeg_decoder *ohmd_jpeg_decoder_new(ohmd_context *ohmd_ctx) {
  ohmd_jpeg_decoder *jd = ohmd_alloc(ohmd_ctx, sizeof(ohmd_jpeg_decoder));
  if (!jd)
    return NULL;
  jd->ohmd_ctx = ohmd_ctx;
  jd->frames_lock = ohmd_create_mutex(ohmd_ctx);
  jd->gray = CGColorSpaceCreateDeviceGray();
  return jd;
}

void ohmd_jpeg_decoder_free(ohmd_jpeg_decoder *jd) {
  if (!jd)
    return;
  for (int i = 0; i < jd->n_alloced_frames; i++) {
    free(jd->alloced_frames[i]->data);
    free(jd->alloced_frames[i]);
  }
  if (jd->gray)
    CGColorSpaceRelease(jd->gray);
  free(jd->patched);
  if (jd->frames_lock)
    ohmd_destroy_mutex(jd->frames_lock);
  free(jd);
}

bool ohmd_jpeg_decoder_decode(ohmd_jpeg_decoder *jd, ohmd_video_frame *in, ohmd_video_frame **out) {
  *out = NULL;
  const uint8_t *bytes = in->data;
  size_t size = in->data_size;
  const size_t scan = scan_without_tables(bytes, size);
  if (scan) {
    uint8_t dht[4 + 2 * 29 + 2 * 179];
    const size_t dht_size = standard_dht(dht);
    if (jd->patched_size < size + dht_size) {
      free(jd->patched);
      jd->patched = malloc(size + dht_size);
      jd->patched_size = jd->patched ? size + dht_size : 0;
      if (!jd->patched)
        return false;
    }
    memcpy(jd->patched, bytes, scan);
    memcpy(jd->patched + scan, dht, dht_size);
    memcpy(jd->patched + scan + dht_size, bytes + scan, size - scan);
    bytes = jd->patched;
    size += dht_size;
  }

  CFDataRef data = CFDataCreateWithBytesNoCopy(NULL, bytes, (CFIndex)size, kCFAllocatorNull);
  CGImageSourceRef source = data ? CGImageSourceCreateWithData(data, NULL) : NULL;
  CGImageRef image = source ? CGImageSourceCreateImageAtIndex(source, 0, NULL) : NULL;
  bool ok = false;
  if (image && (int)CGImageGetWidth(image) == in->width && (int)CGImageGetHeight(image) == in->height) {
    ohmd_video_frame *frame = output_frame(jd, in);
    if (frame) {
      CGContextRef context = CGBitmapContextCreate(frame->data, (size_t)in->width, (size_t)in->height, 8,
                                                   (size_t)in->width, jd->gray, (uint32_t)kCGImageAlphaNone);
      if (context) {
        CGContextSetBlendMode(context, kCGBlendModeCopy);
        CGContextDrawImage(context, CGRectMake(0, 0, in->width, in->height), image);
        CGContextRelease(context);
        frame->start_ts = in->start_ts;
        frame->pts = in->pts;
        frame->stride = in->width;
        frame->width = in->width;
        frame->height = in->height;
        *out = frame;
        ok = true;
      } else {
        ohmd_video_frame_release(frame);
      }
    }
  } else if (image) {
    LOGW("Unexpected JPEG frame size %zux%zu, expected %dx%d", CGImageGetWidth(image), CGImageGetHeight(image),
         in->width, in->height);
  }
  if (image)
    CGImageRelease(image);
  if (source)
    CFRelease(source);
  if (data)
    CFRelease(data);
  return ok;
}
