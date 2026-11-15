// Decodes an Annex B H.264 or HEVC file the way a headset would, to prove that what metalvrd encodes is a
// stream a decoder accepts, and optionally writes one decoded frame as a PNG.
//
//   stream_check FILE [--hevc] [--png OUT.png] [--frame N]
//
// Prints the number of frames decoded; exit status 0 means every access unit decoded.
#import <CoreGraphics/CoreGraphics.h>
#import <CoreMedia/CoreMedia.h>
#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>
#import <VideoToolbox/VideoToolbox.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct Nal {
  const uint8_t *data;
  size_t size;
};

std::vector<Nal> splitAnnexB(const uint8_t *bytes, size_t size) {
  std::vector<Nal> nals;
  size_t i = 0, start = SIZE_MAX;
  while (i + 3 <= size) {
    const bool three = bytes[i] == 0 && bytes[i + 1] == 0 && bytes[i + 2] == 1;
    if (three) {
      if (start != SIZE_MAX) {
        size_t end = i;
        while (end > start && bytes[end - 1] == 0)
          --end;
        nals.push_back({bytes + start, end - start});
      }
      start = i + 3;
      i += 3;
    } else {
      ++i;
    }
  }
  if (start != SIZE_MAX && start < size)
    nals.push_back({bytes + start, size - start});
  return nals;
}

struct State {
  int decoded = 0;
  int failed = 0;
  int want_frame = -1;
  std::string png;
  size_t width = 0, height = 0;
};

bool writePNG(CVPixelBufferRef buffer, const char *path) {
  CVPixelBufferLockBaseAddress(buffer, kCVPixelBufferLock_ReadOnly);
  CGColorSpaceRef space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
  CGContextRef context = CGBitmapContextCreate(
      CVPixelBufferGetBaseAddress(buffer), CVPixelBufferGetWidth(buffer), CVPixelBufferGetHeight(buffer), 8,
      CVPixelBufferGetBytesPerRow(buffer), space,
      static_cast<uint32_t>(kCGImageAlphaNoneSkipFirst) | static_cast<uint32_t>(kCGBitmapByteOrder32Little));
  bool ok = false;
  if (context) {
    CGImageRef image = CGBitmapContextCreateImage(context);
    NSURL *url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:path]];
    CGImageDestinationRef destination =
        CGImageDestinationCreateWithURL((__bridge CFURLRef)url, (__bridge CFStringRef)UTTypePNG.identifier, 1, nullptr);
    if (image && destination) {
      CGImageDestinationAddImage(destination, image, nullptr);
      ok = CGImageDestinationFinalize(destination);
    }
    if (destination)
      CFRelease(destination);
    CGImageRelease(image);
    CGContextRelease(context);
  }
  CGColorSpaceRelease(space);
  CVPixelBufferUnlockBaseAddress(buffer, kCVPixelBufferLock_ReadOnly);
  return ok;
}

void onDecoded(void *refcon, void *, OSStatus status, VTDecodeInfoFlags, CVImageBufferRef image, CMTime, CMTime) {
  auto *state = static_cast<State *>(refcon);
  if (status != noErr || !image) {
    ++state->failed;
    return;
  }
  state->width = CVPixelBufferGetWidth(image);
  state->height = CVPixelBufferGetHeight(image);
  if (state->decoded == state->want_frame && !state->png.empty())
    if (!writePNG(image, state->png.c_str()))
      std::printf("could not write %s\n", state->png.c_str());
  ++state->decoded;
}

} // namespace

int main(int argc, char **argv) {
  @autoreleasepool {
    if (argc < 2) {
      std::printf("usage: stream_check FILE [--hevc] [--png OUT.png] [--frame N]\n");
      return 2;
    }
    bool hevc = false;
    State state;
    state.want_frame = 0;
    for (int i = 2; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--hevc")
        hevc = true;
      else if (arg == "--png" && i + 1 < argc)
        state.png = argv[++i];
      else if (arg == "--frame" && i + 1 < argc)
        state.want_frame = std::atoi(argv[++i]);
    }
    NSData *file = [NSData dataWithContentsOfFile:[NSString stringWithUTF8String:argv[1]]];
    if (!file.length) {
      std::printf("cannot read %s\n", argv[1]);
      return 1;
    }
    const std::vector<Nal> nals = splitAnnexB(static_cast<const uint8_t *>(file.bytes), file.length);

    // parameter sets first, then access units: a unit starts at the slice that holds the picture's first block
    std::vector<Nal> parameter_sets;
    std::vector<std::vector<Nal>> units;
    for (const Nal &nal : nals) {
      if (nal.size < 3)
        continue;
      const int type = hevc ? (nal.data[0] >> 1) & 0x3f : nal.data[0] & 0x1f;
      const bool parameter = hevc ? (type >= 32 && type <= 34) : (type == 7 || type == 8);
      const bool slice = hevc ? type <= 31 : (type == 1 || type == 5);
      if (parameter) {
        if (units.empty())
          parameter_sets.push_back(nal);
        continue;
      }
      if (!slice)
        continue;
      const bool first = hevc ? (nal.data[2] & 0x80) != 0 : (nal.data[1] & 0x80) != 0;
      if (first || units.empty())
        units.emplace_back();
      units.back().push_back(nal);
    }
    if (parameter_sets.empty() || units.empty()) {
      std::printf("no parameter sets or no slices: not a decodable stream\n");
      return 1;
    }

    std::vector<const uint8_t *> pointers;
    std::vector<size_t> sizes;
    for (const Nal &set : parameter_sets) {
      pointers.push_back(set.data);
      sizes.push_back(set.size);
    }
    CMVideoFormatDescriptionRef format = nullptr;
    OSStatus status = hevc ? CMVideoFormatDescriptionCreateFromHEVCParameterSets(
                                 kCFAllocatorDefault, pointers.size(), pointers.data(), sizes.data(), 4, nullptr, &format)
                           : CMVideoFormatDescriptionCreateFromH264ParameterSets(
                                 kCFAllocatorDefault, pointers.size(), pointers.data(), sizes.data(), 4, &format);
    if (status != noErr) {
      std::printf("the parameter sets are not valid (%d)\n", static_cast<int>(status));
      return 1;
    }

    NSDictionary *attributes = @{(id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)};
    VTDecompressionOutputCallbackRecord callback{onDecoded, &state};
    VTDecompressionSessionRef session = nullptr;
    status = VTDecompressionSessionCreate(kCFAllocatorDefault, format, nullptr, (__bridge CFDictionaryRef)attributes,
                                          &callback, &session);
    if (status != noErr) {
      std::printf("no decoder for this stream (%d)\n", static_cast<int>(status));
      return 1;
    }

    for (const auto &unit : units) {
      std::vector<uint8_t> avcc;
      for (const Nal &nal : unit) {
        const uint32_t length = static_cast<uint32_t>(nal.size);
        const uint8_t prefix[4] = {static_cast<uint8_t>(length >> 24), static_cast<uint8_t>(length >> 16),
                                   static_cast<uint8_t>(length >> 8), static_cast<uint8_t>(length)};
        avcc.insert(avcc.end(), prefix, prefix + 4);
        avcc.insert(avcc.end(), nal.data, nal.data + nal.size);
      }
      CMBlockBufferRef block = nullptr;
      if (CMBlockBufferCreateWithMemoryBlock(kCFAllocatorDefault, nullptr, avcc.size(), kCFAllocatorDefault, nullptr,
                                             0, avcc.size(), kCMBlockBufferAssureMemoryNowFlag, &block) != noErr ||
          CMBlockBufferReplaceDataBytes(avcc.data(), block, 0, avcc.size()) != noErr) {
        ++state.failed;
        continue;
      }
      CMSampleBufferRef sample = nullptr;
      const size_t sample_size = avcc.size();
      if (CMSampleBufferCreateReady(kCFAllocatorDefault, block, format, 1, 0, nullptr, 1, &sample_size, &sample) ==
          noErr) {
        if (VTDecompressionSessionDecodeFrame(session, sample, 0, nullptr, nullptr) != noErr)
          ++state.failed;
        CFRelease(sample);
      } else {
        ++state.failed;
      }
      CFRelease(block);
    }
    VTDecompressionSessionWaitForAsynchronousFrames(session);
    VTDecompressionSessionInvalidate(session);
    CFRelease(session);
    CFRelease(format);

    std::printf("%s: %zu access units, %d decoded (%zux%zu), %d failed\n", hevc ? "HEVC" : "H.264", units.size(),
                state.decoded, state.width, state.height, state.failed);
    return state.failed == 0 && state.decoded == static_cast<int>(units.size()) ? 0 : 1;
  }
}
