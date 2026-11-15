// Hardware video encoding of composed frames, frame in / frame out, as Annex B.
#pragma once

#import <CoreVideo/CoreVideo.h>

#include <cstdint>
#include <functional>
#include <vector>

#include "driver.h"

namespace metalvr {

struct EncodedFrame {
  FrameInfo info;
  // set on a keyframe whose parameter sets differ from the last ones reported
  std::vector<uint8_t> config;
  std::vector<uint8_t> data;
};

class Encoder {
public:
  using Output = std::function<void(EncodedFrame &&)>;

  explicit Encoder(Output output);
  ~Encoder();

  // (re)creates the session when the parameters differ from the current ones; a bitrate change alone is applied
  // to the running session
  bool configure(const VideoParams &params);

  // queues `buffer` for encoding; the output callback runs on an encoder thread
  // `info` names the frame; its codec, keyframe and encoded_ns fields are filled in on the way out
  bool encode(CVPixelBufferRef buffer, const FrameInfo &info, bool force_keyframe);

  // waits for every queued frame to come out
  void flush();

  // since the last call: how long frames took from being handed in to coming out, and how many did
  struct Stats {
    uint64_t frames = 0;
    double mean_latency_ms = 0;
    double max_latency_ms = 0;
    uint64_t dropped_total = 0; // frames the encoder gave up on since it was made
  };
  Stats takeStats();

  struct Impl;

private:
  Impl *impl_;
};

} // namespace metalvr
