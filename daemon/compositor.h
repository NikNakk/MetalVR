// Composes a frame's layers into one stereo image: both eyes side by side, left eye in the left half.
#pragma once

#import <CoreVideo/CoreVideo.h>
#import <Metal/Metal.h>

#include <cstdint>
#include <vector>

#include "metalvr_protocol.h"

namespace metalvr {

struct ComposeLayerView {
  id<MTLTexture> texture = nil;
  mvr_rect rect{};
};

struct ComposeLayer {
  uint32_t type = 0;  // mvr_layer_type
  uint32_t flags = 0; // mvr_layer_flags
  uint32_t eye_mask = 3;
  ComposeLayerView views[2]; // projection: per eye; quad: views[0]
  mvr_pose quad_pose{};      // in tracking space, or head space with MVR_LAYER_HEAD_LOCKED
  float quad_width = 0;
  float quad_height = 0;
};

struct ComposeFrame {
  std::vector<ComposeLayer> layers;
  // where each eye was when the frame was rendered, for placing quads
  mvr_pose eye_pose[2]{};
  mvr_fov eye_fov[2]{};
  mvr_pose head_pose{};
};

class Compositor {
public:
  // a 4-byte-per-pixel texture as it is, written as a PNG (for looking at what an application sent)
  bool writeTexturePNG(id<MTLTexture> texture, const char *path);
  explicit Compositor(id<MTLDevice> device);
  ~Compositor();

  // the size of the whole frame; a change drops the buffer pool
  bool configure(uint32_t frame_width, uint32_t frame_height);
  uint32_t frameWidth() const { return width_; }
  uint32_t frameHeight() const { return height_; }

  // Draws the frame and waits for the GPU to finish it. The caller releases the returned buffer; null on failure.
  CVPixelBufferRef compose(const ComposeFrame &frame);

private:
  id<MTLRenderPipelineState> pipeline(bool blend);

  id<MTLDevice> device_;
  id<MTLCommandQueue> queue_;
  id<MTLLibrary> library_;
  id<MTLSamplerState> sampler_;
  id<MTLRenderPipelineState> pipelines_[2]; // opaque, blended
  CVPixelBufferPoolRef pool_ = nullptr;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
};

// writes a composed frame as a PNG, for inspecting what the server would send
bool writePixelBufferPNG(CVPixelBufferRef buffer, const char *path);

} // namespace metalvr
