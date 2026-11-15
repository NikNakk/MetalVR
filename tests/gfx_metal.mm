#include "gfx.h"

#import <Metal/Metal.h>

#include <cstdio>
#include <map>

namespace {

struct Chain {
  uint32_t width = 0, height = 0, array_size = 1;
  std::vector<id<MTLTexture>> images;
};

class MetalGraphics final : public TestGraphics {
public:
  const char *extensionName() const override { return XR_KHR_METAL_ENABLE_EXTENSION_NAME; }

  bool init(XrInstance instance, XrSystemId system, PFN_xrGetInstanceProcAddr get_proc) override {
    PFN_xrGetMetalGraphicsRequirementsKHR get_requirements = nullptr;
    if (XR_FAILED(get_proc(instance, "xrGetMetalGraphicsRequirementsKHR",
                           reinterpret_cast<PFN_xrVoidFunction *>(&get_requirements))))
      return false;
    XrGraphicsRequirementsMetalKHR requirements{XR_TYPE_GRAPHICS_REQUIREMENTS_METAL_KHR};
    if (XR_FAILED(get_requirements(instance, system, &requirements)) || !requirements.metalDevice)
      return false;
    device_ = (__bridge id<MTLDevice>)requirements.metalDevice;
    queue_ = [device_ newCommandQueue];
    binding_.commandQueue = (__bridge void *)queue_;
    return queue_ != nil;
  }

  const void *binding() const override { return &binding_; }

  int64_t chooseFormat(const std::vector<int64_t> &formats, bool srgb) const override {
    const int64_t wanted = srgb ? MTLPixelFormatRGBA8Unorm_sRGB : MTLPixelFormatRGBA8Unorm;
    for (int64_t format : formats)
      if (format == wanted)
        return format;
    return 0;
  }

  bool adopt(int chain_id, XrSwapchain swapchain, PFN_xrEnumerateSwapchainImages enumerate, uint32_t width,
             uint32_t height, int64_t format, uint32_t array_size, uint32_t) override {
    uint32_t count = 0;
    if (XR_FAILED(enumerate(swapchain, 0, &count, nullptr)) || !count)
      return false;
    std::vector<XrSwapchainImageMetalKHR> images(count, {XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR});
    if (XR_FAILED(enumerate(swapchain, count, &count, reinterpret_cast<XrSwapchainImageBaseHeader *>(images.data()))))
      return false;
    Chain chain;
    chain.width = width;
    chain.height = height;
    chain.array_size = array_size;
    for (const auto &image : images)
      chain.images.push_back((__bridge id<MTLTexture>)image.texture);
    chains_[chain_id] = chain;
    return ensurePattern(static_cast<MTLPixelFormat>(format));
  }

  void draw(int chain_id, uint32_t image, uint32_t slice, const float color[4], uint32_t frame) override {
    Chain &chain = chains_[chain_id];
    if (!commands_)
      commands_ = [queue_ commandBuffer];
    id<MTLTexture> target = chain.images[image];
    MTLRenderPassDescriptor *pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = target;
    pass.colorAttachments[0].slice = slice;
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    pass.colorAttachments[0].clearColor = MTLClearColorMake(color[0], color[1], color[2], color[3]);
    [[commands_ renderCommandEncoderWithDescriptor:pass] endEncoding];
    if (!pattern_ || chain.width < kPattern * 2 || chain.height < kPattern * 2)
      return;
    // in the upper half, so a vertically flipped image is told from an upright one
    const uint32_t span = chain.width - kPattern;
    const uint32_t x = (frame * 8) % span;
    const uint32_t y = chain.height / 4 - kPattern / 2;
    id<MTLBlitCommandEncoder> blit = [commands_ blitCommandEncoder];
    [blit copyFromTexture:pattern_
              sourceSlice:0
              sourceLevel:0
             sourceOrigin:MTLOriginMake(0, 0, 0)
               sourceSize:MTLSizeMake(kPattern, kPattern, 1)
                toTexture:target
         destinationSlice:slice
         destinationLevel:0
        destinationOrigin:MTLOriginMake(x, y, 0)];
    [blit endEncoding];
  }

  void submit() override {
    if (commands_) {
      [commands_ commit];
      commands_ = nil;
    }
  }

private:
  static constexpr uint32_t kPattern = 256;

  bool ensurePattern(MTLPixelFormat format) {
    if (pattern_)
      return true;
    // the checkerboard is 8 bits a channel; swapchains of other formats are only cleared
    if (format != MTLPixelFormatRGBA8Unorm && format != MTLPixelFormatRGBA8Unorm_sRGB &&
        format != MTLPixelFormatBGRA8Unorm && format != MTLPixelFormatBGRA8Unorm_sRGB)
      return true;
    std::vector<uint32_t> pixels(kPattern * kPattern);
    for (uint32_t y = 0; y < kPattern; ++y)
      for (uint32_t x = 0; x < kPattern; ++x)
        pixels[y * kPattern + x] = ((x / 32 + y / 32) & 1) ? 0xffffffffu : 0xff000000u;
    MTLTextureDescriptor *desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format
                                                                                    width:kPattern
                                                                                   height:kPattern
                                                                                mipmapped:NO];
    pattern_ = [device_ newTextureWithDescriptor:desc];
    [pattern_ replaceRegion:MTLRegionMake2D(0, 0, kPattern, kPattern)
                mipmapLevel:0
                  withBytes:pixels.data()
                bytesPerRow:kPattern * 4];
    return pattern_ != nil;
  }

  id<MTLDevice> device_ = nil;
  id<MTLCommandQueue> queue_ = nil;
  id<MTLCommandBuffer> commands_ = nil;
  id<MTLTexture> pattern_ = nil;
  XrGraphicsBindingMetalKHR binding_{XR_TYPE_GRAPHICS_BINDING_METAL_KHR};
  std::map<int, Chain> chains_;
};

} // namespace

TestGraphics *createTestGraphics() { return new MetalGraphics(); }
