// The Metal backend, for native macOS applications.
//
// A swapchain image is a Metal texture over a global IOSurface the server opens by its id, so an application's
// rendering reaches the server with no copy. An array swapchain cannot be an IOSurface texture: its images are
// ordinary array textures, and each slice is blitted into its own surface when the image is released.
#include "api.h"

#import <IOSurface/IOSurface.h>
#import <Metal/Metal.h>

namespace metalvr {

namespace {

struct FormatInfo {
  MTLPixelFormat format;
  uint32_t share;       // mvr_format; MVR_FORMAT_UNKNOWN for depth
  uint32_t surface;     // the IOSurface pixel format
  uint32_t bytes_per_pixel;
};

const FormatInfo kFormats[] = {
    {MTLPixelFormatBGRA8Unorm_sRGB, MVR_FORMAT_BGRA8_SRGB, 'BGRA', 4},
    {MTLPixelFormatRGBA8Unorm_sRGB, MVR_FORMAT_RGBA8_SRGB, 'RGBA', 4},
    {MTLPixelFormatBGRA8Unorm, MVR_FORMAT_BGRA8_UNORM, 'BGRA', 4},
    {MTLPixelFormatRGBA8Unorm, MVR_FORMAT_RGBA8_UNORM, 'RGBA', 4},
    {MTLPixelFormatRGBA16Float, MVR_FORMAT_RGBA16_FLOAT, 'RGhA', 8},
    {MTLPixelFormatDepth32Float, MVR_FORMAT_UNKNOWN, 0, 0},
    {MTLPixelFormatDepth32Float_Stencil8, MVR_FORMAT_UNKNOWN, 0, 0},
    {MTLPixelFormatDepth16Unorm, MVR_FORMAT_UNKNOWN, 0, 0},
};

const FormatInfo *formatInfo(int64_t format) {
  for (const FormatInfo &info : kFormats)
    if (info.format == static_cast<MTLPixelFormat>(format))
      return &info;
  return nullptr;
}

struct MetalImages final : SwapchainImages {
  id<MTLCommandQueue> queue = nil;
  NSMutableArray<id<MTLTexture>> *app = [NSMutableArray array];    // what the application renders to
  NSMutableArray<id<MTLTexture>> *shares = [NSMutableArray array]; // empty when `app` are the surfaces themselves
  uint32_t array_size = 1;

  XrResult enumerate(uint32_t capacity, uint32_t *count_out, XrSwapchainImageBaseHeader *images) override {
    if (!count_out)
      return XR_ERROR_VALIDATION_FAILURE;
    *count_out = count;
    if (capacity == 0)
      return XR_SUCCESS;
    if (capacity < count)
      return XR_ERROR_SIZE_INSUFFICIENT;
    if (!images)
      return XR_ERROR_VALIDATION_FAILURE;
    auto *out = reinterpret_cast<XrSwapchainImageMetalKHR *>(images);
    for (uint32_t i = 0; i < count; ++i) {
      if (out[i].type != XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR)
        return XR_ERROR_VALIDATION_FAILURE;
      out[i].texture = (__bridge void *)app[i];
    }
    return XR_SUCCESS;
  }

  void released(uint32_t index) override {
    if (shares.count == 0)
      return;
    @autoreleasepool {
      id<MTLCommandBuffer> commands = [queue commandBuffer];
      id<MTLBlitCommandEncoder> blit = [commands blitCommandEncoder];
      id<MTLTexture> source = app[index];
      for (uint32_t slice = 0; slice < array_size; ++slice) {
        id<MTLTexture> share = shares[index * array_size + slice];
        [blit copyFromTexture:source
                  sourceSlice:slice
                  sourceLevel:0
                 sourceOrigin:MTLOriginMake(0, 0, 0)
                   sourceSize:MTLSizeMake(source.width, source.height, 1)
                    toTexture:share
             destinationSlice:0
             destinationLevel:0
            destinationOrigin:MTLOriginMake(0, 0, 0)];
      }
      [blit endEncoding];
      [commands commit];
    }
  }
};

class MetalBackend final : public GraphicsBackend {
public:
  explicit MetalBackend(id<MTLCommandQueue> queue) : queue_(queue), device_(queue.device) {
    for (const FormatInfo &info : kFormats)
      formats_.push_back(static_cast<int64_t>(info.format));
  }

  uint32_t api() const override { return MVR_GFX_METAL; }
  const std::vector<int64_t> &formats() const override { return formats_; }
  uint32_t maxSampleCount() const override { return kMaxSampleCount; }

  XrResult createSwapchain(const XrSwapchainCreateInfo &info, std::unique_ptr<SwapchainImages> &out,
                           mvr_swapchain_create_request &share, bool &composited) override {
    const FormatInfo *format = formatInfo(info.format);
    if (!format)
      return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
    const bool depth = format->share == MVR_FORMAT_UNKNOWN;
    const uint32_t image_count = (info.createFlags & XR_SWAPCHAIN_CREATE_STATIC_IMAGE_BIT) ? 1 : 3;
    if (!depth && image_count * info.arraySize > MVR_MAX_SWAPCHAIN_IMAGES)
      return XR_ERROR_FEATURE_UNSUPPORTED;

    auto images = std::make_unique<MetalImages>();
    images->queue = queue_;
    images->count = image_count;
    images->array_size = info.arraySize;
    composited = !depth;

    MTLTextureUsage usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    if (info.usageFlags & XR_SWAPCHAIN_USAGE_MUTABLE_FORMAT_BIT)
      usage |= MTLTextureUsagePixelFormatView;
    if (info.usageFlags & XR_SWAPCHAIN_USAGE_UNORDERED_ACCESS_BIT)
      usage |= MTLTextureUsageShaderWrite;

    // a single-slice, single-mip color image is the IOSurface itself
    const bool direct = !depth && info.arraySize == 1 && info.mipCount == 1;

    if (!direct) {
      MTLTextureDescriptor *desc = [MTLTextureDescriptor new];
      desc.textureType = info.arraySize > 1 ? MTLTextureType2DArray : MTLTextureType2D;
      desc.pixelFormat = format->format;
      desc.width = info.width;
      desc.height = info.height;
      desc.arrayLength = info.arraySize;
      desc.mipmapLevelCount = info.mipCount;
      desc.usage = usage;
      desc.storageMode = MTLStorageModePrivate;
      for (uint32_t i = 0; i < image_count; ++i) {
        id<MTLTexture> texture = [device_ newTextureWithDescriptor:desc];
        if (!texture)
          return XR_ERROR_RUNTIME_FAILURE;
        [images->app addObject:texture];
      }
    }

    if (composited) {
      share.width = info.width;
      share.height = info.height;
      share.format = format->share;
      share.array_size = info.arraySize;
      share.share_kind = MVR_SHARE_IOSURFACE_ID;
      share.image_count = image_count;
      for (uint32_t i = 0; i < image_count * info.arraySize; ++i) {
        // the server opens the surface by id, which only a global surface allows
        NSDictionary *properties = @{
          (id)kIOSurfaceWidth : @(info.width),
          (id)kIOSurfaceHeight : @(info.height),
          (id)kIOSurfaceBytesPerElement : @(format->bytes_per_pixel),
          (id)kIOSurfacePixelFormat : @(format->surface),
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
          (id)kIOSurfaceIsGlobal : @YES,
#pragma clang diagnostic pop
        };
        IOSurfaceRef surface = IOSurfaceCreate((__bridge CFDictionaryRef)properties);
        if (!surface)
          return XR_ERROR_RUNTIME_FAILURE;
        MTLTextureDescriptor *desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format->format
                                                                                        width:info.width
                                                                                       height:info.height
                                                                                    mipmapped:NO];
        desc.usage = usage;
        desc.storageMode = MTLStorageModeShared;
        id<MTLTexture> texture = [device_ newTextureWithDescriptor:desc iosurface:surface plane:0];
        share.images[i].iosurface_id = IOSurfaceGetID(surface);
        CFRelease(surface); // the texture keeps it
        if (!texture)
          return XR_ERROR_RUNTIME_FAILURE;
        [(direct ? images->app : images->shares) addObject:texture];
      }
    }
    out = std::move(images);
    return XR_SUCCESS;
  }

  void finishFrame() override {
    // command buffers of one queue run in the order they were committed, so when this empty one is done, so is
    // everything the application committed to the queue before ending the frame
    @autoreleasepool {
      id<MTLCommandBuffer> fence = [queue_ commandBuffer];
      [fence commit];
      [fence waitUntilCompleted];
    }
  }

private:
  id<MTLCommandQueue> queue_;
  id<MTLDevice> device_;
  std::vector<int64_t> formats_;
};

} // namespace

uint32_t platformGraphicsApi() { return MVR_GFX_METAL; }

XrResult createGraphicsBackend(const XrSessionCreateInfo *info, std::unique_ptr<GraphicsBackend> &out) {
  const XrGraphicsBindingMetalKHR *binding = nullptr;
  for (auto *base = static_cast<const XrBaseInStructure *>(info->next); base; base = base->next)
    if (base->type == XR_TYPE_GRAPHICS_BINDING_METAL_KHR)
      binding = reinterpret_cast<const XrGraphicsBindingMetalKHR *>(base);
  if (!binding || !binding->commandQueue)
    return XR_ERROR_GRAPHICS_DEVICE_INVALID;
  id<MTLCommandQueue> queue = (__bridge id<MTLCommandQueue>)binding->commandQueue;
  if (![queue conformsToProtocol:@protocol(MTLCommandQueue)])
    return XR_ERROR_GRAPHICS_DEVICE_INVALID;
  out = std::make_unique<MetalBackend>(queue);
  return XR_SUCCESS;
}

namespace api {

MVR_API xrGetMetalGraphicsRequirementsKHR(XrInstance handle, XrSystemId system_id,
                                          XrGraphicsRequirementsMetalKHR *requirements) {
  MVR_INSTANCE(handle);
  if (!requirements || requirements->type != XR_TYPE_GRAPHICS_REQUIREMENTS_METAL_KHR)
    return XR_ERROR_VALIDATION_FAILURE;
  if (system_id != kSystemId)
    return XR_ERROR_SYSTEM_INVALID;
  // the server composes on the system's default device; a shared surface is only zero-copy on the same one
  static id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  requirements->metalDevice = (__bridge void *)device;
  instance->graphics_requirements_queried = true;
  return XR_SUCCESS;
}

} // namespace api
} // namespace metalvr
