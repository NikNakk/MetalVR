#import "compositor.h"

#import <CoreGraphics/CoreGraphics.h>
#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <cmath>

#include "common.h"
#include "metalvr_math.h"

namespace metalvr {

namespace {

// The target is written through an sRGB view, so every source is brought to linear light first: an sRGB texture
// decodes when sampled, a float texture is linear already, and an 8- or 10-bit UNORM texture is taken to hold
// display-referred values, as applications that pick those formats write them.
NSString *const kShaderSource = @R"(
#include <metal_stdlib>
using namespace metal;

struct Vertex {
  float4 position;
  float2 uv;
  float2 pad;
};

struct Uniforms {
  uint decode_gamma;
  uint unpremultiplied;
  uint opaque;
  uint pad;
};

struct Varyings {
  float4 position [[position]];
  float2 uv;
};

vertex Varyings layer_vertex(uint vid [[vertex_id]], const device Vertex *vertices [[buffer(0)]]) {
  Varyings out;
  out.position = vertices[vid].position;
  out.uv = vertices[vid].uv;
  return out;
}

static float3 srgb_to_linear(float3 c) {
  float3 lo = c / 12.92;
  float3 hi = pow((c + 0.055) / 1.055, float3(2.4));
  return select(hi, lo, c <= float3(0.04045));
}

fragment float4 layer_fragment(Varyings in [[stage_in]], texture2d<float> source [[texture(0)]],
                               sampler smp [[sampler(0)]], constant Uniforms &u [[buffer(0)]]) {
  float4 c = source.sample(smp, in.uv);
  if (u.decode_gamma)
    c.rgb = srgb_to_linear(saturate(c.rgb));
  if (u.opaque)
    c.a = 1.0;
  else if (u.unpremultiplied)
    c.rgb *= c.a;
  return c;
}
)";

struct Vertex {
  float position[4];
  float uv[2];
  float pad[2];
};

struct Uniforms {
  uint32_t decode_gamma;
  uint32_t unpremultiplied;
  uint32_t opaque;
  uint32_t pad;
};

bool decodesWhenSampled(MTLPixelFormat format) {
  switch (format) {
  case MTLPixelFormatRGBA8Unorm_sRGB:
  case MTLPixelFormatBGRA8Unorm_sRGB:
    return true;
  default:
    return false;
  }
}

bool isLinearFormat(MTLPixelFormat format) {
  switch (format) {
  case MTLPixelFormatRGBA16Float:
  case MTLPixelFormatRGBA32Float:
  case MTLPixelFormatRG11B10Float:
    return true;
  default:
    return false;
  }
}

// the clip-space position of a point given in the eye's own space
void project(const mvr_fov &fov, mvr_vec3 p, float out[4]) {
  float tl = std::tan(fov.left), tr = std::tan(fov.right), tu = std::tan(fov.up), td = std::tan(fov.down);
  out[0] = (2.0f * p.x + (tr + tl) * p.z) / (tr - tl);
  out[1] = (2.0f * p.y + (tu + td) * p.z) / (tu - td);
  out[3] = -p.z;
  out[2] = 0.5f * out[3]; // layers are drawn in order, with no depth test
}

} // namespace

Compositor::Compositor(id<MTLDevice> device) : device_(device) {
  queue_ = [device_ newCommandQueue];
  queue_.label = @"metalvr-compositor";
  NSError *error = nil;
  library_ = [device_ newLibraryWithSource:kShaderSource options:nil error:&error];
  if (!library_)
    logInfo("compositor: shader compile failed: %s", error.localizedDescription.UTF8String);
  MTLSamplerDescriptor *sampler = [MTLSamplerDescriptor new];
  sampler.minFilter = MTLSamplerMinMagFilterLinear;
  sampler.magFilter = MTLSamplerMinMagFilterLinear;
  sampler.sAddressMode = MTLSamplerAddressModeClampToEdge;
  sampler.tAddressMode = MTLSamplerAddressModeClampToEdge;
  sampler_ = [device_ newSamplerStateWithDescriptor:sampler];
}

Compositor::~Compositor() {
  if (pool_)
    CVPixelBufferPoolRelease(pool_);
}

id<MTLRenderPipelineState> Compositor::pipeline(bool blend) {
  // the fragment shader premultiplies an unpremultiplied source, so one blend state serves both kinds of alpha
  const int index = blend ? 1 : 0;
  if (pipelines_[index])
    return pipelines_[index];
  if (!library_)
    return nil;
  MTLRenderPipelineDescriptor *desc = [MTLRenderPipelineDescriptor new];
  desc.vertexFunction = [library_ newFunctionWithName:@"layer_vertex"];
  desc.fragmentFunction = [library_ newFunctionWithName:@"layer_fragment"];
  desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
  if (blend) {
    desc.colorAttachments[0].blendingEnabled = YES;
    desc.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorOne;
    desc.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
    desc.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    desc.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
  }
  NSError *error = nil;
  pipelines_[index] = [device_ newRenderPipelineStateWithDescriptor:desc error:&error];
  if (!pipelines_[index])
    logInfo("compositor: pipeline failed: %s", error.localizedDescription.UTF8String);
  return pipelines_[index];
}

bool Compositor::configure(uint32_t frame_width, uint32_t frame_height) {
  if (frame_width == width_ && frame_height == height_ && pool_)
    return true;
  if (pool_) {
    CVPixelBufferPoolRelease(pool_);
    pool_ = nullptr;
  }
  width_ = frame_width;
  height_ = frame_height;
  NSDictionary *attributes = @{
    (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA),
    (id)kCVPixelBufferWidthKey : @(frame_width),
    (id)kCVPixelBufferHeightKey : @(frame_height),
    (id)kCVPixelBufferIOSurfacePropertiesKey : @{},
    (id)kCVPixelBufferMetalCompatibilityKey : @YES,
  };
  CVReturn result =
      CVPixelBufferPoolCreate(kCFAllocatorDefault, nullptr, (__bridge CFDictionaryRef)attributes, &pool_);
  if (result != kCVReturnSuccess) {
    logInfo("compositor: buffer pool %ux%u failed: %d", frame_width, frame_height, result);
    pool_ = nullptr;
    return false;
  }
  return true;
}

CVPixelBufferRef Compositor::compose(const ComposeFrame &frame) {
  if (!pool_ || !library_)
    return nullptr;
  CVPixelBufferRef buffer = nullptr;
  if (CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool_, &buffer) != kCVReturnSuccess || !buffer)
    return nullptr;
  IOSurfaceRef surface = CVPixelBufferGetIOSurface(buffer);
  if (!surface) {
    CVPixelBufferRelease(buffer);
    return nullptr;
  }

  @autoreleasepool {
    MTLTextureDescriptor *target_desc =
        [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                           width:width_
                                                          height:height_
                                                       mipmapped:NO];
    target_desc.usage = MTLTextureUsageRenderTarget;
    target_desc.storageMode = MTLStorageModeShared;
    id<MTLTexture> target = [device_ newTextureWithDescriptor:target_desc iosurface:surface plane:0];
    if (!target) {
      CVPixelBufferRelease(buffer);
      return nullptr;
    }

    MTLRenderPassDescriptor *pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = target;
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    pass.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);

    id<MTLCommandBuffer> commands = [queue_ commandBuffer];
    id<MTLRenderCommandEncoder> encoder = [commands renderCommandEncoderWithDescriptor:pass];
    [encoder setFragmentSamplerState:sampler_ atIndex:0];

    const uint32_t eye_width = width_ / 2;
    bool first = true;
    for (const ComposeLayer &layer : frame.layers) {
      const bool alpha = (layer.flags & MVR_LAYER_BLEND_ALPHA) != 0;
      // the bottom layer covers the eye whatever its alpha holds; a quad never does
      const bool blend = layer.type == MVR_LAYER_QUAD ? alpha : (alpha && !first);
      id<MTLRenderPipelineState> state = pipeline(blend);
      if (!state)
        continue;
      [encoder setRenderPipelineState:state];

      for (uint32_t eye = 0; eye < 2; ++eye) {
        const ComposeLayerView &view = layer.type == MVR_LAYER_QUAD ? layer.views[0] : layer.views[eye];
        if (!view.texture)
          continue;
        if (layer.type == MVR_LAYER_QUAD && !(layer.eye_mask & (1u << eye)))
          continue;

        const float tw = static_cast<float>(view.texture.width), th = static_cast<float>(view.texture.height);
        const float u0 = view.rect.x / tw, u1 = (view.rect.x + view.rect.width) / tw;
        const float v0 = view.rect.y / th, v1 = (view.rect.y + view.rect.height) / th;

        Vertex vertices[4];
        if (layer.type == MVR_LAYER_QUAD) {
          // the quad's corners in the eye's space: bottom-left, bottom-right, top-left, top-right
          mvr_pose quad_in_tracking =
              (layer.flags & MVR_LAYER_HEAD_LOCKED) ? poseMul(frame.head_pose, layer.quad_pose) : layer.quad_pose;
          mvr_pose quad_in_eye = poseMul(poseInverse(frame.eye_pose[eye]), quad_in_tracking);
          const float hw = layer.quad_width * 0.5f, hh = layer.quad_height * 0.5f;
          const mvr_vec3 corners[4] = {{-hw, -hh, 0}, {hw, -hh, 0}, {-hw, hh, 0}, {hw, hh, 0}};
          const float uvs[4][2] = {{u0, v1}, {u1, v1}, {u0, v0}, {u1, v0}};
          for (int i = 0; i < 4; ++i) {
            mvr_vec3 p = quad_in_eye.position + quatRotate(quad_in_eye.orientation, corners[i]);
            project(frame.eye_fov[eye], p, vertices[i].position);
            vertices[i].uv[0] = uvs[i][0];
            vertices[i].uv[1] = uvs[i][1];
          }
        } else {
          const float positions[4][2] = {{-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
          const float uvs[4][2] = {{u0, v1}, {u1, v1}, {u0, v0}, {u1, v0}};
          for (int i = 0; i < 4; ++i) {
            vertices[i].position[0] = positions[i][0];
            vertices[i].position[1] = positions[i][1];
            vertices[i].position[2] = 0.5f;
            vertices[i].position[3] = 1.0f;
            vertices[i].uv[0] = uvs[i][0];
            vertices[i].uv[1] = uvs[i][1];
          }
        }

        Uniforms uniforms{};
        MTLPixelFormat format = view.texture.pixelFormat;
        uniforms.decode_gamma = !decodesWhenSampled(format) && !isLinearFormat(format);
        uniforms.unpremultiplied = (layer.flags & MVR_LAYER_UNPREMULTIPLIED_ALPHA) != 0;
        uniforms.opaque = !blend;

        MTLViewport viewport = {static_cast<double>(eye * eye_width), 0, static_cast<double>(eye_width),
                                static_cast<double>(height_), 0, 1};
        [encoder setViewport:viewport];
        MTLScissorRect scissor = {eye * eye_width, 0, eye_width, height_};
        [encoder setScissorRect:scissor];
        [encoder setVertexBytes:vertices length:sizeof(vertices) atIndex:0];
        [encoder setFragmentBytes:&uniforms length:sizeof(uniforms) atIndex:0];
        [encoder setFragmentTexture:view.texture atIndex:0];
        [encoder drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
      }
      first = false;
    }

    [encoder endEncoding];
    [commands commit];
    [commands waitUntilCompleted];
    if (commands.status != MTLCommandBufferStatusCompleted) {
      logInfo("compositor: command buffer failed: %s", commands.error.localizedDescription.UTF8String);
      CVPixelBufferRelease(buffer);
      return nullptr;
    }
  }
  return buffer;
}

bool writePixelBufferPNG(CVPixelBufferRef buffer, const char *path) {
  if (CVPixelBufferLockBaseAddress(buffer, kCVPixelBufferLock_ReadOnly) != kCVReturnSuccess)
    return false;
  bool ok = false;
  const size_t width = CVPixelBufferGetWidth(buffer), height = CVPixelBufferGetHeight(buffer);
  CGColorSpaceRef space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
  CGContextRef context = CGBitmapContextCreate(
      CVPixelBufferGetBaseAddress(buffer), width, height, 8, CVPixelBufferGetBytesPerRow(buffer), space,
      static_cast<uint32_t>(kCGImageAlphaNoneSkipFirst) | static_cast<uint32_t>(kCGBitmapByteOrder32Little));
  if (context) {
    CGImageRef image = CGBitmapContextCreateImage(context);
    if (image) {
      NSURL *url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:path]];
      CGImageDestinationRef destination = CGImageDestinationCreateWithURL(
          (__bridge CFURLRef)url, (__bridge CFStringRef)UTTypePNG.identifier, 1, nullptr);
      if (destination) {
        CGImageDestinationAddImage(destination, image, nullptr);
        ok = CGImageDestinationFinalize(destination);
        CFRelease(destination);
      }
      CGImageRelease(image);
    }
    CGContextRelease(context);
  }
  CGColorSpaceRelease(space);
  CVPixelBufferUnlockBaseAddress(buffer, kCVPixelBufferLock_ReadOnly);
  return ok;
}

} // namespace metalvr

bool metalvr::Compositor::writeTexturePNG(id<MTLTexture> texture, const char *path, double *mean_color,
                                          double *mean_alpha) {
  if (!texture || texture.textureType != MTLTextureType2D)
    return false;
  const NSUInteger width = texture.width, height = texture.height, row = width * 4;
  id<MTLBuffer> bytes = [device_ newBufferWithLength:row * height options:MTLResourceStorageModeShared];
  id<MTLCommandBuffer> commands = [queue_ commandBuffer];
  id<MTLBlitCommandEncoder> blit = [commands blitCommandEncoder];
  [blit copyFromTexture:texture sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0)
             sourceSize:MTLSizeMake(width, height, 1) toBuffer:bytes destinationOffset:0
    destinationBytesPerRow:row destinationBytesPerImage:row * height];
  [blit endEncoding];
  [commands commit];
  [commands waitUntilCompleted];
  if (commands.error)
    return false;
  if (mean_color && mean_alpha) {
    const uint8_t *px = static_cast<const uint8_t *>(bytes.contents);
    double color = 0, alpha = 0;
    for (NSUInteger i = 0; i < row * height; i += 4) {
      color += px[i] + px[i + 1] + px[i + 2];
      alpha += px[i + 3];
    }
    *mean_color = color / (width * height * 3.0);
    *mean_alpha = alpha / (width * height);
  }
  const MTLPixelFormat f = texture.pixelFormat;
  const bool bgra = f == MTLPixelFormatBGRA8Unorm || f == MTLPixelFormatBGRA8Unorm_sRGB;
  CGColorSpaceRef space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
  const uint32_t info = bgra ? (static_cast<uint32_t>(kCGImageAlphaNoneSkipFirst) | static_cast<uint32_t>(kCGBitmapByteOrder32Little))
                             : static_cast<uint32_t>(kCGImageAlphaNoneSkipLast);
  CGContextRef context = CGBitmapContextCreate(bytes.contents, width, height, 8, row, space, info);
  bool ok = false;
  if (context) {
    CGImageRef image = CGBitmapContextCreateImage(context);
    if (image) {
      NSURL *url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:path]];
      CGImageDestinationRef destination = CGImageDestinationCreateWithURL(
          (__bridge CFURLRef)url, (__bridge CFStringRef)UTTypePNG.identifier, 1, nullptr);
      if (destination) {
        CGImageDestinationAddImage(destination, image, nullptr);
        ok = CGImageDestinationFinalize(destination);
        CFRelease(destination);
      }
      CGImageRelease(image);
    }
    CGContextRelease(context);
  }
  CGColorSpaceRelease(space);
  return ok;
}
