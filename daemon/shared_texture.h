// Opens a swapchain image a client shared, as a Metal texture in this process.
#pragma once

#import <Metal/Metal.h>

#include <cstdint>

namespace metalvr {

// A Direct3D 11 shared texture made by Mullion. `name` is the bootstrap name Mullion registered for the texture's
// Metal shared-texture port. Nil if the name is gone or the port is not a texture's.
id<MTLTexture> openMullionSharedTexture(id<MTLDevice> device, const char *name);

// A global IOSurface, as a texture of `format`.
id<MTLTexture> openIOSurfaceTexture(id<MTLDevice> device, uint32_t iosurface_id, MTLPixelFormat format);

} // namespace metalvr
