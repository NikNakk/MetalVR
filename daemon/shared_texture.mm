#import "shared_texture.h"

#import <IOSurface/IOSurface.h>

#include <bootstrap.h>
#include <mach/mach.h>

#include "common.h"

// Metal can rebuild a shared texture from the mach port of its handle. The selector is not in the public headers;
// Mullion uses the same one to move D3D11 shared textures between processes (winemetal_unix.c).
@interface MTLSharedTextureHandle ()
- (instancetype)initWithMachPort:(mach_port_t)port;
@end

namespace metalvr {

namespace {

// Mullion's shared-name protocol (winemetal_unix.c, `shared_name`): the bootstrap name is a port of the process
// that made the texture; a bare request to it is answered with a send right to the texture's own port.
enum : mach_msg_id_t { kSharedNameRequest = 0x4d4c534e, kSharedNameReply = kSharedNameRequest + 100 };

struct SharedNameReply {
  mach_msg_header_t header;
  mach_msg_body_t body;
  mach_msg_port_descriptor_t port;
};

mach_port_t lookUpSharedPort(const char *name) {
  mach_port_t bootstrap = MACH_PORT_NULL;
  if (task_get_bootstrap_port(mach_task_self(), &bootstrap) != KERN_SUCCESS)
    return MACH_PORT_NULL;
  mach_port_t name_port = MACH_PORT_NULL;
  kern_return_t kr = bootstrap_look_up(bootstrap, name, &name_port);
  mach_port_deallocate(mach_task_self(), bootstrap);
  if (kr != KERN_SUCCESS)
    return MACH_PORT_NULL;

  mach_port_t reply_port = MACH_PORT_NULL;
  if (mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &reply_port) != KERN_SUCCESS) {
    mach_port_deallocate(mach_task_self(), name_port);
    return MACH_PORT_NULL;
  }

  union {
    mach_msg_header_t request;
    SharedNameReply reply;
    char buffer[sizeof(SharedNameReply) + MAX_TRAILER_SIZE];
  } msg = {};
  msg.request.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, MACH_MSG_TYPE_MAKE_SEND_ONCE);
  msg.request.msgh_size = sizeof(msg.request);
  msg.request.msgh_remote_port = name_port;
  msg.request.msgh_local_port = reply_port;
  msg.request.msgh_id = kSharedNameRequest;

  mach_port_t result = MACH_PORT_NULL;
  kr = mach_msg(&msg.request, MACH_SEND_MSG | MACH_RCV_MSG | MACH_SEND_TIMEOUT | MACH_RCV_TIMEOUT,
                sizeof(msg.request), sizeof(msg), reply_port, 2000, MACH_PORT_NULL);
  if (kr == MACH_SEND_TIMED_OUT || kr == MACH_SEND_INTERRUPTED)
    mach_msg_destroy(&msg.request);
  if (kr == KERN_SUCCESS) {
    if (msg.reply.header.msgh_id == kSharedNameReply && (msg.reply.header.msgh_bits & MACH_MSGH_BITS_COMPLEX) &&
        msg.reply.body.msgh_descriptor_count == 1 && msg.reply.port.type == MACH_MSG_PORT_DESCRIPTOR &&
        msg.reply.port.disposition == MACH_MSG_TYPE_PORT_SEND)
      result = msg.reply.port.name;
    else
      mach_msg_destroy(&msg.reply.header);
  }
  mach_port_mod_refs(mach_task_self(), reply_port, MACH_PORT_RIGHT_RECEIVE, -1);
  mach_port_deallocate(mach_task_self(), name_port);
  return result;
}

} // namespace

id<MTLTexture> openMullionSharedTexture(id<MTLDevice> device, const char *name) {
  if (![MTLSharedTextureHandle instancesRespondToSelector:@selector(initWithMachPort:)]) {
    logInfo("this macOS has no -[MTLSharedTextureHandle initWithMachPort:]; D3D11 swapchains cannot be opened");
    return nil;
  }
  mach_port_t port = lookUpSharedPort(name);
  if (port == MACH_PORT_NULL) {
    logInfo("shared texture '%s': the name does not answer", name);
    return nil;
  }
  MTLSharedTextureHandle *handle = [[MTLSharedTextureHandle alloc] initWithMachPort:port];
  id<MTLTexture> texture = handle ? [device newSharedTextureWithHandle:handle] : nil;
  // the handle holds the surface; this process's own send right is no longer needed
  mach_port_deallocate(mach_task_self(), port);
  if (!texture)
    logInfo("shared texture '%s': Metal did not make a texture from the port", name);
  return texture;
}

id<MTLTexture> openIOSurfaceTexture(id<MTLDevice> device, uint32_t iosurface_id, MTLPixelFormat format) {
  IOSurfaceRef surface = IOSurfaceLookup(iosurface_id);
  if (!surface) {
    logInfo("IOSurface %u: not found", iosurface_id);
    return nil;
  }
  MTLTextureDescriptor *desc =
      [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format
                                                         width:IOSurfaceGetWidth(surface)
                                                        height:IOSurfaceGetHeight(surface)
                                                     mipmapped:NO];
  desc.usage = MTLTextureUsageShaderRead;
  desc.storageMode = MTLStorageModeShared;
  id<MTLTexture> texture = [device newTextureWithDescriptor:desc iosurface:surface plane:0];
  CFRelease(surface);
  if (!texture)
    logInfo("IOSurface %u: Metal did not make a texture of it", iosurface_id);
  return texture;
}

} // namespace metalvr
