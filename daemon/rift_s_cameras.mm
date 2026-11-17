#import "rift_s_cameras.h"

#import <AVFoundation/AVFoundation.h>

#include <vector>

#include "common.h"
#include "rift_s_protocol.h"

@interface MetalVRRiftSCameraSink : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate> {
@public
  metalvr::RiftSFrameHandler handler;
  std::vector<uint8_t> grey;
}
@end

@implementation MetalVRRiftSCameraSink
- (void)captureOutput:(AVCaptureOutput *)output
    didOutputSampleBuffer:(CMSampleBufferRef)sample
           fromConnection:(AVCaptureConnection *)connection {
  CVPixelBufferRef buffer = CMSampleBufferGetImageBuffer(sample);
  if (!buffer || CVPixelBufferLockBaseAddress(buffer, kCVPixelBufferLock_ReadOnly) != kCVReturnSuccess)
    return;
  const OSType format = CVPixelBufferGetPixelFormatType(buffer);
  const uint32_t width = static_cast<uint32_t>(CVPixelBufferGetWidth(buffer));
  const uint32_t height = static_cast<uint32_t>(CVPixelBufferGetHeight(buffer));
  const bool planar = CVPixelBufferIsPlanar(buffer);
  const uint8_t *base = static_cast<const uint8_t *>(planar ? CVPixelBufferGetBaseAddressOfPlane(buffer, 0)
                                                            : CVPixelBufferGetBaseAddress(buffer));
  const size_t stride = planar ? CVPixelBufferGetBytesPerRowOfPlane(buffer, 0) : CVPixelBufferGetBytesPerRow(buffer);
  if (base && width * 2 == metalvr::rift_s::kCameraFrameWidth &&
      (format == kCVPixelFormatType_422YpCbCr8_yuvs || format == kCVPixelFormatType_422YpCbCr8)) {
    // the device's own bytes: grey at twice the width it states
    handler(base, width * 2, height, stride);
  } else if (base && width == metalvr::rift_s::kCameraFrameWidth) {
    // full-width pictures: their brightness, from the luma plane or every other byte of packed 4:2:2
    if (planar) {
      handler(base, width, height, stride);
    } else if (format == kCVPixelFormatType_422YpCbCr8_yuvs || format == kCVPixelFormatType_422YpCbCr8) {
      const int luma = format == kCVPixelFormatType_422YpCbCr8_yuvs ? 0 : 1;
      grey.resize(static_cast<size_t>(width) * height);
      for (uint32_t y = 0; y < height; y++)
        for (uint32_t x = 0; x < width; x++) grey[y * width + x] = base[y * stride + 2 * x + luma];
      handler(grey.data(), width, height, width);
    }
  }
  CVPixelBufferUnlockBaseAddress(buffer, kCVPixelBufferLock_ReadOnly);
}
@end

namespace metalvr {
namespace {

class Capture final : public RiftSCameras {
public:
  bool start(AVCaptureDevice *device, RiftSFrameHandler handler) {
    NSError *error = nil;
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
    if (!input) {
      logInfo("rift: cannot open the Rift S's cameras: %s", error.localizedDescription.UTF8String);
      return false;
    }
    session_ = [AVCaptureSession new];
    if (![session_ canAddInput:input])
      return false;
    [session_ addInput:input];
    // the widest format the device offers, at its highest rate (the 60 Hz feed of interleaved tracking and
    // controller frames)
    AVCaptureDeviceFormat *best = nil;
    for (AVCaptureDeviceFormat *format in device.formats) {
      const CMVideoDimensions size = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
      const CMVideoDimensions best_size =
          best ? CMVideoFormatDescriptionGetDimensions(best.formatDescription) : CMVideoDimensions{0, 0};
      if (size.width > best_size.width || (size.width == best_size.width && size.height > best_size.height))
        best = format;
    }
    if (best && [device lockForConfiguration:&error]) {
      device.activeFormat = best;
      [device unlockForConfiguration];
    }
    AVCaptureVideoDataOutput *output = [AVCaptureVideoDataOutput new];
    output.videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_422YpCbCr8_yuvs)};
    output.alwaysDiscardsLateVideoFrames = NO; // every tracking frame matters to the tracker
    sink_ = [MetalVRRiftSCameraSink new];
    sink_->handler = std::move(handler);
    queue_ = dispatch_queue_create("metalvr.rift-s-cameras", DISPATCH_QUEUE_SERIAL);
    [output setSampleBufferDelegate:sink_ queue:queue_];
    if (![session_ canAddOutput:output])
      return false;
    [session_ addOutput:output];
    [session_ startRunning];
    const CMVideoDimensions size =
        best ? CMVideoFormatDescriptionGetDimensions(best.formatDescription) : CMVideoDimensions{0, 0};
    logInfo("rift: reading the Rift S's cameras (%s, %dx%d)", device.localizedName.UTF8String, size.width,
            size.height);
    return true;
  }

  ~Capture() override {
    [session_ stopRunning];
    if (queue_)
      dispatch_sync(queue_, ^{
                    }); // the last frame handled before the handler goes
  }

private:
  AVCaptureSession *session_ = nil;
  MetalVRRiftSCameraSink *sink_ = nil;
  dispatch_queue_t queue_ = nullptr;
};

} // namespace

std::unique_ptr<RiftSCameras> startRiftSCameras(RiftSFrameHandler handler) {
  if ([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo] == AVAuthorizationStatusDenied) {
    logInfo("rift: macOS does not let metalvrd use cameras (System Settings, Privacy & Security, Camera); the "
            "Rift S tracks rotation only");
    return nullptr;
  }
  AVCaptureDeviceDiscoverySession *discovery =
      [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:@[ AVCaptureDeviceTypeExternal ]
                                                             mediaType:AVMediaTypeVideo
                                                              position:AVCaptureDevicePositionUnspecified];
  for (AVCaptureDevice *device in discovery.devices)
    if ([device.localizedName rangeOfString:@"Rift S Sensor" options:NSCaseInsensitiveSearch].location != NSNotFound) {
      auto capture = std::make_unique<Capture>();
      if (capture->start(device, std::move(handler)))
        return capture;
      return nullptr;
    }
  return nullptr;
}

} // namespace metalvr
