#import "camera_passthrough.h"

#import <AVFoundation/AVFoundation.h>

#include <mutex>
#include <string>
#include <vector>

#include "common.h"
#include "metalvr_protocol.h"

@interface MetalVRCameraSink : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate> {
@public
  std::mutex *mutex;
  CVPixelBufferRef *latest;
}
@end

@implementation MetalVRCameraSink
- (void)captureOutput:(AVCaptureOutput *)output
    didOutputSampleBuffer:(CMSampleBufferRef)sample
           fromConnection:(AVCaptureConnection *)connection {
  CVPixelBufferRef buffer = CMSampleBufferGetImageBuffer(sample);
  if (!buffer)
    return;
  CVPixelBufferRetain(buffer);
  std::lock_guard<std::mutex> lock(*mutex);
  if (*latest)
    CVPixelBufferRelease(*latest);
  *latest = buffer;
}
@end

namespace metalvr {
namespace {

class Capture final : public CameraPassthrough {
public:
  bool start(AVCaptureDevice *device, bool stereo) {
    stereo_ = stereo;
    NSError *error = nil;
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
    if (!input) {
      logInfo("passthrough: cannot open %s: %s", device.localizedName.UTF8String,
              error.localizedDescription.UTF8String);
      return false;
    }
    session_ = [AVCaptureSession new];
    if (![session_ canAddInput:input])
      return false;
    [session_ addInput:input];
    AVCaptureVideoDataOutput *output = [AVCaptureVideoDataOutput new];
    output.videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA),
                             (id)kCVPixelBufferMetalCompatibilityKey : @YES};
    output.alwaysDiscardsLateVideoFrames = YES;
    sink_ = [MetalVRCameraSink new];
    sink_->mutex = &mutex_;
    sink_->latest = &latest_;
    queue_ = dispatch_queue_create("metalvr.passthrough", DISPATCH_QUEUE_SERIAL);
    [output setSampleBufferDelegate:sink_ queue:queue_];
    if (![session_ canAddOutput:output])
      return false;
    [session_ addOutput:output];
    [session_ startRunning];
    logInfo("passthrough: %s (%s)", device.localizedName.UTF8String, stereo ? "both eyes side by side" : "one picture");
    return true;
  }

  ~Capture() override {
    [session_ stopRunning];
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_)
      CVPixelBufferRelease(latest_);
    latest_ = nullptr;
  }

  CVPixelBufferRef latest() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_)
      CVPixelBufferRetain(latest_);
    return latest_;
  }

  bool stereo() const override { return stereo_; }

private:
  AVCaptureSession *session_ = nil;
  MetalVRCameraSink *sink_ = nil;
  dispatch_queue_t queue_ = nullptr;
  std::mutex mutex_;
  CVPixelBufferRef latest_ = nullptr;
  bool stereo_ = false;
};

} // namespace

std::unique_ptr<CameraPassthrough> startCameraPassthrough(uint32_t headset) {
  // how each headset's camera names itself to macOS (an Index's is Etron's "3D Camera"; guessed for the Vives)
  std::vector<std::string> names;
  bool stereo = false;
  if (headset == MVR_HEADSET_VALVE_INDEX)
    names = {"3D Camera", "Etron", "Index"}, stereo = true;
  else if (headset == MVR_HEADSET_VIVE_PRO || headset == MVR_HEADSET_VIVE_PRO_2)
    names = {"VIVE Pro", "Vive Pro", "HTC"}, stereo = true;
  else if (headset == MVR_HEADSET_VIVE)
    names = {"VIVE", "Vive", "HTC"};
  else
    return nullptr;
  if ([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo] == AVAuthorizationStatusDenied) {
    logInfo("passthrough: macOS does not let metalvrd use cameras (System Settings, Privacy & Security, Camera)");
    return nullptr;
  }
  NSArray *types = @[ AVCaptureDeviceTypeExternal ];
  AVCaptureDeviceDiscoverySession *discovery =
      [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:types
                                                             mediaType:AVMediaTypeVideo
                                                              position:AVCaptureDevicePositionUnspecified];
  for (AVCaptureDevice *device in discovery.devices)
    for (const std::string &name : names)
      if ([device.localizedName rangeOfString:@(name.c_str()) options:NSCaseInsensitiveSearch].location != NSNotFound) {
        auto capture = std::make_unique<Capture>();
        if (capture->start(device, stereo))
          return capture;
        return nullptr;
      }
  logInfo("passthrough: no camera of this headset among macOS's cameras");
  return nullptr;
}

} // namespace metalvr
