#include "window_capture.h"

#import <Foundation/Foundation.h>
#import <ScreenCaptureKit/ScreenCaptureKit.h>
#import <CoreGraphics/CoreGraphics.h>
#include <libproc.h>

#include <atomic>
#include <mutex>
#include <string>

#include "common.h"

namespace metalvr {
class ScreenWindowCapture;
}

@interface MVRWindowStreamOutput : NSObject <SCStreamOutput, SCStreamDelegate>
@property(nonatomic, assign) metalvr::ScreenWindowCapture *owner;
@end

namespace metalvr {

namespace {

// A process of Wine's: the loader that runs Windows programs is a Mach-O whose path names Wine
bool isWineProcess(pid_t pid) {
  char path[PROC_PIDPATHINFO_MAXSIZE] = {};
  if (proc_pidpath(pid, path, sizeof(path)) <= 0)
    return false;
  std::string p(path);
  for (char &c : p)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return p.find("wine") != std::string::npos;
}

bool containsSteam(NSString *text) {
  return text && [text rangeOfString:@"steam" options:NSCaseInsensitiveSearch].location != NSNotFound;
}

} // namespace

class ScreenWindowCapture final : public WindowCapture {
public:
  ScreenWindowCapture() {
    output_ = [MVRWindowStreamOutput new];
    output_.owner = this;
    queue_ = dispatch_queue_create("metalvr.window-capture", DISPATCH_QUEUE_SERIAL);
    timer_ = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, queue_);
    dispatch_source_set_timer(timer_, dispatch_time(DISPATCH_TIME_NOW, 0), 2 * NSEC_PER_SEC, NSEC_PER_SEC / 4);
    dispatch_source_set_event_handler(timer_, ^{ look(); });
    dispatch_resume(timer_);
  }

  ~ScreenWindowCapture() override {
    dispatch_source_cancel(timer_);
    dispatch_sync(queue_, ^{
      output_.owner = nullptr;
      stopStream();
    });
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_)
      CVPixelBufferRelease(latest_);
  }

  CVPixelBufferRef latest() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_)
      CVPixelBufferRetain(latest_);
    return latest_;
  }

  void received(CMSampleBufferRef sample) {
    CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sample, false);
    if (attachments && CFArrayGetCount(attachments) > 0) {
      NSDictionary *info = (__bridge NSDictionary *)CFArrayGetValueAtIndex(attachments, 0);
      NSNumber *status = info[SCStreamFrameInfoStatus];
      if (status && status.integerValue != SCFrameStatusComplete)
        return;
    }
    CVPixelBufferRef buffer = CMSampleBufferGetImageBuffer(sample);
    if (!buffer)
      return;
    CVPixelBufferRetain(buffer);
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_)
      CVPixelBufferRelease(latest_);
    latest_ = buffer;
  }

  void streamStopped() {
    dispatch_async(queue_, ^{
      stream_ = nil;
      window_id_ = 0;
      clearLatest();
    });
  }

private:
  // on queue_: finds the game's window, and captures it if it is not already
  void look() {
    // Asking for the windows without the permission shows macOS's Screen Recording prompt again, every time: only
    // ask once it is granted (it takes effect for the app that started this server once that app is restarted)
    if (!CGPreflightScreenCaptureAccess()) {
      if (!warned_) {
        logInfo("rift: no Screen Recording permission yet, so the game's window is not shown in the headset (allow it "
                "for the app that starts metalvrd, then restart that app)");
        warned_ = true;
      }
      return;
    }
    [SCShareableContent
        getShareableContentExcludingDesktopWindows:YES
                               onScreenWindowsOnly:YES
                                 completionHandler:^(SCShareableContent *content, NSError *error) {
                                   dispatch_async(queue_, ^{ choose(content, error); });
                                 }];
  }

  void choose(SCShareableContent *content, NSError *error) {
    if (!content) {
      if (!warned_) {
        logInfo("rift: the game's window cannot be shown in the headset: %s (allow Screen Recording for metalvrd, or "
                "for the app that starts it, in System Settings > Privacy & Security)",
                error.localizedDescription.UTF8String ?: "no permission");
        warned_ = true;
      }
      return;
    }
    SCWindow *best = nil;
    double best_area = 0;
    for (SCWindow *window in content.windows) {
      if (!window.onScreen || window.windowLayer != 0 || !window.owningApplication)
        continue;
      if (window.frame.size.width < 320 || window.frame.size.height < 200)
        continue;
      if (containsSteam(window.owningApplication.applicationName) || containsSteam(window.title))
        continue;
      if (!isWineProcess(window.owningApplication.processID))
        continue;
      const double area = window.frame.size.width * window.frame.size.height;
      if (area > best_area) {
        best = window;
        best_area = area;
      }
    }
    if (!best) {
      if (stream_) {
        stopStream();
        clearLatest();
      }
      return;
    }
    if (stream_ && best.windowID == window_id_)
      return;
    stopStream();
    startStream(best);
  }

  void startStream(SCWindow *window) {
    SCContentFilter *filter = [[SCContentFilter alloc] initWithDesktopIndependentWindow:window];
    SCStreamConfiguration *config = [SCStreamConfiguration new];
    // enough for a screen two and a half meters wide in the headset, and no more
    config.width = static_cast<size_t>(std::max(2.0, std::min(1920.0, window.frame.size.width * 2.0)));
    config.height = static_cast<size_t>(std::max(2.0, config.width * window.frame.size.height / window.frame.size.width));
    config.pixelFormat = 'BGRA';
    config.minimumFrameInterval = CMTimeMake(1, 30);
    config.queueDepth = 3;
    config.showsCursor = YES;
    SCStream *stream = [[SCStream alloc] initWithFilter:filter configuration:config delegate:output_];
    NSError *error = nil;
    if (![stream addStreamOutput:output_ type:SCStreamOutputTypeScreen sampleHandlerQueue:queue_ error:&error]) {
      logInfo("rift: cannot capture the game's window: %s", error.localizedDescription.UTF8String);
      return;
    }
    stream_ = stream;
    window_id_ = window.windowID;
    NSString *name = window.owningApplication.applicationName ?: @"?";
    logInfo("rift: showing the window of '%s' (%.0fx%.0f) in the headset until its VR starts", name.UTF8String,
            window.frame.size.width, window.frame.size.height);
    [stream startCaptureWithCompletionHandler:^(NSError *start_error) {
      if (start_error) {
        logInfo("rift: capturing the game's window failed: %s", start_error.localizedDescription.UTF8String);
        streamStopped();
      }
    }];
  }

  void stopStream() {
    if (!stream_)
      return;
    [stream_ stopCaptureWithCompletionHandler:nil];
    stream_ = nil;
    window_id_ = 0;
  }

  void clearLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_)
      CVPixelBufferRelease(latest_);
    latest_ = nullptr;
  }

  MVRWindowStreamOutput *output_ = nil;
  dispatch_queue_t queue_ = nil;
  dispatch_source_t timer_ = nil;
  SCStream *stream_ = nil;
  CGWindowID window_id_ = 0;
  bool warned_ = false;
  std::mutex mutex_;
  CVPixelBufferRef latest_ = nullptr;
};

std::unique_ptr<WindowCapture> makeWindowCapture() { return std::make_unique<ScreenWindowCapture>(); }

} // namespace metalvr

@implementation MVRWindowStreamOutput
- (void)stream:(SCStream *)stream didOutputSampleBuffer:(CMSampleBufferRef)sample ofType:(SCStreamOutputType)type {
  if (type == SCStreamOutputTypeScreen && self.owner)
    self.owner->received(sample);
}
- (void)stream:(SCStream *)stream didStopWithError:(NSError *)error {
  if (self.owner)
    self.owner->streamStopped();
}
@end
