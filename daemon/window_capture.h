// The window of the Windows game that is starting, captured for the headset's home while the game has no VR session
// yet (its launcher, its loading screen): the largest on-screen window of a Wine process that is not Steam's.
#pragma once

#include <CoreVideo/CVPixelBuffer.h>

#include <memory>

namespace metalvr {

class WindowCapture {
public:
  virtual ~WindowCapture() = default;
  // the newest captured picture of the window, retained for the caller; null while there is no such window, or no
  // permission to record the screen
  virtual CVPixelBufferRef latest() = 0;
};

// starts looking for the window at once, and again every few seconds
std::unique_ptr<WindowCapture> makeWindowCapture();

} // namespace metalvr
