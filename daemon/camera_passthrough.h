// The headset's own cameras, shown in its home (docs/HEADSETS.md, phase F): a Valve Index's front cameras (one USB
// video device giving both pictures side by side), or a Vive's or Vive Pro's, captured with AVFoundation as
// macOS's camera stack owns them. The pictures are drawn as straight-ahead views of a fixed field of view: the
// cameras' own calibration (in the headset's configuration) is not applied yet.
#pragma once

#include <CoreVideo/CVPixelBuffer.h>

#include <memory>

namespace metalvr {

class CameraPassthrough {
public:
  virtual ~CameraPassthrough() = default;
  // the newest picture, retained for the caller; null until one comes
  virtual CVPixelBufferRef latest() = 0;
  // whether the picture holds both eyes side by side (an Index's), or one picture for both
  virtual bool stereo() const = 0;
};

// The camera of the headset `headset` (mvr_headset), started; null if macOS has no such camera or does not let
// metalvrd use it (System Settings, Privacy & Security, Camera).
std::unique_ptr<CameraPassthrough> startCameraPassthrough(uint32_t headset);

} // namespace metalvr
