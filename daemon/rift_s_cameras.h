// The Rift S's cameras as macOS sees them: a UVC video device named "Rift S Sensor" (Monado's rift_s_camera.c finds
// it by that name), captured with AVFoundation. Its frames are 8-bit grey, 3200 wide, though the device calls them
// YUYV at 1600: they are asked for as YUYV so that macOS hands the bytes over unconverted, and read as grey. If
// macOS instead gives full-width pictures (from an MJPEG mode), their brightness is taken.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace metalvr {

class RiftSCameras {
public:
  virtual ~RiftSCameras() = default;
};

// grey bytes, their width and height, and the bytes from one row to the next; on AVFoundation's queue
using RiftSFrameHandler = std::function<void(const uint8_t *grey, uint32_t width, uint32_t height, size_t stride)>;

// the capture started; null if macOS has no "Rift S Sensor" camera (yet: it appears after the headset is turned
// on) or does not let metalvrd use cameras (System Settings, Privacy & Security, Camera)
std::unique_ptr<RiftSCameras> startRiftSCameras(RiftSFrameHandler handler);

} // namespace metalvr
