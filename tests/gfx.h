// The graphics half of the smoke test: Direct3D 11 or 12 on Windows (xr_smoke, xr_smoke_d3d12), Metal on macOS.
#pragma once

#ifdef _WIN32
#define XR_USE_PLATFORM_WIN32 1
#define XR_USE_GRAPHICS_API_D3D11 1
#define XR_USE_GRAPHICS_API_D3D12 1
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <d3d11.h>
#include <d3d12.h>
#else
#define XR_USE_GRAPHICS_API_METAL 1
#endif

// the test calls the runtime through pointers it resolves itself, under the functions' own names
#define XR_NO_PROTOTYPES 1
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <cstdint>
#include <vector>

struct TestGraphics {
  virtual ~TestGraphics() = default;
  virtual const char *extensionName() const = 0;
  // asks the runtime for its graphics requirements and makes a device that meets them
  virtual bool init(XrInstance instance, XrSystemId system, PFN_xrGetInstanceProcAddr get_proc) = 0;
  // the XrGraphicsBinding* to chain into the session create info
  virtual const void *binding() const = 0;
  virtual int64_t chooseFormat(const std::vector<int64_t> &formats, bool srgb) const = 0;
  // takes the images of a new swapchain; `id` is the caller's own name for it
  virtual bool adopt(int id, XrSwapchain swapchain, PFN_xrEnumerateSwapchainImages enumerate, uint32_t width,
                     uint32_t height, int64_t format, uint32_t array_size, uint32_t samples) = 0;
  // clears one slice of one image to `color` and stamps a checkerboard that moves with `frame`
  virtual void draw(int id, uint32_t image, uint32_t slice, const float color[4], uint32_t frame) = 0;
  // hands what was drawn to the GPU; called before the images are released
  virtual void submit() = 0;
};

TestGraphics *createTestGraphics();
