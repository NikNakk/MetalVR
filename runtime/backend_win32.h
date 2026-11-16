// What the Windows backends share: the swapchain formats they offer, and the name of a Mullion shared texture's
// Metal port, read from a handle of the share. Direct3D 11 and Direct3D 12 share textures the same way under Mullion.
#pragma once

#include "api.h"

#include <dxgi1_2.h>

namespace metalvr {

struct FormatInfo {
  DXGI_FORMAT format;
  DXGI_FORMAT typeless;
  uint32_t share; // mvr_format; MVR_FORMAT_UNKNOWN for depth
};

// in the order of preference an application sees
inline const FormatInfo kFormats[] = {
    {DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, DXGI_FORMAT_R8G8B8A8_TYPELESS, MVR_FORMAT_RGBA8_SRGB},
    {DXGI_FORMAT_B8G8R8A8_UNORM_SRGB, DXGI_FORMAT_B8G8R8A8_TYPELESS, MVR_FORMAT_BGRA8_SRGB},
    {DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_R8G8B8A8_TYPELESS, MVR_FORMAT_RGBA8_UNORM},
    {DXGI_FORMAT_B8G8R8A8_UNORM, DXGI_FORMAT_B8G8R8A8_TYPELESS, MVR_FORMAT_BGRA8_UNORM},
    {DXGI_FORMAT_R16G16B16A16_FLOAT, DXGI_FORMAT_R16G16B16A16_TYPELESS, MVR_FORMAT_RGBA16_FLOAT},
    // not what an OpenXR application picks first, but what an OpenVR game's own textures can be: OpenComposite
    // asks for the format of the texture the game submits
    {DXGI_FORMAT_R10G10B10A2_UNORM, DXGI_FORMAT_R10G10B10A2_TYPELESS, MVR_FORMAT_RGB10A2_UNORM},
    {DXGI_FORMAT_R16G16B16A16_UNORM, DXGI_FORMAT_R16G16B16A16_TYPELESS, MVR_FORMAT_RGBA16_UNORM},
    {DXGI_FORMAT_B8G8R8X8_UNORM_SRGB, DXGI_FORMAT_B8G8R8X8_TYPELESS, MVR_FORMAT_BGRX8_SRGB},
    {DXGI_FORMAT_B8G8R8X8_UNORM, DXGI_FORMAT_B8G8R8X8_TYPELESS, MVR_FORMAT_BGRX8_UNORM},
    {DXGI_FORMAT_D32_FLOAT, DXGI_FORMAT_R32_TYPELESS, MVR_FORMAT_UNKNOWN},
    {DXGI_FORMAT_D24_UNORM_S8_UINT, DXGI_FORMAT_R24G8_TYPELESS, MVR_FORMAT_UNKNOWN},
    {DXGI_FORMAT_D16_UNORM, DXGI_FORMAT_R16_TYPELESS, MVR_FORMAT_UNKNOWN},
    {DXGI_FORMAT_D32_FLOAT_S8X24_UINT, DXGI_FORMAT_R32G8X24_TYPELESS, MVR_FORMAT_UNKNOWN},
};

inline const FormatInfo *formatInfo(int64_t format) {
  for (const FormatInfo &info : kFormats)
    if (info.format == static_cast<DXGI_FORMAT>(format))
      return &info;
  return nullptr;
}

template <typename T> void release(T *&pointer) {
  if (pointer) {
    pointer->Release();
    pointer = nullptr;
  }
}

// The bootstrap name Mullion registered for a shared texture's Metal port, from an NT handle of the share (which it
// closes): Mullion's handle is a file mapping whose bytes describe the share (mullion_shared_texture.hpp,
// SharedTextureData), the name among them. The name is "MULLION_shared_" and 32 hexadecimal digits
// (MakeUniqueSharedName); it is found by that shape rather than at an offset, so a change to the struct's layout does
// not silently read another field. The share's own mapping name starts the same way and is told apart by what
// follows the prefix.
bool mullionShareNameFromHandle(HANDLE handle, char out[MVR_SHARE_NAME_LEN]);

} // namespace metalvr
