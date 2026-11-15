// The Direct3D 11 backend, for Windows applications under Wine with Mullion.
//
// An application's swapchain images are ordinary textures on its own device: typeless, arrays or multisampled as
// it asked. What the server reads are separate shared textures, one per image and array slice, that Mullion backs
// with Metal shared textures. Releasing an image copies (or resolves) it into its shares, on the GPU, so the
// application-facing texture can be anything Direct3D allows while the share stays a plain 2D texture.
#include "api.h"

#include <d3d11.h>
#include <dxgi1_2.h>

namespace metalvr {

namespace {

struct FormatInfo {
  DXGI_FORMAT format;
  DXGI_FORMAT typeless;
  uint32_t share; // mvr_format; MVR_FORMAT_UNKNOWN for depth
};

// in the order of preference an application sees
const FormatInfo kFormats[] = {
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

const FormatInfo *formatInfo(int64_t format) {
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

// The bootstrap name Mullion registered for a shared texture's Metal port.
//
// Mullion's NT handle to a shared texture is a file mapping whose bytes describe the share, the name among them
// (d3d11_texture_device.cpp, SharedTextureData). The name is "MULLION_shared_" and 32 hexadecimal digits
// (d3d11_resource.hpp, MakeUniqueSharedName); it is found by that shape rather than at an offset, so a change to
// the struct's layout does not silently read another field. The share's own mapping name starts the same way and
// is told apart by what follows the prefix.
bool mullionShareName(ID3D11Texture2D *texture, char out[MVR_SHARE_NAME_LEN]) {
  static const char prefix[] = "MULLION_shared_";
  IDXGIResource1 *resource = nullptr;
  if (FAILED(texture->QueryInterface(__uuidof(IDXGIResource1), reinterpret_cast<void **>(&resource))))
    return false;
  HANDLE handle = nullptr;
  const HRESULT hr =
      resource->CreateSharedHandle(nullptr, DXGI_SHARED_RESOURCE_READ | DXGI_SHARED_RESOURCE_WRITE, nullptr, &handle);
  resource->Release();
  if (FAILED(hr) || !handle)
    return false;
  bool found = false;
  if (void *view = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, 0)) {
    MEMORY_BASIC_INFORMATION region{};
    if (VirtualQuery(view, &region, sizeof(region)) && region.RegionSize > sizeof(prefix)) {
      const char *bytes = static_cast<const char *>(view);
      const size_t size = region.RegionSize;
      for (size_t i = 0; i + sizeof(prefix) < size && !found; ++i) {
        if (std::memcmp(bytes + i, prefix, sizeof(prefix) - 1))
          continue;
        const size_t digits = 32, length = sizeof(prefix) - 1 + digits;
        if (i + length >= size || bytes[i + length])
          continue;
        bool hex = true;
        for (size_t d = sizeof(prefix) - 1; d < length && hex; ++d) {
          const char c = bytes[i + d];
          hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        }
        if (!hex)
          continue;
        std::memset(out, 0, MVR_SHARE_NAME_LEN);
        std::memcpy(out, bytes + i, length);
        found = true;
      }
    }
    UnmapViewOfFile(view);
  }
  CloseHandle(handle);
  return found;
}

struct D3D11Images final : SwapchainImages {
  ID3D11DeviceContext *context = nullptr;
  std::vector<ID3D11Texture2D *> app;    // one per image
  std::vector<ID3D11Texture2D *> shares; // image * array_size + slice; empty for depth
  uint32_t array_size = 1;
  uint32_t mip_count = 1;
  uint32_t sample_count = 1;
  DXGI_FORMAT typed = DXGI_FORMAT_UNKNOWN;

  ~D3D11Images() override {
    for (auto *&texture : app)
      release(texture);
    for (auto *&texture : shares)
      release(texture);
    release(context);
  }

  XrResult enumerate(uint32_t capacity, uint32_t *count_out, XrSwapchainImageBaseHeader *images) override {
    if (!count_out)
      return XR_ERROR_VALIDATION_FAILURE;
    *count_out = count;
    if (capacity == 0)
      return XR_SUCCESS;
    if (capacity < count)
      return XR_ERROR_SIZE_INSUFFICIENT;
    if (!images)
      return XR_ERROR_VALIDATION_FAILURE;
    auto *out = reinterpret_cast<XrSwapchainImageD3D11KHR *>(images);
    for (uint32_t i = 0; i < count; ++i) {
      if (out[i].type != XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR)
        return XR_ERROR_VALIDATION_FAILURE;
      out[i].texture = app[i];
    }
    return XR_SUCCESS;
  }

  void released(uint32_t index) override {
    if (shares.empty())
      return;
    for (uint32_t slice = 0; slice < array_size; ++slice) {
      ID3D11Texture2D *share = shares[index * array_size + slice];
      const UINT source = D3D11CalcSubresource(0, slice, mip_count);
      if (sample_count > 1)
        context->ResolveSubresource(share, 0, app[index], source, typed);
      else
        context->CopySubresourceRegion(share, 0, 0, 0, 0, app[index], source, nullptr);
    }
  }
};

class D3D11Backend final : public GraphicsBackend {
public:
  explicit D3D11Backend(ID3D11Device *device) : device_(device) {
    device_->AddRef();
    device_->GetImmediateContext(&context_);
    for (const FormatInfo &info : kFormats)
      formats_.push_back(info.format);
  }

  ~D3D11Backend() override {
    release(query_);
    release(context_);
    release(device_);
  }

  uint32_t api() const override { return MVR_GFX_D3D11; }
  const std::vector<int64_t> &formats() const override { return formats_; }
  uint32_t maxSampleCount() const override { return kMaxSampleCount; }

  XrResult createSwapchain(const XrSwapchainCreateInfo &info, std::unique_ptr<SwapchainImages> &out,
                           mvr_swapchain_create_request &share, bool &composited) override {
    const FormatInfo *format = formatInfo(info.format);
    if (!format)
      return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
    const bool depth = format->share == MVR_FORMAT_UNKNOWN;
    const uint32_t image_count = (info.createFlags & XR_SWAPCHAIN_CREATE_STATIC_IMAGE_BIT) ? 1 : 3;
    if (!depth && image_count * info.arraySize > MVR_MAX_SWAPCHAIN_IMAGES)
      return XR_ERROR_FEATURE_UNSUPPORTED;

    auto images = std::make_unique<D3D11Images>();
    images->context = context_;
    context_->AddRef();
    images->count = image_count;
    images->array_size = info.arraySize;
    images->mip_count = info.mipCount;
    images->sample_count = info.sampleCount;
    images->typed = format->format;

    D3D11_TEXTURE2D_DESC desc{};
    desc.Width = info.width;
    desc.Height = info.height;
    desc.MipLevels = info.mipCount;
    desc.ArraySize = info.arraySize;
    // typeless, as Direct3D runtimes hand them out: the application picks the view format, sRGB or not
    desc.Format = format->typeless;
    desc.SampleDesc.Count = info.sampleCount;
    desc.Usage = D3D11_USAGE_DEFAULT;
    if (depth) {
      desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
      if ((info.usageFlags & XR_SWAPCHAIN_USAGE_SAMPLED_BIT) && info.sampleCount == 1)
        desc.BindFlags |= D3D11_BIND_SHADER_RESOURCE;
    } else {
      desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
      if ((info.usageFlags & XR_SWAPCHAIN_USAGE_UNORDERED_ACCESS_BIT) && info.sampleCount == 1)
        desc.BindFlags |= D3D11_BIND_UNORDERED_ACCESS;
    }
    for (uint32_t i = 0; i < image_count; ++i) {
      ID3D11Texture2D *texture = nullptr;
      const HRESULT hr = device_->CreateTexture2D(&desc, nullptr, &texture);
      if (FAILED(hr)) {
        logLine("swapchain: CreateTexture2D %ux%u format %d failed (0x%08lx)", info.width, info.height,
                static_cast<int>(desc.Format), static_cast<unsigned long>(hr));
        return XR_ERROR_RUNTIME_FAILURE;
      }
      images->app.push_back(texture);
    }

    composited = !depth;
    if (composited) {
      D3D11_TEXTURE2D_DESC shared{};
      shared.Width = info.width;
      shared.Height = info.height;
      shared.MipLevels = 1;
      shared.ArraySize = 1;
      shared.Format = format->format;
      shared.SampleDesc.Count = 1;
      shared.Usage = D3D11_USAGE_DEFAULT;
      shared.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
      shared.MiscFlags = D3D11_RESOURCE_MISC_SHARED | D3D11_RESOURCE_MISC_SHARED_NTHANDLE;

      share.width = info.width;
      share.height = info.height;
      share.format = format->share;
      share.array_size = info.arraySize;
      share.share_kind = MVR_SHARE_MULLION_BOOTSTRAP;
      share.image_count = image_count;
      for (uint32_t i = 0; i < image_count * info.arraySize; ++i) {
        ID3D11Texture2D *texture = nullptr;
        const HRESULT hr = device_->CreateTexture2D(&shared, nullptr, &texture);
        if (FAILED(hr)) {
          logLine("swapchain: shared texture %ux%u format %d failed (0x%08lx)", info.width, info.height,
                  static_cast<int>(shared.Format), static_cast<unsigned long>(hr));
          return XR_ERROR_RUNTIME_FAILURE;
        }
        images->shares.push_back(texture);
        if (!mullionShareName(texture, share.images[i].name)) {
          // without Mullion's shared textures there is no way to hand the image to the server
          logLine("swapchain: this Direct3D 11 does not expose a Mullion shared texture; MetalVR needs Mullion");
          return XR_ERROR_RUNTIME_FAILURE;
        }
      }
    }
    out = std::move(images);
    return XR_SUCCESS;
  }

  void finishFrame() override {
    if (!query_) {
      D3D11_QUERY_DESC desc{D3D11_QUERY_EVENT, 0};
      if (FAILED(device_->CreateQuery(&desc, &query_))) {
        context_->Flush();
        return;
      }
    }
    // the event completes when the GPU has executed everything issued before it, the copies into the shares
    // included; only then may another process read them
    context_->End(query_);
    context_->Flush();
    const int64_t deadline = localNowNs() + 500'000'000;
    BOOL done = FALSE;
    while (context_->GetData(query_, &done, sizeof(done), 0) != S_OK || !done) {
      if (localNowNs() > deadline) {
        logLine("the GPU did not finish the frame within 500 ms; it is submitted as it is");
        break;
      }
      SwitchToThread();
    }
  }

private:
  ID3D11Device *device_ = nullptr;
  ID3D11DeviceContext *context_ = nullptr;
  ID3D11Query *query_ = nullptr;
  std::vector<int64_t> formats_;
};

} // namespace

uint32_t platformGraphicsApi() { return MVR_GFX_D3D11; }

XrResult createGraphicsBackend(const XrSessionCreateInfo *info, std::unique_ptr<GraphicsBackend> &out) {
  const XrGraphicsBindingD3D11KHR *binding = nullptr;
  for (auto *base = static_cast<const XrBaseInStructure *>(info->next); base; base = base->next)
    if (base->type == XR_TYPE_GRAPHICS_BINDING_D3D11_KHR)
      binding = reinterpret_cast<const XrGraphicsBindingD3D11KHR *>(base);
  if (!binding || !binding->device)
    return XR_ERROR_GRAPHICS_DEVICE_INVALID;
  out = std::make_unique<D3D11Backend>(binding->device);
  return XR_SUCCESS;
}

namespace api {

MVR_API xrGetD3D11GraphicsRequirementsKHR(XrInstance handle, XrSystemId system_id,
                                          XrGraphicsRequirementsD3D11KHR *requirements) {
  MVR_INSTANCE(handle);
  if (!requirements || requirements->type != XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR)
    return XR_ERROR_VALIDATION_FAILURE;
  if (system_id != kSystemId)
    return XR_ERROR_SYSTEM_INVALID;
  requirements->adapterLuid = LUID{};
  requirements->minFeatureLevel = D3D_FEATURE_LEVEL_11_0;
  IDXGIFactory1 *factory = nullptr;
  if (SUCCEEDED(CreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void **>(&factory)))) {
    IDXGIAdapter1 *adapter = nullptr;
    if (SUCCEEDED(factory->EnumAdapters1(0, &adapter))) {
      DXGI_ADAPTER_DESC1 desc{};
      if (SUCCEEDED(adapter->GetDesc1(&desc)))
        requirements->adapterLuid = desc.AdapterLuid;
      adapter->Release();
    }
    factory->Release();
  }
  instance->graphics_requirements_queried = true;
  return XR_SUCCESS;
}

} // namespace api
} // namespace metalvr
