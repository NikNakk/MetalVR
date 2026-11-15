#include "gfx.h"

#include <cstdio>
#include <cstring>
#include <map>

namespace {

struct Chain {
  uint32_t width = 0, height = 0, array_size = 1, samples = 1;
  DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
  std::vector<ID3D11Texture2D *> images;
  std::vector<ID3D11RenderTargetView *> views; // image * array_size + slice
};

class D3D11Graphics final : public TestGraphics {
public:
  ~D3D11Graphics() override {
    for (auto &entry : chains_)
      for (auto *view : entry.second.views)
        view->Release();
    if (pattern_)
      pattern_->Release();
    if (context_)
      context_->Release();
    if (device_)
      device_->Release();
  }

  const char *extensionName() const override { return XR_KHR_D3D11_ENABLE_EXTENSION_NAME; }

  bool init(XrInstance instance, XrSystemId system, PFN_xrGetInstanceProcAddr get_proc) override {
    PFN_xrGetD3D11GraphicsRequirementsKHR get_requirements = nullptr;
    if (XR_FAILED(get_proc(instance, "xrGetD3D11GraphicsRequirementsKHR",
                           reinterpret_cast<PFN_xrVoidFunction *>(&get_requirements))))
      return false;
    XrGraphicsRequirementsD3D11KHR requirements{XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR};
    if (XR_FAILED(get_requirements(instance, system, &requirements)))
      return false;
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0};
    D3D_FEATURE_LEVEL level{};
    const HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 2, D3D11_SDK_VERSION,
                                         &device_, &level, &context_);
    if (FAILED(hr)) {
      std::printf("D3D11CreateDevice failed: 0x%08lx\n", static_cast<unsigned long>(hr));
      return false;
    }
    if (level < requirements.minFeatureLevel)
      return false;
    binding_.device = device_;
    return true;
  }

  const void *binding() const override { return &binding_; }

  int64_t chooseFormat(const std::vector<int64_t> &formats, bool srgb) const override {
    const DXGI_FORMAT wanted = srgb ? DXGI_FORMAT_R8G8B8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM;
    for (int64_t format : formats)
      if (format == wanted)
        return format;
    return 0;
  }

  bool adopt(int id, XrSwapchain swapchain, PFN_xrEnumerateSwapchainImages enumerate, uint32_t width,
             uint32_t height, int64_t format, uint32_t array_size, uint32_t samples) override {
    uint32_t count = 0;
    if (XR_FAILED(enumerate(swapchain, 0, &count, nullptr)) || !count)
      return false;
    std::vector<XrSwapchainImageD3D11KHR> images(count, {XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR});
    if (XR_FAILED(enumerate(swapchain, count, &count, reinterpret_cast<XrSwapchainImageBaseHeader *>(images.data()))))
      return false;
    Chain chain;
    chain.width = width;
    chain.height = height;
    chain.array_size = array_size;
    chain.samples = samples;
    chain.format = static_cast<DXGI_FORMAT>(format);
    for (const auto &image : images) {
      chain.images.push_back(image.texture);
      for (uint32_t slice = 0; slice < array_size; ++slice) {
        // the runtime's textures are typeless: the view names the format
        D3D11_RENDER_TARGET_VIEW_DESC desc{};
        desc.Format = chain.format;
        if (samples > 1) {
          desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2DMSARRAY;
          desc.Texture2DMSArray.FirstArraySlice = slice;
          desc.Texture2DMSArray.ArraySize = 1;
        } else {
          desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2DARRAY;
          desc.Texture2DArray.FirstArraySlice = slice;
          desc.Texture2DArray.ArraySize = 1;
        }
        ID3D11RenderTargetView *view = nullptr;
        const HRESULT hr = device_->CreateRenderTargetView(image.texture, &desc, &view);
        if (FAILED(hr)) {
          std::printf("CreateRenderTargetView failed: 0x%08lx\n", static_cast<unsigned long>(hr));
          return false;
        }
        chain.views.push_back(view);
      }
    }
    chains_[id] = chain;
    return ensurePattern(chain.format);
  }

  void draw(int id, uint32_t image, uint32_t slice, const float color[4], uint32_t frame) override {
    Chain &chain = chains_[id];
    context_->ClearRenderTargetView(chain.views[image * chain.array_size + slice], color);
    if (!pattern_ || chain.samples > 1 || chain.width < kPattern * 2 || chain.height < kPattern * 2)
      return;
    // the checkerboard slides left to right so consecutive frames differ, and sits in the upper half so a
    // vertically flipped image is told from an upright one
    const uint32_t span = chain.width - kPattern;
    const uint32_t x = (frame * 8) % span;
    const uint32_t y = chain.height / 4 - kPattern / 2;
    context_->CopySubresourceRegion(chain.images[image], D3D11CalcSubresource(0, slice, 1), x, y, 0, pattern_, 0,
                                    nullptr);
  }

  void submit() override {}

private:
  static constexpr uint32_t kPattern = 256;

  bool ensurePattern(DXGI_FORMAT format) {
    if (pattern_)
      return true;
    // the checkerboard is 8 bits a channel; swapchains of other formats are only cleared
    switch (format) {
    case DXGI_FORMAT_R8G8B8A8_UNORM:
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
    case DXGI_FORMAT_B8G8R8A8_UNORM:
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
    case DXGI_FORMAT_B8G8R8X8_UNORM:
    case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
      break;
    default:
      return true;
    }
    std::vector<uint32_t> pixels(kPattern * kPattern);
    for (uint32_t y = 0; y < kPattern; ++y)
      for (uint32_t x = 0; x < kPattern; ++x)
        pixels[y * kPattern + x] = ((x / 32 + y / 32) & 1) ? 0xffffffffu : 0xff000000u;
    D3D11_TEXTURE2D_DESC desc{};
    desc.Width = desc.Height = kPattern;
    desc.MipLevels = 1;
    desc.ArraySize = 1;
    desc.Format = format;
    desc.SampleDesc.Count = 1;
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    D3D11_SUBRESOURCE_DATA data{pixels.data(), kPattern * 4, 0};
    const HRESULT hr = device_->CreateTexture2D(&desc, &data, &pattern_);
    if (FAILED(hr))
      std::printf("pattern texture failed: 0x%08lx\n", static_cast<unsigned long>(hr));
    return SUCCEEDED(hr);
  }

  ID3D11Device *device_ = nullptr;
  ID3D11DeviceContext *context_ = nullptr;
  ID3D11Texture2D *pattern_ = nullptr;
  XrGraphicsBindingD3D11KHR binding_{XR_TYPE_GRAPHICS_BINDING_D3D11_KHR};
  std::map<int, Chain> chains_;
};

} // namespace

TestGraphics *createTestGraphics() { return new D3D11Graphics(); }
