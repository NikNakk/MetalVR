// Direct3D 12 for the smoke test (xr_smoke_d3d12): the same clears and moving checkerboard as gfx_d3d11.cpp,
// recorded into one command list a frame and run on the queue the session was made with.
#include "gfx.h"

#include <dxgi1_4.h>

#include <cstdio>
#include <cstring>
#include <map>

namespace {

template <typename T> void release(T *&object) {
  if (object) {
    object->Release();
    object = nullptr;
  }
}

D3D12_RESOURCE_BARRIER transition(ID3D12Resource *resource, UINT subresource, D3D12_RESOURCE_STATES before,
                                  D3D12_RESOURCE_STATES after) {
  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  barrier.Transition.pResource = resource;
  barrier.Transition.Subresource = subresource;
  barrier.Transition.StateBefore = before;
  barrier.Transition.StateAfter = after;
  return barrier;
}

struct Chain {
  uint32_t width = 0, height = 0, array_size = 1, mip_count = 1, samples = 1;
  DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
  std::vector<ID3D12Resource *> images;
  ID3D12DescriptorHeap *views = nullptr; // image * array_size + slice
};

class D3D12Graphics final : public TestGraphics {
public:
  ~D3D12Graphics() override {
    waitIdle();
    for (auto &entry : chains_)
      release(entry.second.views);
    release(pattern_);
    release(list_);
    release(allocator_);
    release(fence_);
    if (event_)
      CloseHandle(event_);
    release(queue_);
    release(device_);
  }

  const char *extensionName() const override { return XR_KHR_D3D12_ENABLE_EXTENSION_NAME; }

  bool init(XrInstance instance, XrSystemId system, PFN_xrGetInstanceProcAddr get_proc) override {
    PFN_xrGetD3D12GraphicsRequirementsKHR get_requirements = nullptr;
    if (XR_FAILED(get_proc(instance, "xrGetD3D12GraphicsRequirementsKHR",
                           reinterpret_cast<PFN_xrVoidFunction *>(&get_requirements))))
      return false;
    XrGraphicsRequirementsD3D12KHR requirements{XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR};
    if (XR_FAILED(get_requirements(instance, system, &requirements)))
      return false;
    HRESULT hr = D3D12CreateDevice(nullptr, requirements.minFeatureLevel, __uuidof(ID3D12Device),
                                   reinterpret_cast<void **>(&device_));
    if (FAILED(hr)) {
      std::printf("D3D12CreateDevice failed: 0x%08lx\n", static_cast<unsigned long>(hr));
      return false;
    }
    D3D12_COMMAND_QUEUE_DESC queue{};
    queue.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    if (FAILED(device_->CreateCommandQueue(&queue, __uuidof(ID3D12CommandQueue), reinterpret_cast<void **>(&queue_))) ||
        FAILED(device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, __uuidof(ID3D12CommandAllocator),
                                               reinterpret_cast<void **>(&allocator_))) ||
        FAILED(device_->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator_, nullptr,
                                          __uuidof(ID3D12GraphicsCommandList), reinterpret_cast<void **>(&list_))) ||
        FAILED(device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, __uuidof(ID3D12Fence),
                                    reinterpret_cast<void **>(&fence_)))) {
      std::printf("Direct3D 12 queue, command list or fence failed\n");
      return false;
    }
    event_ = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    recording_ = true;
    binding_.device = device_;
    binding_.queue = queue_;
    rtv_size_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    return event_ != nullptr;
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
    std::vector<XrSwapchainImageD3D12KHR> images(count, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
    if (XR_FAILED(enumerate(swapchain, count, &count, reinterpret_cast<XrSwapchainImageBaseHeader *>(images.data()))))
      return false;
    Chain chain;
    chain.width = width;
    chain.height = height;
    chain.array_size = array_size;
    chain.samples = samples;
    chain.format = static_cast<DXGI_FORMAT>(format);
    D3D12_DESCRIPTOR_HEAP_DESC heap{};
    heap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    heap.NumDescriptors = count * array_size;
    if (FAILED(device_->CreateDescriptorHeap(&heap, __uuidof(ID3D12DescriptorHeap),
                                             reinterpret_cast<void **>(&chain.views))))
      return false;
    D3D12_CPU_DESCRIPTOR_HANDLE handle = chain.views->GetCPUDescriptorHandleForHeapStart();
    for (const auto &image : images) {
      chain.images.push_back(image.texture);
      chain.mip_count = image.texture->GetDesc().MipLevels;
      for (uint32_t slice = 0; slice < array_size; ++slice) {
        // the runtime's textures are typeless: the view names the format
        D3D12_RENDER_TARGET_VIEW_DESC desc{};
        desc.Format = chain.format;
        if (samples > 1) {
          desc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2DMSARRAY;
          desc.Texture2DMSArray.FirstArraySlice = slice;
          desc.Texture2DMSArray.ArraySize = 1;
        } else {
          desc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2DARRAY;
          desc.Texture2DArray.FirstArraySlice = slice;
          desc.Texture2DArray.ArraySize = 1;
        }
        device_->CreateRenderTargetView(image.texture, &desc, handle);
        handle.ptr += rtv_size_;
      }
    }
    chains_[id] = chain;
    return ensurePattern(chain.format);
  }

  void draw(int id, uint32_t image, uint32_t slice, const float color[4], uint32_t frame) override {
    if (!begin())
      return;
    Chain &chain = chains_[id];
    D3D12_CPU_DESCRIPTOR_HANDLE view = chain.views->GetCPUDescriptorHandleForHeapStart();
    view.ptr += static_cast<SIZE_T>(image * chain.array_size + slice) * rtv_size_;
    list_->ClearRenderTargetView(view, color, 0, nullptr);
    if (!pattern_ || chain.samples > 1 || chain.width < kPattern * 2 || chain.height < kPattern * 2)
      return;
    // the checkerboard slides left to right so consecutive frames differ, and sits in the upper half so a
    // vertically flipped image is told from an upright one
    const uint32_t span = chain.width - kPattern;
    const uint32_t x = (frame * 8) % span;
    const uint32_t y = chain.height / 4 - kPattern / 2;
    ID3D12Resource *target = chain.images[image];
    const UINT subresource = slice * chain.mip_count;
    D3D12_RESOURCE_BARRIER to_copy =
        transition(target, subresource, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
    list_->ResourceBarrier(1, &to_copy);
    D3D12_TEXTURE_COPY_LOCATION to{target, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
    to.SubresourceIndex = subresource;
    D3D12_TEXTURE_COPY_LOCATION from{pattern_, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
    from.SubresourceIndex = 0;
    list_->CopyTextureRegion(&to, x, y, 0, &from, nullptr);
    D3D12_RESOURCE_BARRIER back =
        transition(target, subresource, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
    list_->ResourceBarrier(1, &back);
  }

  void submit() override {
    if (!recording_)
      return;
    run();
  }

private:
  static constexpr uint32_t kPattern = 256;

  // the frame's list, reset once the previous frame's has run
  bool begin() {
    if (recording_)
      return true;
    waitIdle();
    if (FAILED(allocator_->Reset()) || FAILED(list_->Reset(allocator_, nullptr)))
      return false;
    recording_ = true;
    return true;
  }

  void run() {
    recording_ = false;
    if (FAILED(list_->Close())) {
      std::printf("command list did not close\n");
      return;
    }
    ID3D12CommandList *lists[] = {list_};
    queue_->ExecuteCommandLists(1, lists);
    queue_->Signal(fence_, ++fence_value_);
  }

  void waitIdle() {
    if (!fence_ || fence_->GetCompletedValue() >= fence_value_)
      return;
    if (SUCCEEDED(fence_->SetEventOnCompletion(fence_value_, event_)))
      WaitForSingleObject(event_, 2000);
  }

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
    if (!begin())
      return false;
    D3D12_HEAP_PROPERTIES heap{};
    heap.Type = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_DESC desc{};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    desc.Width = desc.Height = kPattern;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.Format = format;
    desc.SampleDesc.Count = 1;
    HRESULT hr = device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_DEST,
                                                  nullptr, __uuidof(ID3D12Resource),
                                                  reinterpret_cast<void **>(&pattern_));
    if (FAILED(hr)) {
      std::printf("pattern texture failed: 0x%08lx\n", static_cast<unsigned long>(hr));
      return false;
    }
    // rows of 1024 bytes already meet the 256-byte pitch alignment
    const uint32_t pitch = kPattern * 4;
    D3D12_HEAP_PROPERTIES upload_heap{};
    upload_heap.Type = D3D12_HEAP_TYPE_UPLOAD;
    D3D12_RESOURCE_DESC buffer{};
    buffer.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    buffer.Width = pitch * kPattern;
    buffer.Height = 1;
    buffer.DepthOrArraySize = 1;
    buffer.MipLevels = 1;
    buffer.SampleDesc.Count = 1;
    buffer.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    ID3D12Resource *upload = nullptr;
    hr = device_->CreateCommittedResource(&upload_heap, D3D12_HEAP_FLAG_NONE, &buffer,
                                          D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, __uuidof(ID3D12Resource),
                                          reinterpret_cast<void **>(&upload));
    if (FAILED(hr)) {
      std::printf("pattern upload buffer failed: 0x%08lx\n", static_cast<unsigned long>(hr));
      return false;
    }
    uint32_t *pixels = nullptr;
    if (FAILED(upload->Map(0, nullptr, reinterpret_cast<void **>(&pixels)))) {
      release(upload);
      return false;
    }
    for (uint32_t y = 0; y < kPattern; ++y)
      for (uint32_t x = 0; x < kPattern; ++x)
        pixels[y * kPattern + x] = ((x / 32 + y / 32) & 1) ? 0xffffffffu : 0xff000000u;
    upload->Unmap(0, nullptr);
    D3D12_TEXTURE_COPY_LOCATION to{pattern_, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
    to.SubresourceIndex = 0;
    D3D12_TEXTURE_COPY_LOCATION from{upload, D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT, {}};
    from.PlacedFootprint.Footprint = {format, kPattern, kPattern, 1, pitch};
    list_->CopyTextureRegion(&to, 0, 0, 0, &from, nullptr);
    D3D12_RESOURCE_BARRIER to_source =
        transition(pattern_, 0, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COPY_SOURCE);
    list_->ResourceBarrier(1, &to_source);
    run();
    waitIdle();
    release(upload);
    return true;
  }

  ID3D12Device *device_ = nullptr;
  ID3D12CommandQueue *queue_ = nullptr;
  ID3D12CommandAllocator *allocator_ = nullptr;
  ID3D12GraphicsCommandList *list_ = nullptr;
  ID3D12Fence *fence_ = nullptr;
  uint64_t fence_value_ = 0;
  HANDLE event_ = nullptr;
  bool recording_ = false;
  UINT rtv_size_ = 0;
  ID3D12Resource *pattern_ = nullptr;
  XrGraphicsBindingD3D12KHR binding_{XR_TYPE_GRAPHICS_BINDING_D3D12_KHR};
  std::map<int, Chain> chains_;
};

} // namespace

TestGraphics *createTestGraphics() { return new D3D12Graphics(); }
