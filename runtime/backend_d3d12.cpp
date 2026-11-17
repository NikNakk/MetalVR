// The Direct3D 12 backend, for Windows applications under Wine with Mullion.
//
// As with Direct3D 11 (backend_d3d11.cpp), an application's swapchain images are ordinary textures on its own
// device, and what the server reads are separate shared textures, one per image and array slice: committed textures
// on a shared heap, which Mullion backs with Metal shared textures. Releasing an image copies (or resolves) it into
// its shares on the application's own queue, so the copies follow its rendering, and the end of a frame waits for
// that queue before the server reads them.
//
// "When an application acquires a swapchain image ... the image will be in the D3D12_RESOURCE_STATE_RENDER_TARGET
// state (color) or D3D12_RESOURCE_STATE_DEPTH_WRITE (depth), and must be returned in that state" (XR_KHR_D3D12_enable).
#include "backend_win32.h"

namespace metalvr {

namespace {

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

// A command list for the copies of one release, reused once the queue is past the value it signalled.
struct CopyList {
  ID3D12CommandAllocator *allocator = nullptr;
  ID3D12GraphicsCommandList *list = nullptr;
  uint64_t fence_value = 0;
};

struct D3D12Images final : SwapchainImages {
  ID3D12Device *device = nullptr;
  ID3D12CommandQueue *queue = nullptr;
  ID3D12Fence *fence = nullptr; // what the copies of each release signal on the queue
  uint64_t fence_value = 0;
  std::vector<CopyList> lists;
  std::vector<ID3D12Resource *> app;    // one per image
  std::vector<ID3D12Resource *> shares; // image * array_size + slice; empty for depth
  uint32_t array_size = 1;
  uint32_t mip_count = 1;
  uint32_t sample_count = 1;
  DXGI_FORMAT typed = DXGI_FORMAT_UNKNOWN;

  ~D3D12Images() override {
    // nothing may still be copying into the shares, or from the application's images, when they go
    if (fence && fence->GetCompletedValue() < fence_value) {
      if (HANDLE event = CreateEventA(nullptr, FALSE, FALSE, nullptr)) {
        if (SUCCEEDED(fence->SetEventOnCompletion(fence_value, event)))
          WaitForSingleObject(event, 1000);
        CloseHandle(event);
      }
    }
    for (CopyList &copy : lists) {
      release(copy.list);
      release(copy.allocator);
    }
    for (auto *&texture : app)
      release(texture);
    for (auto *&texture : shares)
      release(texture);
    release(fence);
    release(queue);
    release(device);
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
    auto *out = reinterpret_cast<XrSwapchainImageD3D12KHR *>(images);
    for (uint32_t i = 0; i < count; ++i) {
      if (out[i].type != XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR)
        return XR_ERROR_VALIDATION_FAILURE;
      out[i].texture = app[i];
    }
    return XR_SUCCESS;
  }

  // a list the queue is done with, or a new one
  CopyList *takeList() {
    const uint64_t completed = fence->GetCompletedValue();
    for (CopyList &copy : lists) {
      if (copy.fence_value <= completed) {
        if (FAILED(copy.allocator->Reset()) || FAILED(copy.list->Reset(copy.allocator, nullptr)))
          return nullptr;
        return &copy;
      }
    }
    CopyList copy;
    if (FAILED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, __uuidof(ID3D12CommandAllocator),
                                              reinterpret_cast<void **>(&copy.allocator))) ||
        FAILED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, copy.allocator, nullptr,
                                         __uuidof(ID3D12GraphicsCommandList), reinterpret_cast<void **>(&copy.list)))) {
      release(copy.allocator);
      return nullptr;
    }
    lists.push_back(copy);
    return &lists.back();
  }

  void released(uint32_t index) override {
    if (shares.empty())
      return;
    CopyList *copy = takeList();
    if (!copy) {
      logLine("swapchain: no command list for a released image's copy");
      return;
    }
    const bool resolve = sample_count > 1;
    const D3D12_RESOURCE_STATES source = resolve ? D3D12_RESOURCE_STATE_RESOLVE_SOURCE : D3D12_RESOURCE_STATE_COPY_SOURCE;
    const D3D12_RESOURCE_STATES dest = resolve ? D3D12_RESOURCE_STATE_RESOLVE_DEST : D3D12_RESOURCE_STATE_COPY_DEST;
    ID3D12GraphicsCommandList *list = copy->list;
    for (uint32_t slice = 0; slice < array_size; ++slice) {
      ID3D12Resource *share = shares[index * array_size + slice];
      const UINT subresource = slice * mip_count; // D3D12CalcSubresource(0, slice, 0, mip_count, array_size)
      D3D12_RESOURCE_BARRIER before[2] = {
          transition(app[index], subresource, D3D12_RESOURCE_STATE_RENDER_TARGET, source),
          transition(share, 0, D3D12_RESOURCE_STATE_COMMON, dest),
      };
      list->ResourceBarrier(2, before);
      if (resolve) {
        list->ResolveSubresource(share, 0, app[index], subresource, typed);
      } else {
        D3D12_TEXTURE_COPY_LOCATION to{share, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
        to.SubresourceIndex = 0;
        D3D12_TEXTURE_COPY_LOCATION from{app[index], D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
        from.SubresourceIndex = subresource;
        list->CopyTextureRegion(&to, 0, 0, 0, &from, nullptr);
      }
      D3D12_RESOURCE_BARRIER after[2] = {
          transition(app[index], subresource, source, D3D12_RESOURCE_STATE_RENDER_TARGET),
          transition(share, 0, dest, D3D12_RESOURCE_STATE_COMMON),
      };
      list->ResourceBarrier(2, after);
    }
    if (FAILED(list->Close())) {
      logLine("swapchain: a released image's copy list did not close");
      return;
    }
    ID3D12CommandList *lists_to_run[] = {list};
    queue->ExecuteCommandLists(1, lists_to_run);
    copy->fence_value = ++fence_value;
    queue->Signal(fence, copy->fence_value);
  }
};

class D3D12Backend final : public GraphicsBackend {
public:
  D3D12Backend(ID3D12Device *device, ID3D12CommandQueue *queue) : device_(device), queue_(queue) {
    device_->AddRef();
    queue_->AddRef();
    for (const FormatInfo &info : kFormats)
      formats_.push_back(info.format);
  }

  ~D3D12Backend() override {
    if (event_)
      CloseHandle(event_);
    if (mark_event_)
      CloseHandle(mark_event_);
    release(fence_);
    release(queue_);
    release(device_);
  }

  bool init() {
    event_ = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    mark_event_ = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    return event_ && mark_event_ && SUCCEEDED(device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, __uuidof(ID3D12Fence),
                                                    reinterpret_cast<void **>(&fence_)));
  }

  uint32_t api() const override { return MVR_GFX_D3D12; }
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

    auto images = std::make_unique<D3D12Images>();
    images->device = device_;
    device_->AddRef();
    images->queue = queue_;
    queue_->AddRef();
    if (FAILED(device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, __uuidof(ID3D12Fence),
                                    reinterpret_cast<void **>(&images->fence))))
      return XR_ERROR_RUNTIME_FAILURE;
    images->count = image_count;
    images->array_size = info.arraySize;
    images->mip_count = info.mipCount;
    images->sample_count = info.sampleCount;
    images->typed = format->format;

    D3D12_HEAP_PROPERTIES heap{};
    heap.Type = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_DESC desc{};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    desc.Width = info.width;
    desc.Height = info.height;
    desc.DepthOrArraySize = static_cast<UINT16>(info.arraySize);
    desc.MipLevels = static_cast<UINT16>(info.mipCount);
    // typeless, as Direct3D runtimes hand them out: the application picks the view format, sRGB or not
    desc.Format = format->typeless;
    desc.SampleDesc.Count = info.sampleCount;
    desc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
    D3D12_RESOURCE_STATES state;
    if (depth) {
      desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
      if (!(info.usageFlags & XR_SWAPCHAIN_USAGE_SAMPLED_BIT))
        desc.Flags |= D3D12_RESOURCE_FLAG_DENY_SHADER_RESOURCE;
      state = D3D12_RESOURCE_STATE_DEPTH_WRITE;
    } else {
      desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
      if ((info.usageFlags & XR_SWAPCHAIN_USAGE_UNORDERED_ACCESS_BIT) && info.sampleCount == 1)
        desc.Flags |= D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
      state = D3D12_RESOURCE_STATE_RENDER_TARGET;
    }
    for (uint32_t i = 0; i < image_count; ++i) {
      ID3D12Resource *texture = nullptr;
      const HRESULT hr = device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, state, nullptr,
                                                          __uuidof(ID3D12Resource), reinterpret_cast<void **>(&texture));
      if (FAILED(hr)) {
        logLine("swapchain: CreateCommittedResource %ux%u format %d failed (0x%08lx)", info.width, info.height,
                static_cast<int>(desc.Format), static_cast<unsigned long>(hr));
        return XR_ERROR_RUNTIME_FAILURE;
      }
      images->app.push_back(texture);
    }

    composited = !depth;
    if (composited) {
      D3D12_RESOURCE_DESC shared = desc;
      shared.DepthOrArraySize = 1;
      shared.MipLevels = 1;
      shared.Format = format->format;
      shared.SampleDesc.Count = 1;
      shared.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

      share.width = info.width;
      share.height = info.height;
      share.format = format->share;
      share.array_size = info.arraySize;
      share.share_kind = MVR_SHARE_MULLION_BOOTSTRAP;
      share.image_count = image_count;
      for (uint32_t i = 0; i < image_count * info.arraySize; ++i) {
        ID3D12Resource *texture = nullptr;
        HRESULT hr = device_->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_SHARED, &shared,
                                                      D3D12_RESOURCE_STATE_COMMON, nullptr, __uuidof(ID3D12Resource),
                                                      reinterpret_cast<void **>(&texture));
        if (FAILED(hr)) {
          logLine("swapchain: shared texture %ux%u format %d failed (0x%08lx)", info.width, info.height,
                  static_cast<int>(shared.Format), static_cast<unsigned long>(hr));
          return XR_ERROR_RUNTIME_FAILURE;
        }
        images->shares.push_back(texture);
        HANDLE handle = nullptr;
        hr = device_->CreateSharedHandle(texture, nullptr, GENERIC_ALL, nullptr, &handle);
        if (FAILED(hr) || !mullionShareNameFromHandle(handle, share.images[i].name)) {
          // without Mullion's shared textures there is no way to hand the image to the server
          logLine("swapchain: this Direct3D 12 does not share textures the way Mullion does (0x%08lx); MetalVR needs "
                  "Mullion's Direct3D 12 with shared textures",
                  static_cast<unsigned long>(hr));
          return XR_ERROR_RUNTIME_FAILURE;
        }
      }
    }
    out = std::move(images);
    return XR_SUCCESS;
  }

  void finishFrame() override {
    // the queue reaches this value once everything submitted before it, the copies into the shares included, has
    // run; only then may another process read them
    const uint64_t value = ++fence_value_;
    if (FAILED(queue_->Signal(fence_, value)) || FAILED(fence_->SetEventOnCompletion(value, event_)))
      return;
    if (WaitForSingleObject(event_, 500) != WAIT_OBJECT_0)
      logLine("the GPU did not finish the frame within 500 ms; it is submitted as it is");
  }

  uint64_t markFrame() override {
    const uint64_t value = ++fence_value_;
    return SUCCEEDED(queue_->Signal(fence_, value)) ? value : 0;
  }

  bool waitMark(uint64_t mark, uint32_t timeout_ms) override {
    if (fence_->GetCompletedValue() >= mark)
      return true;
    return SUCCEEDED(fence_->SetEventOnCompletion(mark, mark_event_)) &&
           WaitForSingleObject(mark_event_, timeout_ms) == WAIT_OBJECT_0;
  }

private:
  ID3D12Device *device_ = nullptr;
  ID3D12CommandQueue *queue_ = nullptr;
  ID3D12Fence *fence_ = nullptr;
  uint64_t fence_value_ = 0;
  HANDLE event_ = nullptr;
  HANDLE mark_event_ = nullptr; // the submitter thread's, apart from finishFrame's
  std::vector<int64_t> formats_;
};

} // namespace

XrResult createD3D12Backend(const XrGraphicsBindingD3D12KHR &binding, std::unique_ptr<GraphicsBackend> &out) {
  if (!binding.device || !binding.queue)
    return XR_ERROR_GRAPHICS_DEVICE_INVALID;
  auto backend = std::make_unique<D3D12Backend>(binding.device, binding.queue);
  if (!backend->init())
    return XR_ERROR_RUNTIME_FAILURE;
  out = std::move(backend);
  return XR_SUCCESS;
}

namespace api {

MVR_API xrGetD3D12GraphicsRequirementsKHR(XrInstance handle, XrSystemId system_id,
                                          XrGraphicsRequirementsD3D12KHR *requirements) {
  MVR_INSTANCE(handle);
  if (!requirements || requirements->type != XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR)
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
