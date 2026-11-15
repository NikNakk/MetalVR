// What the OpenXR loader sees: the negotiation entry point and xrGetInstanceProcAddr.
#include "api.h"

#include <openxr/openxr_loader_negotiation.h>

namespace metalvr {

namespace {

struct Entry {
  const char *name;
  PFN_xrVoidFunction function;
};

XRAPI_ATTR XrResult XRAPI_CALL instanceProcAddr(XrInstance instance, const char *name, PFN_xrVoidFunction *function);

const Entry kEntries[] = {
#define MVR_ENTRY(name) {#name, reinterpret_cast<PFN_xrVoidFunction>(static_cast<PFN_##name>(&api::name))},
    MVR_API_FUNCTIONS(MVR_ENTRY)
#undef MVR_ENTRY
    {"xrGetInstanceProcAddr", reinterpret_cast<PFN_xrVoidFunction>(&instanceProcAddr)},
};

XRAPI_ATTR XrResult XRAPI_CALL instanceProcAddr(XrInstance instance, const char *name, PFN_xrVoidFunction *function) {
  if (!name || !function)
    return XR_ERROR_VALIDATION_FAILURE;
  *function = nullptr;
  if (instance == XR_NULL_HANDLE) {
    // without an instance only the functions that lead to one may be asked for
    if (std::strcmp(name, "xrEnumerateInstanceExtensionProperties") && std::strcmp(name, "xrCreateInstance") &&
        std::strcmp(name, "xrEnumerateApiLayerProperties") && std::strcmp(name, "xrGetInstanceProcAddr"))
      return XR_ERROR_HANDLE_INVALID;
  } else if (!lookup<Instance>(instance)) {
    return XR_ERROR_HANDLE_INVALID;
  }
  return getProcAddr(name, function);
}

} // namespace

XrResult getProcAddr(const char *name, PFN_xrVoidFunction *function) {
  for (const Entry &entry : kEntries)
    if (!std::strcmp(entry.name, name)) {
      *function = entry.function;
      return XR_SUCCESS;
    }
  *function = nullptr;
  return XR_ERROR_FUNCTION_UNSUPPORTED;
}

} // namespace metalvr

extern "C" {

#ifdef _WIN32
__declspec(dllexport)
#else
__attribute__((visibility("default")))
#endif
XrResult XRAPI_CALL
xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo *loader, XrNegotiateRuntimeRequest *runtime) {
  if (!loader || !runtime || loader->structType != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO ||
      loader->structVersion != XR_LOADER_INFO_STRUCT_VERSION || loader->structSize != sizeof(XrNegotiateLoaderInfo) ||
      runtime->structType != XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST ||
      runtime->structVersion != XR_RUNTIME_INFO_STRUCT_VERSION ||
      runtime->structSize != sizeof(XrNegotiateRuntimeRequest))
    return XR_ERROR_INITIALIZATION_FAILED;
  if (loader->minInterfaceVersion > XR_CURRENT_LOADER_RUNTIME_VERSION ||
      loader->maxInterfaceVersion < XR_CURRENT_LOADER_RUNTIME_VERSION)
    return XR_ERROR_INITIALIZATION_FAILED;
  // the loader must be able to speak a version this runtime implements: 1.0 or 1.1
  const XrVersion ours_min = XR_MAKE_VERSION(1, 0, 0);
  const XrVersion ours_max = XR_MAKE_VERSION(1, XR_VERSION_MINOR(XR_CURRENT_API_VERSION), 0xfff);
  if (loader->minApiVersion > ours_max || loader->maxApiVersion < ours_min)
    return XR_ERROR_INITIALIZATION_FAILED;
  runtime->runtimeInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
  runtime->runtimeApiVersion = XR_CURRENT_API_VERSION;
  runtime->getInstanceProcAddr = &metalvr::instanceProcAddr;
  return XR_SUCCESS;
}

} // extern "C"
