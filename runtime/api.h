// The OpenXR entry points this runtime implements, and the checks every one of them starts with.
#pragma once

#include <cstring>
#include <type_traits>

#include "runtime.h"

namespace metalvr {

constexpr XrSystemId kSystemId = 1;
constexpr uint32_t kMaxSwapchainSize = 8192;
#ifdef _WIN32
constexpr uint32_t kMaxSampleCount = 4; // multisampled images are resolved when they are released
#else
constexpr uint32_t kMaxSampleCount = 1;
#endif

#define MVR_API XrResult XRAPI_CALL

#define MVR_INSTANCE(handle)                                                                                           \
  Instance *instance = lookup<Instance>(handle);                                                                       \
  if (!instance)                                                                                                       \
    return XR_ERROR_HANDLE_INVALID;                                                                                    \
  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex)

#define MVR_SESSION(handle)                                                                                            \
  Session *session = lookup<Session>(handle);                                                                          \
  if (!session)                                                                                                        \
    return XR_ERROR_HANDLE_INVALID;                                                                                    \
  Instance *instance = session->instance;                                                                              \
  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex)

// for calls that cannot be answered without the server
#define MVR_ALIVE()                                                                                                    \
  if (instance->lost)                                                                                                  \
    return XR_ERROR_INSTANCE_LOST

#ifdef _WIN32
#define MVR_PLATFORM_FUNCTIONS(X)                                                                                      \
  X(xrGetD3D11GraphicsRequirementsKHR)                                                                                 \
  X(xrGetD3D12GraphicsRequirementsKHR)                                                                                 \
  X(xrConvertWin32PerformanceCounterToTimeKHR)                                                                         \
  X(xrConvertTimeToWin32PerformanceCounterKHR)
#else
#define MVR_PLATFORM_FUNCTIONS(X)                                                                                      \
  X(xrGetMetalGraphicsRequirementsKHR)                                                                                 \
  X(xrConvertTimespecTimeToTimeKHR)                                                                                    \
  X(xrConvertTimeToTimespecTimeKHR)
#endif

#define MVR_API_FUNCTIONS(X)                                                                                           \
  X(xrEnumerateApiLayerProperties)                                                                                     \
  X(xrEnumerateInstanceExtensionProperties)                                                                            \
  X(xrCreateInstance)                                                                                                  \
  X(xrDestroyInstance)                                                                                                 \
  X(xrGetInstanceProperties)                                                                                           \
  X(xrPollEvent)                                                                                                       \
  X(xrResultToString)                                                                                                  \
  X(xrStructureTypeToString)                                                                                           \
  X(xrGetSystem)                                                                                                       \
  X(xrGetSystemProperties)                                                                                             \
  X(xrEnumerateEnvironmentBlendModes)                                                                                  \
  X(xrEnumerateViewConfigurations)                                                                                     \
  X(xrGetViewConfigurationProperties)                                                                                  \
  X(xrEnumerateViewConfigurationViews)                                                                                 \
  X(xrStringToPath)                                                                                                    \
  X(xrPathToString)                                                                                                    \
  X(xrCreateSession)                                                                                                   \
  X(xrDestroySession)                                                                                                  \
  X(xrBeginSession)                                                                                                    \
  X(xrEndSession)                                                                                                      \
  X(xrRequestExitSession)                                                                                              \
  X(xrEnumerateReferenceSpaces)                                                                                        \
  X(xrCreateReferenceSpace)                                                                                            \
  X(xrGetReferenceSpaceBoundsRect)                                                                                     \
  X(xrCreateActionSpace)                                                                                               \
  X(xrLocateSpace)                                                                                                     \
  X(xrLocateSpaces)                                                                                                    \
  X(xrDestroySpace)                                                                                                    \
  X(xrEnumerateSwapchainFormats)                                                                                       \
  X(xrCreateSwapchain)                                                                                                 \
  X(xrDestroySwapchain)                                                                                                \
  X(xrEnumerateSwapchainImages)                                                                                        \
  X(xrAcquireSwapchainImage)                                                                                           \
  X(xrWaitSwapchainImage)                                                                                              \
  X(xrReleaseSwapchainImage)                                                                                           \
  X(xrWaitFrame)                                                                                                       \
  X(xrBeginFrame)                                                                                                      \
  X(xrEndFrame)                                                                                                        \
  X(xrLocateViews)                                                                                                     \
  X(xrGetVisibilityMaskKHR)                                                                                            \
  X(xrEnumerateDisplayRefreshRatesFB)                                                                                  \
  X(xrGetDisplayRefreshRateFB)                                                                                         \
  X(xrRequestDisplayRefreshRateFB)                                                                                     \
  X(xrCreateActionSet)                                                                                                 \
  X(xrDestroyActionSet)                                                                                                \
  X(xrCreateAction)                                                                                                    \
  X(xrDestroyAction)                                                                                                   \
  X(xrSuggestInteractionProfileBindings)                                                                               \
  X(xrAttachSessionActionSets)                                                                                         \
  X(xrGetCurrentInteractionProfile)                                                                                    \
  X(xrGetActionStateBoolean)                                                                                           \
  X(xrGetActionStateFloat)                                                                                             \
  X(xrGetActionStateVector2f)                                                                                          \
  X(xrGetActionStatePose)                                                                                              \
  X(xrSyncActions)                                                                                                     \
  X(xrEnumerateBoundSourcesForAction)                                                                                  \
  X(xrGetInputSourceLocalizedName)                                                                                     \
  X(xrApplyHapticFeedback)                                                                                             \
  X(xrStopHapticFeedback)                                                                                              \
  MVR_PLATFORM_FUNCTIONS(X)

// Each entry point is declared from the OpenXR header's own function-pointer type, so a definition whose
// signature drifts from the specification does not link.
namespace api {
#define MVR_DECLARE(name) std::remove_pointer_t<PFN_##name> name;
MVR_API_FUNCTIONS(MVR_DECLARE)
#undef MVR_DECLARE
} // namespace api

} // namespace metalvr
