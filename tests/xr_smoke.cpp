// An OpenXR application that exercises a whole session against the MetalVR runtime and checks what it sees.
//
// It loads the runtime library directly and negotiates with it the way the OpenXR loader does, so it tests the
// runtime and nothing else. Each eye is cleared to its own color with a moving checkerboard, and a quad layer
// floats in front of the viewer; with `metalvrd --driver sim --snapshot` the composed result can be looked at.
//
//   xr_smoke [--runtime PATH | --loader PATH] [--frames N] [--array] [--samples N] [--unorm] [--no-quad]
//            [--require-position-tracked]
//            [--format N] [--flip] [--expect-profile PATH]
//
// --format asks for one swapchain format by its number (a DXGI_FORMAT or MTLPixelFormat); --flip submits each
// eye with a negative image height, the way an OpenVR game's bottom-up texture arrives. Bindings are suggested
// for Touch and Vive controllers; --expect-profile names the one the runtime must choose (Touch by default, the
// other with METALVR_INTERACTION_PROFILE set).
//
// With --loader it goes through the Khronos loader library at PATH instead, which finds the runtime the way a
// game's loader does: the active-runtime manifest (the registry on Windows, XR_RUNTIME_JSON or the XDG path on
// macOS).
//
// Exit status 0 means every check passed.
#include "gfx.h"

#include <openxr/openxr_loader_negotiation.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#define DEFAULT_RUNTIME "metalvr_openxr.dll"
static void *loadLibrary(const char *path) { return LoadLibraryA(path); }
static void *findSymbol(void *library, const char *name) {
  return reinterpret_cast<void *>(GetProcAddress(static_cast<HMODULE>(library), name));
}
#else
#include <dlfcn.h>
#define DEFAULT_RUNTIME "libmetalvr_openxr.dylib"
static void *loadLibrary(const char *path) { return dlopen(path, RTLD_NOW | RTLD_LOCAL); }
static void *findSymbol(void *library, const char *name) { return dlsym(library, name); }
#endif

#define XR_FUNCTIONS(X)                                                                                                \
  X(xrEnumerateInstanceExtensionProperties)                                                                            \
  X(xrCreateInstance)                                                                                                  \
  X(xrDestroyInstance)                                                                                                 \
  X(xrGetInstanceProperties)                                                                                           \
  X(xrPollEvent)                                                                                                       \
  X(xrGetSystem)                                                                                                       \
  X(xrGetSystemProperties)                                                                                             \
  X(xrEnumerateViewConfigurationViews)                                                                                 \
  X(xrCreateSession)                                                                                                   \
  X(xrDestroySession)                                                                                                  \
  X(xrBeginSession)                                                                                                    \
  X(xrEndSession)                                                                                                      \
  X(xrRequestExitSession)                                                                                              \
  X(xrCreateReferenceSpace)                                                                                            \
  X(xrCreateActionSpace)                                                                                               \
  X(xrLocateSpace)                                                                                                     \
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
  X(xrStringToPath)                                                                                                    \
  X(xrPathToString)                                                                                                    \
  X(xrCreateActionSet)                                                                                                 \
  X(xrCreateAction)                                                                                                    \
  X(xrSuggestInteractionProfileBindings)                                                                               \
  X(xrAttachSessionActionSets)                                                                                         \
  X(xrGetCurrentInteractionProfile)                                                                                    \
  X(xrGetActionStateFloat)                                                                                             \
  X(xrGetActionStateBoolean)                                                                                           \
  X(xrGetActionStatePose)                                                                                              \
  X(xrSyncActions)                                                                                                     \
  X(xrApplyHapticFeedback)

#define DECLARE(name) static PFN_##name name;
XR_FUNCTIONS(DECLARE)
#undef DECLARE

static int g_failures = 0;

#define CHECK(condition, ...)                                                                                          \
  do {                                                                                                                 \
    if (!(condition)) {                                                                                                \
      ++g_failures;                                                                                                    \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__);                                                                 \
      std::printf(__VA_ARGS__);                                                                                        \
      std::printf("\n");                                                                                               \
    }                                                                                                                  \
  } while (0)

#define REQUIRE_XR(call)                                                                                               \
  do {                                                                                                                 \
    const XrResult result_ = (call);                                                                                   \
    if (XR_FAILED(result_)) {                                                                                          \
      std::printf("FATAL %s:%d: %s -> %d\n", __FILE__, __LINE__, #call, static_cast<int>(result_));                    \
      std::fflush(stdout);                                                                                             \
      return 1;                                                                                                        \
    }                                                                                                                  \
  } while (0)

struct EyeChain {
  XrSwapchain swapchain = XR_NULL_HANDLE;
  int id = 0;
  uint32_t slice = 0;
};

static XrPath toPath(XrInstance instance, const char *string) {
  XrPath path = XR_NULL_PATH;
  xrStringToPath(instance, string, &path);
  return path;
}

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::string runtime_path = DEFAULT_RUNTIME;
  if (const char *env = std::getenv("METALVR_RUNTIME"))
    runtime_path = env;
  std::string loader_path;
  std::string expect_profile = "/interaction_profiles/oculus/touch_controller";
  uint32_t frames_wanted = 120;
  bool use_array = false, srgb = true, use_quad = true, flip = false, require_position_tracked = false;
  int64_t format_wanted = 0;
  uint32_t samples = 1;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--runtime" && i + 1 < argc)
      runtime_path = argv[++i];
    else if (arg == "--loader" && i + 1 < argc)
      loader_path = argv[++i];
    else if (arg == "--frames" && i + 1 < argc)
      frames_wanted = static_cast<uint32_t>(std::atoi(argv[++i]));
    else if (arg == "--array")
      use_array = true;
    else if (arg == "--samples" && i + 1 < argc)
      samples = static_cast<uint32_t>(std::atoi(argv[++i]));
    else if (arg == "--unorm")
      srgb = false;
    else if (arg == "--no-quad")
      use_quad = false;
    else if (arg == "--require-position-tracked")
      require_position_tracked = true;
    else if (arg == "--flip")
      flip = true;
    else if (arg == "--expect-profile" && i + 1 < argc)
      expect_profile = argv[++i];
    else if (arg == "--format" && i + 1 < argc)
      format_wanted = std::atoll(argv[++i]);
    else {
      std::printf("unknown argument %s\n", arg.c_str());
      return 2;
    }
  }

  PFN_xrGetInstanceProcAddr get_proc = nullptr;
  if (!loader_path.empty()) {
    // ---- through the Khronos loader, which reads the active runtime's manifest ---------------------------------
    void *loader = loadLibrary(loader_path.c_str());
    if (!loader) {
      std::printf("FATAL cannot load the OpenXR loader %s\n", loader_path.c_str());
      return 1;
    }
    get_proc = reinterpret_cast<PFN_xrGetInstanceProcAddr>(findSymbol(loader, "xrGetInstanceProcAddr"));
    xrCreateInstance = reinterpret_cast<PFN_xrCreateInstance>(findSymbol(loader, "xrCreateInstance"));
    xrEnumerateInstanceExtensionProperties = reinterpret_cast<PFN_xrEnumerateInstanceExtensionProperties>(
        findSymbol(loader, "xrEnumerateInstanceExtensionProperties"));
    if (!get_proc || !xrCreateInstance || !xrEnumerateInstanceExtensionProperties) {
      std::printf("FATAL %s is not an OpenXR loader\n", loader_path.c_str());
      return 1;
    }
    std::printf("using the OpenXR loader %s\n", loader_path.c_str());
  } else {
    // ---- the loader's part: negotiate with the runtime --------------------------------------------------------
    void *library = loadLibrary(runtime_path.c_str());
    if (!library) {
      std::printf("FATAL cannot load runtime %s\n", runtime_path.c_str());
      return 1;
    }
    using Negotiate = XrResult(XRAPI_CALL *)(const XrNegotiateLoaderInfo *, XrNegotiateRuntimeRequest *);
    auto negotiate = reinterpret_cast<Negotiate>(findSymbol(library, "xrNegotiateLoaderRuntimeInterface"));
    if (!negotiate) {
      std::printf("FATAL the runtime does not export xrNegotiateLoaderRuntimeInterface\n");
      return 1;
    }
    XrNegotiateLoaderInfo loader_info{};
    loader_info.structType = XR_LOADER_INTERFACE_STRUCT_LOADER_INFO;
    loader_info.structVersion = XR_LOADER_INFO_STRUCT_VERSION;
    loader_info.structSize = sizeof(loader_info);
    loader_info.minInterfaceVersion = 1;
    loader_info.maxInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
    loader_info.minApiVersion = XR_MAKE_VERSION(1, 0, 0);
    loader_info.maxApiVersion = XR_MAKE_VERSION(1, 0x3ff, 0xfff);
    XrNegotiateRuntimeRequest runtime_request{};
    runtime_request.structType = XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST;
    runtime_request.structVersion = XR_RUNTIME_INFO_STRUCT_VERSION;
    runtime_request.structSize = sizeof(runtime_request);
    REQUIRE_XR(negotiate(&loader_info, &runtime_request));
    get_proc = runtime_request.getInstanceProcAddr;
    REQUIRE_XR(
        get_proc(XR_NULL_HANDLE, "xrCreateInstance", reinterpret_cast<PFN_xrVoidFunction *>(&xrCreateInstance)));
    REQUIRE_XR(get_proc(XR_NULL_HANDLE, "xrEnumerateInstanceExtensionProperties",
                        reinterpret_cast<PFN_xrVoidFunction *>(&xrEnumerateInstanceExtensionProperties)));
    PFN_xrVoidFunction denied = nullptr;
    CHECK(get_proc(XR_NULL_HANDLE, "xrCreateSession", &denied) == XR_ERROR_HANDLE_INVALID,
          "a session function was handed out without an instance");
  }

  TestGraphics *gfx = createTestGraphics();

  uint32_t extension_count = 0;
  REQUIRE_XR(xrEnumerateInstanceExtensionProperties(nullptr, 0, &extension_count, nullptr));
  std::vector<XrExtensionProperties> extensions(extension_count, {XR_TYPE_EXTENSION_PROPERTIES});
  REQUIRE_XR(xrEnumerateInstanceExtensionProperties(nullptr, extension_count, &extension_count, extensions.data()));
  bool has_graphics = false, has_hands = false;
  for (const auto &extension : extensions) {
    has_graphics = has_graphics || !std::strcmp(extension.extensionName, gfx->extensionName());
    has_hands = has_hands || !std::strcmp(extension.extensionName, XR_EXT_HAND_TRACKING_EXTENSION_NAME);
  }
  CHECK(has_graphics, "the runtime does not offer %s", gfx->extensionName());

  // ---- instance and system ------------------------------------------------------------------------------------
  // hand tracking, when the runtime offers it: the simulated headset holds open hands where its controllers are
  const char *enabled[] = {gfx->extensionName(), XR_EXT_HAND_TRACKING_EXTENSION_NAME};
  XrInstanceCreateInfo instance_info{XR_TYPE_INSTANCE_CREATE_INFO};
  std::snprintf(instance_info.applicationInfo.applicationName, XR_MAX_APPLICATION_NAME_SIZE, "xr_smoke");
  instance_info.applicationInfo.apiVersion = XR_API_VERSION_1_0;
  instance_info.enabledExtensionCount = has_hands ? 2 : 1;
  instance_info.enabledExtensionNames = enabled;
  XrInstance instance = XR_NULL_HANDLE;
  REQUIRE_XR(xrCreateInstance(&instance_info, &instance));
#define LOAD(name) REQUIRE_XR(get_proc(instance, #name, reinterpret_cast<PFN_xrVoidFunction *>(&name)));
  XR_FUNCTIONS(LOAD)
#undef LOAD

  XrInstanceProperties instance_properties{XR_TYPE_INSTANCE_PROPERTIES};
  REQUIRE_XR(xrGetInstanceProperties(instance, &instance_properties));
  std::printf("runtime: %s %u.%u.%u\n", instance_properties.runtimeName,
              XR_VERSION_MAJOR(instance_properties.runtimeVersion),
              XR_VERSION_MINOR(instance_properties.runtimeVersion),
              XR_VERSION_PATCH(instance_properties.runtimeVersion));

  XrSystemGetInfo system_info{XR_TYPE_SYSTEM_GET_INFO};
  system_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  XrSystemId system = XR_NULL_SYSTEM_ID;
  REQUIRE_XR(xrGetSystem(instance, &system_info, &system));
  XrSystemProperties system_properties{XR_TYPE_SYSTEM_PROPERTIES};
  XrSystemHandTrackingPropertiesEXT hand_properties{XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
  if (has_hands)
    system_properties.next = &hand_properties;
  REQUIRE_XR(xrGetSystemProperties(instance, system, &system_properties));
  std::printf("system: %s%s\n", system_properties.systemName,
              hand_properties.supportsHandTracking ? " (tracks hands)" : "");
  PFN_xrCreateHandTrackerEXT xrCreateHandTrackerEXT = nullptr;
  PFN_xrDestroyHandTrackerEXT xrDestroyHandTrackerEXT = nullptr;
  PFN_xrLocateHandJointsEXT xrLocateHandJointsEXT = nullptr;
  const bool hands_wanted = has_hands && hand_properties.supportsHandTracking;
  if (hands_wanted) {
    REQUIRE_XR(get_proc(instance, "xrCreateHandTrackerEXT", reinterpret_cast<PFN_xrVoidFunction *>(&xrCreateHandTrackerEXT)));
    REQUIRE_XR(get_proc(instance, "xrDestroyHandTrackerEXT", reinterpret_cast<PFN_xrVoidFunction *>(&xrDestroyHandTrackerEXT)));
    REQUIRE_XR(get_proc(instance, "xrLocateHandJointsEXT", reinterpret_cast<PFN_xrVoidFunction *>(&xrLocateHandJointsEXT)));
  }

  uint32_t view_count = 0;
  XrViewConfigurationView config_views[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
  REQUIRE_XR(xrEnumerateViewConfigurationViews(instance, system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2,
                                               &view_count, config_views));
  const uint32_t width = config_views[0].recommendedImageRectWidth;
  const uint32_t height = config_views[0].recommendedImageRectHeight;
  std::printf("views: %u, %ux%u per eye\n", view_count, width, height);
  CHECK(view_count == 2 && width && height, "bad view configuration");

  // ---- session ------------------------------------------------------------------------------------------------
  {
    // creating a session before asking for the graphics requirements is an error the runtime must catch
    XrSessionCreateInfo early{XR_TYPE_SESSION_CREATE_INFO};
    early.systemId = system;
    XrSession none = XR_NULL_HANDLE;
    CHECK(xrCreateSession(instance, &early, &none) == XR_ERROR_GRAPHICS_REQUIREMENTS_CALL_MISSING,
          "xrCreateSession did not insist on the graphics requirements call");
  }
  if (!gfx->init(instance, system, get_proc)) {
    std::printf("FATAL graphics initialization failed\n");
    return 1;
  }
  XrSessionCreateInfo session_info{XR_TYPE_SESSION_CREATE_INFO};
  session_info.next = gfx->binding();
  session_info.systemId = system;
  XrSession session = XR_NULL_HANDLE;
  REQUIRE_XR(xrCreateSession(instance, &session_info, &session));

  XrReferenceSpaceCreateInfo space_info{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
  space_info.poseInReferenceSpace.orientation.w = 1.0f;
  XrSpace local = XR_NULL_HANDLE, view_space = XR_NULL_HANDLE, stage = XR_NULL_HANDLE;
  space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
  REQUIRE_XR(xrCreateReferenceSpace(session, &space_info, &local));
  space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
  REQUIRE_XR(xrCreateReferenceSpace(session, &space_info, &view_space));
  space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
  REQUIRE_XR(xrCreateReferenceSpace(session, &space_info, &stage));

  // ---- actions ------------------------------------------------------------------------------------------------
  const XrPath hands[2] = {toPath(instance, "/user/hand/left"), toPath(instance, "/user/hand/right")};
  XrActionSetCreateInfo set_info{XR_TYPE_ACTION_SET_CREATE_INFO};
  std::snprintf(set_info.actionSetName, XR_MAX_ACTION_SET_NAME_SIZE, "smoke");
  std::snprintf(set_info.localizedActionSetName, XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE, "Smoke");
  XrActionSet action_set = XR_NULL_HANDLE;
  REQUIRE_XR(xrCreateActionSet(instance, &set_info, &action_set));
  const auto makeAction = [&](const char *name, XrActionType type, XrAction *out) {
    XrActionCreateInfo info{XR_TYPE_ACTION_CREATE_INFO};
    info.actionType = type;
    std::snprintf(info.actionName, XR_MAX_ACTION_NAME_SIZE, "%s", name);
    std::snprintf(info.localizedActionName, XR_MAX_LOCALIZED_ACTION_NAME_SIZE, "%s", name);
    info.countSubactionPaths = 2;
    info.subactionPaths = hands;
    return xrCreateAction(action_set, &info, out);
  };
  XrAction trigger = XR_NULL_HANDLE, primary = XR_NULL_HANDLE, grip = XR_NULL_HANDLE, vibrate = XR_NULL_HANDLE;
  REQUIRE_XR(makeAction("trigger", XR_ACTION_TYPE_FLOAT_INPUT, &trigger));
  REQUIRE_XR(makeAction("primary", XR_ACTION_TYPE_BOOLEAN_INPUT, &primary));
  REQUIRE_XR(makeAction("grip", XR_ACTION_TYPE_POSE_INPUT, &grip));
  REQUIRE_XR(makeAction("vibrate", XR_ACTION_TYPE_VIBRATION_OUTPUT, &vibrate));
  const XrActionSuggestedBinding bindings[] = {
      {trigger, toPath(instance, "/user/hand/left/input/trigger/value")},
      {trigger, toPath(instance, "/user/hand/right/input/trigger/value")},
      {primary, toPath(instance, "/user/hand/left/input/x/click")},
      {primary, toPath(instance, "/user/hand/right/input/a/click")},
      {grip, toPath(instance, "/user/hand/left/input/grip/pose")},
      {grip, toPath(instance, "/user/hand/right/input/grip/pose")},
      {vibrate, toPath(instance, "/user/hand/left/output/haptic")},
      {vibrate, toPath(instance, "/user/hand/right/output/haptic")},
  };
  XrInteractionProfileSuggestedBinding suggested{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
  suggested.interactionProfile = toPath(instance, "/interaction_profiles/oculus/touch_controller");
  suggested.countSuggestedBindings = static_cast<uint32_t>(sizeof(bindings) / sizeof(bindings[0]));
  suggested.suggestedBindings = bindings;
  REQUIRE_XR(xrSuggestInteractionProfileBindings(instance, &suggested));
  // a second controller's bindings: the runtime picks one profile, by its own order unless told otherwise
  const XrActionSuggestedBinding vive_bindings[] = {
      {trigger, toPath(instance, "/user/hand/left/input/trigger/value")},
      {trigger, toPath(instance, "/user/hand/right/input/trigger/value")},
      {primary, toPath(instance, "/user/hand/left/input/menu/click")},
      {primary, toPath(instance, "/user/hand/right/input/menu/click")},
      {grip, toPath(instance, "/user/hand/left/input/grip/pose")},
      {grip, toPath(instance, "/user/hand/right/input/grip/pose")},
      {vibrate, toPath(instance, "/user/hand/left/output/haptic")},
      {vibrate, toPath(instance, "/user/hand/right/output/haptic")},
  };
  suggested.interactionProfile = toPath(instance, "/interaction_profiles/htc/vive_controller");
  suggested.countSuggestedBindings = static_cast<uint32_t>(sizeof(vive_bindings) / sizeof(vive_bindings[0]));
  suggested.suggestedBindings = vive_bindings;
  REQUIRE_XR(xrSuggestInteractionProfileBindings(instance, &suggested));
  XrSessionActionSetsAttachInfo attach{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
  attach.countActionSets = 1;
  attach.actionSets = &action_set;
  REQUIRE_XR(xrAttachSessionActionSets(session, &attach));
  XrHandTrackerEXT hand_trackers[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
  for (int hand = 0; hand < 2 && hands_wanted; ++hand) {
    XrHandTrackerCreateInfoEXT info{XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
    info.hand = hand == 0 ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT;
    info.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
    REQUIRE_XR(xrCreateHandTrackerEXT(session, &info, &hand_trackers[hand]));
  }
  uint32_t hands_seen[2] = {0, 0};
  float worst_wrist = 0;
  XrSpace grip_space[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
  for (int hand = 0; hand < 2; ++hand) {
    XrActionSpaceCreateInfo info{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    info.action = grip;
    info.subactionPath = hands[hand];
    info.poseInActionSpace.orientation.w = 1.0f;
    REQUIRE_XR(xrCreateActionSpace(session, &info, &grip_space[hand]));
  }

  // ---- swapchains ---------------------------------------------------------------------------------------------
  uint32_t format_count = 0;
  REQUIRE_XR(xrEnumerateSwapchainFormats(session, 0, &format_count, nullptr));
  std::vector<int64_t> formats(format_count);
  REQUIRE_XR(xrEnumerateSwapchainFormats(session, format_count, &format_count, formats.data()));
  int64_t format = 0;
  if (format_wanted) {
    for (int64_t offered : formats)
      if (offered == format_wanted)
        format = offered;
  } else {
    format = gfx->chooseFormat(formats, srgb);
  }
  if (!format) {
    std::printf("FATAL no usable swapchain format (the runtime offers");
    for (int64_t offered : formats)
      std::printf(" %lld", static_cast<long long>(offered));
    std::printf(")\n");
    return 1;
  }

  EyeChain eyes[2];
  const auto createChain = [&](int id, uint32_t w, uint32_t h, uint32_t array_size, uint32_t sample_count,
                               XrSwapchain *out) -> XrResult {
    XrSwapchainCreateInfo info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    info.format = format;
    info.sampleCount = sample_count;
    info.width = w;
    info.height = h;
    info.faceCount = 1;
    info.arraySize = array_size;
    info.mipCount = 1;
    const XrResult result = xrCreateSwapchain(session, &info, out);
    if (XR_FAILED(result))
      return result;
    return gfx->adopt(id, *out, xrEnumerateSwapchainImages, w, h, format, array_size, sample_count)
               ? XR_SUCCESS
               : XR_ERROR_RUNTIME_FAILURE;
  };
  if (use_array) {
    XrSwapchain both = XR_NULL_HANDLE;
    REQUIRE_XR(createChain(0, width, height, 2, samples, &both));
    eyes[0] = {both, 0, 0};
    eyes[1] = {both, 0, 1};
  } else {
    for (int eye = 0; eye < 2; ++eye) {
      REQUIRE_XR(createChain(eye, width, height, 1, samples, &eyes[eye].swapchain));
      eyes[eye].id = eye;
    }
  }
  XrSwapchain quad_chain = XR_NULL_HANDLE;
  const uint32_t quad_size = 512;
  if (use_quad)
    REQUIRE_XR(createChain(2, quad_size, quad_size, 1, 1, &quad_chain));
  std::printf("swapchains: %s, format %lld, %u sample(s)%s\n", use_array ? "one array of two" : "one per eye",
              static_cast<long long>(format), samples, use_quad ? ", plus a quad layer" : "");

  // ---- the frame loop -----------------------------------------------------------------------------------------
  std::vector<XrSessionState> states;
  bool running = false, done = false, exit_requested = false, profile_changed = false;
  uint32_t rendered = 0, waited = 0;
  float trigger_min = 1e9f, trigger_max = -1e9f;
  bool primary_seen[2] = {false, false};
  uint32_t grip_valid[2] = {0, 0}, grip_tracked[2] = {0, 0}, views_valid = 0, haptics_ok = 0;
  XrPosef last_views[2]{};
  XrFovf last_fov[2]{};
  XrPosef last_grip[2]{};
  XrPosef head_in_stage{};
  float head_min_x = 1e9f, head_max_x = -1e9f;

  for (int guard = 0; !done && guard < 100000; ++guard) {
    XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(instance, &event) == XR_SUCCESS) {
      if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
        const auto *changed = reinterpret_cast<const XrEventDataSessionStateChanged *>(&event);
        states.push_back(changed->state);
        if (changed->state == XR_SESSION_STATE_READY) {
          XrSessionBeginInfo begin{XR_TYPE_SESSION_BEGIN_INFO};
          begin.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
          REQUIRE_XR(xrBeginSession(session, &begin));
          running = true;
        } else if (changed->state == XR_SESSION_STATE_STOPPING) {
          REQUIRE_XR(xrEndSession(session));
          running = false;
        } else if (changed->state == XR_SESSION_STATE_EXITING || changed->state == XR_SESSION_STATE_LOSS_PENDING) {
          done = true;
        }
      } else if (event.type == XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED) {
        profile_changed = true;
      } else if (event.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
        std::printf("FATAL the runtime lost its server\n");
        return 1;
      }
      event = {XR_TYPE_EVENT_DATA_BUFFER};
    }
    if (!running) {
      if (waited > 100000)
        break;
#ifdef _WIN32
      Sleep(5);
#else
      struct timespec nap {0, 5'000'000};
      nanosleep(&nap, nullptr);
#endif
      if (++waited > 2000) { // ten seconds without a headset
        std::printf("FATAL the session never became ready\n");
        return 1;
      }
      continue;
    }

    XrFrameState frame_state{XR_TYPE_FRAME_STATE};
    REQUIRE_XR(xrWaitFrame(session, nullptr, &frame_state));
    REQUIRE_XR(xrBeginFrame(session, nullptr));

    XrCompositionLayerProjectionView projection_views[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
                                                            {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    XrCompositionLayerProjection projection{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    XrCompositionLayerQuad quad{XR_TYPE_COMPOSITION_LAYER_QUAD};
    std::vector<const XrCompositionLayerBaseHeader *> layers;

    if (frame_state.shouldRender) {
      XrActiveActionSet active{action_set, XR_NULL_PATH};
      XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
      sync.countActiveActionSets = 1;
      sync.activeActionSets = &active;
      const XrResult sync_result = xrSyncActions(session, &sync);
      CHECK(XR_SUCCEEDED(sync_result), "xrSyncActions -> %d", static_cast<int>(sync_result));
      if (sync_result == XR_SUCCESS) {
        XrActionStateGetInfo get{XR_TYPE_ACTION_STATE_GET_INFO};
        get.action = trigger;
        get.subactionPath = hands[1];
        XrActionStateFloat value{XR_TYPE_ACTION_STATE_FLOAT};
        if (XR_SUCCEEDED(xrGetActionStateFloat(session, &get, &value)) && value.isActive) {
          trigger_min = std::fmin(trigger_min, value.currentState);
          trigger_max = std::fmax(trigger_max, value.currentState);
        }
        get.action = primary;
        get.subactionPath = hands[0];
        XrActionStateBoolean pressed{XR_TYPE_ACTION_STATE_BOOLEAN};
        if (XR_SUCCEEDED(xrGetActionStateBoolean(session, &get, &pressed)) && pressed.isActive)
          primary_seen[pressed.currentState ? 1 : 0] = true;
        for (int hand = 0; hand < 2; ++hand) {
          XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
          if (XR_SUCCEEDED(xrLocateSpace(grip_space[hand], local, frame_state.predictedDisplayTime, &location)) &&
              (location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)) {
            ++grip_valid[hand];
            constexpr auto tracked = XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
            if ((location.locationFlags & tracked) == tracked)
              ++grip_tracked[hand];
            last_grip[hand] = location.pose;
            if (hand_trackers[hand]) {
              // the hand's wrist is where its controller's grip is (the simulated headset's hands)
              XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT];
              XrHandJointLocationsEXT located{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
              located.jointCount = XR_HAND_JOINT_COUNT_EXT;
              located.jointLocations = joints;
              XrHandJointsLocateInfoEXT where{XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
              where.baseSpace = local;
              where.time = frame_state.predictedDisplayTime;
              REQUIRE_XR(xrLocateHandJointsEXT(hand_trackers[hand], &where, &located));
              if (located.isActive && (joints[XR_HAND_JOINT_WRIST_EXT].locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)) {
                ++hands_seen[hand];
                const XrVector3f w = joints[XR_HAND_JOINT_WRIST_EXT].pose.position;
                const float d = std::sqrt((w.x - location.pose.position.x) * (w.x - location.pose.position.x) +
                                          (w.y - location.pose.position.y) * (w.y - location.pose.position.y) +
                                          (w.z - location.pose.position.z) * (w.z - location.pose.position.z));
                worst_wrist = std::fmax(worst_wrist, d);
              }
            }
          }
        }
        if (rendered == 10) {
          XrHapticActionInfo haptic_info{XR_TYPE_HAPTIC_ACTION_INFO};
          haptic_info.action = vibrate;
          haptic_info.subactionPath = hands[1];
          XrHapticVibration vibration{XR_TYPE_HAPTIC_VIBRATION};
          vibration.duration = 50'000'000;
          vibration.frequency = 160.0f;
          vibration.amplitude = 0.5f;
          if (xrApplyHapticFeedback(session, &haptic_info,
                                    reinterpret_cast<const XrHapticBaseHeader *>(&vibration)) == XR_SUCCESS)
            ++haptics_ok;
        }
      }

      XrViewLocateInfo locate{XR_TYPE_VIEW_LOCATE_INFO};
      locate.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
      locate.displayTime = frame_state.predictedDisplayTime;
      locate.space = local;
      XrViewState view_state{XR_TYPE_VIEW_STATE};
      XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
      uint32_t located = 0;
      REQUIRE_XR(xrLocateViews(session, &locate, &view_state, 2, &located, views));
      if ((view_state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) &&
          (view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT))
        ++views_valid;
      XrSpaceLocation head{XR_TYPE_SPACE_LOCATION};
      if (XR_SUCCEEDED(xrLocateSpace(view_space, stage, frame_state.predictedDisplayTime, &head))) {
        head_in_stage = head.pose;
        head_min_x = std::fmin(head_min_x, head.pose.orientation.y);
        head_max_x = std::fmax(head_max_x, head.pose.orientation.y);
      }

      // left eye red, right eye blue, so a swapped or duplicated eye is obvious in a snapshot
      const float colors[2][4] = {{0.75f, 0.10f, 0.10f, 1.0f}, {0.10f, 0.10f, 0.75f, 1.0f}};
      uint32_t image_index[3] = {0, 0, 0};
      XrSwapchainImageWaitInfo wait_info{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
      wait_info.timeout = XR_INFINITE_DURATION;
      const int chains = use_array ? 1 : 2;
      for (int chain = 0; chain < chains; ++chain) {
        REQUIRE_XR(xrAcquireSwapchainImage(eyes[chain].swapchain, nullptr, &image_index[chain]));
        REQUIRE_XR(xrWaitSwapchainImage(eyes[chain].swapchain, &wait_info));
      }
      for (int eye = 0; eye < 2; ++eye)
        gfx->draw(eyes[eye].id, image_index[use_array ? 0 : eye], eyes[eye].slice, colors[eye], rendered);
      if (use_quad) {
        REQUIRE_XR(xrAcquireSwapchainImage(quad_chain, nullptr, &image_index[2]));
        REQUIRE_XR(xrWaitSwapchainImage(quad_chain, &wait_info));
        const float yellow[4] = {0.9f, 0.8f, 0.1f, 1.0f};
        gfx->draw(2, image_index[2], 0, yellow, rendered);
      }
      gfx->submit();
      for (int chain = 0; chain < chains; ++chain)
        REQUIRE_XR(xrReleaseSwapchainImage(eyes[chain].swapchain, nullptr));
      if (use_quad)
        REQUIRE_XR(xrReleaseSwapchainImage(quad_chain, nullptr));

      for (int eye = 0; eye < 2; ++eye) {
        last_views[eye] = views[eye].pose;
        last_fov[eye] = views[eye].fov;
        projection_views[eye].pose = views[eye].pose;
        projection_views[eye].fov = views[eye].fov;
        projection_views[eye].subImage.swapchain = eyes[eye].swapchain;
        projection_views[eye].subImage.imageRect = {{0, 0}, {static_cast<int32_t>(width), static_cast<int32_t>(height)}};
        if (flip) // the same rows, bottom row first
          projection_views[eye].subImage.imageRect = {{0, static_cast<int32_t>(height)},
                                                      {static_cast<int32_t>(width), -static_cast<int32_t>(height)}};
        projection_views[eye].subImage.imageArrayIndex = eyes[eye].slice;
      }
      projection.space = local;
      projection.viewCount = 2;
      projection.views = projection_views;
      layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader *>(&projection));
      if (use_quad) {
        // one meter wide, a meter and a half ahead of where the head started, turned a little
        quad.space = local;
        quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        quad.subImage.swapchain = quad_chain;
        quad.subImage.imageRect = {{0, 0}, {static_cast<int32_t>(quad_size), static_cast<int32_t>(quad_size)}};
        quad.pose.orientation = {0.0f, std::sin(0.2f), 0.0f, std::cos(0.2f)};
        quad.pose.position = {0.0f, 0.0f, -1.5f};
        quad.size = {1.0f, 1.0f};
        layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader *>(&quad));
      }
      ++rendered;
    }

    XrFrameEndInfo end{XR_TYPE_FRAME_END_INFO};
    end.displayTime = frame_state.predictedDisplayTime;
    end.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    end.layerCount = static_cast<uint32_t>(layers.size());
    end.layers = layers.data();
    REQUIRE_XR(xrEndFrame(session, &end));

    if (rendered >= frames_wanted && !exit_requested) {
      REQUIRE_XR(xrRequestExitSession(session));
      exit_requested = true;
    }
  }

  // ---- what was seen ------------------------------------------------------------------------------------------
  std::printf("states:");
  for (XrSessionState state : states)
    std::printf(" %d", static_cast<int>(state));
  std::printf("\n");
  const XrSessionState expected[] = {XR_SESSION_STATE_IDLE,         XR_SESSION_STATE_READY,
                                     XR_SESSION_STATE_SYNCHRONIZED, XR_SESSION_STATE_VISIBLE,
                                     XR_SESSION_STATE_FOCUSED,      XR_SESSION_STATE_VISIBLE,
                                     XR_SESSION_STATE_SYNCHRONIZED, XR_SESSION_STATE_STOPPING,
                                     XR_SESSION_STATE_IDLE,         XR_SESSION_STATE_EXITING};
  const size_t expected_count = sizeof(expected) / sizeof(expected[0]);
  bool sequence_ok = states.size() == expected_count;
  for (size_t i = 0; sequence_ok && i < expected_count; ++i)
    sequence_ok = states[i] == expected[i];
  CHECK(sequence_ok, "the session did not walk IDLE..FOCUSED..EXITING in order");
  CHECK(rendered >= frames_wanted, "rendered %u of %u frames", rendered, frames_wanted);
  CHECK(views_valid == rendered, "views were valid in %u of %u frames", views_valid, rendered);

  std::printf("left view  in LOCAL: pos (%.3f %.3f %.3f) fov (%.2f %.2f %.2f %.2f)\n", last_views[0].position.x,
              last_views[0].position.y, last_views[0].position.z, last_fov[0].angleLeft, last_fov[0].angleRight,
              last_fov[0].angleUp, last_fov[0].angleDown);
  std::printf("right view in LOCAL: pos (%.3f %.3f %.3f)\n", last_views[1].position.x, last_views[1].position.y,
              last_views[1].position.z);
  const float dx = last_views[1].position.x - last_views[0].position.x;
  const float dy = last_views[1].position.y - last_views[0].position.y;
  const float dz = last_views[1].position.z - last_views[0].position.z;
  const float ipd = std::sqrt(dx * dx + dy * dy + dz * dz);
  CHECK(ipd > 0.05f && ipd < 0.08f, "the eyes are %.4f m apart", ipd);
  CHECK(last_fov[0].angleLeft < 0 && last_fov[0].angleRight > 0 && last_fov[0].angleUp > 0 &&
            last_fov[0].angleDown < 0,
        "the field of view has the wrong signs");
  std::printf("head in STAGE: pos (%.3f %.3f %.3f)\n", head_in_stage.position.x, head_in_stage.position.y,
              head_in_stage.position.z);
  CHECK(head_in_stage.position.y > 1.0f && head_in_stage.position.y < 2.2f, "the head is %.2f m above the floor",
        head_in_stage.position.y);

  std::printf("right trigger range: %.3f .. %.3f\n", trigger_min, trigger_max);
  std::printf("left primary seen released=%d pressed=%d\n", primary_seen[0], primary_seen[1]);
  std::printf("grip poses valid: left %u, right %u of %u frames; right grip in LOCAL (%.3f %.3f %.3f)\n",
              grip_valid[0], grip_valid[1], rendered, last_grip[1].position.x, last_grip[1].position.y,
              last_grip[1].position.z);
  std::printf("grip poses tracked in position and orientation: left %u, right %u of %u frames\n",
              grip_tracked[0], grip_tracked[1], rendered);
  if (require_position_tracked)
    CHECK(grip_tracked[0] > 0 && grip_tracked[1] > 0, "both controllers must report tracked position and orientation");
  CHECK(profile_changed, "no interaction profile change was announced");
  XrInteractionProfileState profile{XR_TYPE_INTERACTION_PROFILE_STATE};
  REQUIRE_XR(xrGetCurrentInteractionProfile(session, hands[1], &profile));
  char profile_name[XR_MAX_PATH_LENGTH] = "(none)";
  uint32_t profile_length = 0;
  if (profile.interactionProfile != XR_NULL_PATH)
    xrPathToString(instance, profile.interactionProfile, sizeof(profile_name), &profile_length, profile_name);
  std::printf("right hand profile: %s\n", profile_name);
  CHECK(expect_profile == profile_name, "the runtime chose %s, not %s", profile_name, expect_profile.c_str());
  const bool touch = expect_profile == "/interaction_profiles/oculus/touch_controller";
  CHECK(grip_valid[0] > 0 && grip_valid[1] > 0, "a controller was never located");
  CHECK(haptics_ok == 1 || rendered <= 10, "the haptic pulse was not accepted");
  // the simulated headset moves its head and squeezes its trigger; a real one is not required to
  if (std::strstr(system_properties.systemName, "Simulated")) {
    CHECK(last_grip[1].position.x - last_grip[0].position.x > 0.1f, "the hands are on the wrong sides");
    if (frames_wanted >= 100) {
      CHECK(trigger_max - trigger_min > 0.1f, "the trigger did not move");
      // the simulated headset presses X; on the Vive profile the action is on the menu button, which it leaves
      CHECK(primary_seen[0] && (primary_seen[1] || !touch),
            "the primary button was not seen both released and pressed");
      CHECK(head_max_x - head_min_x > 0.01f, "the head did not turn");
    }
  }

  for (int eye = 0; eye < (use_array ? 1 : 2); ++eye)
    REQUIRE_XR(xrDestroySwapchain(eyes[eye].swapchain));
  if (use_quad)
    REQUIRE_XR(xrDestroySwapchain(quad_chain));
  if (hands_wanted) {
    std::printf("hands located: left %u, right %u of %u frames; wrist at most %.3f m from the grip\n", hands_seen[0],
                hands_seen[1], rendered, worst_wrist);
    if (std::strstr(system_properties.systemName, "Simulated") && frames_wanted >= 100)
      CHECK(hands_seen[0] > 0 && hands_seen[1] > 0 && worst_wrist < 0.03f,
            "the simulated hands were not located where the controllers are");
    for (XrHandTrackerEXT tracker : hand_trackers)
      REQUIRE_XR(xrDestroyHandTrackerEXT(tracker));
  }
  REQUIRE_XR(xrDestroySession(session));
  {
    // a session destroyed with its state changes unread takes them with it: an event must not name a dead session
    XrSession second = XR_NULL_HANDLE;
    REQUIRE_XR(xrCreateSession(instance, &session_info, &second));
    REQUIRE_XR(xrDestroySession(second));
    XrEventDataBuffer leftover{XR_TYPE_EVENT_DATA_BUFFER};
    CHECK(xrPollEvent(instance, &leftover) == XR_EVENT_UNAVAILABLE,
          "an event of type %d outlived the session it was about", static_cast<int>(leftover.type));
  }
  delete gfx;
  REQUIRE_XR(xrDestroyInstance(instance));

  std::printf("%s: %u frames rendered, %d check(s) failed\n", g_failures ? "FAILED" : "PASSED", rendered, g_failures);
  return g_failures ? 1 : 0;
}
