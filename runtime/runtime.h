// Internal declarations of the MetalVR OpenXR runtime.
//
// One source tree, two targets: a Windows DLL whose applications render with Direct3D 11 or Direct3D 12 (games under
// Wine, where Mullion turns Direct3D into Metal), and a macOS dylib whose applications render with Metal. Everything here
// is shared; the graphics backend is the only part that differs.
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
#include <winsock2.h>
#include <windows.h>
#include <d3d11.h>
#include <d3d12.h>
#else
#define XR_USE_GRAPHICS_API_METAL 1
#define XR_USE_TIMESPEC 1
#include <time.h>
#endif

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ipc.h"
#include "metalvr_math.h"
#include "metalvr_protocol.h"

namespace metalvr {

#define METALVR_RUNTIME_NAME "MetalVR"
#define METALVR_RUNTIME_VERSION XR_MAKE_VERSION(0, 1, 0)

void logLine(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

struct Instance;
struct Session;
struct Swapchain;
struct Space;
struct ActionSet;
struct Action;

// ---- handles -------------------------------------------------------------------------------------------------

enum class ObjectType : uint32_t { Instance, Session, Swapchain, Space, ActionSet, Action };

struct Object {
  explicit Object(ObjectType t);
  virtual ~Object();
  Object(const Object &) = delete;
  Object &operator=(const Object &) = delete;
  const ObjectType type;
};

// null unless `handle` names a live object of this type
Object *lookupObject(void *handle, ObjectType type);

template <typename T, typename Handle> T *lookup(Handle handle) {
  return static_cast<T *>(lookupObject(reinterpret_cast<void *>(handle), T::kType));
}

// ---- graphics backend ----------------------------------------------------------------------------------------

struct SwapchainImages {
  virtual ~SwapchainImages() = default;
  uint32_t count = 0;
  // fills the application's XrSwapchainImage*KHR array
  virtual XrResult enumerate(uint32_t capacity, uint32_t *count_out, XrSwapchainImageBaseHeader *images) = 0;
  // the application released image `index`: its contents go to the shares the server reads
  virtual void released(uint32_t index) = 0;
};

struct GraphicsBackend {
  virtual ~GraphicsBackend() = default;
  virtual uint32_t api() const = 0; // mvr_graphics_api
  virtual const std::vector<int64_t> &formats() const = 0;
  virtual uint32_t maxSampleCount() const = 0;
  // Makes a swapchain's images. A swapchain the server composes (color) fills `share`, except its swapchain_id;
  // one it never sees (depth) leaves `composited` false.
  virtual XrResult createSwapchain(const XrSwapchainCreateInfo &info, std::unique_ptr<SwapchainImages> &images,
                                   mvr_swapchain_create_request &share, bool &composited) = 0;
  // returns once the GPU has finished everything rendered into released images
  virtual void finishFrame() = 0;
  // Marks the end of everything rendered into released images so far, without waiting for it, and returns the mark;
  // 0 when this backend can only finish a frame in xrEndFrame's own thread (finishFrame).
  virtual uint64_t markFrame() { return 0; }
  // Waits for a mark from markFrame, from the frame submitter's thread; false if it was not reached in time.
  virtual bool waitMark(uint64_t, uint32_t /*timeout_ms*/) { return true; }
};

// the platform's graphics binding, found in the session create info's chain
XrResult createGraphicsBackend(const XrSessionCreateInfo *info, std::unique_ptr<GraphicsBackend> &out);
#ifdef _WIN32
// the Direct3D 12 backend, for a session with a Direct3D 12 binding (backend_d3d12.cpp)
XrResult createD3D12Backend(const XrGraphicsBindingD3D12KHR &binding, std::unique_ptr<GraphicsBackend> &out);
#endif
uint32_t platformGraphicsApi();

// ---- input ---------------------------------------------------------------------------------------------------

// a physical input of one controller, as the server reports it
enum class Source : uint8_t {
  None,
  TriggerValue,
  TriggerClick,
  TriggerTouch,
  SqueezeValue,
  SqueezeClick,
  Thumbstick, // both axes
  ThumbstickX,
  ThumbstickY,
  ThumbstickClick,
  ThumbstickTouch,
  PrimaryClick,
  PrimaryTouch,
  SecondaryClick,
  SecondaryTouch,
  MenuClick,
  SystemClick,
  ThumbrestTouch,
  GripPose,
  AimPose,
  Haptic,
};

struct Binding {
  int hand = 0; // 0 left, 1 right
  Source source = Source::None;
  XrPath path = XR_NULL_PATH;
};

struct ActionState {
  bool active = false;
  bool changed = false;
  float x = 0, y = 0; // boolean: x != 0; float: x; vector: x, y
  XrTime last_change = 0;
};

struct Action : Object {
  static constexpr ObjectType kType = ObjectType::Action;
  Action() : Object(kType) {}
  ActionSet *set = nullptr;
  XrActionType action_type = XR_ACTION_TYPE_BOOLEAN_INPUT;
  std::string name;
  std::string localized_name;
  std::vector<XrPath> subaction_paths;
  // set when the action's set is attached to the session
  std::vector<Binding> bindings;
  ActionState state[2]; // per hand
  ActionState combined;
};

struct ActionSet : Object {
  static constexpr ObjectType kType = ObjectType::ActionSet;
  ActionSet() : Object(kType) {}
  Instance *instance = nullptr;
  std::string name;
  std::string localized_name;
  uint32_t priority = 0;
  bool attached = false;
  std::vector<std::unique_ptr<Action>> actions;
};

struct SuggestedBinding {
  Action *action;
  XrPath binding;
};

// ---- spaces --------------------------------------------------------------------------------------------------

struct Space : Object {
  static constexpr ObjectType kType = ObjectType::Space;
  Space() : Object(kType) {}
  Session *session = nullptr;
  bool is_action = false;
  XrReferenceSpaceType reference = XR_REFERENCE_SPACE_TYPE_VIEW;
  Action *action = nullptr;
  XrPath subaction = XR_NULL_PATH;
  mvr_pose offset = poseIdentity();
};

// where a space is in the server's tracking space at some time
struct Located {
  mvr_pose pose = poseIdentity();
  bool orientation_valid = false;
  bool position_valid = false;
  bool tracked = false;
  bool has_velocity = false;
  bool is_static = false; // does not move in tracking space
  mvr_vec3 linear_velocity{};
  mvr_vec3 angular_velocity{};
};

// ---- swapchains ----------------------------------------------------------------------------------------------

struct Swapchain : Object {
  static constexpr ObjectType kType = ObjectType::Swapchain;
  Swapchain() : Object(kType) {}
  Session *session = nullptr;
  uint32_t id = 0;
  XrSwapchainCreateInfo info{};
  bool composited = false;
  bool opaque = false; // the format has no alpha channel to blend with
  bool is_static = false;
  bool static_acquired = false;
  std::unique_ptr<SwapchainImages> images;
  std::deque<uint32_t> acquired;
  bool waited = false;
  uint32_t next = 0;
  int32_t last_released = -1;
};

// ---- session -------------------------------------------------------------------------------------------------

struct Session : Object {
  static constexpr ObjectType kType = ObjectType::Session;
  Session() : Object(kType) {}
  ~Session() override;

  Instance *instance = nullptr;
  std::unique_ptr<GraphicsBackend> gfx;

  XrSessionState state = XR_SESSION_STATE_UNKNOWN;
  bool running = false;
  bool exit_requested = false;
  bool synchronized = false; // a frame has been waited since the session began

  std::vector<std::unique_ptr<Swapchain>> swapchains;
  std::vector<std::unique_ptr<Space>> spaces;
  uint32_t next_swapchain_id = 1;

  // frame loop
  std::mutex frame_mutex;
  std::condition_variable frame_cv;
  bool wait_pending = false; // xrWaitFrame returned and xrBeginFrame has not taken it
  bool frame_begun = false;
  uint64_t waited_frame_id = 0;
  uint64_t begun_frame_id = 0;
  uint64_t view_sample_id = 0; // the tracking sample xrLocateViews last answered from

  // A frame's end goes to the server from this thread, once the GPU has reached the frame's mark, so the application
  // records its next frame while the GPU finishes this one. One frame at most is waiting; anything else that talks
  // to the server about swapchains or the session waits for it first (drainFrames).
  struct PendingFrame {
    mvr_frame_end_request request{};
    std::vector<mvr_layer> layers;
    uint64_t mark = 0;
  };
  std::thread submitter;
  std::mutex submit_mutex;
  std::condition_variable submit_cv;
  bool submit_has_frame = false;
  bool submit_stop = false;
  PendingFrame submit_frame;
  std::atomic<bool> submit_lost{false}; // a frame's end could not be sent: the instance is lost
  void submitFrame(PendingFrame frame);
  void drainFrames();
  void stopSubmitter();

  // LOCAL's origin: where the head was, heading only, when it was first seen
  bool local_origin_set = false;
  mvr_pose local_origin = poseIdentity();

  // input
  bool action_sets_attached = false;
  std::vector<ActionSet *> attached_sets;
  XrPath active_profile[2] = {XR_NULL_PATH, XR_NULL_PATH};
  bool profile_announced = false;
  mvr_tracking_response input{}; // the sample of the last xrSyncActions
  bool input_valid = false;

  // tracking samples already fetched, by the time they were asked for
  std::deque<mvr_tracking_response> tracking_cache;
  std::deque<int64_t> tracking_cache_times;
  int64_t last_presence_check_ns = 0;

  void setState(XrSessionState next);
  // moves the state machine along from what the server reports
  void updatePresence(uint32_t tracking_flags);
  // the server's poses for `time` (0: now); false when the server is gone
  bool tracking(int64_t time, mvr_tracking_response &out);
  Located locate(const Space &space, const mvr_tracking_response &sample);
};

// ---- instance ------------------------------------------------------------------------------------------------

struct Instance : Object {
  static constexpr ObjectType kType = ObjectType::Instance;
  Instance() : Object(kType) {}
  ~Instance() override;

  // guards everything reachable from the instance; never held while a frame wait sleeps
  std::recursive_mutex mutex;
  Connection connection;
  mvr_system_info system{};
  XrVersion api_version = XR_API_VERSION_1_0;
  std::set<std::string> extensions;
  bool graphics_requirements_queried = false;
  bool lost = false;
  bool loss_announced = false;

  std::deque<XrEventDataBuffer> events;
  std::vector<std::string> paths; // XrPath n is paths[n - 1]
  std::unordered_map<std::string, XrPath> path_ids;
  std::vector<std::unique_ptr<ActionSet>> action_sets;
  std::map<XrPath, std::vector<SuggestedBinding>> suggested; // by interaction profile
  std::unique_ptr<Session> session;

  bool hasExtension(const char *name) const { return extensions.count(name) != 0; }
  XrPath path(const std::string &string);
  const std::string *pathString(XrPath path) const;
  template <typename Event> void pushEvent(const Event &event) {
    XrEventDataBuffer buffer{};
    std::memcpy(&buffer, &event, sizeof(Event));
    events.push_back(buffer);
  }
  // the server stopped answering: every later call that needs it fails, and the application is told once
  void markLost();
  bool refreshSystem();
  XrTime now() { return localNowNs() + connection.clockOffsetNs(); }
};

// ---- helpers -------------------------------------------------------------------------------------------------

inline mvr_pose fromXr(const XrPosef &p) {
  return {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w},
          {p.position.x, p.position.y, p.position.z}};
}
inline XrPosef toXr(const mvr_pose &p) {
  return {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w},
          {p.position.x, p.position.y, p.position.z}};
}
inline XrVector3f toXr(mvr_vec3 v) { return {v.x, v.y, v.z}; }
inline XrFovf toXr(const mvr_fov &f) { return {f.left, f.right, f.up, f.down}; }
inline mvr_fov fromXr(const XrFovf &f) { return {f.angleLeft, f.angleRight, f.angleUp, f.angleDown}; }

// copies a string into a fixed OpenXR char array
template <size_t N> void copyString(char (&out)[N], const std::string &in) {
  const size_t n = in.size() < N - 1 ? in.size() : N - 1;
  std::memcpy(out, in.data(), n);
  out[n] = 0;
}

// the two-call idiom for arrays of plain values
template <typename T>
XrResult fillArray(uint32_t capacity, uint32_t *count_out, T *out, const std::vector<T> &values) {
  if (!count_out)
    return XR_ERROR_VALIDATION_FAILURE;
  *count_out = static_cast<uint32_t>(values.size());
  if (capacity == 0)
    return XR_SUCCESS;
  if (capacity < values.size())
    return XR_ERROR_SIZE_INSUFFICIENT;
  if (!out)
    return XR_ERROR_VALIDATION_FAILURE;
  for (size_t i = 0; i < values.size(); ++i)
    out[i] = values[i];
  return XR_SUCCESS;
}

// input.cpp
void attachBindings(Session &session);
Located locateActionSpace(Session &session, const Space &space, const mvr_tracking_response &sample);

// dispatch.cpp
XrResult getProcAddr(const char *name, PFN_xrVoidFunction *function);

} // namespace metalvr
