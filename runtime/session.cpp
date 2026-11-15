// Sessions: lifecycle, reference spaces, swapchains, the frame loop and view location.
#include "api.h"

#include <algorithm>
#include <chrono>

namespace metalvr {

// ---- session state -------------------------------------------------------------------------------------------

Session::~Session() {
  if (!instance->lost) {
    for (auto &swapchain : swapchains) {
      if (!swapchain->composited)
        continue;
      mvr_swapchain_destroy_request request{swapchain->id};
      mvr_result_response response{};
      instance->connection.call(MVR_MSG_SWAPCHAIN_DESTROY, request, response);
    }
    if (running) {
      mvr_result_response response{};
      instance->connection.call(MVR_MSG_SESSION_END, response);
    }
  }
  swapchains.clear();
  spaces.clear();
  gfx.reset();
}

void Session::setState(XrSessionState next) {
  if (state == next)
    return;
  state = next;
  XrEventDataSessionStateChanged event{XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED};
  event.session = reinterpret_cast<XrSession>(this);
  event.state = next;
  event.time = instance->now();
  instance->pushEvent(event);
  logLine("session state -> %d", static_cast<int>(next));
}

void Session::updatePresence(uint32_t flags) {
  last_presence_check_ns = localNowNs();
  if (state == XR_SESSION_STATE_LOSS_PENDING || state == XR_SESSION_STATE_EXITING)
    return;
  const bool connected = (flags & MVR_TRACKING_HMD_CONNECTED) != 0;
  const bool presenting = connected && (flags & MVR_TRACKING_PRESENTING) != 0;
  if (!running) {
    if (state == XR_SESSION_STATE_IDLE && connected && !exit_requested)
      setState(XR_SESSION_STATE_READY);
    return;
  }
  if (state == XR_SESSION_STATE_STOPPING)
    return;
  if (!connected || exit_requested) {
    // the headset went, or the application asked to leave: walk down to STOPPING, which asks for xrEndSession
    if (state == XR_SESSION_STATE_FOCUSED)
      setState(XR_SESSION_STATE_VISIBLE);
    if (state == XR_SESSION_STATE_VISIBLE || state == XR_SESSION_STATE_READY)
      setState(XR_SESSION_STATE_SYNCHRONIZED);
    setState(XR_SESSION_STATE_STOPPING);
    return;
  }
  if (!synchronized)
    return;
  if (state == XR_SESSION_STATE_READY)
    setState(XR_SESSION_STATE_SYNCHRONIZED);
  if (presenting) {
    if (state == XR_SESSION_STATE_SYNCHRONIZED)
      setState(XR_SESSION_STATE_VISIBLE);
    if (state == XR_SESSION_STATE_VISIBLE)
      setState(XR_SESSION_STATE_FOCUSED);
  } else {
    if (state == XR_SESSION_STATE_FOCUSED)
      setState(XR_SESSION_STATE_VISIBLE);
    if (state == XR_SESSION_STATE_VISIBLE)
      setState(XR_SESSION_STATE_SYNCHRONIZED);
  }
}

bool Session::tracking(int64_t time, mvr_tracking_response &out) {
  if (instance->lost)
    return false;
  if (time != 0) {
    // one answer per time: every space located for a frame comes from the same sample
    for (size_t i = 0; i < tracking_cache_times.size(); ++i)
      if (tracking_cache_times[i] == time) {
        out = tracking_cache[i];
        return true;
      }
  }
  mvr_tracking_request request{time};
  if (!instance->connection.call(MVR_MSG_TRACKING, request, out) || out.result != MVR_OK) {
    instance->markLost();
    return false;
  }
  if (time != 0) {
    tracking_cache.push_back(out);
    tracking_cache_times.push_back(time);
    if (tracking_cache.size() > 8) {
      tracking_cache.pop_front();
      tracking_cache_times.pop_front();
    }
  }
  if (!local_origin_set && (out.flags & MVR_TRACKING_HEAD_VALID) && quatIsValid(out.head.orientation)) {
    local_origin = {quatYawOnly(out.head.orientation), out.head.position};
    local_origin_set = true;
  }
  return true;
}

Located Session::locate(const Space &space, const mvr_tracking_response &sample) {
  Located result;
  if (space.is_action) {
    result = locateActionSpace(*this, space, sample);
  } else {
    switch (space.reference) {
    case XR_REFERENCE_SPACE_TYPE_VIEW:
      result.pose = sample.head;
      result.orientation_valid = result.position_valid = result.tracked =
          (sample.flags & MVR_TRACKING_HEAD_VALID) != 0;
      result.has_velocity = result.tracked;
      result.linear_velocity = sample.head_linear_velocity;
      result.angular_velocity = sample.head_angular_velocity;
      break;
    case XR_REFERENCE_SPACE_TYPE_LOCAL:
    case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR:
      // before the head has been seen there is no origin to give; a seated eye height stands in and is not kept
      result.pose = local_origin_set ? local_origin : mvr_pose{quatIdentity(), {0, 1.6f, 0}};
      if (space.reference == XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR)
        result.pose.position.y = 0;
      result.orientation_valid = result.position_valid = result.tracked = true;
      result.is_static = true;
      break;
    default: // STAGE is the tracking space itself
      result.orientation_valid = result.position_valid = result.tracked = true;
      result.is_static = true;
      break;
    }
  }
  result.pose = poseMul(result.pose, space.offset);
  return result;
}

namespace {

bool poseValid(const XrPosef &pose) {
  return quatIsValid({pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}) &&
         std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z);
}

bool referenceSpaceSupported(const Instance &instance, XrReferenceSpaceType type) {
  switch (type) {
  case XR_REFERENCE_SPACE_TYPE_VIEW:
  case XR_REFERENCE_SPACE_TYPE_LOCAL:
  case XR_REFERENCE_SPACE_TYPE_STAGE:
    return true;
  case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR:
    return XR_VERSION_MINOR(instance.api_version) >= 1 || instance.hasExtension(XR_EXT_LOCAL_FLOOR_EXTENSION_NAME);
  default:
    return false;
  }
}

struct Relation {
  XrSpaceLocationFlags flags = 0;
  XrPosef pose{{0, 0, 0, 1}, {0, 0, 0}};
  XrSpaceVelocityFlags velocity_flags = 0;
  XrVector3f linear{0, 0, 0};
  XrVector3f angular{0, 0, 0};
};

// `space` as seen from `base`
Relation relate(Session &session, const Space &space, const Space &base, const mvr_tracking_response &sample) {
  const Located a = session.locate(space, sample);
  const Located b = session.locate(base, sample);
  Relation r;
  if (a.orientation_valid && b.orientation_valid)
    r.flags |= XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
  if (a.position_valid && b.position_valid && a.orientation_valid && b.orientation_valid)
    r.flags |= XR_SPACE_LOCATION_POSITION_VALID_BIT;
  if (a.tracked && b.tracked) {
    if (r.flags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
      r.flags |= XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    if (r.flags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
      r.flags |= XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
  }
  if (r.flags)
    r.pose = toXr(poseMul(poseInverse(b.pose), a.pose));
  // a velocity is only reported against a base that does not move
  if (b.is_static && r.flags) {
    const mvr_quat to_base = quatConj(b.pose.orientation);
    if (a.has_velocity) {
      r.linear = toXr(quatRotate(to_base, a.linear_velocity));
      r.angular = toXr(quatRotate(to_base, a.angular_velocity));
      r.velocity_flags = XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
    } else if (a.is_static) {
      r.velocity_flags = XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
    }
  }
  return r;
}

template <typename T> T *findInChain(void *next, XrStructureType type) {
  for (auto *base = static_cast<XrBaseOutStructure *>(next); base; base = base->next)
    if (base->type == type)
      return reinterpret_cast<T *>(base);
  return nullptr;
}

template <typename T> const T *findInChain(const void *next, XrStructureType type) {
  for (auto *base = static_cast<const XrBaseInStructure *>(next); base; base = base->next)
    if (base->type == type)
      return reinterpret_cast<const T *>(base);
  return nullptr;
}

Swapchain *findSwapchain(Session &session, XrSwapchain handle) {
  Swapchain *swapchain = lookup<Swapchain>(handle);
  return swapchain && swapchain->session == &session ? swapchain : nullptr;
}

} // namespace

namespace api {

// ---- lifecycle -----------------------------------------------------------------------------------------------

MVR_API xrCreateSession(XrInstance handle, const XrSessionCreateInfo *info, XrSession *out) {
  MVR_INSTANCE(handle);
  if (!info || !out || info->type != XR_TYPE_SESSION_CREATE_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (info->systemId != kSystemId)
    return XR_ERROR_SYSTEM_INVALID;
  MVR_ALIVE();
  if (instance->session)
    return XR_ERROR_LIMIT_REACHED;
  if (!instance->graphics_requirements_queried)
    return XR_ERROR_GRAPHICS_REQUIREMENTS_CALL_MISSING;

  auto session = std::make_unique<Session>();
  session->instance = instance;
  const XrResult result = createGraphicsBackend(info, session->gfx);
  if (XR_FAILED(result))
    return result;

  instance->session = std::move(session);
  Session *created = instance->session.get();
  created->setState(XR_SESSION_STATE_IDLE);
  mvr_tracking_response sample{};
  if (created->tracking(0, sample))
    created->updatePresence(sample.flags);
  *out = reinterpret_cast<XrSession>(created);
  return XR_SUCCESS;
}

MVR_API xrDestroySession(XrSession handle) {
  MVR_SESSION(handle);
  {
    // a frame wait in another thread ends with the session
    std::lock_guard<std::mutex> lock(session->frame_mutex);
    session->running = false;
    session->wait_pending = false;
  }
  session->frame_cv.notify_all();
  // an event about this session means nothing once it is gone, and names a handle that may be reused
  auto &events = instance->events;
  events.erase(std::remove_if(events.begin(), events.end(),
                              [&](const XrEventDataBuffer &event) {
                                switch (event.type) {
                                case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED:
                                  return reinterpret_cast<const XrEventDataSessionStateChanged *>(&event)->session ==
                                         handle;
                                case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                                  return reinterpret_cast<const XrEventDataInteractionProfileChanged *>(&event)
                                             ->session == handle;
                                default:
                                  return false;
                                }
                              }),
               events.end());
  // actions bound to this session are free to be attached to the next one
  for (ActionSet *set : session->attached_sets) {
    set->attached = false;
    for (auto &action : set->actions)
      action->bindings.clear();
  }
  instance->session.reset();
  return XR_SUCCESS;
}

MVR_API xrBeginSession(XrSession handle, const XrSessionBeginInfo *info) {
  MVR_SESSION(handle);
  if (!info || info->type != XR_TYPE_SESSION_BEGIN_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (info->primaryViewConfigurationType != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)
    return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
  if (session->running)
    return XR_ERROR_SESSION_RUNNING;
  if (session->state != XR_SESSION_STATE_READY)
    return XR_ERROR_SESSION_NOT_READY;
  MVR_ALIVE();
  mvr_result_response response{};
  if (!instance->connection.call(MVR_MSG_SESSION_BEGIN, response)) {
    instance->markLost();
    return XR_ERROR_INSTANCE_LOST;
  }
  if (response.result != MVR_OK) {
    // another application holds the headset
    logLine("xrBeginSession: the server refused the session (%u)", response.result);
    return XR_ERROR_RUNTIME_FAILURE;
  }
  session->running = true;
  session->synchronized = false;
  session->wait_pending = false;
  session->frame_begun = false;
  return XR_SUCCESS;
}

MVR_API xrEndSession(XrSession handle) {
  MVR_SESSION(handle);
  if (!session->running)
    return XR_ERROR_SESSION_NOT_RUNNING;
  if (session->state != XR_SESSION_STATE_STOPPING)
    return XR_ERROR_SESSION_NOT_STOPPING;
  if (!instance->lost) {
    mvr_result_response response{};
    if (!instance->connection.call(MVR_MSG_SESSION_END, response))
      instance->markLost();
  }
  {
    std::lock_guard<std::mutex> lock(session->frame_mutex);
    session->running = false;
    session->wait_pending = false;
    session->frame_begun = false;
  }
  session->frame_cv.notify_all();
  if (session->state != XR_SESSION_STATE_LOSS_PENDING) {
    session->setState(XR_SESSION_STATE_IDLE);
    if (session->exit_requested)
      session->setState(XR_SESSION_STATE_EXITING);
  }
  return XR_SUCCESS;
}

MVR_API xrRequestExitSession(XrSession handle) {
  MVR_SESSION(handle);
  if (!session->running)
    return XR_ERROR_SESSION_NOT_RUNNING;
  session->exit_requested = true;
  session->updatePresence(MVR_TRACKING_HMD_CONNECTED);
  return XR_SUCCESS;
}

// ---- spaces --------------------------------------------------------------------------------------------------

MVR_API xrEnumerateReferenceSpaces(XrSession handle, uint32_t capacity, uint32_t *count,
                                   XrReferenceSpaceType *spaces) {
  MVR_SESSION(handle);
  std::vector<XrReferenceSpaceType> types{XR_REFERENCE_SPACE_TYPE_VIEW, XR_REFERENCE_SPACE_TYPE_LOCAL,
                                          XR_REFERENCE_SPACE_TYPE_STAGE};
  if (referenceSpaceSupported(*instance, XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR))
    types.push_back(XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR);
  return fillArray(capacity, count, spaces, types);
}

MVR_API xrCreateReferenceSpace(XrSession handle, const XrReferenceSpaceCreateInfo *info, XrSpace *out) {
  MVR_SESSION(handle);
  if (!info || !out || info->type != XR_TYPE_REFERENCE_SPACE_CREATE_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!referenceSpaceSupported(*instance, info->referenceSpaceType))
    return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
  if (!poseValid(info->poseInReferenceSpace))
    return XR_ERROR_POSE_INVALID;
  auto space = std::make_unique<Space>();
  space->session = session;
  space->reference = info->referenceSpaceType;
  space->offset = fromXr(info->poseInReferenceSpace);
  *out = reinterpret_cast<XrSpace>(space.get());
  session->spaces.push_back(std::move(space));
  return XR_SUCCESS;
}

MVR_API xrGetReferenceSpaceBoundsRect(XrSession handle, XrReferenceSpaceType type, XrExtent2Df *bounds) {
  MVR_SESSION(handle);
  if (!bounds)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!referenceSpaceSupported(*instance, type))
    return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
  // the headset's play area is not carried over the stream
  bounds->width = 0;
  bounds->height = 0;
  return XR_SPACE_BOUNDS_UNAVAILABLE;
}

MVR_API xrCreateActionSpace(XrSession handle, const XrActionSpaceCreateInfo *info, XrSpace *out) {
  MVR_SESSION(handle);
  if (!info || !out || info->type != XR_TYPE_ACTION_SPACE_CREATE_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  Action *action = lookup<Action>(info->action);
  if (!action)
    return XR_ERROR_HANDLE_INVALID;
  if (action->action_type != XR_ACTION_TYPE_POSE_INPUT)
    return XR_ERROR_ACTION_TYPE_MISMATCH;
  if (info->subactionPath != XR_NULL_PATH &&
      std::find(action->subaction_paths.begin(), action->subaction_paths.end(), info->subactionPath) ==
          action->subaction_paths.end())
    return XR_ERROR_PATH_UNSUPPORTED;
  if (!poseValid(info->poseInActionSpace))
    return XR_ERROR_POSE_INVALID;
  auto space = std::make_unique<Space>();
  space->session = session;
  space->is_action = true;
  space->action = action;
  space->subaction = info->subactionPath;
  space->offset = fromXr(info->poseInActionSpace);
  *out = reinterpret_cast<XrSpace>(space.get());
  session->spaces.push_back(std::move(space));
  return XR_SUCCESS;
}

MVR_API xrLocateSpace(XrSpace space_handle, XrSpace base_handle, XrTime time, XrSpaceLocation *location) {
  Space *space = lookup<Space>(space_handle);
  Space *base = lookup<Space>(base_handle);
  if (!space || !base)
    return XR_ERROR_HANDLE_INVALID;
  Session *session = space->session;
  Instance *instance = session->instance;
  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex);
  if (!location || location->type != XR_TYPE_SPACE_LOCATION)
    return XR_ERROR_VALIDATION_FAILURE;
  if (time <= 0)
    return XR_ERROR_TIME_INVALID;
  mvr_tracking_response sample{};
  if (!session->tracking(time, sample))
    return XR_ERROR_INSTANCE_LOST;
  const Relation relation = relate(*session, *space, *base, sample);
  location->locationFlags = relation.flags;
  location->pose = relation.pose;
  if (auto *velocity = findInChain<XrSpaceVelocity>(location->next, XR_TYPE_SPACE_VELOCITY)) {
    velocity->velocityFlags = relation.velocity_flags;
    velocity->linearVelocity = relation.linear;
    velocity->angularVelocity = relation.angular;
  }
  return XR_SUCCESS;
}

MVR_API xrLocateSpaces(XrSession handle, const XrSpacesLocateInfo *info, XrSpaceLocations *locations) {
  MVR_SESSION(handle);
  if (!info || !locations || info->type != XR_TYPE_SPACES_LOCATE_INFO || locations->type != XR_TYPE_SPACE_LOCATIONS ||
      !info->spaces || !locations->locations || locations->locationCount != info->spaceCount)
    return XR_ERROR_VALIDATION_FAILURE;
  if (info->time <= 0)
    return XR_ERROR_TIME_INVALID;
  Space *base = lookup<Space>(info->baseSpace);
  if (!base)
    return XR_ERROR_HANDLE_INVALID;
  mvr_tracking_response sample{};
  if (!session->tracking(info->time, sample))
    return XR_ERROR_INSTANCE_LOST;
  auto *velocities = findInChain<XrSpaceVelocities>(locations->next, XR_TYPE_SPACE_VELOCITIES);
  if (velocities && (velocities->velocityCount != info->spaceCount || !velocities->velocities))
    return XR_ERROR_VALIDATION_FAILURE;
  for (uint32_t i = 0; i < info->spaceCount; ++i) {
    Space *space = lookup<Space>(info->spaces[i]);
    if (!space)
      return XR_ERROR_HANDLE_INVALID;
    const Relation relation = relate(*session, *space, *base, sample);
    locations->locations[i].locationFlags = relation.flags;
    locations->locations[i].pose = relation.pose;
    if (velocities) {
      velocities->velocities[i].velocityFlags = relation.velocity_flags;
      velocities->velocities[i].linearVelocity = relation.linear;
      velocities->velocities[i].angularVelocity = relation.angular;
    }
  }
  return XR_SUCCESS;
}

MVR_API xrDestroySpace(XrSpace handle) {
  Space *space = lookup<Space>(handle);
  if (!space)
    return XR_ERROR_HANDLE_INVALID;
  Session *session = space->session;
  std::lock_guard<std::recursive_mutex> instance_lock(session->instance->mutex);
  auto &spaces = session->spaces;
  spaces.erase(std::remove_if(spaces.begin(), spaces.end(), [&](const auto &owned) { return owned.get() == space; }),
               spaces.end());
  return XR_SUCCESS;
}

// ---- swapchains ----------------------------------------------------------------------------------------------

MVR_API xrEnumerateSwapchainFormats(XrSession handle, uint32_t capacity, uint32_t *count, int64_t *formats) {
  MVR_SESSION(handle);
  return fillArray(capacity, count, formats, session->gfx->formats());
}

MVR_API xrCreateSwapchain(XrSession handle, const XrSwapchainCreateInfo *info, XrSwapchain *out) {
  MVR_SESSION(handle);
  if (!info || !out || info->type != XR_TYPE_SWAPCHAIN_CREATE_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!info->width || !info->height || !info->arraySize || !info->mipCount || !info->sampleCount ||
      info->width > kMaxSwapchainSize || info->height > kMaxSwapchainSize)
    return XR_ERROR_VALIDATION_FAILURE;
  if (info->faceCount != 1) // cube swapchains belong to cube layers, which are not offered
    return XR_ERROR_FEATURE_UNSUPPORTED;
  if (info->sampleCount > session->gfx->maxSampleCount())
    return XR_ERROR_FEATURE_UNSUPPORTED;
  const auto &formats = session->gfx->formats();
  if (std::find(formats.begin(), formats.end(), info->format) == formats.end())
    return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
  if (info->createFlags & XR_SWAPCHAIN_CREATE_PROTECTED_CONTENT_BIT)
    return XR_ERROR_FEATURE_UNSUPPORTED;
  MVR_ALIVE();

  auto swapchain = std::make_unique<Swapchain>();
  swapchain->session = session;
  swapchain->info = *info;
  swapchain->info.next = nullptr;
  swapchain->is_static = (info->createFlags & XR_SWAPCHAIN_CREATE_STATIC_IMAGE_BIT) != 0;
  swapchain->id = session->next_swapchain_id++;

  mvr_swapchain_create_request share{};
  XrResult result = session->gfx->createSwapchain(*info, swapchain->images, share, swapchain->composited);
  if (XR_FAILED(result))
    return result;
  if (swapchain->composited) {
    swapchain->opaque = share.format == MVR_FORMAT_BGRX8_UNORM || share.format == MVR_FORMAT_BGRX8_SRGB;
    share.swapchain_id = swapchain->id;
    mvr_result_response response{};
    if (!instance->connection.call(MVR_MSG_SWAPCHAIN_CREATE, share, response)) {
      instance->markLost();
      return XR_ERROR_INSTANCE_LOST;
    }
    if (response.result != MVR_OK) {
      logLine("xrCreateSwapchain: the server could not open the images (%u)", response.result);
      return XR_ERROR_RUNTIME_FAILURE;
    }
  }
  *out = reinterpret_cast<XrSwapchain>(swapchain.get());
  session->swapchains.push_back(std::move(swapchain));
  return XR_SUCCESS;
}

MVR_API xrDestroySwapchain(XrSwapchain handle) {
  Swapchain *swapchain = lookup<Swapchain>(handle);
  if (!swapchain)
    return XR_ERROR_HANDLE_INVALID;
  Session *session = swapchain->session;
  Instance *instance = session->instance;
  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex);
  if (swapchain->composited && !instance->lost) {
    mvr_swapchain_destroy_request request{swapchain->id};
    mvr_result_response response{};
    if (!instance->connection.call(MVR_MSG_SWAPCHAIN_DESTROY, request, response))
      instance->markLost();
  }
  auto &list = session->swapchains;
  list.erase(std::remove_if(list.begin(), list.end(), [&](const auto &owned) { return owned.get() == swapchain; }),
             list.end());
  return XR_SUCCESS;
}

MVR_API xrEnumerateSwapchainImages(XrSwapchain handle, uint32_t capacity, uint32_t *count,
                                   XrSwapchainImageBaseHeader *images) {
  Swapchain *swapchain = lookup<Swapchain>(handle);
  if (!swapchain)
    return XR_ERROR_HANDLE_INVALID;
  std::lock_guard<std::recursive_mutex> instance_lock(swapchain->session->instance->mutex);
  return swapchain->images->enumerate(capacity, count, images);
}

MVR_API xrAcquireSwapchainImage(XrSwapchain handle, const XrSwapchainImageAcquireInfo *, uint32_t *index) {
  Swapchain *swapchain = lookup<Swapchain>(handle);
  if (!swapchain)
    return XR_ERROR_HANDLE_INVALID;
  std::lock_guard<std::recursive_mutex> instance_lock(swapchain->session->instance->mutex);
  if (!index)
    return XR_ERROR_VALIDATION_FAILURE;
  if (swapchain->acquired.size() >= swapchain->images->count)
    return XR_ERROR_CALL_ORDER_INVALID;
  if (swapchain->is_static && swapchain->static_acquired)
    return XR_ERROR_CALL_ORDER_INVALID;
  swapchain->static_acquired = true;
  *index = swapchain->next;
  swapchain->acquired.push_back(swapchain->next);
  swapchain->next = (swapchain->next + 1) % swapchain->images->count;
  return XR_SUCCESS;
}

MVR_API xrWaitSwapchainImage(XrSwapchain handle, const XrSwapchainImageWaitInfo *info) {
  Swapchain *swapchain = lookup<Swapchain>(handle);
  if (!swapchain)
    return XR_ERROR_HANDLE_INVALID;
  std::lock_guard<std::recursive_mutex> instance_lock(swapchain->session->instance->mutex);
  if (!info || info->type != XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (swapchain->acquired.empty() || swapchain->waited)
    return XR_ERROR_CALL_ORDER_INVALID;
  // the server reads a share only inside xrEndFrame, so an acquired image is always free to write
  swapchain->waited = true;
  return XR_SUCCESS;
}

MVR_API xrReleaseSwapchainImage(XrSwapchain handle, const XrSwapchainImageReleaseInfo *) {
  Swapchain *swapchain = lookup<Swapchain>(handle);
  if (!swapchain)
    return XR_ERROR_HANDLE_INVALID;
  std::lock_guard<std::recursive_mutex> instance_lock(swapchain->session->instance->mutex);
  if (swapchain->acquired.empty() || !swapchain->waited)
    return XR_ERROR_CALL_ORDER_INVALID;
  const uint32_t index = swapchain->acquired.front();
  swapchain->acquired.pop_front();
  swapchain->waited = false;
  swapchain->images->released(index);
  swapchain->last_released = static_cast<int32_t>(index);
  return XR_SUCCESS;
}

// ---- frame loop ----------------------------------------------------------------------------------------------

MVR_API xrWaitFrame(XrSession handle, const XrFrameWaitInfo *, XrFrameState *state) {
  Session *session = lookup<Session>(handle);
  if (!session)
    return XR_ERROR_HANDLE_INVALID;
  Instance *instance = session->instance;
  if (!state || state->type != XR_TYPE_FRAME_STATE)
    return XR_ERROR_VALIDATION_FAILURE;
  {
    // a frame is waited once the one before it has begun; the instance lock is not held while that takes time
    std::unique_lock<std::mutex> lock(session->frame_mutex);
    if (!session->running)
      return XR_ERROR_SESSION_NOT_RUNNING;
    session->frame_cv.wait_for(lock, std::chrono::seconds(1),
                               [&] { return !session->wait_pending || !session->running; });
    if (!session->running)
      return XR_ERROR_SESSION_NOT_RUNNING;
  }
  if (instance->lost)
    return XR_ERROR_INSTANCE_LOST;

  mvr_frame_wait_response response{};
  if (!instance->connection.call(MVR_MSG_FRAME_WAIT, response) || response.result != MVR_OK) {
    std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex);
    instance->markLost();
    return XR_ERROR_INSTANCE_LOST;
  }
  sleepUntilLocalNs(response.wake_time_ns - instance->connection.clockOffsetNs());

  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex);
  if (lookup<Session>(handle) != session || !session->running)
    return XR_ERROR_SESSION_NOT_RUNNING;
  session->synchronized = true;
  mvr_tracking_response sample{};
  if (session->tracking(0, sample))
    session->updatePresence(sample.flags);
  {
    std::lock_guard<std::mutex> lock(session->frame_mutex);
    session->wait_pending = true;
    session->waited_frame_id = response.frame_id;
  }
  state->predictedDisplayTime = response.predicted_display_time_ns;
  state->predictedDisplayPeriod = response.period_ns;
  state->shouldRender = response.should_render && (session->state == XR_SESSION_STATE_VISIBLE ||
                                                   session->state == XR_SESSION_STATE_FOCUSED)
                            ? XR_TRUE
                            : XR_FALSE;
  return instance->lost ? XR_ERROR_INSTANCE_LOST : XR_SUCCESS;
}

MVR_API xrBeginFrame(XrSession handle, const XrFrameBeginInfo *) {
  MVR_SESSION(handle);
  if (!session->running)
    return XR_ERROR_SESSION_NOT_RUNNING;
  XrResult result;
  {
    std::lock_guard<std::mutex> lock(session->frame_mutex);
    if (!session->wait_pending)
      return XR_ERROR_CALL_ORDER_INVALID;
    session->wait_pending = false;
    // a frame begun and never ended is dropped in favor of this one
    result = session->frame_begun ? XR_FRAME_DISCARDED : XR_SUCCESS;
    session->frame_begun = true;
    session->begun_frame_id = session->waited_frame_id;
  }
  session->frame_cv.notify_all();
  return result;
}

MVR_API xrEndFrame(XrSession handle, const XrFrameEndInfo *info) {
  MVR_SESSION(handle);
  if (!info || info->type != XR_TYPE_FRAME_END_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!session->running)
    return XR_ERROR_SESSION_NOT_RUNNING;
  if (!session->frame_begun)
    return XR_ERROR_CALL_ORDER_INVALID;
  if (info->displayTime <= 0)
    return XR_ERROR_TIME_INVALID;
  if (info->environmentBlendMode != XR_ENVIRONMENT_BLEND_MODE_OPAQUE)
    return XR_ERROR_ENVIRONMENT_BLEND_MODE_UNSUPPORTED;
  if (info->layerCount > MVR_MAX_LAYERS)
    return XR_ERROR_LAYER_LIMIT_EXCEEDED;
  if (info->layerCount && !info->layers)
    return XR_ERROR_VALIDATION_FAILURE;
  MVR_ALIVE();

  mvr_tracking_response sample{};
  if (info->layerCount && !session->tracking(info->displayTime, sample))
    return XR_ERROR_INSTANCE_LOST;

  std::vector<mvr_layer> layers;
  bool opaque_source = false; // set by subImage when a layer reads a swapchain with no alpha channel
  const auto subImage = [&](const XrSwapchainSubImage &sub, mvr_layer_view &out) -> XrResult {
    Swapchain *swapchain = findSwapchain(*session, sub.swapchain);
    if (!swapchain)
      return XR_ERROR_HANDLE_INVALID;
    if (!swapchain->composited || swapchain->last_released < 0)
      return XR_ERROR_LAYER_INVALID;
    if (sub.imageArrayIndex >= swapchain->info.arraySize)
      return XR_ERROR_VALIDATION_FAILURE;
    // A negative extent mirrors the image along that axis. OpenXR leaves it undefined; the PC runtimes take it,
    // and OpenVR games reach this runtime through OpenComposite, which submits a bottom-up texture that way.
    const XrRect2Di &rect = sub.imageRect;
    const int64_t x0 = std::min<int64_t>(rect.offset.x, static_cast<int64_t>(rect.offset.x) + rect.extent.width);
    const int64_t x1 = std::max<int64_t>(rect.offset.x, static_cast<int64_t>(rect.offset.x) + rect.extent.width);
    const int64_t y0 = std::min<int64_t>(rect.offset.y, static_cast<int64_t>(rect.offset.y) + rect.extent.height);
    const int64_t y1 = std::max<int64_t>(rect.offset.y, static_cast<int64_t>(rect.offset.y) + rect.extent.height);
    if (!rect.extent.width || !rect.extent.height || x0 < 0 || y0 < 0 || x1 > swapchain->info.width ||
        y1 > swapchain->info.height)
      return XR_ERROR_SWAPCHAIN_RECT_INVALID;
    if (swapchain->opaque)
      opaque_source = true;
    out.swapchain_id = swapchain->id;
    out.image_index = static_cast<uint32_t>(swapchain->last_released);
    out.array_index = sub.imageArrayIndex;
    out.rect = {rect.offset.x, rect.offset.y, rect.extent.width, rect.extent.height};
    return XR_SUCCESS;
  };
  const auto layerFlags = [](XrCompositionLayerFlags flags) {
    uint32_t out = 0;
    if (flags & XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT)
      out |= MVR_LAYER_BLEND_ALPHA;
    if (flags & XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT)
      out |= MVR_LAYER_UNPREMULTIPLIED_ALPHA;
    return out;
  };

  for (uint32_t i = 0; i < info->layerCount; ++i) {
    const XrCompositionLayerBaseHeader *header = info->layers[i];
    if (!header)
      return XR_ERROR_LAYER_INVALID;
    Space *space = lookup<Space>(header->space);
    if (!space || space->session != session)
      return XR_ERROR_HANDLE_INVALID;
    const Located space_located = session->locate(*space, sample);
    mvr_layer layer{};
    layer.flags = layerFlags(header->layerFlags);
    layer.eye_mask = 3;
    opaque_source = false;

    if (header->type == XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
      const auto *projection = reinterpret_cast<const XrCompositionLayerProjection *>(header);
      if (projection->viewCount != 2 || !projection->views)
        return XR_ERROR_VALIDATION_FAILURE;
      layer.type = MVR_LAYER_PROJECTION;
      for (uint32_t eye = 0; eye < 2; ++eye) {
        const XrCompositionLayerProjectionView &view = projection->views[eye];
        if (view.type != XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW)
          return XR_ERROR_VALIDATION_FAILURE;
        if (!poseValid(view.pose))
          return XR_ERROR_POSE_INVALID;
        const XrResult result = subImage(view.subImage, layer.views[eye]);
        if (XR_FAILED(result))
          return result;
        layer.views[eye].pose = poseMul(space_located.pose, fromXr(view.pose));
        layer.views[eye].fov = fromXr(view.fov);
      }
    } else if (header->type == XR_TYPE_COMPOSITION_LAYER_QUAD) {
      const auto *quad = reinterpret_cast<const XrCompositionLayerQuad *>(header);
      if (!poseValid(quad->pose))
        return XR_ERROR_POSE_INVALID;
      layer.type = MVR_LAYER_QUAD;
      const XrResult result = subImage(quad->subImage, layer.views[0]);
      if (XR_FAILED(result))
        return result;
      layer.eye_mask = quad->eyeVisibility == XR_EYE_VISIBILITY_LEFT    ? 1u
                       : quad->eyeVisibility == XR_EYE_VISIBILITY_RIGHT ? 2u
                                                                        : 3u;
      layer.quad_width = quad->size.width;
      layer.quad_height = quad->size.height;
      if (!space->is_action && space->reference == XR_REFERENCE_SPACE_TYPE_VIEW) {
        // a quad in view space rides the head: the server places it against the pose the frame was rendered with
        layer.flags |= MVR_LAYER_HEAD_LOCKED;
        layer.quad_pose = poseMul(space->offset, fromXr(quad->pose));
      } else {
        layer.quad_pose = poseMul(space_located.pose, fromXr(quad->pose));
      }
    } else {
      // cylinder, equirect and cube layers are not offered, so no enabled extension makes them valid here
      return XR_ERROR_LAYER_INVALID;
    }
    if (opaque_source)
      layer.flags &= ~static_cast<uint32_t>(MVR_LAYER_BLEND_ALPHA | MVR_LAYER_UNPREMULTIPLIED_ALPHA);
    layers.push_back(layer);
  }

  // what was rendered into the released images must be on the GPU's side of the fence before the server reads it
  if (!layers.empty())
    session->gfx->finishFrame();

  mvr_frame_end_request request{};
  request.frame_id = session->begun_frame_id;
  request.sample_id = session->view_sample_id;
  request.display_time_ns = info->displayTime;
  request.layer_count = static_cast<uint32_t>(layers.size());
  mvr_result_response response{};
  const bool sent = instance->connection.call(MVR_MSG_FRAME_END, &request, sizeof(request), &response,
                                              sizeof(response), layers.data(), layers.size() * sizeof(mvr_layer));
  {
    std::lock_guard<std::mutex> lock(session->frame_mutex);
    session->frame_begun = false;
  }
  if (!sent) {
    instance->markLost();
    return XR_ERROR_INSTANCE_LOST;
  }
  if (response.result != MVR_OK) {
    logLine("xrEndFrame: the server rejected the frame (%u)", response.result);
    return XR_ERROR_RUNTIME_FAILURE;
  }
  return XR_SUCCESS;
}

MVR_API xrLocateViews(XrSession handle, const XrViewLocateInfo *info, XrViewState *state, uint32_t capacity,
                      uint32_t *count, XrView *views) {
  MVR_SESSION(handle);
  if (!info || !state || !count || info->type != XR_TYPE_VIEW_LOCATE_INFO || state->type != XR_TYPE_VIEW_STATE)
    return XR_ERROR_VALIDATION_FAILURE;
  if (info->viewConfigurationType != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)
    return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
  Space *base = lookup<Space>(info->space);
  if (!base || base->session != session)
    return XR_ERROR_HANDLE_INVALID;
  if (info->displayTime <= 0)
    return XR_ERROR_TIME_INVALID;
  *count = 2;
  if (capacity == 0)
    return XR_SUCCESS;
  if (capacity < 2)
    return XR_ERROR_SIZE_INSUFFICIENT;
  if (!views)
    return XR_ERROR_VALIDATION_FAILURE;
  mvr_tracking_response sample{};
  if (!session->tracking(info->displayTime, sample))
    return XR_ERROR_INSTANCE_LOST;
  session->view_sample_id = sample.sample_id;

  const bool head_valid = (sample.flags & MVR_TRACKING_HEAD_VALID) != 0;
  const bool view_relative = !base->is_action && base->reference == XR_REFERENCE_SPACE_TYPE_VIEW;
  mvr_pose to_base;
  if (view_relative) {
    // where the eyes are in the head does not depend on where the head is: it is valid before the head is tracked,
    // which is when an application reads its eye offsets and interpupillary distance
    to_base = poseInverse(poseMul(sample.head, base->offset));
    state->viewStateFlags = XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT;
    if (head_valid)
      state->viewStateFlags |= XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT;
  } else {
    const Located located = session->locate(*base, sample);
    const bool valid = head_valid && located.orientation_valid && located.position_valid;
    state->viewStateFlags = valid ? (XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT |
                                     XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT)
                                  : 0;
    to_base = poseInverse(located.pose);
  }
  for (uint32_t eye = 0; eye < 2; ++eye) {
    if (views[eye].type != XR_TYPE_VIEW)
      return XR_ERROR_VALIDATION_FAILURE;
    views[eye].pose = toXr(poseMul(to_base, sample.views[eye].pose));
    views[eye].fov = toXr(sample.views[eye].fov);
  }
  return XR_SUCCESS;
}

// ---- XR_KHR_visibility_mask ----------------------------------------------------------------------------------

MVR_API xrGetVisibilityMaskKHR(XrSession handle, XrViewConfigurationType type, uint32_t view_index,
                               XrVisibilityMaskTypeKHR, XrVisibilityMaskKHR *mask) {
  MVR_SESSION(handle);
  if (!mask || mask->type != XR_TYPE_VISIBILITY_MASK_KHR)
    return XR_ERROR_VALIDATION_FAILURE;
  if (type != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)
    return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
  if (view_index > 1)
    return XR_ERROR_VALIDATION_FAILURE;
  // no mask: the whole image reaches the headset
  mask->vertexCountOutput = 0;
  mask->indexCountOutput = 0;
  return XR_SUCCESS;
}

// ---- XR_FB_display_refresh_rate ------------------------------------------------------------------------------
// The headset's rate is whatever the stream was negotiated at; it is reported, and only that rate can be asked for.

MVR_API xrEnumerateDisplayRefreshRatesFB(XrSession handle, uint32_t capacity, uint32_t *count, float *rates) {
  MVR_SESSION(handle);
  return fillArray<float>(capacity, count, rates, {instance->system.refresh_hz});
}

MVR_API xrGetDisplayRefreshRateFB(XrSession handle, float *rate) {
  MVR_SESSION(handle);
  if (!rate)
    return XR_ERROR_VALIDATION_FAILURE;
  *rate = instance->system.refresh_hz;
  return XR_SUCCESS;
}

MVR_API xrRequestDisplayRefreshRateFB(XrSession handle, float rate) {
  MVR_SESSION(handle);
  if (rate != 0.0f && std::fabs(rate - instance->system.refresh_hz) > 0.5f)
    return XR_ERROR_DISPLAY_REFRESH_RATE_UNSUPPORTED_FB;
  return XR_SUCCESS;
}

} // namespace api
} // namespace metalvr
