// XR_EXT_hand_tracking: the hands' joints as the headset tracks them (MVR_MSG_HANDS), located in any space.
#include "api.h"

#include <algorithm>

namespace metalvr {
namespace api {

MVR_API xrCreateHandTrackerEXT(XrSession handle, const XrHandTrackerCreateInfoEXT *info, XrHandTrackerEXT *out) {
  MVR_SESSION(handle);
  if (!info || !out || info->type != XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!instance->hasExtension(XR_EXT_HAND_TRACKING_EXTENSION_NAME))
    return XR_ERROR_FUNCTION_UNSUPPORTED;
  if (info->hand != XR_HAND_LEFT_EXT && info->hand != XR_HAND_RIGHT_EXT)
    return XR_ERROR_VALIDATION_FAILURE;
  if (info->handJointSet != XR_HAND_JOINT_SET_DEFAULT_EXT)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!(instance->system.flags & MVR_SYSTEM_HAND_TRACKING))
    return XR_ERROR_FEATURE_UNSUPPORTED;
  auto tracker = std::make_unique<HandTracker>();
  tracker->session = session;
  tracker->hand = info->hand == XR_HAND_LEFT_EXT ? 0 : 1;
  *out = reinterpret_cast<XrHandTrackerEXT>(tracker.get());
  session->hand_trackers.push_back(std::move(tracker));
  return XR_SUCCESS;
}

MVR_API xrDestroyHandTrackerEXT(XrHandTrackerEXT handle) {
  HandTracker *tracker = lookup<HandTracker>(handle);
  if (!tracker)
    return XR_ERROR_HANDLE_INVALID;
  Session *session = tracker->session;
  std::lock_guard<std::recursive_mutex> lock(session->instance->mutex);
  auto &list = session->hand_trackers;
  list.erase(std::remove_if(list.begin(), list.end(), [&](const auto &t) { return t.get() == tracker; }), list.end());
  return XR_SUCCESS;
}

MVR_API xrLocateHandJointsEXT(XrHandTrackerEXT handle, const XrHandJointsLocateInfoEXT *info,
                              XrHandJointLocationsEXT *locations) {
  HandTracker *tracker = lookup<HandTracker>(handle);
  if (!tracker)
    return XR_ERROR_HANDLE_INVALID;
  Session *session = tracker->session;
  Instance *instance = session->instance;
  std::lock_guard<std::recursive_mutex> lock(instance->mutex);
  if (!info || !locations || info->type != XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT ||
      locations->type != XR_TYPE_HAND_JOINT_LOCATIONS_EXT || !locations->jointLocations)
    return XR_ERROR_VALIDATION_FAILURE;
  if (locations->jointCount != XR_HAND_JOINT_COUNT_EXT)
    return XR_ERROR_VALIDATION_FAILURE;
  const Space *base = lookup<Space>(info->baseSpace);
  if (!base)
    return XR_ERROR_HANDLE_INVALID;
  if (info->time <= 0)
    return XR_ERROR_TIME_INVALID;
  MVR_ALIVE();

  XrHandJointVelocitiesEXT *velocities = nullptr;
  for (auto *next = static_cast<XrBaseOutStructure *>(locations->next); next; next = next->next)
    if (next->type == XR_TYPE_HAND_JOINT_VELOCITIES_EXT)
      velocities = reinterpret_cast<XrHandJointVelocitiesEXT *>(next);
  if (velocities && (velocities->jointCount != XR_HAND_JOINT_COUNT_EXT || !velocities->jointVelocities))
    return XR_ERROR_VALIDATION_FAILURE;

  locations->isActive = XR_FALSE;
  for (uint32_t j = 0; j < XR_HAND_JOINT_COUNT_EXT; ++j) {
    locations->jointLocations[j].locationFlags = 0;
    locations->jointLocations[j].pose = {{0, 0, 0, 1}, {0, 0, 0}};
    locations->jointLocations[j].radius = 0;
    if (velocities)
      velocities->jointVelocities[j].velocityFlags = 0; // the server sends no joint velocities
  }

  mvr_hands_request request{info->time};
  mvr_hands_response response{};
  if (!instance->connection.call(MVR_MSG_HANDS, request, response) || response.result != MVR_OK) {
    instance->markLost();
    return XR_ERROR_INSTANCE_LOST;
  }
  const mvr_hand &hand = response.hands[tracker->hand];
  if (!(hand.flags & MVR_HAND_TRACKED))
    return XR_SUCCESS;

  // the joints are in the tracking space; the base space is where the server's tracking for that time puts it
  mvr_tracking_response sample{};
  if (!session->tracking(info->time, sample))
    return XR_ERROR_INSTANCE_LOST;
  const Located located = session->locate(*base, sample);
  if (!located.orientation_valid || !located.position_valid)
    return XR_SUCCESS;
  const mvr_pose to_base = poseInverse(located.pose);
  constexpr XrSpaceLocationFlags kValid = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
                                          XR_SPACE_LOCATION_POSITION_VALID_BIT |
                                          XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT |
                                          XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
  for (uint32_t j = 0; j < XR_HAND_JOINT_COUNT_EXT; ++j) {
    const mvr_pose p = poseMul(to_base, hand.joints[j]);
    XrHandJointLocationEXT &out = locations->jointLocations[j];
    out.pose.orientation = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
    out.pose.position = {p.position.x, p.position.y, p.position.z};
    out.radius = hand.radii[j];
    out.locationFlags = kValid;
  }
  locations->isActive = XR_TRUE;
  return XR_SUCCESS;
}

} // namespace api
} // namespace metalvr
