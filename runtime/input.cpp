// Actions: sets, suggested bindings, state, action spaces and haptics.
//
// The server says which controllers the user holds (mvr_system_info.controllers). Their own interaction profile is
// preferred when the application suggests bindings for it; every other profile is read as a naming of the same
// physical inputs, so a game written for other controllers still gets working bindings without the runtime
// pretending to be that hardware.
#include "api.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace metalvr {

namespace {

constexpr const char *kTouchPlusProfile = "/interaction_profiles/meta/touch_plus_controller";
constexpr const char *kTouchPlusExtensionProfile = "/interaction_profiles/meta/touch_controller_plus";
constexpr const char *kFrameProfile = "/interaction_profiles/valve/frame_controller_valve";

// the profiles this runtime binds, most specific layout first, and whether each has a thumbstick of its own
struct Profile {
  const char *path;
  bool has_thumbstick;
};
const Profile kProfiles[] = {
    {"/interaction_profiles/oculus/touch_controller", true},
    {"/interaction_profiles/valve/index_controller", true},
    {"/interaction_profiles/microsoft/motion_controller", true},
    {"/interaction_profiles/htc/vive_controller", false},
    {"/interaction_profiles/khr/simple_controller", false},
    {kTouchPlusProfile, true},
    {kTouchPlusExtensionProfile, true},
    {kFrameProfile, true},
};

// the profiles that name the controllers of a kind as they are, best first
std::vector<const char *> nativeProfiles(uint32_t controllers) {
  switch (controllers) {
  case MVR_CONTROLLERS_TOUCH:
  // the Sense controllers have no OpenXR profile of their own: Touch, which has their layout (Monado binds them so)
  case MVR_CONTROLLERS_PS_SENSE:
    return {"/interaction_profiles/oculus/touch_controller"};
  case MVR_CONTROLLERS_TOUCH_PLUS:
    return {kTouchPlusProfile, kTouchPlusExtensionProfile, "/interaction_profiles/oculus/touch_controller"};
  case MVR_CONTROLLERS_INDEX:
    return {"/interaction_profiles/valve/index_controller"};
  case MVR_CONTROLLERS_VIVE_WAND:
    return {"/interaction_profiles/htc/vive_controller"};
  case MVR_CONTROLLERS_STEAM_FRAME:
    return {kFrameProfile};
  default:
    return {};
  }
}

bool hasThumbstick(uint32_t controllers) { return controllers != MVR_CONTROLLERS_VIVE_WAND; }
bool hasTrackpad(uint32_t controllers) {
  return controllers == MVR_CONTROLLERS_INDEX || controllers == MVR_CONTROLLERS_VIVE_WAND;
}

// The order the profiles are tried in: METALVR_INTERACTION_PROFILE names one to try first, for a game whose
// bindings for another controller are the better ones ("/interaction_profiles/htc/vive_controller"); then the
// profiles of the controllers in the user's hands; then the rest.
std::vector<Profile> profileOrder(uint32_t controllers) {
  std::vector<Profile> order;
  const auto add = [&](const char *path) {
    for (const Profile &have : order)
      if (!std::strcmp(have.path, path))
        return;
    for (const Profile &profile : kProfiles)
      if (!std::strcmp(profile.path, path))
        order.push_back(profile);
  };
  const char *preferred = std::getenv("METALVR_INTERACTION_PROFILE");
  if (preferred && *preferred) {
    bool known = false;
    for (const Profile &profile : kProfiles)
      if (!std::strcmp(profile.path, preferred)) {
        order.push_back(profile);
        known = true;
      }
    if (!known)
      logLine("METALVR_INTERACTION_PROFILE=%s is not a profile this runtime binds; ignored", preferred);
  }
  for (const char *path : nativeProfiles(controllers))
    add(path);
  for (const Profile &profile : kProfiles)
    add(profile.path);
  return order;
}

bool startsWith(const std::string &s, const char *prefix) { return s.rfind(prefix, 0) == 0; }

// "/user/hand/left/input/trigger/value" -> hand, "trigger", "value"
bool parseBinding(const std::string &path, int &hand, std::string &identifier, std::string &component) {
  std::string rest;
  if (startsWith(path, "/user/hand/left/")) {
    hand = 0;
    rest = path.substr(16);
  } else if (startsWith(path, "/user/hand/right/")) {
    hand = 1;
    rest = path.substr(17);
  } else {
    return false;
  }
  if (startsWith(rest, "input/"))
    rest = rest.substr(6);
  else if (startsWith(rest, "output/"))
    rest = rest.substr(7);
  else
    return false;
  const size_t slash = rest.find('/');
  identifier = rest.substr(0, slash);
  component = slash == std::string::npos ? std::string() : rest.substr(slash + 1);
  return !identifier.empty();
}

struct PadSources {
  Source both, x, y, click, touch, force;
};
constexpr PadSources kStick = {Source::Thumbstick,      Source::ThumbstickX,     Source::ThumbstickY,
                               Source::ThumbstickClick, Source::ThumbstickTouch, Source::None};
constexpr PadSources kPad = {Source::Trackpad,      Source::TrackpadX,     Source::TrackpadY,
                             Source::TrackpadClick, Source::TrackpadTouch, Source::TrackpadForce};

Source padSource(const PadSources &pad, const std::string &component, XrActionType type) {
  if (component == "x")
    return pad.x;
  if (component == "y")
    return pad.y;
  if (component == "click")
    return pad.click;
  if (component == "touch")
    return pad.touch;
  if (component == "force")
    return pad.force;
  return type == XR_ACTION_TYPE_VECTOR2F_INPUT ? pad.both
         : type == XR_ACTION_TYPE_BOOLEAN_INPUT ? pad.click
                                                : Source::None;
}

// What a binding of a profile reads on the controllers in the user's hands. `has_thumbstick` is whether the
// profile names a thumbstick of its own; `controllers` is the physical kind (mvr_controller_kind).
Source sourceFor(const std::string &identifier, const std::string &component, XrActionType type,
                 bool has_thumbstick, const char *profile, uint32_t controllers, int hand) {
  const bool wants_bool = type == XR_ACTION_TYPE_BOOLEAN_INPUT;
  if (identifier == "trackpad") {
    // a trackpad of its own (Index, Vive) is read as itself
    if (hasTrackpad(controllers))
      return padSource(kPad, component, type);
    // On a controller with no trackpad, the trackpad of a profile without a stick (Vive's) is where the thumb
    // steers, and it is read as the stick. Of a profile that has both, it is a second input these controllers do
    // not have: bound to the stick as well, one push would fire the game's stick and trackpad actions together.
    return has_thumbstick ? Source::None : padSource(kStick, component, type);
  }
  if (identifier == "thumbstick")
    // a game written for sticks, on Vive wands: the thumb steers on the trackpad
    return padSource(hasThumbstick(controllers) ? kStick : kPad, component, type);
  if (identifier == "trigger" || identifier == "select") {
    if (component == "click")
      return Source::TriggerClick;
    if (component == "touch" || component == "proximity")
      return Source::TriggerTouch;
    if (component == "value")
      return Source::TriggerValue;
    if (component == "force")
      return Source::None; // Touch Plus: no controller reports it through the server
    return wants_bool ? Source::TriggerClick : Source::TriggerValue;
  }
  if (identifier == "trigger_curl" || identifier == "trigger_slide")
    return Source::None;
  if (identifier == "squeeze") {
    if (component == "click")
      return Source::SqueezeClick;
    if (component == "force")
      return controllers == MVR_CONTROLLERS_INDEX ? Source::SqueezeForce : Source::SqueezeValue;
    if (component == "value")
      return Source::SqueezeValue;
    return wants_bool ? Source::SqueezeClick : Source::SqueezeValue;
  }
  if (identifier == "bumper")
    return component == "touch" ? Source::BumperTouch : Source::BumperClick;
  if (identifier == "dpad_up")
    return Source::DpadUp;
  if (identifier == "dpad_down")
    return Source::DpadDown;
  if (identifier == "dpad_left")
    return Source::DpadLeft;
  if (identifier == "dpad_right")
    return Source::DpadRight;
  if (identifier == "view")
    return Source::ViewClick;
  // the Steam Frame's right controller has X and Y beside A and B
  const bool frame_right = hand == 1 && !std::strcmp(profile, kFrameProfile);
  if (frame_right && identifier == "x")
    return component == "touch" ? Source::TertiaryTouch : Source::TertiaryClick;
  if (frame_right && identifier == "y")
    return component == "touch" ? Source::QuaternaryTouch : Source::QuaternaryClick;
  if (identifier == "a" || identifier == "x")
    return component == "touch" ? Source::PrimaryTouch : Source::PrimaryClick;
  if (identifier == "b" || identifier == "y")
    return component == "touch" ? Source::SecondaryTouch : Source::SecondaryClick;
  if (identifier == "menu")
    return Source::MenuClick;
  if (identifier == "system")
    return component == "touch" ? Source::None : Source::SystemClick;
  if (identifier == "thumbrest" || identifier == "thumb_resting_surfaces")
    return Source::ThumbrestTouch;
  if (identifier == "grip" || identifier == "palm_ext" || identifier == "grip_surface")
    return Source::GripPose;
  if (identifier == "aim")
    return Source::AimPose;
  if (identifier == "haptic")
    return Source::Haptic;
  return Source::None;
}

// the value of a physical input: digital inputs are 0 or 1
void readSource(const mvr_controller_state &c, Source source, float &x, float &y) {
  x = y = 0;
  const auto bit = [](uint32_t mask, uint32_t flag) { return (mask & flag) ? 1.0f : 0.0f; };
  switch (source) {
  case Source::TriggerValue:
    x = c.trigger;
    break;
  case Source::TriggerClick:
    x = (c.buttons & MVR_BUTTON_TRIGGER) || c.trigger > 0.7f ? 1.0f : 0.0f;
    break;
  case Source::TriggerTouch:
    x = (c.touches & MVR_BUTTON_TRIGGER) || c.trigger > 0.05f ? 1.0f : 0.0f;
    break;
  case Source::SqueezeValue:
    x = c.squeeze;
    break;
  case Source::SqueezeClick:
    x = (c.buttons & MVR_BUTTON_SQUEEZE) || c.squeeze > 0.7f ? 1.0f : 0.0f;
    break;
  case Source::Thumbstick:
    x = c.thumbstick_x;
    y = c.thumbstick_y;
    break;
  case Source::ThumbstickX:
    x = c.thumbstick_x;
    break;
  case Source::ThumbstickY:
    x = c.thumbstick_y;
    break;
  case Source::ThumbstickClick:
    x = bit(c.buttons, MVR_BUTTON_THUMBSTICK);
    break;
  case Source::ThumbstickTouch:
    x = bit(c.touches, MVR_BUTTON_THUMBSTICK);
    break;
  case Source::PrimaryClick:
    x = bit(c.buttons, MVR_BUTTON_PRIMARY);
    break;
  case Source::PrimaryTouch:
    x = bit(c.touches, MVR_BUTTON_PRIMARY);
    break;
  case Source::SecondaryClick:
    x = bit(c.buttons, MVR_BUTTON_SECONDARY);
    break;
  case Source::SecondaryTouch:
    x = bit(c.touches, MVR_BUTTON_SECONDARY);
    break;
  case Source::MenuClick:
    x = bit(c.buttons, MVR_BUTTON_MENU);
    break;
  case Source::SystemClick:
    x = bit(c.buttons, MVR_BUTTON_SYSTEM);
    break;
  case Source::ThumbrestTouch:
    x = bit(c.touches, MVR_BUTTON_THUMBREST);
    break;
  case Source::Trackpad:
    x = c.trackpad_x;
    y = c.trackpad_y;
    break;
  case Source::TrackpadX:
    x = c.trackpad_x;
    break;
  case Source::TrackpadY:
    x = c.trackpad_y;
    break;
  case Source::TrackpadClick:
    x = bit(c.buttons, MVR_BUTTON_TRACKPAD);
    break;
  case Source::TrackpadTouch:
    x = bit(c.touches, MVR_BUTTON_TRACKPAD);
    break;
  case Source::TrackpadForce:
    x = c.trackpad_force;
    break;
  case Source::SqueezeForce:
    x = c.squeeze_force;
    break;
  case Source::BumperClick:
    x = bit(c.buttons, MVR_BUTTON_BUMPER);
    break;
  case Source::BumperTouch:
    x = bit(c.touches | c.buttons, MVR_BUTTON_BUMPER);
    break;
  case Source::DpadUp:
    x = bit(c.buttons, MVR_BUTTON_DPAD_UP);
    break;
  case Source::DpadDown:
    x = bit(c.buttons, MVR_BUTTON_DPAD_DOWN);
    break;
  case Source::DpadLeft:
    x = bit(c.buttons, MVR_BUTTON_DPAD_LEFT);
    break;
  case Source::DpadRight:
    x = bit(c.buttons, MVR_BUTTON_DPAD_RIGHT);
    break;
  case Source::ViewClick:
    x = bit(c.buttons, MVR_BUTTON_VIEW);
    break;
  case Source::TertiaryClick:
    x = bit(c.buttons, MVR_BUTTON_TERTIARY);
    break;
  case Source::TertiaryTouch:
    x = bit(c.touches, MVR_BUTTON_TERTIARY);
    break;
  case Source::QuaternaryClick:
    x = bit(c.buttons, MVR_BUTTON_QUATERNARY);
    break;
  case Source::QuaternaryTouch:
    x = bit(c.touches, MVR_BUTTON_QUATERNARY);
    break;
  default:
    break;
  }
}

bool isInputSource(Source source) {
  return source != Source::None && source != Source::GripPose && source != Source::AimPose &&
         source != Source::Haptic;
}

int handOf(Instance &instance, XrPath subaction) {
  const std::string *string = instance.pathString(subaction);
  if (!string)
    return -1;
  if (*string == "/user/hand/left")
    return 0;
  if (*string == "/user/hand/right")
    return 1;
  return -1;
}

bool nameValid(const char *name, size_t max) {
  const size_t length = strnlen(name, max);
  if (length == 0 || length >= max)
    return false;
  for (size_t i = 0; i < length; ++i) {
    const char c = name[i];
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.'))
      return false;
  }
  return true;
}

// resolves an action handle and the hands a subaction path selects; hands[h] is whether hand h is asked for
XrResult resolveAction(XrAction handle, XrPath subaction, XrActionType type, Session &session, Action *&action,
                       bool hands[2], bool &combined) {
  action = lookup<Action>(handle);
  if (!action)
    return XR_ERROR_HANDLE_INVALID;
  if (!action->set->attached || !session.action_sets_attached ||
      std::find(session.attached_sets.begin(), session.attached_sets.end(), action->set) ==
          session.attached_sets.end())
    return XR_ERROR_ACTIONSET_NOT_ATTACHED;
  if (action->action_type != type)
    return XR_ERROR_ACTION_TYPE_MISMATCH;
  combined = subaction == XR_NULL_PATH;
  hands[0] = hands[1] = combined;
  if (!combined) {
    if (std::find(action->subaction_paths.begin(), action->subaction_paths.end(), subaction) ==
        action->subaction_paths.end())
      return XR_ERROR_PATH_UNSUPPORTED;
    const int hand = handOf(*session.instance, subaction);
    if (hand >= 0)
      hands[hand] = true;
  }
  return XR_SUCCESS;
}

const ActionState &stateFor(const Action &action, const bool hands[2], bool combined) {
  if (combined)
    return action.combined;
  return hands[0] ? action.state[0] : action.state[1];
}

} // namespace

// Chooses each hand's interaction profile and turns the suggested bindings of that profile into physical sources.
void attachBindings(Session &session) {
  Instance &instance = *session.instance;
  for (int hand = 0; hand < 2; ++hand)
    session.active_profile[hand] = XR_NULL_PATH;

  const uint32_t controllers = instance.system.controllers;
  for (const Profile &profile : profileOrder(controllers)) {
    auto id = instance.path_ids.find(profile.path);
    if (id == instance.path_ids.end())
      continue;
    auto suggested = instance.suggested.find(id->second);
    if (suggested == instance.suggested.end())
      continue;
    for (int hand = 0; hand < 2; ++hand) {
      if (session.active_profile[hand] != XR_NULL_PATH)
        continue;
      bool bound = false;
      for (const SuggestedBinding &binding : suggested->second) {
        if (!binding.action->set->attached)
          continue;
        const std::string *path = instance.pathString(binding.binding);
        int binding_hand = 0;
        std::string identifier, component;
        if (!path || !parseBinding(*path, binding_hand, identifier, component) || binding_hand != hand)
          continue;
        const Source source = sourceFor(identifier, component, binding.action->action_type, profile.has_thumbstick,
                                        profile.path, controllers, hand);
        if (source == Source::None)
          continue;
        binding.action->bindings.push_back({hand, source, binding.binding});
        bound = true;
      }
      if (bound)
        session.active_profile[hand] = id->second;
    }
  }
}

Located locateActionSpace(Session &session, const Space &space, const mvr_tracking_response &sample) {
  Located result;
  const Action *action = space.action;
  if (!action || !lookup<Action>(reinterpret_cast<XrAction>(space.action)))
    return result;
  const int wanted = space.subaction == XR_NULL_PATH ? -1 : handOf(*session.instance, space.subaction);
  for (const Binding &binding : action->bindings) {
    if (binding.source != Source::GripPose && binding.source != Source::AimPose)
      continue;
    if (wanted >= 0 && binding.hand != wanted)
      continue;
    const mvr_controller_state &controller = sample.controllers[binding.hand];
    if (!(controller.flags & MVR_CONTROLLER_CONNECTED))
      continue;
    result.pose = binding.source == Source::AimPose ? controller.aim : controller.grip;
    const uint32_t flags = controller.flags;
    const bool valid = (flags & MVR_CONTROLLER_POSE_VALID) && quatIsValid(result.pose.orientation);
    result.position_valid = valid;
    if (flags & MVR_CONTROLLER_SPLIT_POSE) {
      result.orientation_valid = (flags & MVR_CONTROLLER_ORIENTATION_VALID) && quatIsValid(result.pose.orientation);
      result.orientation_tracked = result.orientation_valid && (flags & MVR_CONTROLLER_ORIENTATION_TRACKED);
      result.position_tracked = valid && (flags & MVR_CONTROLLER_POSITION_TRACKED);
      result.linear_velocity_valid = result.position_valid && (flags & MVR_CONTROLLER_LINEAR_VELOCITY_VALID);
      result.angular_velocity_valid = result.orientation_valid && (flags & MVR_CONTROLLER_ANGULAR_VELOCITY_VALID);
    } else {
      result.orientation_valid = result.orientation_tracked = result.position_tracked = valid;
      result.linear_velocity_valid = result.angular_velocity_valid = valid;
    }
    result.linear_velocity = controller.linear_velocity;
    result.angular_velocity = controller.angular_velocity;
    break;
  }
  return result;
}

namespace api {

MVR_API xrCreateActionSet(XrInstance handle, const XrActionSetCreateInfo *info, XrActionSet *out) {
  MVR_INSTANCE(handle);
  if (!info || !out || info->type != XR_TYPE_ACTION_SET_CREATE_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!nameValid(info->actionSetName, XR_MAX_ACTION_SET_NAME_SIZE))
    return XR_ERROR_PATH_FORMAT_INVALID;
  const size_t localized = strnlen(info->localizedActionSetName, XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE);
  if (localized == 0 || localized >= XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE)
    return XR_ERROR_LOCALIZED_NAME_INVALID;
  for (const auto &set : instance->action_sets) {
    if (set->name == info->actionSetName)
      return XR_ERROR_NAME_DUPLICATED;
    if (set->localized_name == info->localizedActionSetName)
      return XR_ERROR_LOCALIZED_NAME_DUPLICATED;
  }
  auto set = std::make_unique<ActionSet>();
  set->instance = instance;
  set->name = info->actionSetName;
  set->localized_name = info->localizedActionSetName;
  set->priority = info->priority;
  *out = reinterpret_cast<XrActionSet>(set.get());
  instance->action_sets.push_back(std::move(set));
  return XR_SUCCESS;
}

MVR_API xrDestroyActionSet(XrActionSet handle) {
  ActionSet *set = lookup<ActionSet>(handle);
  if (!set)
    return XR_ERROR_HANDLE_INVALID;
  Instance *instance = set->instance;
  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex);
  for (auto &entry : instance->suggested) {
    auto &list = entry.second;
    list.erase(std::remove_if(list.begin(), list.end(),
                              [&](const SuggestedBinding &binding) { return binding.action->set == set; }),
               list.end());
  }
  if (instance->session) {
    auto &attached = instance->session->attached_sets;
    attached.erase(std::remove(attached.begin(), attached.end(), set), attached.end());
  }
  auto &sets = instance->action_sets;
  sets.erase(std::remove_if(sets.begin(), sets.end(), [&](const auto &owned) { return owned.get() == set; }),
             sets.end());
  return XR_SUCCESS;
}

MVR_API xrCreateAction(XrActionSet handle, const XrActionCreateInfo *info, XrAction *out) {
  ActionSet *set = lookup<ActionSet>(handle);
  if (!set)
    return XR_ERROR_HANDLE_INVALID;
  Instance *instance = set->instance;
  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex);
  if (!info || !out || info->type != XR_TYPE_ACTION_CREATE_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (set->attached)
    return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
  if (!nameValid(info->actionName, XR_MAX_ACTION_NAME_SIZE))
    return XR_ERROR_PATH_FORMAT_INVALID;
  const size_t localized = strnlen(info->localizedActionName, XR_MAX_LOCALIZED_ACTION_NAME_SIZE);
  if (localized == 0 || localized >= XR_MAX_LOCALIZED_ACTION_NAME_SIZE)
    return XR_ERROR_LOCALIZED_NAME_INVALID;
  switch (info->actionType) {
  case XR_ACTION_TYPE_BOOLEAN_INPUT:
  case XR_ACTION_TYPE_FLOAT_INPUT:
  case XR_ACTION_TYPE_VECTOR2F_INPUT:
  case XR_ACTION_TYPE_POSE_INPUT:
  case XR_ACTION_TYPE_VIBRATION_OUTPUT:
    break;
  default:
    return XR_ERROR_VALIDATION_FAILURE;
  }
  for (const auto &action : set->actions) {
    if (action->name == info->actionName)
      return XR_ERROR_NAME_DUPLICATED;
    if (action->localized_name == info->localizedActionName)
      return XR_ERROR_LOCALIZED_NAME_DUPLICATED;
  }
  auto action = std::make_unique<Action>();
  action->set = set;
  action->action_type = info->actionType;
  action->name = info->actionName;
  action->localized_name = info->localizedActionName;
  for (uint32_t i = 0; i < info->countSubactionPaths; ++i) {
    const XrPath path = info->subactionPaths[i];
    if (!instance->pathString(path))
      return XR_ERROR_PATH_INVALID;
    if (std::find(action->subaction_paths.begin(), action->subaction_paths.end(), path) !=
        action->subaction_paths.end())
      return XR_ERROR_PATH_UNSUPPORTED;
    action->subaction_paths.push_back(path);
  }
  *out = reinterpret_cast<XrAction>(action.get());
  set->actions.push_back(std::move(action));
  return XR_SUCCESS;
}

MVR_API xrDestroyAction(XrAction handle) {
  Action *action = lookup<Action>(handle);
  if (!action)
    return XR_ERROR_HANDLE_INVALID;
  ActionSet *set = action->set;
  Instance *instance = set->instance;
  std::lock_guard<std::recursive_mutex> instance_lock(instance->mutex);
  for (auto &entry : instance->suggested) {
    auto &list = entry.second;
    list.erase(std::remove_if(list.begin(), list.end(),
                              [&](const SuggestedBinding &binding) { return binding.action == action; }),
               list.end());
  }
  auto &actions = set->actions;
  actions.erase(
      std::remove_if(actions.begin(), actions.end(), [&](const auto &owned) { return owned.get() == action; }),
      actions.end());
  return XR_SUCCESS;
}

MVR_API xrSuggestInteractionProfileBindings(XrInstance handle, const XrInteractionProfileSuggestedBinding *info) {
  MVR_INSTANCE(handle);
  if (!info || info->type != XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING || !info->countSuggestedBindings ||
      !info->suggestedBindings)
    return XR_ERROR_VALIDATION_FAILURE;
  const std::string *profile = instance->pathString(info->interactionProfile);
  if (!profile)
    return XR_ERROR_PATH_INVALID;
  if (!startsWith(*profile, "/interaction_profiles/"))
    return XR_ERROR_PATH_UNSUPPORTED;
  std::vector<SuggestedBinding> bindings;
  for (uint32_t i = 0; i < info->countSuggestedBindings; ++i) {
    Action *action = lookup<Action>(info->suggestedBindings[i].action);
    if (!action)
      return XR_ERROR_HANDLE_INVALID;
    if (action->set->attached)
      return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
    if (!instance->pathString(info->suggestedBindings[i].binding))
      return XR_ERROR_PATH_INVALID;
    bindings.push_back({action, info->suggestedBindings[i].binding});
  }
  // a later suggestion for the same profile replaces the earlier one. Profiles this runtime does not bind are
  // kept and ignored: an application that suggests for every controller it knows must not fail here.
  instance->suggested[info->interactionProfile] = std::move(bindings);
  return XR_SUCCESS;
}

MVR_API xrAttachSessionActionSets(XrSession handle, const XrSessionActionSetsAttachInfo *info) {
  MVR_SESSION(handle);
  if (!info || info->type != XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO || !info->countActionSets || !info->actionSets)
    return XR_ERROR_VALIDATION_FAILURE;
  if (session->action_sets_attached)
    return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
  std::vector<ActionSet *> sets;
  for (uint32_t i = 0; i < info->countActionSets; ++i) {
    ActionSet *set = lookup<ActionSet>(info->actionSets[i]);
    if (!set)
      return XR_ERROR_HANDLE_INVALID;
    sets.push_back(set);
  }
  for (ActionSet *set : sets) {
    set->attached = true;
    for (auto &action : set->actions) {
      action->bindings.clear();
      action->state[0] = action->state[1] = action->combined = ActionState{};
    }
  }
  session->attached_sets = std::move(sets);
  session->action_sets_attached = true;
  attachBindings(*session);
  return XR_SUCCESS;
}

MVR_API xrGetCurrentInteractionProfile(XrSession handle, XrPath top_level, XrInteractionProfileState *state) {
  MVR_SESSION(handle);
  if (!state || state->type != XR_TYPE_INTERACTION_PROFILE_STATE)
    return XR_ERROR_VALIDATION_FAILURE;
  const std::string *path = instance->pathString(top_level);
  if (!path)
    return XR_ERROR_PATH_INVALID;
  if (!session->action_sets_attached)
    return XR_ERROR_ACTIONSET_NOT_ATTACHED;
  const int hand = handOf(*instance, top_level);
  if (hand < 0) {
    if (*path == "/user/head" || *path == "/user/gamepad" || startsWith(*path, "/user/treadmill") ||
        startsWith(*path, "/user/eyes_ext")) {
      state->interactionProfile = XR_NULL_PATH;
      return XR_SUCCESS;
    }
    return XR_ERROR_PATH_UNSUPPORTED;
  }
  // a profile becomes current with the first sync, when the application is told
  state->interactionProfile = session->profile_announced ? session->active_profile[hand] : XR_NULL_PATH;
  return XR_SUCCESS;
}

MVR_API xrSyncActions(XrSession handle, const XrActionsSyncInfo *info) {
  MVR_SESSION(handle);
  if (!info || info->type != XR_TYPE_ACTIONS_SYNC_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (info->countActiveActionSets && !info->activeActionSets)
    return XR_ERROR_VALIDATION_FAILURE;
  std::vector<std::pair<ActionSet *, XrPath>> active;
  for (uint32_t i = 0; i < info->countActiveActionSets; ++i) {
    ActionSet *set = lookup<ActionSet>(info->activeActionSets[i].actionSet);
    if (!set)
      return XR_ERROR_HANDLE_INVALID;
    if (std::find(session->attached_sets.begin(), session->attached_sets.end(), set) == session->attached_sets.end())
      return XR_ERROR_ACTIONSET_NOT_ATTACHED;
    active.push_back({set, info->activeActionSets[i].subactionPath});
  }
  MVR_ALIVE();

  if (!session->profile_announced && session->action_sets_attached) {
    session->profile_announced = true;
    if (session->active_profile[0] != XR_NULL_PATH || session->active_profile[1] != XR_NULL_PATH) {
      XrEventDataInteractionProfileChanged event{XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED};
      event.session = handle;
      instance->pushEvent(event);
    }
  }

  // input reaches an application only while it has focus
  const bool focused = session->state == XR_SESSION_STATE_FOCUSED;
  mvr_tracking_response sample{};
  if (!session->tracking(0, sample))
    return XR_ERROR_INSTANCE_LOST;
  session->input = sample;
  session->input_valid = true;
  const XrTime now = sample.time_ns;

  for (ActionSet *set : session->attached_sets) {
    // which hands this set is synced for: none if the set is not active
    bool set_hands[2] = {false, false};
    for (const auto &entry : active) {
      if (entry.first != set)
        continue;
      if (entry.second == XR_NULL_PATH) {
        set_hands[0] = set_hands[1] = true;
      } else {
        const int hand = handOf(*instance, entry.second);
        if (hand >= 0)
          set_hands[hand] = true;
      }
    }
    for (auto &action : set->actions) {
      if (action->action_type == XR_ACTION_TYPE_VIBRATION_OUTPUT)
        continue;
      for (int hand = 0; hand < 2; ++hand) {
        ActionState next;
        const mvr_controller_state &controller = sample.controllers[hand];
        const bool usable = focused && set_hands[hand] && (controller.flags & MVR_CONTROLLER_CONNECTED);
        for (const Binding &binding : action->bindings) {
          if (binding.hand != hand || !usable)
            continue;
          if (action->action_type == XR_ACTION_TYPE_POSE_INPUT) {
            if (binding.source == Source::GripPose || binding.source == Source::AimPose)
              next.active = true;
            continue;
          }
          if (!isInputSource(binding.source))
            continue;
          float x = 0, y = 0;
          readSource(controller, binding.source, x, y);
          if (action->action_type == XR_ACTION_TYPE_BOOLEAN_INPUT) {
            // an analog input is pressed past the middle of its travel
            const bool pressed = std::fabs(x) > 0.5f || std::fabs(y) > 0.5f;
            x = pressed ? 1.0f : 0.0f;
            y = 0;
          }
          // several inputs on one action: the one furthest from rest wins
          if (!next.active || x * x + y * y > next.x * next.x + next.y * next.y) {
            next.x = x;
            next.y = y;
          }
          next.active = true;
        }
        ActionState &current = action->state[hand];
        next.changed = current.active && next.active && (next.x != current.x || next.y != current.y);
        next.last_change = next.changed ? now : (next.active ? current.last_change : 0);
        current = next;
      }
      // with no subaction path the action is its hands together
      ActionState merged;
      for (int hand = 0; hand < 2; ++hand) {
        const ActionState &state = action->state[hand];
        if (!state.active)
          continue;
        if (!merged.active || state.x * state.x + state.y * state.y > merged.x * merged.x + merged.y * merged.y) {
          merged.x = state.x;
          merged.y = state.y;
        }
        merged.active = true;
      }
      merged.changed = action->combined.active && merged.active &&
                       (merged.x != action->combined.x || merged.y != action->combined.y);
      merged.last_change = merged.changed ? now : (merged.active ? action->combined.last_change : 0);
      action->combined = merged;
    }
  }
  return focused ? XR_SUCCESS : XR_SESSION_NOT_FOCUSED;
}

MVR_API xrGetActionStateBoolean(XrSession handle, const XrActionStateGetInfo *info, XrActionStateBoolean *state) {
  MVR_SESSION(handle);
  if (!info || !state || info->type != XR_TYPE_ACTION_STATE_GET_INFO || state->type != XR_TYPE_ACTION_STATE_BOOLEAN)
    return XR_ERROR_VALIDATION_FAILURE;
  Action *action = nullptr;
  bool hands[2], combined;
  const XrResult result =
      resolveAction(info->action, info->subactionPath, XR_ACTION_TYPE_BOOLEAN_INPUT, *session, action, hands, combined);
  if (XR_FAILED(result))
    return result;
  const ActionState &s = stateFor(*action, hands, combined);
  state->currentState = s.active && s.x != 0 ? XR_TRUE : XR_FALSE;
  state->changedSinceLastSync = s.changed ? XR_TRUE : XR_FALSE;
  state->lastChangeTime = s.last_change;
  state->isActive = s.active ? XR_TRUE : XR_FALSE;
  return XR_SUCCESS;
}

MVR_API xrGetActionStateFloat(XrSession handle, const XrActionStateGetInfo *info, XrActionStateFloat *state) {
  MVR_SESSION(handle);
  if (!info || !state || info->type != XR_TYPE_ACTION_STATE_GET_INFO || state->type != XR_TYPE_ACTION_STATE_FLOAT)
    return XR_ERROR_VALIDATION_FAILURE;
  Action *action = nullptr;
  bool hands[2], combined;
  const XrResult result =
      resolveAction(info->action, info->subactionPath, XR_ACTION_TYPE_FLOAT_INPUT, *session, action, hands, combined);
  if (XR_FAILED(result))
    return result;
  const ActionState &s = stateFor(*action, hands, combined);
  state->currentState = s.active ? s.x : 0.0f;
  state->changedSinceLastSync = s.changed ? XR_TRUE : XR_FALSE;
  state->lastChangeTime = s.last_change;
  state->isActive = s.active ? XR_TRUE : XR_FALSE;
  return XR_SUCCESS;
}

MVR_API xrGetActionStateVector2f(XrSession handle, const XrActionStateGetInfo *info, XrActionStateVector2f *state) {
  MVR_SESSION(handle);
  if (!info || !state || info->type != XR_TYPE_ACTION_STATE_GET_INFO || state->type != XR_TYPE_ACTION_STATE_VECTOR2F)
    return XR_ERROR_VALIDATION_FAILURE;
  Action *action = nullptr;
  bool hands[2], combined;
  const XrResult result = resolveAction(info->action, info->subactionPath, XR_ACTION_TYPE_VECTOR2F_INPUT, *session,
                                        action, hands, combined);
  if (XR_FAILED(result))
    return result;
  const ActionState &s = stateFor(*action, hands, combined);
  state->currentState = s.active ? XrVector2f{s.x, s.y} : XrVector2f{0, 0};
  state->changedSinceLastSync = s.changed ? XR_TRUE : XR_FALSE;
  state->lastChangeTime = s.last_change;
  state->isActive = s.active ? XR_TRUE : XR_FALSE;
  return XR_SUCCESS;
}

MVR_API xrGetActionStatePose(XrSession handle, const XrActionStateGetInfo *info, XrActionStatePose *state) {
  MVR_SESSION(handle);
  if (!info || !state || info->type != XR_TYPE_ACTION_STATE_GET_INFO || state->type != XR_TYPE_ACTION_STATE_POSE)
    return XR_ERROR_VALIDATION_FAILURE;
  Action *action = nullptr;
  bool hands[2], combined;
  const XrResult result =
      resolveAction(info->action, info->subactionPath, XR_ACTION_TYPE_POSE_INPUT, *session, action, hands, combined);
  if (XR_FAILED(result))
    return result;
  state->isActive = stateFor(*action, hands, combined).active ? XR_TRUE : XR_FALSE;
  return XR_SUCCESS;
}

MVR_API xrEnumerateBoundSourcesForAction(XrSession handle, const XrBoundSourcesForActionEnumerateInfo *info,
                                         uint32_t capacity, uint32_t *count, XrPath *sources) {
  MVR_SESSION(handle);
  if (!info || info->type != XR_TYPE_BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  Action *action = lookup<Action>(info->action);
  if (!action)
    return XR_ERROR_HANDLE_INVALID;
  if (!action->set->attached || !session->action_sets_attached)
    return XR_ERROR_ACTIONSET_NOT_ATTACHED;
  std::vector<XrPath> paths;
  for (const Binding &binding : action->bindings)
    paths.push_back(binding.path);
  return fillArray(capacity, count, sources, paths);
}

MVR_API xrGetInputSourceLocalizedName(XrSession handle, const XrInputSourceLocalizedNameGetInfo *info,
                                      uint32_t capacity, uint32_t *count, char *buffer) {
  MVR_SESSION(handle);
  if (!info || !count || info->type != XR_TYPE_INPUT_SOURCE_LOCALIZED_NAME_GET_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (!info->whichComponents)
    return XR_ERROR_VALIDATION_FAILURE;
  const std::string *path = instance->pathString(info->sourcePath);
  if (!path)
    return XR_ERROR_PATH_INVALID;
  if (!session->action_sets_attached)
    return XR_ERROR_ACTIONSET_NOT_ATTACHED;
  int hand = 0;
  std::string identifier, component;
  if (!parseBinding(*path, hand, identifier, component))
    return XR_ERROR_PATH_UNSUPPORTED;
  std::string name;
  const auto append = [&](const std::string &part) {
    if (!name.empty())
      name += ' ';
    name += part;
  };
  if (info->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_USER_PATH_BIT)
    append(hand == 0 ? "Left Hand" : "Right Hand");
  if (info->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_INTERACTION_PROFILE_BIT)
    append("Controller");
  if (info->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_COMPONENT_BIT) {
    std::string label = identifier;
    if (!label.empty())
      label[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(label[0])));
    append(label);
  }
  *count = static_cast<uint32_t>(name.size() + 1);
  if (capacity == 0)
    return XR_SUCCESS;
  if (capacity < *count)
    return XR_ERROR_SIZE_INSUFFICIENT;
  if (!buffer)
    return XR_ERROR_VALIDATION_FAILURE;
  std::memcpy(buffer, name.c_str(), *count);
  return XR_SUCCESS;
}

namespace {
XrResult sendHaptic(Session &session, const XrHapticActionInfo *info, float duration_s, float frequency,
                    float amplitude) {
  Instance *instance = session.instance;
  Action *action = nullptr;
  bool hands[2], combined;
  const XrResult result = resolveAction(info->action, info->subactionPath, XR_ACTION_TYPE_VIBRATION_OUTPUT, session,
                                        action, hands, combined);
  if (XR_FAILED(result))
    return result;
  if (instance->lost)
    return XR_ERROR_INSTANCE_LOST;
  if (session.state != XR_SESSION_STATE_FOCUSED)
    return XR_SESSION_NOT_FOCUSED;
  bool sent[2] = {false, false};
  for (const Binding &binding : action->bindings) {
    if (binding.source != Source::Haptic || !hands[binding.hand] || sent[binding.hand])
      continue;
    sent[binding.hand] = true;
    mvr_haptic_request request{static_cast<uint32_t>(binding.hand), duration_s, frequency, amplitude};
    mvr_result_response response{};
    if (!instance->connection.call(MVR_MSG_HAPTIC, request, response)) {
      instance->markLost();
      return XR_ERROR_INSTANCE_LOST;
    }
  }
  return XR_SUCCESS;
}
} // namespace

MVR_API xrApplyHapticFeedback(XrSession handle, const XrHapticActionInfo *info, const XrHapticBaseHeader *haptic) {
  MVR_SESSION(handle);
  if (!info || !haptic || info->type != XR_TYPE_HAPTIC_ACTION_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  if (haptic->type != XR_TYPE_HAPTIC_VIBRATION)
    return XR_ERROR_VALIDATION_FAILURE;
  const auto *vibration = reinterpret_cast<const XrHapticVibration *>(haptic);
  // XR_MIN_HAPTIC_DURATION asks for the shortest pulse the device makes
  const float duration = vibration->duration == XR_MIN_HAPTIC_DURATION ? 0.01f
                                                                        : static_cast<float>(vibration->duration / 1e9);
  return sendHaptic(*session, info, duration > 0 ? duration : 0.01f, vibration->frequency, vibration->amplitude);
}

MVR_API xrStopHapticFeedback(XrSession handle, const XrHapticActionInfo *info) {
  MVR_SESSION(handle);
  if (!info || info->type != XR_TYPE_HAPTIC_ACTION_INFO)
    return XR_ERROR_VALIDATION_FAILURE;
  return sendHaptic(*session, info, 0, 0, 0);
}

} // namespace api
} // namespace metalvr
