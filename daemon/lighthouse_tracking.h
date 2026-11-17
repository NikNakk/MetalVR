// Position tracking for the Lighthouse headsets with their base stations (docs/HEADSETS.md): libsurvive (MIT,
// https://github.com/cntools/libsurvive), built by scripts/build-libsurvive.sh and linked when it is there. It reads
// the headset's and the controllers' light sensors through hidapi, solves where each is and where the base stations
// stand, and this hands those poses over in OpenXR's axes (+y up, -z forward), in the tracker's own space.
//
// The Lighthouse backend keeps the rest of the headset (its lenses, its IMU, which turns the head, its controllers'
// inputs); the driver takes positions from here, and pulls the head's heading toward the tracker's, as it does with
// a Rift's sensors.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "metalvr_protocol.h"

namespace metalvr {

struct TrackedPose {
  mvr_quat orientation{0, 0, 0, 1};
  mvr_vec3 position{}, velocity{};
  bool position_tracked = false; // the base stations saw it in the last half second
};

class LighthouseTracking {
public:
  virtual ~LighthouseTracking() = default;
  enum Device { kHead = 0, kLeft = 1, kRight = 2 };
  // the latest pose of a device; false while libsurvive has none
  virtual bool pose(int device, TrackedPose &out) = 0;
  // the base stations libsurvive knows, where it has them
  virtual uint32_t references(mvr_tracking_reference *out) = 0;
  // every object libsurvive has a pose for, by its name (for checking)
  virtual std::vector<std::pair<std::string, TrackedPose>> objects() = 0;
};

// libsurvive, started: nothing if MetalVR was built without it or it does not start. `simulated`: libsurvive's own
// simulated device and base stations instead of USB, for checking the whole path without a headset.
std::unique_ptr<LighthouseTracking> startLighthouseTracking(bool simulated = false);
bool lighthouseTrackingBuilt();

} // namespace metalvr
