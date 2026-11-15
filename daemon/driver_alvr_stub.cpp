// Built in place of driver_alvr.cpp when MetalVR is configured without ALVR.
#include "driver.h"

namespace metalvr {

std::unique_ptr<Driver> makeAlvrDriver(const AlvrOptions &) { return nullptr; }

} // namespace metalvr
