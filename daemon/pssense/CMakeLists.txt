cmake_minimum_required(VERSION 3.24)
project(MetalVRSense VERSION 0.1.0 LANGUAGES C CXX)

# Build the same device drivers and tracking pipeline as Monado, without its runtime or compositor.
include("${MONADO_SOURCE_DIR}/cmake/MonadoSetOptions.cmake")
monado_set_options(ENABLE XRT_BUILD_DRIVER_PSVR2 XRT_BUILD_DRIVER_PSSENSE
  XRT_HAVE_LIBUSB XRT_MODULE_CONSTELLATION_TRACKING)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_Ceres ON)
add_subdirectory("${MONADO_SOURCE_DIR}" monado EXCLUDE_FROM_ALL)
if(NOT TARGET drv_psvr2 OR NOT TARGET drv_pssense)
  message(FATAL_ERROR "Monado cannot build both PSVR2 and Sense drivers")
endif()

add_library(metalvr_pssense SHARED bridge.c)
target_include_directories(metalvr_pssense PRIVATE ../../protocol
  "${MONADO_SOURCE_DIR}/src/xrt/drivers")
target_link_libraries(metalvr_pssense PRIVATE target_instance_no_comp st_prober xrt-interfaces aux_os
  "-framework Foundation" "-framework CoreFoundation" "-framework IOKit" "-framework Security")
# Keep Monado's implementation symbols private to the bridge.
target_link_options(metalvr_pssense PRIVATE "-Wl,-exported_symbol,_mvr_pssense_get_bridge")
