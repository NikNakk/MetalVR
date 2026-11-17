# The ALVR driver needs ALVR's server_core, which scripts/build-alvr.sh builds into build/alvr. It is linked when
# that library is there, so a tree without it still builds a metalvrd that has only the simulated headset.
set(METALVR_ALVR_LIBRARY "${CMAKE_SOURCE_DIR}/build/alvr/target/release/libalvr_server_core.a"
    CACHE FILEPATH "libalvr_server_core built by scripts/build-alvr.sh")
set(METALVR_ALVR_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/build/alvr/include"
    CACHE PATH "directory of the generated alvr_server_core.h")
# what scripts/build-alvr.sh prints as ALVR_LINK_FLAGS; each framework is one token so repeated options survive
set(METALVR_ALVR_LINK_FLAGS
    "-Wl,-framework,CoreFoundation -lobjc -Wl,-framework,IOKit -Wl,-framework,AudioUnit -Wl,-framework,CoreAudio -Wl,-framework,Security -lbz2 -Wl,-framework,AppKit -Wl,-framework,Foundation -liconv"
    CACHE STRING "link flags the ALVR library needs")
if(EXISTS "${METALVR_ALVR_LIBRARY}" AND EXISTS "${METALVR_ALVR_INCLUDE_DIR}/alvr_server_core.h")
  set(alvr_default ON)
else()
  set(alvr_default OFF)
endif()
option(METALVR_WITH_ALVR "Link ALVR's server_core for streaming to a headset" ${alvr_default})

add_executable(metalvrd
  main.mm
  server.mm
  compositor.mm
  encoder.mm
  shared_texture.mm
  driver_sim.cpp
  driver_rift.mm
  window_capture.mm
)

if(METALVR_WITH_ALVR)
  if(NOT EXISTS "${METALVR_ALVR_LIBRARY}")
    message(FATAL_ERROR "METALVR_WITH_ALVR needs ${METALVR_ALVR_LIBRARY}; run scripts/build-alvr.sh")
  endif()
  message(STATUS "metalvrd: with the ALVR driver (${METALVR_ALVR_LIBRARY})")
  target_sources(metalvrd PRIVATE driver_alvr.cpp)
  target_include_directories(metalvrd PRIVATE "${METALVR_ALVR_INCLUDE_DIR}")
  target_link_libraries(metalvrd PRIVATE "${METALVR_ALVR_LIBRARY}")
  separate_arguments(alvr_flags NATIVE_COMMAND "${METALVR_ALVR_LINK_FLAGS}")
  target_link_options(metalvrd PRIVATE ${alvr_flags})
else()
  message(STATUS "metalvrd: without the ALVR driver (scripts/build-alvr.sh builds it)")
  target_sources(metalvrd PRIVATE driver_alvr_stub.cpp)
endif()

# Positional tracking for the Rift CV1 with its sensors is OpenHMD's tracker (Boost Software License 1.0), which
# scripts/fetch-deps.sh puts in external/OpenHMD-rift. Its sources are compiled unchanged; rift_tracking/ gives
# them the libusb calls they make (on IOUSBHost) and gives the Rift driver a narrow interface to them. Without
# the sources the Rift driver tracks rotation only.
set(METALVR_OPENHMD_DIR "${CMAKE_SOURCE_DIR}/external/OpenHMD-rift" CACHE PATH "OpenHMD with the Rift tracker")
if(EXISTS "${METALVR_OPENHMD_DIR}/src/drv_oculus_rift/rift-tracker.c")
  set(rift_tracking_default ON)
else()
  set(rift_tracking_default OFF)
endif()
option(METALVR_WITH_RIFT_TRACKING "Track the Rift CV1's position with its sensors" ${rift_tracking_default})

if(METALVR_WITH_RIFT_TRACKING)
  set(ohmd "${METALVR_OPENHMD_DIR}/src")
  set(rift "${ohmd}/drv_oculus_rift")
  if(NOT EXISTS "${rift}/rift-tracker.c")
    message(FATAL_ERROR "METALVR_WITH_RIFT_TRACKING needs ${METALVR_OPENHMD_DIR}; run scripts/fetch-deps.sh")
  endif()
  message(STATUS "metalvrd: with Rift positional tracking (${METALVR_OPENHMD_DIR})")
  # rift-tracker.c is not listed: rift_tracking.c includes it
  add_library(rift_tracking STATIC
    rift_tracking/rift_tracking.c
    rift_tracking/ohmd_compat.c
    rift_tracking/camera_tap.c
    rift_tracking/camera_math.c
    rift_tracking/uvc_stop.c
    rift_tracking/usb_shim.mm
    "${ohmd}/omath.c"
    "${ohmd}/matrices.c"
    "${ohmd}/ukf.c"
    "${ohmd}/unscented.c"
    "${ohmd}/exponential-filter.c"
    "${ohmd}/platform-posix.c"
    "${rift}/packet.c"
    "${rift}/correspondence_search.c"
    "${rift}/led_search.c"
    "${rift}/ohmd-video.c"
    "${rift}/rift-kalman-6dof.c"
    "${rift}/rift-sensor.c"
    "${rift}/rift-sensor-blobwatch.c"
    "${rift}/rift-sensor-flicker.c"
    "${rift}/rift-sensor-maths.c"
    "${rift}/rift-sensor-pose-helper.c"
    "${rift}/rift-sensor-pose-search.c"
    "${rift}/rift-sensor-usb.c"
    "${rift}/sensor/ar0134.c"
    "${rift}/sensor/esp570.c"
    "${rift}/sensor/esp770u.c"
    "${rift}/sensor/mt9v034.c"
    "${rift}/sensor/uvc.c"
  )
  target_include_directories(rift_tracking
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/rift_tracking"
    PRIVATE "${ohmd}" "${METALVR_OPENHMD_DIR}/include" "${rift}")
  # OpenHMD's sources are not ours to tidy: their warnings are off, and what they assume of Linux is supplied by
  # ohmd_compat.h, included in front of each of them
  # HAVE_OPENCV: the camera geometry OpenHMD takes from OpenCV is in rift_tracking/camera_math.c
  target_compile_definitions(rift_tracking PRIVATE HAVE_LIBJPEG=0 HAVE_OPENCV=1 DRIVER_OCULUS_RIFT)
  target_compile_options(rift_tracking PRIVATE -w -fobjc-arc
    $<$<COMPILE_LANGUAGE:C>:-include> $<$<COMPILE_LANGUAGE:C>:${CMAKE_CURRENT_SOURCE_DIR}/rift_tracking/ohmd_compat.h>)
  # camera_tap.c sees every picture on its way to the blob detector, and each sensor's pose finder as it is set up
  set_source_files_properties("${rift}/rift-sensor.c" PROPERTIES
    COMPILE_DEFINITIONS "blobwatch_process=metalvr_blobwatch_process;rift_pose_finder_init=metalvr_pose_finder_init")
  # uvc_stop.c stops a sensor's video without freeing frames the analysis threads still hold
  set_source_files_properties("${rift}/rift-sensor-usb.c" PROPERTIES COMPILE_DEFINITIONS
    "rift_sensor_uvc_stream_stop=metalvr_uvc_stream_stop;rift_sensor_uvc_stream_clear=metalvr_uvc_stream_clear")
  target_link_libraries(rift_tracking PUBLIC "-framework Foundation" "-framework IOKit" "-framework IOUSBHost")
  target_link_libraries(metalvrd PRIVATE rift_tracking)
  target_compile_definitions(metalvrd PRIVATE METALVR_RIFT_TRACKING=1)
else()
  message(STATUS "metalvrd: without Rift positional tracking (scripts/fetch-deps.sh fetches OpenHMD)")
endif()

target_include_directories(metalvrd PRIVATE "${CMAKE_SOURCE_DIR}/protocol" "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(metalvrd PRIVATE -fobjc-arc)
target_link_libraries(metalvrd PRIVATE
  "-framework Foundation"
  "-framework Metal"
  "-framework CoreVideo"
  "-framework CoreMedia"
  "-framework VideoToolbox"
  "-framework IOSurface"
  "-framework CoreGraphics"
  "-framework ImageIO"
  "-framework UniformTypeIdentifiers"
  "-framework AppKit"
  "-framework IOKit"
  "-framework QuartzCore"
  "-framework ScreenCaptureKit"
)
