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
)
