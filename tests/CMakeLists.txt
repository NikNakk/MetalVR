if(WIN32)
  add_executable(xr_smoke xr_smoke.cpp gfx_d3d11.cpp)
  target_link_options(xr_smoke PRIVATE -static -static-libgcc -static-libstdc++)
  target_link_libraries(xr_smoke PRIVATE d3d11 dxgi)
  add_executable(xr_smoke_d3d12 xr_smoke.cpp gfx_d3d12.cpp)
  target_link_options(xr_smoke_d3d12 PRIVATE -static -static-libgcc -static-libstdc++)
  target_link_libraries(xr_smoke_d3d12 PRIVATE d3d12 dxgi)
  target_include_directories(xr_smoke_d3d12 PRIVATE "${OPENXR_INCLUDE_DIR}")
  add_dependencies(xr_smoke_d3d12 metalvr_openxr)
elseif(APPLE)
  add_executable(xr_smoke xr_smoke.cpp gfx_metal.mm)
  set_source_files_properties(gfx_metal.mm PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
  target_link_libraries(xr_smoke PRIVATE "-framework Foundation" "-framework Metal")

  add_executable(stream_check stream_check.mm)
  set_source_files_properties(stream_check.mm PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
  target_link_libraries(stream_check PRIVATE
    "-framework Foundation" "-framework CoreMedia" "-framework CoreVideo" "-framework VideoToolbox"
    "-framework CoreGraphics" "-framework ImageIO" "-framework UniformTypeIdentifiers")

  add_executable(frame_check frame_check.mm)
  set_source_files_properties(frame_check.mm PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
  target_link_libraries(frame_check PRIVATE
    "-framework Foundation" "-framework CoreGraphics" "-framework ImageIO")

  if(TARGET rift_tracking)
    # the Rift sensors by themselves: opens them and runs their cameras
    add_executable(rift_tracking_smoke rift_tracking_smoke.c)
    target_link_libraries(rift_tracking_smoke PRIVATE rift_tracking)
  endif()

  if(METALVR_WITH_ALVR)
    # the ALVR driver by itself: starts it, reads its defaults, stops it, starts it again
    add_executable(alvr_driver_smoke alvr_driver_smoke.cpp "${CMAKE_SOURCE_DIR}/daemon/driver_alvr.cpp")
    target_include_directories(alvr_driver_smoke PRIVATE
      "${CMAKE_SOURCE_DIR}/daemon" "${CMAKE_SOURCE_DIR}/protocol" "${METALVR_ALVR_INCLUDE_DIR}")
    target_link_libraries(alvr_driver_smoke PRIVATE "${METALVR_ALVR_LIBRARY}")
    separate_arguments(alvr_test_flags NATIVE_COMMAND "${METALVR_ALVR_LINK_FLAGS}")
    target_link_options(alvr_driver_smoke PRIVATE ${alvr_test_flags})
  endif()
endif()

target_include_directories(xr_smoke PRIVATE "${OPENXR_INCLUDE_DIR}")
# the client loads the runtime by path at run time; it only has to be built first
add_dependencies(xr_smoke metalvr_openxr)
