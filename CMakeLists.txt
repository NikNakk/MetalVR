cmake_minimum_required(VERSION 3.24)
project(MetalVR VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(OPENXR_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/OpenXR-SDK/include" CACHE PATH "Khronos OpenXR headers")
if(NOT EXISTS "${OPENXR_INCLUDE_DIR}/openxr/openxr.h")
  message(FATAL_ERROR "OpenXR headers not found; run scripts/fetch-deps.sh")
endif()

# OpenXR structs are initialized by their type alone, `{XR_TYPE_...}`, everywhere
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)

# The OpenXR runtime builds for two targets from one source tree: a Windows DLL (Direct3D 11, for games under
# Wine) with a MinGW toolchain, and a macOS dylib (Metal, for native applications).
if(APPLE)
  enable_language(OBJCXX)
  set(CMAKE_OBJCXX_STANDARD 20)
  set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)
endif()

add_subdirectory(runtime)

if(WIN32)
  add_subdirectory(steamvr)
endif()

if(APPLE)
  add_subdirectory(daemon)
endif()

option(METALVR_BUILD_TESTS "Build the test clients" ON)
if(METALVR_BUILD_TESTS)
  add_subdirectory(tests)
endif()
