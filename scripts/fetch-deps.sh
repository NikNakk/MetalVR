#!/bin/sh
# Fetches what MetalVR builds against into external/ (not kept in the repository).
#   OpenXR-SDK   Khronos headers and loader (Apache-2.0), needed by everything
#   OpenHMD-rift  OpenHMD with the Rift's positional tracker (BSL-1.0), which metalvrd's Rift driver compiles in for
#                tracking with the Rift's sensors; without it the driver tracks rotation only
#   libsurvive, hidapi  the Lighthouse tracker (MIT) and its USB layer (BSD), which scripts/build-libsurvive.sh builds
#                for tracking the Lighthouse headsets with their base stations
#   basalt and what it needs (oneTBB, OpenCV, fmt)  Monado's Basalt (BSD-3-Clause), the visual-inertial tracker
#                scripts/build-basalt.sh builds for the Rift S's camera tracking
# ALVR (scripts/build-alvr.sh) and OpenComposite (scripts/build-opencomposite.sh) fetch themselves.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
# a tag can be moved; the commit it must be is what is checked
openxr_tag=release-1.1.65
openxr_commit=24a1f2b92d212501a2b1b8d30ffd53abf55fcfa8
mkdir -p "$root/external"
if [ ! -f "$root/external/OpenXR-SDK/include/openxr/openxr.h" ]; then
  git clone --depth 1 --branch "$openxr_tag" https://github.com/KhronosGroup/OpenXR-SDK.git "$root/external/OpenXR-SDK"
fi
found=$(git -C "$root/external/OpenXR-SDK" rev-parse HEAD)
if [ "$found" != "$openxr_commit" ]; then
  echo "external/OpenXR-SDK is at $found, not the pinned $openxr_commit ($openxr_tag)" >&2
  exit 1
fi
echo "OpenXR-SDK: $openxr_tag ($openxr_commit)"

# the branch with the Constellation tracker; it moves, so the commit is what is fetched and checked
openhmd_commit=a4ebf094e8be5c53c43c5f68d92e87afa5bdf0c8
if [ ! -f "$root/external/OpenHMD-rift/src/drv_oculus_rift/rift-tracker.c" ]; then
  git init -q "$root/external/OpenHMD-rift"
  git -C "$root/external/OpenHMD-rift" fetch -q --depth 1 https://github.com/thaytan/OpenHMD.git "$openhmd_commit"
  git -C "$root/external/OpenHMD-rift" checkout -q FETCH_HEAD
fi
found=$(git -C "$root/external/OpenHMD-rift" rev-parse HEAD)
if [ "$found" != "$openhmd_commit" ]; then
  echo "external/OpenHMD-rift is at $found, not the pinned $openhmd_commit" >&2
  exit 1
fi
echo "OpenHMD-rift: $openhmd_commit"

# libsurvive and hidapi: commits, as above
fetch_commit() { # directory, repository, commit
  if [ ! -d "$root/external/$1/.git" ]; then
    git init -q "$root/external/$1"
    git -C "$root/external/$1" fetch -q --depth 1 "$2" "$3"
    git -C "$root/external/$1" checkout -q FETCH_HEAD
  fi
  found=$(git -C "$root/external/$1" rev-parse HEAD)
  if [ "$found" != "$3" ]; then
    echo "external/$1 is at $found, not the pinned $3" >&2
    exit 1
  fi
  echo "$1: $3"
}
fetch_commit libsurvive https://github.com/cntools/libsurvive.git 44b1e2f0cb95e5697bc0b86f58fd8520a813b713
fetch_commit hidapi https://github.com/libusb/hidapi.git d3013f0af3f4029d82872c1a9487ea461a56dee4   # hidapi-0.14.0
# Basalt, Monado's fork with its VIT interface, and only the submodules its library uses
fetch_commit basalt https://gitlab.freedesktop.org/mateosss/basalt.git df6e970c8da7636eb401a09e3317fbeaaf829b9a
git -C "$root/external/basalt" submodule update -q --init --depth 1 thirdparty/basalt-headers thirdparty/opengv \
  thirdparty/magic_enum thirdparty/CLI11
git -C "$root/external/basalt/thirdparty/basalt-headers" submodule update -q --init --depth 1 thirdparty/Sophus \
  thirdparty/cereal thirdparty/eigen
fetch_commit oneTBB https://github.com/uxlfoundation/oneTBB.git 0c0ff192a2304e114bc9e6557582dfba101360ff  # v2022.0.0
fetch_commit opencv https://github.com/opencv/opencv.git 71d3237a093b60a27601c20e9ee6c3e52154e8b1     # 4.10.0
fetch_commit fmt https://github.com/fmtlib/fmt.git 0c9fce2ffefecfdce794e1859584e25877b7b592           # 11.0.2
