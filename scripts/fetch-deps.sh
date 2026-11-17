#!/bin/sh
# Fetches what MetalVR builds against into external/ (not kept in the repository).
#   OpenXR-SDK   Khronos headers and loader (Apache-2.0), needed by everything
#   OpenHMD-rift  OpenHMD with the Rift's positional tracker (BSL-1.0), which metalvrd's Rift driver compiles in for
#                tracking with the Rift's sensors; without it the driver tracks rotation only
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
