#!/bin/sh
# Fetches what MetalVR builds against into external/ (not kept in the repository).
#   OpenXR-SDK   Khronos headers and loader (Apache-2.0), needed by everything
# ALVR (scripts/build-alvr.sh) and OpenComposite (scripts/build-opencomposite.sh) fetch themselves.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
openxr_tag=release-1.1.65
mkdir -p "$root/external"
if [ ! -f "$root/external/OpenXR-SDK/include/openxr/openxr.h" ]; then
  git clone --depth 1 --branch "$openxr_tag" https://github.com/KhronosGroup/OpenXR-SDK.git "$root/external/OpenXR-SDK"
fi
echo "OpenXR-SDK: $(git -C "$root/external/OpenXR-SDK" describe --tags --always)"
