#!/bin/sh
# Builds libsurvive (MIT), the Lighthouse tracker metalvrd's Lighthouse driver takes positions from (docs/HEADSETS.md),
# statically into build/libsurvive, with hidapi (BSD) as its USB layer: macOS's own HID driver holds the headsets'
# HID interfaces, which hidapi reaches through IOHIDManager and libusb could not claim. scripts/fetch-deps.sh fetches
# both into external/. Configure build/native again afterwards and metalvrd links it.
#
#   scripts/build-libsurvive.sh
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
out=$root/build/libsurvive
for dir in libsurvive hidapi; do
  [ -d "$root/external/$dir" ] || { echo "external/$dir is missing: run scripts/fetch-deps.sh" >&2; exit 1; }
done
# the changes MetalVR needs (patches/libsurvive), applied once
for patch in "$root"/patches/libsurvive/*.patch; do
  [ -f "$patch" ] || continue
  if git -C "$root/external/libsurvive" apply --reverse --check "$patch" 2>/dev/null; then
    :
  elif git -C "$root/external/libsurvive" apply --check "$patch" 2>/dev/null; then
    echo "applying $(basename "$patch")"
    git -C "$root/external/libsurvive" apply "$patch"
  else
    echo "patch does not apply to external/libsurvive: $patch" >&2
    exit 1
  fi
done

common="-G Ninja -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=15.0"

# shellcheck disable=SC2086
cmake -S "$root/external/hidapi" -B "$out/hidapi-build" $common -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_INSTALL_PREFIX="$out/prefix" >"$out.hidapi.log" 2>&1 || { cat "$out.hidapi.log"; exit 1; }
cmake --build "$out/hidapi-build" >>"$out.hidapi.log" 2>&1 && cmake --install "$out/hidapi-build" >>"$out.hidapi.log" 2>&1

# the core and the plugins Lighthouse tracking needs, each a static library that registers itself when linked whole
# shellcheck disable=SC2086
PKG_CONFIG_PATH="$out/prefix/lib/pkgconfig" cmake -S "$root/external/libsurvive" -B "$out/build" $common \
  -DBUILD_STATIC=ON -DDO_CORE_BUILD=ON -DUSE_HIDAPI=ON -DBUILD_APPLICATIONS=OFF -DENABLE_TESTS=OFF \
  -DBUILD_GATT_SUPPORT=OFF -DUSE_ASAN=OFF \
  -DENABLE_driver_vive=ON -DENABLE_driver_simulator=ON -DENABLE_disambiguator_statebased=ON -DENABLE_poser_kalman_only=ON \
  -DENABLE_poser_mpfit=ON -DENABLE_driver_global_scene_solver=ON \
  >"$out.log" 2>&1 || { cat "$out.log"; exit 1; }
cmake --build "$out/build" --target survive driver_vive driver_simulator disambiguator_statebased poser_kalman_only poser_mpfit \
  driver_global_scene_solver >>"$out.log" 2>&1 || { tail -40 "$out.log"; exit 1; }
echo "libsurvive $(git -C "$root/external/libsurvive" rev-parse --short HEAD) built into $out/build"
find "$out/build" -name "*.a" | sed "s|$out/build/|  |"
