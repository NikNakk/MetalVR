#!/bin/bash
# Builds OpenComposite's 64-bit openvr_api.dll with MinGW-w64: the DLL that stands in for a SteamVR game's
# openvr_api.dll and turns its OpenVR calls into OpenXR calls, which MetalVR's runtime then answers.
#
# OpenComposite is GPLv3 and MetalVR is MIT. Nothing of it is copied into or linked with MetalVR: it is fetched,
# built and shipped as its own DLL, with the changes MinGW needs kept as patches/opencomposite/*.patch.
#
# Idempotent: clones OpenComposite at the pinned commit if external/OpenComposite is missing, fetches the Vulkan
# headers its build reads, applies the patches, configures and builds. Everything it writes is under
# external/OpenComposite and build/opencomposite. It needs the network only for what is not there yet.
#
# Prints, last:
#   OPENCOMPOSITE_DLL=<path to openvr_api.dll>          replaces a game's openvr_api.dll
#   OPENCOMPOSITE_VRCLIENT=<path to vrclient_x64.dll>   the same file under the name an OpenVR runtime carries
#
# OPENCOMPOSITE_BUILD_TYPE  CMake build type (default Release)
set -euo pipefail

OC_COMMIT="cff07db75c4823afe93ed7027b03d5f7bc86f164" # branch openxr, 2025-07-21
OC_URL="https://gitlab.com/znixian/OpenOVR.git"
# OpenComposite's Windows build wants a Vulkan SDK under libs/vulkan (its README points at a prebuilt archive).
# The headers come from Khronos instead, and the import library is generated from them.
VULKAN_HEADERS_TAG="vulkan-sdk-1.3.296.0"
VULKAN_HEADERS_COMMIT="29f979ee5aa58b7b005f805ea8df7a855c39ff37"
VULKAN_HEADERS_URL="https://github.com/KhronosGroup/Vulkan-Headers.git"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OC_DIR="$ROOT/external/OpenComposite"
PATCH_DIR="$ROOT/patches/opencomposite"
BUILD_DIR="$ROOT/build/opencomposite"
DEPS_DIR="$BUILD_DIR/deps"
TOOLCHAIN="$ROOT/cmake/mingw-x86_64.cmake"
BUILD_TYPE="${OPENCOMPOSITE_BUILD_TYPE:-Release}"

say() { printf 'build-opencomposite: %s\n' "$*" >&2; }
die() { say "$*"; exit 1; }

for tool in git cmake ninja python3; do
  command -v "$tool" >/dev/null || die "$tool is required"
done
for tool in x86_64-w64-mingw32-g++ x86_64-w64-mingw32-windres x86_64-w64-mingw32-dlltool; do
  command -v "$tool" >/dev/null || die "$tool is required (MinGW-w64: brew install mingw-w64)"
done
[ -f "$TOOLCHAIN" ] || die "$TOOLCHAIN is missing"

mkdir -p "$BUILD_DIR" "$DEPS_DIR"

# --- source -----------------------------------------------------------------------------------------------------
if [ ! -d "$OC_DIR/.git" ]; then
  [ -e "$OC_DIR" ] && die "$OC_DIR exists but is not a git checkout"
  say "cloning OpenComposite at $OC_COMMIT"
  mkdir -p "$(dirname "$OC_DIR")"
  git clone --quiet "$OC_URL" "$OC_DIR"
  git -C "$OC_DIR" -c advice.detachedHead=false checkout --quiet "$OC_COMMIT"
fi
have="$(git -C "$OC_DIR" rev-parse HEAD)"
[ "$have" = "$OC_COMMIT" ] ||
  die "$OC_DIR is at $have, not the pinned $OC_COMMIT; remove the directory to clone it again"
# the OpenXR loader (built into the DLL) and glm; libs/libunwind is only read by Linux builds
if [ ! -f "$OC_DIR/libs/openxr-sdk/src/loader/loader_core.cpp" ] || [ ! -f "$OC_DIR/libs/glm/glm/glm.hpp" ]; then
  say "fetching the openxr-sdk and glm submodules"
  git -C "$OC_DIR" submodule --quiet update --init libs/openxr-sdk libs/glm
fi

# --- Vulkan headers and import library ----------------------------------------------------------------------------
# OpenComposite compiles its Vulkan path on Windows whatever the game uses, and its CMake names
# libs/vulkan/Include and libs/vulkan/Lib/vulkan-1.lib (a directory its own .gitignore leaves out).
headers="$DEPS_DIR/Vulkan-Headers"
if [ "$(git -C "$headers" rev-parse HEAD 2>/dev/null || true)" != "$VULKAN_HEADERS_COMMIT" ]; then
  say "fetching Vulkan-Headers $VULKAN_HEADERS_TAG"
  rm -rf "$headers"
  git clone --quiet --depth 1 --branch "$VULKAN_HEADERS_TAG" -c advice.detachedHead=false \
    "$VULKAN_HEADERS_URL" "$headers"
  [ "$(git -C "$headers" rev-parse HEAD)" = "$VULKAN_HEADERS_COMMIT" ] ||
    die "Vulkan-Headers $VULKAN_HEADERS_TAG is not the pinned $VULKAN_HEADERS_COMMIT"
fi
vulkan="$OC_DIR/libs/vulkan"
stamp="$vulkan/.metalvr-vulkan-headers"
if [ "$(cat "$stamp" 2>/dev/null || true)" != "$VULKAN_HEADERS_COMMIT" ] || [ ! -f "$vulkan/Lib/vulkan-1.lib" ]; then
  say "preparing libs/vulkan (headers and a generated vulkan-1 import library)"
  rm -rf "$vulkan"
  mkdir -p "$vulkan/Include" "$vulkan/Lib"
  cp -R "$headers/include/vulkan" "$headers/include/vk_video" "$vulkan/Include/"
  # every function the headers declare, as an import from vulkan-1.dll; the linker keeps the ones that are used
  def="$DEPS_DIR/vulkan-1.def"
  {
    echo "LIBRARY vulkan-1.dll"
    echo "EXPORTS"
    grep -ohE 'VKAPI_CALL vk[A-Za-z0-9_]+\(' "$vulkan/Include/vulkan/vulkan_core.h" \
      "$vulkan/Include/vulkan/vulkan_win32.h" | sed -E 's/VKAPI_CALL (vk[A-Za-z0-9_]+)\(/  \1/' | sort -u
  } >"$def"
  x86_64-w64-mingw32-dlltool -d "$def" -l "$vulkan/Lib/vulkan-1.lib"
  printf '%s\n' "$VULKAN_HEADERS_COMMIT" >"$stamp"
fi

# --- patches ----------------------------------------------------------------------------------------------------
shopt -s nullglob
for patch in "$PATCH_DIR"/*.patch; do
  if git -C "$OC_DIR" apply --reverse --check "$patch" 2>/dev/null; then
    say "patch already applied: $(basename "$patch")"
  elif git -C "$OC_DIR" apply --check "$patch" 2>/dev/null; then
    say "applying patch: $(basename "$patch")"
    git -C "$OC_DIR" apply "$patch"
  else
    die "patch does not apply to OpenComposite $OC_COMMIT: $patch"
  fi
done
shopt -u nullglob

# --- build ------------------------------------------------------------------------------------------------------
# OC_VERSION is given: left unset, OpenComposite asks git from the build directory, which answers for MetalVR's
# repository rather than its own.
say "configuring ($BUILD_TYPE)"
if ! cmake -S "$OC_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DOC_VERSION="${OC_COMMIT:0:8}-mingw" >"$BUILD_DIR/configure.log" 2>&1; then
  tail -n 40 "$BUILD_DIR/configure.log" >&2
  die "cmake failed; full log: $BUILD_DIR/configure.log"
fi
say "building"
if ! cmake --build "$BUILD_DIR" >"$BUILD_DIR/build.log" 2>&1; then
  grep -E 'error:|undefined reference|FAILED' "$BUILD_DIR/build.log" | head -n 40 >&2 || true
  die "the build failed; full log: $BUILD_DIR/build.log"
fi

# OpenComposite's build names its output for a system-wide OpenVR runtime; a game's copy is called openvr_api.dll
vrclient="$BUILD_DIR/bin/vrclient_x64.dll"
[ -f "$vrclient" ] || die "the build succeeded but $vrclient is missing; log: $BUILD_DIR/build.log"
dll="$BUILD_DIR/openvr_api.dll"
cp -f "$vrclient" "$dll"

echo "OPENCOMPOSITE_DLL=$dll"
echo "OPENCOMPOSITE_VRCLIENT=$vrclient"
