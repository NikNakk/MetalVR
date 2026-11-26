#!/bin/sh
# Build the headless PSVR2 and Sense drivers from the Monado source we use for tracking.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
revision=fbffda93b9d01d20b2dd1b0b8548cc016275d46c
source_dir=${METALVR_MONADO_SOURCE:-"$root/external/monado-sense"}
build_dir=${METALVR_PSSENSE_BUILD:-"$root/build/pssense"}
macos_floor=${MACOSX_DEPLOYMENT_TARGET:-15.0}
jobs=${CMAKE_BUILD_PARALLEL_LEVEL:-4}

if [ -z "${METALVR_MONADO_SOURCE:-}" ] && ! git -C "$source_dir" cat-file -e "$revision^{commit}" 2>/dev/null; then
  mkdir -p "$source_dir"
  git -C "$source_dir" init -q
  git -C "$source_dir" fetch --depth 1 https://github.com/NikNakk/monado.git "$revision"
fi
if [ -z "${METALVR_MONADO_SOURCE:-}" ] &&
   [ "$(git -C "$source_dir" rev-parse --verify HEAD 2>/dev/null || true)" != "$revision" ]; then
  git -C "$source_dir" checkout --detach "$revision"
fi

# Build a shared libusb for the same deployment target as metalvrd. Homebrew's
# binary can target a newer macOS than the release.
mkdir -p "$build_dir"
build_dir=$(cd "$build_dir" && pwd)
usb_version=1.0.30
usb_sha=fea36f34f9156400209595e300840767ab1a385ede1dc7ee893015aea9c6dbaf
deps="$root/build/pssense-deps"
archive="$deps/libusb-$usb_version.tar.bz2"
usb_source="$deps/libusb-$usb_version"
usb_build="$build_dir/libusb-macos-$macos_floor"
usb_prefix="$usb_build/prefix"
usb_library="$usb_prefix/lib/libusb-1.0.0.dylib"
mkdir -p "$deps" "$usb_build"
if [ ! -f "$archive" ]; then
  curl -fLsS "https://github.com/libusb/libusb/releases/download/v$usb_version/${archive##*/}" -o "$archive.part"
  printf '%s  %s\n' "$usb_sha" "$archive.part" | shasum -a 256 -c -
  mv "$archive.part" "$archive"
fi
printf '%s  %s\n' "$usb_sha" "$archive" | shasum -a 256 -c -
if [ ! -x "$usb_source/configure" ]; then
  tar -xf "$archive" -C "$deps"
fi
if [ ! -f "$usb_library" ]; then
  (
    cd "$usb_build"
    export MACOSX_DEPLOYMENT_TARGET="$macos_floor"
    # The v27 SDK declares pipe2 even when targeting macOS 15, where it is unavailable.
    ac_cv_func_pipe2=no lt_cv_sys_max_cmd_len=65536 CFLAGS="${CFLAGS:-} -arch arm64" \
      "$usb_source/configure" --prefix="$usb_prefix" --disable-static
    make -j "$jobs"
    make install
  )
fi
if [ "$(otool -D "$usb_library" | tail -1)" != '@loader_path/libusb-1.0.0.dylib' ]; then
  install_name_tool -id '@loader_path/libusb-1.0.0.dylib' "$usb_library"
fi
cp "$usb_library" "$build_dir/"

cmake -S "$root/daemon/pssense" -B "$build_dir" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET="$macos_floor" -DMONADO_SOURCE_DIR="$source_dir" \
  -DLIBUSB1_LIBRARY="$usb_library" -DLIBUSB1_INCLUDE_DIR="$usb_prefix/include/libusb-1.0"
cmake --build "$build_dir" --target metalvr_pssense --parallel "$jobs"
codesign --force --sign - "$build_dir/libusb-1.0.0.dylib" "$build_dir/libmetalvr_pssense.dylib"

mkdir -p "$build_dir/notices/Monado" "$build_dir/notices/libusb"
cp -R "$source_dir/LICENSE" "$source_dir/LICENSES" "$build_dir/notices/Monado/"
cp "$usb_source/COPYING" "$archive" "$build_dir/notices/libusb/"
if [ -z "${METALVR_MONADO_SOURCE:-}" ]; then
  printf 'https://github.com/NikNakk/monado/tree/%s\n' "$revision" > "$build_dir/notices/Monado/SOURCE"
else
  printf '%s\n' 'Built from the local checkout selected by METALVR_MONADO_SOURCE.' > "$build_dir/notices/Monado/SOURCE"
fi
