#!/bin/bash
# Builds ALVR's server_core as a static library MetalVR's daemon links, plus its C header.
#
# Idempotent: clones ALVR at the pinned tag if external/ALVR is missing, fetches the one OpenVR header ALVR's build
# reads, applies patches/alvr/*.patch, builds the release profile, and generates the header with cbindgen. Everything
# it writes is under external/ALVR and build/alvr, apart from cargo's own download cache (CARGO_HOME).
#
# Prints, last:
#   ALVR_LIBRARY=<path to libalvr_server_core.a>
#   ALVR_INCLUDE_DIR=<directory holding alvr_server_core.h>
#   ALVR_HEADER=<path to alvr_server_core.h>
#   ALVR_LINK_FLAGS=<system libraries and frameworks the static library needs, one token each, no duplicates>
set -euo pipefail

ALVR_TAG="v20.14.1"
ALVR_URL="https://github.com/alvr-org/ALVR.git"
CBINDGEN_VERSION="0.29.4"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ALVR_DIR="$ROOT/external/ALVR"
PATCH_DIR="$ROOT/patches/alvr"
BUILD_DIR="$ROOT/build/alvr"
TARGET_DIR="$BUILD_DIR/target"
INCLUDE_DIR="$BUILD_DIR/include"
TOOLS_DIR="$BUILD_DIR/tools"

say() { printf 'build-alvr: %s\n' "$*" >&2; }
die() { say "$*"; exit 1; }

command -v git >/dev/null || die "git is required"
command -v cargo >/dev/null || die "cargo is required (install Rust: https://rustup.rs)"

mkdir -p "$BUILD_DIR" "$INCLUDE_DIR"

# --- source -----------------------------------------------------------------------------------------------------
if [ ! -d "$ALVR_DIR/.git" ]; then
  [ -e "$ALVR_DIR" ] && die "$ALVR_DIR exists but is not a git checkout"
  say "cloning ALVR $ALVR_TAG"
  mkdir -p "$(dirname "$ALVR_DIR")"
  git clone --quiet --depth 1 --branch "$ALVR_TAG" -c advice.detachedHead=false "$ALVR_URL" "$ALVR_DIR"
fi

have="$(git -C "$ALVR_DIR" rev-parse HEAD)"
want="$(git -C "$ALVR_DIR" rev-parse --verify --quiet "refs/tags/$ALVR_TAG^{commit}" || true)"
[ -n "$want" ] || die "$ALVR_DIR has no tag $ALVR_TAG; remove the directory to clone it again"
[ "$have" = "$want" ] || die "$ALVR_DIR is at $have, not $ALVR_TAG ($want); remove the directory to clone it again"

# alvr_session's build script reads openvr/headers/openvr_driver.h from the openvr submodule and nothing else, so
# fetch that one file at the commit ALVR pins instead of the whole (several hundred MB) repository.
openvr_sha="$(git -C "$ALVR_DIR" ls-tree HEAD openvr | awk '{print $3}')"
[ -n "$openvr_sha" ] || die "ALVR $ALVR_TAG has no openvr submodule entry"
openvr_dir="$ALVR_DIR/openvr"
if [ ! -f "$openvr_dir/headers/openvr_driver.h" ] ||
  [ "$(git -C "$openvr_dir" rev-parse HEAD 2>/dev/null || true)" != "$openvr_sha" ]; then
  say "fetching openvr_driver.h at $openvr_sha"
  openvr_url="$(git -C "$ALVR_DIR" config -f .gitmodules submodule.openvr.url)"
  mkdir -p "$openvr_dir"
  if [ ! -e "$openvr_dir/.git" ]; then
    git -C "$openvr_dir" init --quiet
    git -C "$openvr_dir" remote add origin "$openvr_url"
  fi
  git -C "$openvr_dir" sparse-checkout set --no-cone /headers/openvr_driver.h
  git -C "$openvr_dir" fetch --quiet --depth 1 --filter=blob:none origin "$openvr_sha"
  git -C "$openvr_dir" -c advice.detachedHead=false checkout --quiet FETCH_HEAD
  [ -f "$openvr_dir/headers/openvr_driver.h" ] || die "openvr_driver.h is missing after the fetch"
fi

# --- patches ----------------------------------------------------------------------------------------------------
shopt -s nullglob
for patch in "$PATCH_DIR"/*.patch; do
  if git -C "$ALVR_DIR" apply --reverse --check "$patch" 2>/dev/null; then
    say "patch already applied: $(basename "$patch")"
  elif git -C "$ALVR_DIR" apply --check "$patch" 2>/dev/null; then
    say "applying patch: $(basename "$patch")"
    git -C "$ALVR_DIR" apply "$patch"
  else
    die "patch does not apply to ALVR $ALVR_TAG: $patch"
  fi
done
shopt -u nullglob

# --- library ----------------------------------------------------------------------------------------------------
# server_core declares crate-type ["rlib", "cdylib"]; `cargo rustc --crate-type` builds the same crate as a static
# library without editing its manifest. --locked keeps ALVR's own Cargo.lock.
say "building alvr_server_core (release, staticlib)"
cargo_log="$BUILD_DIR/cargo-build.log"
if ! (cd "$ALVR_DIR" && CARGO_TARGET_DIR="$TARGET_DIR" cargo rustc --release --locked -p alvr_server_core --lib \
  --crate-type staticlib -- --print native-static-libs) >"$cargo_log" 2>&1; then
  tail -n 40 "$cargo_log" >&2
  die "cargo failed; full log: $cargo_log"
fi
library="$TARGET_DIR/release/libalvr_server_core.a"
[ -f "$library" ] || die "cargo succeeded but $library is missing; log: $cargo_log"

# rustc names the system libraries a static library needs; keep the answer, since cargo only repeats it when it
# actually recompiles the crate.
flags_file="$BUILD_DIR/native-static-libs.txt"
if native="$(grep -m1 'native-static-libs:' "$cargo_log")"; then
  printf '%s\n' "${native#*native-static-libs: }" >"$flags_file"
fi
[ -s "$flags_file" ] || die "rustc did not report native-static-libs; remove $TARGET_DIR/release and run again"

# --- header -----------------------------------------------------------------------------------------------------
cbindgen="$TOOLS_DIR/bin/cbindgen"
if [ ! -x "$cbindgen" ] || [ "$("$cbindgen" --version)" != "cbindgen $CBINDGEN_VERSION" ]; then
  say "installing cbindgen $CBINDGEN_VERSION into $TOOLS_DIR"
  if ! cargo install --quiet --locked --force --version "$CBINDGEN_VERSION" --root "$TOOLS_DIR" cbindgen \
    >"$BUILD_DIR/cbindgen-install.log" 2>&1; then
    tail -n 40 "$BUILD_DIR/cbindgen-install.log" >&2
    die "could not install cbindgen"
  fi
fi

say "generating alvr_server_core.h"
header="$INCLUDE_DIR/alvr_server_core.h"
if ! (cd "$ALVR_DIR/alvr/server_core" && "$cbindgen" --quiet --output "$header") >"$BUILD_DIR/cbindgen.log" 2>&1; then
  cat "$BUILD_DIR/cbindgen.log" >&2
  die "cbindgen failed"
fi
[ -s "$header" ] || die "cbindgen wrote no header"

# rustc's list repeats itself and spells frameworks as two words; build systems that drop repeated options (CMake's
# target_link_options) would mangle that, so give each framework as one token and each library once. libSystem,
# libc and libm are linked into every macOS program already.
link_flags=""
framework=0
for token in $(cat "$flags_file"); do
  if [ "$framework" = 1 ]; then
    token="-Wl,-framework,$token"
    framework=0
  elif [ "$token" = "-framework" ]; then
    framework=1
    continue
  fi
  case "$token" in -lSystem | -lc | -lm) continue ;; esac
  case " $link_flags " in *" $token "*) ;; *) link_flags="$link_flags $token" ;; esac
done
link_flags="${link_flags# }"

echo "ALVR_LIBRARY=$library"
echo "ALVR_INCLUDE_DIR=$INCLUDE_DIR"
echo "ALVR_HEADER=$header"
echo "ALVR_LINK_FLAGS=$link_flags"
