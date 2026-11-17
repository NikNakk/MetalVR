#!/bin/zsh
# Builds a MetalVR release for MetalSharp and packs it as dist/metalvr-VERSION-macos-arm64.tar.gz, laid out as
# MetalSharp installs it (~/.metalsharp/metalvr-runtime):
#   bin/metalvrd                               the server (arm64), with ALVR's server_core and the Rift tracker
#   bin/metalvr, bin/metalvr-rift              the helper, and the Rift CV1 script (docs/RIFT_CV1.md)
#   lib/metalvr/x86_64-windows/                the OpenXR runtime and its manifest, and OpenComposite's openvr_api.dll
#   lib/metalvr/steamvr/metalvr/               the SteamVR driver
#   lib/metalvr/LICENSE*, OpenComposite/       the licenses of what is redistributed
#   share/metalvr/home-logo.png                the logo the Rift's home shows
#   docs/                                      the Rift CV1, Rift and SteamVR guides
#
#   package-release.sh VERSION                 e.g. 0.1.0; builds in build/release-native and build/release-win64
#
# Needs what docs/BUILDING.md lists, including the Rust toolchain for ALVR.
set -e
V=${1:?usage: package-release.sh VERSION}
ROOT=${0:A:h:h}
cd $ROOT
[[ -z $(git status --porcelain --untracked-files=no) ]] || { echo >&2 "!! the checkout has uncommitted changes"; exit 1; }
REV=$(git rev-parse --short HEAD)
N=metalvr-$V-macos-arm64
OUT=$ROOT/dist/$N
MACOS_FLOOR=15.0   # MetalSharp's

sh scripts/fetch-deps.sh
bash scripts/build-alvr.sh
cmake -S . -B build/release-native -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=$MACOS_FLOOR -DMETALVR_WITH_ALVR=ON
cmake --build build/release-native --target metalvrd
(
  unset CFLAGS CXXFLAGS OBJCFLAGS LDFLAGS CPPFLAGS CC CXX OBJC   # macOS flags must not reach the PE cross compiler
  cmake -S . -B build/release-win64 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-x86_64.cmake
  cmake --build build/release-win64 --target metalvr_openxr driver_metalvr
  bash scripts/build-opencomposite.sh >/dev/null
)

D=build/release-native/daemon/metalvrd
W=build/release-win64
for f in $D $W/runtime/metalvr_openxr.dll $W/runtime/metalvr_openxr.json build/opencomposite/openvr_api.dll \
         $W/steamvr/metalvr/bin/win64/driver_metalvr.dll; do
  [[ -f $f ]] || { echo >&2 "!! not built: $f"; exit 1; }
done
$D --help 2>&1 | grep -q -- '--trust' || { echo >&2 "!! metalvrd was built without the ALVR driver"; exit 1; }
$D --help 2>&1 | grep -q -- 'rift' || { echo >&2 "!! metalvrd was built without the Rift driver"; exit 1; }

rm -rf $OUT; mkdir -p $OUT/bin $OUT/lib/metalvr/x86_64-windows $OUT/lib/metalvr/steamvr $OUT/share/metalvr $OUT/docs
cp $D $OUT/bin/metalvrd
cp scripts/metalsharp/metalvr scripts/metalsharp/metalvr-rift $OUT/bin/
chmod +x $OUT/bin/*
cp $W/runtime/metalvr_openxr.dll $W/runtime/metalvr_openxr.json build/opencomposite/openvr_api.dll $OUT/lib/metalvr/x86_64-windows/
cp -R $W/steamvr/metalvr $OUT/lib/metalvr/steamvr/
cp scripts/metalsharp/home-logo.png $OUT/share/metalvr/
cp docs/RIFT_CV1.md docs/RIFT.md docs/STEAMVR.md $OUT/docs/
# what is redistributed, with its license: MetalVR (MIT), ALVR's server_core (MIT) and OpenHMD's Rift tracker
# (BSL-1.0) inside metalvrd, and OpenComposite (GPL-3.0) with the Khronos loader and OpenVR headers compiled in
cp LICENSE $OUT/lib/metalvr/LICENSE
cp external/ALVR/LICENSE $OUT/lib/metalvr/LICENSE.ALVR
cp external/OpenHMD-rift/LICENSE $OUT/lib/metalvr/LICENSE.OpenHMD
O=external/OpenComposite
mkdir -p $OUT/lib/metalvr/OpenComposite
cp $O/LICENSE.txt $O/LICENSE_*.txt $O/LICENCE_*.txt $OUT/lib/metalvr/OpenComposite/
cp $O/libs/openxr-sdk/LICENSE $OUT/lib/metalvr/OpenComposite/LICENSE_OPENXR_SDK.txt
echo "MetalVR $V ($REV, metalsharp/MetalVR), ALVR $(git -C external/ALVR describe --tags --always), OpenHMD $(git -C external/OpenHMD-rift rev-parse --short HEAD), OpenComposite $(git -C $O rev-parse --short HEAD)" \
  > $OUT/lib/metalvr/VERSION
codesign --force --sign - $OUT/bin/metalvrd

tar -C $ROOT/dist -czf $ROOT/dist/$N.tar.gz $N
shasum -a 256 $ROOT/dist/$N.tar.gz | sed "s| .*/| |" > $ROOT/dist/$N.tar.gz.sha256
echo "$ROOT/dist/$N.tar.gz ($(cat $OUT/lib/metalvr/VERSION))"
