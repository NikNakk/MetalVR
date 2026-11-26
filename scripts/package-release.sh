#!/bin/zsh
# Builds a MetalVR release for MetalSharp and packs it as dist/metalvr-VERSION-macos-arm64.tar.gz, laid out as
# MetalSharp installs it (~/.metalsharp/metalvr-runtime):
#   bin/metalvrd                               the server (arm64), with ALVR's server_core and the Rift tracker
#   bin/metalvr, bin/metalvr-rift              the helper, and the script for a headset on this Mac's own display
#   bin/metalvr-lighthouse, bin/metalvr-psvr2  (docs/headsets/): the Rift's, the Lighthouse's
#                                              and the PlayStation VR2's
#   lib/libbasalt.dylib, lib/libtbb.12.dylib  Basalt, which metalvrd loads for the Rift S's camera tracking
#   lib/libmetalvr_pssense.dylib, lib/libusb-1.0.0.dylib  Monado Sense 6DoF (with METALVR_PACKAGE_PSSENSE=1)
#   lib/metalvr/x86_64-windows/                the OpenXR runtime and its manifest, and OpenComposite's openvr_api.dll
#   lib/metalvr/steamvr/metalvr/               the SteamVR driver
#   lib/metalvr/LICENSE*, OpenComposite/       the licenses of what is redistributed
#   share/metalvr/home-logo.png                the logo the Rift's home shows
#   share/metalvr/calibration/                 PSVR2 reference calibration (with METALVR_PACKAGE_PSSENSE=1)
#   docs/                                      the headset guides (docs/headsets), the driver, ALVR and SteamVR
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
sh scripts/build-libsurvive.sh
sh scripts/build-basalt.sh
SENSE=
if [[ ${METALVR_PACKAGE_PSSENSE:-0} == 1 ]]; then
  SENSE=$ROOT/build/release-pssense
  MACOSX_DEPLOYMENT_TARGET=$MACOS_FLOOR METALVR_PSSENSE_BUILD=$SENSE sh scripts/build-pssense.sh
fi

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
B=build/basalt/prefix/lib
for f in $D $B/libbasalt.dylib $B/libtbb.12.dylib $W/runtime/metalvr_openxr.dll $W/runtime/metalvr_openxr.json build/opencomposite/openvr_api.dll \
         $W/steamvr/metalvr/bin/win64/driver_metalvr.dll; do
  [[ -f $f ]] || { echo >&2 "!! not built: $f"; exit 1; }
done
$D --help 2>&1 | grep -q -- '--trust' || { echo >&2 "!! metalvrd was built without the ALVR driver"; exit 1; }
$D --help 2>&1 | grep -q -- 'rift' || { echo >&2 "!! metalvrd was built without the Rift driver"; exit 1; }
$D --survive-check >/dev/null 2>&1 || { echo >&2 "!! metalvrd was built without libsurvive"; exit 1; }

rm -rf $OUT; mkdir -p $OUT/bin $OUT/lib/metalvr/x86_64-windows $OUT/lib/metalvr/steamvr $OUT/share/metalvr $OUT/docs
cp $D $OUT/bin/metalvrd
cp scripts/metalsharp/metalvr scripts/metalsharp/metalvr-rift $OUT/bin/
ln -s metalvr-rift $OUT/bin/metalvr-lighthouse   # the same script, for the Lighthouse headsets
ln -s metalvr-rift $OUT/bin/metalvr-psvr2        # and for the PlayStation VR2
chmod +x $OUT/bin/metalvrd $OUT/bin/metalvr $OUT/bin/metalvr-rift
# Basalt (with OpenCV and fmt inside it) and oneTBB, found by metalvrd as bin/../lib/libbasalt.dylib; Basalt finds
# oneTBB beside it (@loader_path)
cp -L $B/libbasalt.dylib $OUT/lib/libbasalt.dylib
cp -L $B/libtbb.12.dylib $OUT/lib/libtbb.12.dylib
if [[ -n $SENSE ]]; then
  cp "$SENSE/libmetalvr_pssense.dylib" "$SENSE/libusb-1.0.0.dylib" "$OUT/lib/"
  cp -R "$SENSE/notices/Monado" "$SENSE/notices/libusb" "$OUT/lib/metalvr/"
  mkdir -p "$OUT/share/metalvr/calibration"
  cp calibration/psvr2-mode4-reference.json "$OUT/share/metalvr/calibration/"
fi
cp $W/runtime/metalvr_openxr.dll $W/runtime/metalvr_openxr.json build/opencomposite/openvr_api.dll $OUT/lib/metalvr/x86_64-windows/
cp -R $W/steamvr/metalvr $OUT/lib/metalvr/steamvr/
cp scripts/metalsharp/home-logo.png $OUT/share/metalvr/
cp -R docs/headsets $OUT/docs/
cp docs/RIFT.md docs/STEAMVR.md docs/HEADSETS.md docs/ALVR.md $OUT/docs/
# what is redistributed, with its license: MetalVR (MIT), ALVR's server_core (MIT), OpenHMD's Rift tracker
# (BSL-1.0), libsurvive (MIT) and hidapi (BSD) inside metalvrd, Basalt (BSD-3-Clause) with what it builds in
# (OpenCV, Apache-2.0; fmt, MIT; Sophus, MIT; cereal, BSD; Eigen, MPL-2.0; OpenGV, BSD; CLI11, BSD; magic_enum, MIT)
# and oneTBB (Apache-2.0), and OpenComposite (GPL-3.0) with the Khronos loader and OpenVR headers compiled in
cp LICENSE $OUT/lib/metalvr/LICENSE
cp external/ALVR/LICENSE $OUT/lib/metalvr/LICENSE.ALVR
cp external/OpenHMD-rift/LICENSE $OUT/lib/metalvr/LICENSE.OpenHMD
cp external/libsurvive/LICENSE $OUT/lib/metalvr/LICENSE.libsurvive
cp external/hidapi/LICENSE-bsd.txt $OUT/lib/metalvr/LICENSE.hidapi
mkdir -p $OUT/lib/metalvr/Basalt
BT=external/basalt/thirdparty
cp external/basalt/LICENSE $OUT/lib/metalvr/Basalt/LICENSE.Basalt
cp external/oneTBB/LICENSE.txt $OUT/lib/metalvr/Basalt/LICENSE.oneTBB
cp external/opencv/LICENSE $OUT/lib/metalvr/Basalt/LICENSE.OpenCV
cp external/fmt/LICENSE $OUT/lib/metalvr/Basalt/LICENSE.fmt
cp $BT/basalt-headers/thirdparty/Sophus/LICENSE.txt $OUT/lib/metalvr/Basalt/LICENSE.Sophus
cp $BT/basalt-headers/thirdparty/cereal/LICENSE $OUT/lib/metalvr/Basalt/LICENSE.cereal
cp $BT/basalt-headers/thirdparty/eigen/COPYING.MPL2 $OUT/lib/metalvr/Basalt/LICENSE.Eigen
cp $BT/opengv/License.txt $OUT/lib/metalvr/Basalt/LICENSE.OpenGV
cp $BT/CLI11/LICENSE $OUT/lib/metalvr/Basalt/LICENSE.CLI11
cp $BT/magic_enum/LICENSE $OUT/lib/metalvr/Basalt/LICENSE.magic_enum
O=external/OpenComposite
mkdir -p $OUT/lib/metalvr/OpenComposite
cp $O/LICENSE.txt $O/LICENSE_*.txt $O/LICENCE_*.txt $OUT/lib/metalvr/OpenComposite/
cp $O/libs/openxr-sdk/LICENSE $OUT/lib/metalvr/OpenComposite/LICENSE_OPENXR_SDK.txt
echo "MetalVR $V ($REV, metalsharp/MetalVR), ALVR $(git -C external/ALVR describe --tags --always), OpenHMD $(git -C external/OpenHMD-rift rev-parse --short HEAD), libsurvive $(git -C external/libsurvive rev-parse --short HEAD), Basalt $(git -C external/basalt rev-parse --short HEAD), OpenComposite $(git -C $O rev-parse --short HEAD)" \
  > $OUT/lib/metalvr/VERSION
codesign --force --sign - $OUT/bin/metalvrd $OUT/lib/libbasalt.dylib $OUT/lib/libtbb.12.dylib

tar -C $ROOT/dist -czf $ROOT/dist/$N.tar.gz $N
(cd $ROOT/dist && shasum -a 256 $N.tar.gz > $N.tar.gz.sha256)   # the format shasum -c reads
echo "$ROOT/dist/$N.tar.gz ($(cat $OUT/lib/metalvr/VERSION))"
