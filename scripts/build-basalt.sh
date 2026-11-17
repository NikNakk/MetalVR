#!/bin/sh
# Builds the Rift S's camera tracking (docs/HEADSETS.md, phase I) into build/basalt/prefix, without installing anything
# outside the build folder:
#   Basalt (BSD-3-Clause), Monado's fork, as libbasalt.dylib with Monado's VIT interface: the visual-inertial tracker
#     metalvrd loads when it starts a Rift S, and gives the headset's cameras and IMU
#   oneTBB (Apache-2.0), OpenCV 4.10 core/imgproc/calib3d/features2d/flann (Apache-2.0) and fmt (MIT), what Basalt needs
# scripts/fetch-deps.sh fetches all of it into external/. metalvrd finds the libraries in build/basalt/prefix/lib (or
# next to itself in a release) when it runs; it is not linked against them.
#
#   scripts/build-basalt.sh
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
out=$root/build/basalt
prefix=$out/prefix
jobs=$(sysctl -n hw.ncpu)
for dir in basalt oneTBB opencv fmt; do
  [ -d "$root/external/$dir" ] || { echo "external/$dir is missing: run scripts/fetch-deps.sh" >&2; exit 1; }
done
mkdir -p "$out"

common="-G Ninja -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64
  -DCMAKE_OSX_DEPLOYMENT_TARGET=15.0 -DCMAKE_INSTALL_PREFIX=$prefix -DCMAKE_PREFIX_PATH=$prefix
  -DCMAKE_INSTALL_NAME_DIR=@rpath"

step() { # name, then the source directory and its cmake options
  name=$1; shift
  src=$1; shift
  if [ -f "$out/$name.done" ]; then echo "$name: built"; return; fi
  echo "$name: building"
  # shellcheck disable=SC2086
  { cmake -S "$src" -B "$out/$name-build" $common "$@" && cmake --build "$out/$name-build" -j "$jobs" &&
    cmake --install "$out/$name-build"; } >"$out/$name.log" 2>&1 || { tail -40 "$out/$name.log"; exit 1; }
  touch "$out/$name.done"
}

step oneTBB "$root/external/oneTBB" -DTBB_TEST=OFF -DTBB_EXAMPLES=OFF -DTBB_STRICT=OFF -DBUILD_SHARED_LIBS=ON
step fmt "$root/external/fmt" -DFMT_TEST=OFF -DFMT_DOC=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
step opencv "$root/external/opencv" -DBUILD_LIST=core,imgproc,calib3d,features2d,flann,highgui \
  -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DWITH_TBB=OFF -DWITH_OPENCL=OFF -DWITH_FFMPEG=OFF \
  -DWITH_JPEG=OFF -DWITH_PNG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENJPEG=OFF -DWITH_JASPER=OFF \
  -DWITH_OPENEXR=OFF -DWITH_IMGCODEC_HDR=OFF -DWITH_IMGCODEC_SUNRASTER=OFF -DWITH_IMGCODEC_PXM=OFF \
  -DWITH_IMGCODEC_PFM=OFF -DWITH_AVFOUNDATION=OFF -DWITH_ITT=OFF -DWITH_IPP=OFF -DWITH_LAPACK=OFF -DWITH_EIGEN=OFF \
  -DWITH_PROTOBUF=OFF -DWITH_QT=OFF -DWITH_GTK=OFF -DWITH_COCOA=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF -DBUILD_opencv_python3=OFF -DBUILD_ZLIB=ON \
  -DWITH_ADE=OFF -DBUILD_opencv_gapi=OFF
step basalt "$root/external/basalt" -DBASALT_BUILD_SHARED_LIBRARY_ONLY=ON -DBASALT_BUILD_VISUALIZATION=OFF \
  -DBASALT_ENABLE_ROSBAG2=OFF -DBASALT_INSTANTIATIONS_DOUBLE=OFF -DBUILD_TESTING=OFF \
  -DSKIP_PERFORMANCE_COMPARISON=ON -DBUILD_SANDBOX=OFF -DBUILD_DOC=OFF \
  -DEIGEN_ROOT="$root/external/basalt/thirdparty/basalt-headers/thirdparty/eigen" \
  -DOpenCV_DIR="$prefix/lib/cmake/opencv4" -DCMAKE_INSTALL_RPATH=@loader_path

echo "basalt $(git -C "$root/external/basalt" rev-parse --short HEAD) built into $prefix"
ls "$prefix/lib" | grep -E '^lib(basalt|tbb).*\.dylib$' | sed 's/^/  /'
