#!/bin/bash
# Builds ALVR's headset client with MetalVR's patches (patches/alvr, among them 0003, which lets it run on the
# Steam Frame and name itself there) as an APK to install on a standalone headset: build/alvr-client/
# alvr_client_android.apk. Everything it needs is fetched into build/android, nothing is installed elsewhere:
#   the Android NDK r27c, the SDK's command-line tools with platform 32, build-tools 34 and platform-tools,
#   a JDK 17 (Eclipse Temurin), cargo-apk (ALVR's fork), and Khronos's OpenXR loader for Android 1.0.34.
# Google's SDK license is shown by sdkmanager the first time, for you to accept or decline; nothing is accepted
# for you. Only the generic OpenXR loader is bundled, which is the one the Steam Frame (and any headset ALVR has no
# vendor loader for) uses.
#
#   scripts/build-alvr-client.sh                 then: adb install -r build/alvr-client/alvr_client_android.apk
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
A="$ROOT/build/android"
NDK_VERSION=r27c
TOOLS_ZIP=commandlinetools-mac-11076708_latest.zip
CARGO_APK_COMMIT=0fd3126dad5aa1c5f0f26cdae3410f2e5af62c60
LOADER_URL=https://github.com/KhronosGroup/OpenXR-SDK-Source/releases/download/release-1.0.34/openxr_loader_for_android-1.0.34.aar
say() { printf 'build-alvr-client: %s\n' "$*" >&2; }

# the server's build applies the patches and checks ALVR's commit
bash "$ROOT/scripts/build-alvr.sh" >/dev/null
mkdir -p "$A"

if [ ! -d "$A/android-ndk-$NDK_VERSION" ]; then
  say "fetching the Android NDK $NDK_VERSION"
  curl -fsSL -o "$A/ndk.zip" "https://dl.google.com/android/repository/android-ndk-$NDK_VERSION-darwin.zip"
  unzip -q "$A/ndk.zip" -d "$A" && rm "$A/ndk.zip"
fi
JAVA_HOME="$(ls -d "$A"/jdk-dl/jdk-17*/Contents/Home 2>/dev/null | head -1 || true)"
if [ -z "$JAVA_HOME" ]; then
  say "fetching a JDK 17"
  mkdir -p "$A/jdk-dl"
  curl -fsSL -o "$A/jdk-dl/jdk.tar.gz" \
    "https://api.adoptium.net/v3/binary/latest/17/ga/mac/aarch64/jdk/hotspot/normal/eclipse"
  tar -xzf "$A/jdk-dl/jdk.tar.gz" -C "$A/jdk-dl" && rm "$A/jdk-dl/jdk.tar.gz"
  JAVA_HOME="$(ls -d "$A"/jdk-dl/jdk-17*/Contents/Home | head -1)"
fi
export JAVA_HOME
SDKMANAGER="$A/sdk/cmdline-tools/latest/bin/sdkmanager"
if [ ! -x "$SDKMANAGER" ]; then
  say "fetching the Android SDK's command-line tools"
  mkdir -p "$A/sdk/cmdline-tools"
  curl -fsSL -o "$A/tools.zip" "https://dl.google.com/android/repository/$TOOLS_ZIP"
  rm -rf "$A/sdk/cmdline-tools/latest" "$A/sdk/cmdline-tools-dl"
  unzip -q "$A/tools.zip" -d "$A/sdk/cmdline-tools-dl" && rm "$A/tools.zip"
  mv "$A/sdk/cmdline-tools-dl/cmdline-tools" "$A/sdk/cmdline-tools/latest" && rmdir "$A/sdk/cmdline-tools-dl"
fi
if [ ! -d "$A/sdk/platforms/android-32" ] || [ ! -d "$A/sdk/build-tools/34.0.0" ]; then
  say "Google's Android SDK license follows; installing needs it accepted"
  "$SDKMANAGER" --sdk_root="$A/sdk" --licenses
  "$SDKMANAGER" --sdk_root="$A/sdk" "platforms;android-32" "build-tools;34.0.0" "platform-tools"
fi
if [ ! -x "$A/tools/bin/cargo-apk" ]; then
  say "building cargo-apk"
  cargo install --quiet --git https://github.com/zarik5/cargo-apk --rev "$CARGO_APK_COMMIT" cargo-apk --root "$A/tools"
fi
rustup target add aarch64-linux-android >/dev/null 2>&1
loader="$ROOT/external/ALVR/deps/android_openxr/arm64-v8a/libopenxr_loader.so"
if [ ! -f "$loader" ]; then
  say "fetching Khronos's OpenXR loader for Android"
  rm -rf "$A/loader-dl" && mkdir -p "$A/loader-dl" "$(dirname "$loader")"
  curl -fsSL -o "$A/loader-dl/loader.aar" "$LOADER_URL"
  unzip -q "$A/loader-dl/loader.aar" -d "$A/loader-dl/x"
  cp "$A/loader-dl/x/prefab/modules/openxr_loader/libs/android.arm64-v8a/libopenxr_loader.so" "$loader"
fi

export ANDROID_HOME="$A/sdk" ANDROID_SDK_ROOT="$A/sdk"
export ANDROID_NDK_ROOT="$A/android-ndk-$NDK_VERSION" ANDROID_NDK_HOME="$A/android-ndk-$NDK_VERSION"
export PATH="$A/tools/bin:$PATH"
say "building the client (release)"
(cd "$ROOT/external/ALVR" && cargo xtask build-client --release)
mkdir -p "$ROOT/build/alvr-client"
cp "$ROOT/external/ALVR/build/alvr_client_android/alvr_client_android.apk" "$ROOT/build/alvr-client/"
say "built $ROOT/build/alvr-client/alvr_client_android.apk"
