#!/bin/sh
# Makes MetalVR the active OpenXR runtime of a Wine prefix.
#
#   scripts/install-prefix.sh WINE PREFIX [RUNTIME_DIR]
#
# WINE         the wine binary that runs the prefix
# PREFIX       the prefix (WINEPREFIX)
# RUNTIME_DIR  where metalvr_openxr.dll and metalvr_openxr.json were built (default build/win64/runtime)
#
# The runtime is copied to C:\Program Files\MetalVR and named as ActiveRuntime under
# HKLM\SOFTWARE\Khronos\OpenXR\1, which is where every game's OpenXR loader looks. `--remove` as a fourth
# argument takes both away again.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
if [ $# -lt 2 ]; then
  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//' >&2
  exit 2
fi
wine=$1
prefix=$2
runtime=${3:-$root/build/win64/runtime}
action=${4:-}

target="$prefix/drive_c/Program Files/MetalVR"
key='HKLM\SOFTWARE\Khronos\OpenXR\1'
export WINEPREFIX="$prefix"
export WINEDEBUG=${WINEDEBUG:--all}

if [ "$action" = "--remove" ]; then
  "$wine" reg delete "$key" /v ActiveRuntime /f >/dev/null 2>&1 || true
  rm -rf "$target"
  echo "MetalVR removed from $prefix"
  exit 0
fi

for file in metalvr_openxr.dll metalvr_openxr.json; do
  if [ ! -f "$runtime/$file" ]; then
    echo "$runtime/$file is missing; build the Windows runtime first" >&2
    exit 1
  fi
done
if [ ! -d "$prefix/drive_c" ]; then
  echo "$prefix is not a Wine prefix" >&2
  exit 1
fi
mkdir -p "$target"
cp -f "$runtime/metalvr_openxr.dll" "$runtime/metalvr_openxr.json" "$target/"
"$wine" reg add "$key" /v ActiveRuntime /t REG_SZ /d 'C:\Program Files\MetalVR\metalvr_openxr.json' /f >/dev/null
echo "MetalVR is the active OpenXR runtime of $prefix"
