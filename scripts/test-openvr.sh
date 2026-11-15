#!/bin/sh
# Runs the SteamVR path end to end: an OpenVR Direct3D 11 application, through OpenComposite's openvr_api.dll,
# on MetalVR's runtime under Wine with Mullion, against metalvrd with the simulated headset.
#
#   METALVR_WINE_ROOT=/path/to/MetalSharp-Wine-x86_64.app/Contents scripts/test-openvr.sh
#
# Needs build/opencomposite/openvr_api.dll (scripts/build-opencomposite.sh) and both MetalVR builds. Every case
# must exit cleanly, have none of its frames rejected, and compose the picture the application submitted.
set -u
root=$(cd "$(dirname "$0")/.." && pwd)
native=${METALVR_BUILD:-$root/build/native}
win=${METALVR_WIN_BUILD:-$root/build/win64}
out=$root/build/out
stage=$out/openvr-run
port=${METALVR_PORT:-37958}
wine_root=${METALVR_WINE_ROOT:-$HOME/.metalsharp/runtime/MetalSharp-Wine-x86_64.app/Contents}
api=$root/build/opencomposite/openvr_api.dll
headers=$root/external/OpenComposite/OpenVRHeaders

if [ -x "$wine_root/MacOS/wine" ]; then
  wine=$wine_root/MacOS/wine
  server=$wine_root/MacOS/wineserver
else
  wine=$wine_root/bin/wine
  server=$wine_root/bin/wineserver
fi
[ -x "$wine" ] || { echo "no wine under $wine_root" >&2; exit 2; }
[ -f "$api" ] || { echo "$api is missing; run scripts/build-opencomposite.sh" >&2; exit 2; }

export WINEPREFIX=${METALVR_WINEPREFIX:-$root/build/prefix}
export WINEDEBUG=${WINEDEBUG:--all}
export GRAPHICS_BACKEND=${GRAPHICS_BACKEND:-dxmt}
export WINEDLLOVERRIDES="${WINEDLLOVERRIDES:-mscoree,mshtml=d}"
[ -d "$WINEPREFIX/drive_c/windows/system32" ] || "$wine" wineboot -u >/dev/null 2>&1
"$root/scripts/install-prefix.sh" "$wine" "$WINEPREFIX" "$win/runtime" >/dev/null || exit 2

# the application, beside the DLL under the name it is linked against
rm -rf "$stage"
mkdir -p "$stage"
cp "$api" "$stage/vrclient_x64.dll"
cp "$root/tests/openvr/actions.json" "$root/tests/openvr/bindings_touch.json" "$stage/"
x86_64-w64-mingw32-g++ -O1 -static -I"$headers" "$root/tests/openvr/ovr_smoke.cpp" -o "$stage/ovr_smoke.exe" \
  "$stage/vrclient_x64.dll" -ld3d11 || { echo "the OpenVR test application did not build" >&2; exit 2; }

failures=0
run() { # name, frame-check pattern, extra grep that must match the client log (or ""), ovr_smoke arguments...
  name=$1 pattern=$2 expect=$3
  shift 3
  "$native/daemon/metalvrd" --driver sim --port "$port" --verbose --dump "$stage/$name.h264" \
    --snapshot "$stage/$name.png" --snapshot-frame 100 >"$stage/$name.daemon.log" 2>&1 &
  daemon=$!
  sleep 1
  (cd "$stage" && METALVR_PORT=$port METALVR_LOG=1 "$wine" ovr_smoke.exe "$@") >"$stage/$name.client.log" 2>&1 &
  app=$!
  # an OpenComposite abort is a message box that never goes away by itself
  waited=0
  while kill -0 "$app" 2>/dev/null && [ "$waited" -lt 90 ]; do
    sleep 1
    waited=$((waited + 1))
  done
  problem=
  if kill -0 "$app" 2>/dev/null; then
    "$server" -k 2>/dev/null
    kill "$app" 2>/dev/null
    problem="did not finish (an OpenComposite abort shows a message box)"
  fi
  wait "$app" 2>/dev/null
  status=$?
  sleep 1
  kill "$daemon" 2>/dev/null
  wait "$daemon" 2>/dev/null
  [ -n "$problem" ] || [ "$status" -eq 0 ] || problem="exit status $status"
  [ -n "$problem" ] || grep -q '^done' "$stage/$name.client.log" || problem="did not reach VR_Shutdown"
  if [ -z "$problem" ] && grep -q 'rejected the frame' "$stage/$name.client.log"; then
    problem="frames were rejected by the server"
  fi
  if [ -z "$problem" ] && [ -n "$expect" ] && ! grep -q -e "$expect" "$stage/$name.client.log" "$stage/$name.daemon.log"; then
    problem="no '$expect' in the logs"
  fi
  if [ -z "$problem" ] && ! "$native/tests/frame_check" "$stage/$name.png" --pattern "$pattern" >"$stage/$name.frame.log" 2>&1; then
    problem="the composed frame is wrong: $(tr '\n' ' ' <"$stage/$name.frame.log")"
  fi
  if [ -z "$problem" ]; then
    echo "ok    $name"
  else
    echo "FAIL  $name: $problem"
    failures=$((failures + 1))
  fi
}

run legacy none 'ctl1 conn=1 valid=1 ctl2 conn=1 valid=1' --frames 150
run upright top '' --frames 150 --pattern
run flipped bottom '' --frames 150 --pattern --flipped
run typeless-msaa none '' --frames 150 --typeless --msaa
run manifest none 'sim: haptic' --frames 150 --manifest "$(printf 'Z:%s' "$stage/actions.json" | tr / '\\')"

[ "$failures" -eq 0 ] && echo "the SteamVR path works for every case" || echo "$failures case(s) failed; logs in $stage"
exit "$failures"
