#!/bin/sh
# Runs the smoke test the way a game runs: the Windows test client and runtime DLL under Wine with Mullion's
# Direct3D 11, against the native metalvrd with the simulated headset. Leaves the encoded stream, one composed
# frame and the logs in build/out/.
#
#   METALVR_WINE_ROOT=/path/to/MetalSharp-Wine-x86_64.app/Contents scripts/test-wine.sh [xr_smoke arguments...]
#
# METALVR_WINE_ROOT   a Wine tree: MacOS/wine (or bin/wine) with Mullion (lib/dxmt or lib/mullion beside lib/wine,
#                     as MetalSharp-Wine ships it)
# METALVR_WINEPREFIX  the prefix to run in (default build/prefix, created on first use)
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
native=${METALVR_BUILD:-$root/build/native}
win=${METALVR_WIN_BUILD:-$root/build/win64}
out=$root/build/out
name=${METALVR_TEST_NAME:-wine}
port=${METALVR_PORT:-37951}
wine_root=${METALVR_WINE_ROOT:-$HOME/.metalsharp/runtime/MetalSharp-Wine-x86_64.app/Contents}

if [ -x "$wine_root/MacOS/wine" ]; then
  wine=$wine_root/MacOS/wine
elif [ -x "$wine_root/bin/wine" ]; then
  wine=$wine_root/bin/wine
else
  echo "no wine under $wine_root" >&2
  exit 2
fi
mullion=
for candidate in "$wine_root/lib/mullion" "$wine_root/lib/dxmt"; do
  # x86_64 Wine carries x86_64 DLLs; arm64 Wine carries ARM64X ones that x86_64 programs load as ARM64EC
  if [ -f "$candidate/x86_64-windows/d3d11.dll" ] || [ -f "$candidate/aarch64-windows/d3d11.dll" ]; then
    mullion=$candidate
    break
  fi
done
if [ -z "$mullion" ]; then
  echo "no Mullion (lib/mullion or lib/dxmt with a d3d11.dll) under $wine_root" >&2
  exit 2
fi

mkdir -p "$out"
rm -f "$out/$name.h264" "$out/$name.png" "$out/$name.daemon.log" "$out/$name.client.log"

export WINEPREFIX=${METALVR_WINEPREFIX:-$root/build/prefix}
export WINEDEBUG=${WINEDEBUG:--all}
# MetalSharp-Wine chooses its Direct3D by GRAPHICS_BACKEND: with "dxmt" its ntdll routes d3d11, dxgi and winemetal
# to the Mullion tree beside Wine. A stock Wine with Mullion installed into lib/wine needs nothing and ignores this.
export GRAPHICS_BACKEND=${GRAPHICS_BACKEND:-dxmt}
export WINEDLLOVERRIDES="${WINEDLLOVERRIDES:-mscoree,mshtml=d}"
if [ ! -d "$WINEPREFIX/drive_c/windows/system32" ]; then
  "$wine" wineboot -u >/dev/null 2>&1 || true
fi

"$native/daemon/metalvrd" --driver sim --port "$port" --verbose \
  --dump "$out/$name.h264" --snapshot "$out/$name.png" --snapshot-frame 60 --exit-after-session ${METALVR_DAEMON_ARGS:-} \
  >"$out/$name.daemon.log" 2>&1 &
daemon=$!
trap 'kill "$daemon" 2>/dev/null || true' EXIT
sleep 1

status=0
# METALVR_LOADER names a Khronos loader DLL: the client then finds the runtime as a game does, through the
# prefix's ActiveRuntime (scripts/install-prefix.sh), instead of loading the runtime DLL by path
if [ -n "${METALVR_LOADER:-}" ]; then
  how="--loader $(printf 'Z:%s' "$METALVR_LOADER" | tr / '\\')"
else
  how="--runtime $(printf 'Z:%s' "$win/runtime/metalvr_openxr.dll" | tr / '\\')"
fi
(cd "$win/tests" && METALVR_PORT=$port METALVR_LOG=1 "$wine" xr_smoke.exe $how "$@") \
  >"$out/$name.client.log" 2>&1 || status=$?

# The server leaves by itself once the session's client is gone, after the encoder has put out its last frames;
# the stream is only complete when it has exited.
waited=0
while kill -0 "$daemon" 2>/dev/null && [ "$waited" -lt 100 ]; do
  sleep 0.1
  waited=$((waited + 1))
done
trap - EXIT
kill "$daemon" 2>/dev/null || true
wait "$daemon" 2>/dev/null || true

echo "--- client"
cat "$out/$name.client.log"
echo "--- server"
cat "$out/$name.daemon.log"

# The client passing only says the runtime accepted the frames. The stream the headset would get has to hold
# every one of them, and decode.
echo "--- stream"
case " ${METALVR_DAEMON_ARGS:-} " in
*" hevc "*) codec=--hevc ;;
*) codec= ;;
esac
rendered=$(sed -n 's/^PASSED: \([0-9]*\) frames rendered.*/\1/p' "$out/$name.client.log")
if check=$("$native/tests/stream_check" "$out/$name.h264" $codec 2>&1); then
  echo "$check"
  decoded=$(printf '%s\n' "$check" | sed -n 's/.* \([0-9]*\) decoded.*/\1/p')
  if [ -n "$rendered" ] && [ "$decoded" != "$rendered" ]; then
    echo "FAILED: the application rendered $rendered frames and the stream holds $decoded"
    [ "$status" -ne 0 ] || status=1
  fi
else
  echo "$check"
  echo "FAILED: the encoded stream does not decode"
  [ "$status" -ne 0 ] || status=1
fi

# and the composed frame has to be the picture the client drew
echo "--- frame"
pattern=none
case " $* " in
*" --no-quad "*)
  case " $* " in
  *" --samples "* | *" --format "*) ;;
  *" --flip "*) pattern=bottom ;;
  *) pattern=top ;;
  esac
  ;;
esac
"$native/tests/frame_check" "$out/$name.png" --pattern "$pattern" || { [ "$status" -ne 0 ] || status=1; }
exit "$status"
