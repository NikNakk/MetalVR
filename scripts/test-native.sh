#!/bin/sh
# Runs the smoke test natively: metalvrd with the simulated headset, and the Metal test client against the
# macOS runtime. Leaves the encoded stream, one composed frame and the logs in build/out/.
#
#   scripts/test-native.sh [xr_smoke arguments...]
#
# METALVR_DAEMON_ARGS adds arguments to metalvrd, e.g. "--eye 2144x2336 --codec hevc".
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
build=${METALVR_BUILD:-$root/build/native}
out=$root/build/out
name=${METALVR_TEST_NAME:-native}
port=${METALVR_PORT:-37950}
mkdir -p "$out"
rm -f "$out/$name.h264" "$out/$name.png" "$out/$name.daemon.log" "$out/$name.client.log"

"$build/daemon/metalvrd" --driver sim --port "$port" --verbose \
  --dump "$out/$name.h264" --snapshot "$out/$name.png" --snapshot-frame 60 --exit-after-session ${METALVR_DAEMON_ARGS:-} \
  >"$out/$name.daemon.log" 2>&1 &
daemon=$!
trap 'kill "$daemon" 2>/dev/null || true' EXIT
sleep 1

status=0
METALVR_PORT=$port METALVR_LOG=1 "$build/tests/xr_smoke" \
  --runtime "$build/runtime/libmetalvr_openxr.dylib" "$@" >"$out/$name.client.log" 2>&1 || status=$?

# the server leaves by itself once the session's client is gone
for _ in 1 2 3 4 5 6 7 8 9 10; do
  kill -0 "$daemon" 2>/dev/null || break
  sleep 0.3
done
trap - EXIT
kill "$daemon" 2>/dev/null || true

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
if check=$("$build/tests/stream_check" "$out/$name.h264" $codec 2>&1); then
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
"$build/tests/frame_check" "$out/$name.png" --pattern "$pattern" || { [ "$status" -ne 0 ] || status=1; }
exit "$status"
