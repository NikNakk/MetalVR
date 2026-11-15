#!/bin/sh
# Checks that the runtime fails the way it says it does, natively:
#   1. no server            -> xrCreateInstance is XR_ERROR_RUNTIME_UNAVAILABLE (-51)
#   2. server dies mid-run  -> the application is told the instance is lost, and does not hang
#   3. a second application -> cannot begin a session while the first holds the headset
set -u
root=$(cd "$(dirname "$0")/.." && pwd)
build=${METALVR_BUILD:-$root/build/native}
out=$root/build/out
port=${METALVR_PORT:-37953}
runtime=$build/runtime/libmetalvr_openxr.dylib
client="$build/tests/xr_smoke --runtime $runtime"
mkdir -p "$out"
failures=0

check() { # description, file, pattern
  if grep -q -e "$3" "$2"; then
    echo "ok    $1"
  else
    echo "FAIL  $1 (no '$3' in $2)"
    failures=$((failures + 1))
  fi
}

# 1
METALVR_PORT=$port $client --frames 5 >"$out/fail-noserver.log" 2>&1
[ $? -ne 0 ] || { echo "FAIL  the client succeeded with no server"; failures=$((failures + 1)); }
check "no server: runtime unavailable" "$out/fail-noserver.log" "xrCreateInstance(&instance_info, &instance) -> -51"

# 2
"$build/daemon/metalvrd" --driver sim --port "$port" ${METALVR_DAEMON_ARGS:-} >"$out/fail-kill.daemon.log" 2>&1 &
daemon=$!
sleep 1
METALVR_PORT=$port $client --frames 1000000 >"$out/fail-kill.log" 2>&1 &
app=$!
sleep 2
kill -9 "$daemon" 2>/dev/null
waited=0
while kill -0 "$app" 2>/dev/null && [ "$waited" -lt 50 ]; do
  sleep 0.1
  waited=$((waited + 1))
done
if kill -0 "$app" 2>/dev/null; then
  echo "FAIL  the client hung after the server died"
  failures=$((failures + 1))
  kill -9 "$app" 2>/dev/null
fi
wait "$app" 2>/dev/null
check "server killed: instance lost" "$out/fail-kill.log" "-> -13\|lost its server"

# 3
"$build/daemon/metalvrd" --driver sim --port "$port" ${METALVR_DAEMON_ARGS:-} >"$out/fail-busy.daemon.log" 2>&1 &
daemon=$!
sleep 1
METALVR_PORT=$port $client --frames 400 >"$out/fail-busy-first.log" 2>&1 &
first=$!
sleep 2
METALVR_PORT=$port $client --frames 5 >"$out/fail-busy-second.log" 2>&1
check "second application: session refused" "$out/fail-busy-second.log" "xrBeginSession(session, &begin) -> -2"
wait "$first"
[ $? -eq 0 ] || { echo "FAIL  the first application was disturbed by the second"; failures=$((failures + 1)); }
check "first application: unaffected" "$out/fail-busy-first.log" "PASSED"
kill "$daemon" 2>/dev/null
wait "$daemon" 2>/dev/null

[ "$failures" -eq 0 ] && echo "all failure cases behave" || echo "$failures failure case(s) wrong"
exit "$failures"
