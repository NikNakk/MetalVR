#!/bin/zsh
# Which headset the ALVR driver names, without a headset: for each model an ALVR client reports, a session whose
# trusted client last reported it, and the driver must name that headset before anything connects. A name MetalVR
# does not know (a Pico, "Android (unknown)" from the stock client on a Steam Frame) must stay an unknown headset,
# never a Rift; MetalVR's patched client names a Steam Frame (patches/alvr/0003).
#
#   scripts/test-alvr-models.sh            (needs build/native with the ALVR driver: scripts/build-alvr.sh)
set -eu
root=${0:A:h:h}
build=${METALVR_BUILD:-$root/build/native}
smoke=$build/tests/alvr_driver_smoke
[[ -x $smoke ]] || { print -r -- "no $smoke: build with the ALVR driver first" >&2; exit 1; }
work=$(mktemp -d)
trap 'rm -rf $work' EXIT
host=1234.client

# a full session as ALVR writes it, made once
"$smoke" $work/seed 0.5 $host >$work/seed.log 2>&1 || { cat $work/seed.log; exit 1; }

failed=0
for pair in "Quest 1:quest-1" "Quest 2:quest-2" "Quest 3:quest-3" "Quest 3S:quest-3s" "Quest Pro:quest-pro" \
            "Steam Frame:steam-frame" "Pico 4:unknown" "Android (unknown):unknown" "Unknown:unknown"; do
  name=${pair%%:*} key=${pair##*:}
  dir=$work/${key}-${name// /_}
  cp -R $work/seed $dir
  python3 - "$dir/session.json" "$host" "$name" <<'PY'
import json, sys
path, host, name = sys.argv[1:4]
session = json.load(open(path))
session["client_connections"][host]["display_name"] = name
json.dump(session, open(path, "w"), indent=2)
PY
  if METALVR_EXPECT_HEADSET=$key "$smoke" $dir 0.3 $host >$dir.log 2>&1; then
    print -r -- "ok    \"$name\" -> $key"
  else
    print -r -- "FAIL  \"$name\" -> $key"; grep -E "FAIL|headset before" $dir.log; failed=1
  fi
done

# a headset on USB: without an adb the driver refuses to start (ALVR would otherwise download one); with one it
# starts and ALVR downloads nothing
if PATH=/usr/bin:/bin "$smoke" $work/wired-none 0.3 client.wired >$work/wired-none.log 2>&1; then
  print -r -- "FAIL  wired without adb started"; failed=1
elif grep -q "needs adb" $work/wired-none.log; then
  print -r -- "ok    wired without adb refused"
else
  print -r -- "FAIL  wired without adb: no reason given"; cat $work/wired-none.log; failed=1
fi
mkdir -p $work/bin && print '#!/bin/sh\nexit 0' >$work/bin/adb && chmod +x $work/bin/adb
if PATH=$work/bin:/usr/bin:/bin "$smoke" $work/wired 0.3 client.wired >$work/wired.log 2>&1 &&
   [[ ! -e $work/wired/platform-tools ]]; then
  print -r -- "ok    wired with adb started"
else
  print -r -- "FAIL  wired with adb"; grep -E "FAIL|adb" $work/wired.log; failed=1
fi
exit $failed
