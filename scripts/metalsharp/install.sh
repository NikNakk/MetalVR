#!/bin/zsh
# Installs MetalVR beside MetalSharp's runtime, for testing it from MetalSharp before a release carries it:
#   ~/.metalsharp/metalvr-runtime/bin/metalvrd, bin/metalvr (the helper), lib/metalvr (the OpenXR runtime,
#   OpenComposite, licenses)
# and makes it the OpenXR runtime of the Steam prefix MetalSharp's Steam games share. Nothing inside
# MetalSharp-Wine.app or MetalSharp-Wine-x86_64.app is touched, so their signatures stay as they are.
#
#   install.sh            install from SRC (MetalSharp Wine's install/ after scripts/build-metalvr.sh)
#   install.sh restore    put every changed game back, take MetalVR out of the prefix, and remove the files
# ALVR's settings and the trusted headsets (~/.metalsharp/metalvr) are kept across both.
set -e
D=${0:A:h}
SRC=${SRC:?set SRC to the install directory of MetalSharp Wine after scripts/build-metalvr.sh}
MS=$HOME/.metalsharp
T=$MS/metalvr-runtime   # outside runtime/, which MetalSharp moves aside when it switches runtimes
CONF=$MS/metalvr
if [[ $1 == restore ]]; then
  if [[ -x $T/bin/metalvr ]]; then
    if [[ -s $CONF/games ]]; then for g in ${(f)"$(<$CONF/games)"}; do $T/bin/metalvr game $g --off || true; done; fi
    $T/bin/metalvr prefix --off || true
  fi
  pgrep -x metalvrd >/dev/null && echo "metalvrd is still running: stop it (Ctrl-C in its terminal)"
  rm -rf $T
  echo "MetalVR removed from $T (settings kept in $CONF)"
  exit 0
fi
for f in bin/metalvrd lib/metalvr/x86_64-windows/metalvr_openxr.dll lib/metalvr/x86_64-windows/metalvr_openxr.json \
         lib/metalvr/x86_64-windows/openvr_api.dll lib/metalvr/VERSION; do
  [[ -f $SRC/$f ]] || { echo >&2 "!! $SRC/$f is missing: zsh scripts/build-metalvr.sh in MetalSharp Wine"; exit 1; }
done
pgrep -x metalvrd >/dev/null && { echo >&2 "!! metalvrd is running: stop it first"; exit 1; }
rm -rf $T; mkdir -p $T/bin $T/lib $CONF
cp -p $SRC/bin/metalvrd $T/bin/metalvrd
cp -Rp $SRC/lib/metalvr $T/lib/metalvr
cp -p $D/metalvr $T/bin/metalvr; chmod +x $T/bin/metalvr
# the SteamVR driver, from the MetalVR build (metalvr steamvr registers it)
SVR=${SVR:-${D:h:h}/build/win64/steamvr/metalvr}
[[ -f $SVR/bin/win64/driver_metalvr.dll ]] && { mkdir -p $T/lib/metalvr/steamvr; cp -Rp $SVR $T/lib/metalvr/steamvr/; }
# the logo the headset's home shows
[[ -f $D/home-logo.png ]] && { mkdir -p $T/share/metalvr; cp -p $D/home-logo.png $T/share/metalvr/home-logo.png; }
codesign --verify $T/bin/metalvrd
# a copy of the prefix's registry as it was, once, before the one value is added
[[ -f $CONF/system.reg.orig ]] || cp -p $MS/prefix-steam/system.reg $CONF/system.reg.orig
$T/bin/metalvr prefix
echo "installed: $(cat $T/lib/metalvr/VERSION)"
echo "next: $T/bin/metalvr start --trust <the name the headset's ALVR app shows>"
