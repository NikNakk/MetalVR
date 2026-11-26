# Sony PlayStation VR2

**Tried on hardware:** native OpenXR rendering at 4000×2040 and 90 Hz, and optical Sense tracking with the
Monado bridge described below.

The PS VR2 has its own driver, `psvr2`. It tracks itself: its cameras and processor work out where it is and send
the finished pose over USB, so MetalVR needs no base stations and no camera tracker for it.

## What you need

- **Sony's PlayStation VR2 PC adapter**, with its power supply. The headset's one USB-C cable carries video, USB and
  12 V power, which no Mac port gives together.
- An Apple silicon Mac with a **DisplayPort output**: a USB-C or Thunderbolt port with a USB-C to DisplayPort cable
  into the adapter's DisplayPort input. The adapter's USB plug into a USB 3 port on the Mac.
- The Sense controllers paired with the Mac over Bluetooth (System Settings, Bluetooth; hold the controller's
  Create or Options button and the PS button until its light blinks).
- MetalSharp, its Windows Steam, and SteamVR there for SteamVR games.

## Setting up and playing

The steps are the CV1's ([RIFT_CV1.md](RIFT_CV1.md)) with `metalvr-psvr2` in place of `metalvr-rift`, except that
there is no display step: macOS does not hide the PS VR2's display (it ships no override for Sony's displays).

```sh
metalvr-psvr2 install          # from the unpacked release, if not installed yet
metalvr-psvr2 setup            # once: SteamVR in MetalSharp's Steam
metalvr-psvr2 steamvr          # the PlayStation VR2 server, then SteamVR
metalvr-psvr2 game APPID       # the server, then a Steam game in VR
```

Or run the server by itself: `metalvr psvr2` (`metalvrd --driver psvr2`).

## Sense 6DoF

The optional bridge uses the same PSVR2 USB driver, Sense HID driver and optical tracking pipeline as the
macOS Monado fork. MetalVR handles presentation and exposes the hands through its existing Touch bindings.
The bridge and both controllers use the same head tracking space. OpenXR reports Monado's position and
orientation tracking status separately, including during optical loss and reacquisition.

Install the build dependencies with `brew install cmake ninja eigen pkgconf`, then build from a MetalVR checkout.

```sh
scripts/build-pssense.sh
```

The script fetches a pinned Monado revision. Point `METALVR_MONADO_SOURCE` at a local Monado checkout to build
against the same source you're developing there. The bridge loads from `build/pssense` in a source build,
or from `lib/` in a packaged release. Set `METALVR_PSSENSE_LIBRARY` to select another build.

Pair both Sense controllers with macOS. Close other applications or services that have the PSVR2 open.
Use the same four-camera calibration JSON as your Monado 6DoF setup for this headset.

```sh
export PSVR2_SENSE_6DOF=1
export PSVR2_SENSE_6DOF_CALIBRATION=/absolute/path/to/your-mode4-calibration.json
build/native/daemon/metalvrd --driver psvr2 --verbose
```

For an installed release, run `metalvr-psvr2 start` with the same environment. To include the bridge in a
MetalVR release, build with `METALVR_PACKAGE_PSSENSE=1 scripts/package-release.sh VERSION`. Calibration
is specific to the headset and is supplied when starting the daemon.

The daemon logs `using Monado for headset and Sense tracking` once it opens the headset. To verify tracking
through native OpenXR, run the following in a second terminal with both controllers in the headset's camera
view. The test prints how many frames had tracked position and orientation for each hand.

```sh
METALVR_HEADSET_WAIT=15 build/native/tests/xr_smoke \
  --runtime "$PWD/build/native/runtime/libmetalvr_openxr.dylib" \
  --frames 1500 --require-position-tracked
```

The standard PSVR2 backend provides controller rotation and an arm model. It is used when 6DoF is disabled
or the bridge or calibration path is unavailable. Once the bridge opens the headset, Monado reports any
calibration or tracker startup failures and manages the controllers' tracking state.

## Automated checks without the headset

| | Checked by |
|---|---|
| Its status report: worn, the function button, the IPD dial, and its IMU at 2 kHz turned into the headset's axes | `headset-protocols` |
| Its own tracking's pose: the record, its axes, Monado's correction, and the quaternion kept continuous | `headset-protocols` |
| The requests that run it: camera mode, brightness, firmware and serial number, lens calibration | `headset-protocols` |
| Its lens: Monado's model with its colour table and each eye's calibration, on the CPU and in the shader | `headset-protocols`, `panel-psvr2` |
| The Sense controllers' reports: buttons (cross and circle as A and B, square and triangle as X and Y), stick, trigger, touches, battery, the Bluetooth CRC | `headset-protocols` |
| Their IMU calibration, and their vibration report | `headset-protocols` |
| The PS VR2 and its Sense controllers as OpenXR sees them (Touch) | `native-psvr2-sense` |
| The driver waiting for the headset without one, and stopping cleanly | by hand |

Built, but not covered by a test: opening the headset with IOUSBHost and reading its endpoints, the Sense
controllers over IOHID, and SteamVR showing them as Touch controllers named for the Sense.

## Not supported

- Eye tracking, passthrough, the headset's rumble, and the adaptive triggers.

## When it doesn't work

- **No headset found.** `metalvr-psvr2 status` says whether a PlayStation VR2 (`054c:0cde`) is on USB; the log says
  which interface or request failed.
- **The headset is found, its display stays dark.** The log says `psvr2: the headset's DisplayPort input has a
  signal` once the adapter's video reaches it; check the DisplayPort cable and the adapter's power.
- **No controllers.** The log names each Sense controller when it connects; pair them in System Settings first.

The research and sources: [HEADSETS.md](../HEADSETS.md#phase-j-sony-playstation-vr2).
