# Valve Index

**Tried on hardware:** no. Built from the open Lighthouse drivers (Monado's, OpenHMD's and libsurvive) and checked
without the headset. If you have one, a report is very welcome.

The Index uses the `lighthouse` driver, which it shares with the HTC Vives ([HTC_VIVE.md](HTC_VIVE.md)), and the
`metalvr-lighthouse` script (the release's `metalvr-rift` under another name, which looks for Lighthouse headsets).
Its position and its controllers' come from your base stations, through libsurvive, which the release includes.

## What you need

- An Apple silicon Mac with a **DisplayPort output**: a USB-C or Thunderbolt port with a USB-C to DisplayPort cable
  or adapter. The Index's USB in a USB 3 port, and its power adapter.
- Base stations (1.0 or 2.0), and the Index controllers already paired with the headset (pairing is done once in
  SteamVR on Windows or Linux; the pairing lives in the headset's receivers).
- MetalSharp, its Windows Steam, and SteamVR there for SteamVR games.

## Setting up and playing

The steps are the CV1's ([RIFT_CV1.md](RIFT_CV1.md)) with `metalvr-lighthouse` in place of `metalvr-rift`:

```sh
metalvr-lighthouse install       # from the unpacked release, if not installed yet
metalvr-lighthouse display       # once per Mac: macOS hides the Index's display (5996:91a8) as a headset's
metalvr-lighthouse setup         # once: SteamVR in MetalSharp's Steam
metalvr-lighthouse steamvr       # the Lighthouse server, then SteamVR
metalvr-lighthouse game APPID    # the Lighthouse server, then a Steam game in VR
```

- `metalvr-lighthouse start --refresh 120` (or 80, 90, 144) sets the Index's refresh rate.
- `--passthrough` shows the room through the Index's front cameras in MetalVR's home; macOS asks for camera access
  the first time.
- The first time, libsurvive works out where your base stations stand: keep the headset still in view of them for a
  few seconds. It remembers them in `~/Library/Application Support/MetalVR/libsurvive.json`.

## What works, checked without the headset

| | Checked by |
|---|---|
| Finding the Index on USB (`28de:2300`, confirmed by the model in its configuration) | `headset-protocols` |
| Its configuration from its firmware: lenses, canted displays, IMU calibration | `headset-protocols` |
| Its lens correction and canted eyes, drawn offscreen | `panel-index` |
| Head rotation from its IMU, the proximity sensor, the IPD | `headset-protocols` |
| The Index controllers through their Watchman radio: buttons, trackpad, each finger's curl, grip force, rotation, battery, haptics | `headset-protocols` |
| The Index controller profile in OpenXR | `native-index-touch` |
| Positions of the head and controllers from the base stations, through libsurvive | `libsurvive` (libsurvive's own simulated room) |
| The home showing the room through the front cameras | `passthrough` |

Built, but not covered by a test: SteamVR's Index controllers with each finger's curl and the hand skeleton, from
MetalVR's SteamVR driver.

## Unknown until tried on one

- Whether macOS shows the Index's display through a USB-C to DisplayPort adapter, and which refresh rates it offers
  (144 Hz only just fits DisplayPort 1.2).
- Whether the display needs a power-on command, and USB access to its interfaces on macOS.
- libsurvive's tracking with real base stations on a Mac (it has run only in its simulated room).
- The controllers' pairing state and haptics on real hardware.
- The cameras are drawn without their own calibration: the passthrough view is a fixed field of view.

## Not supported

- Eye tracking (the Index has none) and passthrough for games (only MetalVR's home shows it).

## When it doesn't work

- **The headset stays dark.** `metalvr-lighthouse status`, then `metalvr-lighthouse display` if the display is
  hidden; check the DisplayPort cable and the headset's power.
- **Rotation only.** The log says whether libsurvive started and what it sees; check the base stations are on and
  in view.
- Everything else is as for the CV1 ([RIFT_CV1.md](RIFT_CV1.md#when-it-doesnt-work)).

The research and sources: [HEADSETS.md](../HEADSETS.md#phase-f-valve-index).
