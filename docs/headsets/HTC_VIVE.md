# HTC Vive, Vive Pro and Vive Pro 2

**Tried on hardware:** no. Built from the open Lighthouse drivers (Monado's, OpenHMD's, libsurvive, and the
community Vive Pro 2 driver) and checked without the headsets. Reports from owners are very welcome.

The Vives use the `lighthouse` driver, shared with the [Valve Index](VALVE_INDEX.md), and the `metalvr-lighthouse`
script, which finds which one is on USB. Their position and their controllers' come from your base stations through
libsurvive, which the release includes.

| | Vive | Vive Pro | Vive Pro 2 |
|---|---|---|---|
| Video into the Mac | HDMI (the Mac's own HDMI port) or mini DisplayPort | DisplayPort (USB-C or Thunderbolt to DisplayPort) | DisplayPort; its sharpest modes need DisplayPort's compression (DSC) |
| USB | `0bb4:2c87` with `28de:2000` | `0bb4:0309` or `030e` with `28de:2300` | `0bb4:0342` with `28de:2300` |
| Display macOS hides as a headset's | `22d2:aa01` | `22d2:aa02` or `aa03` (which is a guess) | `22d2:aa04` |
| Base stations | 1.0 | 1.0 or 2.0 | 1.0 or 2.0 |
| Controllers | Vive wands (or Index controllers) | Vive wands or Index controllers | Vive wands or Index controllers |

## What you need

- An Apple silicon Mac with the video output in the table, the headset's link box (Vive, Vive Pro) or its adapter
  (Vive Pro 2), its USB in a USB 3 port, and its power.
- Base stations, and controllers already paired with the headset (pairing is done once in SteamVR on Windows or
  Linux; the pairing lives in the headset's receivers).
- MetalSharp, its Windows Steam, and SteamVR there for SteamVR games.

## Setting up and playing

As for the Index ([VALVE_INDEX.md](VALVE_INDEX.md#setting-up-and-playing)): `metalvr-lighthouse install`, `display`,
`setup`, then `steamvr` or `game APPID`. `metalvr-lighthouse status` names the Vive it finds on USB.

**The Vive Pro 2's modes.** `metalvr-lighthouse start --pro2-mode N` switches the headset to a mode first:

| N | Size | Rate |
|---|---|---|
| 0 | 2448×1224 | 90 Hz |
| 1 | 2448×1224 | 120 Hz |
| 2 | 3264×1632 | 90 Hz |
| 3 | 3680×1836 | 90 Hz |
| 4 | 4896×2448 | 90 Hz (needs DSC) |
| 5 | 4896×2448 | 120 Hz (needs DSC) |

Mode 2 is the largest an Apple silicon Mac is expected to drive without DSC.

`--passthrough` shows the room through the headset's front camera (or the Vive Pro's two) in MetalVR's home.

## What works, checked without the headset

| | Checked by |
|---|---|
| Finding each on USB and telling them apart (Vive, Vive Pro, Vive Pro 2, Index; a Rift is never taken for one) | `headset-protocols` |
| Their configuration from their firmware: lenses, IMU calibration | `headset-protocols` |
| Each one's lens correction, the Vive Pro 2's upside-down image, drawn offscreen | `panel-vive`, `panel-vive-pro`, `panel-vive-pro-2` |
| Head rotation from the IMU; the Vive Pro's and Pro 2's IPD and proximity | `headset-protocols` |
| The Vive Pro 2's mode commands | `headset-protocols` |
| Vive wands through their Watchman radio: buttons, trackpad, trigger, rotation, battery, haptics | `headset-protocols` |
| The Vive controller profile in OpenXR | `native-vive-wands` |
| Positions from the base stations, through libsurvive | `libsurvive` (libsurvive's own simulated room) |
| The display overrides the script writes, including the Vive Pro 2's EDID patches copied from Apple's | `metalvr display` with `METALVR_DISPLAY_DRY_RUN=1`, by hand |

Built, but not covered by a test: SteamVR's Vive wands from MetalVR's SteamVR driver.

## Unknown until tried on one

- Whether macOS shows each one's display and in which modes; the pixel-clock limit; DSC for the Vive Pro 2.
- Power-on, the IMU's axes, and USB access to their interfaces on macOS.
- Which of `aa02` and `aa03` is the Vive Pro's display.
- The Vive Pro 2's `chipreset` re-enumeration after a mode change.
- libsurvive's tracking with real base stations on a Mac, and Lighthouse 1.0 tracking quality.
- The lens correction on real lenses, and haptics.

## Not supported

- The original Vive's cameras through anything but MetalVR's home; eye tracking (the Vive Pro Eye's).
- Without base stations, rotation only.

## When it doesn't work

As for the Index ([VALVE_INDEX.md](VALVE_INDEX.md#when-it-doesnt-work)). A Vive's HDMI must go into the Mac's own
HDMI port: through a hub or adapter, macOS may read the display and never turn it on.

The research and sources: [HEADSETS.md](../HEADSETS.md#phase-g-htc-vive) and
[HEADSETS.md](../HEADSETS.md#phase-h-htc-vive-pro-and-vive-pro-2).
