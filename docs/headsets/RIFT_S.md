# Oculus Rift S

**Tried on hardware:** no. Built from the open Rift S drivers (OpenHMD's and Monado's) and checked without the
headset. If you have one, a report of how far it gets is the most useful thing you can send.

The Rift S uses the same driver and the same `metalvr-rift` script as the [Rift CV1](RIFT_CV1.md): whichever is on
USB decides. It has no external sensors: its position comes from its own five cameras.

## What you need

- An Apple silicon Mac with a **DisplayPort output**: a USB-C or Thunderbolt port with a USB-C to DisplayPort cable
  or adapter (the Rift S has no HDMI). Its USB in a USB 3 port.
- A MetalVR release (it includes Basalt, the camera tracker; building from source, `scripts/build-basalt.sh`).
- MetalSharp, its Windows Steam, and SteamVR there for SteamVR games, as for the CV1.

## Setting up and playing

The steps are the CV1's ([RIFT_CV1.md](RIFT_CV1.md)): `metalvr-rift install`, `metalvr-rift display` (the Rift S's
display is `3ed2:0012`, which macOS hides as a headset's, as it does the CV1's), `metalvr-rift setup`, then
`metalvr-rift steamvr` or `metalvr-rift game APPID`. `metalvr-rift status` says "a Rift S is on USB" once it sees it.

**Allow the camera.** The first time the server starts with a Rift S, macOS asks whether the app that started it
(Terminal, or MetalSharp) may use the camera: the Rift S's cameras reach macOS as a camera named "Rift S Sensor".
Without that, the headset tracks rotation only.

## What works, checked without the headset

| | Checked by |
|---|---|
| Finding it on USB (`2833:0051`), its three HID interfaces, power-on and keep-alive, the screen following the proximity sensor | `headset_protocol_check` |
| Its IMU with its own calibration (firmware block 0x0e) | `headset_protocol_check` |
| The portrait 1440×2560 panel split into two eyes, each turned, with Monado's lens values | `panel-rift-s` |
| Touch controllers: buttons, stick, trigger, grip, touches and rotation, through the headset's radio | `headset_protocol_check` |
| Position from its cameras: their calibration (firmware block 0x0f), their lenses fitted for Basalt, the camera frames' layout, auto exposure, and Basalt's visual-inertial tracking | `headset_protocol_check`, `rift-s-tracking` (a synthetic Rift S tracked to 0.2 cm over a 5 m path) |

## Unknown until tried on one

- Whether macOS brings its DisplayPort link up at 80 Hz, unrotated, through a USB-C adapter.
- The order of power-on and proximity, and its USB interfaces as macOS presents them.
- Whether macOS's camera driver hands over the "Rift S Sensor" frames unconverted, and how Basalt tracks on real
  pictures.
- The lens correction: no correct Rift S lens model is published; Monado's values are a starting point to tune by eye.
- The IMU's axis signs.

## Not supported

- The controllers' position (no open driver tracks it): they are rotation only, held where a hand would be.
- The controllers' haptics (the command is not known).

## When it doesn't work

- **No position, only rotation.** The log says why: `the Rift S tracks rotation only: ...` (no Basalt, or no camera
  calibration), or no `reading the Rift S's cameras` line (macOS has no "Rift S Sensor" camera, or camera access was
  refused: System Settings, Privacy & Security, Camera).
- Everything else is as for the CV1 ([RIFT_CV1.md](RIFT_CV1.md#when-it-doesnt-work)).

The research and sources: [HEADSETS.md](../HEADSETS.md#phase-i-oculus-rift-s).
