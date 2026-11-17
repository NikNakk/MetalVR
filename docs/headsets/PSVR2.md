# Sony PlayStation VR2

**Tried on hardware:** no. Built from Monado's PlayStation VR2 and Sense controller drivers and checked without the
headset. If you have one, a report of how far it gets is the most useful thing you can send.

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

## What works, checked without the headset

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

## Unknown until tried on one

- Whether macOS lets MetalVR open the headset's USB interfaces and take its vendor requests.
- Whether the adapter's display comes up on a Mac at 4000×2040 and 90 or 120 Hz (it may need DisplayPort's
  compression), and its display's product code (until it is known, MetalVR takes any Sony display and names it in
  the log).
- The Sense controllers' pairing with a Mac, and their IMU's axes.
- Whether the headset's tracking starts as it does on Linux, with the cameras run as Monado runs them.

## Not supported

- The controllers' position: the headset's cameras see their LEDs, but no open driver solves that yet. They are
  rotation only, held where a hand would be.
- Eye tracking, passthrough, the headset's rumble, and the adaptive triggers.

## When it doesn't work

- **No headset found.** `metalvr-psvr2 status` says whether a PlayStation VR2 (`054c:0cde`) is on USB; the log says
  which interface or request failed.
- **The headset is found, its display stays dark.** The log says `psvr2: the headset's DisplayPort input has a
  signal` once the adapter's video reaches it; check the DisplayPort cable and the adapter's power.
- **No controllers.** The log names each Sense controller when it connects; pair them in System Settings first.

The research and sources: [HEADSETS.md](../HEADSETS.md#phase-j-sony-playstation-vr2).
