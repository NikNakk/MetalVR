# Oculus Rift CV1

`metalvrd --driver rift` drives an Oculus Rift CV1 plugged into the Mac itself: its HID interface turns the
headset on and gives its rotation, and frames are drawn on its display with its lens distortion. Oculus's own
software is not involved, and none is needed.

Head rotation only, so far: the head stays where a seated user's would be (`--ipd` sets the eye distance, otherwise
the headset's slider is read). Positional tracking with the Rift Sensors and the Touch controllers are not there yet.

## Setting the Mac up

1. **The headset's HDMI goes into one of the Mac's own HDMI ports.** Through a USB-C hub or adapter, macOS may read
   the headset's EDID and still never put its display online. Its USB plug goes into any USB 3 port.
2. **macOS hides the headset's display.** It ships an override for Oculus's displays,
   `/System/Library/Displays/Contents/Resources/Overrides/DisplayVendorID-3ed2/DisplayProductID-4`, that marks the
   CV1 a head-mounted display (`is-hmd`), which keeps it off the desktop. An override of your own in `/Library`
   lifts that, once:

   ```sh
   sudo mkdir -p /Library/Displays/Contents/Resources/Overrides/DisplayVendorID-3ed2
   sudo /usr/libexec/PlistBuddy -c "Add :DisplayVendorID integer 16082" -c "Add :DisplayProductID integer 4" \
     -c "Add :is-hmd bool false" /Library/Displays/Contents/Resources/Overrides/DisplayVendorID-3ed2/DisplayProductID-4
   ```

   Deleting that file undoes it.
3. The headset's display is dark until software turns it on, so it appears only once `metalvrd` runs. The first
   time, macOS may place it among the other displays and make it the main one: in System Settings, Displays, put the
   menu bar back on a monitor. macOS remembers that arrangement.

## Running

```sh
build/native/daemon/metalvrd --driver rift --verbose
```

It waits for the headset on USB, turns it on, and draws on its display as soon as macOS shows it. While no
application draws, the headset shows a test scene: a sphere fixed in the world, with a line every 15 degrees, a cyan
horizon and a red mark straight ahead, for checking head tracking and the lens correction without an application.
Straight ahead is the way the headset faces when `metalvrd` starts, and again each time the headset is put on (its
proximity sensor). Applications then use MetalVR as their OpenXR runtime as with any other driver (BUILDING.md,
METALSHARP_WINE.md, OPENCOMPOSITE.md).
`--eye WxH` changes the per-eye render size applications are asked for (default 1344x1600).

## How it works

- **HID** (2833:0031, USB interface 0): feature report 9 gives the panel's size and lens geometry; report 2 has the
  headset calibrate its IMU itself; report 17 sets a 10 s keep-alive, renewed every 9.5 s; report 29 turns the
  display, audio and LEDs on (and off when `metalvrd` stops). IN report 11 carries the IMU at 1000 Hz, two samples
  per report.
- **Rotation**: the gyroscope integrated, with the accelerometer pulling the tilt toward gravity, predicted to each
  frame's display time. Without the sensors the heading drifts slowly; putting the headset on again re-centers it.
- **Position** (with the Rift's sensors, below): the sensors' cameras watch the infrared LEDs of the headset and the
  Touch controllers, and what they see is fused with each device's IMU.
- **Display**: a borderless window over the headset's screen, with a `CAMetalLayer` synchronised to its 90 Hz. The
  window moves with the headset's screen whenever macOS rearranges the displays (making another display the main
  one moves every display's frame), so that it never covers another display. Each composed frame is drawn per eye
  through the CV1's Catmull-Rom lens curve with colour fringing correction, at the default eye relief.

## Position tracking with the Rift's sensors

With one or more Rift sensors (2833:0211) on **USB 3** ports, the headset and both Touch controllers are tracked in
position as well as rotation. Nothing needs setting up: `metalvrd` opens every sensor it finds when the headset comes
on, and says so in its log (`rift: 2 sensors watching for the headset's 34 LEDs`, then `rift: the sensors see the
headset; its position is tracked`). A sensor works out where it stands the first time it sees the headset, so put the
headset where the sensors can see it once.

- The origin is where the headset was when a sensor first saw it, and again at each re-center (putting the headset
  on, or the remote's Oculus button). Height is `--head-height` plus real movement, not a measured floor.
- A device the sensors lose sight of stays where it was last seen and goes on turning from its IMU.
- A Touch controller is tracked once it has been picked up: a controller lying asleep does not answer the read of its
  calibration, which holds its LED positions.
- A sensor on a USB 2 port sends compressed video, which is not decoded: it is skipped, with a line in the log.
- `kill -USR1` of `metalvrd` while the home shows writes what each sensor's camera sees to
  `~/.metalsharp/logs/metalvr-sensor-<n>.png`, with the number of bright spots (LEDs) it found, for checking that a
  sensor has the headset in view.
- Without sensors, or in a build without the tracker, the driver tracks rotation only and the head stays at its
  seated place.

The tracker is [OpenHMD](https://github.com/thaytan/OpenHMD)'s (`rift-kalman-filter` branch, Boost Software License
1.0): `scripts/fetch-deps.sh` fetches it into `external/OpenHMD-rift`, and its sources are compiled unchanged into
`metalvrd`. `daemon/rift_tracking/` gives them what they expect of Linux and of other libraries: libusb's calls on
IOUSBHost (the sensors are read directly; no macOS driver claims them), the camera geometry OpenHMD takes from OpenCV
(lens distortion and pose fitting), and a narrow interface for the Rift driver.

The report formats, the IMU sample packing, the lens curve and the field of view follow Monado's Rift driver
(`src/xrt/drivers/rift`, `src/xrt/auxiliary/util/u_distortion_mesh_rift.c`; BSL-1.0).
