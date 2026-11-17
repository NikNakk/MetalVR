# Valve Steam Frame

**Tried on hardware:** no. MetalVR streams to the Frame with ALVR, like a Quest, through ALVR's client patched by
MetalVR to run there. The patched client is built and checked, but has not run on a Frame.

Valve's own streaming (Steam Link) needs Windows SteamVR and a PC's GPU encoder, so it is not an option on a Mac.
The stock ALVR client shows a blank screen on the Frame: it requires an OpenXR extension
(`XR_KHR_convert_timespec_time`) that Valve's runtime does not have. MetalVR's patch
(`patches/alvr/0003-client-runs-on-the-steam-frame.patch`) makes the client do without it, and name the headset
"Steam Frame" so that MetalVR recognizes it.

## What you need

- An Apple silicon Mac and the Frame on the same network (Wi-Fi 6E or 7 on the Frame's side; the Mac preferably
  wired).
- The Frame in developer mode, for installing the client with `adb`, and Android's platform-tools on the Mac
  (`brew install android-platform-tools`).
- The patched client's APK. Build it from this repository:

  ```sh
  scripts/build-alvr-client.sh      # fetches the Android tools into build/android; shows Google's SDK license
  ```

  which writes `build/alvr-client/alvr_client_android.apk`.
- MetalSharp, its Windows Steam, and SteamVR there for SteamVR games.

## Setting up

1. Install the client on the Frame:

   ```sh
   adb install -r build/alvr-client/alvr_client_android.apk
   ```

2. Install MetalVR and set SteamVR up as for a Quest ([QUEST.md](QUEST.md#setting-up)).
3. Start the client on the Frame, note its name (such as `1234.client`), and start the server trusting it:

   ```sh
   ~/.metalsharp/metalvr-runtime/bin/metalvr start --trust 1234.client
   ```

The patched client names the headset "Steam Frame" by itself. With the stock client (which reports "Android
(unknown)"), add `--headset steam-frame`.

## What works, checked without the headset

| | Checked by |
|---|---|
| The patched client: it builds as an APK (verified: signed, arm64, the "Steam Frame" name compiled in), and its Android code compiles | `scripts/build-alvr-client.sh`, by hand |
| MetalVR naming the headset from the client's "Steam Frame" | `alvr-models`, `headset-protocols` |
| The Frame's entry in the model table (2160×2160 per eye, 72 to 120 Hz) | `headset-protocols` |
| ALVR's foveated encoding (`--foveated-encoding`), fixed at the centre | `foveation`, `native-foveated` |

Built, but not covered by a test: its controllers' extra inputs (the bumper, D-pad, View button, and X and Y on the
right) through OpenXR (Valve's `frame_controller_valve` profile, with Touch as the fallback) and SteamVR. The
simulated headset presses them when it stands in for a Frame (`metalvrd --driver sim --headset steam-frame`).

## Unknown until tried on one

- Whether the patched client runs and streams on the Frame. The Frame's Android container has only software video
  decoders, so its decoding speed decides the usable resolution and rate.
- Latency, the real field of view, and the headset's USB network (whether a cable could carry the stream on a Mac).
- SteamVR's own names for the Frame's controllers (they are shown as Touch controllers until then).

## Not supported

- Foveation following the eyes' gaze (it needs the client to send the gaze).
- Everything a Quest lacks too ([QUEST.md](QUEST.md#not-supported)).

The research and sources: [HEADSETS.md](../HEADSETS.md#phase-e-valve-steam-frame); the patch:
[ALVR.md](../ALVR.md#the-patches).
