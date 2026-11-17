# Meta Quest 1, 2, 3, 3S and Pro

**Tried on hardware:** no. The streaming path has run end to end against ALVR's own client code on one Mac, but no
Quest has shown a picture from MetalVR yet. If you have one, a report is very welcome.

A Quest runs the stock [ALVR](https://github.com/alvr-org/ALVR) client (version 20.x) and MetalVR streams to it:
`metalvrd`'s `alvr` driver links ALVR's own server, and encodes with the Mac's hardware encoder. Nothing Meta makes
runs on the Mac.

## What you need

- An Apple silicon Mac and the Quest on the same network (5 GHz or 6 GHz Wi-Fi, the Mac preferably wired), or the
  Quest on USB.
- ALVR's client **v20.x** on the Quest (from the Meta store, or sideloaded from ALVR's releases).
- MetalSharp, its Windows Steam, and SteamVR there for SteamVR games.

## Setting up

1. Install MetalVR (`metalvr-rift install` from an unpacked release, which also makes MetalVR the Steam prefix's
   OpenXR runtime; [RIFT_CV1.md](RIFT_CV1.md#setting-up-once)), and for SteamVR, `metalvr-rift setup`.
2. Start the ALVR client on the Quest. It shows its name, such as `1234.client`.
3. Start the server, trusting that headset (once; it is remembered):

   ```sh
   ~/.metalsharp/metalvr-runtime/bin/metalvr start --trust 1234.client
   ```

   A headset that is not trusted is refused and named in the server's output.

**On USB instead of Wi-Fi:** turn on the Quest's developer mode and USB debugging, install Android's platform-tools
(`brew install android-platform-tools`), and trust the wired name: `metalvr start --trust client.wired` (or
`--adb PATH` for another `adb`).

Then start games from MetalSharp as usual, or SteamVR with `metalvr launch steamvr`.

## Which Quest it is

The client says which Quest it runs on, and MetalVR names the headset by it: its name and refresh rates in OpenXR and
SteamVR, its controllers (Touch on a Quest 1 or 2, Touch Plus on a 3, 3S or Pro), and a warning when the stream is
more than its video decoder takes (ALVR's default is too much for a Quest 1). `--headset quest-2` (or `quest-1`,
`quest-3`, `quest-3s`, `quest-pro`) names it instead, for a client that does not say.

## What works, checked without the headset

| | Checked by |
|---|---|
| The model from what the client reports, never mistaking a 3S for a 3 | `alvr-models` |
| The driver starting, trusting only the named headsets, and stopping; wired mode refusing to start without an `adb` | `alvr-driver`, `alvr-models` |
| The stream (hardware HEVC, the handshake, poses, inputs, haptics), against ALVR's own client code on the same Mac | by hand, not in the suite: [ALVR.md](../ALVR.md#what-has-been-verified-and-what-has-not) |
| Hand tracking: the joints the client sends, as `XR_EXT_hand_tracking` and SteamVR's hand skeletons | `native` (the simulated headset's hands) |
| ALVR's foveated encoding (`--foveated-encoding`): each eye squeezed toward its edges as ALVR's client expects | `foveation`, `native-foveated` |
| Each Quest's controllers as OpenXR profiles and SteamVR models | `headset-protocols`, the simulated headset |

Built, but not covered by a test (ALVR sends them only from a headset): the headset's and controllers' battery
levels, and the play area's bounds as OpenXR's stage bounds.

## Unknown until tried on one

- That the Quest's decoder accepts the stream and shows it; latency, reprojection and judder.
- The resolution and refresh rates the client really reports, and HEVC at 120 Hz.
- The controllers' grip and aim poses (the aim offset is an estimate) and their buttons from real hardware.
- Wi-Fi streaming and discovery (the checks used the same machine), and wired mode through adb from a Mac.
- Whether ALVR's current client still runs on a Quest 1.

## Not supported

- Audio in either direction; eye and face tracking; changing the refresh rate (the stream stays at the negotiated
  one); AV1; trusting a headset while the server runs (start it again with `--trust`).
- Touch Plus's force, curl and slide, which ALVR does not send.

How the ALVR driver works: [ALVR.md](../ALVR.md). The research and sources:
[HEADSETS.md](../HEADSETS.md#phase-a-meta-quest-1).
