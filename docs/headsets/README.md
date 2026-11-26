# Headsets

One guide per headset: what you need, how to set it up and play, what MetalVR's tests check without the headset,
and what is still unknown until someone tries one. Only the Rift CV1 has been run on real hardware so far. The others
are built from their specifications and the open drivers that support them, and reports from owners are very
welcome ([CONTRIBUTING.md](../../CONTRIBUTING.md)).

| Headset | How it reaches the Mac | Tried on hardware | Guide |
|---|---|---|---|
| Oculus Rift CV1 | HDMI and USB on the Mac; Rift Sensors | yes | [RIFT_CV1.md](RIFT_CV1.md) |
| Oculus Rift S | DisplayPort and USB on the Mac; its own cameras | no | [RIFT_S.md](RIFT_S.md) |
| Meta Quest 1, 2, 3, 3S, Pro | streamed with ALVR, over Wi-Fi or USB | no | [QUEST.md](QUEST.md) |
| Valve Steam Frame | streamed with ALVR's client, patched by MetalVR | no | [STEAM_FRAME.md](STEAM_FRAME.md) |
| Valve Index | DisplayPort and USB on the Mac; Lighthouse base stations | no | [VALVE_INDEX.md](VALVE_INDEX.md) |
| HTC Vive, Vive Pro, Vive Pro 2 | HDMI or DisplayPort and USB on the Mac; Lighthouse base stations | no | [HTC_VIVE.md](HTC_VIVE.md) |
| Sony PlayStation VR2 | Sony's PC adapter: DisplayPort and USB on the Mac | yes | [PSVR2.md](PSVR2.md) |

Every headset needs the same things on the game's side: [MetalSharp](https://github.com/metalsharp/MetalSharp) with
its Windows Steam, and for SteamVR games, SteamVR in that Steam and MetalVR's SteamVR driver
([STEAMVR.md](../STEAMVR.md)). OpenXR games use MetalVR directly.

The research behind each headset (specifications, how each is recognized, sources) is in
[HEADSETS.md](../HEADSETS.md).
