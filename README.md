## MetalVR

[![CI](https://github.com/metalsharp/MetalVR/actions/workflows/ci.yml/badge.svg)](https://github.com/metalsharp/MetalVR/actions/workflows/ci.yml)

An open-source OpenXR runtime for Apple silicon Macs.

PC VR games run under Wine with D3D12/11 translated to Metal by [Mullion](https://github.com/dttdrv/mullion).
MetalVR is the OpenXR runtime those games talk to, and the native server that composes their frames and shows
them on a headset: an Oculus Rift CV1 plugged into the Mac itself, or a standalone headset running the stock
[ALVR](https://github.com/alvr-org/ALVR) client, streamed with hardware encoding. SteamVR games reach it through
MetalVR's SteamVR driver or [OpenComposite](https://gitlab.com/znixian/OpenOVR), and native macOS OpenXR
applications use the same server through a Metal runtime.

Current Status; 

Testing is happening with an Oculus Rift CV1. First we had to create a vr server and apply specific commands for MacOS to recognize and load the headset display. We tied this to a 'Start VR Server' button on a dedicated MetalVR launch route. We tied Steam Launches to launch with mullion also, since this program relies on Mullion to load it's driver for d3d11/12. After this, we were able to get SteamVR to recognize the headset and render the tutorial through the headsets display at 90hz, and the headsets resolution. After more tests we were able to launch Subnautica through steam with SteamVR opened and the VR Sever running. BeamNG also was able to use the headset but currently experiences a crash in its driver. 

Since then, the Rift's Touch controllers show in the headset and track, all three sensors (one of them on USB 2) track the headset and the controllers in position and show in SteamVR as base stations, and the flashing coloured blocks in the VR composition are fixed. ABE VR and Raw Data play through SteamVR.

Still to do; 

- Test the support for other headsets (Oculus Rift S, Meta Quest 1/2/3/3S, Valve Index, HTC Vive, Vive Pro and Pro 2, Steam Frame) on the real hardware: it is built from their specifications and the open drivers for them, and has not met one yet ([docs/HEADSETS.md](docs/HEADSETS.md)).
- BeamNG's crash in its driver.

Contribution is very welcome because I do not own every headset. The experience will be optimized for an Oculus Rift Cv1 first, followed by conclusive updates from users and myself where applicable. 

## Oculus Rift CV1

A Rift CV1 on the Mac's own HDMI and USB, with its sensors and Touch controllers, needs no Oculus software. The
release's `metalvr-rift` script does each step:

```sh
metalvr-rift install          # from the unpacked release: into ~/.metalsharp/metalvr-runtime
metalvr-rift display          # once per Mac: let macOS show the headset's display (asks for your password)
metalvr-rift setup            # once: register MetalVR's driver with SteamVR in MetalSharp's Steam
metalvr-rift steamvr          # start the Rift server, then SteamVR
metalvr-rift game APPID       # start the Rift server, then a Steam game in VR
```

**[docs/RIFT_CV1.md](docs/RIFT_CV1.md)** walks through it: what you need, setting up, playing, and what to do when
the headset stays dark. SteamVR and its games need MetalSharp-Wine's Mullion at
[aaf2tbz/mullion@998c32a6](https://github.com/aaf2tbz/mullion/commit/998c32a61095985e7034c7b56553d79aa81c7193) or
later, which shares Direct3D 11 buffers between processes for VR; `metalvr-rift setup` switches that on
(`MULLION_METALVR=1`).

## How it works

```
game.exe ─ OpenXR ─▶ metalvr_openxr.dll ─┐                         ┌─▶ simulated headset (tests)
            (Direct3D 11 via Mullion)    ├─ loopback ─▶ metalvrd ──┤
mac app ── OpenXR ─▶ libmetalvr_openxr ──┘   compose · encode      └─▶ ALVR ─▶ headset
            (Metal)
```

- One runtime source tree builds as a Windows DLL and a macOS dylib.
- Swapchain images never leave the GPU: under Wine they are Mullion shared textures, natively they are
  IOSurfaces.
- `metalvrd` composes both eyes into one frame and encodes H.264 or HEVC with VideoToolbox.

## Quick start

```sh
scripts/fetch-deps.sh
cmake -S . -B build/native -G Ninja && cmake --build build/native
cmake -S . -B build/win64 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-x86_64.cmake && cmake --build build/win64
scripts/test-all.sh
```

Then run the server and make MetalVR a Wine prefix's OpenXR runtime:

```sh
build/native/daemon/metalvrd --trust 1234.client      # the name the headset's ALVR app shows
scripts/install-prefix.sh /path/to/wine /path/to/prefix
```

[docs/BUILDING.md](docs/BUILDING.md) has the requirements and the rest.

## Documentation

| | |
|---|---|
| [Building, running and testing](docs/BUILDING.md) | requirements, builds, options, the test suites |
| [Architecture](docs/ARCHITECTURE.md) | the runtime, the server, the protocol between them, and what is not there yet |
| [MetalSharp-Wine integration](docs/METALSHARP_WINE.md) | what a Wine distribution ships and sets, and the contract with Mullion |
| [Headset streaming with ALVR](docs/ALVR.md) | building the driver, pairing a headset, how ALVR's API is used |
| [SteamVR games with OpenComposite](docs/OPENCOMPOSITE.md) | building `openvr_api.dll`, installing it for a game, known gaps |
| [Oculus Rift CV1 with MetalSharp](docs/RIFT_CV1.md) | the `metalvr-rift` script: from a release to a game in the headset |
| [Oculus Rift CV1 driver](docs/RIFT.md) | how the CV1 driver works: display, IMU, sensors, tracking, pacing |
| [SteamVR](docs/STEAMVR.md) | MetalVR's SteamVR driver: direct mode, requirements, its messages |
| [Headsets](docs/HEADSETS.md) | every headset MetalVR supports or plans to: specs, how each is recognized, what works |

## What works

- OpenXR 1.0 and 1.1 core: sessions, spaces, swapchains, the frame loop, projection and quad layers, actions,
  haptics.
- `XR_KHR_D3D11_enable` under Wine with Mullion, and `XR_KHR_metal_enable` on macOS.
- Hardware H.264 and HEVC encoding, at a headset's resolution and refresh rate.
- OpenVR applications through OpenComposite: rendering, legacy and action-manifest input, haptics.
- The ALVR driver, against ALVR's own client networking code on one machine.
- An Oculus Rift CV1 on the Mac's HDMI and USB: its display with lens correction, head rotation from its IMU,
  position tracking with up to four Rift Sensors (USB 3 or USB 2), Touch controllers, and SteamVR through
  MetalVR's own driver.
- Built, untested on hardware ([docs/HEADSETS.md](docs/HEADSETS.md)): a Rift S, Vive, Vive Pro, Vive Pro 2 and
  Index on the Mac's own display (rotation only), and the Quests and Steam Frame over ALVR, each recognized as
  itself with its own controllers in OpenXR and SteamVR.

Not yet: a standalone headset over ALVR in practice, Direct3D 12 / Vulkan / OpenGL applications, hand tracking,
audio, 32-bit games.

## Contributing

Game reports, bug reports and pull requests are welcome: see [CONTRIBUTING.md](CONTRIBUTING.md). Report
security issues privately, as [SECURITY.md](SECURITY.md) describes.

## License

MIT; see [LICENSE](LICENSE). MetalVR is written from scratch against public specifications and open-source
code. The components it builds against or beside keep their own licenses, and the patches under `patches/`
carry the license of what they patch: see [patches/README.md](patches/README.md).
