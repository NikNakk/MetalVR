## MetalVR

[![CI](https://github.com/metalsharp/MetalVR/actions/workflows/ci.yml/badge.svg)](https://github.com/metalsharp/MetalVR/actions/workflows/ci.yml)

An open-source OpenXR runtime for Apple silicon Macs.

PC VR games run under Wine with Direct3D 11 translated to Metal by [Mullion](https://github.com/dttdrv/mullion).
MetalVR is the OpenXR runtime those games talk to, and the native server that composes their frames, encodes
them in hardware, and streams them to a headset running the stock [ALVR](https://github.com/alvr-org/ALVR)
client. SteamVR games reach it through [OpenComposite](https://gitlab.com/znixian/OpenOVR), and native macOS
OpenXR applications use the same server through a Metal runtime.

Current Status; 

Testing is happening with an Oculus Rift CV1. First we had to create a vr server and apply specific commands for MacOS to recognize and load the headset display. We tied this to a 'Start VR Server' button on a dedicated MetalVR launch route. We tied Steam Launches to launch with mullion also, since this program relies on Mullion to load it's driver for d3d11/12. After this, we were able to get SteamVR to recognize the headset and render the tutorial through the headsets display at 90hz, and the headsets resolution. After more tests we were able to launch Subnautica through steam with SteamVR opened and the VR Sever running. BeamNG also was able to use the headset but currently experiences a crash in its driver. 

Still to do; 

- Fix and confirm the Rift's hand controllers showing both in the headset and tracking on the system. 
- Fix and confirm base stations tracking the headset accurately (Currently three, one on left/right, one extra behind). This is the harder and longer piece of work. 
- Optimize the experience for other headsets, such as CV2, Oculus Rift S, Meta Quest 1/2/3, Valve Index, Steam Frame. 
- Reduce the graphical issues through the VR Composition. (Most noticeable in Steam's own VR environment). 

Contribution is very welcome because I do not own every headset. The experience will be optimized for an Oculus Rift Cv1 first, followed by conclusive updates from users and myself where applicable. 


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
| [Oculus Rift CV1](docs/RIFT.md) | a CV1 on the Mac's own HDMI and USB: setting the Mac up, running, what works |

## What works

- OpenXR 1.0 and 1.1 core: sessions, spaces, swapchains, the frame loop, projection and quad layers, actions,
  haptics.
- `XR_KHR_D3D11_enable` under Wine with Mullion, and `XR_KHR_metal_enable` on macOS.
- Hardware H.264 and HEVC encoding, at a headset's resolution and refresh rate.
- OpenVR applications through OpenComposite: rendering, legacy and action-manifest input, haptics.
- The ALVR driver, against ALVR's own client networking code on one machine.

Not yet: a real headset or game, Direct3D 12 / Vulkan / OpenGL applications, hand tracking, audio, 32-bit
games.

## Contributing

Game reports, bug reports and pull requests are welcome: see [CONTRIBUTING.md](CONTRIBUTING.md). Report
security issues privately, as [SECURITY.md](SECURITY.md) describes.

## License

MIT; see [LICENSE](LICENSE). MetalVR is written from scratch against public specifications and open-source
code. The components it builds against or beside keep their own licenses, and the patches under `patches/`
carry the license of what they patch: see [patches/README.md](patches/README.md).
