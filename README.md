# MetalVR

An open-source OpenXR runtime for Apple silicon Macs.

PC VR games run under Wine with Direct3D 11 translated to Metal by [Mullion](https://github.com/dttdrv/mullion).
MetalVR is the OpenXR runtime those games talk to, and the native server that composes their frames, encodes
them in hardware, and streams them to a headset running the stock [ALVR](https://github.com/alvr-org/ALVR)
client. SteamVR games reach it through [OpenComposite](https://gitlab.com/znixian/OpenOVR), and native macOS
OpenXR applications use the same server through a Metal runtime.

> **Status: early.** Every path below passes its tests against a simulated headset. No physical headset and no
> shipping game has been tested yet.

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

## What works

- OpenXR 1.0 and 1.1 core: sessions, spaces, swapchains, the frame loop, projection and quad layers, actions,
  haptics.
- `XR_KHR_D3D11_enable` under Wine with Mullion, and `XR_KHR_metal_enable` on macOS.
- Hardware H.264 and HEVC encoding, at a headset's resolution and refresh rate.
- OpenVR applications through OpenComposite: rendering, legacy and action-manifest input, haptics.
- The ALVR driver, against ALVR's own client networking code on one machine.

Not yet: a real headset or game, Direct3D 12 / Vulkan / OpenGL applications, hand tracking, audio, 32-bit
games.

## License

MIT; see [LICENSE](LICENSE). MetalVR is written from scratch against public specifications and open-source
code. The components it builds against or beside keep their own licenses, and the patches under `patches/`
carry the license of what they patch: see [patches/README.md](patches/README.md).
