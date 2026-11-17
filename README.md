## MetalVR

[![CI](https://github.com/metalsharp/MetalVR/actions/workflows/ci.yml/badge.svg)](https://github.com/metalsharp/MetalVR/actions/workflows/ci.yml)

An open-source OpenXR runtime for Apple silicon Macs. PC VR games run under Wine with Direct3D translated to Metal by
[Mullion](https://github.com/dttdrv/mullion); MetalVR is the runtime they talk to and the server that shows their
frames on a headset. SteamVR games reach it through MetalVR's SteamVR driver, and native macOS OpenXR applications
through a Metal runtime.

**Status:** an Oculus Rift CV1 on the Mac's own HDMI and USB plays SteamVR games (Subnautica, ABE VR, Raw Data) with
its sensors and Touch controllers. Every other headset below is built and checked without the headset, and has not
met one yet: reports from owners are very welcome.

## Headsets

| Headset | Tried on hardware | Guide |
|---|---|---|
| Oculus Rift CV1 | yes | [docs/headsets/RIFT_CV1.md](docs/headsets/RIFT_CV1.md) |
| Oculus Rift S | no | [docs/headsets/RIFT_S.md](docs/headsets/RIFT_S.md) |
| Meta Quest 1, 2, 3, 3S, Pro (ALVR) | no | [docs/headsets/QUEST.md](docs/headsets/QUEST.md) |
| Valve Steam Frame (ALVR) | no | [docs/headsets/STEAM_FRAME.md](docs/headsets/STEAM_FRAME.md) |
| Valve Index | no | [docs/headsets/VALVE_INDEX.md](docs/headsets/VALVE_INDEX.md) |
| HTC Vive, Vive Pro, Vive Pro 2 | no | [docs/headsets/HTC_VIVE.md](docs/headsets/HTC_VIVE.md) |
| Sony PlayStation VR2 | no | [docs/headsets/PSVR2.md](docs/headsets/PSVR2.md) |

Each guide covers what you need, setting up and playing, what is checked in code, and what is still unknown.

## Building

```sh
scripts/fetch-deps.sh
cmake -S . -B build/native -G Ninja && cmake --build build/native
cmake -S . -B build/win64 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-x86_64.cmake && cmake --build build/win64
scripts/test-all.sh
```

Requirements, the optional parts (ALVR, libsurvive, Basalt, OpenComposite) and the test suites:
[docs/BUILDING.md](docs/BUILDING.md).

## Documentation

| | |
|---|---|
| [Headset guides](docs/headsets/README.md) | one per headset |
| [Building, running and testing](docs/BUILDING.md) | requirements, builds, options, the test suites |
| [Architecture](docs/ARCHITECTURE.md) | the runtime, the server, the protocol between them, what works and what is not there yet |
| [SteamVR](docs/STEAMVR.md) | MetalVR's SteamVR driver: direct mode, requirements, its messages |
| [The direct-display driver](docs/RIFT.md) | how headsets on the Mac's own display are driven: display, IMU, sensors, pacing |
| [ALVR](docs/ALVR.md) | streaming to standalone headsets: the driver, pairing, MetalVR's patches |
| [OpenComposite](docs/OPENCOMPOSITE.md) | SteamVR games without SteamVR |
| [MetalSharp-Wine integration](docs/METALSHARP_WINE.md) | what a Wine distribution ships and sets, and the contract with Mullion |
| [Headset research](docs/HEADSETS.md) | each headset's specifications, how it is recognized, and the sources |

## Contributing

Game reports, headset reports, bug reports and pull requests are welcome: see [CONTRIBUTING.md](CONTRIBUTING.md).
Report security issues privately, as [SECURITY.md](SECURITY.md) describes. Known problems: BeamNG.drive crashes in its
graphics driver.

## License

MIT; see [LICENSE](LICENSE). MetalVR is written from scratch against public specifications and open-source code. The
components it builds against or beside keep their own licenses, and the patches under `patches/` carry the license of
what they patch: see [patches/README.md](patches/README.md).
