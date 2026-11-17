# Contributing to MetalVR

Thanks for helping. MetalVR is an OpenXR runtime for Apple silicon Macs and the server that streams its frames
to a headset; the [README](README.md) says what it does and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) how it
is built and what is missing.

## Before you start

- **A game that does not work, or does**: open a *Game report*. It asks for the game, the Wine and Mullion
  versions, the headset and the logs. If the game is broken without VR too, it is a Mullion or Wine matter.
- **A headset MetalVR has not met yet**: follow its guide in [docs/headsets](docs/headsets/README.md) and open a
  *Bug report* saying how far it got, with the server's log (`metalvrd --verbose`). Each guide lists what is still
  unknown about that headset, which is what such a report answers.
- **Bugs**: open a *Bug report* with the commit you built, the Mac and macOS version, and the output of
  `metalvrd --verbose` and of the application with `METALVR_LOG=1`.
- **Security issues**: never in a public issue. See [SECURITY.md](SECURITY.md).
- **Larger changes** (a new graphics binding, a protocol change, a new headset driver, a new bundled
  component): open an issue first, so the approach is agreed before the work.
- **Elsewhere:** some parts live in other repositories. Contribute those there:
  - Direct3D to Metal: [dttdrv/mullion](https://github.com/dttdrv/mullion).
  - Wine for macOS: [metalsharp/MetalSharp-Wine](https://github.com/metalsharp/MetalSharp-Wine).
  - Streaming and the headset client: [alvr-org/ALVR](https://github.com/alvr-org/ALVR).
  - OpenVR on OpenXR: [OpenComposite](https://gitlab.com/znixian/OpenOVR).

## Where changes go

| Change | Where |
|---|---|
| OpenXR behaviour: sessions, spaces, the frame loop, actions | `runtime/` (shared by the Windows DLL and the macOS dylib) |
| How images reach the server | `runtime/backend_d3d11.cpp`, `runtime/backend_metal.mm`, `daemon/shared_texture.mm` |
| What the runtime and the server say to each other | `protocol/metalvr_protocol.h`, and both sides in the same change |
| Composition, encoding, pacing | `daemon/` |
| A headset or transport | a `Driver` in `daemon/` (`driver.h`) |
| ALVR or OpenComposite themselves | a patch under `patches/`, applied by the build script; never an edit in `external/` |

## Building and testing

[docs/BUILDING.md](docs/BUILDING.md) has the steps:

```sh
scripts/fetch-deps.sh
cmake -S . -B build/native -G Ninja && cmake --build build/native
cmake -S . -B build/win64 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-x86_64.cmake && cmake --build build/win64
```

Before opening a pull request:

- Build both targets.
- Run the tests for what you changed. `scripts/test-all.sh` runs everything a Mac with a Wine and Mullion can;
  `scripts/test-native.sh` and `scripts/test-failures.sh` need neither.
- A change to the ALVR driver, encoding, timing or input is only proven on a headset. Say which headset and
  client you used, or say that you had none.
- Say what you ran and what you could not. CI builds both targets, ALVR's driver and OpenComposite, and runs the
  native tests with a software encoder; it has no Wine, no Mullion and no headset, so it cannot stand in for the
  rest.

A new behaviour comes with a check that fails without it: a case in `tests/xr_smoke.cpp` or
`tests/openvr/ovr_smoke.cpp`, or in the stream and frame checkers.

## Rules

- **Clean-room.** MetalVR is written against public specifications and open-source code. Do not contribute
  anything taken from a closed-source VR runtime or streaming product: no decompiled or disassembled code, no
  copied protocol tables, no assets. Describing observable behaviour of a game or a headset is fine.
- **One runtime.** Windows and macOS build from the same `runtime/` sources. A platform difference belongs in a
  graphics backend or `ipc.cpp`, not in a second copy of the logic.
- **Fail honestly.** A missing server, headset or encoder is reported as what it is. No fallback that turns a
  failure into a stream that looks fine and is not; a degraded mode is something the user asks for by name.
- **Loopback and trust.** The server listens on the loopback interface only, and a headset connects only when
  it was named with `--trust`. Changes there need an issue first.
- **Pinned downloads.** Every dependency the scripts fetch is pinned to a commit and checked against it.
- **No personal data.** No Wine prefixes, registries, or logs with personal paths in commits or issues.
- **Style.** Match the code around the change: C++20, two-space indent, 120 columns, comments that say why.
- **Licenses.** MetalVR's own code is MIT ([LICENSE](LICENSE)), and a contribution is made under it. A patch
  under `patches/` carries the license of what it patches ([patches/README.md](patches/README.md)): MIT for
  ALVR, GPL-3.0 for OpenComposite, which is why OpenComposite is built as its own DLL and never linked in.
