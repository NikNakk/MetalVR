# Building, running and testing

## Requirements

- An Apple silicon Mac. MetalVR is developed and tested on macOS 27.
- Xcode command line tools, CMake 3.24 or later, Ninja.
- MinGW-w64 for the Windows runtime: `brew install mingw-w64`.
- A Rust toolchain, only for the ALVR driver.
- A Wine with [Mullion](https://github.com/dttdrv/mullion), only to run Windows applications and the Wine
  tests. [MetalSharp-Wine](https://github.com/metalsharp/MetalSharp-Wine) is what the tests default to.

## Build

```sh
scripts/fetch-deps.sh                      # Khronos OpenXR headers and loader, into external/

cmake -S . -B build/native -G Ninja        # metalvrd, libmetalvr_openxr.dylib, test tools
cmake --build build/native

cmake -S . -B build/win64 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-x86_64.cmake
cmake --build build/win64                  # metalvr_openxr.dll, xr_smoke.exe
```

Two optional pieces are built by their own scripts, which fetch what they need into `external/`:

| | |
|---|---|
| `scripts/build-alvr.sh` | ALVR's `server_core`, for streaming to a headset. Configure `build/native` again afterwards and `metalvrd` links it. See [ALVR.md](ALVR.md). |
| `scripts/build-opencomposite.sh` | OpenComposite's `openvr_api.dll`, for SteamVR games. See [OPENCOMPOSITE.md](OPENCOMPOSITE.md). |

Without ALVR, `metalvrd` has only the simulated headset.

## Run

```sh
build/native/daemon/metalvrd --trust 1234.client
```

The name is the one the headset's ALVR client shows. Only headsets named with `--trust` may connect; one that is
not is logged with its name and refused. `metalvrd --driver sim` runs with the simulated headset instead, and
`metalvrd --help` lists every option.

The server must be running before an application creates its OpenXR instance.

**Windows applications under Wine.** Make MetalVR the prefix's OpenXR runtime, then start the game on Mullion:

```sh
scripts/install-prefix.sh /path/to/wine /path/to/prefix
```

[METALSHARP_WINE.md](METALSHARP_WINE.md) covers the details, and [OPENCOMPOSITE.md](OPENCOMPOSITE.md) covers
SteamVR games.

**Native macOS applications.** Point the OpenXR loader at the runtime's manifest:

```sh
XR_RUNTIME_JSON=$PWD/build/native/runtime/metalvr_openxr.json ./your-app
```

### Environment

Read by the runtime, from the application's environment:

| Variable | Effect |
|---|---|
| `METALVR_PORT` | the loopback port, for the runtime and the server (default 37913) |
| `METALVR_LOG=1` | the runtime logs to the application's stderr |
| `METALVR_HEADSET_WAIT=<seconds>` | wait that long for a headset to connect before reporting that there is none |
| `METALVR_INTERACTION_PROFILE=<path>` | the interaction profile to prefer when an application suggests bindings for several |

## Test

```sh
scripts/test-all.sh         # everything below, one line per case
scripts/test-native.sh      # Metal application → metalvrd
scripts/test-wine.sh        # Direct3D 11 application under Wine + Mullion → metalvrd
scripts/test-openvr.sh      # OpenVR application → OpenComposite → MetalVR, under Wine + Mullion
scripts/test-failures.sh    # no server, server killed mid-session, second application
```

All of them use the simulated headset. CI runs the native ones on GitHub's macOS runners, which have no
hardware video encoder, with `METALVR_DAEMON_ARGS="--software-encoder --eye 640x704"`; that option is for such
machines only and is far too slow for a headset. A case passes only if

- the test application's own checks pass (`tests/xr_smoke.cpp`, `tests/openvr/ovr_smoke.cpp`),
- the encoded stream holds every frame the application rendered and decodes (`tests/stream_check.mm`), and
- the composed frame is the picture the application drew (`tests/frame_check.mm`).

Each run leaves the stream, one composed frame as a PNG, and both logs in `build/out/`.

`xr_smoke` takes `--array`, `--samples N`, `--unorm`, `--format N`, `--flip`, `--no-quad`, and `--loader PATH`
to go through the Khronos loader instead of loading the runtime directly. `METALVR_DAEMON_ARGS` passes options to
the server, for example `"--eye 2144x2336 --codec hevc"` for a headset-sized stream. The Wine tests take
`METALVR_WINE_ROOT` (a Wine tree with Mullion) and `METALVR_WINEPREFIX`.

## Layout

```
protocol/   the wire protocol and pose math shared by both halves
runtime/    the OpenXR runtime: Windows DLL and macOS dylib from one tree
daemon/     metalvrd: server, compositor, encoder, headset drivers
tests/      test applications and the stream and frame checkers
scripts/    dependency fetch, ALVR and OpenComposite builds, prefix install, tests
patches/    the changes MetalVR's builds make to ALVR and OpenComposite
docs/       this documentation
```
