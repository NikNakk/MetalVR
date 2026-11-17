# Architecture

MetalVR has two halves and one protocol between them.

```
 Windows game (under Wine)                       native macOS application
   │ OpenXR, Direct3D 11                           │ OpenXR, Metal
   ▼                                               ▼
 openxr_loader.dll                               libopenxr_loader.dylib
   │ ActiveRuntime (registry)                      │ XR_RUNTIME_JSON / active_runtime.json
   ▼                                               ▼
 metalvr_openxr.dll  ─────────┐          ┌──────  libmetalvr_openxr.dylib
   Direct3D 11 backend        │          │         Metal backend
   images: Mullion shared     │ TCP on   │         images: global IOSurfaces
   textures (Metal ports)     │ loopback │
                              ▼          ▼
                            metalvrd  (native)
                              ├─ opens every swapchain image as a Metal texture
                              ├─ compositor: layers → one stereo frame (Metal)
                              ├─ encoder: VideoToolbox H.264 / HEVC, frame in / frame out
                              └─ driver: where tracking comes from and video goes
                                   ├─ sim   deterministic headset, optional stream dump
                                   ├─ alvr  ALVR's server_core → stock ALVR client on the headset
                                   └─ rift, lighthouse, psvr2
                                            a headset on the Mac's own display and USB (RIFT.md)
```

## One runtime, two targets

`runtime/` is a single OpenXR runtime compiled twice: as a Windows DLL with MinGW-w64, and as a macOS dylib.
Everything OpenXR — instance, session state machine, spaces, the frame loop, actions — is shared
(`instance.cpp`, `session.cpp`, `input.cpp`, `dispatch.cpp`). Only the graphics backend differs:

| | Windows (`backend_d3d11.cpp`) | macOS (`backend_metal.mm`) |
|---|---|---|
| Extension | `XR_KHR_D3D11_enable` | `XR_KHR_metal_enable` |
| Application-facing image | typeless D3D11 texture on the application's device: arrays and MSAA as asked | the IOSurface texture itself (single slice), or a private array texture |
| What the server reads | one shared 2D texture per image and slice | one global IOSurface per image and slice |
| How it gets there | copy or MSAA resolve at `xrReleaseSwapchainImage` | nothing (direct), or a blit per slice (arrays) |
| "The GPU is done" | `D3D11_QUERY_EVENT`, which Mullion signals from the GPU | an empty command buffer on the application's queue, waited for |

The runtime holds no tracking or display state of its own. It asks the server.

## The Direct3D 11 path and Mullion

A Direct3D 11 texture created with `D3D11_RESOURCE_MISC_SHARED | D3D11_RESOURCE_MISC_SHARED_NTHANDLE` is, in
Mullion, a Metal shared texture whose mach port is published under a bootstrap name. The runtime asks for the
texture's NT handle (`IDXGIResource1::CreateSharedHandle`), which in Mullion is a file mapping describing the
share, reads the bootstrap name out of it, and sends the name to the server. The server looks the name up, asks
the port behind it for the texture's own port (Mullion's shared-name protocol, `winemetal_unix.c`), and rebuilds
the texture with `-[MTLSharedTextureHandle initWithMachPort:]` and `newSharedTextureWithHandle:`.

So the game's GPU output reaches the server's compositor without leaving the GPU and without any change to
Mullion or Wine. The runtime DLL is an ordinary Windows DLL: no Wine unixlib, no Wine headers.

This depends on two things that are Mullion's and not a public contract:

- the name's shape (`MULLION_shared_` and 32 hexadecimal digits) inside the handle's mapping, found by that
  shape rather than at an offset;
- the request and reply message ids of the shared-name protocol.

Both are small and stable today, and both fail closed: a Direct3D 11 that is not Mullion makes
`xrCreateSwapchain` return `XR_ERROR_RUNTIME_FAILURE` with a log line saying so. A Mullion interface that hands
out the name (or the port) directly would replace the first; see [METALSHARP_WINE.md](METALSHARP_WINE.md).

`-[MTLSharedTextureHandle initWithMachPort:]` is not in Metal's public headers. Mullion relies on the same
selector. The server checks that it exists and reports plainly if a macOS release removes it.

## The protocol

`protocol/metalvr_protocol.h` is the whole contract: fixed-width, packed, little-endian structs over one TCP
connection per runtime instance, on the loopback interface only. Every request has exactly one response and no
request blocks in the server: frame pacing is a wake time the client sleeps to, so a slow frame cannot stall
another thread's `xrLocateSpace`.

- Time is the server's monotonic clock. The runtime measures the offset to its own clock once (`MVR_MSG_CLOCK`),
  and `XrTime` *is* server time, so nothing is converted on the hot path.
- Poses are in the server's tracking space, which is OpenXR `STAGE`: +Y up, −Z forward, floor origin.
- A tracking answer carries a `sample_id`. The frame rendered with those poses hands it back in `FRAME_END`, and
  the driver gives it to the headset so it reprojects from the pose the frame was drawn with.

TCP was chosen because it is the one transport a Windows DLL under any Wine and a native process both have
without extra code. The connection is unauthenticated; binding to loopback is the boundary.

## The server

`daemon/` owns everything on the headset's side of the protocol.

- **Session ownership.** One client holds the session at a time. A second `SESSION_BEGIN` gets `MVR_ERR_BUSY`,
  which the runtime reports as a failed `xrBeginSession`.
- **Composition** (`compositor.mm`). Layers are drawn in order into one IOSurface-backed BGRA frame, both eyes
  side by side. Projection layers are a straight copy of the submitted sub-image into the eye's half — the
  application rendered with the field of view the server gave it. Quad layers are projected with each eye's pose
  and field of view. The target is written through an sRGB view, so blending is in linear light; UNORM sources are
  taken as display-referred and decoded in the shader.
- **Encoding** (`encoder.mm`). VideoToolbox, hardware only, real time, no frame reordering; keyframes on request
  rather than on a timer. Output is Annex B. VideoToolbox's low-latency rate control is deliberately not used:
  measured, it was slower at a headset's resolution and dropped H.264 frames there.
- **Drivers** (`driver.h`). `sim` produces deterministic poses and inputs and can dump the stream; it exists so
  the path from an application's swapchain to encoded video is testable with no headset. `alvr` is ALVR's
  `server_core`; see [ALVR.md](ALVR.md).

## Session state

The runtime's state machine follows the server's flags, not assumptions:

- `IDLE → READY` when a headset is connected.
- `SYNCHRONIZED → VISIBLE → FOCUSED` once the application is in its frame loop and the headset is presenting.
- A headset that goes away walks the session down to `STOPPING`; it is `READY` again when the headset returns.
- A server that stops answering loses the instance: one `XrEventDataInstanceLossPending`, `LOSS_PENDING`, and
  `XR_ERROR_INSTANCE_LOST` from every call that needs the server. Nothing is retried or faked.
- No server at `xrCreateInstance` is `XR_ERROR_RUNTIME_UNAVAILABLE`, so a game that can run flat does.
- No headset at `xrGetSystem` is `XR_ERROR_FORM_FACTOR_UNAVAILABLE`: the resolution and field of view an
  application would be told are the headset's own, and are not known before it connects.

## Input

The server says which headset it drives and which controllers are in the user's hands (`mvr_system_info`'s
`headset` and `controllers`, described by `protocol/metalvr_headsets.h`), and reports each controller's inputs in
one layout (`mvr_controller_state`: buttons, touches, trigger, squeeze, stick, trackpad, forces, finger curls). Their
own interaction profile is preferred when the application suggested bindings for it (Touch, Touch Plus, Index, Vive,
Steam Frame); every other profile is read as a naming of the same physical inputs — `a`/`x` are the primary button,
a Vive profile's trackpad is a Touch controller's thumbstick and a Touch profile's thumbstick a Vive wand's trackpad
— so a game that ships bindings only for other controllers still gets input, without the runtime claiming to be
that hardware. With the controllers unknown, the order is Touch, Index, WMR, Vive, simple controller, Touch Plus,
Steam Frame.

## What works

- OpenXR 1.0 and 1.1 core: sessions, spaces, swapchains, the frame loop, projection and quad layers, actions,
  haptics; `XR_EXT_hand_tracking`, `XR_FB_display_refresh_rate`, and the Touch Plus and Steam Frame profiles.
- `XR_KHR_D3D11_enable` under Wine with Mullion, and `XR_KHR_metal_enable` on macOS.
- Hardware H.264 and HEVC encoding at a headset's resolution and refresh rate, with ALVR's foveated encoding.
- OpenVR applications through MetalVR's SteamVR driver, or through OpenComposite: rendering, legacy and
  action-manifest input, haptics.
- The headsets in [headsets/README.md](headsets/README.md), each with what is checked and what is not.

## What is not there yet

- Graphics bindings other than Direct3D 11 (Windows) and Metal (macOS): Direct3D 12, Vulkan, OpenGL.
- Layer types beyond projection and quad (cylinder, equirect, cube); depth submitted with a projection layer is
  accepted and unused.
- Eye tracking, passthrough for applications (the Lighthouse headsets' home shows the room with `--passthrough`),
  gaze-following foveation. Hand tracking is `XR_EXT_hand_tracking`, from headsets that track hands.
- 32-bit Windows applications.
- Play-area bounds from the headsets MetalVR drives itself (an ALVR headset's are reported), and recentering.
- Game audio and microphone.
- Starting the server on demand; today it must be running before the application creates its instance.
