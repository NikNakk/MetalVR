# SteamVR (OpenVR) games through OpenComposite

A SteamVR game talks OpenVR to an `openvr_api.dll` beside its executable. [OpenComposite](https://gitlab.com/znixian/OpenOVR)
is a replacement for that DLL which answers the game's OpenVR calls by making OpenXR calls. Under Wine those calls
reach MetalVR's OpenXR runtime (`metalvr_openxr.dll`), so the chain is:

```text
game (OpenVR, Direct3D 11) -> openvr_api.dll (OpenComposite) -> metalvr_openxr.dll -> metalvrd -> headset
```

This file records what OpenComposite needs from an OpenXR runtime, where MetalVR differs, how the DLL is built
here, and what has and has not been run. Source references are `file:line` in OpenComposite at the pinned commit,
before the patches below are applied.

## Version and license

| | |
|---|---|
| Repository | `https://gitlab.com/znixian/OpenOVR.git`, branch `openxr` |
| Pinned commit | `cff07db75c4823afe93ed7027b03d5f7bc86f164` (2025-07-21) |
| Bundled OpenXR loader | Khronos OpenXR-SDK `release-1.0.12`, submodule `libs/openxr-sdk`, compiled into the DLL |
| License | GPLv3 (`LICENSE.txt`); OpenVR headers BSD-3 (`LICENSE_OPENVR.txt`) |

MetalVR is MIT. OpenComposite is not copied into MetalVR's sources and not linked with any MetalVR binary. It is
cloned into `external/OpenComposite`, built as its own DLL, and shipped as a separate file. The only MetalVR-owned
text that touches it is `patches/opencomposite/*.patch`; those patches are changes to a GPLv3 work and are GPLv3
themselves. Anyone who is given the DLL must be offered the OpenComposite source at the pinned commit plus the
patches.

## Building

```sh
scripts/build-opencomposite.sh
```

The script clones the pinned commit if `external/OpenComposite` is missing, fetches Khronos Vulkan-Headers
(`vulkan-sdk-1.3.296.0`) into `build/opencomposite/deps`, applies the patches, and builds with MinGW-w64 through
`cmake/mingw-x86_64.cmake`. It prints the two paths of its result, which are the same file under two names:

- `build/opencomposite/openvr_api.dll`: replaces a game's `openvr_api.dll`
- `build/opencomposite/bin/vrclient_x64.dll`: the name an installed OpenVR runtime carries

It needs `git`, `cmake`, `ninja`, `python3` and the `x86_64-w64-mingw32` toolchain, and the network for whatever is
not on disk yet. Nothing else has to be installed.

What had to change for MinGW (OpenComposite's Windows build is written for MSVC):

| Patch | Why |
|---|---|
| `0001-cmake-build-with-mingw-w64.patch` | `-fvisibility=hidden` was passed to `windres`, which rejects it. The sources test `WIN32`, which GCC does not define with compiler extensions off. GNU `ld` needs the two static libraries grouped and does not read `#pragma comment(lib)`, so `d3d11 dxgi d3dcompiler shlwapi` are named. The DLL links the MinGW runtime statically. |
| `0002-sources-compile-with-gcc-for-windows.patch` | `atlbase.h` (ATL, MSVC only) is included but not needed. `CD3D11_VIEWPORT` does not exist in MinGW's `d3d11.h`. `std::ifstream(std::wstring)` is an MSVC extension. `OOVR_ABORTF` with no arguments does not compile with GCC. `fsync` is `_commit`. `PathStripPathA` needs `shlwapi.h`. |
| `0003-logging-construct-the-log-stream-on-first-use.patch` | Not a compile fix. The first build crashed while the DLL loaded: the static `Config` object logs from its constructor before the log's static `std::ofstream` existed. The stream is now constructed on first use. |

OpenComposite's CMake also wants a Vulkan SDK at `libs/vulkan` (its README points at a prebuilt archive). The
script fills that directory, which OpenComposite's own `.gitignore` leaves out, with the Khronos headers and an
import library generated from them with `dlltool`. No prebuilt binary is downloaded.

The DLL imports `d3d11.dll`, `dxgi.dll`, `D3DCOMPILER_47.dll`, `vulkan-1.dll`, `OPENGL32.dll`, `SHLWAPI.dll`,
`SHELL32.dll`, `USER32.dll`, `ADVAPI32.dll`, `KERNEL32.dll` and the UCRT (`api-ms-win-crt-*`). Wine provides all
of them. `vulkan-1.dll` is a load-time import even though a Direct3D 11 game never calls it.

Only the 64-bit DLL is built. A 32-bit game needs a 32-bit OpenComposite and a 32-bit MetalVR runtime; neither
exists.

## How OpenComposite finds the runtime

The OpenXR loader is compiled into the DLL (`CMakeLists.txt:136-192`), so the game needs no `openxr_loader.dll`.
Core `xr*` functions are the loader's own exports linked statically; extension functions are fetched with
`xrGetInstanceProcAddr` (`OpenOVR/Misc/xrutil.cpp:44-99`).

The loader (1.0.12) looks for the runtime manifest in this order (`libs/openxr-sdk/src/loader/manifest_file.cpp:616-631`):

1. the environment variable `XR_RUNTIME_JSON`, ignored when the process runs at high integrity
   (`src/common/platform_utils.hpp:202-225, 270-275`);
2. the registry value `ActiveRuntime` (a `REG_SZ` path to the manifest) under
   `HKEY_LOCAL_MACHINE\SOFTWARE\Khronos\OpenXR\1`, in the 64-bit view for a 64-bit game
   (`manifest_file.cpp:342-372`).

`scripts/install-prefix.sh` writes exactly that value for a Wine prefix. The loader then loads the manifest's
`library_path` and calls `xrNegotiateLoaderRuntimeInterface` with interface version 1 and API versions 1.0.0 to
1.1023.4095; it accepts any runtime whose API major version is 1 (`runtime_interface.cpp:76-118`). MetalVR's
negotiation and manifest (`file_format_version` 1.0.0) are accepted by it.

## Installing for a game

1. Make MetalVR the prefix's OpenXR runtime: `scripts/install-prefix.sh WINE PREFIX`.
2. Find the game's `openvr_api.dll` (Unity: `<Game>_Data/Plugins/x86_64/`; Unreal:
   `Engine/Binaries/ThirdParty/OpenVR/OpenVRvX_Y_Z/Win64/`), keep a copy, and put
   `build/opencomposite/openvr_api.dll` in its place.
3. Start `metalvrd`, connect the headset, then start the game. A game started before the headset is connected
   aborts in `VR_Init` unless `METALVR_HEADSET_WAIT=<seconds>` is set in its environment.

OpenComposite's log is `%LOCALAPPDATA%\OpenComposite\logs\opencomposite.log` inside the prefix
(`OpenOVR/logging.cpp:165-192`). A fatal error shows a message box and then aborts the process
(`logging.cpp:289-315`); nearly every failed OpenXR call is fatal (`OOVR_FAILED_XR_ABORT`, `logging.h:115-129`).

Optional `opencomposite.ini`, read from beside the DLL and then from the working directory
(`OpenOVR/Misc/Config.cpp:184-241`). An unknown option name is fatal. Defaults are right for MetalVR; the ones
that matter:

| Option | Default | Note for MetalVR |
|---|---|---|
| `initUsingVulkan` | `false` | Leave it. MetalVR has no `XR_KHR_vulkan_enable`, so it has no effect. |
| `invertUsingShaders` | `false` | Leave it: the runtime takes flipped images itself, and `true` shifts gamma and aborts on typeless game textures. |
| `supersampleRatio` | `1.0` | Scales the size the game is told to render at. |
| `enableHiddenMeshFix` | `true` | No effect while MetalVR's visibility mask is empty. |
| `dx10Mode` | `false` | Leave it: `true` hits an unimplemented path. |
| `logAllOpenVRCalls`, `logGetTrackedProperty` | `false` | For diagnosing a game. |

Prefix-wide alternative, not tried here: an unmodified game's own `openvr_api.dll` reads
`%LOCALAPPDATA%\openvr\openvrpaths.vrpath` and loads `<runtime>\bin\vrclient_x64.dll` from the first path under
`"runtime"`. OpenComposite's build output has that layout (`bin/vrclient_x64.dll`, `bin/version.txt`,
`openvrpaths.vrpath.in`), and the DLL exports `VRClientCoreFactory` for it. In that mode it writes `applist.json`
beside itself (`OpenOVR/Reimpl/BaseClientCore.cpp:56-92`), so its directory must be writable.

## What has been run

`scripts/test-openvr.sh` builds `tests/openvr/ovr_smoke.cpp`, an OpenVR Direct3D 11 application, with MinGW
against the OpenVR header OpenComposite carries, and runs it on the built `openvr_api.dll` under Wine with
Mullion against `metalvrd --driver sim`. Each case must exit cleanly, have no frame rejected, and compose the
picture it submitted.

| Case | What it covers |
|---|---|
| `legacy` | `VR_IsHmdPresent`, `VR_Init`, recommended size, projection, eye-to-head, hidden area mesh; 150 frames of `R8G8B8A8_UNORM` textures; legacy input (`GetControllerState`): both controllers appear as `oculus_touch` with valid poses |
| `upright` | a texture with a marked top edge composes upright |
| `flipped` | the same with bounds `vMin` 1, `vMax` 0 composes upside down |
| `typeless-msaa` | `R8G8B8A8_TYPELESS`, 4x multisampled game textures |
| `manifest` | `SetActionManifestPath`: digital, vector2 and pose actions read, the haptic reaches the server |

Every case also goes through OpenComposite's session restarts (first `Submit`, first input use) and
`VR_Shutdown`. All five pass on MetalSharp-Wine x86_64 (wine-11.17, Mullion 39d1e4a4, `GRAPHICS_BACKEND=dxmt`),
2026-11-15.

Not run: a real game, an MSVC-built caller, a real headset (ALVR driver), overlays, the skybox path, 10- and
16-bit game textures, side-by-side textures, a headset that is absent or connects late.

## What OpenComposite asks of the runtime

### Extensions

It enables an extension only if `xrEnumerateInstanceExtensionProperties` lists it
(`DrvOpenXR/DrvOpenXR.cpp:136-230`), with API version 1.0.0 (`:177`).

| Extension | Status | Where |
|---|---|---|
| `XR_KHR_D3D11_enable` | required for a Direct3D 11 game. Without it (and without Vulkan) there is no temporary session: `OOVR_FALSE_ABORT(temporaryGraphics)` | `DrvOpenXR.cpp:183-186`, `DrvOpenXR/XrBackend.cpp:65-71, 201-212`, `OpenOVR/Misc/xr_ext.h:139-143` |
| `XR_KHR_D3D12_enable`, `XR_KHR_vulkan_enable`, `XR_KHR_opengl_enable` | required only for a game that submits that kind of texture | `DrvOpenXR.cpp:188-205`, `XrBackend.cpp:226-374` |
| `XR_KHR_visibility_mask` | optional: hidden area mesh, otherwise an empty mesh | `DrvOpenXR.cpp:220-221`, `DrvOpenXR/XrHMD.cpp:164-242` |
| `XR_EXT_hand_tracking` | optional: skeletal input and a pose fallback | `DrvOpenXR.cpp:223-224`, `OpenOVR/Reimpl/BaseInput.cpp:755-761`, `DrvOpenXR/XrController.cpp:206-232` |
| `XR_EXT_hp_mixed_reality_controller` | optional: adds the Reverb G2 profile | `DrvOpenXR.cpp:229-230`, `OpenOVR/Misc/Input/InteractionProfile.cpp:105-106` |
| `XR_MNDX_xdev_space` | optional: generic trackers | `DrvOpenXR.cpp:226-227`, `XrBackend.cpp:979-1101` |
| `XR_EXT_debug_utils` | provided by the loader, which removes it before the runtime sees the list | `DrvOpenXR.cpp:216-217`, `libs/openxr-sdk/src/loader/loader_instance.cpp:80-83, 197-209` |

Not used at all: `XR_KHR_composition_layer_depth`, `XR_KHR_win32_convert_performance_counter_time`,
`XR_FB_display_refresh_rate`, `XR_EXT_local_floor`, cylinder, equirect and cube layers.

### Functions

Every one of these is called somewhere in OpenComposite; all are in MetalVR's `runtime/api.h` except the last row.

| Group | Functions |
|---|---|
| Instance, system | `xrEnumerateApiLayerProperties`, `xrEnumerateInstanceExtensionProperties`, `xrCreateInstance`, `xrDestroyInstance`, `xrGetInstanceProcAddr`, `xrPollEvent`, `xrResultToString`, `xrGetSystem`, `xrGetSystemProperties`, `xrEnumerateViewConfigurationViews`, `xrStringToPath`, `xrPathToString` |
| Session, spaces | `xrCreateSession`, `xrDestroySession`, `xrBeginSession`, `xrEndSession`, `xrRequestExitSession`, `xrCreateReferenceSpace`, `xrGetReferenceSpaceBoundsRect`, `xrCreateActionSpace`, `xrLocateSpace`, `xrDestroySpace`, `xrLocateViews` |
| Swapchains, frames | `xrCreateSwapchain`, `xrDestroySwapchain`, `xrEnumerateSwapchainImages`, `xrAcquireSwapchainImage`, `xrWaitSwapchainImage`, `xrReleaseSwapchainImage`, `xrWaitFrame`, `xrBeginFrame`, `xrEndFrame` (`xrEnumerateSwapchainFormats` only in the OpenGL path) |
| Input | `xrCreateActionSet`, `xrDestroyActionSet`, `xrCreateAction`, `xrSuggestInteractionProfileBindings`, `xrAttachSessionActionSets`, `xrGetCurrentInteractionProfile`, `xrSyncActions`, `xrGetActionStateBoolean`, `xrGetActionStateFloat`, `xrGetActionStateVector2f`, `xrEnumerateBoundSourcesForAction`, `xrApplyHapticFeedback` |
| Extensions | `xrGetD3D11GraphicsRequirementsKHR`, `xrGetVisibilityMaskKHR` |
| Not in MetalVR, optional | D3D12, Vulkan and OpenGL requirement functions; `xrCreateHandTrackerEXT`, `xrDestroyHandTrackerEXT`, `xrLocateHandJointsEXT`; the six `XR_MNDX_xdev_space` functions; debug-utils messenger functions (debug builds, answered by the loader) |

Never called: `xrGetActionStatePose`, `xrStopHapticFeedback`, `xrGetInputSourceLocalizedName`, `xrDestroyAction`,
`xrEnumerateReferenceSpaces`, `xrEnumerateEnvironmentBlendModes`, `xrLocateSpaces`.

### Direct3D 11

- **Two sessions, at least.** `VR_Init` creates a session on a device OpenComposite makes itself, on the adapter
  whose LUID `xrGetD3D11GraphicsRequirementsKHR` returns (`DrvOpenXR/tmp_gfx/TemporaryD3D11.cpp:34-58`). The
  first `Submit` (or overlay texture) ends and destroys it, calls `xrGetSystem` again, and creates a session on
  the device of the game's texture with a plain `XrGraphicsBindingD3D11KHR{device}`
  (`XrBackend.cpp:184-212`, `DrvOpenXR.cpp:316-382`). The first use of input restarts the session once more
  (`XrBackend.cpp:1103-1114`). `xrGetD3D11GraphicsRequirementsKHR` is not called again before the input restart.
- **Format** is not chosen from `xrEnumerateSwapchainFormats`. It is derived from the game's texture: the UNORM
  or UNORM_SRGB member of its format family. `ColorSpace_Gamma` gives sRGB, `ColorSpace_Linear` gives UNORM,
  `ColorSpace_Auto` gives sRGB for 8-bit formats and UNORM above (`dx11compositor.cpp:203-226`). Families known:
  `R8G8B8A8`, `B8G8R8A8`, `B8G8R8X8` (UNORM and sRGB); `R16G16B16A16`, `R10G10B10A2` (UNORM only);
  `B5G6R5`, `B5G5R5A1`, `B4G4R4A4`, `R10G10B10_XR_BIAS_A2`, `BC1` (`:502-517`). Anything else, float formats
  included, is fatal inside OpenComposite (`:204-206`).
- **Create info** (`:232-243`): width and height of the game's texture, `mipCount` of the game's texture,
  `sampleCount` 1, `arraySize` 1, `faceCount` 1, usage `TRANSFER_DST | SAMPLED | COLOR_ATTACHMENT`. One
  swapchain per eye and one per overlay. A failure is fatal (`:243-245`).
- **Images.** It creates a render target view of every image with the typed format and
  `D3D11_RTV_DIMENSION_TEXTURE2D`, mip 0 (`:86-101, 258-262`), which works on typeless and on typed textures. It
  writes an image with `CopySubresourceRegion(image, 0, 0, 0, 0, gameTexture, slice, &box)` (`:389-397`); for a
  multisampled game texture it first resolves into a texture of its own (`:264-273, 390-394`). So the runtime is
  never asked for a multisampled or array swapchain, and the copy goes from a typed or typeless game texture into
  the runtime's texture.
- **Per frame and eye**: acquire, wait with a 500 ms timeout (`XR_TIMEOUT_EXPIRED` is fatal), copy, release
  (`:297-312, 400-402`).
- **Bounds.** With `invertUsingShaders` off the whole texture is copied and the game's bounds become the layer's
  `imageRect`, including a **negative `extent.height`** when `vMin > vMax` and a non-zero offset for side-by-side
  textures (`OpenOVR/Compositor/compositor.cpp:14-52`).
- **Immediate context.** OpenComposite uses the game's immediate context on whatever thread calls `Submit`
  (`dx11compositor.cpp:103-106`); the shader path changes and partly restores its state (`:329-386`). It compiles
  two HLSL shaders with `D3DCompile` when the first texture arrives, whatever the settings (`:68-84, 108-112`).

### Session and frame loop

- **Reference spaces**: STAGE, LOCAL and VIEW, created at session start (`OpenOVR/Misc/xrutil.cpp:102-118`).
  LOCAL is recreated with a non-identity offset on `ResetSeatedZeroPose` (`OpenOVR/Reimpl/BaseSystem.cpp:851-877`).
  LOCAL_FLOOR is not used.
- **State.** After `xrCreateSession` it polls every 250 ms until the first state event arrives, without a limit
  (`XrBackend.cpp:838-862`). READY leads to `xrBeginSession`, STOPPING to `xrEndSession`, LOSS_PENDING to
  `VREvent_Quit` (`:771-803`). An event whose `session` is not the current one is fatal (`:761`). Shutdown is
  `xrRequestExitSession`, then up to ten 250 ms polls for EXITING, then `xrDestroySession` (`DrvOpenXR.cpp:361-379`).
- **Frames.** `WaitGetPoses` calls `xrWaitFrame` and `xrBeginFrame` back to back on the game's thread
  (`XrBackend.cpp:392-413`); `xrEndFrame` follows from the second `Submit` or from `PostPresentHandoff`
  (`:485-550`). A second `WaitGetPoses` without a submit begins a frame again, so the runtime sees
  `xrBeginFrame` twice. Nothing happens while the session is not running. `shouldRender` is ignored.
- **Layers**: one `XrCompositionLayerProjection` with `layerFlags` 0, two views, in STAGE or LOCAL (`:519-534`);
  `XrCompositionLayerQuad` with `BLEND_TEXTURE_SOURCE_ALPHA`, both eyes, in STAGE or LOCAL, for overlays and the
  skybox (`OpenOVR/Reimpl/BaseOverlay.cpp:84-152, 195-201`, `XrBackend.cpp:599-618`). Zero layers when the game
  submitted nothing. Blend mode OPAQUE.
- **`xrGetSystem` failing** is fatal in `VR_Init` (`DrvOpenXR.cpp:80-87`). `VR_IsHmdPresent` reports it as "no
  headset" instead, with an instance of its own that it destroys again (`OCOVR/openvr_api.cpp:282-319`).
- **FOCUSED** is `IsInputAvailable` (`XrBackend.cpp:741-744`) and the condition for looking for controllers
  before the game has loaded actions (`:832-835`). Rendering does not depend on it.
- **Poses**: `xrLocateSpace` with an `XrSpaceVelocity` chained, always (`OpenOVR/Misc/xrmoreutils.cpp:10-39`).
  Time is the last `predictedDisplayTime`, or before the first frame the time of the last state event, or 1
  (`xrutil.cpp:126-129`).
- **Bounds**: `xrGetReferenceSpaceBoundsRect(STAGE)`; `XR_SPACE_BOUNDS_UNAVAILABLE` means "no play area"
  (`XrBackend.cpp:701-727`).
- **Refresh rate**: not asked for. `Prop_DisplayFrequency_Float` is the constant 90 (`XrHMD.cpp:331-332`).

### Input

- **Profiles suggested**, each time actions are loaded (`InteractionProfile.cpp:101-116`):
  `microsoft/motion_controller`, `valve/index_controller`, `htc/vive_controller`, `oculus/touch_controller`,
  `khr/simple_controller`, and `hp/mixed_reality_controller` when its extension is enabled. A failing
  `xrSuggestInteractionProfileBindings` is fatal (`BaseInput.cpp:700, 1034`, `XrBackend.cpp:1188`).
- **Paths**, both hands unless noted (`OpenOVR/Misc/Input/*InteractionProfile.cpp`):
  - Touch: `x`, `y` click and touch, `menu/click` (left); `a`, `b` click and touch (right); `squeeze/value`,
    `trigger/value`, `trigger/touch`, `thumbstick`, `thumbstick/x`, `/y`, `/click`, `/touch`,
    `thumbrest/touch`, `grip/pose`, `aim/pose`, `output/haptic`.
  - Index: `a`, `b` click and touch, `squeeze/value`, `squeeze/force`, `trigger/click`, `/value`, `/touch`,
    `thumbstick` and its `x`, `y`, `click`, `touch`, `trackpad` and its `x`, `y`, `force`, `touch`, poses, haptic.
  - Vive: `squeeze/click`, `menu/click`, `trigger/click`, `/value`, `trackpad` and its `x`, `y`, `click`,
    `touch`, poses, haptic.
  - Windows MR: `menu/click`, `squeeze/click`, `trigger/value`, `thumbstick` and its `x`, `y`, `click`,
    `trackpad` and its `x`, `y`, `click`, `touch`, poses, haptic.
  - Simple: `select/click`, `menu/click`, poses, haptic.
  - No `system` path, no `palm_ext` or `grip_surface`, nothing outside `/user/hand/left` and `/right`.
- **Action types.** A game's boolean action is created as a FLOAT action and thresholded by OpenComposite
  (`BaseInput.cpp:561`); vector1 is FLOAT, vector2 is VECTOR2F, vibration is VIBRATION_OUTPUT, all with both
  hands as subaction paths (`:554-589`). Poses and skeletons get no action of their own: every pose comes from
  the grip and aim pose actions of the set `opencomposite-actions`, which has one action per hand and no
  subaction paths (`:1180-1243`). A boolean action bound to an analog path, and a float action bound to a click,
  both occur.
- **The current profile decides everything.** A controller exists for the game only after
  `XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED` arrives and `xrGetCurrentInteractionProfile` returns one of the
  profiles above for that hand (`XrBackend.cpp:808-810, 909-977`). A path it does not know is fatal (`:955-958`).
  The profile sets the controller type the game is told: `oculus_touch`, `knuckles`, `vive_controller`,
  `holographic_controller`, `hpmotioncontroller` (`GetOpenVRName` in each profile).
- **Which bindings.** For each profile it loads the game's binding file for that controller type. A profile the
  game has no file for gets the file of `oculus_touch`, else `knuckles`, else `generic`, else the first one,
  translated path by path; what does not translate is dropped (`BaseInput.cpp:594-654, 765-1035`).
- **Legacy input** (`GetControllerState`): buttons, stick, trigger and grip from `opencomposite-actions`
  (`:2641-2730`); haptic pulses with `XR_FREQUENCY_UNSPECIFIED` and the pulse's own duration, which is a few
  milliseconds at most (`:2732-2753`).

### Beyond the DLL

- No SteamVR installation, registry key or `vrclient` is consulted when the game's own `openvr_api.dll` is
  replaced. `VR_IsRuntimeInstalled` is always true (`openvr_api.cpp:334-338`).
- `VR_RuntimePath` and `VR_GetVRInitErrorAsSymbol` are stubs that abort (`:340-343`, `:212-215`). A game that
  asks where SteamVR is installed dies there.
- Interfaces OpenComposite has not implemented, and individual stubbed methods, abort with a message box naming
  them. That is an OpenComposite gap, not a runtime one; the log says which.

## What the comparison found, and where it stands

The first comparison of OpenComposite's needs with MetalVR's runtime found the gaps below. "Run" means observed
in a run; "source" means read from both sources and not run.

### Closed in the runtime

- **Flipped images** (run). OpenComposite submits `imageRect` with `extent.height < 0` for a game whose bounds
  have `vMin > vMax`. The runtime used to reject it (`XR_ERROR_SWAPCHAIN_RECT_INVALID` on every frame, nothing
  shown). A negative extent is now a mirrored rect, for projection and quad layers, carried to the server as a
  negative rect and composed mirrored. `scripts/test-openvr.sh` (case `flipped`) and `xr_smoke --flip` check
  the composed frame. `invertUsingShaders` is no longer needed and should stay off.
- **Formats** (run from OpenXR, source for OpenComposite). `R10G10B10A2_UNORM`, `R16G16B16A16_UNORM`,
  `B8G8R8X8_UNORM` and `B8G8R8X8_UNORM_SRGB` are offered beside the RGBA8, BGRA8 and RGBA16F ones; a missing
  format is fatal in OpenComposite (`dx11compositor.cpp:243-245`). Each was run through Mullion's shared textures
  with `xr_smoke --format N`. A BGRX layer never blends: its alpha channel holds nothing.
- **A headset that is not connected yet** (source). `xrGetSystem` returns `XR_ERROR_FORM_FACTOR_UNAVAILABLE`
  without a headset, and OpenComposite aborts on that in `VR_Init` (`DrvOpenXR.cpp:86`). The answer is still the
  honest one by default; `METALVR_HEADSET_WAIT=<seconds>` in the game's environment makes `xrGetSystem` wait that
  long for a headset to connect first.
- **Preferred interaction profile** (source). The runtime takes the first of Touch, Index, Windows MR, Vive,
  Simple that the application suggested usable bindings for; through OpenComposite that is always Touch, which is
  the right default and wrong for a game that ships only Vive or Index bindings (OpenComposite drops their
  trackpad bindings when it translates to Touch). `METALVR_INTERACTION_PROFILE=/interaction_profiles/htc/vive_controller`
  (or another of the five) puts that profile first. A profile outside OpenComposite's list is never returned:
  that would be fatal (`XrBackend.cpp:955-958`).
- **Trackpad and thumbstick on one stick** (source). On a profile with both (Index, Windows MR) the trackpad is
  now unbound instead of doubling the stick; on Vive, which has no stick, the trackpad is the stick.
- **Events outliving their session** (source). `xrDestroySession` drops the queued events that name the session;
  OpenComposite aborts on a state event for another session (`XrBackend.cpp:761`).
- **Eye offsets before tracking** (source). `xrLocateViews` against the VIEW space is valid whether or not the
  head is tracked, so `GetEyeToHeadTransform` and the IPD are right at startup.

### Open

1. **OpenComposite's typed-to-typeless copy is only run for RGBA8** (run for `R8G8B8A8_UNORM`, typeless and
   4x MSAA sources). It copies the game's texture into MetalVR's typeless image with `CopySubresourceRegion`.
   The runtime's own copy from that image into the share is run for every format; a BGRA, 10-bit or 16-bit game
   texture through OpenComposite is not.
2. **Controllers appear only when the session is FOCUSED, for games that have not loaded actions** (source; the
   focused case ran). MetalVR is FOCUSED only while the server reports the headset presenting, so the ALVR driver
   must report that as soon as frames flow.
3. **Legacy haptic pulses are very short** (source): a few milliseconds, no frequency, usually once per frame,
   forwarded as they are. Check on a headset; if nothing is felt, give them a floor of 10 to 20 ms.
4. **Grip pose alignment is unchecked** (source). OpenComposite applies a fixed transform from the OpenXR grip
   pose to SteamVR's controller pose for Touch (`OculusInteractionProfile.cpp:85-128`). If what ALVR delivers is
   not the OpenXR grip pose, hands sit offset in every game.
5. **The game is told 90 Hz** whatever the headset runs at (run). Constant in OpenComposite (`XrHMD.cpp:331-332`);
   a fourth patch could ask `XR_FB_display_refresh_rate`, which the runtime answers.
6. **No hidden area mesh and no play area** (run and source). Both are valid answers; a real mask would save the
   game some rendering, and a play area needs the headset's bounds in the protocol.
7. **Three sessions per launch** (run): OpenComposite's temporary one, the game's at its first `Submit`, and a
   restart at its first input use. Each costs at least 250 ms and recreates the swapchains. Harmless with the
   simulated headset; unchecked against a streaming client.
8. **Swapchains are as large as the game's texture**, 8192 at most (source), and take the game's mip count
   (level 0 is shared). A larger texture is fatal in OpenComposite; raise `kMaxSwapchainSize` if a game needs it.
9. **Float game textures abort inside OpenComposite** (`dx11compositor.cpp:204-206`). Not a runtime matter.

## Unverified

- No real SteamVR game has been started. The DLL is built with MinGW; games are built with MSVC and call it
  through C++ interfaces. No OpenVR interface in `OpenVRHeaders/` declares an overloaded virtual method (checked
  by script), which is the usual way the two compilers' vtables differ, but the combination has not been run.
- Everything marked "source" above, including the closed items that were only read and not run through
  OpenComposite.
- The ALVR driver: a headset that connects late, is taken off, or disconnects mid-game.
- Overlays, the skybox override, `PostPresentHandoff`, array and side-by-side textures.
- The prefix-wide install through `openvrpaths.vrpath`.
- Whether `XR_RUNTIME_JSON` is honored under Wine: the loader ignores it at high integrity and that was not
  checked; the registry route is what ran.

