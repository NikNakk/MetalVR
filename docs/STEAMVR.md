# SteamVR

MetalVR's SteamVR driver makes the headset `metalvrd` drives one of SteamVR's own. Windows SteamVR, running under
MetalSharp's Wine, lists it as a headset (an Oculus Rift CV1 shows as "Oculus Rift CV1"), and SteamVR games and
SteamVR Home render into it. OpenXR games don't need SteamVR: they use MetalVR's OpenXR runtime directly.

## How it works

SteamVR loads `driver_metalvr.dll` into its server process, `vrserver.exe`. The driver:

- connects to `metalvrd` on the loopback port, as the OpenXR runtime does, and registers one HMD with the
  headset's name, refresh rate, per-eye render size and field of view;
- sends SteamVR the head's pose about 500 times a second;
- takes SteamVR's frames in **direct mode**. SteamVR's compositor renders each eye into textures the driver made.
  These are Mullion shared textures, so `metalvrd` opens the same Metal textures and shows each frame with its own
  lens correction and reprojection. SteamVR's own distortion is switched off (the identity).

Each `Present` from the compositor waits for SteamVR's sync texture, then sends the frame to `metalvrd` as one
projection layer. The compositor's next frame starts at the headset's next refresh, as `metalvrd` paces it.

## Setting it up

1. Install MetalVR (`install.sh`). It copies the driver folder into
   `~/.metalsharp/metalvr-runtime/lib/metalvr/steamvr/metalvr`.
2. Start SteamVR once, so the prefix has an `openvrpaths.vrpath`, then quit it.
3. Register the driver with the Steam prefix's SteamVR:

   ```
   ~/.metalsharp/metalvr-runtime/bin/metalvr steamvr
   ```

   `metalvr steamvr --off` takes it out again. The file as it was before the first change is kept beside it as
   `openvrpaths.vrpath.metalvr-orig`.
4. Start `metalvrd` (for a Rift: `Rift Server.command`, or `metalvr rift --verbose`), then start SteamVR.

SteamVR needs `metalvrd` running when it starts. Without it the driver reports no headset, and SteamVR says so.

## Requirements and limits

- SteamVR, and the driver inside it, must run on Mullion's Direct3D 11 (MetalSharp's `dxmt` pipeline): the
  driver's textures are Mullion shared textures.
- Only one application can show on the headset at a time. While SteamVR's compositor holds `metalvrd`'s session,
  an OpenXR game started outside SteamVR waits; a game started from SteamVR goes through SteamVR.
- Oculus Touch controllers are registered with the headset and follow what `metalvrd` hears from them over the
  headset's radio (buttons, triggers, thumbsticks, haptics); their position needs the Constellation sensors, which
  aren't supported yet.
- The headset is tracked in rotation only (3DoF), at a fixed seated height.
- SteamVR covers the whole view with a fade color when it judges the application's frames or the tracking not good
  enough, and with MetalVR it never lifts that fade: the headset shows one flat color (teal). `metalvr steamvr`
  turns both fades off (`steamvr.doNotFadeToGrid`, `steamvr.forceFadeOnBadTracking`).
- MetalSharp starts a game on its MetalVR route through the game's VR launch entry (`steam://launch/<id>/VR`). A
  game without VR support still starts flat: Subnautica: Below Zero, for one, ships no OpenVR runtime.

## When it doesn't work

SteamVR writes the driver's messages, each starting with `metalvr:`, to `vrserver.txt` in Steam's `logs` folder
inside the prefix (`drive_c/Program Files (x86)/Steam/logs`).

| Message | Meaning |
| --- | --- |
| `metalvrd is not running` | Start `metalvrd` first, then SteamVR. |
| `D3D11CreateDevice failed` / `shared texture … failed` | SteamVR isn't running on Mullion's Direct3D 11. |
| `metalvrd's session is held by another application` | An OpenXR game is showing on the headset; close it. |
| `format N is not one the headset can show` | The compositor asked for a texture format the server can't open. |
