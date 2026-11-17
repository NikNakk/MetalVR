# Headset support plan

MetalVR drives one real headset today, the Oculus Rift CV1 ([RIFT.md](RIFT.md)), plus any headset that runs the
stock ALVR client ([ALVR.md](ALVR.md)). This plan adds ten more, one phase each:

| Phase | Headset | How it reaches the Mac | Driver |
|---|---|---|---|
| A | Meta Quest 1 | ALVR over Wi-Fi, or USB-C through adb | `alvr` |
| B | Meta Quest 2 | ALVR over Wi-Fi, or USB-C through adb | `alvr` |
| C | Meta Quest 3 | ALVR over Wi-Fi, or USB-C through adb | `alvr` |
| D | Meta Quest 3S | ALVR over Wi-Fi, or USB-C through adb | `alvr` |
| E | Valve Steam Frame | ALVR's client, patched by MetalVR to run there, over Wi-Fi | `alvr` |
| F | Valve Index | DisplayPort and USB on the Mac, Lighthouse base stations | new `lighthouse` |
| G | HTC Vive | HDMI (or DisplayPort) and USB on the Mac, Lighthouse 1.0 | `lighthouse` |
| H | HTC Vive Pro / Vive Pro 2 | DisplayPort and USB on the Mac, Lighthouse | `lighthouse` |
| I | Oculus Rift S | DisplayPort and USB on the Mac, inside-out cameras | `rift` (extended) |
| J | Sony PlayStation VR2 | Sony's PC adapter: DisplayPort and USB on the Mac; the headset tracks itself | new `psvr2` |

None of these headsets has been tried with MetalVR. Everything below comes from published specifications, the open
drivers that support them (ALVR, OpenHMD, Monado, libsurvive, the community Vive Pro 2 driver) and MetalVR's own code.
Facts are marked **(guess)** where no source confirms them and **(checked here)** where they were confirmed on a
development Mac. Each phase ends with what can be built and tested without the headset, and what cannot be known until
one is plugged in.

## What is built

Every phase's work that can be done without the headset is in the code, and checked without one
(`scripts/test-all.sh`: `headset-protocols`, `panel-*`, `alvr-models`, `native-vive-wands`, `native-index-touch`,
`native-psvr2-sense`, `rift-s-tracking`).
None of it has met a headset yet.

| Phase | How to run it | Built | Not built |
|---|---|---|---|
| A–D Quests | `metalvrd --trust 1234.client` (as before); `--headset quest-3` names the model if the client does not; a headset on USB: `--trust client.wired`, with `adb` on PATH or `--adb PATH` | the model from the client's report (`display_name`), its refresh rates, decoder budget and controllers; Touch Plus profile; battery; play area; SteamVR's Quest controllers; wired mode without ALVR's adb download; bare hands as `XR_EXT_hand_tracking` | choosing the refresh rate (the stream's negotiated one stays); Touch Plus's force, curl and slide (ALVR does not send them) |
| E Steam Frame | `metalvrd --headset steam-frame --trust NAME` (`--foveated-encoding` for any ALVR headset) | the model, its controller profile and buttons (bumper, D-pad, View, X and Y on the right) through the protocol, runtime and SteamVR; ALVR's foveated encoding, fixed at the center | the patched client's APK (`scripts/build-alvr-client.sh`) on a real Frame, its decoding speed, foveation following the eyes' gaze, SteamVR's own names for its controllers |
| F Index, G Vive, H Vive Pro / Pro 2 | `metalvrd --driver lighthouse`, or `metalvr-lighthouse start` (`--refresh 120`, `--pro2-mode 2`) | finding each on USB, its firmware configuration (lenses, IMU, cant), its lens correction, head rotation from its IMU, IPD and proximity, power-on, the Vive Pro 2's modes, refresh-rate choice, the controllers through their Watchman receivers (buttons, trackpad, fingers, forces, rotation, battery, haptics), SteamVR's Index and Vive controllers with the hand skeleton, the display overrides, the room through the headset's cameras in its home (`--passthrough`); positions of the head and controllers from the base stations (1.0 or 2.0) through libsurvive (`scripts/build-libsurvive.sh`), which also shows the base stations in SteamVR | libsurvive's tracking has only been run against its own simulated room; the cameras are drawn without their calibration |
| I Rift S | `metalvrd --driver rift`, or `metalvr-rift start` (the same as for the CV1: which one is on USB decides) | finding it, power-on, keep-alive, the screen following the proximity sensor, its IMU with its firmware calibration, the portrait panel with Monado's lens values, Touch controllers' buttons, stick, trigger and grip, and their rotation, the display override; the head's position from its five cameras and IMU through Basalt's visual-inertial tracking (`scripts/build-basalt.sh`), with the cameras' firmware calibration, as Monado does it | the cameras through macOS (whether macOS's UVC driver hands over the "Rift S Sensor" frames unconverted), the tracking on real pictures (it has run only on a synthetic headset); the controllers' position (no open driver has it); haptics (the command is not known); a correct lens model (none is published) |
| J PlayStation VR2 | `metalvrd --driver psvr2`, or `metalvr-psvr2 start` | finding it on USB (IOUSBHost, its vendor interfaces), its serial number and lens calibration, its IMU at 2 kHz, the proximity sensor and IPD dial, its position and heading from its own tracking (the headset solves it and sends it finished), the cameras run as Monado runs them, Monado's lens model with its lookup table, the Sense controllers over Bluetooth (buttons, stick, trigger, touches, battery, rotation from their calibrated IMU, vibration) as Touch controllers to games | the headset itself (none of it has run on one); its display's product code in the EDID (any Sony display is taken); the controllers' position (their LEDs are seen by the headset's cameras; no open driver solves that yet); eye tracking; passthrough; the headset's rumble; the adaptive triggers |

The shared work S1 to S10 below is built. S7 took a different shape than planned: rather than a presenter moved
into its own file, the CV1's driver became the one direct-display driver, and each other headset's USB side is a
backend behind a narrow interface (`daemon/direct_backend.h`): the window, pacing, home, lens correction and head
rotation are written once.

## What had to change first

Read across the first nine phases, MetalVR's code assumed a Rift in several places (all changed since, see above):

- **The headset had no identity.** `mvr_system_info` carried a name string and nothing else, and the ALVR driver
  named every headset "MetalVR ALVR Headset", although ALVR records the exact model the client reported ("Quest 2",
  "Quest 3S") in `session.json` under `client_connections.<host>.display_name`.
- **SteamVR was always shown Rift CV1 controllers**, with the CV1's grip offset and `willDriftInYaw`; the HMD's
  manufacturer was "Oculus" only when the name contained "Rift".
- **One refresh rate.** `xrEnumerateDisplayRefreshRatesFB` returned the single negotiated rate.
- **Controllers were Touch-shaped.** `mvr_controller_state` had no trackpad, finger curl, grip force, bumper, D-pad
  or View button, and the runtime reported whichever profile the game suggested first, so a Vive wand could be handed
  Touch bindings with A/B buttons it does not have.
- **The direct-display code was CV1-only**: the CV1's display IDs (`3ed2:0004`), a fixed 90 Hz, the panel split
  into left and right halves.
- **ALVR's battery and play-area events were dropped.** (Hand tracking is still not there.)

Each was fixed once, in the first phase that needed it, and reused by the phases after it:

| # | Shared work | First needed in | Reused by |
|---|---|---|---|
| S1 | A headset model and a controller kind in `mvr_system_info` (protocol version bump). SteamVR, OpenXR's `systemName` and the interaction profile are chosen from them; the `strstr(name, "Rift")` test goes. | A | all |
| S2 | ALVR model detection: read `display_name` from `session.json` on connect, look it up in a per-model table (name, defaults before `VIEWS_CONFIG`, refresh list, codec limits, controller kind). An unknown name gives generic defaults, never a Rift. | A | B–E |
| S3 | ALVR battery (`ALVR_EVENT_BATTERY`) and play area (`PLAYSPACE_SYNC`) through to OpenXR stage bounds and SteamVR battery properties. | A | B–E |
| S4 | Wired ALVR: document `--trust client.wired`, and require an `adb` the user installed. ALVR otherwise downloads Google's platform-tools at runtime (`alvr/adb/src/commands.rs`), which MetalVR must refuse. | A | B–D |
| S5 | The headset's refresh-rate list in `mvr_system_info` and `XR_FB_display_refresh_rate`. Changing the rate on an ALVR headset means renegotiating the stream, so the list may start read-only. | B | C–I |
| S6 | Touch Plus: `/interaction_profiles/meta/touch_plus_controller` and `XR_META_touch_controller_plus`. | C | D |
| S7 | One direct-display presenter, moved out of `driver_rift.mm` now that a second driver needs it: the window on the headset's display, pacing, the home and test scenes, `Fusion`. The display's IDs, refresh rate, panel rotation and eye split become per-model parameters. | F | G, H, I |
| S8 | `display` for every direct-display headset: the macOS override that shows it, keeping any EDID patches Apple's own override carries (the Vive Pro 2's). | F | G, H, I |
| S9 | Lighthouse core: read the headset's zlib-compressed JSON config, its lens distortion (Monado's Vive model), the IMU, the Watchman radio for controllers, and haptics. Base-station tracking through libsurvive after that. | F | G, H |
| S10 | Controller protocol: trackpad axes and touch, finger curl, grip and trackpad force, bumper, D-pad and View. | E or F | E–H |

## Recognizing the headset

MetalVR never assumes a headset. Each driver identifies one by evidence that cannot belong to another, and a driver
that finds nothing it recognizes reports no headset. It does not fall back to a Rift or to another model.

| Headset | Identified by | macOS display override |
|---|---|---|
| Rift CV1 (today) | USB `2833:0031` | `3ed2:0004` |
| Quest 1 | ALVR `display_name` "Quest 1"; over adb, `ro.product.device` = `monterey` | streamed, none |
| Quest 2 | "Quest 2"; `hollywood` | streamed, none |
| Quest 3 | "Quest 3" (exact match; "Quest 3S" shares the prefix); `eureka` | streamed, none |
| Quest 3S | "Quest 3S"; `panther` | streamed, none |
| Steam Frame | "Steam Frame" from MetalVR's patched client (`patches/alvr/0003`); the stock client says "Android (unknown)" | streamed, none |
| Valve Index | USB `28de:2300` with no HTC mainboard present, confirmed by the model in its config JSON | `5996:91a8` **(checked here)** |
| HTC Vive | USB `0bb4:2c87` (HTC mainboard) with `28de:2000` | `22d2:aa01` **(checked here)** |
| Vive Pro | USB `0bb4:0309` (Monado, OpenHMD) or `0bb4:030e` (libsurvive) with `28de:2300` | `22d2:aa02` (guess) |
| Vive Pro 2 | USB `0bb4:0342` with `28de:2300` | `22d2:aa04` (inferred from its timings) |
| Rift S | USB `2833:0051`, confirmed by feature report 6 (1440×2560, 80 Hz) | `3ed2:0012` **(checked here)** |
| PlayStation VR2 | USB `054c:0cde` (Monado); Sense controllers `054c:0e45` (left) and `0e46` (right) | Sony's `4dd9`, product not known |

Quest USB product IDs change with the USB mode (MTP, adb, Link) and are shared across models, so they are not used.
`28de:2300` is also used by the Vive Pro line and a Vive Tracker, so the Index is only recognized once its config
confirms it.

macOS marks every direct-display headset in this plan as a head-mounted display and keeps it off the desktop, as it
does the CV1: `/System/Library/Displays/Contents/Resources/Overrides` on a development Mac holds `is-hmd` overrides for
`3ed2` products 1, 2, 3, 4 and 12, `22d2` products aa01 to aa04, and `5996:91a8`. It holds none for Sony's displays
(**checked here**): the PlayStation VR2's display should appear as an ordinary one, with no override.

---

## Phase A: Meta Quest 1

| | |
|---|---|
| Model | Oculus Quest (1st generation), 2019; codename `monterey`; 64 GB and 128 GB. Discontinued 2020; Meta stopped updates in 2024. |
| Resolution | Two PenTile OLED panels, 1440×1600 per eye (2880×1600) |
| Refresh | 72 Hz only |
| Lenses | Fresnel-type, from the Oculus Go **(guess: not stated as Fresnel by Meta)**; IPD slider 58–72 mm |
| Field of view | about 93° × 93° visible; the runtime renders about 104° × 100° |
| Tracking | Inside-out, 4 cameras (Oculus Insight); controllers tracked by the cameras through IR LEDs in their rings; hand tracking since 2020; no eye or face tracking |
| Connectivity | Wireless (Wi-Fi 5) or USB-C. Standalone: Snapdragon 835, 4 GB |
| Audio | Speakers in the strap, two 3.5 mm jacks, microphones |
| Controllers | Touch (2nd generation, ring on top): A/B or X/Y (click and touch), Menu or Oculus, trigger (analog, touch), grip (analog), thumbstick (2 axes, click, touch), thumbrest touch; one AA each |
| Haptics | One motor per controller; none in the headset |
| Expandability | Straps and batteries only |
| Video decoding | H.264 and HEVC up to 2160p60; no AV1 |

**Path in MetalVR.** ALVR over Wi-Fi works as it is, with the ALVR client sideloaded (it is not on Meta's store for
Quest 1). Wired streaming over USB-C uses ALVR's adb mode, which forwards its ports through `adb` on the Mac. Meta's own
Link and Air Link use a closed protocol and Windows-only software; they cannot be used from a Mac.

**Work.**
1. S1, S2: a Quest 1 entry in the model table. 1216×1344 per eye **(guess)** as the default before the client's
   values arrive, 72 Hz, HEVC only, and a frame size the Snapdragon 835 can decode: about 6.9 Mpx at 72 Hz, for example
   2 × 1664 × 1856 **(derived)**. ALVR's default 4288-wide frame is too wide for it.
2. S1: SteamVR shows Quest Touch controllers (render model names from ALVR's `server_openvr/src/props.rs`,
   manufacturer "Oculus"), with no CV1 grip offset and `willDriftInYaw` false.
3. S3: battery and play area.
4. S4: wired mode with a user-installed `adb`; `docs/ALVR.md` covers sideloading and wired setup.

**Testable now:** model detection from fixture `session.json` files, the table, the seed it produces, SteamVR
properties per model, battery and play-area handling from synthetic ALVR events, the encoder at Quest 1 sizes on the
simulated headset.
**Needs the headset:** the resolution and refresh list the client really reports (the client falls back to 90 Hz when
the runtime lacks `XR_FB_display_refresh_rate`, which a Quest 1 panel cannot do), the decoder's real ceiling, the
controller grip and aim offsets, whether ALVR's current client still runs on Quest 1 firmware, wired mode on macOS.

**Risks:** ALVR keeps a separate OpenXR loader for Quest 1 and may drop it. Every frame size above the decoder's
limit fails on the headset, not on the Mac.

## Phase B: Meta Quest 2

| | |
|---|---|
| Model | Meta (Oculus) Quest 2, 2020; codename `hollywood`; 64/128/256 GB. Discontinued 2024. |
| Resolution | One fast-switch LCD, 1832×1920 per eye (3664×1920) |
| Refresh | 72, 90, 120 Hz (120 experimental); 60 and 80 Hz unconfirmed |
| Lenses | Fresnel; IPD in three positions, 58/63/68 mm |
| Field of view | about 97° × 93° visible (community measurements; no official figure) |
| Tracking | Inside-out, 4 IR cameras; controllers through IR LEDs in their rings; hand tracking; no eye or face tracking |
| Connectivity | Wireless (Wi-Fi 6) or USB-C. Standalone: Snapdragon XR2 Gen 1, 6 GB |
| Audio | Speakers in the strap, 3.5 mm jack, 2 microphones |
| Controllers | Touch (3rd generation, ring): same inputs as Quest 1; one AA, about 30 h |
| Haptics | One LRA per controller **(guess)** |
| Expandability | Elite Strap with or without battery |
| Video decoding | HEVC and H.264; no AV1 |

**Path in MetalVR.** As Quest 1: ALVR over Wi-Fi, or wired through adb. Link is not usable.

**Work.**
1. A Quest 2 entry in the model table (S2), and S5: the refresh list (72/90/120). The ALVR driver's current defaults
   (2144×2336 at 72 Hz) were meant as a Quest 2's but match neither its native nor its default render size; replace
   them with the table's.
2. SteamVR: `oculus_quest2_controller_left/right` and ALVR's Quest 2 model number.
3. Seed 90 Hz as the preferred rate for a known Quest 2.

**Testable now:** as phase A, plus the refresh list through `xrEnumerateDisplayRefreshRatesFB`.
**Needs the headset:** the real recommended size and refresh list, HEVC decoding at 120 Hz, controller offsets,
haptics, wired mode.

**Risks:** the client's model string comes from ALVR's code and could change between ALVR versions; MetalVR matches
the version it pins and falls back to generic.

## Phase C: Meta Quest 3

| | |
|---|---|
| Model | Meta Quest 3, 2023; codename `eureka`; 128 GB and 512 GB |
| Resolution | Two LCDs, 2064×2208 per eye (4128×2208); 1680×1760 default render size |
| Refresh | 72, 80, 90, 120 Hz; 96 and 100 Hz for newer OpenXR apps; up to 144 Hz and beyond in extended modes |
| Lenses | Two-element pancake; IPD wheel, 53–75 mm |
| Field of view | 110° × 96° |
| Tracking | Inside-out: 4 IR cameras, 2 color cameras, a depth projector; color passthrough; Touch Plus controllers without rings, tracked by LEDs in their faces; hand tracking; no eye or face tracking |
| Connectivity | Wireless (Wi-Fi 6E) or USB-C. Standalone: Snapdragon XR2 Gen 2, 8 GB |
| Audio | Speakers in the strap, 3.5 mm jack, microphones |
| Controllers | Touch Plus: A/B or X/Y (click, touch), Menu, Oculus, trigger (value, touch, and in OpenXR proximity, force, curl, slide), grip, thumbstick (axes, click, touch), thumbrest touch; one AA, about 30 h |
| Haptics | TruTouch, one voice-coil motor per controller; none in the headset |
| Expandability | Straps, battery straps, charging dock |
| Video decoding | AV1, HEVC, H.264 |

**Path in MetalVR.** ALVR over Wi-Fi or adb. The Mac cannot use the Quest 3's AV1: the encoders on an Apple M6 are
H.264, HEVC, ProRes and JPEG **(checked here)**, and Apple silicon decodes AV1 but does not encode it. MetalVR already
turns AV1 requests into HEVC.

**Work.**
1. A Quest 3 entry (S2), with an exact match on "Quest 3".
2. S6: the Touch Plus profile ahead of `oculus/touch_controller`. ALVR's own client binds every Quest's controllers as
   plain Touch, so trigger proximity maps to trigger touch, and force, curl and slide are reported as absent. Say so in
   `docs/ALVR.md`.
3. SteamVR: `oculus_quest_plus_controller_*`, "Meta Quest 3 (Left Controller)", manufacturer "Meta".
4. Later: hand tracking through `alvr_get_hand_skeleton` and `XR_EXT_hand_tracking`.

**Testable now:** detection and table; Touch Plus bindings and profile reporting with synthetic input; SteamVR
properties.
**Needs the headset:** the reported resolution and refresh list, HEVC decoding at about 4288×2240, controller offsets,
haptics.

**Risks:** a game that suggests only Touch Plus bindings works only once S6 is in. The Touch Plus profile promises
inputs (force, curl, slide) that ALVR does not send.

## Phase D: Meta Quest 3S

| | |
|---|---|
| Model | Meta Quest 3S, 2024; codename `panther`; 128 GB and 256 GB; an Xbox edition (2025) with the same hardware |
| Resolution | One LCD, 1832×1920 per eye (3664×1920) |
| Refresh | 72, 80, 90, 120 Hz (96 and 100 Hz possibly, sources disagree); no extended modes |
| Lenses | Fresnel; IPD in three positions, 58/63/68 mm |
| Field of view | about 96° × 90° |
| Tracking | Inside-out: 4 IR cameras, 2 color cameras, no depth sensor; Touch Plus controllers; hand tracking |
| Connectivity | Wireless (Wi-Fi 6E) or USB-C. Standalone: Snapdragon XR2 Gen 2, 8 GB |
| Audio | Speakers in the strap, microphones, **no headphone jack** (audio over USB-C) |
| Controllers | Touch Plus, as Quest 3 |
| Haptics | TruTouch in the controllers |
| Expandability | Straps, battery straps; an Action button on the headset |
| Video decoding | AV1, HEVC, H.264 |

**Path in MetalVR.** As Quest 3. The 3S is told apart from the Quest 3 only by ALVR's "Quest 3S", or `panther` over adb.

**Work.** A Quest 3S entry in the table: Quest 3 behavior with the Quest 2's panel size and lens steps, and a refresh
list that stops at 120 Hz. Everything else comes from phases A to C.

**Testable now:** detection, especially that "Quest 3S" is not taken for "Quest 3".
**Needs the headset:** the reported resolution, refresh list (96 and 100 Hz) and IPD per lens step.

## Phase E: Valve Steam Frame

| | |
|---|---|
| Model | Valve Steam Frame; 256 GB and 1 TB. Announced November 2025; launched September 2026 through a reservation queue, with orders running into 2027 (sources as of November 2026). |
| Resolution | Two LCDs, 2160×2160 per eye |
| Refresh | 72, 80, 90, 96, 108, 120 Hz, and 144 Hz experimental |
| Lenses | Pancake; IPD dial, 60–70 mm per Valve (57.6–69.6 mm per one review) |
| Field of view | "up to 110°" (Valve gives no breakdown) |
| Tracking | Inside-out: 4 monochrome fisheye cameras with IR illuminators; controllers tracked by the cameras through IR LEDs; 2 eye-tracking cameras for foveated streaming; no hand tracking; no Lighthouse support found |
| Connectivity | Wireless: the headset has Wi-Fi 7, and the bundled USB adapter for the PC is a Wi-Fi 6E card (`28de:2432`, a Realtek RTL8852CU). Wired: one USB-C 2.0 port. Standalone: Snapdragon 8 Gen 3, 16 GB, SteamOS on arm64; Android apps run in a container ("Lepton") |
| Audio | Two drivers per ear in the strap; microphone array |
| Controllers | Ringless, IR LEDs, IMU, TMR thumbsticks. A/B/X/Y all on the right controller, a D-pad and View on the left; each has a bumper, trigger, squeeze, Menu/System; capacitive sensing on every input and the handle; one AA, about 40 h |
| Haptics | In the controllers |
| Expandability | A 44-pin front port for cameras or PCIe, with published CAD files; developer mode gives ssh, adb and remote desktop |

**Path in MetalVR.** This is the one phase MetalVR cannot finish alone.
- **Valve's own streaming is closed.** Steam Link VR is undocumented and changes with Steam updates, and needs Windows
  SteamVR with a PC GPU encoder. The adapter is a plain Wi-Fi card with no macOS driver, and Wine cannot pass a network
  card through. Valve does not support Mac hosts. This route is closed.
- **ALVR is the only route, and it does not work yet.** The stock ALVR 20.14.1 client shows a blank screen in Lepton
  (Valve's runtime lacks `XR_KHR_convert_timespec_time`), and Lepton has only software video decoders. A community fork
  runs with software decoding. A native Linux arm64 client, like WiVRn's, would reach the hardware decoder. Whatever
  client works has to speak the protocol of the ALVR version MetalVR pins.
- **Wired:** USB-C 2.0 has the bandwidth for 80–150 Mb/s HEVC, but the class of USB network it offers, and so whether
  macOS can use it, is undocumented.
- **No direct display:** the Frame is a computer, not a display.

**Work.**
1. A Steam Frame entry in the table, chosen explicitly (`--headset steam-frame`) until a client identifies itself:
   HEVC 8-bit only, 2160×2160 per eye, 90 Hz default, 72–120 Hz allowed.
2. S10: bumper, D-pad and View bits; the Frame's own profile
   (`/interaction_profiles/valve/frame_controller_valve`, behind Valve's `XR_VALVE_frame_controller_interaction`),
   with Touch as the fallback. SteamVR's controller names for the Frame are unknown until they can be read from a
   SteamVR install.
3. Foveated streaming, in two steps: ALVR's static foveated encoding in `daemon/compositor.mm`, matched to the client's
   unwarp shader; then gaze-driven foveation, which needs a modified client to send gaze and undo the warp.
4. A client: ALVR's own, patched (`patches/alvr/0003-client-runs-on-the-steam-frame.patch`) to ask for
   `XR_KHR_convert_timespec_time` only when the runtime has it, and to name itself "Steam Frame", built as an APK
   by `scripts/build-alvr-client.sh` and installed with adb. It still decodes in
   software in Lepton; a native client that reaches the hardware decoder remains for later.

**Testable now:** the explicit model choice and table, the controller bits and profile, static foveation against
ALVR's shader math on the simulated headset.
**Needs the headset (and a working client):** everything about the stream: decoding speed, latency, the real field of
view, the USB network class, the headset's USB IDs, SteamVR's controller names.

**Risks:** no working client today; the vendor controller extension has already been renamed once; hardware may not
arrive before 2027.

## Phase F: Valve Index

| | |
|---|---|
| Model | Valve Index, 2019; sold as headset, controllers, base stations, or kits. Succeeded by the Steam Frame. |
| Resolution | Two LCDs (full RGB), 1440×1600 per eye (2880×1600) |
| Refresh | 80, 90, 120 Hz, and 144 Hz experimental |
| Lenses | Dual-element, canted; IPD slider 58–70 mm; eye-relief knob (changes the field of view) |
| Field of view | about 108° (unofficial; Valve says "about 20° more than Vive") |
| Tracking | Lighthouse (SteamVR Tracking 2.0 sensors): works with 1.0 base stations (up to 2) and 2.0 (up to 4); IMU. Stereo 960×960 global-shutter color cameras for passthrough, not tracking |
| Connectivity | Tethered: DisplayPort 1.2, USB 3.0 and 12 V in one 5 m cable with a breakaway. On a Mac: a USB-C/Thunderbolt to DisplayPort cable or adapter; Valve does not support HDMI adapters |
| Audio | Off-ear BMR speakers, 3.5 mm jack, dual microphones |
| Controllers | Index controllers: A, B, System, analog trigger, thumbstick, track button with force sensor, grip force sensor, per-finger capacitive sensing (finger tracking), IMU; USB-C charging, 1100 mAh, 7+ h. Vive wands and Vive Trackers also work |
| Haptics | HD LRA haptics in each controller |
| Expandability | Front USB port ("frunk") for peripherals; face gaskets |

**Path in MetalVR.** A direct-display headset like the CV1: frames drawn on its display through the Mac's
DisplayPort, its USB devices opened through IOHIDManager. The Index is the first Lighthouse headset, so this phase
builds what G and H reuse.

**Work.**
1. S7: the shared direct-display presenter out of `daemon/driver_rift.mm`.
2. S8: `display` for the Index: `/Library/Displays/Contents/Resources/Overrides/DisplayVendorID-5996/DisplayProductID-91a8`
   with `DisplayVendorID` 22934, `DisplayProductID` 37288, `is-hmd` false. The command and its guide move from
   `metalvr-rift` to a script that covers every direct-display headset.
3. S9: `daemon/driver_lighthouse.mm` with a per-model table, `--driver lighthouse` in `daemon/main.mm`:
   - recognize the Index as above, and report no headset when unsure;
   - read the config (feature report 0x10, then 0x11 until a 0-length chunk; zlib; JSON);
   - the Index lens distortion (per eye and color channel, ported from Monado's `u_compute_distortion_vive`, BSL-1.0)
     and its canted eye transforms;
   - the IMU (report 0x20) into `Fusion`, for rotation first;
   - refresh choice, 80 to 144 Hz, through `CGDisplaySetDisplayMode`.
4. S10 and the Watchman radio (`28de:2102`): Index controller input, finger curl and force into the protocol;
   `/interaction_profiles/valve/index_controller` with no trackpad folding; SteamVR's `{indexcontroller}` profile and
   skeletal input; haptics.
5. Position: libsurvive (MIT; Lighthouse 1.0 and 2.0) fed through IOHIDManager, since macOS's own HID driver holds the
   interfaces libusb would need; base stations as `trackingReferences`.
6. Later: passthrough from the cameras through AVFoundation.

**Testable now:** config reassembly and parsing on fixtures, distortion against Monado's output, IMU decoding,
identification over mock device lists (Index vs Vive Pro vs Tracker vs Rift), the controller protocol and bindings,
the override plist.
**Needs the headset:** whether macOS brings its display up through a USB-C to DisplayPort adapter at all (the Rift
notes already record macOS reading a headset's EDID and not showing it through a hub), which modes it offers (120,
144 Hz; 144 Hz only just fits DisplayPort 1.2), whether the display needs a power-on command, HID access, tracking
quality, controller pairing, haptics.

**Risks:** the display path through an adapter is the largest unknown in the plan. libsurvive has never been built for
macOS. The Index controllers' radio protocol is partly reverse-engineered. The headset is discontinued.

## Phase G: HTC Vive

| | |
|---|---|
| Model | HTC Vive, 2016 (a lighter revision in 2017; the Vive Pre was the developer edition) |
| Resolution | Two AMOLED panels (PenTile), 1080×1200 per eye (2160×1200) |
| Refresh | 90 Hz |
| Lenses | Fresnel; IPD knob, about 60.8–74.6 mm measured |
| Field of view | about 110° |
| Tracking | Lighthouse 1.0 only (32 headset sensors, 24 per wand; 2.0 base stations do not work with it); IMU; proximity sensor; one front camera |
| Connectivity | Tethered through a link box: HDMI 1.4 or mini-DisplayPort, USB, power. On a Mac: its own HDMI port |
| Audio | 3.5 mm jack and microphone; the Deluxe Audio Strap adds headphones |
| Controllers | Vive wands: trackpad (axes, touch, click), two-stage trigger, grip (one button), Menu, System; micro-USB, about 6 h |
| Haptics | One actuator per wand, driven by a USB report to the receiver in the headset, which forwards it by radio |
| Expandability | Deluxe Audio Strap, Wireless Adapter, TPCast, Vive Trackers. Index controllers need 2.0 base stations, so not with a Vive's 1.0 stations |

**Path in MetalVR.** The `lighthouse` driver from phase F, with a Vive model entry.

**Work.**
1. A Vive entry: `0bb4:2c87` (mainboard) and `28de:2000` (IMU and receiver), 1080×1200 at 90 Hz, Lighthouse 1.0,
   wands. Power-on is a feature report to the mainboard (`04 78 29 38 01 …`); whether the display needs it to light is
   unknown.
2. `display` for the Vive: `DisplayVendorID-22d2/DisplayProductID-aa01` (8914 / 43521).
3. Wands: trackpad, grip, Menu, System into the protocol (S10); the runtime offers `htc/vive_controller` when wands are
   attached, instead of the game's first suggested profile; SteamVR's `vive_controller` profile, manufacturer "HTC".
   Haptics as a pulse report through the receiver (`ff 8f 07 00` and pulse timings, tuned on hardware).

**Testable now:** the model entry and identification, the wand mapping and profile choice, haptic report encoding,
the override plist.
**Needs the headset:** the HDMI mode on a Mac, power-on, IMU axes, the real config, wand pairing (assumed already
stored in the receiver), Lighthouse 1.0 tracking quality.

**Risks:** Lighthouse 1.0 base stations are scarce secondhand; without base stations the Vive has only rotation.

## Phase H: HTC Vive Pro and Vive Pro 2

| | Vive Pro (2018; Vive Pro Eye 2019) | Vive Pro 2 (2021) |
|---|---|---|
| Resolution | Two AMOLED panels, 1440×1600 per eye | Two LCDs, 2448×2448 per eye (4896×2448) |
| Refresh | 90 Hz | 90 and 120 Hz |
| Lenses | IPD knob with sensor; adjustable eye relief | IPD 57–70 mm with sensor; adjustable eye relief |
| Field of view | 110° | up to 120° horizontal |
| Tracking | Lighthouse 1.0 and 2.0; IMU, proximity, IPD sensors; stereo front cameras; the Pro Eye adds 120 Hz eye tracking | Lighthouse 1.0 and 2.0; IMU, proximity, IPD sensors |
| Connectivity | Link box: DisplayPort 1.2, USB 3.0, power | Link Box 2.0: DisplayPort with Display Stream Compression for full resolution, USB 3.0, power |
| Audio | Removable on-ear headphones, microphones | Removable on-ear headphones, dual microphones |
| Controllers | Vive wands (2018 kit); Index controllers also work | Wands or Index controllers |
| Haptics | In the controllers | In the controllers |
| Expandability | Wireless Adapter, Vive Trackers, Facial Tracker, USB-C port | Wireless Adapter (up to 3264×1632 at 90 Hz), Vive Trackers, Facial Tracker, USB-C port |

The Vive Pro 2 has six display modes: 2448×1224 at 90 and 120 Hz, 3264×1632 at 90 Hz, 3680×1836 at 90 Hz, and
4896×2448 at 90 and 120 Hz. Only the last two need Display Stream Compression (DSC).

**Path in MetalVR.** The `lighthouse` driver, from DisplayPort over USB-C or Thunderbolt (not the Mac's HDMI port).

**Work.**
1. Model entries: Vive Pro by `0bb4:0309` or `0bb4:030e`, Vive Pro 2 by `0bb4:0342`; both with `28de:2300`, the
   presence of the HTC mainboard telling them from an Index.
2. The Vive Pro uses the Lighthouse distortion model from its config. The Vive Pro 2 uses HTC's own lens correction
   (Monado's code for it), read from the HTC mainboard's JSON.
3. The Vive Pro 2's mode select: a feature report (`wireless,0`, `dtd,N`, `chipreset`), after which the headset
   re-enumerates and the driver waits for it. Default to 3264×1632 at 90 Hz; 2448×1224 as the safe floor. Treat the
   4896-wide modes as unavailable until a Mac is seen to drive them: macOS's use of DSC with a third-party display is
   undocumented, and a cap near 600 MHz may rule out 3680×1836 (reported, not confirmed).
4. IPD and proximity from the HTC status report.
5. `display`: macOS's own override for `22d2:aa04` does not only hide the display, it adds the Pro 2's non-DSC display
   timings to its EDID. MetalVR's override must carry those patches along, or macOS may have no usable mode. Whether a
   `/Library` override replaces Apple's or merges with it is unknown, so copy them either way.
6. Eye tracking (Pro Eye) stays out of scope: its tracker uses Windows-only SDKs.

**Testable now:** identification, config parsing, status and mode-select reports, the override plist with its EDID
patches checked against Apple's file.
**Needs the headset:** whether macOS shows its display and which modes it offers, the pixel-clock limit, DSC, HID access,
`chipreset` re-enumeration, distortion, tracking.

**Risks:** the Vive Pro 2 at full resolution may never run on a Mac. The `aa02` and `aa03` assignments are guesses.
Without base stations there is only rotation.

## Phase I: Oculus Rift S

| | |
|---|---|
| Model | Oculus Rift S, made with Lenovo, 2019; discontinued 2021 |
| Resolution | One LCD, 1280×1440 per eye (2560×1440). The panel is portrait, 1440×2560: the left eye is the top half, the right eye the bottom, each rotated |
| Refresh | 80 Hz |
| Lenses | From the Oculus Go; no hardware IPD (software only, lenses fixed near 63.5 mm) |
| Field of view | 115° claimed (uncited); open drivers assume 105° horizontal |
| Tracking | Inside-out (Oculus Insight): 5 cameras in one USB video device, and an IMU; controllers tracked by the cameras |
| Connectivity | Tethered: DisplayPort 1.2 and USB 3.0 in one 5 m cable, powered from USB; no HDMI. On a Mac: USB-C/Thunderbolt to DisplayPort or mini-DisplayPort |
| Audio | Speakers in the strap, 3.5 mm jack, microphone |
| Controllers | Touch (2nd generation, as Quest 1): A/B/X/Y, Menu or Oculus, trigger, grip, thumbstick, touch on face buttons, stick and trigger; one AA |
| Haptics | In the controllers; no open driver implements them |
| Expandability | None |

**Rift S against the CV1, as MetalVR sees them.** USB `2833:0051` (CV1 `0031`); display `3ed2:0012` (CV1 `0004`);
DisplayPort instead of HDMI; 80 Hz instead of 90; a rotated portrait panel; its own power-on (reports 0x07, 0x14, 0x0A,
0x02, 0x05) and a keep-alive every second instead of every ten; the screen switched by its proximity sensor; its own
IMU report (0x65, calibrated from a JSON block in the headset); controllers on its own interface (report 0x67); no
external sensors.

**Path in MetalVR.** The existing `rift` driver, choosing the model by USB product ID. OpenHMD's Rift S driver is
already in MetalVR's fetched sources (`external/OpenHMD-rift/src/drv_oculus_rift_s`, BSL-1.0); Monado's (BSL-1.0) is
the other reference.

**Work.**
1. A model table in `daemon/driver_rift.mm` on top of S7: IDs, display, refresh, field of view, eye size, panel
   rotation and eye split.
2. The Rift S power-on, keep-alive, proximity-driven screen, IMU and calibration, as a parser file of its own.
3. The distortion shader for the rotated, top-and-bottom split. No correct Rift S distortion is published: both open
   drivers reuse the CV1's and mark it wrong. Start from theirs, with a calibration hook for tuning by eye.
4. Controllers: input from report 0x67 into the existing Touch mapping; rotation only (no open driver tracks their
   position); SteamVR's Rift S render models; no haptics until the command is known.
5. `metalvr-rift`: an override for `3ed2:0012` (16082 / 18); detect and report the model by USB product ID (49 or 81)
   rather than by the name "Rift", which a Rift S does not match.
6. Position tracking from the cameras, as Monado does it: the five cameras arrive as one UVC device ("Rift S
   Sensor"), read with AVFoundation (`daemon/rift_s_cameras.mm`); their calibration (firmware block 0x0f, Fisheye62
   lenses fitted to KB4) and the IMU go to Basalt, Monado's fork, loaded at run time through its VIT interface
   (`daemon/camera_tracking.cpp`, `daemon/rift_s_tracking.cpp`); an auto exposure keeps the pictures dark enough to
   track. `scripts/build-basalt.sh` builds Basalt with oneTBB, OpenCV and fmt into `build/basalt`. Checked on a
   synthetic Rift S (`tests/rift_s_tracking_check`): rendered camera frames in the device's own layout, IMU reports
   across the IMU clock's wrap, and the tracked path against the true one.

**Testable now:** the parsers on constructed bytes, the model table, IMU scaling, the rotated eye split rendered
offscreen and checked, the script's override and detection with mocked `ioreg` output.
**Needs the headset:** whether macOS brings the DisplayPort link up and at 80 Hz without rotating it, the order of
power-on and proximity, USB names and interfaces on macOS, distortion, axis signs.

**Risks:** the distortion has to be tuned by eye. Rotation-only controllers will disappoint in games built for tracked
hands. DisplayPort through USB-C is unreliable even on Windows for this headset.

---

## Phase J: Sony PlayStation VR2

| | |
|---|---|
| Model | PlayStation VR2 (CFI-ZVR1), 2023; on PCs through Sony's PlayStation VR2 PC adapter (2024) |
| Resolution | 2000×2040 per eye (one 4000×2040 image across both panels, Monado), OLED |
| Refresh | 90 or 120 Hz |
| Lenses | Fresnel, with an IPD dial (59 to 72 mm, reported in the headset's status) |
| Field of view | about 110° (Sony); Monado's per eye: 61.5° toward the temple, 43.5° toward the nose, 53° up and down |
| Tracking | Inside-out with four cameras, solved on the headset itself, which sends its pose over USB; eye tracking |
| Connectivity | One USB-C cable carrying DisplayPort, USB and 12 V power. A PC needs Sony's adapter (DisplayPort 1.4 in, USB 3 in, its own power), since few ports give DisplayPort and 12 V together; on a Mac the same: the adapter's DisplayPort from a USB-C or Thunderbolt port through a USB-C to DisplayPort cable, its USB into the Mac |
| Audio | 3.5 mm jack on the headset |
| Controllers | Sense controllers: cross/circle (right) and square/triangle (left), Options and Create, PS button, stick, trigger with touch, grip, finger touch on the buttons; adaptive triggers; haptics; tracking LEDs seen by the headset's cameras; Bluetooth to the computer |
| Haptics | In the controllers; a rumble motor in the headset |
| Expandability | None |

**Path in MetalVR.** The direct-display driver with a backend of its own (`daemon/backend_psvr2.mm`), from Monado's
`psvr2` and `pssense` drivers (BSL-1.0):
- The headset's USB side is vendor interfaces, not HID: status and IMU on an interrupt endpoint (interface 7,
  alternate 1), its own tracking's pose on a bulk endpoint (interface 3), and the camera, LED-detector, relocalizer and
  VD streams, which Monado reads and drops while the cameras run in mode 0x0a; MetalVR does the same. Controls (camera
  mode, brightness, firmware information, the lens calibration) are vendor requests on the default pipe. MetalVR opens
  it with IOUSBHost.
- The head's pose: the headset's own, its axes turned and corrected as Monado does (`psvr2::headFromSlam`), handed to
  the driver as a tracker's pose, which takes the position from it and pulls the IMU's heading toward it, as it does
  with the Rift's sensors and the base stations.
- The lens: Monado's model, a radial lookup table per colour with each eye's offset and tilt from the headset's
  calibration (report 0x8f), as lens model 3 in the driver's shader.
- The Sense controllers over Bluetooth: their input report 0x31 (CRC-checked), their IMU calibration (feature report
  5, which also switches them into those reports), and vibration through output report 0x31. Games are given the Touch
  profile, as Monado binds them; SteamVR shows them as Touch controllers named for the Sense.

**Testable now:** every report and request, on constructed bytes (`headset_protocol_check`: status and IMU axes, the
tracking record and its quaternion continuity, requests, firmware, lens calibration and model, the controllers' reports,
CRC, calibration and vibration); the panel through its lens (`--panel-check psvr2`); the model as OpenXR and SteamVR
see it (`native-psvr2-sense`); the driver waiting for the headset and stopping cleanly.
**Needs the headset:** whether macOS lets IOUSBHost open its interfaces and take the vendor requests (through the
device, or else an interface); whether the adapter's display comes up at 4000×2040 and 90 or 120 Hz on a Mac (it may
need DisplayPort's compression); its EDID product code; the Sense controllers' pairing with a Mac and their IMU axes.

**Risks:** the display path through the adapter is the largest unknown. The controllers are rotation only, and games
built for tracked hands will feel it, as with the Rift S.

---

## Order and dependencies

The phases run A to I, but not all of them can finish in that order:

- **A to D** depend only on MetalVR and the ALVR version it pins. Each needs a headset only for its last check.
- **E** depends on ALVR's client running on the Frame. MetalVR patches the client for Valve's runtime; whether it
  streams well with software decoding is known only on a headset.
- **F** builds the shared direct-display and Lighthouse code. G and H are small once F exists.
- **I** reuses S7 from F and otherwise stands alone.
- **J** reuses the direct-display driver from F and I; its tracking is the headset's own.
- The largest unknowns are on the Mac: whether macOS shows a headset's display through a USB-C to DisplayPort adapter
  (F, H, I), whether it drives the Vive Pro 2 at all (H), and whether libsurvive works on macOS (F, G, H).

## Sources

The research reports behind this plan cite their sources fact by fact. The main ones:

- ALVR (vendored at `external/ALVR`): `alvr/system_info/src/lib.rs` (model detection), `client_core/src/connection.rs`,
  `client_openxr/src/lib.rs` and `interaction.rs`, `server_core/src/connection.rs`, `adb/src/commands.rs`,
  `server_openvr/src/props.rs`.
- OpenHMD (vendored at `external/OpenHMD-rift`), Monado's driver documentation
  (https://monado.pages.freedesktop.org/monado/), libsurvive (https://github.com/cntools/libsurvive), the community
  Vive Pro 2 driver (https://github.com/CertainLach/VivePro2-Linux-Driver), Valve's udev rules
  (https://github.com/ValveSoftware/steam-devices), Linux's EDID quirks (`drivers/gpu/drm/drm_edid.c`).
- Khronos OpenXR registry (`XR_META_touch_controller_plus`); Valve's Steam Frame input documentation
  (https://partner.steamgames.com/doc/steamhardware/steamframe/input); Meta's refresh-rate documentation
  (https://developers.meta.com/horizon/documentation/native/android/mobile-display-refresh-rate/).
- Specifications: Meta, Valve and HTC product pages, Road to VR, UploadVR, Tom's Hardware, Wikipedia; for the Steam
  Frame, https://vr.org/steam-frame-specs (updated November 2026) and ALVR issue
  https://github.com/alvr-org/ALVR/issues/3411.
- macOS's own display overrides in `/System/Library/Displays/Contents/Resources/Overrides`.
- PlayStation VR2: Monado's `drivers/psvr2` and `drivers/pssense` (in the reference checkout of Monado), the Linux VR
  Adventures wiki (https://vronlinux.org/docs/hardware/psvr2/), and the PSVR2 Toolkit
  (https://github.com/BnuuySolutions/PSVR2Toolkit).
