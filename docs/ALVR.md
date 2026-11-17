# The ALVR driver

metalvrd's `alvr` driver streams to a headset running the stock [ALVR](https://github.com/alvr-org/ALVR) client.
MetalVR does not reimplement ALVR's protocol: it links ALVR's own `alvr_server_core` crate (MIT) and
`daemon/driver_alvr.cpp` implements `metalvr::Driver` on its C API. `server_core` owns discovery, the handshake,
the sockets, bitrate control and statistics; metalvrd owns composition and encoding.

Pinned to ALVR **v20.14.1** (`scripts/build-alvr.sh`). ALVR's protocol id is its major version, so any v20.x
client is accepted.

## Building

```
scripts/build-alvr.sh
```

It is idempotent and writes only under `external/ALVR` and `build/alvr` (plus cargo's download cache). It

1. clones ALVR at the tag if `external/ALVR` is missing, and refuses to build a checkout at any other commit;
2. fetches `openvr/headers/openvr_driver.h` at the commit ALVR's submodule pins. ALVR's `alvr_session` build
   script reads that one file; the script fetches it with a sparse, blob-filtered checkout instead of the whole
   OpenVR repository (480 KB instead of about 650 MB);
3. applies `patches/alvr/*.patch`;
4. builds `alvr_server_core` as a static library with
   `cargo rustc --release --locked -p alvr_server_core --lib --crate-type staticlib` (the crate declares
   `rlib` and `cdylib`; `--crate-type` adds the static library without editing its manifest);
5. installs `cbindgen` 0.29.4 into `build/alvr/tools` and generates the C header the way ALVR's own
   `cargo xtask build-server-lib` does.

It ends by printing `ALVR_LIBRARY`, `ALVR_INCLUDE_DIR`, `ALVR_HEADER` and `ALVR_LINK_FLAGS`.

| | |
|---|---|
| library | `build/alvr/target/release/libalvr_server_core.a` |
| header | `build/alvr/include/alvr_server_core.h` |
| link | `-Wl,-framework,CoreFoundation -lobjc -Wl,-framework,IOKit -Wl,-framework,AudioUnit -Wl,-framework,CoreAudio -Wl,-framework,Security -lbz2 -Wl,-framework,AppKit -Wl,-framework,Foundation -liconv` |

The link line is what rustc reports for the library (`--print native-static-libs`) without its duplicates and
without `-lSystem -lc -lm`, each framework written as one token so that CMake's `target_link_options`, which
drops repeated options, cannot mangle it. With today's code only IOKit, AudioUnit, CoreAudio, Security, AppKit
and `-lbz2` are needed for the link to succeed.

```
cmake -DMETALVR_WITH_ALVR=ON \
  -DMETALVR_ALVR_LIBRARY=$PWD/build/alvr/target/release/libalvr_server_core.a \
  -DMETALVR_ALVR_INCLUDE_DIR=$PWD/build/alvr/include \
  "-DMETALVR_ALVR_LINK_FLAGS=<the ALVR_LINK_FLAGS line>" …
```

The header compiles as C++20 under `-Wall -Wextra`. Under `-Wpedantic` clang reports its two anonymous structs
(`-Wgnu-anonymous-struct`).

### The patches

- `0001-c-api-button-value-field-is-valid-c.patch` renames one field of `AlvrButtonValue` from `float` to
  `float_value`. Unpatched, cbindgen emits `float float;`, which is not C or C++, so the header cannot be included.
- `0002-web-server-listens-on-loopback.patch` binds ALVR's dashboard API (TCP 8082) to 127.0.0.1 instead of every
  interface. That API takes unauthenticated requests that trust a client or change any setting, so left on the
  network it would let any machine there decide which headsets may connect.
- `0003-client-runs-on-the-steam-frame.patch` changes ALVR's headset client, not the server. Valve's OpenXR runtime
  on the Steam Frame has no `XR_KHR_convert_timespec_time`, which the stock client requires, so it shows a blank
  screen there; patched, the client asks for that extension only when the runtime has it, and does without the
  runtime's clock when it does not (every place that reads it already handles having none). It also names a headset
  whose maker is Valve "Steam Frame", so MetalVR recognizes it. `scripts/build-alvr.sh` applies it with the others but
  builds only `server_core`; `scripts/build-alvr-client.sh` builds the client as
  `build/alvr-client/alvr_client_android.apk` (fetching the Android NDK, SDK, a JDK 17 and cargo-apk into
  `build/android`; Google's SDK license is shown for you to accept), to install with
  `adb install -r build/alvr-client/alvr_client_android.apk` on a Steam Frame in developer mode.

Nothing else in ALVR is changed.

## Pairing a headset

ALVR streams only to clients its session lists as trusted. MetalVR trusts exactly the headsets it is told to:

```
metalvrd --trust 1234.client
```

The name is the hostname the ALVR client shows on the headset. `--trust` can be repeated, and a headset stays
trusted in `<config_dir>/session.json` afterwards. A headset that announces itself without being trusted is
refused and named once in metalvrd's output:

```
[alvr] headset '1234.client' wants to connect and is not trusted; start metalvrd with --trust 1234.client to allow it
```

There is no switch that trusts every headset, and ALVR's own `auto_trust_clients` setting is pinned off.

How it works: `server_core` reads its client list from `session.json` once per process and its C API has no call
to change it, so the driver writes the trusted entries into that file before ALVR loads it, then checks the list
ALVR ended up with and refuses to start if a requested headset is not trusted in it. ALVR knows a client by the
exact hostname it announces — `1234.client` from the stock client's broadcast, a `name.local.` from a client found
through mDNS — so each name is written both ways, and `1234.client`, `1234.client.local` and `1234.client.local.`
all mean the same headset. Untrusted announcements are found by reading the client list back from `session.json`,
where ALVR records them.

ALVR listens on UDP 9943 (client announcements) and mDNS, streams on port 9944, and serves its dashboard API on
127.0.0.1:8082. Its logs are `<config_dir>/session_log.txt` (rewritten on each start) and
`<config_dir>/crash_log.txt`.

The first connection after the negotiated stream changes (always the very first one) is refused once: ALVR tells
the client "the streamer is restarting", logs `Cannot restart SteamVR. No dashboard process found`, and the
client connects again about two seconds later. That is ALVR expecting SteamVR to restart; nothing needs to.

## Which headset it is

The stock client tells ALVR what it runs on, and ALVR keeps that in `session.json` as the client's
`display_name`: "Quest 1", "Quest 2", "Quest 3", "Quest 3S", "Quest Pro" (ALVR's `alvr_system_info`). The driver
reads it when the headset connects, and before that from what the trusted headset reported last time, and names
the headset by it (`metalvr_headsets.h`): OpenXR's `systemName` and `vendorId`, SteamVR's model and controllers,
the interaction profile games are given first (Touch Plus on a Quest 3, 3S or Pro), the refresh rates
`XR_FB_display_refresh_rate` lists, and the warning when the negotiated stream is more than the headset's video
decoder is known to take (ALVR's default is too much for a Quest 1). The name is matched whole: a "Quest 3S" is
never taken for a "Quest 3". Anything else (a Pico, a Vive Focus, "Android (unknown)") is an unknown headset with
Touch-shaped controllers, which is what ALVR's client sends for every headset; never a Rift.

`--headset quest-2` (or `quest-1`, `quest-3`, `quest-3s`, `quest-pro`, `steam-frame`) names the headset instead,
for a client that does not say: the stock ALVR client on a Steam Frame reports "Android (unknown)"; the client built
with `0003-client-runs-on-the-steam-frame.patch` reports "Steam Frame", which MetalVR names by itself. A headset that reports
another model than the one named is logged, and the one named is used. A session MetalVR creates for a named
headset asks for its usual PC VR rate (90 Hz; 72 Hz for a Quest 1) instead of ALVR's 72 Hz; an existing session
keeps what its settings say.

The headset's battery and its controllers' (`ALVR_EVENT_BATTERY`) go to OpenXR applications through the protocol
and to SteamVR as battery levels; the size of its play-area boundary (`ALVR_EVENT_PLAYSPACE_SYNC`) is the STAGE
space's bounds (`xrGetReferenceSpaceBoundsRect`).

## A headset on USB

ALVR reaches a headset on USB through adb, under the client name `client.wired`:

```
metalvrd --trust client.wired
```

The headset needs USB debugging on (its developer mode), and the Mac needs Android's platform-tools
(`brew install android-platform-tools`), or `--adb PATH` naming an `adb`. Without one, metalvrd does not start:
ALVR would otherwise download Google's platform-tools by itself at run time, which MetalVR does not allow. ALVR's
wired mode forwards its ports over adb; it has not been tried from a Mac with a headset.

## How the C API is used

Initialization, once per process, in `start()`:

1. `session.json` is prepared (see *Pinned settings* and *Pairing a headset*) — before ALVR reads it, which it
   does once.
2. `alvr_initialize_environment(config_dir, config_dir)`
3. `alvr_initialize_logging(session_log.txt, crash_log.txt)`

then, every time ALVR is brought up (`start()`, and the restarts below):

4. `alvr_initialize()` → the last negotiated per-eye sizes
5. `alvr_get_settings_json()` → the pins are checked, and the client list in `session.json` is checked against
   the headsets asked for; `start()` fails if either does not hold
6. the stream configuration is read from `session.json` (`openvr_config`: sizes, refresh rate, codec)
7. `alvr_set_video_config_nals()` if the encoder's parameter sets are already known
8. `alvr_start_connection()`

Events are polled on the driver's own thread with `alvr_poll_event(&event, 10 ms)`:

| event | what the driver does |
|---|---|
| `CLIENT_CONNECTED` | reads what ALVR negotiated from `session.json`, updates `systemInfo()`/`videoParams()`, asks for a keyframe |
| `CLIENT_DISCONNECTED` | `connected()` goes false, inputs reset |
| `VIEWS_CONFIG` | per-eye FOV, IPD and view offsets |
| `TRACKING_UPDATED{ts}` | `alvr_get_device_motion()` for `/user/head`, `/user/hand/left`, `/user/hand/right` at `ts`; that is the sample `tracking()` returns |
| `BUTTONS_UPDATED` | `alvr_get_buttons()`, mapped to `mvr_button` bits and analog values |
| `REQUEST_IDR` | `consumeKeyframeRequest()` becomes true, at most once per `connection.minimum_idr_interval_ms` |
| `RESTART_PENDING` | `alvr_shutdown()`, then steps 4–8 |
| `SHUTDOWN_PENDING` | `alvr_shutdown()`; the driver idles until metalvrd restarts |
| `BATTERY`, `PLAYSPACE_SYNC`, `CAPTURE_FRAME` | ignored |

The same thread calls `alvr_get_dynamic_encoder_params()` every 100 ms; its bitrate is `videoParams().bitrate_bps`.
Once a second it reads the client list from `session.json` to report headsets that are not trusted.

Per frame, from metalvrd's threads:

- `sendVideoConfig` → `alvr_set_video_config_nals(codec, parameter sets)`
- `sendFrame(frame, …)` → `alvr_report_present(frame.sample_id, now − frame.submitted_ns)`,
  `alvr_report_composed(frame.sample_id, now − frame.composed_ns)`, then
  `alvr_send_video_nal(frame.sample_id, bytes, frame.keyframe)`
- `haptic` → `alvr_send_haptics(hand path id, duration, frequency, amplitude)`

`stop()` joins the event thread and calls `alvr_shutdown()`.

Not used: `alvr_get_hand_skeleton`, `alvr_duration_until_next_vsync`, `alvr_restart`, `alvr_get_time_ns`
(which returns about zero in this ALVR version) and the `alvr_error`/`alvr_warn`/… logging calls.

`server_core` is a set of process-wide singletons: one config directory per process and one running instance.

### Timestamps

ALVR names every tracking packet by the client's own timestamp for it, and everything else is keyed on that
number: `alvr_get_device_motion`, `alvr_send_video_nal`, `alvr_report_present`, `alvr_report_composed`. The driver
uses it as `sample_id`. A frame must be sent with the `sample_id` of the poses it was *rendered* with: the client
looks up the head pose it sent under that timestamp and reprojects from it.

It is not a time on metalvrd's clock. The client predicts each packet's poses itself, for when it expects a frame
drawn from them to reach the display, so `tracking(target_time_ns)` returns the newest packet as it is and does
not predict to `target_time_ns`. The head arrives without velocities (the client zeroes them).

### Statistics

ALVR keeps three moments per tracking packet and charges the time between them to the application, the
compositor and the encoder. In ALVR's source (`statistics.rs`):

| ALVR's moment | set by | metalvrd's time | ALVR's interval ending there |
|---|---|---|---|
| tracking received | ALVR, when the packet arrives | — | |
| frame present | `alvr_report_present(ts, offset)` | `FrameInfo::submitted_ns`: the application ended the frame | "game time": tracking received → present |
| frame composed | `alvr_report_composed(ts, offset)` | `FrameInfo::composed_ns`: composed and handed to the encoder | "server compositor": present → composed |
| frame encoded | `alvr_send_video_nal(ts, …)`, at the call | `FrameInfo::encoded_ns`, in effect | "encoder": composed → encoded |

`offset` is a non-negative duration: ALVR records the moment as its own clock's now minus `offset`. The driver
therefore passes how long ago each of metalvrd's times was, `now − submitted_ns` and `now − composed_ns`, both
read from the same clock immediately before the calls. "Encoded" takes no offset — it is the instant
`alvr_send_video_nal` is called — so ALVR's encoder interval is `encoded_ns − composed_ns` plus the hand-over from
the encoder's callback to the driver. The two reports come before `alvr_send_video_nal`, which reads "composed"
to feed the encoder interval to ALVR's bitrate control. Offsets are limited to one second: ALVR subtracts them
from a clock that starts at boot.

What is left of the client's measured total after these and the client's own intervals is what ALVR calls network
latency.

### Video

- Frames are `2 × stream width` by `stream height`, left eye in the left half, no foveation: `videoParams()`.
- The decoder gets its parameter sets only from `sendVideoConfig`; a frame whose codec is not the one those
  parameter sets were given for is dropped. Like the stock server, the driver strips
  leading parameter sets and access unit delimiters from the frames it forwards.
- ALVR hands the parameter sets to a client only when the client asks for a keyframe, which it does when video
  reaches it and it has no decoder. So the first keyframe of a connection is normally discarded by the client
  and a second one requested.
- The driver drops frames until the first keyframe of a connection and keeps `consumeKeyframeRequest()` true
  until one arrives.
- If `sendVideoConfig` delivers different parameter sets while a headset is connected, the driver restarts ALVR
  so the client reconnects and fetches them (about two seconds). Reconfigure the encoder when `generation()`
  changes, not mid-stream.

## Coordinates and units

ALVR's client is an OpenXR application and passes OpenXR conventions through, so poses need no axis conversion:
right-handed, +Y up, −Z forward, meters, quaternions `x y z w`. The client tracks in its STAGE space, and ALVR
recenters it whenever the client sends its play space — when a stream starts and when the headset recenters —
keeping the floor height, putting the origin under the head and zeroing the yaw (ALVR's default `LocalFloor` and
`Yaw` recentering modes). That is the tracking space metalvrd expects. FOV angles are OpenXR's: radians, left and
down negative.

- Views: head pose composed with the per-eye transform from `VIEWS_CONFIG`. ALVR's client reports parallel views
  (±IPD/2 on X, no rotation) with the FOV adjusted to match; it reprojects from exactly these.
- Controllers: ALVR carries one pose per hand, the OpenXR grip pose. The aim pose is derived from it with a fixed
  offset (40° pitch down, 25 mm down, 85 mm forward) — an estimate for Touch controllers.
- ALVR rotates a controller's angular velocity into the controller's frame for SteamVR; the driver rotates it
  back into the tracking space.
- Buttons: the Touch-family paths (`a/x`, `b/y`, `menu`, `system`, `trigger`, `squeeze`, `thumbstick`,
  `thumbrest`, with `click`/`touch`/`value` as they exist). `trigger` and `squeeze` count as pressed from their
  analog value (on at 0.55, off at 0.45) unless the headset reports a click for them.

## Pinned settings

metalvrd composes a plain side-by-side frame, encodes ordinary 8-bit video, and returns ALVR's poses unchanged.
The ALVR settings that assume the SteamVR driver's processing are reset in `session.json` each time metalvrd
starts, and `start()` fails if ALVR's effective settings still disagree:

| setting | pinned to | why |
|---|---|---|
| `video.foveated_encoding` | off, or on with `--foveated-encoding` | the client undoes the squeeze, so the frame must have it exactly when the client is told it does (below) |
| `video.encoder_config.use_full_range` (+ server override) | limited range | with full range the client rescales the decoded picture, assuming its decoder treated the stream as limited |
| `use_10bit`, `hdr.enable_hdr`, `encoding_gamma` (+ server overrides) | 8-bit, off, 1.0 | not produced |
| `audio.game_audio`, `audio.microphone` | off | not implemented |
| `headset.controllers.steamvr_pipeline_frames` | 0 | otherwise controllers are predicted that much short of the head, for SteamVR to finish |
| `headset.controllers.left_controller_{position,rotation}_offset` | 0 | the defaults fit SteamVR's controller models; zero is the grip pose |
| `connection.client_discovery.auto_trust_clients` | off | only the headsets named to metalvrd may connect |

A config directory without a `session.json` is seeded with these and with HEVC as the preferred codec (ALVR's
default frame for a Quest is 4288 pixels wide, more macroblocks than H.264 allows below level 6). Everything else
is ALVR's default and can be changed in `session.json` while metalvrd is stopped (ALVR keeps the session in memory
and rewrites the file), or through ALVR's dashboard API from the same machine.

**The encoder must therefore produce video-range (16–235) 8-bit 4:2:0.**

### Foveated encoding

`--foveated-encoding` turns ALVR's foveated encoding on: each eye of the composed frame is squeezed toward its edges
before it is encoded, its center kept whole, and the client stretches it back, so the headset decodes fewer pixels
(a 2144x2336 eye is sent as 1280x1216 with ALVR's default shape). The shape is ALVR's own
`video.foveated_encoding` settings, read from the negotiated stream; metalvrd's squeeze is ALVR's server shader
ported (`daemon/foveation.h`, the compositor's `squeeze_fragment`), and checked against ALVR's client's stretch:
`tests/headset_protocol_check.cpp` (the formulas, both ways), `metalvrd --foveation-check` (the GPU's squeeze
against the formula) and the `native-foveated` test (a squeezed stream end to end). The focus is fixed at the
eye's center as ALVR shapes it; following the eye's gaze (a Steam Frame's eye tracking) would need a client that
sends the gaze and stretches around it.

## Not implemented

- Audio in either direction.
- Body, face and eye tracking. (Bare hands are passed on: the joints ALVR's client sends are OpenXR's
  `XR_EXT_hand_tracking`.)
- Changing the refresh rate: `XR_FB_display_refresh_rate` lists the headset's rates, and the stream stays at the
  one negotiated.
- AV1 (HEVC is sent instead).
- Gaze-following foveation, 10-bit, HDR, server-side color correction.
- ALVR's frame pacing hint (`alvr_duration_until_next_vsync`).
- Prediction on the server: poses are the client's.
- Trusting a headset while metalvrd runs: ALVR reads its client list once per process.

## What has been verified, and what has not

Verified on macOS arm64 without a headset:

- `scripts/build-alvr.sh` from a clean tree; `tests/alvr_driver_smoke.cpp` (start, defaults, the trusted list,
  stop, start again); metalvrd building and linking with the driver through CMake.
- The driver against ALVR's own `client_core` (the networking half of the stock client) running headless on the
  same machine: discovery, the handshake including the first "restarting" round, the negotiated size, rate,
  codec and range, `VIEWS_CONFIG`, head and controller poses and velocities, recentering, button and analog
  inputs, haptics, keyframe requests, decoder config delivery, frames arriving under the timestamps of the
  tracking packets they were tagged with, and the reconnect after a parameter-set change.
- Trust, with that client and metalvrd: without `--trust` the client was named once in twelve seconds of
  announcements and never streamed to; with `--trust` given its name plus `.local.` it connected. The dashboard
  API answered on 127.0.0.1 and refused connections on the machine's network address.
- End to end with that client: an OpenXR application (`xr_smoke`, 1000 frames at 72 Hz) through the runtime,
  metalvrd and VideoToolbox (HEVC, 4288×2336). The client received the encoder's parameter sets as its decoder
  config and 998 of the 1000 frames (the first two arrived before it had a decoder); what it received, from its
  first keyframe on, decodes with VideoToolbox on the Mac (992 of 992 frames) to the composed stereo picture.
- Statistics, read from ALVR's event stream during that run: application 9.6 ms, compositor 2.3 ms, encoder
  12.9 ms, network 0.2 ms on average, against metalvrd's own 2.3–2.5 ms of composition and 12.8–13.0 ms of
  encoding.

**Not verified — all of it needs a real headset:**

- That a headset's decoder accepts the stream and the client displays it. Nothing here has run on Android's
  decoder or been seen through lenses.
- Reprojection, latency and judder on the headset.
- Colors: the limited-range pin is derived from reading the client's shader, not from looking at a picture.
- The grip pose on real controllers, and the aim pose offset, which is an estimate.
- Button mapping from a real controller (the loopback client sent ALVR's button paths directly).
- UDP streaming and mDNS discovery (the loopback used TCP and broadcast, the only way two ALVR ends share a
  machine).
- How long the stock client takes to reconnect after the driver restarts ALVR.
- Adaptive bitrate (only the constant default was seen), and H.264.
- `RESTART_PENDING` and `SHUTDOWN_PENDING`, which need a dashboard.

Known rough edges inherited from ALVR: it reports some errors (its web server failing to bind, a Rust panic)
through a native alert dialog from a helper thread, and a failure to bind UDP 9943 or TCP 8082 is only visible in
its log. Trust is by announced hostname, which is ALVR's model: another device on the network that announces a
trusted headset's name is treated as that headset.
