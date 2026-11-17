# Oculus Rift CV1 with MetalSharp

The fastest path from a MetalVR release to a game in a Rift CV1, with the `metalvr-rift` script that comes in the
release (`bin/metalvr-rift`; in this repository, `scripts/metalsharp/metalvr-rift`). How the Rift driver works is in
[RIFT.md](RIFT.md), and SteamVR's side in [STEAMVR.md](STEAMVR.md).

The same steps work for a **Rift S** (its DisplayPort cable into the Mac's own DisplayPort, or USB-C/Thunderbolt to
DisplayPort; untried on a Mac) and, with `metalvr-lighthouse` in place of `metalvr-rift`, for an **HTC Vive, Vive
Pro, Vive Pro 2 or Valve Index**: the script finds which headset is on USB. What works on each and what does not yet
is in [HEADSETS.md](HEADSETS.md).

## What you need

- An Apple silicon Mac with an **HDMI port of its own**. Through a USB-C hub or adapter, macOS may read the headset
  and still never turn its display on.
- An Oculus Rift CV1: its HDMI in the Mac's HDMI port, its USB in a USB 3 port. Rift Sensors are optional: with
  them the headset and the Touch controllers are tracked in position, without them in rotation only.
- [MetalSharp](https://github.com/metalsharp/MetalSharp) with its Windows Steam, and SteamVR installed in that Steam
  for SteamVR games.
- A MetalSharp-Wine runtime whose Mullion is
  [aaf2tbz/mullion@998c32a6](https://github.com/aaf2tbz/mullion/commit/998c32a61095985e7034c7b56553d79aa81c7193) or
  later (MetalSharp-Wine's `src/mullion`). SteamVR's compositor and its games share Direct3D 11 buffers between
  processes, which that Mullion does with `MULLION_METALVR=1`; `metalvr-rift setup` sets it for the Steam prefix
  ([STEAMVR.md](STEAMVR.md)). With an older Mullion, SteamVR and its games show black in the headset.

Oculus's own software is not used, and none is needed.

## Setting up (once)

Download `metalvr-VERSION-macos-arm64.tar.gz` from the [releases](https://github.com/metalsharp/MetalVR/releases),
then in Terminal:

```sh
cd ~/Downloads
tar -xzf metalvr-*-macos-arm64.tar.gz
metalvr-*-macos-arm64/bin/metalvr-rift install
```

`install` copies MetalVR to `~/.metalsharp/metalvr-runtime`, where MetalSharp looks for it, and makes MetalVR the
Steam prefix's OpenXR runtime. From then on the script is `~/.metalsharp/metalvr-runtime/bin/metalvr-rift`:

```sh
alias metalvr-rift=~/.metalsharp/metalvr-runtime/bin/metalvr-rift   # or add it to ~/.zshrc
```

**Let macOS show the headset.** macOS marks the CV1's display a head-mounted display and keeps it off the desktop.
This writes an override in `/Library/Displays` that lifts that, and asks for your password:

```sh
metalvr-rift display
```

Unplug the headset's HDMI and plug it in again afterwards. `metalvr-rift display --undo` takes the override out. The
first time the headset's display comes on, macOS may make it the main display: in System Settings, Displays, move the
menu bar back to your monitor. macOS remembers that.

**Set SteamVR up.** In MetalSharp's Steam, install SteamVR, start it once and quit it (it writes the file that lists
its drivers), then:

```sh
metalvr-rift setup
```

That registers MetalVR's SteamVR driver, sets `MULLION_METALVR=1` for the Steam prefix, keeps the Steam client itself
out of VR (under Wine it hangs once SteamVR starts), and turns off SteamVR's dashboard desktop view and fade to a flat color, which don't work under Wine. Quit
Steam afterwards so that it starts again with these settings.

## Playing

```sh
metalvr-rift steamvr          # the Rift server, then SteamVR
metalvr-rift game 450390      # the Rift server, then a Steam game's VR entry (its id is in its store address)
```

Each starts the Rift server first if it isn't running, and Steam through MetalSharp if it isn't open (MetalSharp
has to be open). The headset shows MetalVR's home, a starfield, until a game draws.

- Put the headset where the sensors can see it once: each sensor works out where it stands from it. Putting the
  headset on, or the remote's Oculus button, re-centers the view.
- Pick each Touch controller up once: a controller lying asleep is not tracked until it wakes.

| | |
|---|---|
| `metalvr-rift start` | start only the Rift server (its log: `~/.metalsharp/logs/rift-server.log`) |
| `metalvr-rift stop` | stop it; the headset's display turns off |
| `metalvr-rift status` | what is installed, set up, plugged in and running |

OpenXR games don't need SteamVR: with the server running, start them from MetalSharp as usual and they use MetalVR
directly.

## When it doesn't work

- **The headset stays dark.** `metalvr-rift status`: the display override, the headset on USB, the server. The log
  says `rift: headset on` once the headset answers and `rift: drawing on the headset's display` once macOS shows its
  display. If only the first appears, check the HDMI is in the Mac's own port and run `metalvr-rift display`.
- **SteamVR or the game is black in the headset.** Run `metalvr-rift setup` again (it sets `MULLION_METALVR=1`) and
  restart Steam, and check the runtime's Mullion is new enough (see What you need).
- **SteamVR shows no headset.** Start the server before SteamVR; [STEAMVR.md](STEAMVR.md) lists the driver's
  messages.
- **Steam's windows load forever after SteamVR starts.** `metalvr-rift setup` was not run, or Steam was not restarted
  after it.
