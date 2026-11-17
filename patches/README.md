# Patches

MetalVR does not vendor the projects it builds against. Their sources are fetched into `external/` by the build
scripts, and the few changes MetalVR needs are kept here and applied by those scripts.

A patch is a change to someone else's work and carries that work's license, not MetalVR's:

| Directory | Applies to | License of the patches |
|---|---|---|
| `alvr/` | [ALVR](https://github.com/alvr-org/ALVR) v20.14.1 | MIT, as ALVR |
| `opencomposite/` | [OpenComposite](https://gitlab.com/znixian/OpenOVR) | GPL-3.0, as OpenComposite |
| `libsurvive/` | [libsurvive](https://github.com/cntools/libsurvive) | MIT, as libsurvive |

ALVR's `server_core` is linked into `metalvrd`. OpenComposite is built as its own `openvr_api.dll` and is never
linked into MetalVR.
