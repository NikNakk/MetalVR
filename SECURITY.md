# Security policy

## Supported versions

MetalVR has no releases yet. Security fixes go into `main`; build from the latest commit.

## Reporting a vulnerability

Report vulnerabilities privately through GitHub: on this repository's
**Security** tab, choose **Report a vulnerability**. Do not open a public
issue, pull request or discussion for a vulnerability.

Include what you can of:

- the MetalVR commit you built (`git rev-parse --short HEAD`), and whether
  `metalvrd` was built with the ALVR driver;
- the macOS version and Mac model, and for the Wine path the Wine and Mullion
  versions;
- the steps or program that trigger it, and a crash log if there is one;
- what an attacker could achieve, and from where: another process on the Mac,
  a Windows program in the prefix, or a machine on the network.

Do not send passwords, private keys, personal data, whole Wine prefixes or
large archives.

You will get a reply within 7 days. Once the report is confirmed, a fix is
prepared privately and released, and the advisory is published with credit to
you unless you would rather stay anonymous.

## Scope

MetalVR runs with the user's own privileges, beside the applications that use
it. In scope:

- **The server's protocol port.** `metalvrd` accepts OpenXR runtimes on the
  loopback interface only and does not authenticate them: any process of any
  user on the same Mac can connect, read poses and submit frames. That much is
  by design. A way to reach the port from another machine, or to make a
  malformed request corrupt memory, crash the server or read another
  process's images, is a vulnerability.
- **Headset trust.** A headset streams only if it was named with `--trust`.
  Anything that lets an unnamed device receive the stream or send input is a
  vulnerability. Trust is by the name a device announces, as in ALVR: a device
  on the same network that announces a trusted headset's name is treated as
  that headset, and MetalVR adds no encryption to the stream. That is a known
  limit of the ALVR transport; run it on a network you trust.
- **ALVR's control API.** MetalVR patches it to listen on the loopback
  interface only (`patches/alvr`). Reaching it from the network is a
  vulnerability.
- **The runtime in an application's process**: memory-safety bugs in
  `metalvr_openxr.dll` or `libmetalvr_openxr.dylib` that an application's
  input, or a server's answer, can trigger.
- **Shared images.** A swapchain image is shared by name so the server can
  open it: a Mullion shared texture under Wine, a global IOSurface natively.
  Another process on the same Mac that learns or guesses the name can open
  it too; that is how the sharing works today, and a known limit. A way to
  write through it into the application's or the server's memory is a
  vulnerability.
- **The build and install scripts**: a dependency fetched without its pin
  being checked, or `scripts/install-prefix.sh` writing outside the prefix it
  was given.

Out of scope, and better reported where they belong (tell us as well if
MetalVR is affected):

- **Wine**, which is not a sandbox: a Windows program can do whatever a Mac
  program run by the same user can. [WineHQ](https://www.winehq.org/), or
  [metalsharp/MetalSharp-Wine](https://github.com/metalsharp/MetalSharp-Wine/security)
  for its own changes.
- **Mullion**: [dttdrv/mullion](https://github.com/dttdrv/mullion).
- **ALVR itself**, its protocol and its headset client:
  [alvr-org/ALVR](https://github.com/alvr-org/ALVR).
- **OpenComposite**: [its repository](https://gitlab.com/znixian/OpenOVR).
- **The Khronos OpenXR loader**:
  [KhronosGroup/OpenXR-SDK-Source](https://github.com/KhronosGroup/OpenXR-SDK-Source).
