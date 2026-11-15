## Summary

<!-- What changes and why. Link the issue it fixes, if any. -->

## Testing

<!-- Tick what you ran and note the results. A test that could not be run is worth saying too. -->

- [ ] Built for macOS and for Windows (`build/native`, `build/win64`)
- [ ] `scripts/test-native.sh` and `scripts/test-failures.sh`
- [ ] `scripts/test-wine.sh`, for changes to the runtime, the protocol or the server (say which Wine and Mullion)
- [ ] `scripts/test-openvr.sh`, for changes that OpenVR games reach through OpenComposite
- [ ] On a headset, for changes to the ALVR driver, encoding, timing or input (say which headset and client)

## Checklist

- [ ] The change is my own work, or open-source code whose license allows it here; nothing is taken from a closed-source product
- [ ] Changes to ALVR or OpenComposite are patches under `patches/`, not edits in `external/`
- [ ] New downloads are pinned to a commit
- [ ] Documentation is updated where behaviour, options or the protocol change
- [ ] No Wine prefixes, logs with personal paths, or other personal data
