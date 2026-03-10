# Testing

## Firmware Testing Approach

This is embedded firmware — standard unit testing is limited. Verification is primarily done via:

1. **Build verification** — `just build` must succeed with zero warnings
2. **Hardware testing** — Flash to Waveshare ESP32-S3-Zero, observe LED states and USB enumeration
3. **CDC loopback** — Connect to CDC serial port and verify echo behavior
4. **HID injection** — Open a text editor on the host, verify typed characters match expected output

## Static Analysis

- `cppcheck` for C code analysis (run via `make lint` at repo root)
- `clang-format` for formatting (Google style, 4-space indent, 100-char limit)

## Integration Checklist (before PR)

- [ ] `just build` succeeds cleanly
- [ ] Device enumerates as composite USB (HID + CDC visible in system USB info)
- [ ] Status LED cycles through BOOT → USB_READY correctly
- [ ] HID keyboard types demo string correctly
- [ ] CDC echo works bidirectionally
- [ ] Monitor output (GPIO43/44 UART) shows no errors or assertion failures
