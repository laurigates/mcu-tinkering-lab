# Security Policy

## Supported Versions

This is an active hobbyist / lab monorepo. Only the current `main` branch
of each firmware project receives security fixes. Pinned release tags
(`v*.*.*`) are not patched — re-flash from the latest `main` to pick up
fixes.

| Version | Supported |
|---------|-----------|
| `main`  | Yes       |
| Tagged releases | No (re-flash from `main`) |

## Reporting a Vulnerability

Please report vulnerabilities **privately** via GitHub Security Advisories:

https://github.com/laurigates/mcu-tinkering-lab/security/advisories/new

Do not file a public issue, open a PR with the fix, or discuss the
problem in public channels until a fix has landed on `main`.

When reporting, please include:

- The affected project (e.g. `packages/robocar/main`, `packages/networking/wireguard-ha`).
- The commit SHA or release tag you tested against.
- Reproduction steps or a proof-of-concept, where possible.
- Your assessment of impact (credential exposure, RCE on the device,
  unauthenticated OTA, etc.).

## Response Expectations

- **Acknowledgement**: within ~7 days of receipt.
- **Triage and remediation timeline**: communicated after triage; expect
  best-effort rather than enterprise SLA — this is a personal project.
- **Disclosure**: coordinated. Once a fix is on `main`, the advisory
  will be published with credit to the reporter (unless anonymity is
  requested).

## Scope

In scope:

- Firmware vulnerabilities in this repository's projects (memory safety,
  command injection, auth bypass, etc.).
- Leaked credentials or secrets in committed code or build artifacts.
- Insecure OTA update flows (downgrade, MITM, unsigned images).
- Insecure transport defaults (plaintext MQTT/HTTP where TLS is
  expected, weak TLS configuration).
- Web flasher (`docs/flasher/`) issues that could push malicious
  firmware to a user's device.

Out of scope:

- Third-party silicon errata (ESP32, STM32, NRF, etc.) — report to the
  vendor.
- Denial-of-service via radio interference (jamming, 2.4 GHz noise).
- Vulnerabilities in upstream dependencies (ESP-IDF, Bluepad32, vendored
  components) — report upstream, then open a tracking issue here once
  the upstream advisory is public.
- Physical attacks requiring direct hardware access (e.g. JTAG/UART
  probing of an unprotected dev board).
- Issues that require the user to flash an attacker-controlled binary
  themselves.
