# Architecture Decision Records

ADRs for facedancer-espdancer-fw. Numbered, immutable once Accepted. Each
carries a Confidence level; low-confidence decisions call out the experiment
that would raise it.

| # | Title | Confidence |
|---|-------|------------|
| 0001 | ESP32-S3-Zero — reuse xbox-switch-bridge provenance | Very High |
| 0002 | Raw DCD relay, not the TinyUSB HID class driver | High |
| 0003 | WiFi SoftAP as the control channel (not UART) | High |
| 0004 | Three sdkconfig build variants | Very High |
| 0005 | Single-writer relay pump (no mutex) | High |
| 0006 | Vendor a DCD-only TinyUSB port (bypass `usbd`) — resolves 0002 | High |

Format: Context / Decision / Consequences / Alternatives Considered.