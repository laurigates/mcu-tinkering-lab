# Architecture Decision Records

This directory contains Architecture Decision Records (ADRs) for the IT Troubleshooter project.

## Index

| ADR | Title | Status | Date |
|-----|-------|--------|------|
| [0001](0001-esp32s3-mcu-selection.md) | ESP32-S3 MCU Selection | Accepted | 2026-03-09 |
| [0002](0002-hid-boot-protocol-keyboard.md) | HID Boot Protocol Keyboard | Accepted | 2026-03-09 |
| [0003](0003-cdc-acm-to-ncm-phased.md) | CDC-ACM in Phase 1, CDC-NCM in Phase 4 | Accepted | 2026-03-09 |
| [0004](0004-caller-driven-status-led.md) | Caller-Driven Status LED (No Background Task) | Accepted | 2026-03-09 |
| [0005](0005-rmt-dma-disabled.md) | RMT DMA Disabled for WS2812 Driver | Accepted | 2026-03-09 |
| [0006](0006-external-uart-debug.md) | External USB-UART Adapter for Debug Monitoring | Accepted | 2026-03-09 |
| [0007](0007-core-assignment-strategy.md) | Core Assignment Strategy | Accepted | 2026-03-09 |
| [0008](0008-phased-delivery.md) | Phased Delivery Approach | Accepted | 2026-03-09 |

## Creating New ADRs

Name new ADRs: `{NNNN}-{kebab-case-title}.md`

Find next number:
```bash
ls docs/adrs/ | grep -E '^[0-9]{4}-' | sort -r | head -1 | cut -d'-' -f1
```

## ADR Status Values

- **Proposed** — Under discussion
- **Accepted** — Decision made and implemented
- **Deprecated** — No longer applies
- **Superseded** — Replaced by a later ADR (link to successor)
