# ADR-0005: RMT DMA Disabled for WS2812 Driver

**Status**: Accepted
**Date**: 2026-03-09
**Confidence**: 9/10

## Context

The ESP-IDF `led_strip` component drives WS2812 LEDs via RMT (Remote Control peripheral). On ESP32-S3, RMT can optionally use DMA for zero-copy transfers. TinyUSB also uses DMA for USB endpoint transfers. Both sharing DMA resources causes a conflict.

## Decision

Configure the LED strip driver with **DMA disabled**: `with_dma = false`, `mem_block_symbols = 48`.

## Rationale

Symptom observed in the xbox-switch-bridge project (same hardware): with `with_dma = true`, the RMT channel malfunctions after `tinyusb_driver_install()` is called — the LED output becomes corrupted or stops entirely. Root cause is DMA resource contention between TinyUSB's USB endpoint DMA and RMT DMA on ESP32-S3.

With `with_dma = false`, RMT uses its internal memory buffer instead of DMA. For a single WS2812 LED (24 bits = 48 RMT symbols), this is fully within the ESP32-S3's RMT memory capacity.

`mem_block_symbols = 48` is ESP32-S3 specific — generic ESP32-S3 boards use 64, but the Waveshare ESP32-S3-Zero requires 48 due to its memory block configuration.

## Evidence

- `components/status_led/status_led.c`: `strip_config.with_dma = false` and `rmt_config.mem_block_symbols = 48`
- Pattern confirmed by xbox-switch-bridge project (`../xbox-switch-bridge/components/status_led/`)
- Same hardware (Waveshare ESP32-S3-Zero), same constraint

## Consequences

- No DMA overhead for RMT — CPU handles RMT transfers (negligible for 1 LED)
- RMT and TinyUSB coexist without conflict
- `mem_block_symbols = 48` must be preserved if LED count is increased (verify capacity)
- If adding more WS2812 LEDs (e.g., a strip), revisit DMA isolation strategy
