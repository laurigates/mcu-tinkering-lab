# RP2350 (Cortex-M33 / Hazard3)

**Used in:** Seeed Studio XIAO RP2350 (balancebot)

## Key Specs

| Parameter | Value |
|-----------|-------|
| Cores | Dual Arm Cortex-M33 (FPU, DSP, TrustZone) **or** dual RISC-V Hazard3 — architecture selected at boot |
| Clock | 150 MHz |
| SRAM | 520 KB in 10 independent banks |
| OTP | 8 KB one-time-programmable storage (keys, config) |
| Flash/PSRAM | External QSPI (XIP), up to 16 MB + 16 MB on a second chip select; no internal flash on RP2350A/B |
| PIO | 12 programmable I/O state machines (3 blocks) |
| Peripherals | 2× UART, 2× SPI, 2× I2C, 24× PWM channels, HSTX |
| USB | USB 1.1 controller + PHY, host and device |
| Security | Secure boot (signed images), Arm TrustZone, hardware SHA-256, glitch/fault-injection detectors, per-peripheral security domains |
| GPIO | 5 V-tolerant (powered), 3.3 V-failsafe (unpowered) |
| Power | On-chip switched-mode core supply; low-quiescent LDO mode for sleep |

## Family Variants

| Product | Package | Internal flash | GPIO | ADC inputs |
|---------|---------|----------------|------|------------|
| RP2350A | QFN-60 (7×7 mm) | none | 30 | 4 |
| RP2350B | QFN-80 (10×10 mm) | none | 48 | 8 |
| RP2354A | QFN-60 | 2 MB | 30 | 4 |
| RP2354B | QFN-80 | 2 MB | 48 | 8 |

The XIAO RP2350 uses the **RP2350A** with a 2 MB external QSPI flash.

## Datasheets & References

- **RP2350 datasheet PDF (Raspberry Pi):** <https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf>
- **Hardware design guide PDF:** <https://datasheets.raspberrypi.com/rp2350/hardware-design-with-rp2350.pdf>
- **Product documentation:** <https://www.raspberrypi.com/documentation/microcontrollers/silicon.html>
- **Pico C/C++ SDK book:** <https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf>
- **pico-sdk (GitHub):** <https://github.com/raspberrypi/pico-sdk>
