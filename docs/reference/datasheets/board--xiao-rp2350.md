# Seeed Studio XIAO RP2350

**Role in project:** Balancebot (self-balancing robot controller)

## Key Specs

| Parameter | Value |
|-----------|-------|
| MCU | RP2350A (dual Cortex-M33 @ 150 MHz with FPU, or dual RISC-V Hazard3) |
| SRAM | 520 KB (10 banks) |
| Flash | 2 MB external QSPI (XIP); no PSRAM |
| USB | Type-C, USB 1.1 host/device (native, no bridge chip) |
| GPIO | 19 multifunction (11 side headers + 8 bottom castellated pads) |
| ADC | 3 channels, 12-bit (A0–A2) + battery sense |
| Interfaces | 2× I2C, 2× UART, 2× SPI, PWM on all pins |
| PIO | 12 state machines (3 blocks) |
| LEDs | User (GPIO25, active LOW), power, WS2812 RGB (data GPIO22, power GPIO23) |
| Buttons | RESET ("B" on bottom), BOOT (BOOTSEL) |
| Battery | LiPo charge management; voltage sense on GPIO29/A3 via /2 divider (enable: GPIO19 high) |
| Sleep current | 27 µA (3.7 V battery: 57 µA; 5 V: 205 µA) |
| Operating temp | -20°C ~ 70°C |
| Dimensions | 21 × 17.8 mm |

## Pin Notes

- Default I2C1: SDA=GPIO6 (D4), SCL=GPIO7 (D5); default UART0: TX=GPIO0 (D6), RX=GPIO1 (D7)
- Bottom pads D11–D18 carry the second UART/SPI/I2C sets
- Pico SDK board: `PICO_BOARD=seeed_xiao_rp2350` — note the SDK board header
  claims 4 MB flash; the board ships with 2 MB
- BOOTSEL: hold BOOT while plugging in (or hold BOOT, tap RESET) → `RP2350` UF2 drive

## Silicon Errata (A2 stepping, as shipped)

- **RP2350-E9**: internal GPIO pull-downs cannot hold an undriven input low
  — a floating input latches at ~2.2 V (reads high). Use external pull-downs
  (≤ 8.2 kΩ) or design so inputs are always actively driven. Pull-ups work
  normally.
- **RP2350-E10**: UF2 downloads carry a picotool workaround block that
  **erases the last flash block** on every flash. Don't store persistent
  data in the last 4 KB sector.
- Full list: RP2350 datasheet, Appendix E (fixed in the A3 stepping).

## Datasheets & References

- **RP2350 datasheet PDF (Raspberry Pi):** <https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf>
- **Board schematic PDF (Seeed):** <https://files.seeedstudio.com/wiki/XIAO-RP2350/res/Seeed-Studio-XIAO-RP2350-v1.0.pdf>
- **Pinout sheet (XLSX):** <https://files.seeedstudio.com/wiki/XIAO-RP2350/res/XIAO-RP2350-pinout-sheet.xlsx>
- **Schematic + PCB sources (ZIP):** <https://files.seeedstudio.com/wiki/XIAO-RP2350/res/XIAO_RP2350_v1.0_SCH&PCB_240626.zip>
- **Wiki:** <https://wiki.seeedstudio.com/getting-started-xiao-rp2350/>
- **OSHW documentation (GitHub):** <https://github.com/Seeed-Studio/OSHW-XIAO-Series/blob/main/document/SeeedStudio_XIAO_RP2350/XIAO-RP2350.md>
- **Product page:** <https://www.seeedstudio.com/Seeed-XIAO-RP2350-p-5944.html>
- **Board reference (this repo):** [../boards/xiao-rp2350.md](../boards/xiao-rp2350.md)
