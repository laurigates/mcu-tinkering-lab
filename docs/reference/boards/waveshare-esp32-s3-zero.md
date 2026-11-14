# Waveshare ESP32-S3-Zero

A 23.5 x 18 mm castellated ESP32-S3 board with one USB-C port wired straight to
the chip's native USB, no USB-UART bridge, a WS2812 RGB LED on GPIO21, and BOOT
and RESET buttons. Four projects here run on it because they emulate USB devices
over that port: [switch-usb-proxy](../../../packages/input-gaming/switch-usb-proxy/),
[xbox-switch-bridge](../../../packages/input-gaming/xbox-switch-bridge/),
[it-troubleshooter](../../../packages/networking/it-troubleshooter/) and
[facedancer-espdancer-fw](../../../packages/usb-tools/facedancer-espdancer-fw/).
The board summary is
[`board--waveshare-esp32-s3-zero.md`](../datasheets/board--waveshare-esp32-s3-zero.md),
the chip is [`mcu--esp32-s3.md`](../datasheets/mcu--esp32-s3.md), and the
chip-family page, which lists this board among other ESP32-S3 boards, is
[`esp32-s3.md`](esp32-s3.md).

## The fitted board

Identified from Waveshare's wiki and drawings, not from a fitted board: no board
is attached in this repository's sessions, and no project records a listing or
item number.

- **Name and variants:** Waveshare sells the ESP32-S3-Zero without pin headers
  and the ESP32-S3-Zero-M with them soldered: the same chip, with or without
  pin headers (wiki § Introduction).
- **Silkscreen** (top face, the wiki's top-view photo `ESP32-S3-Zero_02.jpg`,
  § Dimensions): `5V`, `GND`, `3V3`, `1` to `6` down the left edge, `TX`, `RX`,
  `13` to `7` down the right edge, `16`, `15`, `14` on three inner pads, and
  `BOOT` and `RESET` printed below the two buttons. The wiki's pinout drawing
  shows another artwork, with `B` and `R` above the buttons and a different
  component placement. The bottom face carries `ESP32-S3-Zero` and eight more
  numbered pads (pinout drawing, bottom view; 2D drawing
  `ESP32-S3-Zero-2D-size.jpg`, which is a bottom view).
- **Not to be confused with:** other small ESP32-S3 boards with castellated
  edges and one USB-C port, among them the ESP32-S3 SuperMini that ThinkPack's
  chatterbox and finderbox use. This page covers only the Waveshare board;
  check a board's silkscreen against the pin-mapping table below before wiring
  it from this page.

## Specifications

| Feature | Value | Source |
|---|---|---|
| Chip | ESP32-S3FH4R2, Xtensa LX7 dual core, up to 240 MHz | Schematic U2; wiki § Features |
| Flash | 4 MB in the chip package, quad SPI | ESP32-S3 datasheet v2.2 Table 1-1 |
| PSRAM | 2 MB in the chip package, quad SPI | Datasheet Table 1-1 |
| VDD_SPI | 3.3 V for this part | Datasheet Table 1-1 |
| Radio | 2.4 GHz Wi-Fi 802.11 b/g/n, Bluetooth 5 LE, on-board ceramic antenna | Wiki § Features; schematic J1 |
| Crystal | 40 MHz (X1) | Schematic |
| USB | USB-C to GPIO19/GPIO20 through 22 Ω (R7, R8); 5.1 kΩ on CC1 and CC2 (R4, R5) | Schematic |
| USB-UART bridge | None | Wiki § Hardware Description; wiki FAQ "onboard download circuit? No" |
| Regulator | ME6217C33M5G LDO, VBUS to 3V3, 800 mA maximum | Schematic U3; wiki § Interfaces item 3 |
| Status LED | WS2812B-0807 on GPIO21, powered from 3V3 | Schematic U1; wiki § Hardware Description |
| Buttons | BOOT (K2) on GPIO0, RESET (K1) on CHIP_PU, each with a 10 kΩ pull-up to 3V3 | Schematic |
| GPIOs on pads | 26: 15 on the castellated edges, 11 on inner and bottom pads | Schematic P15 and test-point pads; pinout drawing |
| Header row spacing | 15.24 mm | Wiki FAQ |
| Size | 23.50 x 18.00 mm, 2.54 mm pad pitch | 2D drawing (bottom view) |
| Ambient temperature | −40 to 85 °C | Datasheet Table 1-1 (ESP32-S3FH4R2 row); wiki FAQ |

The wiki's Features list says "24 × GPIO pins". The schematic brings 26 GPIOs to
pads; 24 matches if TX and RX (GPIO43, GPIO44) are left out of the count.

## Pin mapping

`Side` and `Pos` are the physical position of each castellated pad, viewing the
board component side up with the USB-C connector at the top: `L` and `R` are the
left and right edges, `Pos 1` is the pad nearest the USB-C connector. Waveshare
publishes no Eagle or KiCad board file for this board (wiki § Board Design Files
lists a schematic PDF, a 2D drawing JPEG and a STEP model), so the order comes
from two vendor documents that agree pad for pad:

- the pinout drawing on the wiki, `ESP32-S3-Zero-details-inter.jpg` (top view:
  `5V`, `GND`, `3V3(OUT)`, `GP1` to `GP6` on the left; `TX`, `RX`, `GP13` to
  `GP7` on the right), and
- the schematic's 18-pin connector P15, numbered 1 at `TX` and running
  clockwise in the top view to 18 at `VBUS`: pin 1 IO43 (through R6), 2 IO44,
  3 to 15 IO13 down to IO1, 16 3V3, 17 GND, 18 VBUS. That is the reverse of
  the repo's `board-dip-ccw` numbering
  ([parts README](../parts/README.md)), which counts counter-clockwise from
  `L 1`: P15 pin *k* is `board-dip-ccw` pin 19−*k*.

The top-view photo on the wiki (`ESP32-S3-Zero_02.jpg`, § Dimensions) shows
the same silkscreen order. GPIO functions are from the
ESP32-S3 datasheet v2.2: ADC and touch channels from Table 2-8, UART0 from
Table 2-4, strapping from Table 3-1. `tools/hardware/` reads this table
(ADR-021); keep it the only table in this file whose first column names a pin
and which has a column headed `GPIO`.

| Pin | GPIO | Side | Pos | Default Function | Alternate Functions |
|-----|------|------|-----|------------------|---------------------|
| 5V | — | L | 1 | VBUS: USB-C VBUS and the LDO input, no diode between them | 3.7–6 V in when powered from this pad |
| GND | — | L | 2 | Ground | — |
| 3V3 | — | L | 3 | 3.3 V out from the ME6217C33M5G | — |
| 1 | GPIO1 | L | 4 | GPIO | ADC1_CH0, TOUCH1 |
| 2 | GPIO2 | L | 5 | GPIO | ADC1_CH1, TOUCH2 |
| 3 | GPIO3 | L | 6 | GPIO, strapping: JTAG signal source, floating at reset | ADC1_CH2, TOUCH3 |
| 4 | GPIO4 | L | 7 | GPIO | ADC1_CH3, TOUCH4 |
| 5 | GPIO5 | L | 8 | GPIO | ADC1_CH4, TOUCH5 |
| 6 | GPIO6 | L | 9 | GPIO | ADC1_CH5, TOUCH6 |
| TX | GPIO43 | R | 1 | U0TXD, through R6 499 Ω | — |
| RX | GPIO44 | R | 2 | U0RXD | — |
| 13 | GPIO13 | R | 3 | GPIO | ADC2_CH2, TOUCH13 |
| 12 | GPIO12 | R | 4 | GPIO | ADC2_CH1, TOUCH12 |
| 11 | GPIO11 | R | 5 | GPIO | ADC2_CH0, TOUCH11 |
| 10 | GPIO10 | R | 6 | GPIO | ADC1_CH9, TOUCH10 |
| 9 | GPIO9 | R | 7 | GPIO | ADC1_CH8, TOUCH9 |
| 8 | GPIO8 | R | 8 | GPIO | ADC1_CH7, TOUCH8 |
| 7 | GPIO7 | R | 9 | GPIO | ADC1_CH6, TOUCH7 |

### Pads off the castellated edges

Eleven more GPIOs reach pads that are not on the edge rows: three round pads on
the top face beside the right edge (`16`, `15`, `14`, top to bottom), and a
column of eight on the bottom face (`45`, `42`, `41`, `40`, `39`, `38`, `18`,
`17`, top to bottom in the drawing's bottom view). Waveshare calls the top-face
ones pads "for soldering or using pogo pins" (wiki FAQ). They carry no `Side` or
`Pos` because they are not on a header.

| GPIO | Pad | Face | Function | Source |
|------|-----|------|----------|--------|
| GPIO14 | 14 | Top | ADC2_CH3, TOUCH14 | Schematic; datasheet Table 2-8 |
| GPIO15 | 15 | Top | ADC2_CH4, XTAL_32K_P | Schematic; Table 2-8 |
| GPIO16 | 16 | Top | ADC2_CH5, XTAL_32K_N | Schematic; Table 2-8 |
| GPIO17 | 17 | Bottom | ADC2_CH6; U1TXD on IO MUX | Schematic; Tables 2-8 and 2-4 |
| GPIO18 | 18 | Bottom | ADC2_CH7; U1RXD on IO MUX | Schematic; Tables 2-8 and 2-4 |
| GPIO38 | 38 | Bottom | GPIO | Schematic |
| GPIO39 | 39 | Bottom | MTCK (JTAG) | Schematic; Table 2-4 |
| GPIO40 | 40 | Bottom | MTDO (JTAG) | Schematic; Table 2-4 |
| GPIO41 | 41 | Bottom | MTDI (JTAG) | Schematic; Table 2-4 |
| GPIO42 | 42 | Bottom | MTMS (JTAG) | Schematic; Table 2-4 |
| GPIO45 | 45 | Bottom | strapping: VDD_SPI voltage, weak pull-down; leave low at reset (3.3 V flash) | Schematic; Tables 3-1 and 3-4 |

### On the board, not on a pad

| GPIO | Connected to | Source |
|------|--------------|--------|
| GPIO0 | BOOT button K2 to GND, 10 kΩ (R3) pull-up; strapping: boot mode | Schematic; datasheet Tables 3-1 and 3-3 |
| GPIO19 | USB D− through R7 22 Ω | Schematic |
| GPIO20 | USB D+ through R8 22 Ω | Schematic |
| GPIO21 | WS2812B-0807 DIN, 1 kΩ (R1) pull-up to 3V3 | Schematic; wiki § Hardware Description |
| GPIO26–GPIO32 | In-package flash and PSRAM SPI bus; do not use | Datasheet Table 2-14 and § 2.3.5 (Priority 4) |
| GPIO33–GPIO37 | Not connected (chip pins 38–42 marked no-connect) | Schematic |
| GPIO46 | Not connected; strapping: boot mode and ROM log, weak pull-down | Schematic; datasheet Table 3-1 |
| GPIO47, GPIO48 | Not connected (nets end in no-connect marks) | Schematic |

Waveshare's wiki says GPIO33–GPIO37 are "reserved for Octal PSRAM". The
ESP32-S3FH4R2's PSRAM is quad SPI (datasheet Table 1-1), and GPIO33–GPIO37 carry
flash or PSRAM only in 8-line mode (Table 2-14; § 2.3.5 lists them as free when
no 8-line memory is connected). On this board they are simply not wired out.

## Power

- **From USB-C:** VBUS feeds the ME6217C33M5G (schematic U3), whose 3.3 V output
  supplies the chip, the LED and the `3V3` pad. Waveshare rates the regulator at
  800 mA maximum (wiki § Interfaces item 3).
- **From the `5V` pad:** 3.7–6 V (wiki § Hardware Connection), at least 500 mA
  at 5 V (wiki FAQ).
- The `5V` pad is the VBUS net itself: the schematic shows no diode or switch
  between the USB-C VBUS pins and P15 pin 18. Supplying the pad while a USB
  cable is connected puts the two sources in parallel.

## Flashing

There is no USB-UART bridge and no auto-reset circuit, so the ROM bootloader is
reached through the USB-C port's USB-Serial-JTAG controller, which appears as
`/dev/ttyACM*` or `/dev/cu.usbmodem*`.

- **What Waveshare documents:** hold BOOT (GPIO0) while connecting the USB-C
  cable (wiki § Hardware Description and § Hardware Connection), or press BOOT
  and then RESET (wiki § Interfaces item 6). If flashing fails, hold BOOT while
  connecting, release it, and try again (wiki FAQ).
- **Why that works:** GPIO0 low at reset with GPIO46 low selects Joint Download
  Boot, which includes USB-Serial-JTAG download (datasheet Table 3-3). GPIO46
  is unconnected here and has a weak pull-down (Table 3-1), so BOOT alone
  decides the mode.
- **What the projects do:** xbox-switch-bridge's
  [WIRING.md](../../../packages/input-gaming/xbox-switch-bridge/WIRING.md)
  § "Flashing the ESP32-S3-Zero" gives hold BOOT, press RESET, release both,
  then flash on `/dev/ttyACM0`. Once a TinyUSB image runs, USB-Serial-JTAG
  is gone from the bus (it-troubleshooter
  [ADR-0006](../../../packages/networking/it-troubleshooter/docs/adrs/0006-external-uart-debug.md),
  xbox-switch-bridge
  [ADR-0007](../../../packages/input-gaming/xbox-switch-bridge/docs/adrs/0007-wifi-udp-log-broadcasting.md)),
  so a re-flash starts from the BOOT entry, and logs go to a USB-UART adapter
  on GPIO43/GPIO44 or, in xbox-switch-bridge's production build, over Wi-Fi UDP.
- **Over UART:** a USB-UART adapter on the `TX`/`RX` pads can flash too, but
  neither CHIP_PU (EN) nor GPIO0 reaches a pad (schematic P15), so an adapter's
  RTS/DTR cannot reset the board into the bootloader: enter download mode with
  the buttons first, as switch-usb-proxy's `uart-flash` recipe comment says
  ([justfile](../../../packages/input-gaming/switch-usb-proxy/justfile)).

## ESP-IDF configuration

The memory settings follow the chip, not Waveshare's "Octal PSRAM" wording.
These lines are from xbox-switch-bridge's `sdkconfig.defaults` (lines 4, 35
and 40–42); it-troubleshooter and facedancer-espdancer-fw set the same:

```ini
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_QUAD=y
CONFIG_SPIRAM_SPEED_80M=y
```

## Known issues and quirks

- **The LED runs below its datasheet supply.** The WS2812B-0807 is powered from
  3V3 (schematic U1 pin 1), and Waveshare's own LED datasheet gives a chip
  operating voltage of 3.5–5.5 V (XL-0807RGBC-WS2812B, Electrical
  characteristics). Three projects here drive it from GPIO21 regardless.
- **GPIO43 has 499 Ω in series** (R6) between the chip and the `TX` pad. It does
  not matter for a UART; it limits what the pad can drive as a plain output.
- **The wiki's OLED example is wrong for this chip.** Its FAQ suggests SDA on
  GPIO21 and SCL on GPIO22. GPIO21 drives the LED, and the ESP32-S3 has no
  GPIO22 (datasheet Table 2-4 runs GPIO21 then GPIO26).
- **Keep the antenna clear.** Waveshare asks that no PCB, metal or plastic part
  cover the ceramic antenna on a carrier board (wiki § Hardware Description).
- **USB-Serial-JTAG and USB-OTG share the one port.** See Flashing above.

## In this repository

All four projects name this board in their `flasher.json` (`"board":
"Waveshare ESP32-S3-Zero"`). None has a `hardware.toml`, so `tools/hardware`
does not read this page for them yet.

| Project | GPIOs used | Where it says so |
|---|---|---|
| [switch-usb-proxy](../../../packages/input-gaming/switch-usb-proxy/) | GPIO43 TX, GPIO44 RX (UART1, 921600 baud, to the PC); GPIO19/GPIO20 USB to the Switch | `main/main.c` lines 41–44; `CLAUDE.md` § Hardware Connections |
| [xbox-switch-bridge](../../../packages/input-gaming/xbox-switch-bridge/) | GPIO21 WS2812; GPIO43/GPIO44 debug UART; GPIO19/GPIO20 USB; BOOT and RESET | `components/status_led/status_led.c` line 18; `WIRING.md` § Waveshare ESP32-S3-Zero Pinout |
| [it-troubleshooter](../../../packages/networking/it-troubleshooter/) | GPIO21 WS2812; GPIO43/GPIO44 debug UART at 115200 baud | `components/status_led/status_led.c` line 16; `docs/adrs/0006-external-uart-debug.md` |
| [facedancer-espdancer-fw](../../../packages/usb-tools/facedancer-espdancer-fw/) | GPIO21 WS2812; GPIO43/GPIO44 debug UART in the `debug-uart` build | `components/status_led/status_led.c` line 23; `README.md` § Hardware required |

Every GPIO the projects use matches the vendor sources above. Two project
statements disagree with them and are tracked outside this page:
xbox-switch-bridge's `flash-uart` recipe asks for RTS wired to EN and DTR to
GPIO0, which do not reach a pad, and its `WIRING.md` calls the BOOT entry "DFU
mode" where Waveshare and the datasheet say download mode.

## Source

- **Pin order, pad functions and on-board wiring:** Waveshare's schematic
  `ESP32-S3-Zero-Sch.pdf` (one page, read 2026-11-14) and the wiki's pinout
  drawing `ESP32-S3-Zero-details-inter.jpg`, which agree pad for pad.
- **Top-face silkscreen:** the wiki's top-view photo `ESP32-S3-Zero_02.jpg`
  (§ Dimensions).
- **Dimensions, pad pitch and bottom-face pads:** the wiki's
  `ESP32-S3-Zero-2D-size.jpg` (§ Board Design Files), a bottom view.
- **Power, flashing and variants:** the Waveshare wiki page, revision
  `oldid=111000`.
- **Chip facts:** Espressif ESP32-S3 Series Datasheet v2.2: Tables 1-1, 2-4,
  2-8, 2-14, 3-1, 3-3, 3-4 and § 2.3.5.
- **LED supply range:** Waveshare's copy of the XL-0807RGBC-WS2812B datasheet.
- **Use in this repository:** the four projects' sources, cited by file above.

**Not checked on a fitted board:** none of this page. The silkscreen, the pad
order, the bottom-face pad positions, the R6 resistor and the missing VBUS diode
are all read from Waveshare's documents.

## Check on the physical board

- [ ] The edge silkscreen reads `5V`, `GND`, `3V3`, `1`–`6` down the left and
  `TX`, `RX`, `13`–`7` down the right, USB-C at the top, component side up.
- [ ] Which button artwork the fitted board carries: `BOOT` and `RESET` below
  the buttons (wiki photo `ESP32-S3-Zero_02.jpg`) or `B` and `R` above them
  (wiki pinout drawing).
- [ ] Continuity: each edge pad reaches the GPIO the table gives (a pad-by-pad
  check with a short test image toggling each GPIO also settles it).
- [ ] The top-face pads `16`, `15`, `14` and the bottom-face pads `45`, `42`,
  `41`, `40`, `39`, `38`, `18`, `17` are where the drawing puts them and reach
  those GPIOs.
- [ ] No diode between USB-C VBUS and the `5V` pad (continuity with no USB
  cable connected).
- [ ] R6 (499 Ω) sits between GPIO43 and the `TX` pad (measure pad to chip).
- [ ] The chip marking reads ESP32-S3FH4R2, and `esptool.py flash_id` reports
  4 MB.
- [ ] Holding BOOT while plugging in the cable enumerates the ROM's
  USB-Serial-JTAG port without pressing RESET.
- [ ] The WS2812 lights at full white from 3V3 without colour error.

## References

- [Waveshare wiki: ESP32-S3-Zero](https://www.waveshare.com/wiki/ESP32-S3-Zero)
- [Schematic PDF](https://files.waveshare.com/wiki/ESP32-S3-Zero/ESP32-S3-Zero-Sch.pdf)
- [Pinout drawing](https://www.waveshare.com/w/upload/8/87/ESP32-S3-Zero-details-inter.jpg)
- [Top-view photo with dimensions](https://www.waveshare.com/w/upload/e/e0/ESP32-S3-Zero_02.jpg)
- [2D dimension drawing, bottom view](https://files.waveshare.com/wiki/ESP32-S3-Zero/ESP32-S3-Zero-2D-size.jpg)
- [3D model (STEP)](https://files.waveshare.com/wiki/ESP32-S3-Zero/ESP32-S3-Zero_v2-138eb3753cf14afa3252058c380ca416.stp)
- [ESP32-S3 Series Datasheet (Espressif)](https://documentation.espressif.com/esp32-s3_datasheet_en.pdf)
- [XL-0807RGBC-WS2812B LED datasheet (Waveshare copy)](https://files.waveshare.com/wiki/common/XL-0807RGBC-WS2812B.pdf)
