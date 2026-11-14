# ESP32-S3 SuperMini

A thumb-sized ESP32-S3 board with a USB-C connector wired to the chip's native
USB, sold under the name "ESP32-S3 SuperMini" by several makers. It is the
board of the ThinkPack chatterbox and finderbox (their `flasher.json` `board`
field), and boombox, glowbug and the `thinkpack-rc522` component name it too.
The silicon is described in [`mcu--esp32-s3.md`](../datasheets/mcu--esp32-s3.md);
this page holds only what is particular to the board.

## The fitted board

"ESP32-S3 SuperMini" is a listing name, not one product. No single vendor owns
it: CircuitPython files it under maker "Generic" and says "It is available at a
number of sites without a listed manufacturer"
([circuitpython.org](https://circuitpython.org/board/generic_esp32s3_supermini/)).
**Which variant Lauri owns is not known.** No listing link or photo of the
fitted boards is in the repository, and no project records a chip marking or an
`esptool.py chip_id` read.

The variants a primary source describes:

| Variant | Distinguishing parts | Chip named | Described by |
|---|---|---|---|
| TENSTAR ROBOT ESP32-S3 SuperMini, with charger | `B+`/`B-` battery pads and a `BOOST` jumper on the underside, a `BAT` charge LED beside the USB-C connector, 20 underside pads | ESP32-S3FH4R2; the chip in the drawing's photo reads `FH4R2` at that image's resolution | The maker's pinout drawing, TENSTAR ROBOT watermark ([esphome-devices `Pinouts.webp`](https://github.com/esphome/esphome-devices/blob/ff0009586b5666ab149551faaaeab1900ebe3a5c/src/docs/devices/TENSTAR-ROBOT-ESP32-S3-SuperMini/Pinouts.webp); the same image is the "board datasheet" espboards.dev links, [alicdn](https://ae01.alicdn.com/kf/S47f3120f532e4c78811801f40c8208aaD.png)); the ESPHome device page, which says it "also has a battery charging circuit" |
| Generic ESP32-S3 SuperMini | 15 GPIO on the castellated edges and 20 on underside pads; no charger mentioned. A unit marked `HW747 V0.0.2` drives its RGB LED in RGB rather than GRB order | ESP32-S3FH4R2 | CircuitPython's board page and board definition (`adafruit/circuitpython` at `519f3530`, `ports/espressif/boards/generic_esp32s3_supermini/`) |
| "A random ESP32-S3 Super Mini product" | A plain LED and an addressable RGB LED, both on GPIO48 | ESP32-S3FH4R2 | [UnsignedArduino/ESP32-S3-Super-Mini-Test](https://github.com/UnsignedArduino/ESP32-S3-Super-Mini-Test) `README.md` lines 3 and 11 (at `b9ba4904`) |

Every source found names the ESP32-S3FH4R2. No source was found for a
SuperMini with octal PSRAM (an R8 or R16V part).

Do not confuse it with the [ESP32-C3 Super Mini](esp32-c3-super-mini.md), a
RISC-V board of nearly the same outline, or with the Waveshare ESP32-S3-Zero,
which carries the same ESP32-S3FH4R2 in a different layout
([`board--waveshare-esp32-s3-zero.md`](../datasheets/board--waveshare-esp32-s3-zero.md)).

## Specifications

| Feature | Value | Source |
|---|---|---|
| Chip | ESP32-S3FH4R2: dual-core Xtensa LX7 | Tenstar drawing, "Chip model: ESP32S3FH4R2"; CircuitPython board page; [ESP32-S3 datasheet v2.2](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf) p. 3 for the core |
| Flash | 4 MB, in the chip package, quad SPI | Datasheet Table 1-1 (p. 13); CircuitPython `mpconfigboard.mk` lines 8 and 10 (`qio`, `4MB`); ESPHome YAML `flash_size: 4MB` |
| PSRAM | 2 MB, in the chip package, quad SPI | Datasheet Table 1-1 (p. 13); CircuitPython `mpconfigboard.mk` lines 12 and 14 (`qio`, `2MB`); ESPHome YAML `psram: mode: quad`; UnsignedArduino `boards/esp32-s3-supermini.json` line 6 (`qio_qspi`) |
| USB | USB-C to the chip's USB pins, GPIO19 (D−) and GPIO20 (D+), which are not broken out | Datasheet Table 2-8 (p. 24); neither pin appears in the Tenstar drawing or in CircuitPython `pins.c` |
| Buttons | BOOT on GPIO0, RESET | CircuitPython `pins.c` lines 12–13; CircuitPython board page; Tenstar drawing labels both |
| Addressable RGB LED | One, data on GPIO48, GRB order | CircuitPython `mpconfigboard.h` line 14; ESPHome `index.md` line 26 and YAML `pin: GPIO48`, `rgb_order: GRB`; Tenstar drawing ("WS2818 RGB LED, GP48, DIN"); UnsignedArduino `src/main.cpp` line 5. The part is called WS2812 (CircuitPython), WS2812B (ESPHome) and WS2818 (Tenstar, UnsignedArduino) |
| Plain LED | Also on GPIO48. Blue according to Tenstar ("Onboard RGB, blue LED: common GPIO48 pin"), red according to espboards.dev | Tenstar drawing; UnsignedArduino `README.md` line 11; [espboards.dev](https://www.espboards.dev/esp32/esp32-s3-super-mini/) |
| Charge LED and charger (Tenstar variant) | `BAT` LED, no GPIO. Battery on the `B+`/`B-` pads; closing `BOOST` raises the charge current from 100 mA to 300 mA, "only do this if your battery is larger than 500mAh" | Tenstar drawing; ESPHome `index.md` line 19 |
| Power pins | `5V`, `GND`, `3V3` (labelled "3V3(OUT)") | Tenstar drawing |
| Antenna | On the board, at the end opposite the USB-C connector | Tenstar drawing photo ("WIFI Antenna") |
| Dimensions | 23.5 × 18 mm; the same drawing's text says 22.52 × 18 mm | Tenstar drawing photo dimension arrows and parameter list; espboards.dev says 23.5 × 18 mm |

## Pin mapping

Silkscreen names are the Tenstar drawing's. With the USB-C connector at the
top and the components facing you, the left edge runs `TX`, `RX`, `1`–`7` from
the top and the right edge runs `5V`, `GND`, `3V3`, `13`–`8` from the top. The
Tenstar drawing's top view and the comments in CircuitPython `pins.c` (lines
17, 54 and 79–80) give that order; neither is a board file, so it is not
written as `Side`/`Pos` here. The 20 underside pads are listed in GPIO order.

Analog and touch channels are from the datasheet's Table 2-8 (p. 24), JTAG and
UART0 functions from Table 2-4 (p. 21), strapping functions from §3 and Table
3-1 (p. 32).

| Pin | GPIO | Location | Default function | Alternate functions | Source |
|---|---|---|---|---|---|
| TX | GPIO43 | Left edge | UART0 TX (U0TXD) | — | Tenstar; CircuitPython `pins.c` 18–20 |
| RX | GPIO44 | Left edge | UART0 RX (U0RXD) | — | Tenstar; CircuitPython `pins.c` 22–24 |
| 1 | GPIO1 | Left edge | GPIO | ADC1_CH0, TOUCH1 | Tenstar; CircuitPython `pins.c` 26–28 |
| 2 | GPIO2 | Left edge | GPIO | ADC1_CH1, TOUCH2 | Tenstar; CircuitPython `pins.c` 30–32 |
| 3 | GPIO3 | Left edge | GPIO; strapping (JTAG signal source) | ADC1_CH2, TOUCH3 | Tenstar; CircuitPython `pins.c` 34–36 |
| 4 | GPIO4 | Left edge | GPIO | ADC1_CH3, TOUCH4 | Tenstar; CircuitPython `pins.c` 38–40 |
| 5 | GPIO5 | Left edge | GPIO | ADC1_CH4, TOUCH5 | Tenstar; CircuitPython `pins.c` 42–44 |
| 6 | GPIO6 | Left edge | GPIO | ADC1_CH5, TOUCH6 | Tenstar; CircuitPython `pins.c` 46–48 |
| 7 | GPIO7 | Left edge | GPIO | ADC1_CH6, TOUCH7 | Tenstar; CircuitPython `pins.c` 50–52 |
| 5V | — | Right edge | 5 V | — | Tenstar |
| GND | — | Right edge | Ground | — | Tenstar |
| 3V3 | — | Right edge | 3.3 V out | — | Tenstar |
| 13 | GPIO13 | Right edge | GPIO | ADC2_CH2, TOUCH13 | Tenstar; CircuitPython `pins.c` 75–77 |
| 12 | GPIO12 | Right edge | GPIO | ADC2_CH1, TOUCH12 | Tenstar; CircuitPython `pins.c` 71–73 |
| 11 | GPIO11 | Right edge | GPIO | ADC2_CH0, TOUCH11 | Tenstar; CircuitPython `pins.c` 67–69 |
| 10 | GPIO10 | Right edge | GPIO | ADC1_CH9, TOUCH10 | Tenstar; CircuitPython `pins.c` 63–65 |
| 9 | GPIO9 | Right edge | GPIO | ADC1_CH8, TOUCH9 | Tenstar; CircuitPython `pins.c` 59–61 |
| 8 | GPIO8 | Right edge | GPIO | ADC1_CH7, TOUCH8 | Tenstar; CircuitPython `pins.c` 55–57 |
| 14 | GPIO14 | Underside pad | GPIO | ADC2_CH3, TOUCH14 | Tenstar; CircuitPython `pins.c` 83–85 |
| 15 | GPIO15 | Underside pad | GPIO | ADC2_CH4, XTAL_32K_P | Tenstar; CircuitPython `pins.c` 87–89 |
| 16 | GPIO16 | Underside pad | GPIO | ADC2_CH5, XTAL_32K_N | Tenstar; CircuitPython `pins.c` 91–93 |
| 17 | GPIO17 | Underside pad | GPIO | ADC2_CH6 | Tenstar; CircuitPython `pins.c` 95–97 |
| 18 | GPIO18 | Underside pad | GPIO | ADC2_CH7 | Tenstar; CircuitPython `pins.c` 99–101 |
| 21 | GPIO21 | Underside pad | GPIO | — | Tenstar; CircuitPython `pins.c` 104–105 |
| 33 | GPIO33 | Underside pad | GPIO | Octal SPI DQ4, used only with 8-line flash or PSRAM | Tenstar; CircuitPython `pins.c` 107–108; datasheet Table 2-14 (p. 31) |
| 34 | GPIO34 | Underside pad | GPIO | Octal SPI DQ5, as GPIO33 | Tenstar; CircuitPython `pins.c` 110–111; datasheet Table 2-14 |
| 35 | GPIO35 | Underside pad | GPIO | Octal SPI DQ6, as GPIO33 | Tenstar; CircuitPython `pins.c` 113–114; datasheet Table 2-14 |
| 36 | GPIO36 | Underside pad | GPIO | Octal SPI DQ7, as GPIO33 | Tenstar; CircuitPython `pins.c` 116–117; datasheet Table 2-14 |
| 37 | GPIO37 | Underside pad | GPIO | Octal SPI DQS, as GPIO33 | Tenstar; CircuitPython `pins.c` 119–120; datasheet Table 2-14 |
| 38 | GPIO38 | Underside pad | GPIO | — | Tenstar; CircuitPython `pins.c` 122–123 |
| 39 | GPIO39 | Underside pad | GPIO | JTAG MTCK | Tenstar; CircuitPython `pins.c` 127–128 |
| 40 | GPIO40 | Underside pad | GPIO | JTAG MTDO | Tenstar; CircuitPython `pins.c` 130–131 |
| 41 | GPIO41 | Underside pad | GPIO | JTAG MTDI | Tenstar; CircuitPython `pins.c` 133–134 |
| 42 | GPIO42 | Underside pad | GPIO | JTAG MTMS | Tenstar; CircuitPython `pins.c` 136–137 |
| 45 | GPIO45 | Underside pad | GPIO; strapping (VDD_SPI voltage) | — | Tenstar; CircuitPython `pins.c` 139–140 |
| 46 | GPIO46 | Underside pad | GPIO; strapping (boot mode, ROM log) | — | Tenstar; CircuitPython `pins.c` 142–143 |
| 47 | GPIO47 | Underside pad | GPIO | — | Tenstar; CircuitPython `pins.c` 145–146 |
| 48 | GPIO48 | Underside pad | On-board RGB LED data and plain LED | — | Tenstar; CircuitPython `pins.c` 149–151 |

Notes on the table:

- **On-board LEDs:** the addressable RGB LED and the plain LED share GPIO48
  (Tenstar drawing; UnsignedArduino `README.md` line 11; espboards.dev).
  UnsignedArduino drives the plain LED with `digitalWrite` and the RGB LED
  with FastLED; espboards.dev says the two cannot be used independently.
- **BOOT button:** GPIO0, which has no pad (CircuitPython `pins.c` line 13).
- **GPIO48 is not a strapping pin.** espboards.dev's table marks it as one;
  the datasheet lists GPIO0, GPIO3, GPIO45 and GPIO46 (§3, p. 32).
- CircuitPython's board page counts 20 underside pads, and its `pins.c` lists
  19 of them plus GPIO48 as the NeoPixel without placing it; the Tenstar
  drawing shows GPIO48 as an underside pad.

### Strapping pins

| GPIO | Selects | Default at reset |
|---|---|---|
| GPIO0 | Boot mode, with GPIO46; BOOT button | Weak pull-up, 1 |
| GPIO3 | JTAG signal source | Floating |
| GPIO45 | VDD_SPI voltage | Weak pull-down, 0 |
| GPIO46 | Boot mode, with GPIO0; ROM message printing | Weak pull-down, 0 |

From datasheet §3 and Table 3-1 (p. 32).

### Pins that are not free

| GPIO | Why | Source |
|---|---|---|
| GPIO19, GPIO20 | USB D− and D+ to the USB-C connector; not broken out | Datasheet Table 2-8 (p. 24), §2.3.4 (p. 25) |
| GPIO26–GPIO32 | SPI0/1 to the in-package flash and PSRAM; not broken out | Datasheet §2.3.5 (p. 26), Table 2-14 (p. 31) |
| GPIO33–GPIO37 | Free on the FH4R2: they carry octal SPI only on parts with 8-line flash or PSRAM | Datasheet §2.3.5 (p. 26), Table 2-14 (p. 31) |

## Power

The board is powered from USB-C (ESPHome device page, "Powered by USB-C"). The
Tenstar drawing labels the edge pins `5V`, `GND` and "3V3(OUT)"; no source
says whether `5V` may be fed as an input. The Tenstar variant charges a
battery on `B+`/`B-` at 100 mA, or 300 mA with `BOOST` closed. No source names
the regulator, the charger IC or the battery chemistry.

## Flashing

The USB-C connector is the chip's own USB Serial/JTAG, so no USB-UART bridge
is involved. To force download mode, hold BOOT and press RESET (CircuitPython
board page), or hold BOOT while plugging in (espboards.dev). UnsignedArduino
`README.md` line 7 reports that some boards need this for every upload.

ESP-IDF settings for the ESP32-S3FH4R2 every source names (symbols checked
against `tools/kconfig_symbols/idf-v5.5.6.txt`):

```ini
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
# PSRAM in the package is 2 MB quad SPI (datasheet Table 1-1), not octal
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_QUAD=y
```

## Known issues and quirks

- **Weak Wi-Fi.** CircuitPython sets its default transmit power to 15 to
  "reduce wifi.radio.tx_power due to the antenna design of this board" (`mpconfigboard.h` lines 19–20).
  espboards.dev reports units with the antenna soldered backwards.
- **The Tenstar drawing's prose is copied from the ESP32-C3 board.** Its
  description and parameter list say "32-bit RISC-V single-core processor,
  running at up to 160 MHz", "400KB SRAM" and a 43 µA deep-sleep current: the
  figures of the [ESP32-C3 Super Mini](esp32-c3-super-mini.md), and contrary
  to its own "Chip model: ESP32S3FH4R2" line and the ESP32-S3 datasheet. This
  page takes only the drawing's chip model, pin diagram, LED and charger notes
  and photo from it.
- **Native USB drops** when the firmware crashes or deep-sleeps
  (espboards.dev); hold BOOT while plugging in to recover.

## In this repository

The flasher.json `board` field of chatterbox and finderbox names the board;
boombox and glowbug name it in their READMEs, justfiles and `main` sources.
GPIOs each project assigns, read from the firmware sources:

| GPIO | Project | Use | Where |
|---|---|---|---|
| GPIO1 | chatterbox, finderbox, boombox, glowbug | Battery monitor input (`thinkpack_power_init`, `adc_gpio = 1`; boombox and glowbug call it a placeholder) | `main/main.c` of each |
| GPIO1 | boombox | Tempo potentiometer, ADC1_CH0 | `boombox/main/pot_reader.c` line 22 |
| GPIO1 | glowbug | Light sensor, ADC1_CH0 | `glowbug/main/light_sensor.h` lines 24, 27 |
| GPIO2 | boombox | Pitch potentiometer, ADC1_CH1 | `boombox/main/pot_reader.c` line 23 |
| GPIO2 | finderbox | Piezo, LEDC | `finderbox/main/piezo.h` line 23 |
| GPIO4 | chatterbox | I2S TX BCLK | `chatterbox/main/audio_engine.c` line 27 |
| GPIO4 | finderbox, glowbug | External WS2812B ring data | `main/led_ring.h` of each |
| GPIO4 | boombox | External WS2812 data | `boombox/main/led_single.c` line 20 |
| GPIO5 | chatterbox | I2S TX LRCLK | `chatterbox/main/audio_engine.c` line 28 |
| GPIO5 | boombox | Piezo, LEDC | `boombox/main/tone_engine.c` line 23 |
| GPIO6 | chatterbox | I2S TX DOUT | `chatterbox/main/audio_engine.c` line 29 |
| GPIO6 | glowbug | I2C SDA (MPU6050) | `glowbug/main/imu.h` line 23 |
| GPIO7 | glowbug | I2C SCL (MPU6050) | `glowbug/main/imu.h` line 26 |
| GPIO9 | boombox, glowbug | Button, active low | `boombox/main/button.c` line 19, `glowbug/main/button.h` line 25 |
| GPIO10 | chatterbox | I2S RX BCLK | `chatterbox/main/audio_engine.c` line 31 |
| GPIO10 | thinkpack-rc522 (`BOARD_SUPERMINI`) | RC522 CS | `components/thinkpack-rc522/rc522_driver.c` line 33 |
| GPIO11 | chatterbox | I2S RX LRCLK | `chatterbox/main/audio_engine.c` line 32 |
| GPIO11 | thinkpack-rc522 (`BOARD_SUPERMINI`) | RC522 MOSI | `rc522_driver.c` line 32 |
| GPIO12 | chatterbox | I2S RX DIN | `chatterbox/main/audio_engine.c` line 33 |
| GPIO12 | thinkpack-rc522 (`BOARD_SUPERMINI`) | RC522 SCK | `rc522_driver.c` line 30 |
| GPIO13 | thinkpack-rc522 (`BOARD_SUPERMINI`) | RC522 MISO | `rc522_driver.c` line 31 |
| GPIO14 | chatterbox | Touch pad, TOUCH14 | `chatterbox/main/touch_driver.c` line 25 |
| GPIO14 | thinkpack-rc522 (`BOARD_SUPERMINI`) | RC522 RST | `rc522_driver.c` line 34 |

Pins the boxes' wiring documents name but no firmware drives: GPIO15, a status
LED chatterbox reserves ([`WIRING.md`](../../../packages/thinkpack/chatterbox/WIRING.md));
GPIO9, a finderbox button ([`WIRING.md`](../../../packages/thinkpack/finderbox/WIRING.md)),
with no button code in `finderbox/main/`. No project uses the on-board LEDs on
GPIO48, or a strapping pin.

`BOARD_SUPERMINI` is defined in no `CMakeLists.txt` or build file, so
`thinkpack-rc522` compiles its XIAO ESP32-S3 pin block (SCK 7, MISO 8, MOSI 9,
CS 5, RST 4) in finderbox, not the block above.

Project files that state the memory: chatterbox's
[`WIRING.md`](../../../packages/thinkpack/chatterbox/WIRING.md) says "4 MB
flash, octal PSRAM" and that GPIO35–37 are taken by the octal PSRAM, and its
[`sdkconfig.defaults`](../../../packages/thinkpack/chatterbox/sdkconfig.defaults)
sets `CONFIG_SPIRAM_MODE_OCT=y`. finderbox, boombox and glowbug set
`CONFIG_SPIRAM=n`. All four set `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`.

## Source

- **Pin names, edge order, underside pads, LEDs and charger:** the TENSTAR
  ROBOT pinout drawing, the maker's own document, watermarked, as committed to
  `esphome/esphome-devices` at `ff000958`
  (`src/docs/devices/TENSTAR-ROBOT-ESP32-S3-SuperMini/Pinouts.webp`). The
  copy espboards.dev links on alicdn is the same image: a pixel comparison of
  the two decoded images gave a mean absolute difference of 0.49 of 255.
- **GPIO-to-name map, the BOOT button, the RGB LED and memory modes:**
  CircuitPython's board definition, `adafruit/circuitpython` at
  `519f353038965db798d8476b80a193ad4877c965`,
  `ports/espressif/boards/generic_esp32s3_supermini/` (`pins.c`,
  `mpconfigboard.h`, `mpconfigboard.mk`), and its board page.
- **Memory mode and LED order:** the ESPHome device page
  (`src/docs/devices/TENSTAR-ROBOT-ESP32-S3-SuperMini/index.md`, published
  2026-01-11) and UnsignedArduino/ESP32-S3-Super-Mini-Test at `b9ba4904`.
- **Silicon facts** (memory, analog and touch channels, strapping, reserved
  pins): ESP32-S3 Series Datasheet v2.2, by table and page as cited above.
- **Repository witness:** the firmware sources at the paths listed under
  "In this repository".

**Nothing on this page has been checked on a fitted board. Which variant the
projects' boards are, their chip marking, and the edge order are all read from
other people's boards and documents.**

## Check on the physical board

- [ ] Read the chip marking, or run `esptool.py chip_id` and `flash_id`, and
      record which ESP32-S3 variant it is (every source says ESP32-S3FH4R2,
      which has 2 MB quad PSRAM, not octal).
- [ ] Record which variant it is: battery pads and `BOOST` jumper on the
      underside (Tenstar) or not, and any marking such as `HW747 V0.0.2`.
- [ ] Confirm the edge order: left `TX`, `RX`, `1`–`7` and right `5V`, `GND`,
      `3V3`, `13`–`8`, read from the top with USB-C up.
- [ ] Count the underside pads and confirm GPIO48 is one of them.
- [ ] Confirm the RGB LED's colour order (GRB, or RGB as on the `HW747
      V0.0.2` unit CircuitPython names).
- [ ] Note the plain LED's colour (blue per Tenstar, red per espboards.dev)
      and whether it lights with GPIO48 high or low.
- [ ] Read the part numbers of the 3.3 V regulator and, on a charger variant,
      the charger IC.
- [ ] Check whether `5V` connects to the USB-C VBUS, so it can be fed as an
      input, and which battery the charger is for.
- [ ] Measure the outline (23.5 × 18 mm or 22.52 × 18 mm).
- [ ] Check the antenna orientation (espboards.dev reports units soldered
      backwards).

## References

- [ESP32-S3 silicon page in this repository](../datasheets/mcu--esp32-s3.md)
- [ESP32-S3 Series Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf), read at v2.2
- [TENSTAR ROBOT ESP32-S3 SuperMini on devices.esphome.io](https://devices.esphome.io/devices/tenstar-robot-esp32-s3-supermini/)
- [TENSTAR ROBOT pinout drawing in esphome-devices](https://github.com/esphome/esphome-devices/blob/ff0009586b5666ab149551faaaeab1900ebe3a5c/src/docs/devices/TENSTAR-ROBOT-ESP32-S3-SuperMini/Pinouts.webp)
- [Generic ESP32-S3 SuperMini on circuitpython.org](https://circuitpython.org/board/generic_esp32s3_supermini/)
- [CircuitPython board definition](https://github.com/adafruit/circuitpython/tree/519f353038965db798d8476b80a193ad4877c965/ports/espressif/boards/generic_esp32s3_supermini)
- [ESP32-S3 Super Mini on espboards.dev](https://www.espboards.dev/esp32/esp32-s3-super-mini/)
- [UnsignedArduino/ESP32-S3-Super-Mini-Test](https://github.com/UnsignedArduino/ESP32-S3-Super-Mini-Test)
