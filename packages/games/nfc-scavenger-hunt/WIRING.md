# NFC Scavenger Hunt — Wiring Guide

Every pin below is read from the source file and line named in its row. None
of it has been checked against a built board for this document. Where the
source does not say something (supply voltages, the amplifier's gain and
shutdown pins, resistor values, which RC522 or MAX98357A module), this page
says so instead of filling it in.

## Board

The source has two pin sets, selected at compile time by `BOARD_SUPERMINI`:

- **XIAO ESP32-S3** — the `#else` branch, used by every build today.
- **ESP32-S3 SuperMini** — the `#if defined(BOARD_SUPERMINI)` branch.

**Which one a build uses:** the XIAO set. `BOARD_SUPERMINI` is defined
nowhere in the repository: not in either `CMakeLists.txt`, not in
`main/CMakeLists.txt`, not in the `thinkpack-rc522` component, and it is not
a Kconfig symbol, so `sdkconfig.defaults` cannot set it.
`git grep -n BOARD_SUPERMINI` finds only the `#if` lines, comments and
documentation. The
release binary on the web flasher is therefore the XIAO pinout.

The define is tested in three files across two components:
`main/i2s_audio.c`, `main/status_led.c` and
[`thinkpack-rc522/rc522_driver.c`](../../components/thinkpack-rc522/rc522_driver.c).
A definition added to only one component switches its pins and leaves the
other on the XIAO set.

The build is set up for an ESP32-S3 with octal PSRAM and a 4 MB flash size
(`sdkconfig.defaults` lines 11-12 and 59-60; the comment there says the
XIAO has 8 MB). Board pages: [XIAO ESP32-S3](../../../docs/reference/boards/xiao-esp32s3.md);
the repo has no page for the ESP32-S3 SuperMini.

## Pin assignments — XIAO ESP32-S3 (built)

| GPIO | XIAO pad | Function | Direction | Component | Source |
|------|----------|----------|-----------|-----------|--------|
| GPIO7 | D8 | SPI SCK | Output | RC522 | `rc522_driver.c:37` |
| GPIO8 | D9 | SPI MISO | Input | RC522 | `rc522_driver.c:38` |
| GPIO9 | D10 | SPI MOSI | Output | RC522 | `rc522_driver.c:39` |
| GPIO5 | D4 | SPI CS | Output | RC522 | `rc522_driver.c:40` |
| GPIO4 | D3 | RST | Output | RC522 | `rc522_driver.c:41` |
| GPIO6 | D5 | I2S BCLK | Output | MAX98357A | `i2s_audio.c:19` |
| GPIO43 | D6 | I2S LRCLK (WS) | Output | MAX98357A | `i2s_audio.c:20` |
| GPIO44 | D7 | I2S data out (DOUT) | Output | MAX98357A | `i2s_audio.c:21` |
| GPIO3 | D2 | Reset-progress button | Input, internal pull-up | Push button to GND | `main.c:24` |
| GPIO2 | D1 | Status LED | Output, active high | LED to GND | `status_led.c:18` |

The XIAO pad names come from the
[board page's pin table](../../../docs/reference/boards/xiao-esp32s3.md).
The XIAO's own user LED is on GPIO21 (same page), so on this board the
status LED is an external part on D1.

## Pin assignments — ESP32-S3 SuperMini (not built)

| GPIO | Function | Direction | Component | Source |
|------|----------|-----------|-----------|--------|
| GPIO12 | SPI SCK | Output | RC522 | `rc522_driver.c:30` |
| GPIO13 | SPI MISO | Input | RC522 | `rc522_driver.c:31` |
| GPIO11 | SPI MOSI | Output | RC522 | `rc522_driver.c:32` |
| GPIO10 | SPI CS | Output | RC522 | `rc522_driver.c:33` |
| GPIO14 | RST | Output | RC522 | `rc522_driver.c:34` |
| GPIO4 | I2S BCLK | Output | MAX98357A | `i2s_audio.c:15` |
| GPIO5 | I2S LRCLK (WS) | Output | MAX98357A | `i2s_audio.c:16` |
| GPIO6 | I2S data out (DOUT) | Output | MAX98357A | `i2s_audio.c:17` |
| GPIO3 | Reset-progress button | Input, internal pull-up | Push button to GND | `main.c:23-24` ("same GPIO on both boards") |
| GPIO48 | Status LED | Output | On-board WS2812 | `status_led.c:12-16` |

`status_led.c` drives GPIO48 as a plain on/off output and says in a comment
that a WS2812 driver would be needed for colour. Whether a WS2812 responds
to a plain level on its data line was not checked.

## Wiring diagram (XIAO set)

```
XIAO ESP32-S3                 RC522 (SPI)
  D8  GPIO7  ───────────────  SCK
  D9  GPIO8  ───────────────  MISO
  D10 GPIO9  ───────────────  MOSI
  D4  GPIO5  ───────────────  chip select
  D3  GPIO4  ───────────────  reset

                              MAX98357A (I2S)
  D5  GPIO6  ───────────────  BCLK
  D6  GPIO43 ───────────────  LRCLK (WS)
  D7  GPIO44 ───────────────  data (DOUT)

  D2  GPIO3  ──── button ───  GND
  D1  GPIO2  ──── LED ──────  GND
```

Supply and ground for the RC522 and the amplifier are not recorded in the
source. Neither are the amplifier's GAIN and SD pins or the speaker.

## Components

| Component | What the source says | Not recorded |
|-----------|----------------------|--------------|
| RC522 (MFRC522) NFC reader | SPI2_HOST, 5 MHz (`rc522_driver.c:29,36,331`); reads 4- or 7-byte ISO14443A UIDs ([component README](../../components/thinkpack-rc522/README.md)) | Module, supply voltage |
| MAX98357A I2S amplifier | Named in `i2s_audio.h:10`; Philips I2S, 16-bit stereo, 24 kHz, mono duplicated to both slots (`i2s_audio.c:51-55,102-104`) | Module, supply, GAIN and SD pins, speaker. Repo page for one breakout: [adafruit-max98357a.md](../../../docs/reference/boards/adafruit-max98357a.md) |
| Push button | Active low on GPIO3; holding it 2 s clears progress (`main.c:25,128-134,148-150`) | Part |
| Status LED | Active high (`status_led.c:27`) | LED colour, resistor value |

## Status LED

| Mode | Pattern (`status_led.c:35-67`) | When (`main.c`) |
|------|-------------------------------|-----------------|
| `LED_RAPID_BLINK` | toggles every 100 ms | boot; fetching a voice clue |
| `LED_SLOW_BLINK` | toggles every 500 ms | ready, scanning for tags |
| `LED_SOLID_ON` | on | playing a voice clue |
| `LED_PULSE` | six 80 ms toggles, then 300 ms off | all tags found |

## Notes and hazards

- **GPIO3 is a strapping pin** (JTAG interface selection, sampled at reset;
  [board page](../../../docs/reference/boards/xiao-esp32s3.md)). Holding
  the button during a reset changes what the chip samples there.
- **GPIO43 and GPIO44 are UART0 TX and RX** on the XIAO. The ROM bootloader
  prints on UART0 at every reset, and ESP-IDF's console defaults to UART0 on
  the ESP32-S3 ([board page](../../../docs/reference/boards/xiao-esp32s3.md),
  GPIO43 row). This project's `sdkconfig.defaults` sets no
  `CONFIG_ESP_CONSOLE_*`, so that output reaches the amplifier's LRCLK and data
  lines until `i2s_audio_init()` claims the pins. Whether it is audible was
  not checked.
- **No RC522 means no boot.** `main.c:226` wraps `thinkpack_rc522_init()` in
  `ESP_ERROR_CHECK`, and the init returns `ESP_ERR_NOT_FOUND` when the chip
  does not answer (`thinkpack_rc522.h:36`), so a missing or miswired reader aborts at start-up.
