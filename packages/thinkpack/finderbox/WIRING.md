# Finderbox Wiring

ESP32-S3 SuperMini with an MFRC522 NFC reader, a WS2812B LED ring, a
piezo buzzer and a user button.

## ESP32-S3 SuperMini pin constraints

The SuperMini breaks out only ~11 usable GPIOs. Several of them are
dual-use (strapping, USB), so pin selection for a multi-peripheral board
like Finderbox is tight:

- **Strapping pins to avoid for outputs at boot:** GPIO0, GPIO3, GPIO45,
  GPIO46. Do not drive them from a peripheral that starts actively
  during reset.
- **USB-Serial-JTAG:** GPIO19 and GPIO20 are consumed by the on-board
  USB-C connector — do not repurpose.
- **Flash / PSRAM bus:** GPIO26–32 are used by the internal SPI flash
  (non-Octal S3 modules) and are not exposed.

With those out, the SPI bus for the RC522 is placed on the HSPI-capable
group (`SPI2_HOST` / FSPI on S3): SCK=12, MISO=13, MOSI=11, CS=10,
RST=14. This matches the existing battle-tested `BOARD_SUPERMINI` pins
in the nfc-scavenger-hunt driver. `SPI3_HOST` is left free in case a
second SPI peripheral is added later.

## Pin table

| GPIO | Signal          | Direction | Notes                                |
|------|-----------------|-----------|--------------------------------------|
| 2    | Piezo (LEDC)    | Output    | 10-bit PWM, 50 % duty; `piezo.c`     |
| 4    | WS2812B DIN     | Output    | RMT; see `led_ring.c`                |
| 9    | Button          | Input     | Active LOW, internal pull-up         |
| 10   | RC522 SDA/CS    | Output    | SPI2_HOST (FSPI) CS                  |
| 11   | RC522 MOSI      | Output    | SPI2_HOST MOSI                       |
| 12   | RC522 SCK       | Output    | SPI2_HOST SCK                        |
| 13   | RC522 MISO      | Input     | SPI2_HOST MISO                       |
| 14   | RC522 RST       | Output    | Hardware reset pulse at boot         |

Build with `-DBOARD_SUPERMINI` to select this pin group (the default in
`rc522_driver.c` is the XIAO ESP32-S3 set: SCK=7, MISO=8, MOSI=9, CS=5,
RST=4).

## MFRC522 module

| Module pin | ESP32-S3 GPIO | Notes                               |
|------------|---------------|-------------------------------------|
| VCC        | 3V3           | 3.3 V only — module is 3.3 V logic  |
| GND        | GND           |                                     |
| SCK        | GPIO12        |                                     |
| MISO       | GPIO13        |                                     |
| MOSI       | GPIO11        |                                     |
| SDA        | GPIO10        | SPI chip-select                     |
| RST        | GPIO14        |                                     |
| IRQ        | —             | Not connected; driver polls         |

## WS2812B ring

- 5 V → ring VCC (external supply; do **not** drive 12 LEDs from the
  SuperMini 3V3 rail).
- GND → common ground with the SuperMini.
- DIN → GPIO4 (3.3 V logic is accepted).

## Hardware verification status

Not yet tested on hardware. The RC522 driver timing (SPI clock, reset
timing, TPrescaler) is inherited from
`packages/games/nfc-scavenger-hunt/main/rc522_driver.c`, which is known
working. The SuperMini pin allocation mirrors the `BOARD_SUPERMINI`
branch from that driver.
