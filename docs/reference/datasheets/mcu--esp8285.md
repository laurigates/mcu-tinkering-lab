# ESP8285 (Tensilica L106, flash in package)

**Used in:** [ESP_IR_TR IR transceiver](../boards/esp-ir-tr.md), via an Ai-Thinker ESP-01M module

An ESP8266 with its SPI flash inside the package. Software treats it as an
ESP8266: ESPHome and PlatformIO select it with `board: esp8285`. esptool
detects it as the ESP8266 family and names the variant from eFuse bits
(`ESP8285N08`, `ESP8285H16`, … in `esptool/targets/esp8266.py`). Espressif marks both variants
**NRND** (not recommended for new designs) on every page of datasheet v2.6.

All values below are from the ESP8285 datasheet v2.6 (linked below); the
section or table is given for each.

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| CPU | Tensilica L106 32-bit RISC, up to 160 MHz | §3.1.1 |
| Flash | Built-in: 1 MB (ESP8285N08) or 2 MB (ESP8285H16) | Table 1-1 |
| Flash SPI mode | Dual SPI, so `dout`/`dio` in flashing tools, not `qio` | §3.1.3 |
| User SRAM | Less than 75 kB (heap + data) in station mode, connected | §3.1.2 |
| Wi-Fi | 802.11 b/g/n, 2.4 GHz | §1.1 |
| Bluetooth | None | — |
| Operating voltage | 2.7–3.6 V | Table 1-2 |
| Operating current | 80 mA average | Table 1-2 |
| ADC | One 10-bit SAR ADC on `TOUT` (pin 6) | §4.9 |
| Interfaces | UART, SDIO, SPI, I2C, I2S, IR remote control, PWM | Table 1-2 |
| Firmware upgrade | UART download, OTA | Table 1-2 |
| Operating temp | –40 to 85 °C (N08), –40 to 105 °C (H16) | Table 1-1 |

## Power Modes

| Mode | Current | Source |
|------|---------|--------|
| Modem-sleep (CPU on, radio off) | 15 mA | Table 3-4 |
| Light-sleep | 0.9 mA | Table 3-4 |
| Deep-sleep (RTC only) | 20 µA | Table 3-4 |
| Shut down | 0.5 µA | Table 3-4 |

## Pin Notes

- **Strapping:** GPIO0, GPIO2 and MTDO (GPIO15) select the boot mode
  (§2, note 1 under Table 2-1). GPIO0 low at reset enters UART download mode.
- **U0TXD must not be pulled low externally during power-up** (§2, note 3).
- **Embedded flash:** `SDIO_CMD`, `SDIO_CLK`, `SDIO_DATA_0` and `SDIO_DATA_1`
  connect the internal flash and are not recommended for other use (§2, note 2).
  Note 2 does not list `SDIO_DATA_2` (GPIO9) or `SDIO_DATA_3` (GPIO10), so these
  two are free on the ESP8285, unlike on an ESP8266 with external quad flash.
  That is inferred from the note; the datasheet does not state it directly.
- **GPIO16** is the deep-sleep wake pin and must connect to `EXT_RSTB` for timed wake (Table 2-1).
- **Which flash variant you have:** an ESP-01M doesn't say on its shield.
  `esptool.py chip_id` prints the variant name, and `esptool.py flash_id` the
  flash size. On 1 MB an OTA update needs the running and the new image to
  fit side by side.

## Datasheets & References

- **ESP8285 datasheet v2.6 (Espressif):** <https://www.espressif.com/sites/default/files/documentation/0a-esp8285_datasheet_en.pdf>
- **ESP8266 Technical Reference (Espressif), shared peripheral set:** <https://www.espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf>
