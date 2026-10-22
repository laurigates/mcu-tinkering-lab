# ESP_IR_TR ESP8285 IR Transceiver (ESP-01M)

A 24 × 24 mm Wi-Fi IR transceiver: an Ai-Thinker ESP-01M module (ESP8285, flash
in package) standing upright on a small carrier with a 38 kHz IR receiver, one
IR LED behind a transistor, and a 3.3 V regulator. It has no USB port; it is
flashed and powered through a pin header. Sold as an IR remote bridge and
listed by Tasmota as `ESP_IR_TR`. For the SoC's electrical facts see
[`mcu--esp8285.md`](../datasheets/mcu--esp8285.md).

**The fitted board:** Lauri's unit is silkscreened `ESP_IR_TR_V1.1` (as
reported; the listing photos do not show this marking). The closest listing
found is YourCee's "ESP32 ESP8266 Infrared Transmitter Receiver NEC Protocol
WIFI Transmission Module IR Remote Control Switch ESP8285 ESP-01M 38kHz",
AliExpress item [32844087693](https://www.aliexpress.com/item/32844087693.html).
That listing sells several different boards under one title. This page is
about the **black ESP8285 board** in its photo 5 ("ESP8285 Infrared Switch
Module") and in the Tasmota template's photo. The listing's other boards
are different products and none of this page applies to them:

| Listing board | Module | Size | Back-face marking |
|---|---|---|---|
| **Black, upright module (this page)** | ESP-01M (ESP8285) | 24 × 24 mm | not shown |
| Blue, flat module, USB-C | ESP8266MOD (ESP-12 type) | 33 × 33 mm | `ESP_IR_TR_WIFI (303ESPIRTR3)` |
| Blue, flat module, USB-C | ESP32-32E N4 | — | not shown |
| Blue, four IR LEDs ("IR 360°") | none (IR head only) | — | not shown |

## Specifications

| Feature | Details | Source |
|---------|---------|--------|
| Module | Ai-Thinker ESP-01M, FCC ID 2AHMR-ESP01M | Listing photos 5, 6 |
| SoC | ESP8285 (ESP8266 core, 1 or 2 MB flash in package) | Listing title; [datasheet page](../datasheets/mcu--esp8285.md) |
| Flash | 1 MB (ESP8285N08) or 2 MB (ESP8285H16); variant not stated | ESP8285 datasheet Table 1-1 |
| Wi-Fi | 802.11 b/g/n, 2.4 GHz, PCB antenna on the module | ESP8285 datasheet §1.1 |
| IR receiver | `VS838`-marked 38 kHz demodulating receiver, `IR_RX` | Listing photos 5 and Tasmota photo |
| IR transmitter | One clear 5 mm IR LED, `IR_TX`, switched by transistor `Q1` | Listing photo 5, Tasmota photo |
| Regulator | AMS1117-3.3 (`U2`, SOT-223) | Tasmota photo marking `AMS1117 3.3` |
| Header | 4 pins plus a 2-pin header with a jumper cap; labels not legible | Listing photo 5 |
| USB | None, flash through a 3.3 V USB-serial adapter | Listing photos |
| Dimensions | 24 × 24 mm board; module stands 18 mm wide, 15 mm above the board | Listing photo 5 dimension arrows |
| Mounting | One hole, front corner | Listing photo 5 |

## Pin Mapping

The board exposes no GPIO header of its own; the GPIOs that matter are the
ones wired to the IR parts. The IR pins are from the
[Tasmota template](https://templates.blakadder.com/ESP_IR_TR.html)
(`"GPIO":[0,0,0,0,1056,0,0,0,0,0,1088,0,0,0]`: IRsend on GPIO4, IRrecv on
GPIO14). The header rows are what an ESP-01M IR board needs to be flashable,
**not** read from this board: the photos show four header pins whose labels
cannot be read, so the names and the order are open (see the checklist).
No `Side`/`Pos` is claimed for that reason.

| Pin | GPIO | Default Function | Alternate Functions | Notes |
|-----|------|------------------|---------------------|-------|
| IR_TX | GPIO4 | IR LED, via `Q1` | — | Tasmota template. Drive polarity through `Q1` unverified |
| IR_RX | GPIO14 | 38 kHz receiver output | HSPI_CLK | Tasmota template. Receiver output idles high, so ESPHome needs `inverted: true` |
| TX | GPIO1 | UART0 TX | — | Presumed on the header; not legible |
| RX | GPIO3 | UART0 RX | — | Presumed on the header; not legible |
| IO0 | GPIO0 | Boot mode, strapping | SPI_CS2 | Presumed on the 2-pin jumper to GND; jumper fitted = download mode |
| VCC | — | Supply in | — | Presumed 5 V into the AMS1117; see Power |
| GND | — | Ground | — | |

## Power

The board carries an AMS1117-3.3 regulator, so its supply pin presumably feeds
that regulator and expects about 5 V, as most resellers state. The search
summary of the AliExpress listing quoted 3.0–3.6 V instead. Measure which
header pin goes to the AMS1117 input and which to its 3.3 V output before
connecting anything. Resellers also warn there is **no reverse-polarity
protection**.

## Flashing

1. Fit the jumper (presumed `IO0`–`GND`) to hold GPIO0 low at power-up.
2. Connect a **3.3 V-logic** USB-serial adapter: adapter TX to board RX,
   adapter RX to board TX, GND to GND. Power from the adapter's 5 V pin if the
   header's supply pin is confirmed to feed the AMS1117.
3. Check you're talking to the right chip: `esptool.py chip_id` should name an
   `ESP8285` variant; `esptool.py flash_id` gives the flash size.
4. Flash, remove the jumper, power-cycle.

A Home Assistant forum user could not program this board family until they
switched to a CH340-based adapter.

## ESPHome

```yaml
esp8266:
  board: esp8285

remote_transmitter:
  pin: GPIO4
  carrier_duty_percent: 50%

remote_receiver:
  pin:
    number: GPIO14
    inverted: true
  dump: nec
```

Keep the build lean on 1 MB (no `web_server`, no `captive_portal`, low logger
level) so OTA has room for two images. The flash variant decides that limit.

## Known Issues and Quirks

- **IR range is limited.** A single LED, and on similar boards the module itself
  partly shadows it. A Home Assistant forum user reported poor spread on this
  board. Aim it at the fan's receiver.
- **One title, several boards.** Listings for "ESP_IR_TR" and "ESP8285 IR
  transceiver" mix this board with the ESP8266MOD and ESP32 boards in the table
  above. Pins and power differ between them.

## In This Repository

Planned as the IR controller for the ceiling fan (Pepeo Bayu 122 cm
DC, IR remote), replacing a 2018 D1 mini + IRremoteESP8266 sketch. Not yet a
package; no `hardware.toml` names this page.

## Source

**Nothing on this page has been checked on the fitted board.** The IR GPIOs
come from the Tasmota community template; the parts, dimensions and header
count were read from the AliExpress listing's photos and the Tasmota device
photo, not from a vendor board file or schematic (none is published). The
listing photos are of a product line, not of Lauri's `V1.1` unit.
`.claude/rules/board-layout-from-vendor-files.md` covers why a photo is weak
evidence for header order, which is why no order is claimed.

## Check on the physical board

- [ ] Silkscreen on both faces: `ESP_IR_TR_V1.1`, header labels, jumper label.
      Photograph both faces and record the header order here.
- [ ] Which header pin feeds the AMS1117 input (5 V) and which carries 3.3 V.
- [ ] The 2-pin jumper is `IO0`–`GND`.
- [ ] `esptool.py chip_id` names an ESP8285 variant; `flash_id` gives 1 or 2 MB.
- [ ] `remote_receiver` on GPIO14 with `inverted: true` logs NEC codes from
      the fan's remote.
- [ ] `remote_transmitter` on GPIO4 lights the IR LED (a phone camera shows
      it), and the fan responds; if not, try `inverted: true` on the
      transmitter pin.

## References

- [ESP_IR_TR Tasmota template (blakadder)](https://templates.blakadder.com/ESP_IR_TR.html)
- [YourCee AliExpress listing 32844087693](https://www.aliexpress.com/item/32844087693.html)
- [ESP8285 datasheet v2.6 (Espressif)](https://www.espressif.com/sites/default/files/documentation/0a-esp8285_datasheet_en.pdf)
- [esp_ir_tr programming issue, CH340 fix (Home Assistant Community)](https://community.home-assistant.io/t/not-able-to-program-communicate-with-esp-ir-tr-module-esp8285-all-in-one-ir-tr-to-program/598182)
- [ESPHome IR blaster hardware hacking, range on this board (Home Assistant Community)](https://community.home-assistant.io/t/esphome-ir-blaster-remote-control-hardware-hacking-tuya-brain-transplants/752992)
- [HiLetgo ESP8285 ESP-01M IR module (Amazon), reverse-polarity warning](https://www.amazon.com/HiLetgo-Infrared-Transmitter-Receiver-Transceiver/dp/B09KGXNZ2Q)
