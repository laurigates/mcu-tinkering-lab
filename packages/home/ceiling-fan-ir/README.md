# Ceiling Fan IR

ESPHome firmware that drives a Pepeo Bayu 122 cm DC ceiling fan through its IR
remote protocol, from an ESP_IR_TR board (ESP-01M / ESP8285). It replaces a
2018 D1 mini + IRremoteESP8266 sketch that only ever sent "power on" when an
MQTT temperature feed passed 26 °C. Decisions like that now belong in a Home
Assistant automation; the device only knows how to talk to the fan.

Board reference: [`docs/reference/boards/esp-ir-tr.md`](../../../docs/reference/boards/esp-ir-tr.md).

## Status: stage 1, verify the codes

The current config does two things:

- **Receives:** `remote_receiver` on GPIO14 logs every NEC frame from the fan's
  own remote.
- **Sends:** one Home Assistant button per known code, each transmitting on
  GPIO4.

Fan, light and timer entities come in stage 2, once the codes below are
confirmed against the real remote.

## IR Codes

All codes use NEC address `0x5941`. They come from the 2018 sketch's
`sendNEC(0x829A....)` values, bit-reversed per byte into ESPHome's standard
NEC order (ESPHome 2021.12+). Every command passes NEC's inverse-byte check.
None have been sent to the fan from this firmware yet.

| Button | 2018 sketch | ESPHome command |
|---|---|---|
| Power On | `0x829AA05F` | `0xFA05` |
| Power Off | `0x829A50AF` | `0xF50A` |
| Speed 1 | `0x829A8A75` | `0xAE51` |
| Speed 2 | `0x829A22DD` | `0xBB44` |
| Speed 3 | `0x829A0AF5` | `0xAF50` |
| Speed 4 | `0x829A32CD` | `0xB34C` |
| Speed 5 | `0x829A02FD` | `0xBF40` |
| Light Power | `0x829AA25D` | `0xBA45` |
| Light Dim Down | `0x829A807F` | `0xFE01` |
| Light Dim Up | `0x829AB04F` | `0xF20D` |
| Reverse | `0x829AB24D` | `0xB24D` |
| Timer | `0x829A827D` | `0xBE41` |

Two things the stage-1 logs need to settle:

- **Colour temperature.** The fan's light has three modes (3000/4000/5000 K)
  but the 2018 sketch has no code for switching between them. Press the
  remote's colour button, and try a long press on the light button, to find
  it.
- **Timer vs. sleep.** The 2018 sketch called this code `SLEEP`; the product
  listing says the remote has a timer. Confirm it is the same button.

## Flashing

The board has no USB port. Wiring (`just ceiling-fan::pins`):

| Header pin | Signal | Connect to |
|---|---|---|
| 1 (board corner) | GND | adapter GND |
| 2 | 5V | adapter 5V |
| 3 | RX | adapter TX (3.3 V logic) |
| 4 | TX | adapter RX |
| 5–6 | IO0, GND | jumper fitted = flash mode |

```sh
just ceiling-fan::config        # creates secrets.yaml; fill in WiFi, API key, OTA password
just ceiling-fan::first-flash   # jumper fitted
# remove the jumper, power-cycle
just ceiling-fan::logs          # press buttons on the fan's remote
```

Later updates go over WiFi with `just ceiling-fan::wireless`.

## Flash Budget

The ESP8285 has 1 MB of flash, and OTA needs the running image and the new one
side by side. The stage-1 build uses 386,243 of 1,023,984 bytes (37.7%) and
31,144 of 81,920 bytes of RAM (`esphome compile`, ESPHome 2026.9.1). For that
reason the config leaves out `web_server`, `captive_portal` and the fallback AP.
