# Ceiling Fan IR

ESPHome firmware that drives a Pepeo Bayu 122 cm DC ceiling fan through its IR
remote protocol, from an ESP_IR_TR board (ESP-01M / ESP8285). It replaces a
2018 D1 mini + IRremoteESP8266 sketch that only ever sent "power on" when an
MQTT temperature feed passed 26 °C. Decisions like that now belong in a Home
Assistant automation; the device only knows how to talk to the fan.

Board reference: [`docs/reference/boards/esp-ir-tr.md`](../../../docs/reference/boards/esp-ir-tr.md).

## Entities

| Entity | Type | Sends |
|---|---|---|
| Fan | fan: on/off, 5 speeds, direction | Power On/Off, the speed code, Reverse |
| Light | select: Off, Cool (5000 K), Neutral (4000 K), Warm (3000 K) | Light Power, as many presses as the cycle needs |
| Light Dim Down / Dim Up | button | one dim step |
| Timer | button | the remote's timer code |
| Resync: Mark All Off | button (config) | nothing |

### How the assumed state works

IR is one-way, so every state above is assumed, not read from the fan. Two
things keep it in step:

- **Sending only differences.** The firmware remembers what it last sent. A
  change from Home Assistant transmits only what differs: Power On, then the
  speed (always after a power-on, because the fan ignores speed codes while
  off), then Reverse if the direction changed. Codes go out queued, 150 ms
  apart.
- **Listening to the remote.** The on-board receiver decodes the fan's own
  remote and updates the entities the same way, so a press on the remote
  shows up in Home Assistant. Speed presses while the fan is off are ignored,
  as the fan ignores them. Anything received within 400 ms of a transmission
  is dropped, because the receiver hears the board's own LED and Reverse and
  Light are toggles.

The state restored after a reboot is taken as the fan's actual state, and is
not re-sent. If the entities drift from the fan anyway (the remote was used
where the receiver could not see it), turn the fan and light off with the
remote and press **Resync: Mark All Off**.

The light button cycles Off, Cool, Neutral, Warm, Off (observed by Lauri,
2026-10-22). Dimming is relative, so it stays as two buttons rather than a
brightness.

## IR Codes

All codes use NEC address `0x5941`. They come from the 2018 sketch's
`sendNEC(0x829A....)` values, bit-reversed per byte into ESPHome's standard
NEC order (ESPHome 2021.12+). On 2026-10-22 every button on the fan's remote
was decoded by this board's receiver, and each matched the table; no other
codes appeared. The colour temperature has no code of its own. It is part of
the light button's cycle.

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
| Timer (the sketch's `SLEEP`) | `0x829A827D` | `0xBE41` |

## Bench Checks

Transmission from this firmware has not been tried on the fan yet:

- [ ] Fan on at speed 3 from Home Assistant: the fan starts at speed 3.
      If nothing happens, check the LED with a phone camera; no flicker means
      the transmitter pin needs `inverted: true`.
- [ ] Light set from Off to Warm: three presses arrive and the light ends on
      warm. If presses are lost, lengthen the 150 ms gap in the `send` script.
- [ ] Reverse from Home Assistant, while running and while off.
- [ ] A press on the physical remote updates Home Assistant, and causes no
      second change on the fan.

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
side by side. The current build uses 398,099 of 1,023,984 bytes (38.9%) and
31,100 of 81,920 bytes of RAM (`esphome compile`, ESPHome 2026.9.1). For that
reason the config leaves out `web_server`, `captive_portal` and the fallback AP.
