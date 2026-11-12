# Adafruit VL53L4CD Time of Flight Distance Sensor (#5396)

> **Candidate, not yet bought.** Nothing on this page has been checked against
> a fitted board. Chosen because Adafruit publishes the Eagle board file, so
> the header order below is read from it rather than from a photo.

Breakout for ST's VL53L4CD time-of-flight sensor: a 2.8 V regulator, I2C level
shifting, a 6-pin header along one edge and two STEMMA QT (Qwiic-compatible)
connectors. robocar plans two of them, aimed down at the front corners
as cliff detectors on TCA9548A channels 5 and 6. For the sensor itself see
[`sensor--vl53l4cd.md`](../datasheets/sensor--vl53l4cd.md).

## Specifications

| Parameter | Value | Source |
|-----------|-------|--------|
| Outline | 25.40 × 17.78 mm | Board file, `Dimension` layer (20) extents |
| Supply (VIN) | 3–5 V | Bottom-face silkscreen "VLogic/Vcc: 3-5VDC"; Adafruit README ("any 3-5V power or logic microcontroller") |
| Sensor rail | 2.8 V from U3, an LP5907-2.8 LDO | Board file: VCC to U3 pads 1 and 3, U3 pad 5 on net `2.8V` |
| I2C level | Shifted between VIN and 2.8 V (Q3, dual BSS138) | Board file nets `SDA`/`SDA_3V`, `SCL`/`SCL_3V` |
| I2C address | 0x29 (7-bit), fixed on the board | Bottom-face silkscreen "I2C addr=0x29"; datasheet page |
| Range | "1 to 1300mm" | Bottom-face silkscreen; README. The datasheet's own figures are on the datasheet page |
| Mounting holes | Four, plated, 2.5 mm package (`MOUNTINGHOLE_2.5_PLATED`) | Board file elements `U$30`, `U$31`, `U$33`, `U$34` |
| Header | 6 × 0.1 in, one row (`1X06_ROUND_70`) | Board file element `JP2` |

## Physical layout

Viewed from the component side, in the board file's own orientation: the
6-pin header runs along the bottom edge, `VIN` at the left, with a STEMMA QT
connector on each short edge. Labels alternate above and below the row on the
top face.

`Side`/`Pos` follow the convention in
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py): `L`/`R`/`T`/`B`
edge, `Pos 1` at the left of a top or bottom edge. `Header` is the vendor's
element and pad. Names are the top-face silkscreen; the board file calls pin 1
`VCC`, and the silkscreen prints `VIN`.

| Pin | Side | Pos | Header | Notes |
|-----|------|-----|--------|-------|
| VIN | B | 1 | JP2.1 | 3–5 V in; also the I2C pull-up level (net `VCC`) |
| GND | B | 2 | JP2.2 | |
| SCL | B | 3 | JP2.3 | Host side of the level shifter, 10 kΩ to VIN |
| SDA | B | 4 | JP2.4 | Host side of the level shifter, 10 kΩ to VIN |
| GPIO | B | 5 | JP2.5 | Sensor GPIO1 interrupt, open drain, 10 kΩ to **2.8 V**, not level-shifted |
| XSHUT | B | 6 | JP2.6 | Sensor XSHUT through a diode, 10 kΩ to 2.8 V on the sensor side; active low |

### STEMMA QT connectors

Both are JST SH 4-pin, top-mounted, wired in parallel with the header's power
and I2C pins (`tools/breakout-pinout.py`, elements `CONN3` and `CONN4`):

| Connector | Edge | Pad 1 | Pad 2 | Pad 3 | Pad 4 |
|-----------|------|-------|-------|-------|-------|
| CONN4 | Left | GND | VCC (VIN) | SDA | SCL |
| CONN3 | Right | GND | VCC (VIN) | SDA | SCL |

## Pad centres

Not recorded while the board is a candidate: `tools/hardware/test_padcheck.py`
pins the pages carrying a pad-centre table to the breakouts robocar
mounts. When the board is adopted, add the table with
`PYTHONPATH=tools python3 -m hardware.padcheck --print adafruit-vl53l4cd` and add
the slug to that test's `BREAKOUTS`. The header pads lie on a 2.54 mm pitch
along one row (`JP2`, board file at the commit under Source).

## Power and level shifting

Traced through the board file's `<signal>` contact lists:

- **Regulator.** `VCC` (VIN, both STEMMA QT pad 2s) feeds U3, an LP5907-2.8,
  on its input (pad 1) and enable (pad 3); U3 pad 5 is the `2.8V` net, which
  supplies the sensor's AVDD (U1 pad 11) and AVDDVCSEL (U1 pad 1). C4 (10 µF)
  sits on VCC, C5 (10 µF) and C6 (0.1 µF) on 2.8 V. The sensor's own range is
  2.6–3.5 V (DS13812 Table 10), so it is never exposed to a 5 V VIN.
- **I2C.** Q3 (dual BSS138) bridges host `SDA`/`SCL` to the sensor-side
  `SDA_3V`/`SCL_3V`, gates on 2.8 V. R7, a 10 kΩ ×4 resistor pack, pulls both
  sides up: host side to VCC (R7 pads 1/3 to 6/8), sensor side to 2.8 V
  (R7 pads 2/4 to 5/7). Despite the `_3V` names, the sensor side sits at 2.8 V.
  There is no solder jumper to remove the host-side pull-ups.
- **XSHUT.** The header pin goes to the cathode of D1 (1N4148); the anode is
  the sensor's XSHUT (U1 pad 5), held at 2.8 V by R1 (10 kΩ). A host driving
  low pulls XSHUT down through the diode; a host driving 3.3 V or 5 V high is
  blocked by it, and the sensor sees 2.8 V. Left unconnected, the sensor stays
  enabled. Adafruit: "active low, and is logic-level shifted so you can use 3V
  or 5V logic" (pinouts page).
- **GPIO.** The header pin is the sensor's GPIO1 (U1 pad 7) directly, with R2
  (10 kΩ) to 2.8 V and nothing between it and the pin. It is open drain (DS13812
  Table 2), so the high level a host sees is 2.8 V. Adafruit: "2.8V logic level
  output … can be read by 3.3V and most 5V logic microcontrollers" (pinouts page).
- **Power LED.** D2 (green, label `on` on the top face) runs from VCC through
  R3 and SJ1, a closed solder jumper on the bottom face (element `SJ1`, `MR0`).
  "Cut the trace on this jumper to cut power to the "on" LED" (pinouts page).

## Known issues and quirks

- **The sensor footprint is labelled VL53L0X** in both the board and the
  schematic (element `U1`, value `VL53L0X`, package `LGA12_ST`). The pads
  carry the VL53L4CD's pin functions (pad 5 XSHUT, 7 GPIO1, 9 SDA, 10 SCL,
  1 and 11 supply), and ST states the VL53L4CD is pin-compatible with the
  VL53L0X (DS13812 Features). The label is Adafruit's library reuse, not a
  different part; read the marking on the module of a fitted board.
- **Fixed address.** Each board answers only at 0x29, so two on one bus need
  separate mux channels (the plan) or XSHUT sequencing with an address change
  through the driver.
- **Pull-ups stack with the bus.** The host-side 10 kΩ pull-ups are in
  parallel with whatever the TCA9548A channel or carrier adds.

## In this repository

Not yet used. Planned for robocar, TCA9548A channels 5 and 6
(`packages/robotics/robocar/WIRING.md` lists ch5–7 as reserved for "IMU / ToF /
future sensors"). No `hardware.toml` entry until the checklist below is done.

## Source

Header order, nets, parts and silkscreen were read from Adafruit's Eagle board
file, not from a photo (`.claude/rules/board-layout-from-vendor-files.md`):
[`adafruit/Adafruit-VL53L4CD-PCB`](https://github.com/adafruit/Adafruit-VL53L4CD-PCB),
`Adafruit VL53L4CD.brd` at commit
`de475b8aede92ff30e66671ded9ec015fc0df6ba`, elements `JP2`, `CONN3` and
`CONN4`. Adafruit publishes the design under CC BY-SA (`README.md` and
`license.txt` at that commit). The pin descriptions quoted above are from
Adafruit's learn guide pinouts page.

```sh
gh api "repos/adafruit/Adafruit-VL53L4CD-PCB/contents/Adafruit%20VL53L4CD.brd?ref=de475b8aede92ff30e66671ded9ec015fc0df6ba" --jq '.content' | base64 -d > /tmp/vl53l4cd.brd
python3 tools/breakout-pinout.py /tmp/vl53l4cd.brd --element JP2 --element CONN3 --element CONN4
```

**Not checked on a fitted board:** everything. The board revision bought may
differ from the file at this commit.

## Check on the physical board

- [ ] The board is Adafruit #5396 and its silkscreen order matches the table
      (VIN, GND, SCL, SDA, GPIO, XSHUT, left to right on the top face).
- [ ] The module is marked as a VL53L4CD, despite the footprint's VL53L0X label.
- [ ] With VIN at 3.3 V, the sensor rail (C5 or U3 pad 5) reads about 2.8 V.
      The LP5907's dropout at the sensor's 40 mA peak (DS13812 Table 12 note 2)
      was not looked up, so 3.3 V in is unverified against the regulator.
- [ ] XSHUT driven low by an ESP32-S3 GPIO through D1 resets the sensor. The
      datasheet gives no XSHUT VIL (Table 13 covers only GPIO1 and I2C), so
      the low level after the diode drop has not been compared with a threshold.
- [ ] GPIO's 2.8 V high reads as high on the XIAO ESP32-S3. The ESP32-S3's VIH
      was not looked up for this page.
- [ ] The model ID register 0x010F reads 0xEB through the TCA9548A channel.

## References

- Product page: <https://www.adafruit.com/product/5396>
- Learn guide, pinouts: <https://learn.adafruit.com/adafruit-vl53l4cd-time-of-flight-distance-sensor/pinouts>
- Board files: <https://github.com/adafruit/Adafruit-VL53L4CD-PCB/tree/de475b8aede92ff30e66671ded9ec015fc0df6ba>
- CircuitPython driver: <https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CD>
- Sensor: [`sensor--vl53l4cd.md`](../datasheets/sensor--vl53l4cd.md)
