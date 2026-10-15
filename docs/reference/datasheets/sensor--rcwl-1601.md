# RCWL-1601 — Ultrasonic Rangefinder Module (3–5.5 V, HC-SR04 compatible)

**Module brand:** RCWL (logo and `RCWL-1601` on the transducer face). No
datasheet for the module, or for the ranging chip fitted to it, was found from
the maker; the facts below come from the sources listed under
[Datasheets & References](#datasheets--references), each cited where used.
**Used in:** robocar-unified — the obstacle reflex in
`packages/robocar/unified/main/reactive_controller.c`, driver in
`main/ultrasonic.c`, which today drives the module in GPIO mode (TRIG on GPIO3 /
XIAO D2, ECHO on GPIO4 / XIAO D3). The robot is moving to two rangefinders,
front and rear, both in I2C mode (see [I2C mode](#i2c-mode)). The repo's
`hardware.toml` still calls this part `HC-SR04P`. Physical header
layout: [`../boards/rcwl-1601.md`](../boards/rcwl-1601.md).

## Which chip is on it

The module is sold as "CS100A compatible", and the AliExpress listing shows a
loose CS100A chip. The board in the same listing's photos carries a 16-pin chip
`U2` marked `RCWL-9600` / `20200402` (listing photo 5), and its back face has
`UART` and `IIC` mode pads. The CS100A has no UART or I2C pins: its pin table
lists only the transducer, TRIG, ECHO, crystal, supply, two test outputs, an
unused input and a power-down pin (CS100A manual p. 2, §2, Table 1). So the mode
pads mean the fitted chip is not a CS100A, whatever the listing says. **Read
the marking on the fitted board.**

Everything below that is cited to the CS100A manual applies to the TRIG/ECHO
interface the CS100A defines and the module copies; whether an RCWL-9600 meets
the same timing has not been read in any source.

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| Supply | 3.0–5.5 V | Adafruit #4007 ("Voltage: 3V - 5.5V"); Tinytronics ("Voltage: 3.0-5.5V") |
| Logic level | Same at 3 V and 5 V supply | Adafruit #4007 ("the logic level will be the same") |
| Operating current | ≈2.2 mA | Adafruit #4007; Tinytronics |
| Range | 2–400 cm at 3.3 V, 2–450 cm at 5.0 V | Tinytronics |
| Recommended range | 10–250 cm | Adafruit #4007 |
| Resolution | 1 mm | Adafruit #4007; Tinytronics |
| Measurement cycle | 50 ms | Adafruit #4007; Tinytronics ("HC-SR04 has 200 mS") |
| Transducer frequency | 40 kHz | Adafruit #4007 |
| Outline | 40 × 18 mm | Adafruit #4007 diagram; Tinytronics; listing photos 1 and 6 |
| I2C address (I2C mode) | 0x57, 7-bit, fixed | Adafruit #4742 (the I2C-mode RCWL-1601) |

CS100A chip figures, for comparison with a board that does carry one:

| Parameter | Value | Source |
|-----------|-------|--------|
| Supply (AVDD, DVDD) | 3–5.5 V | CS100A manual p. 2, Table 1 (pins 4, 9); p. 4, §5 Table 2 |
| Operating current | 5.3 mA | p. 4, §5 Table 2 |
| Power-down current | 0.3 mA (PD pin high) | p. 4, Table 2; p. 2, Table 1 pin 16 |
| Range | 8 m | p. 4, Table 2 |
| Accuracy | 0.1 cm + 1 % | p. 5, Table 2 |
| Blind zone | < 2 cm | p. 5, Table 2 |
| Operating temperature | −40 to 85 °C | p. 4, Table 2 |
| Clock | 8 MHz crystal | p. 1, §1; p. 2, Table 1 pins 13–14 |

## Pinout (module header)

Silkscreen on the transducer face, left to right with `RCWL-1601` at the top
(listing photo 6; see the board page for the not-yet-verified physical order):

| Silkscreen | GPIO mode | UART mode | I2C mode |
|------------|-----------|-----------|----------|
| `Vcc` | Supply | Supply | Supply |
| `Trig` / `SCL` / `Rx` | TRIG in | RX | SCL |
| `Echo` / `Tx` / `SDA` | ECHO out | TX | SDA |
| `Gnd` | Ground | Ground | Ground |

Adafruit's I2C-mode RCWL-1601 gives the same order: "Power, SCL, SDA, Ground",
and has no pull-up resistors on the board (Adafruit #4742).

## Interface mode

The back face has two unpopulated resistor positions, `R5` beside the label
`UART` and `R4` beside `IIC` (listing photos 1 and 5). With both open the board
is expected to be in GPIO (HC-SR04 trigger/echo) mode, the mode it ships in.

No source covering the RCWL-1601 says what fitting both does. The closest documented sibling, Olimex's RCWL-1604, selects mode with two
resistor positions: both open is GPIO ("default like HC-SR04"), 10 kΩ on one is
I2C, 10 kΩ on the other is UART, 10 kΩ on both is 1-Wire; UART runs at
`9600,N,1`, and I2C is address 0x57 (write 0x01 to start, read three bytes
H, M, L) (Olimex SNS-RCWL1604 page). That is a different board with different
labels (`M1`/`M2`); treat it as a hint, not as this board's behaviour.

Adafruit's I2C-mode RCWL-1601 (#4742) is the same PCB with 10 kΩ (marked
`103`) on `R4` and `R5` empty; see the board page.

**For robocar-unified: fit 10 kΩ on R4, leave R5 empty (I2C mode).** Each
rangefinder sits alone on its own TCA9548A channel (3 and 4).

## I2C mode

The protocol below is from Adafruit #4742 and Olimex SNS-RCWL1604; the two
agree on everything except the conversion wait.

- **Address:** 0x57, 7-bit, fixed: "the sensor listens on 7-bit address 0x57
  (address cannot be changed)" (Adafruit #4742). Olimex gives the same address
  as the 8-bit write and read bytes 0xAE and 0xAF.
- **Trigger:** write one byte, `0x01`, to 0x57 (Adafruit #4742: "writing a '1'
  to I2C to start a ranging session"; Olimex: "write to address 0xAE 0x01
  start measurement").
- **Wait** for the conversion. See below.
- **Read:** three bytes from 0x57, most significant first (Olimex: "H,M,L").
  The distance in µm is `(H << 16) | (M << 8) | L`; divide by 1000 for mm
  (Adafruit #4742: "the 3 byte response which is appended together for the
  distance in um"; Olimex: "(H<<16 + M<<8 + L)/1000 in mm").
- **Pull-ups:** none on the module (Adafruit #4742: "there's no built-in
  pullup resistors"). The bus needs external pull-ups.
- **Bus sharing:** Adafruit #4742: "it doesn't like to share the I2C bus, we
  think it gets confused by other commands and can lock up the bus". Give it
  a bus, or a mux channel, of its own.

**Conversion wait.** Adafruit #4742 says to "wait a few milliseconds". Drivers
for sibling RCWL I2C modules wait much longer: M5Stack's Unit Sonic library
(RCWL-9620) calls `delay(120)` between the trigger and the read, and the
RCWL_1X05 library (RCWL-1605/1005) defaults to a 100 ms timeout, documented as
"100 ms is recommended, but lower values might work". The CS100A's
out-of-range echo of about 66 ms (see the historical section below) is a
further hint that a ranging cycle can take tens of milliseconds. Plan for
100–120 ms per reading until the bench measurement on the board page settles
it; none of these sources is a datasheet for the RCWL-9600.

## GPIO-mode timing (historical)

These facts describe the trigger/echo interface that `main/ultrasonic.c`
drives today. They no longer apply once the module runs in I2C mode, and are
kept for reference and for a board left in GPIO mode.

From the CS100A manual — the chip the module claims compatibility with:

- **Trigger:** a high pulse *longer than* 10 µs on TRIG starts a measurement
  (p. 2, Table 1, pin 8); "generally about 50 µs is recommended" (p. 3, §4).
  The chip then sends eight 40 kHz pulses (p. 3, §4).
- **Echo:** the width of the ECHO high pulse is the round-trip time (p. 2,
  Table 1, pin 7); distance = (high time × 340 m/s) / 2 (p. 4, §4).
- **Out of range:** ECHO still goes high, for about **66 ms** (p. 4, §4,
  Fig. 6).
- **Measurement cycle:** the next measurement can start once the ECHO pulse has
  ended, so the worst case is a little over 66 ms; 70 ms is enough (p. 4, §4).

The module listings give a 50 ms measurement cycle (Adafruit #4007,
Tinytronics), which does not cover a 66 ms out-of-range pulse if the fitted
chip behaves like a CS100A. Measure the out-of-range ECHO width on the fitted
board (see the board page).

## Application notes for this robot

- **3.3 V operation.** Supply is specified from 3.0 V (Adafruit #4007,
  Tinytronics), and the output logic level is the same at 3 V and 5 V supply
  (Adafruit #4007), so on the XIAO's 3V3 rail the bus needs no level shifter
  (in GPIO mode, ECHO needed none either).
  Range at 3.3 V is quoted as 2–400 cm against 2–450 cm at 5 V (Tinytronics).
- **Sensitivity resistor (CS100A boards).** The 22 MΩ pull-down `R2` in the
  reference circuit sets sensitivity: 22 MΩ recommended, 5.1–22 MΩ allowed;
  lowering it rejects interference and stray long-range readings of tens of cm
  to over a metre, at the cost of range (p. 3, §3).
- **Distance conversion (GPIO mode).** The firmware's `pulse_us / 58` is the same
  (340 m/s)/2 relation as p. 4 §4, expressed in µs per cm.

## Datasheets & References

- **CS100A manual V1.1 (Chinese, 苏州顺憬志联新材料科技有限公司,
  www.100sensor.com):** the copy vendored by LibDriver,
  <https://github.com/libdriver/cs100/blob/54bb666da38a7ab08e3c51153b23251487a9248a/datasheet/cs100_datasheet.pdf>
  (7 pages; page numbers above are PDF pages)
- **Adafruit #4007** (RCWL-1601, GPIO mode): <https://www.adafruit.com/product/4007>,
  dimension drawing <https://cdn-shop.adafruit.com/product-files/4007/4007_diagram.png>
- **Adafruit #4742** (RCWL-1601, I2C mode): <https://www.adafruit.com/product/4742>,
  back-face photo <https://cdn-shop.adafruit.com/970x728/4742-02.jpg>
- **Tinytronics RCWL-1601:** <https://www.tinytronics.nl/en/ultrasonic-sensor-rcwl-1601>
- **M5Stack Unit Sonic library** (RCWL-9620, I2C at 0x57; the `delay(120)`
  wait): <https://github.com/m5stack/M5Unit-Sonic/blob/7e0007bbc58ffafa8e8eb6d778c8051dd0b56c78/src/Unit_Sonic.cpp>
- **RCWL_1X05 library** (RCWL-1605/1005 I2C mode; the 100 ms timeout):
  <https://github.com/ftjuh/RCWL_1X05/blob/c63f3168298b795df93eab0734dded30ca872f7e/README.md>
- **Olimex SNS-RCWL1604** (sibling board, mode resistors):
  <https://www.olimex.com/Products/Components/Sensors/Distance/SNS-RCWL1604/>
- **Listing of the fitted module:** <https://www.aliexpress.com/item/1005001622236950.html>
  ("RCWL-1601 Ultrasonic Sensor 3-5V")
- **Not found:** a datasheet for the RCWL-9600 chip or the RCWL-1601 module
  from RCWL.
