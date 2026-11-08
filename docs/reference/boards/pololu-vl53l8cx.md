# Pololu VL53L8CX Time-of-Flight 8×8-Zone Distance Sensor Carrier (#3419)

**Candidate, not yet bought.** A breakout for ST's VL53L8CX multizone
time-of-flight sensor, with a 3.3 V and a 1.8 V regulator and an NXS0108 level
shifter, so it runs from one 3.2–5.5 V supply with I/O at the VIN level.
Considered for robocar-unified as a forward-and-down floor-edge and obstacle
sensor on a free TCA9548A channel (5–7). For the silicon see
[`sensor--vl53l8cx.md`](../datasheets/sensor--vl53l8cx.md).

## Why this board

No VL53L8CX breakout with Eagle files was found: SparkFun, Adafruit and DFRobot
have no VL53L8CX board on GitHub or in a web search (2026-11-08), and the
Pololu page lists none. So `tools/breakout-pinout.py` has no board file to read
for any candidate. Of the two vendor boards:

| Board | Published | Why / why not |
|---|---|---|
| **Pololu #3419** (this page) | Schematic PDF, dimension drawing PDF, STEP model, labelled top and bottom photos | Regulators and a level shifter on board, so it takes the robot's 3.3 V bus directly; 13 × 23 mm, two M2 holes |
| ST SATEL-VL53L8 | A "CAD Design Files" ZIP on st.com (format not checked: st.com did not answer from this host) | Header named in stm32duino/VL53L8CX `README.md`: SPI_I2C_n, LPn, NCS, MISO, MOSI_SDA, MCLK_SCL, PWREN, IOVDD, 3V3, 1V8, 5V, plus GPIO1 and GND. Its regulators and level shifting were not read |

## The board

- **Silkscreen:** top face `CORE/IOVDD`, `AVDD`, `VIN`, `GND`, `SDA/MOSI`,
  `SCL/MCLK`, `MISO`, `CS`, `INT`, and `GND`, `SPI/I2C`, `LP`, `SYNC` (Pololu
  pinout photo `0J12380`). Bottom face, mirrored: the same names, `© 2024`,
  `Pololu`, dev code `irs19b` and `J14398` (Pololu photo `0J12379`).
- **Size:** 12.7 × 22.9 mm (0.5″ × 0.9″), 0.7 g without headers; two
  Ø2.18 mm holes for #2/M2 screws, 2.5 mm in from the top and bottom corners on
  the right edge (top view) (Pololu dimension drawing, 14 Feb 2024; product
  specs).
- **Headers:** 13 holes, Ø1.02 mm, 2.54 mm pitch. A 9-pin row 1.27 mm in from
  the left edge (top view), and a 4-pin row 1.27 mm in from the right edge
  starting 8.89 mm below the top edge, level with the left row's fourth pin
  (dimension drawing, connector callouts).
- **The sensor ships under a protective liner** that has to be removed before
  use (Pololu photo `0J12381` caption; DS14161 §10.3).

## Pin mapping

Viewed from the component side, sensor up, mounting holes on the right: the
orientation of Pololu's pinout photo. `Side`/`Pos` follow
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py); `Pos 1` is the
top of each edge, so `R1` sits level with `L4`. `Header` is the connector and
pin in Pololu's schematic (`JP1`, `JP2`). The pin order comes from Pololu's
schematic, labelled photos and dimension drawing, which agree; there is no
board file to run `tools/breakout-pinout.py` on.

| Pin | Side | Pos | Header | Notes |
|-----|------|-----|--------|-------|
| CORE/IOVDD | L | 1 | JP1.1 | 1.8 V regulator output; feeds CORE_1V8 and IOVDD. About 50 mA spare |
| AVDD | L | 2 | JP1.2 | 3.3 V regulator output. About 200 mA spare |
| VIN | L | 3 | JP1.3 | Supply in, 3.2–5.5 V. Sets the I/O level of every signal pin |
| GND | L | 4 | JP1.4 | |
| SDA/MOSI | L | 5 | JP1.5 | Level-shifted |
| SCL/MCLK | L | 6 | JP1.6 | Level-shifted |
| MISO | L | 7 | JP1.7 | SPI only |
| CS | L | 8 | JP1.8 | Datasheet `NCS`. Pulled high |
| INT | L | 9 | JP1.9 | Level-shifted interrupt out |
| GND | R | 1 | JP2.1 | Beside `SPI/I2C`, for a shorting block |
| SPI/I2C | R | 2 | JP2.2 | Datasheet `SPI_I2C_N`. **Pulled high = SPI**; tie to GND for I²C |
| LP | R | 3 | JP2.3 | Datasheet `LPn`. Pulled high |
| SYNC | R | 4 | JP2.4 | External sync in |

Pin functions: Pololu #3419 product page, pinout section.

## Power and levels

From the Pololu product page unless noted.

- **VIN 3.2–5.5 V.** "A VIN voltage lower than about 3.4 V will cause the AVDD
  to drop below the nominal 3.3 V; however, the VL53L8CX accepts an AVDD
  voltage as low as 3.13 V." The sensor's minimum is 3.13 V (DS14161 Table 10).
  Neither regulator's part number is printed in the schematic ("3.3V LDO",
  "1.8V LDO"), so the dropout at VIN = 3.3 V is not known.
- **I/O level = VIN.** "The level shifter shifts all I/O lines to this level
  (and pulls them high by default)." The translator is an NXS0108 (schematic,
  `U4`) between the CORE/IOVDD and VIN rails, with a 10 kΩ resistor `R1` on its
  OE net (schematic; which rail feeds which side, and where `R1` returns, were
  not legible in the PDF text). Pull-up values are not given.
- **Supply current:** about 100 mA typical while ranging, peaks up to about
  150 mA (product specs). The silicon alone: AVDD 43 mA and CORE_1V8 50 mA
  typical while ranging (DS14161 Table 12).
- **Decoupling on board:** 1 µF + 0.1 µF at each regulator input, 0.1 µF + 1 µF
  on AVDD, 0.1 µF + 4.7 µF on the 1.8 V output, 0.1 µF + 0.1 µF + 4.7 µF at
  the sensor, 0.1 µF on each translator supply (schematic `C1`–`C13`).

## Interface

- **I²C:** drive `SPI/I2C` low, "a convenient way to do this is by connecting
  it to the adjacent GND pin (e.g. with a pair of header pins and a shorting
  block)". 7-bit address 0101001b (0x29); a new address lasts only until reset
  or power-off. Fast-mode Plus, 1 MHz (Pololu page).
- **SPI:** leave `SPI/I2C` high; MCLK, MOSI, MISO, CS. Up to 3 MHz, and "in our
  experience, the SPI frequency must be higher than about 1.1 MHz or problems
  with initializing the sensor can result" (Pololu page).
- **The level shifter is load-sensitive:** "more sensitive to external loads
  than the circuit we use for level shifting on most of our other I²C boards
  … keep connected communication wires short (ideally under 3 inches or 8 cm)
  and limit other devices on the same I²C or SPI bus. … adding a small amount
  of capacitance (a few tens of picofarads) to the interface lines can help
  prevent undesired signal oscillations" (Pololu page).

## Known issues and quirks

- **Pololu's page has two copy errors** in its SPI paragraph: "data out (MOSI)"
  for MISO, and "the SPI interface on the LPS22DF". The pinout section names
  the pins correctly.
- **Not for 8-bit MCUs:** "ST's API for the VL53L8CX typically uses over
  90 KB of program memory" (Pololu page).

## Working driver code for the ESP32-S3

What the code below states, separately from what this page infers from it
(`.claude/rules/working-code-as-a-source.md`):

| Source | Platform | Bus clock | What else it shows |
|---|---|---|---|
| RJRP44/VL53L8CX-Library @ `e9d2370` (Espressif component `rjrp44/vl53l8cx`, ULD 2.0.0) | ESP-IDF, new `i2c_master` driver; README wires a SATEL-VL53L8 to an ESP32-S3 | 1 MHz: `#define VL53L8CX_MAX_CLK_SPEED 1000000` (`include/platform.h` line 48), used as `scl_speed_hz` (`examples/ranging_basic/main/main.c` line 103) | Each ULD multi-byte write is one `i2c_master_multi_buffer_transmit()` (`src/platform.c` line 46), so each 32 KiB firmware chunk is one bus transaction. Example puts `VL53L8CX_Configuration` and `VL53L8CX_ResultsData` on `app_main`'s stack (`main.c` lines 111–112); README: on a stack overflow raise the main task stack "to at least `7168`" (line 75). README asks for ESP-IDF v6 (line 29) |
| SG-O/VL53L_MZ_esphome @ `8122d57` | ESPHome; "tested on the ESP32-C6 using the ESP-IDF and on the ESP32-S3 using both ESP-IDF and Arduino" (`README.md` line 11) | Examples set `frequency: 400kHz` (`example.yaml` line 44); README: "I²C bus configured at up to 1000 kHz" (line 29) | Multiple sensors on one bus via the LP pin (line 21) |
| stm32duino/VL53L8CX 2.1.0 @ `a93a9d6` (ST's Arduino port, ULD 1.3.0) | Arduino, any architecture (`library.properties`) | Not set by the library | Examples written for the SATEL board (`README.md`) |
| SparkFun VL53L5CX library @ `2486070` (sibling chip) | Arduino | Measured upload times per clock (Example2 lines 12–27) | Chunks transfers to the Wire buffer, 32 bytes by default, 128 on ESP32 (Example2 line 20; `src/SparkFun_VL53L5CX_IO.cpp` lines 48–56) |

**Through an I²C multiplexer:** no report was found, for this chip or the
VL53L5CX. Searched 2026-11-08: `gh search issues` for "VL53L8CX multiplexer",
"VL53L5CX TCA9548A", "VL53L5CX multiplexer", "VL53L5CX mux", and every
VL53L8CX and VL53L5CX issue filtered for mux, TCA, PCA9548, ESP32, I2C and
timeout. The nearest is a shared-bus crash: an ESP32-S3 board-support package
crashed "when multiple peripherals are enabled and one of them is loading large
amounts of data, for example the Firmware of a VL53L5CX sensor", because its
I2C wrapper used the bus even when its mutex take timed out
(Juventus-Technikerschule-HF/EduboardV2_SW_ESP32S3_Template_WithBSP #2).

**RAM** (estimates from the ULD 1.3.0 headers, not measured):
`VL53L8CX_Configuration` holds a 488-byte offset buffer, a 776-byte Xtalk
buffer and a temporary buffer of at least 1024 bytes, 1452 with every output
on at one target (`vl53l8cx_api.h` lines 164–165, 253–267, 288–292), so about
2.7 kB. `VL53L8CX_ResultsData` with every output on at one target is about
1.4 kB (the per-field sizes in UM3109 Table 3). The firmware image and default
configuration are constant arrays in `vl53l8cx_buffers.h`.

## For robocar-unified, if it is bought

Inferences from the facts above and from this repo's code; none is measured.

- **The mux caps the clock at 400 kHz, and the bus currently runs at 100 kHz.**
  The TCA9548A "is a Fast-mode part" and the whole bus runs at
  `I2C_MASTER_FREQ_HZ` 100 kHz because of the RCWL-1601s
  (`packages/robocar/unified/main/i2c_bus.c` lines 173–183,
  `main/pin_config.h` line 39). At 100 kHz the sibling's measured upload is
  9.4 s; at 400 kHz, 2.5–2.8 s (SparkFun Example2).
- **The upload holds the bus the motors use.** The PCA9685 on channel 0 is
  reached through the same upstream SDA/SCL. A driver that writes each 32 KiB
  chunk as one transaction, as RJRP44's does, keeps the bus for about 3 s per
  chunk at 100 kHz (32768 bytes × 9 bits ÷ 100 kHz; estimate). Either upload
  before the motors start, or split the platform's `WrMulti` into short
  transactions that release the bus lock between them.
- **An 8×8 frame at 15 Hz does not fit a 100 kHz bus with every output on:**
  about 1452 bytes a frame (silicon page) is about 0.13 s a read at 100 kHz
  (estimate). Disable the outputs the robot does not use in the platform
  header (UM3109 §5.2), or run 4×4.
- **VIN on the 3.3 V rail.** VIN sets the I/O level, so it has to match the
  mux channel's pull-up voltage. At VIN = 3.3 V, AVDD sits below 3.3 V by the
  regulator's dropout; whether it stays above the sensor's 3.13 V is to
  measure.
- **Channel pull-ups:** the TCA9548A's downstream channels have none
  ([`adafruit-tca9548a.md`](adafruit-tca9548a.md) § Pull-ups). Whether the
  NXS0108's own pull-ups are enough for I²C on the channel, and whether adding
  the carrier's usual 2.2 kΩ upsets the translator Pololu calls load-sensitive,
  is to test.
- **One sensor per channel needs no address change and no LP wiring;** leave
  `LP` pulled high. `INT` is optional: the ULD can poll
  `vl53l8cx_check_data_ready()` (UM3109 §5.3).

## In this repository

Not yet used. No `hardware.toml` names it.

## Source

- **Pin order and names:** Pololu's schematic PDF (connectors `JP1` 1–9 and
  `JP2` 1–4), its labelled top-view pinout photo, its bottom-view photo and its
  dimension drawing. All four agree. No board file exists to run
  `tools/breakout-pinout.py` on.
- **Electrical and interface facts:** Pololu #3419 product page and schematic;
  sensor limits from DS14161 Rev 7.
- **Driver facts:** the repositories in the table above, at the commits named.
- **Not checked on a fitted board: everything.** The board has not been
  bought.

## Check on the physical board

- [ ] Silkscreen order matches the table: `CORE/IOVDD` at the top of the
      9-pin row, `GND` at the top of the 4-pin row, level with the 9-pin row's
      `GND`
- [ ] AVDD voltage with VIN on the robot's 3.3 V rail, idle and while ranging
      (must stay ≥ 3.13 V)
- [ ] Regulator part markings (U1, U2), for their dropout
- [ ] SDA/SCL idle level and the NXS0108's pull-up strength with nothing
      attached; whether I²C on a TCA9548A channel works with no extra pull-ups,
      and with 2.2 kΩ added
- [ ] `SPI/I2C` shorted to the adjacent GND: the sensor ACKs 0x29 on its mux
      channel
- [ ] Firmware upload time through the mux at 100 kHz and at 400 kHz
- [ ] Sharpener default read back after init (DS says 14 %, UM3109 says 5 %)
- [ ] Supply current while ranging, against Pololu's ~100 mA typical, 150 mA
      peak
- [ ] Liner removed before the first ranging test

## References

- Product page: <https://www.pololu.com/product/3419>
- Schematic (PDF): <https://www.pololu.com/file/0J2031/vl53l8cx-time-of-flight-sensor-schematic.pdf>
- Dimension drawing (PDF): <https://www.pololu.com/file/0J2032/vl53l8cx-carrier-dimensions.pdf>
- 3D model (STEP): <https://www.pololu.com/file/0J2033/vl53l8cx-carrier-model.step>
- Pinout photo: <https://a.pololu-files.com/picture/0J12380.1200.jpg>
- Bottom view with dimensions: <https://a.pololu-files.com/picture/0J12379.1200.jpg>
- ST SATEL-VL53L8: <https://www.st.com/en/evaluation-tools/satel-vl53l8.html>
- RJRP44/VL53L8CX-Library: <https://github.com/RJRP44/VL53L8CX-Library/tree/e9d237001c1666d7284edfd161ffc40f6066302d>
- SG-O/VL53L_MZ_esphome: <https://github.com/SG-O/VL53L_MZ_esphome/tree/8122d57642f9335e5fc4b075e3611266062c9d30>
- stm32duino/VL53L8CX: <https://github.com/stm32duino/VL53L8CX/tree/a93a9d6796f2a74835a4088f225daec343153c62>
- EduboardV2 BSP #2 (shared-bus crash during a VL53L5CX upload): <https://github.com/Juventus-Technikerschule-HF/EduboardV2_SW_ESP32S3_Template_WithBSP/issues/2>
