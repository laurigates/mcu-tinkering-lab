# VL53L8CX — 8×8 Multizone Time-of-Flight Sensor (ST)

**Used in:** nothing yet. Candidate for robocar-unified: one sensor aimed
forward and down to see floor edges and obstacles across the robot's width, on
a free TCA9548A channel (5–7). Nothing is bought. Candidate breakout:
[`../boards/pololu-vl53l8cx.md`](../boards/pololu-vl53l8cx.md).

A 6.4 × 3.0 × 1.75 mm optical LGA16 module containing a 940 nm VCSEL, a SPAD
receiver array and a microcontroller that runs firmware the host uploads at
every power-up (DS14161 Features, Table 1; UM3109 §4.1). ST publishes no
register map; the host drives it through ST's Ultra Lite Driver (ULD), package
**STSW-IMG040** (Pololu #3419 page; RJRP44/VL53L8CX-Library `README.md` lines
4–7, which link `st.com/.../stsw-img040.html`).

Unless noted, values are from **DS14161 Rev 7 (August 2023)** and **UM3109
Rev 7 (April 2024)**, read from the copies Pololu hosts (st.com did not answer
from this host). ST has published later datasheet revisions: a search result
lists Rev 8 (October 2024) and a July 2025 edition covering VL53L8CX/CP/CH.
Those were not read; see the checklist.

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| Zones | 4×4 (16) or 8×8 (64); 4×4 is the power-up default | DS Features; UM3109 §4.3 |
| Max ranging rate | 60 Hz at 4×4, 15 Hz at 8×8; min 1 Hz; default 1 Hz | UM3109 §4.4, Table 2 |
| Range | 2–400 cm per zone | DS Table 1 |
| Field of view (detection volume) | 45° horizontal, 45° vertical, 65° diagonal | DS §2.2, Table 2 |
| Collector exclusion zone (cover-glass opening) | 57.9° × 57.9°, 86.6° diagonal | DS Table 2 |
| Field of illumination | 43.4° × 43.4° at 75 % of peak; 57.9° × 57.9° at 10 % | DS §2.3 |
| Targets per zone | Up to 4 (`VL53L8CX_NB_TARGET_PER_ZONE`, set at compile time, default 1); two targets need ≥ 600 mm separation | UM3109 §4.10 |
| Target order | Strongest (default) or closest | UM3109 §4.9 |
| Ranging modes | Continuous (VCSEL on throughout) or autonomous (VCSEL on for the integration time; default 5 ms; the ULD's default mode) | UM3109 §4.5, §4.6 |
| Emitter | 940 nm VCSEL, Class 1 (IEC 60825-1:2014) | DS Features, §9 |
| Interface | I²C up to 1 MHz, or SPI up to 3 MHz (mode CPOL = 1, CPHA = 1) | DS Table 1, Table 6, Table 8, §5 |
| I²C address | 0x52 as the 8-bit write byte, so 0x29 7-bit; changeable, volatile | DS §4; Pololu #3419 ("0101001b"); stm32duino/VL53L8CX #22 |
| Supplies | AVDD 3.3 V, CORE_1V8 1.8 V, IOVDD 1.2 V or 1.8 V | DS Table 1, §3.3 |
| Operating temperature | −30 to 85 °C | DS Table 1, Table 10 |
| Order code | VL53L8CXV0GC/1, marking `L8A-` | DS Table 24, §10.1 |

### Supplies (DS Table 10, Table 9)

| Rail | Min | Typ | Max | Abs max |
|------|-----|-----|-----|---------|
| AVDD (analog + VCSEL) | 3.13 V | 3.3 V | 3.47 V | 3.47 V |
| CORE_1V8 | 1.62 V | 1.8 V | 1.98 V | 1.98 V |
| IOVDD, 1.2 V configuration | 1.08 V | 1.2 V | 1.32 V | 1.98 V |
| IOVDD, 1.8 V configuration | 1.62 V | 1.8 V | 1.98 V | 1.98 V |

- **All digital signals must be driven to the IOVDD level** (DS Table 3 note).
  The I/O is at most 1.98 V, so a 3.3 V host needs a level shifter.
- Apply all three rails together and remove them together; minimum slew
  0.001 V/µs on AVDD and 0.012 V/µs on CORE_1V8 and IOVDD (DS §3.3, Table 5).
  With IOVDD at 1.8 V, ST recommends one supply for IOVDD and CORE_1V8 (DS
  Table 10 note).

### Current (DS Table 12, 23 °C typical / worst-case max)

| State | AVDD | CORE_1V8 | IOVDD |
|-------|------|----------|-------|
| LP idle | 55 µA / 390 µA | 0.01 µA / 0.5 µA | 0.5 µA / 2 µA |
| HP idle | 1 mA / 1.6 mA | 3 mA / 17 mA | 0.0003 mA / 0.002 mA |
| Active ranging | 43 mA / 50 mA | 50 mA / 80 mA | 0.003 mA / 0.006 mA |

Peaks are the average plus 10 mA on each rail, and 4×4 or 8×8 makes no
difference (DS Table 12 notes). Continuous mode draws about 215 mW (Table 13);
autonomous at 1 Hz and 5 ms integration is 1.6 mW (4×4) or 6.7 mW (8×8)
(Table 14).

## Range: dark vs ambient light

Full-FoV target, 90 % detection rate, no cover glass, 5 kLux = 2 W/m² at
940 nm (DS §7.2.1). Typical / minimum, mm:

| Target | 4×4 @ 30 Hz, dark | 4×4 @ 30 Hz, 5 kLux | 8×8 @ 15 Hz, dark | 8×8 @ 15 Hz, 5 kLux |
|--------|------|------|------|------|
| White 88 %, inner zones | 4000 / 4000 | 2850 / 2850 | 4000 / 4000 | 1550 / 1100 |
| White 88 %, corner zones | 4000 / 4000 | 2850 / 2700 | 3950 / 2900 | 1400 / 1100 |
| Grey 54 %, inner | 4000 / 4000 | 2600 / 2550 | 3300 / 2350 | 1400 / 1000 |
| Grey 17 %, inner | 4000 / 4000 | 1650 / 1600 | 2450 / 1500 | 1150 / 900 |
| Grey 17 %, corner | 3950 / 3900 | 1550 / 1500 | 1950 / 1300 | 950 / 700 |

From DS Table 16 (4×4) and Table 17 (8×8); the light-grey corner rows are in
the tables and omitted here. Autonomous mode at 1 Hz and 5 ms integration
ranges shorter: 8×8 white inner 3600 mm dark, 1250 mm at 5 kLux (Table 20).

**Accuracy** (2.7 σ, continuous): ±8–12 mm from 20–200 mm, ±3–5 % from
200–4000 mm in the dark, worse in ambient (up to ±8 % for 17 % grey at 8×8)
(DS Table 18). Corner zones may be up to 4 % worse, and mounting tilt adds
typically 1–2 % (Table 18 notes). Offset drift after autocalibration is
typically 0.1 mm/°C; autocalibration runs at each ranging start (DS §7.4).

## Pins (DS §2.5, Table 3)

| Ball | Signal | Notes |
|------|--------|-------|
| A1 | INT | Interrupt, open drain, 47 kΩ pull-up to IOVDD required; pulses and self-clears after ~100 µs (UM3109 §5.3) |
| A2 | LPn | Low disables I²C while in low-power mode; used to give sensors on one bus distinct addresses. Unused or SPI: 47 kΩ pull-up to IOVDD |
| A3 | IOVDD | 1.2 V or 1.8 V |
| A4 | SDA / MOSI | I²C: 2.2 kΩ pull-up to IOVDD required |
| A5 | SCL / MCLK | I²C: 2.2 kΩ pull-up to IOVDD required |
| A6, A7, C6 | RSVD1–3 | Ground |
| B1 | SYNC | GPIO, open drain, 47 kΩ pull-up; a rising edge can start one acquisition (UM3109 §4.15) |
| B4 | THERMALPAD | Ground plane, for heat |
| B7 | CORE_1V8 | 1.8 V core supply |
| C1 | SPI_I2C_N | Low (47 kΩ pull-down) selects I²C, high (47 kΩ pull-up) SPI. Pulsing 0 → 1 → 0 resets the I²C interface only, not the sensor |
| C2 | NCS | SPI chip select, 47 kΩ pull-up; I²C: 47 kΩ pull-down |
| C3, C7 | GND | |
| C4 | AVDD | 3.3 V |
| C5 | MISO | SPI only, push-pull to IOVDD; I²C: leave unconnected |

**There is no I2C_RST pin.** DS Table 3's note says "toggling the I2C_RST pin
resets the sensor I²C communication only"; the table itself gives that function
to C1, SPI_I2C_N, and UM3109 §4.2 says the same ("Reset the I²C communication by
toggling the SPI_I2C_N pin"). Reading the note as a separate pin is an error.
The only full reset is a power cycle: all three rails low, wait 10 ms, rails
high (UM3109 §4.2).

## Firmware upload at init

- `vl53l8cx_init()` copies the firmware, "~84 Kbytes", over I²C or SPI and
  then boots it (UM3109 §4.1). In the ULD 1.3.0 source it is three writes of
  0x8000, 0x8000 and 0x5000 bytes, 86,016 bytes in all
  (stm32duino/VL53L8CX `src/vl53l8cx_api.c` lines 309–317 at `a93a9d6`).
- **It is lost on every power-off and in deep sleep**; sleep (LP idle) keeps it
  (UM3109 §4.7). So every boot pays the upload.
- **How long:** neither ST document gives a time. SparkFun measured the
  VL53L5CX, whose upload is the same ~86,000 bytes, at 9.4 s at 100 kHz with
  32-byte transfers, 2.8 s at 400 kHz, 2.5 s at 400 kHz with 128-byte
  transfers, and 1.4 s at 1 MHz with 128-byte transfers
  (sparkfun/SparkFun_VL53L5CX_Arduino_Library
  `examples/Example2_FastStartup/Example2_FastStartup.ino` lines 12–27 at
  `2486070`). No VL53L8CX measurement was found; at the same clock it should be
  comparable, since the byte count matches (inference).
- After the copy the ULD polls for the boot to finish with a 2 s timeout
  (`_vl53l8cx_poll_for_answer`, `vl53l8cx_api.c` lines 23–47).
- The ULD and the firmware image are coupled: a ULD version only works with the
  image it ships (mp-extras/vl53l5cx #6, maintainer comment: "The ULD and FW
  are coupled, so you cannot change one without the other").
- Program size: ST's API "typically uses over 90 KB of program memory"
  (Pololu #3419 page).

ULD versions seen: 1.3.0 in stm32duino/VL53L8CX 2.1.0 (`vl53l8cx_api.h` line
32, `VL53L8CX_API_REVISION "VL53L8CX_1.3.0"`); 2.0.0 in RJRP44/VL53L8CX-Library
(`README.md` lines 4–8). Which is current on st.com was not checked.

## Per-zone target status (UM3109 §5.5, Table 4)

| Status | Meaning |
|--------|---------|
| 0 | Ranging data not updated |
| 1 | Signal rate too low on SPAD array |
| 2 | Target phase |
| 3 | Sigma estimator too high |
| 4 | Target consistency failed |
| **5** | **Range valid** (100 % confidence) |
| 6 | Wrap-around not performed, typically the first range (50 %) |
| 7 | Rate consistency failed |
| 8 | Signal rate too low for the current target |
| 9 | Range valid with large pulse, possibly merged targets (50 %) |
| 10 | Range valid, but no target at previous range |
| 11 | Measurement consistency failed |
| 12 | Target blurred by another one, due to sharpener |
| 13 | Target detected but inconsistent data; frequent for secondary targets |
| 255 | No target detected (only if "number of targets detected" output is enabled) |

UM3109 gives 5 full confidence, 6 and 9 50 %, everything else below 50 %. ST
recommends keeping the "number of targets detected" and "target status" outputs
enabled to filter on (UM3109 §5.2). Driver return codes: 0 OK, 127 invalid
setting, 255 major error, usually an I²C/SPI timeout (UM3109 Table 5).

## Notes for a forward-and-down robot sensor

- **The image is flipped.** The Rx lens flips the scene horizontally and
  vertically: zone 0, bottom-left of the SPAD array, sees the top-right of the
  scene (DS §7.1.3, UM3109 §2.2).
- **One zone does not report both a floor edge and an obstacle closer than
  600 mm apart** (UM3109 §4.10 note).
- **Behind a window**, crosstalk below 60 cm can exceed the real return and
  read as a near target; calibrate with the Xtalk plugin against a target at
  600–3000 mm filling the FoV (UM3109 §3.2, Table 1).
- **Short range at low frame rate:** below 5 cm, stay at 5 Hz or more in
  continuous mode (UM3109 Table 2 note 1).
- **Indoors, multiple light paths can produce false targets**; with a timing
  budget above 100 ms ST recommends strongest-target order (UM3109 §4.6 note).
- **Per-frame I²C payload grows with resolution and enabled outputs.** The ULD
  sums the enabled output blocks per zone count (`vl53l8cx_api.c` lines
  503–583). With every output on and one target, an 8×8 frame is about
  1452 bytes; distance, status and target count alone about 328 bytes
  (computed from the `L5CX_*_SIZE` macros, `vl53l8cx_api.h` lines 193–256;
  estimate).

## Source disagreements

| Item | Sources | Settled |
|------|---------|---------|
| Field of view | Product page and DS Features: "65° diagonal"; UM3109 Introduction: "45° x 45° square FoV" | **No conflict.** DS Table 2 gives both for the same detection volume: 45° H, 45° V, 65° diagonal, measured at 1 m on an 88 % target in the dark, 8×8, 15 Hz (DS §2.2 notes). |
| Max SPI clock | DS Rev 7 Features, Table 1 and Table 8: **3 MHz**; UM3109 Rev 7 §2.3: "maximum capability of the SPI is 20 MHz"; Pololu: 3 MHz | **3 MHz.** DS revision history, Rev 3 (27 Apr 2023): "Modified SPI maximum value to 3 MHz in Features, Table 1 … and Table 8". UM3109 Rev 7 (Apr 2024) was revised four times after that without touching §2.3, so its 20 MHz is the pre-Rev-3 figure left in place. The data brief was not read (st.com unreachable). Pololu adds that below about 1.1 MHz SPI initialisation can fail ("in our experience"). |
| Sharpener default | DS §2.2 note: "14% sharpener (default value)"; UM3109 §4.8: "The default value is 5%" | **Open.** Read it back with `vl53l8cx_get_sharpener_percent()` after init. SG-O/VL53L_MZ_esphome uses 14 % (`README.md` line 96). |
| I²C address | DS §4: "device 8-bit address of 0x52"; Pololu: 7-bit 0101001b | Same address: 0x52 is the write byte, 0x29 the 7-bit address. ESP-IDF code passes `VL53L8CX_DEFAULT_I2C_ADDRESS >> 1` (RJRP44 `examples/ranging_basic/main/main.c` line 102). |

## Siblings (one line each)

- **VL53L5CX:** "essentially" the predecessor, with the same multizone
  capability and FoV, worse in ambient light, and without the VL53L8CX's SPI
  (Pololu #3419 page). Same ~86 kB firmware upload (SparkFun Example2 line 12).
  It needs its own ULD and firmware, not the VL53L8CX's (mp-extras/vl53l5cx
  #6; SG-O/VL53L_MZ_esphome ships separate `vl53l5cx_`/`vl53l8cx_api_wrapper.h`).
- **VL53L7CX:** 90° diagonal FoV, 3.5 m range (SG-O/VL53L_MZ_esphome
  `README.md` line 8); "similar but has a wider FoV and a slightly shorter
  range" (Pololu #3419 page). Its own ULD (`vl53l7cx_api_wrapper.h`, same repo).
  Pin compatibility with the VL53L8CX was not checked against its datasheet.

## Datasheets & References

- **DS14161 Rev 7, VL53L8CX datasheet (ST):** <https://www.st.com/resource/en/datasheet/vl53l8cx.pdf>; copy read: <https://www.pololu.com/file/0J2029/vl53l8cx.pdf>
- **UM3109 Rev 7, ULD user manual (ST):** <https://www.st.com/resource/en/user_manual/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf>; copy read: <https://www.pololu.com/file/0J2030/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf>
- **STSW-IMG040, VL53L8CX ULD (ST):** <https://www.st.com/content/st_com/en/products/embedded-software/imaging-software/stsw-img040.html>
- **Product page (ST):** <https://www.st.com/en/imaging-and-photonics-solutions/vl53l8cx.html>
- **stm32duino/VL53L8CX** (ULD 1.3.0 as an Arduino library), commit `a93a9d6796f2a74835a4088f225daec343153c62`: <https://github.com/stm32duino/VL53L8CX/tree/a93a9d6796f2a74835a4088f225daec343153c62>
- **RJRP44/VL53L8CX-Library** (ULD 2.0.0 for ESP-IDF), commit `e9d237001c1666d7284edfd161ffc40f6066302d`: <https://github.com/RJRP44/VL53L8CX-Library/tree/e9d237001c1666d7284edfd161ffc40f6066302d>
- **SparkFun VL53L5CX Arduino library, Example2_FastStartup** (upload times): <https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library/blob/248607060e49d18bdf064c6aa411ffe515993d16/examples/Example2_FastStartup/Example2_FastStartup.ino>
- **SG-O/VL53L_MZ_esphome** (ESPHome component for L5CX/L7CX/L8CX), commit `8122d57`: <https://github.com/SG-O/VL53L_MZ_esphome/tree/8122d57642f9335e5fc4b075e3611266062c9d30>
- **mp-extras/vl53l5cx #6** (ULD/firmware coupling): <https://github.com/mp-extras/vl53l5cx/issues/6>
- **stm32duino/VL53L8CX #22** (address survives an MCU reset): <https://github.com/stm32duino/VL53L8CX/issues/22>
