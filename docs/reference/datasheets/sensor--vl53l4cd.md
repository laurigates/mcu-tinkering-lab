# VL53L4CD — Time-of-Flight Proximity Sensor (ST, 940 nm, 0–1300 mm)

**Used in:** not yet. robocar-unified plans two, aimed down at the front
corners as cliff and table-edge detectors, on TCA9548A channels 5 and 6 (both
parts answer only at 0x29, so each needs a channel of its own). Nothing is
bought. Candidate breakout:
[`../boards/adafruit-vl53l4cd.md`](../boards/adafruit-vl53l4cd.md).

Values below are from ST's datasheet DS13812 **Rev 3 (January 2022)**, cited by
section or table, unless another source is named. Rev 3 is the copy SparkFun
hosts; ST's own download could not be fetched from this machine, and mirrors
list Rev 5 and Rev 6 (November 2022), which have **not** been read. See
[Open items](#open-items).

## Ranging principle

Direct time of flight: a 940 nm VCSEL (vertical-cavity surface-emitting laser)
emits, and a SPAD (single-photon avalanche diode) array receives; the module
"records an absolute distance measurement regardless of the target color and
reflectance" (Description, p. 2). The laser is Class 1 under IEC 60825-1:2014
provided ST's recommended settings are used, "particularly the maximum timing
budget" (§8). All processing runs in the module's firmware; the host reads a
distance, a signal rate and a status (§2.5, §2.6).

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| Package | Optical LGA12, 4.4 × 2.4 × 1 mm | Table 1 |
| Emitter | 940 nm VCSEL, Class 1 | Features; §8 |
| Supply AVDD | 2.6–3.5 V, 2.8 V typical | Table 10 |
| I/O level (IOVDD) | 1.6–3.5 V, 1.8 or 2.8 V typical | Table 10 |
| Absolute max, AVDD and SCL/SDA/XSHUT/GPIO1 | −0.5 to 3.6 V | Table 9 |
| Supply sequencing | None; I/Os are fail-safe "with no diode connecting them to AVDD" | §4.2 |
| Current, HW standby (XSHUT low) | 3 / 5 / 7 µA (min / typ / max) | Table 12 |
| Current, SW standby | 4 / 6 / 9 µA | Table 12 |
| Current, ranging average | 22 mA typ, 24 mA max; peaks reach 40 mA | Table 12 and note 2 |
| Boot time after XSHUT rises | 1.2 ms max | §2.7, notes under Figures 6 and 7 |
| Minimum distance | 0 mm detection; linear from 1 mm | §5.2 |
| Maximum distance (indoor, white 88 %, 33 ms budget) | 1200 mm at ≥90 % detection rate, 1300 mm at ≥50 % | Table 14 |
| Maximum distance (indoor, grey 17 %, 33 ms budget) | 450 mm at ≥90 %, 475 mm at ≥50 % | Table 14 |
| Maximum distance (outdoor overcast) | white 550 / 600 mm, grey 400 / 450 mm (≥90 % / ≥50 %) | Table 14 |
| Field of view (detection volume) | 22° at 100 mm, 18° at 1000 mm (white target, dark) | Table 16 |
| Collector exclusion cone | 25° | Table 16 |
| Emitter field of illumination | 16° × 16° (1/e²) | §7, Figure 22 |
| Ranging rate | Up to 100 Hz | Features |
| Timing budget | 10–200 ms, default 50 ms (driver) | ST ULD `vl53l4cd_api.h`, `SetRangeTiming` doc comment; `SensorInit` calls `SetRangeTiming(dev, 50, 0)` (`vl53l4cd_api.c` line 233) |
| I2C | Up to 1 MHz (Fast-mode Plus) | Table 1; §3; Table 5 |
| I2C address | 0x52 as the 8-bit write byte = **0x29** 7-bit | Table 1; §3, Figure 10 (bits `0101001`); Pololu `AddressDefault = 0b0101001` |
| Model ID | Register 0x010F reads 0xEB; 0x0110 reads 0xAA | Table 7 |
| Operating temperature | −30 to 85 °C | Table 1; Table 10 |
| Compression load | 25 N max | §9.6.3 |
| Moisture sensitivity | MSL 3 | §9.6.4 |
| Order code | VL53L4CDV0DH/1 | Table 19 |

### The 1200 vs 1300 mm disagreement

Both figures are in the datasheet. Table 14 gives 1200 mm as the typical range
on a white target at a 90 % minimum detection rate, and 1300 mm at a 50 %
rate. Rev 1 and Rev 2 quoted 1200 mm on the front page; the Rev 3 revision
history (Table 20) records "Modify the FoV range from 1200 mm to 1300 mm in
Section Features and Section Description". Vendor pages follow one or the
other: Adafruit's board silkscreen and README say "1 to 1300mm", and Pololu
sells its carrier as "120cm Max". Neither number matters for cliff detection,
which works below 200 mm.

## Pins

From Table 2 and Figure 2 (Figure 2 is a bottom view):

| Pad | Signal | Type | Notes |
|-----|--------|------|-------|
| 1 | AVDDVCSEL | Supply | VCSEL supply, to the main supply |
| 2 | AVSSVCSEL | Ground | VCSEL ground |
| 3, 4, 6, 12 | GND, GND2, GND3, GND4 | Ground | All to ground (Table 2 notes) |
| 5 | XSHUT | Input | Shutdown, **active low** |
| 7 | GPIO1 | Output | Interrupt, **open drain** |
| 8 | DNC | — | Do not connect; leave floating |
| 9 | SDA | I/O | I2C data |
| 10 | SCL | Input | I2C clock |
| 11 | AVDD | Supply | Main supply |

- **XSHUT** must always be driven, or pulled up when the host's state is
  unknown, "to avoid leakage current"; 10 kΩ is the recommended pull-up (§1.4).
  It may be high only while AVDD is on (§2.7; Table 10 note 1). Held low with
  AVDD present, the part is in HW standby (§2.7). Left tied to AVDD through the
  pull-up, the part boots straight to SW standby (§2.7, Option 2).
- **GPIO1** is driven low when a measurement is ready (§2.9). If the host does
  not read the result and clear the interrupt, ranging stops ("the ranging
  operation inside the device is on hold", §2.9). ST "strongly recommends" the
  pin over polling (§2.9); 10 kΩ pull-up recommended, leave unconnected if
  unused (§1.4). The ULD can also raise it only when a distance crosses a
  threshold (`VL53L4CD_SetDetectionThresholds`, windows below/above/outside/
  inside, ST ULD `vl53l4cd_api.h`).
- **I2C pull-ups** belong on the host side, once per bus (§1.4, §3). Suggested
  values for 1.8–2.8 V: 3.6 kΩ at CL ≤ 90 pF in Fast mode (Table 3), 1.5 kΩ
  plus 100 Ω series at CL ≤ 90 pF in Fast-mode Plus (Table 4). SDA/SCL
  VIH min is 1.12 V, VIL max 0.6 V (Table 13).

## Driver

The datasheet names the host driver as the **Ultra Lite Driver (ULD)**,
documented in user manual **UM2931**, and withholds the register map: "full
register details are not exposed" (§2.1, §3.2). ST's own
`STMicroelectronics/x-cube-tof1` release notes link the "VL53L4CD driver on
st.com" to `st.com/en/embedded-software/stsw-img026.html`, so the ULD's ST part
number is **STSW-IMG026** (Pololu's README names it the same way). The ULD
release bundled in x-cube-tof1 at the commit read is "VL53L4 ULD V2.1.1".

Ports that wrap the same code: `stm32duino/VL53L4CD` (Arduino, v1.0.5),
`pololu/vl53l4cd-arduino` (a re-implementation "based on" the ULD), and
`adafruit/Adafruit_CircuitPython_VL53L4CD`. No ESP-IDF port has been chosen;
the ULD's platform layer is a set of I2C read/write and wait functions.

Calibration: offset calibration is needed in every application, and crosstalk
calibration as well when a cover glass is fitted (§2.3; procedure in UM2931).
The ULD exposes `SetOffset` (−1024 to 1023 mm) and `SetXtalk` (0–128 kcps) (ST
ULD `vl53l4cd_api.h`). Range drift with temperature is an offset, not a gain;
the ULD's `StartTemperatureUpdate` re-runs it when temperature has moved by more
than 8 °C, with ranging stopped (§5.5; ST ULD `vl53l4cd_api.h`).

## Range status codes

`VL53L4CD_GetResult` reads register 0x0089, masks it to 5 bits and remaps the
raw value through a 24-entry table to the status it reports; a result with zero
active SPADs is reported as 255 (ST ULD `vl53l4cd_api.c`, `status_rtn`, lines
466–468; the stm32duino port has the identical table at
`src/vl53l4cd_api.cpp` lines 460–462). **Only status 0 is a valid distance**:
"If the status is equal to 0, the data are valid" (ST ULD `vl53l4cd_api.h`,
`VL53L4CD_ResultsData_t`).

The full status table is in UM2931, which has **not** been read (ST's server
refused the download). The meanings below are assembled from what each source
states:

| Status | Meaning | Source |
|--------|---------|--------|
| 0 | Valid | ST ULD header (above); Adafruit `RANGE_VALID` |
| 1 | Sigma (standard deviation of the return) above the sigma threshold | ST ULD header, `SetSigmaThreshold` comment ("will be equal to 1"); Adafruit `RANGE_WARN_SIGMA_ABOVE` |
| 2 | Signal below the signal threshold | ST ULD header, `SetSignalThreshold` comment ("will be equal to 2") |
| 3 | Distance below detection threshold | Adafruit `RANGE_ERROR_DISTANCE_BELOW_DETECTION_THRESHOLD` |
| 4 | Invalid phase | Adafruit `RANGE_ERROR_INVALID_PHASE` |
| 5 | Hardware fail | Adafruit `RANGE_ERROR_HW_FAIL` |
| 6 | No wrap-around check (warning) | Adafruit `RANGE_WARN_NO_WRAP_AROUND_CHECK` |
| 7 | Wrapped target, phase mismatch | Adafruit `RANGE_ERROR_WRAPPED_TARGET_PHASE_MISMATCH` |
| 8 | Processing fail | Adafruit `RANGE_ERROR_PROCESSING_FAIL` |
| 9 | Crosstalk fail | Adafruit `RANGE_ERROR_CROSSTALK_FAIL` |
| 10 | Interrupt error | Adafruit `RANGE_ERROR_INTERRUPT` |
| 11 | Merged target | Adafruit `RANGE_ERROR_MERGED_TARGET` |
| 12 | Signal too weak | Adafruit `RANGE_ERROR_SIGNAL_TOO_WEAK` |
| 13 | Produced by the remap table; named in no source read | ST ULD `status_rtn` (raw 13 → 13) |
| 255 | Other error, including zero SPADs | Adafruit `RANGE_ERROR_OTHER`; ST ULD `GetResult` |

**Sources disagree on 1 and 2.** ST's ULD header says sigma → 1 and signal → 2.
ST application note AN5870, as quoted in a search result, says a sigma above
threshold "returns an error of 2"; a third-party board guide (Robu R101050)
lists "1: Signal fail, 2: Sigma fail". Adafruit names constant 2
`RANGE_WARN_SIGMA_BELOW`, which matches neither. The ULD source is the code
that sets the value, so this page follows it, but UM2931 should settle it.

**Default thresholds also disagree.** The ULD header says the sigma threshold
defaults to 15 mm and the signal threshold to 1024 kcps; the comment on the
default-configuration table in the same driver says sigma "default value 90 mm"
(ST ULD `vl53l4cd_api.c` lines 89–90). The bytes there (0x00, 0xA0) would be
40 mm if the "14.2 format" the comment names holds (160 / 4); that decode is an
inference. Read `GetSigmaThreshold` and `GetSignalThreshold` on the bench.

## Ambient light and cover glass

- Ranging figures assume no cover glass, 23 °C, 2.8 V, the full FoV covered,
  and offset calibrated at 100 mm on a grey 17 % target (§5.1). "Indoor" means
  no light in 940 ± 30 nm; "outdoor overcast" is 10 kcps/SPAD of ambient,
  about 5 klux daylight (§5.1).
- Ambient IR shortens range: white-target range falls from 1200 to 550 mm
  outdoors at the 90 % rate (Table 14), and accuracy above 200 mm degrades
  from ±3 % to ±8 % (Table 15).
- A cover glass needs crosstalk calibration (§2.3). The cover-window opening
  must be at least as wide as the 25° collector exclusion cone (§7, Table 16).

## For cliff detection

ST lists "wall tracking and cliff detection for robotics" among the
applications (Features, p. 1). Relevant figures for a sensor 3–15 cm above
the floor:

- **Accuracy at 1–100 mm:** ±7 mm on white, ±6 mm on grey, indoors; ±8 / ±7 mm
  outdoor overcast. **At 101–200 mm:** ±8 mm indoors, ±9 mm outdoors, both
  targets (Table 15, 33 ms budget, at least 90 % of readings within it).
- **No minimum-distance blind zone:** detection from 0 mm, linear from 1 mm
  (§5.2), so a sensor mounted close to the floor is within its linear range.
- **Spot size:** at the 22° detection volume measured at 100 mm (Table 16) the
  sensed spot is about 2 × h × tan 11°, roughly 39 mm across at h = 100 mm and
  12 mm at 30 mm. This is arithmetic on Table 16, not a stated figure, and the
  table was measured against a target that fills the FoV.
- **Dark and glossy floors:** the datasheet characterises only grey 17 % and
  white 88 % targets (§5.1). It states nothing for surfaces darker than 17 %,
  or for specular (glossy, mirror-like) surfaces, beyond the general claim that
  ranging is independent of colour and reflectance (p. 2). A black or shiny
  floor may return status 2 or 12 (weak signal) while the floor is still
  there. Test the robot's actual floors and table tops before trusting a
  non-zero status as "edge".
- **What a cliff looks like:** past an edge the sensor either ranges the lower
  floor (a valid, much longer distance) or finds no target (a non-zero
  status). Treating any non-zero status as a cliff, as planned, is the
  fail-safe reading; also treat a valid distance above a threshold as a
  cliff, since a floor a metre down can still range as status 0 (Table 14:
  1200 mm on white).

## Datasheets & References

- **VL53L4CD datasheet DS13812 Rev 3 (ST, January 2022):** ST's URL
  <https://www.st.com/resource/en/datasheet/vl53l4cd.pdf> (not fetched; the
  server refused automated downloads); the Rev 3 copy read for this page,
  hosted by SparkFun: <https://cdn.sparkfun.com/assets/d/b/b/7/c/vl53l4cd.pdf>
- **ULD (STSW-IMG026):** <https://www.st.com/en/embedded-software/stsw-img026.html>;
  user manual UM2931 (not read)
- **ST X-CUBE-TOF1** (ULD source, `Drivers/BSP/Components/vl53l4cd/modules/`,
  and the STSW-IMG026 link in `Projects/Release_Notes.html` line 1503):
  <https://github.com/STMicroelectronics/x-cube-tof1/tree/c12709c3ac802d91dd2a215178f53cb94c024709>
- **stm32duino/VL53L4CD** (Arduino port of the ULD, v1.0.5):
  <https://github.com/stm32duino/VL53L4CD/tree/b6ed0e21735ccd0f220cd8f35cf896f9e11d463d>
- **pololu/vl53l4cd-arduino** (7-bit address `0b0101001`, `src/VL53L4CD.h`):
  <https://github.com/pololu/vl53l4cd-arduino/tree/bc865905fc97085f40ca9edab478210c01195fde>
- **Adafruit_CircuitPython_VL53L4CD** (status constants, `adafruit_vl53l4cd.py`
  lines 70–83; timing budget 10–200 ms, lines 291–296):
  <https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CD/blob/1196cb63efb310c3bed212cde2c0c842e5f9f5dc/adafruit_vl53l4cd.py>
- **AN5870**, *Using the VL53L4CD under ultra-low-power mode* (ST; not read,
  quoted from a search result):
  <https://www.st.com/resource/en/application_note/an5870-using-the-vl53l4cd-under-ultralowpower-mode-stmicroelectronics.pdf>
- **Robu R101050** (third-party board guide; status list):
  <https://st.robu.in/wp-content/uploads/2024/02/R101050-VL53L4CD.pdf>

## Open items

- [ ] Read DS13812's current revision from ST (Rev 5 and Rev 6 exist per
      mirrors) and check whether any value above changed since Rev 3.
- [ ] Read UM2931's range-status table: settle the meaning of statuses 1 and 2
      (ULD header vs AN5870), and name statuses 3–13.
- [ ] Read the default sigma and signal thresholds back from a sensor
      (15 mm vs 90 mm vs 40 mm decoded).
- [ ] Measure on the robot's floors (light, dark, glossy, and a table edge) at
      the planned mounting height: distance, status and signal rate.
- [ ] Choose the timing budget for cliff polling; 10 ms is the driver's
      minimum and 33 ms is what the datasheet tables assume.
