# MPU-6050 — 6-Axis Gyroscope + Accelerometer (TDK InvenSense, I2C)

**Used in:** balancebot (fitted, on a [GY-521 breakout](../boards/gy-521.md),
I2C1 at 400 kHz, driver `packages/robotics/balancebot/src/mpu6050.c`);
glowbug (`packages/thinkpack/glowbug/main/imu.c`, I2C at 100 kHz, accelerometer
only). robocar reserves TCA9548A channels 5–7 for "IMU / ToF / future
sensors" (`packages/robotics/robocar/main/pin_config.h`, `WIRING.md`) but
assigns no IMU to a channel yet.

The MPU-6000 and MPU-6050 are one die in two packagings: "identical to each
other with two exceptions". The MPU-6050 is I2C only, at up to 400 kHz, and has
a separate VLOGIC pin that sets its interface voltage; the MPU-6000 adds SPI
and has no VLOGIC pin (RM §2). Values below apply to the MPU-6050 unless a row
says otherwise.

**Sources.** *PS* is the MPU-6000/MPU-6050 Product Specification
PS-MPU-6000A-00 **Rev 3.1** (2011-10-24); *RM* is the Register Map and
Descriptions RM-MPU-6000A-00 **Rev 4.0** (2012-03-09). Both were read from
SparkFun's mirrors (links below), because TDK's own datasheet URL redirects to
a site search for automated clients. TDK's current revision of the PS is not
the one read here; see [Source disagreements](#source-disagreements).

## Lifecycle: obsolete

TDK's product page lists **Product Status: Obsolete** and **Recommended
Alternate Part No.: ICM-42670-P**, with the note "(Interchangeability is not
guaranteed.)". The live page returned HTTP 403 to every client tried on
2026-11-08 (curl, WebFetch), so this is read from the Wayback Machine snapshot
of 2025-09-15
(<https://web.archive.org/web/20250915090618/https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=MPU-6050>).
DigiKey agrees: Part Status "Obsolete", "This product is no longer
manufactured" (DigiKey 4038010, read 2026-11-08). Neither page gives a PCN or
an end-of-life date.

What this means here: GY-521 modules are still sold, so the chips on them come
from old stock or from clones whose provenance no listing states. A new design
should start from the ICM-42670-P or another current part and its own
datasheet; it is not register compatible (TDK's own wording above), so the
drivers in this repo do not carry over. Whether the ICM-42670-P is itself
current was not checked.

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| VDD (operating) | 2.375–3.46 V | PS §6.3 |
| VDD (absolute maximum) | −0.5 to +6 V | PS §6.9 |
| VLOGIC (MPU-6050 only) | 1.71 V to VDD; must be ≤ VDD at all times | PS §6.3, §10.2 |
| VLOGIC (absolute maximum) | −0.5 V to VDD + 0.5 V | PS §6.9 |
| SCL, SDA, AD0, INT, AUX_DA input (absolute maximum) | −0.5 V to VDD + 0.5 V | PS §6.9 |
| VDD ramp | Monotonic, 10–90 % in ≤ 100 ms | PS §6.3 |
| Start-up time for register read/write | 20 ms typ, 100 ms max | PS §6.3 |
| Supply current, gyro + accel (DMP off) | 3.8 mA typ | PS §6.3 |
| Supply current, accel only | 500 µA typ | PS §6.3 |
| Full-chip idle (sleep) | 5 µA typ | PS §6.3 |
| Operating temperature (absolute maximum) | −40 to +105 °C; specified performance −40 to +85 °C | PS §6.9, §6.3 |
| ESD | 2 kV HBM, 200 V MM | PS §6.9 |
| Package | 24-lead QFN, 4 × 4 × 0.9 mm | PS §11.2 |

### Gyroscope

| FS_SEL (GYRO_CONFIG bits 4:3) | Range | Sensitivity | Source |
|---|---|---|---|
| 0 | ±250 °/s | 131 LSB/(°/s) | PS §6.1; RM §4.4 |
| 1 | ±500 °/s | 65.5 LSB/(°/s) | PS §6.1; RM §4.4 |
| 2 | ±1000 °/s | 32.8 LSB/(°/s) | PS §6.1; RM §4.4 |
| 3 | ±2000 °/s | 16.4 LSB/(°/s) | PS §6.1; RM §4.4 |

- Scale-factor tolerance ±3 % at 25 °C; initial zero-rate output ±20 °/s, and a
  further ±20 °/s over −40 to +85 °C (PS §6.1). That is why balancebot
  averages a gyro bias at rest (`mpu6050_calibrate_gyro()`, the `cal` command).
- Rate noise 0.005 °/s/√Hz at 10 Hz; total RMS noise 0.05 °/s with DLPF_CFG = 2
  (PS §6.1).
- ADC word length 16 bits; output data rate 4 Hz to 8 kHz (PS §6.1).

### Accelerometer

| AFS_SEL (ACCEL_CONFIG bits 4:3) | Range | Sensitivity | Source |
|---|---|---|---|
| 0 | ±2 g | 16,384 LSB/g | PS §6.2 |
| 1 | ±4 g | 8,192 LSB/g | PS §6.2 |
| 2 | ±8 g | 4,096 LSB/g | PS §6.2 |
| 3 | ±16 g | 2,048 LSB/g | PS §6.2 |

- Output in two's complement, 16 bits; initial calibration tolerance ±3 %;
  zero-g tolerance ±50 mg on X and Y, ±80 mg on Z (PS §6.2).
- Noise 400 µg/√Hz at 10 Hz, AFS_SEL = 0, ODR 1 kHz (PS §6.2; the PDF prints
  the unit as "g/√Hz", and TDK's product page gives 400 µg/√Hz).
- Output data rate 4 Hz to 1 kHz (PS §6.2). Above a 1 kHz sample rate the same
  accelerometer sample is repeated (RM §4.2).

## I2C interface

| Parameter | Value | Source |
|-----------|-------|--------|
| Role | Always a slave to the host | PS §7.10 |
| Clock | 100 kHz Standard-mode or 400 kHz Fast-mode, all registers | PS §6.4 |
| Address | `1101000` (0x68) with AD0 = 0; `1101001` (0x69) with AD0 = 1 | PS §6.4 |
| Logic level | Set by VLOGIC: VIH ≥ 0.7 × VLOGIC, VIL ≤ 0.3 × VLOGIC | PS §6.4, §6.5, §7.10 |
| SDA sink | 3 mA at VOL 0.4 V | PS §6.5 |
| Pin capacitance | < 10 pF per I/O | PS §6.5 |

`I2C_IF_DIS` (USER_CTRL bit 4): on the MPU-6050, "Always write 0" (RM §4.29).

### WHO_AM_I (0x75)

Reads **0x68** at power-on: bits 6:1 are `110100`, bits 7 and 0 read 0. It
holds the upper six bits of the address and **does not reflect AD0**, so a part
strapped to 0x69 still answers 0x68 (RM §4.34). A WHO_AM_I check therefore
proves the chip answered, not which address it is on; it has to be read at the
address in use.

## Power-up state

- **Every register resets to 0x00 except PWR_MGMT_1 (0x6B) = 0x40 and
  WHO_AM_I (0x75) = 0x68** (RM §3).
- 0x40 in PWR_MGMT_1 is `SLEEP` = 1: **the part powers up asleep** and returns
  no new data until SLEEP is cleared (RM §4.30).
- CLKSEL resets to 0, the internal 8 MHz oscillator. RM §4.30: "it is highly
  recommended that the device be configured to use one of the gyroscopes (or
  an external clock source) as the clock reference for improved stability"
  (CLKSEL = 1, 2 or 3 selects the X, Y or Z gyro PLL).
- `DEVICE_RESET` (bit 7) restores every register to the values above and clears
  itself (RM §4.30).
- With every register at 0x00: DLPF off, gyro ±250 °/s, accel ±2 g, sample rate
  divider 0, no interrupts enabled, FIFO off, INT active high push-pull pulsed
  (from RM §3 and the field descriptions in §4.3–§4.5, §4.15, §4.16, §4.29).

## Sample rate and DLPF

Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV), where the gyroscope
output rate is 8 kHz with the DLPF off (DLPF_CFG 0 or 7) and 1 kHz with it on
(RM §4.2). The sample rate drives the sensor registers, the FIFO, the DMP and
motion detection (RM §4.2).

`CONFIG` (0x1A) DLPF_CFG, applied to both sensors (RM §4.3):

| DLPF_CFG | Accel bandwidth / delay | Gyro bandwidth / delay | Gyro Fs |
|---|---|---|---|
| 0 | 260 Hz / 0 ms | 256 Hz / 0.98 ms | 8 kHz |
| 1 | 184 Hz / 2.0 ms | 188 Hz / 1.9 ms | 1 kHz |
| 2 | 94 Hz / 3.0 ms | 98 Hz / 2.8 ms | 1 kHz |
| 3 | 44 Hz / 4.9 ms | 42 Hz / 4.8 ms | 1 kHz |
| 4 | 21 Hz / 8.5 ms | 20 Hz / 8.3 ms | 1 kHz |
| 5 | 10 Hz / 13.8 ms | 10 Hz / 13.4 ms | 1 kHz |
| 6 | 5 Hz / 19.0 ms | 5 Hz / 18.6 ms | 1 kHz |
| 7 | reserved | reserved | 8 kHz |

balancebot uses DLPF_CFG = 2 and SMPLRT_DIV = 1: 1 kHz / 2 = 500 Hz, the
control-loop rate (`src/mpu6050.c`).

## INT pin

Configured in `INT_PIN_CFG` (0x37), all bits 0 at reset (RM §4.15, §3):

| Bit | 0 (reset) | 1 |
|---|---|---|
| `INT_LEVEL` (7) | active high | active low |
| `INT_OPEN` (6) | push-pull | open drain |
| `LATCH_INT_EN` (5) | a 50 µs pulse | held until the interrupt is cleared |
| `INT_RD_CLEAR` (4) | status cleared only by reading INT_STATUS (0x3A) | cleared by any read |
| `I2C_BYPASS_EN` (1) | host cannot reach the auxiliary bus | host reaches the auxiliary bus directly, if I2C_MST_EN = 0 |

- Output levels: VOH ≥ 0.9 × VLOGIC, VOL ≤ 0.1 × VLOGIC (1 MΩ load); in open
  drain, VOL ≤ 0.1 V at 0.3 mA (PS §6.4). The pulse is 50 µs with
  LATCH_INT_EN = 0 (PS §6.4).
- Sources: clock-source lock, data ready (FIFO and data registers), the
  accelerometer motion events, and an auxiliary-bus NACK (PS §7.18). Enable
  them in `INT_ENABLE` (0x38).
- balancebot writes 0x30 (active high, push-pull, latched, cleared on any read)
  and enables data ready only (`src/mpu6050.c`).

## FIFO

1024 bytes, accessible over the serial interface. `FIFO_EN` (0x23) selects what
is written: gyro, accelerometer, temperature, auxiliary-sensor data, FSYNC.
A counter (`FIFO_COUNT_H/L`, 0x72–0x73) tracks the valid bytes, and both count
registers latch when FIFO_COUNT_H is read; data is read by burst from
`FIFO_R_W` (0x74) (PS §5.3, §7.17; RM §4.32, §4.33). Neither driver in this
repo uses it.

## Temperature sensor

`TEMP_OUT_H/L` (0x41–0x42), 16-bit signed, updated at the sample rate (RM
§4.19):

    Temperature in °C = TEMP_OUT / 340 + 36.53

PS §6.3 gives the same scale as 340 LSB/°C (untrimmed) and an offset of −521
LSB at 35 °C (35 + 521/340 = 36.53, so the two documents agree). It measures
the die, range −40 to +85 °C, linearity ±1 °C (PS §6.3, §7.19).

A burst read of the sensor registers returns one sampling instant, because the
user-facing registers copy the internal set only while the serial interface is
idle (RM §4.19). balancebot reads accel, temperature and gyro as one 14-byte
burst from 0x3B; glowbug reads the six accelerometer bytes from 0x3B.

## Auxiliary I2C (XDA / XCL)

Pins 6 (AUX_DA) and 7 (AUX_CL), the GY-521's `XDA` and `XCL`, are a second I2C
bus for an external sensor such as a magnetometer (PS §7.1, §7.11):

- **Master mode** (`I2C_MST_EN`, USER_CTRL bit 5): the MPU-6050 reads up to 24
  bytes from up to four slaves into `EXT_SENS_DATA` (0x49–0x60), plus a fifth
  slave in single-byte mode (PS §7.11; RM §4.9–§4.13, §4.21). The master clock
  comes from a divider on the internal 8 MHz clock (`I2C_MST_CLK`, RM §4.8).
- **Pass-through mode** (`I2C_BYPASS_EN` = 1, `I2C_MST_EN` = 0): analog
  switches join AUX_DA/AUX_CL to SDA/SCL, so the host addresses the external
  sensor itself (PS §7.11; RM §4.15).
- **Logic level:** VLOGIC when `AUX_VDDIO` = 0 (its power-on value), VDD when 1
  (PS §10.2). Sink 1 mA at 0.4 V (PS §6.5).

Nothing in this repo uses the auxiliary bus; the balancebot carrier leaves XDA
and XCL unconnected (`docs/reference/parts/gy-521.toml`, `no_connect`).

## Pin notes (chip)

From PS §7.1 (MPU-6050 column) and the external parts in PS §7.3:

| Pin | Name | Note |
|-----|------|------|
| 1 | CLKIN | Connect to GND if unused |
| 6, 7 | AUX_DA, AUX_CL | Auxiliary I2C |
| 8 | VLOGIC | Digital I/O supply; 10 nF bypass (C4) |
| 9 | AD0 | Address LSB |
| 10 | REGOUT | 0.1 µF regulator filter capacitor (C1) |
| 11 | FSYNC | Connect to GND if unused |
| 12 | INT | Totem pole or open drain |
| 13 | VDD | 0.1 µF bypass (C2) |
| 18 | GND | |
| 19, 21 | RESV | Do not connect |
| 20 | CPOUT | 2.2 nF, 50 V charge-pump capacitor (C3) |
| 22 | CLKOUT | |
| 23, 24 | SCL, SDA | |
| 2–5, 14–17 | NC | Not internally connected |

## Source disagreements

- **VDD range.** PS §6.3 specifies 2.375–3.46 V; PS §7.20 describes the bias
  and LDO block's input as "an unregulated VDD of 2.1V to 3.6V". Design to the
  §6.3 table, which is the specification; §7.20 describes the internal block.
- **VLOGIC range.** PS §6.3 gives VLOGIC 1.71 V to VDD; TDK's product page
  lists "VDDIO I/O Power Supply / V 2.375 to 3.46" (2025-09-15 snapshot). The
  page's figure is the same as its VDD figure and may be a table copy; the
  datasheet table is the more specific source.
- **Datasheet revision.** A web-search summary of TDK's datasheet link
  describes it as Rev 3.4 (2013); the copy read here is Rev 3.1 (2011), and
  TDK's own URL could not be fetched. Values that changed between 3.1 and 3.4
  are not known. The register map read is Rev 4.0; whether TDK's current copy
  is later was not checked.

## Datasheets & References

- **Product Specification Rev 3.1** (PS-MPU-6000A-00), SparkFun mirror:
  <https://cdn.sparkfun.com/datasheets/Components/General%20IC/PS-MPU-6000A.pdf>
- **Register Map and Descriptions Rev 4.0** (RM-MPU-6000A-00), SparkFun mirror:
  <https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf>
- **TDK's datasheet URL** (redirects to a site search for automated clients on
  2026-11-08):
  <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf>
- **TDK product page** (HTTP 403 on 2026-11-08):
  <https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=MPU-6050>;
  Wayback snapshot 2025-09-15, cited above for the lifecycle status.
- **DigiKey 4038010** (status Obsolete):
  <https://www.digikey.at/en/products/detail/tdk-invensense/MPU-6050/4038010>
- Breakout used here: [`../boards/gy-521.md`](../boards/gy-521.md)
