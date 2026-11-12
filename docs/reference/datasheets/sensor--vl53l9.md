# VL53L9CX — ST dToF 3D Lidar Module (54 × 42 zones)

**Used in:** nothing yet. It is a candidate depth sensor for
[robocar](../../../packages/robotics/robocar/CLAUDE.md), whose sensors
sit on a TCA9548A behind the [XIAO ESP32-S3 Sense](../boards/xiao-esp32s3-sense.md).
Nothing has been bought. See [Fit for robocar](#fit-for-robocar-esp32-s3)
for the verdict.

ST's FlightSense direct time-of-flight module: a 54 × 42 SPAD return array, two
940 nm VCSELs, a processing SoC and a PMIC in one reflowable optical LGA. ST
announced it as "VL53L9" in 2024. The datasheet and the only order code name
it **VL53L9CX**. Unless marked otherwise, the values below come from datasheet
**DS14879 Rev 7 (10 July 2026)**, with the section or table given. That revision
does not carry the word "prerelease" anywhere (checked with
`grep -i prerelease` on the `pdftotext` output).

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| Zones (binning → output) | 54×42 (2), 24×20 (4), 18×14 (6), 12×10 (8), 8×6 (12), 4×4 (24) | §3.2; Table 12 |
| Field of view | 55° H × 42° V (71° diagonal); collector exclusion zone 62° × 48° | p. 1 features; Table 3 |
| Field of illumination | 57.6° × 43° | Table 4 |
| Range | 50–8800 mm per zone; precision mode < 5 cm to 8.8 m, ambient mode 45 cm to 8.5 m | Table 2; §3.1 "Device ranging modes" |
| Frame rate | Up to 100 Hz | Table 2; p. 1 |
| Emitter | 2 VCSEL, 940 nm, Class 1 | Table 2; p. 2 description |
| Outputs | Depth, amplitude, ambient (each UINT16 per zone) plus a 4-bit DSS index per zone and a 100-byte status line | §5.1, Table 12 notes 2–3 |
| Control interface | I²C up to 1 MHz (Fm+), address 0x52 8-bit / 0x29 7-bit; I3C 12.5 MHz | Table 2; §4.1 |
| Output interface | MIPI CSI-2, one data lane, 200 Mbps–1 Gbps; or I3C 12.5 Mbps; or reading the frame over I²C | Table 2; §5.2; §5.3 |
| AVDD | 2.8 V or 3.3 V ± 5 % (driver default 2.8 V); abs. max 3.63 V | Table 8 note 2; Table 15 |
| DVDD | 1.2 V ± 5 %; abs. max 1.26 V | Table 8; Table 15 |
| IOVDD | 1.2 V or 1.8 V ± 10 % (driver default 1.8 V); **abs. max 1.98 V** | Table 8 note 1; Table 15 |
| VBAT_RX, VBAT_LDD | 2.8–4.8 V, no configuration needed | Table 8 note 3; Table 16 |
| AP_CLK | Required, 6–27 MHz at IOVDD level, ± 100 ppm, ≤ 200 ps jitter | §2.11; Table 6 pin E11 |
| Ranging current, typ (max) | AVDD 20 (40) mA, DVDD 50 (120) mA, IOVDD 5 (10) mA, VBAT_LDD 110 (250) mA, VBAT_RX 20 (250) mA; peaks may be 50 % above max | Table 18 and notes |
| Power, 54×42 | 225–235 mW indoor; up to 820 mW max (ambient mode, sunny, 62 % target) | Table 36 |
| Operating temperature | −30 to 70 °C ambient | Table 14 |
| Size | 12.83 × 6.10 × 4.64 mm | Table 2; §9 |
| Order code | VL53L9CXV0VE/1, optical LGA, tape and reel, MOQ 1000 | Table 39 |

### Profiles ST quotes (DS14879 Table 9)

| Profile | Resolution | Mode | Frame rate | Max range | Typ. power |
|---|---|---|---|---|---|
| Gaming | 54×42 | Precision (no DSS) | 100 fps | 5 m | 420 mW (5 klx) |
| Room mapping | 54×42 | Ambient, ULP | 30 fps | 8 m | 200 mW (5 klx) |
| AR glasses | 54×42 | Ambient, LP | 20 fps | 3 m | 600 mW (100 klx) |
| Autofocus | 24×20 | Precision, ULP | 15 fps | 8.8 m | 80 mW (indoor) |
| Wake on approach | 12×10 | Ambient, ULP | 1 fps | 8.5 m | 12.5 mW (dark) |

So the 100 fps headline comes with a 5 m range limit, and full range at 54 × 42 is quoted at 30 fps.

## The 2024 and 2026 numbers

| Claim | Feb 2024 announcement | 2026 launch coverage | DS14879 Rev 7 |
|---|---|---|---|
| Name | "VL53L9" | VL53L9CX | VL53L9CX |
| Zones | 54×42 = 2,268 ("2.3k") | 54×42, "2.3K" | 54×42 (§3.2) |
| Frame rate | 60 fps (Hackster) | 100 Hz (CNX Software) | Up to 100 Hz (Table 2). The 60 fps/60 Hz figure survives only in the application list on pp. 1–2 |
| Range | 5 cm to 10 m (Hackster) | "5 cm to 9 m" in CNX's intro; "< 5 cm to 8.8 m" in its spec list | 50–8800 mm (Table 2) |
| FoV | — | 55° × 42°, 71° diagonal; CNX's intro says 72° | 55° × 42°, 71° diagonal |
| Availability | Sampling to lead customers, GA "early 2025" (Hackster) | Mass production from early July 2026 (CNX, 22 June 2026) | First public datasheet revision 18 May 2026 (Rev 5, revision history) |

The datasheet supports **100 Hz, 54 × 42, 55° × 42°, up to 8.8 m**. None of the
sources here gives 9 m, 2,304 zones or 54° × 42°: 54 × 42 is 2,268, and
54° matches the zone count, not the FoV. The 2025 GA date slipped: the datasheet's
first public release is May 2026.

## Lifecycle and availability

- **ST product page status: not read.** st.com drops automated requests
  (HTTP/2 stream reset, then timeouts). Two secondary readings disagree. A search-engine
  snippet of the ST page says marketing status "Evaluation", package
  "OPTICAL LGA 12.1X5.1X4.5MM". jaredwatkins.com, an AI-maintained page that
  labels itself possibly inaccurate, quotes the ST page as "Product is in volume
  production" (October 2026) and a longevity commitment to 07/2033. The datasheet
  only says "currently available in the format below" (§14). **Open — read
  <https://www.st.com/en/imaging-and-photonics-solutions/vl53l9cx.html> in a
  browser.**
- **Module at retail:** the order code's MOQ is 1000 on tape and reel (Table 39).
  A DigiKey keyword search shows one part in "Optical Sensors", but the
  page shows no row detail, so loose-module stock and price are open.
- **Evaluation board:** STEVAL-VL53L9 is "Active" at DigiKey (497-STEVAL-VL53L9-ND),
  €71.36 ex VAT, 0 in stock with 1 expected 13 Nov 2026, 4-week lead time
  (DigiKey page, read 2026-11-08). See [Boards](#boards).

## Pins (module, Table 6)

The module has 42 pads, but only the host-facing ones are listed here.

| Pad | Name | Notes |
|---|---|---|
| A11 / A12 | SCL / SDA | I²C or I3C. Pull-ups on the host side; 2.2 kΩ is typical at 1 Mbps (§2.7 note). No pull-ups when I3C is used |
| A10 | INTR | Frame-ready interrupt |
| B12 | XSHUT | Drive high to enable |
| D12 | SYNC_IN | Drive low to trigger a frame (external sync mode) |
| E11 | AP_CLK | 6–27 MHz device clock |
| A5/A6, A8/A9 | DATA_P/N, CLK_P/N | MIPI CSI-2, 100 Ω differential (Table 7). Clock lane 100–500 MHz |
| E6/E7, C12, E8, B1, D1 | AVDD, DVDD, IOVDD, VBAT_LDD, VBAT_RX | Five supplies (§3.1 power-up). AVDD, DVDD and IOVDD can rise in any order |

All digital I/O, AP_CLK included, references IOVDD: V_IH ≥ 0.7 × IOVDD for
I²C/XSHUT/INTR/SYNC_IN and ≥ 0.9 × IOVDD for AP_CLK (Tables 19–20). The
absolute-maximum table (Table 15) lists only the supplies, with no input-pin
rating. With IOVDD at 1.98 V abs. max, assume the pins don't tolerate 3.3 V. The STEVAL board fits level shifters on every
host line (below), and the VanBruce driver says outright that "3.3 V on any pin can kill it".

## Interface behaviour that decides the host code

From DS14879 §4.1–4.2 unless noted:

- **No repeated start between a write and a read of the same register.** The
  driver issues firmware commands as a write to 0x400 then a read-back, and
  this must be START/WRITE/STOP followed by START/READ/STOP. ESP-IDF's
  combined `i2c_master_transmit_receive()` issues a repeated start, so it is
  the wrong call here.
- **On a communication error the device NACKs everything** until the host sends
  a STOP.
- **"Start condition with stop immediately after is not supported."** The
  VanBruce driver README warns that `i2cdetect`'s empty probes can wedge the
  device. A firmware bus scan across a mux channel the sensor sits on would do
  the same.
- 16-bit register index, big-endian; 32-bit register payloads, little-endian (§4.1, Table 11).
- ST recommends an I3C host for bandwidth. Reading frames over I²C works, but
  "the achievable frame rate is lower depending on the final map resolutions"
  (§3.1 "Output interfaces"; §2.7 last note). With I²C/I3C output "the data is not
  updated until the host acknowledges the frame" (§5.3).

### Frame size (Table 12)

Each frame holds 3 UINT16 arrays (depth, amplitude, ambient), the DSS map at
4 bits per zone, and the 100-byte status line. ST's driver reads exactly that
(`RAW_BUFFER_SIZE` in `vl53l9.c` line 84).

| Output | Bytes per frame | I²C wire time at 100 kHz | 400 kHz | 800 kHz | 1 MHz |
|---|---|---|---|---|---|
| 54×42 | 14 842 | 1.34 s | 334 ms | 167 ms | 134 ms |
| 24×20 | 3 844 | 346 ms | 86 ms | 43 ms | 35 ms |
| 18×14 | 1 738 | 156 ms | 39 ms | 20 ms | 16 ms |
| 12×10 | 880 | 79 ms | 20 ms | 10 ms | 8 ms |

The wire times are computed as bytes × 9 clocks ÷ bus frequency, ignoring
addressing, the DSS map/unmap commands and the ACK command. Treat them as a
lower bound. One measurement agrees: the VanBruce driver reports "~14.9 KB ≈
150–170 ms at 1 MHz, ~400 ms at 400 kHz" for a full frame on a Raspberry Pi 5.

### Data format notes

- **Depth valid flag:** bit 15 of each depth word is a valid flag and the low 15
  bits are millimetres. The VanBruce driver README says the datasheet does not
  document this and that it was confirmed on hardware. Not checked here.
- **Radial distance:** the characterization in §8.2 applies "the radial to
  perpendicular conversion", so the raw depth is presumably radial. This is
  inferred from the test conditions, not stated as a data format.
- **Image orientation:** the Rx lens flips the image horizontally and vertically (§8.1).
- **Post-processing:** confidence and reflectance maps are "generated in
  postprocessing" on the host (p. 1 features), and ST "recommends that the
  customer use the software processing pipeline provided" (§3.4). The data
  format is in technical note TN1596, not read here. The pipeline sources in
  `kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2` (`Firmware/vendor/vl53l9_processing/st/`:
  confidence, flying-pixel, radial-to-perpendicular, reflectance, ratenorm) are under
  **ST licence SLA0111**, not BSD (that repo's README, § License).

## Driver and SDK

- **ST low-level driver ("ULD") v1.0.0:** `vl53l9.c/.h`, `vl53l9_reg.h`, `vl53l9_platform.h`
  and `vl53l9_patch.h` (BSD-3-Clause per the copies below). The integrator implements
  `vl53l9_platform.h`: byte read/write at a 16-bit address, `wait_ms`, and three
  getters for the IOVDD level, AVDD level and AP_CLK frequency (lines 33–49). The
  driver is bus-agnostic.
  Public copy: `STMicroelectronics/fp-sns-datalog2`, `Drivers/BSP/Components/vl53l9/`
  at commit `63059c6e` (2026-10-02), used by `Projects/NUCLEO-H563ZI/Applications/DATALOG2_VL53L9`.
  A community port says the same sources come in ST package **STSW-IMG053**
  (`76EHwan/ESP32_STEVAL_VL53L9`, `docs/st-community-question.md`). That package
  name has not been checked on st.com.
- **Boot loads a firmware patch:** `vl53l9_init()` writes `g_vl53l9_fw_patch[]`,
  9 865 bytes (patch version 0.17, `vl53l9_patch.h` lines 27–30), over the
  control bus after every XSHUT cycle. Then it boots the device and checks the
  patch revision (`vl53l9.c` lines 178–192). Calibration data is 2 332 bytes
  (`VL53L9_CALIB_DATA_SIZE`, `vl53l9.h` line 62).
- **Frame period** is 10 ms–1 s in autonomous sync mode (`vl53l9.h` lines 249–252).
  Binning values accepted by the setter are 2, 3, 4, 6, 8, 12 and 24 (line 283).
  The driver's size table and DS §3.2 have no entry for 3.
- **Portability defects in ST's `vl53l9.c`** (commit `63059c6e`), reported by
  `76EHwan/ESP32_STEVAL_VL53L9` and confirmed by reading ST's file:
  - 1-byte enum locals are filled through `vl53l9_read8(..., (uint8_t *)&state)`,
    e.g. `_get_fsm_state()` (lines 866–870) and the `vl53l9_context_t context`
    locals (lines 649, 714, 762, 990). Arm EABI uses 1-byte enums, but on a
    4-byte-enum toolchain the upper three bytes are uninitialized stack. Whether
    ESP-IDF's Xtensa GCC uses 4-byte enums has not been checked here;
    zero-initializing the locals fixes it either way.
  - `_init_default_config()` ends with `vl53l9_read32(..., VL53L9_REGADDR_CAB_DIST_SCALE, &data)`
    after setting `data`, where a write was evidently intended (line 985).
  - `vl53l9_get_status()` reads all five LDD status bytes into
    `status->laser_driver` index 0 (line 821).

## Boards

- **STEVAL-VL53L9 (ST):** the module on a small board with 3.3 V in, ST LDK130 LDOs
  for 2.8 V (AVDD), 1.8 V (IOVDD) and 1.2 V (DVDD), two PI4ULS3V204 4-bit
  level shifters between a jumper-selected host IOVDD (J3/JP3: 3.3 V or 1.8 V) and
  the sensor's 1.8 V, a 12 MHz ECS-2520MVLC oscillator (Y1), an M24C64 EEPROM, a
  22-way 0.5 mm FFC for MIPI (J1), and an 8×1 2.54 mm header (J2) (STEVAL-VL53L9
  BOM Rev 2, May 2026; schematic Rev 1, March 2026, pp. 2–5). J2, by schematic pin
  number: 1 GND, 2 SDA, 3 SCL, 4 AP_CLK, 5 XSHUT, 6 INTR, 7 SYNC_IN, 8 3V3
  (schematic p. 2). Host-side SDA/SCL pull-ups R7/R8 are 2.2 kΩ to the host IOVDD
  (schematic p. 4; BOM item 3). The BOM fits R25, and the schematic note reads "Fit OE
  resistor to disable oscillator" (p. 5). So as shipped, AP_CLK has to come from
  the host through the level shifter (R23). Two community sources (VanBruce README;
  kamibukuro5656 README) say to remove R23 and R25 to run from Y1.
- **X-NUCLEO-53L9A1 (ST):** Arduino UNO R3 form-factor expansion board, $100 per CNX
  Software. Its schematic was not read.
- **Pololu, SparkFun, Adafruit, DFRobot, Pimoroni:** no VL53L9 product turned up
  in a web search (2026-11-08). That is a search-level negative; their catalogues
  were not checked one by one.
- **Community designs:** `tokla-chiya/VL53L9_KICAD` (an I²C-only breakout schematic;
  its README says the PCB "is still empty and needs placement/routing") and
  `RemiFM/USB-LiDAR-VL53L9` (USB device with an IMU). Neither can be bought.

**Why there is no board page.** The skill wants a board page for a breakout with
published board files. The STEVAL schematic and BOM came from a GitHub mirror,
and no ST board file (Gerber/CAD) has been seen, because st.com could not be
read. J2 is a single row, so its schematic numbering is enough to wire it, and
it is recorded above. Write
`docs/reference/boards/steval-vl53l9.md` once ST's board files or a fitted board
confirm pin 1's position and the as-shipped resistor population.

## Fit for robocar (ESP32-S3)

**Verdict: not practical as a full-resolution depth camera. Usable only as a
low-resolution (24 × 20 or smaller), few-hertz I²C sensor through the
STEVAL board's level shifters, on a mux channel of its own. Expect a driver port
with known ST-source defects and no published reference on an ESP32-S3.**

1. **No MIPI CSI-2, no I3C on the host.** The ESP32-S3 has a DVP camera interface
   (8–16 bit, up to 40 MHz) and two I²C controllers
   (ESP32-S3 datasheet v2.2 §4.2.1.4, §4.2.1.2). It has no CSI-2 receiver and no I3C
   controller. The two paths ST designs for (CSI-2 at 1 Gbps, I3C at 12.5 MHz) are
   out, and the frame has to be read over I²C (DS14879 §5.3).
   The only ESP32 shown streaming this sensor is an ESP32-P4 over CSI-2
   (`kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2`). A software I3C exists for the
   RP2040/RP2350 PIO (`earlynerd/VL53L9-Arduino`, working at 2.0–3.2 MHz); the
   ESP32-S3 has no PIO.
2. **I²C bandwidth caps resolution and rate.** The ESP32-S3's I²C goes "up to
   800 kbit/s (constrained by SCL and SDA pull-up strength)" (datasheet §4.2.1.2),
   below the sensor's 1 MHz. robocar runs its whole bus at
   **100 kHz** (`I2C_MASTER_FREQ_HZ`, `main/pin_config.h` line 39) because the
   RCWL-1601s NACK at 400 kHz ([sensor--rcwl-1601.md](sensor--rcwl-1601.md#i2c-mode)).
   A 54 × 42 frame is then ≥ 1.34 s of wire time and would hold the bus the 30 Hz
   executor shares with the PCA9685. At 24 × 20 it is ≥ 346 ms per frame (about 3 fps
   with nothing else on the bus), and at 12 × 10 ≥ 79 ms. The second I²C controller at
   400–800 kHz gets 54 × 42 down to 167–334 ms (3–6 fps) but needs two more pins.
3. **1.8 V I/O plus a clock.** IOVDD is 1.2/1.8 V, abs. max 1.98 V (Table 15).
   SDA, SCL, XSHUT, SYNC_IN, INTR and AP_CLK all need translation from the XIAO's
   3.3 V. The STEVAL board provides this. A bare module would need a carrier
   with three LDOs, level shifters and either an oscillator or a 6–27 MHz host
   clock at ± 100 ppm (§2.11). On top of the bus, the sensor needs XSHUT (driven
   high, no internal pull-up per VanBruce) and SYNC_IN (held high). INTR is optional
   because FRAME_READY can be polled (`vl53l9_poll_frame()`).
4. **Host RAM and CPU are not the limit.** A full raw frame is 14 842 B, the
   calibration block 2 332 B and the firmware patch 9 865 B in flash. Nothing in
   the ULD needs more. ST's host post-processing (confidence, reflectance, TNR) is
   a separate SLA0111-licensed pipeline whose RAM and CPU cost was not measured.
   Raw depth with the bit-15 valid flag is usable without it (VanBruce).
5. **Power.** 225–235 mW indoors at 54 × 42, up to 820 mW worst case (Table 36). The
   ranging maxima (Table 18) total about 670 mA, almost all on 3.3 V when VBAT and
   AVDD share it as on the STEVAL, with peaks up to 50 % higher. That is more than
   the whole robot's sensor load today. Which rail would feed it, and that rail's
   capacity, are open.
6. **Driver risk.** On a classic ESP32 at 400 kHz, `76EHwan/ESP32_STEVAL_VL53L9`
   (`docs/st-community-question.md`, HEAD `8ce070b0`) got through boot, patch
   upload, calibration read and `vl53l9_start()`, then hit a firmware fault
   (ERROR_CODE 0x0F00, and 0x0903 in CSI mode) on the first frame. The reference
   amplitude was 0 and the author suspects a heat-damaged module. That is
   unresolved, so no ESP32-family I²C frame read has been shown to work. The
   Raspberry Pi 5 I²C driver (`VanBruce/vl53l9cx-python`, HEAD `71659d89`) shows the
   I²C-only path works on a Linux host.

**What it would take:** a STEVAL-VL53L9 with J3 at 3.3 V and R23/R25 removed
(so it runs from Y1), alone on a spare TCA9548A channel (ch5–7 are reserved). That
avoids empty-transaction probes from other code and keeps its 2.2 kΩ host pull-ups
off the shared segment. Add one GPIO for XSHUT, tie SYNC_IN high, run binning 4–8 in
manual sync, and use chunked reads so the executor's PCA9685 writes interleave. The
alternative is a different host: an ESP32-P4 over CSI-2 (shown working), or an
STM32 with I3C (ST's own reference).

## Check before buying or wiring

- [ ] ST product-page status for VL53L9CX (Active / Evaluation / NRND) and the
      package line, which disagrees with the datasheet (12.1 × 5.1 × 4.5 in the BOM and
      search snippet vs 12.83 × 6.10 × 4.64 mm in Table 2)
- [ ] Whether the loose module (VL53L9CXV0VE/1) can be bought in single units, and at what price
- [ ] Whether ST publishes STEVAL-VL53L9 board files (Gerber/CAD) and a user manual
      (UM number). Only the schematic, BOM and DigiKey's data brief link have been seen
- [ ] STEVAL J2 pin 1's physical position, and the shipped R23/R24/R25 population
      (BOM Rev 2 fits R25, which disables Y1)
- [ ] Which side of the level shifters the STEVAL EEPROM U4 (7-bit 0x55) sits on,
      and whether it answers on the host bus (schematic p. 5)
- [ ] Whether the STEVAL has a pull-up or pull-down on XSHUT and SYNC_IN
      (47 kΩ R1–R6, R9, R10, partly DNP; not traced)
- [ ] The ESP32-S3 I²C at 800 kHz with the STEVAL's 2.2 kΩ pull-ups in parallel
      with the host's, and whether 100 kHz frame reads time out in the device
- [ ] Whether ESP-IDF's Xtensa GCC uses 4-byte enums (decides whether the ST enum
      defect bites unpatched)
- [ ] Bit-15 depth valid flag (undocumented; VanBruce only)
- [ ] TN1596 (post-processing and data format) and TN1579 (thermal), not read
- [ ] The 3.3 V rail on the robocar carrier that could supply ~670 mA plus peaks
- [ ] Whether `76EHwan`'s first-frame fault was the board or the port (watch the repo
      and its ST community thread)

## Datasheets & References

- **DS14879 Rev 7, VL53L9CX datasheet (ST, July 2026):**
  <https://www.st.com/resource/en/datasheet/vl53l9cx.pdf>. The copy read here is the
  mirror in `RemiFM/USB-LiDAR-VL53L9` at `doc/DS_vl53l9cx.pdf`, commit `cb14a3d3`
  (51 pages; page numbers above are the datasheet's own)
- **ST product page:** <https://www.st.com/en/imaging-and-photonics-solutions/vl53l9cx.html> (not readable by script)
- **STEVAL-VL53L9 schematic (Rev 1, March 2026) and BOM (Rev 2, May 2026), ST:**
  mirrored in `tokla-chiya/VL53L9_KICAD` at commit `95a80054`
  (`steval-vl53l9-schematic.pdf`, `steval-vl53l9-bom.pdf`)
- **STEVAL-VL53L9 at DigiKey:** <https://www.digikey.fi/en/products/detail/stmicroelectronics/STEVAL-VL53L9/29294599>
- **ST ULD driver:** <https://github.com/STMicroelectronics/fp-sns-datalog2/tree/63059c6efe0f2a0e98e2ccc68c82121f60d4facf/Drivers/BSP/Components/vl53l9>
- **ESP32-S3 datasheet v2.2 (Espressif):** <https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf>
- **VanBruce/vl53l9cx-python** (Raspberry Pi 5 I²C driver, hardware notes):
  <https://github.com/VanBruce/vl53l9cx-python/blob/71659d89aadd81fecfe7f7b0edac59ee578a74ef/README.md>
- **76EHwan/ESP32_STEVAL_VL53L9** (ESP32 I²C port, first-frame fault, driver defects):
  <https://github.com/76EHwan/ESP32_STEVAL_VL53L9/blob/8ce070b02684b1d7bca3680d935b7d7ba33b39c5/docs/st-community-question.md>
- **kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2** (ESP32-P4 over CSI-2, STEVAL rework, SLA0111 pipeline):
  <https://github.com/kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2/blob/12dcbdeab81ecc5a1e559dcbfd745124152d0049/README.md>
- **earlynerd/VL53L9-Arduino** (RP2350 PIO I3C):
  <https://github.com/earlynerd/VL53L9-Arduino/blob/0048b13777e83f5b426ce16db31057a552da8c41/README.md>
- **Hackster, Feb 2024 announcement coverage** (60 fps, 5 cm–10 m, GA early 2025):
  <https://hackster.io/news/stmicro-unveils-the-high-resolution-all-in-one-vl53l9-direct-time-of-flight-lidar-sensor-2e9616eac192>
- **CNX Software, 22 June 2026** (100 Hz, mass production July 2026, eval board prices):
  <https://www.cnx-software.com/2026/06/22/st-vl53l9cx-direct-time-of-flight-3d-lidar-supports-5cm-to-9m-range-2-3k-zones-resolution/>
- **Secondary, unverified:** <https://www.jaredwatkins.com/research/robotics/sensors/vl53l9cx/> (AI-maintained summary; used only for the lifecycle disagreement)
