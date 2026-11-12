# OV3660 — 3MP CMOS Image Sensor (OmniBSI, DVP)

**Manufacturer:** OmniVision Technologies
**Used in:** robocar, on the Seeed XIAO ESP32-S3 Sense camera board
([`../boards/xiao-esp32s3-sense.md`](../boards/xiao-esp32s3-sense.md)). The
board logs `PID=0x3660` at boot; `camera_pins.h` called it an OV2640 for years,
which was wrong (see
[`camera-and-log.md`](../../../packages/robotics/robocar/docs/firmware/camera-and-log.md#the-sensor-is-an-ov3660-not-an-ov2640)
and [`camera-sensor-identity.md`](../../../.claude/rules/camera-sensor-identity.md)).
The firmware runs it at QVGA JPEG, quality 15, `fb_count = 2`, XCLK 20 MHz
(`packages/robotics/robocar/main/camera.c` lines 41–63).

**Differs from the [OV2640](camera--ov2640.md):** 16-bit register addresses
(no bank switching), SCCB address 0x3C rather than 0x30, and
`set_gainceiling()` takes a raw 10-bit ceiling rather than a 0–6 enum index.

Seeed's wiki now says the OV3660 is out of stock and new Sense boards ship
with an OV5640 (autofocus), or with either sensor during the transition
(Seeed XIAO ESP32-S3 getting-started page, "OV2640 camera has been
discontinued…"). The OV5640 answers on the same address 0x3C; only the PID
tells them apart (see [Identification](#identification)). A replacement Sense
board needs its PID read before any OV3660 fact here is applied to it.

Datasheet citations are to the OV3660 datasheet v1.3, May 2011, marked
**PRELIMINARY SPECIFICATION**; its key-spec figures are footnoted "target
values … subject to change based on real measurements", and several entries
(dark current, power consumption) are `TBD`.

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| Active array | 2048 × 1536 (QXGA, 3 MP) | DS key specifications |
| Full array | 2080 columns × 1548 rows | DS §2.3 |
| Pixel | 1.4 × 1.4 µm, OmniBSI (backside-illuminated) | DS features; key specifications |
| Optical format | 1/5" | DS features; key specifications |
| Image area | 2912 × 2167.2 µm | DS key specifications |
| Package | 51-pin CSP3, 5010 × 4960 µm; ordering code OV03660-A51A | DS ordering information; key specifications |
| Shutter | Rolling | DS key specifications; §3.4.1 |
| Output formats | RAW RGB, RGB565/555/444, CCIR656, YCbCr422, compression | DS features |
| On-chip JPEG | Yes, called "compression" (six compression timing modes, §5.1; registers `0x4400`–`0x440E`, Table 5-1). The datasheet never uses the word JPEG; esp32-camera drives it as `PIXFORMAT_JPEG` | DS §5.1, Table 5-1; `ov3660.c` 221–223 |
| Output interface | DVP parallel, 8/10-bit; no MIPI in this datasheet | DS features; key specifications; §5.2 |
| Max frame rate | 2048×1536: 15 fps · 1080p: 20 fps · 720p: 45 fps · XGA: 45 fps · VGA: 60 fps · QVGA: 120 fps (target values) | DS key specifications |
| Input clock (XVCLK) | 6–27 MHz | DS key specifications |
| Sensitivity | 670 mV/lux·s | DS key specifications |
| Max S/N | 34 dB | DS key specifications |
| Dynamic range | 70 dB @ 8× gain | DS key specifications |
| Max exposure | 1560 × tROW | DS key specifications |
| Lens CRA | 27.6° | DS key specifications |
| Dark current | TBD | DS key specifications |
| Operating temperature | −20 to 70 °C junction; stable image 0 to 50 °C | DS Table 8-2 |

## Supply rails

| Rail | Min | Typ | Max | Source |
|------|-----|-----|-----|--------|
| AVDD (analog) | 2.6 V | 2.8 V | 3.0 V | DS Table 8-3 |
| DVDD (core) | 1.425 V | 1.5 V | 1.575 V | DS Table 8-3 |
| DOVDD (I/O) | 1.71 V | 1.8 V | 3.0 V | DS Table 8-3 |

- DVDD can come from the embedded 1.5 V regulator ("recommended", DS
  Figure 2-2 note 5) or an external source. With 2.8 V I/O OmniVision
  recommends an external DVDD because of heat in the internal regulator
  (DS §2.4).
- Active current (internal DVDD, DOVDD 1.8 V): IDD-A 28 mA, IDD-IO 70 mA;
  standby 20 µA (DS Table 8-3). Key specifications give "active: 98 mA".
- Absolute maximum: VDD-A 4.5 V, VDD-D 3 V, VDD-IO 4.5 V; I/O −0.3 V to
  VDD-IO + 1 V (DS Table 8-1).
- Power-up with internal DVDD: DOVDD stable before AVDD; PWDN high during
  power-on and low no sooner than 5 ms after AVDD is stable; XVCLK running at
  least 2 ms before register access; registers accessible 20 ms after PWDN
  goes low (DS §2.4.1).

**On the XIAO Sense camera board** (Seeed "XIAO ESP32S3 Exp. Board v1.2"
schematic, connector JA1): AVDD and DOVDD are both 2.8 V (`AVCC_2V8`,
`VCC_2V8` from U1 SGM2036S-2.8), DVDD is the net `VCC_1V3` fed by U2
SGM2036S-1.5, so the board supplies DVDD externally at 2.8 V I/O, as §2.4
recommends. The schematic's revision table says U2 was changed to the 1.5 V
part on 2026-08-10, and its block diagram still labels that rail 1.3 V; it
does not name the earlier U2, and which revision the robot carries is
unknown (checklist). RESET has a 10 kΩ/100 nF pull-up and PWDN a 10 kΩ
pull-down on the board, consistent with the firmware's `CAM_PIN_PWDN` and
`CAM_PIN_RESET` = −1 (`camera_pins.h` lines 25–26). The data, sync, clock and
SCCB nets on JA1 match `camera_pins.h` lines 27–43 (IO10 XMCLK, IO40/IO39
SDA/SCL, Y9–Y2 = IO48, 11, 12, 14, 16, 18, 17, 15, VSYNC IO38, HREF IO47,
PCLK IO13).

## Identification

| Item | Value | Source |
|------|-------|--------|
| SCCB address | 0x3C 7-bit (write byte 0x78); programmable via `0x3100[7:1]`, default 0x78 | DS §6.2; Table 7-2 |
| SCCB message | 16-bit sub-address, 8-bit data | DS §6.2, Figure 6-1 |
| SCCB clock | 400 kHz max | DS Table 6-1 |
| Chip ID high | `0x300A` = **0x36** (read-only) | DS Table 2-6; Table 7-1 |
| Chip ID low | `0x300B` = **0x60** (read-only) | DS Table 2-6; Table 7-1 |
| PID esp32-camera compares | `OV3660_PID = 0x3660`, `OV3660_SCCB_ADDR = 0x3C` | `sensor.h` lines 22, 66 |

esp32-camera's `esp32_camera_ov3660_detect()` reads `0x300A` and `0x300B`
with 16-bit-addressed SCCB reads and accepts only `0x3660`, logging
`Mismatch PID=0x%x` otherwise (`ov3660.c` lines 999–1013). The OV5640 shares
address 0x3C, so probing the address alone cannot tell the two apart.

The robot's boot log reports `PID=0x3660`
([`camera-and-log.md`](../../../packages/robotics/robocar/docs/firmware/camera-and-log.md#the-sensor-is-an-ov3660-not-an-ov2640)),
which matches the datasheet's 0x36/0x60.

## Exposure, gain and the gain ceiling

- **Exposure:** `{0x3500[3:0], 0x3501, 0x3502}` in 1/16 row periods
  (DS §3.4.1).
- **Gain:** `{0x350A[1:0], 0x350B}` / 16. Up to 16× analog plus 4× digital;
  the first 15.9375× (`0x350A[1:0] = 0`) is analog (DS §3.4.1; Table 7-5).
- **AEC/AGC:** built in; enabled by `0x3503[0]` (AEC) and `0x3503[1]` (AGC)
  = 0, auto being the reset state. OmniVision recommends enabling or
  disabling both together to avoid flicker (DS §3.4.2; Table 7-5).
- **Gain ceiling:** `0x3A18[1:0]` = ceiling[9:8], `0x3A19[7:0]` =
  ceiling[7:0], both "Real gain format" (DS Table 7-9). Real gain format is
  the `/16` encoding of §3.4.1, so the ceiling is a 10-bit value in 1/16×
  steps: 248 = 15.5×, 1023 ≈ 63.9×. This is the datasheet basis for
  `set_gainceiling()` taking a raw 10-bit value on this sensor.
- esp32-camera's `set_gainceiling()` writes `(level >> 8) & 3` to `0x3A18`
  and `level & 0xFF` to `0x3A19` with no range check (`ov3660.c` lines
  843–855), and the status init reads the ceiling back as
  `read_reg16(0x3A18) & 0x3FF` (line 977). `esp_camera_init()` forces
  `GAINCEILING_2X` only when the PID is the OV2640's (`esp_camera.c` lines
  358–359), so on this sensor the value after init is the driver's init
  table: `0x3A18 = 0x00`, `0x3A19 = 0xF8` = 248 = 15.5×
  (`ov3660_settings.h` lines 65–66, `sensor_default_regs`).
- **The chip's own reset default differs:** DS Table 7-9 lists `0x3A18`
  default `0x07` and `0x3A19` default `0xC0`. Read through the 10-bit field
  that is 0x3C0 = 960 = 60×; `0x07` also sets bit 2, which the same table
  marks Reserved. The firmware comment and docs call 248 the "power-on
  default"; it is the value esp32-camera writes during init, not the
  silicon reset value. The practical claim (15.5× after `esp_camera_init()`)
  stands.

## Driver notes (esp32-camera)

- Maximum frame size `FRAMESIZE_QXGA`, JPEG supported (`sensor.c` line 8);
  `set_framesize()` clamps anything larger to QXGA (`ov3660.c` lines
  315–317).
- Pixel formats accepted: YUV422, grayscale, RGB565 (RGB888 maps to the
  RGB565 table), JPEG, RAW (`ov3660.c` lines 202–233).
- In JPEG mode the driver picks its PLL by frame size and XCLK: QXGA or a
  16 MHz XCLK gets the branch commented "40MHz SYSCLK and 10MHz PCLK",
  anything else (the robot: QVGA at 20 MHz) the one commented "50MHz SYSCLK
  and 10MHz PCLK" (`ov3660.c` lines 358–365). The comments do not say which
  XCLK the figures assume.
- The robot's `idf_component.yml` allows `espressif/esp32-camera: '^2.0.0'`
  and the project tracks no `dependencies.lock`, so the resolved version is
  whatever the last build fetched. Line numbers above are from the pinned
  commit below.

## Open checklist

- [ ] Which revision of the Sense camera board the robot carries, and what
      voltage its DVDD regulator (U2) outputs (1.3 V net name vs. the
      1.5 V part fitted from 2026-08-10). 1.3 V would be below the
      1.425 V DVDD minimum in DS Table 8-3. Measure DVDD at the camera
      FPC (JA1 pin 10) or read U2's marking.
- [ ] Which camera module Seeed fits (lens, FPC pinout); the "OV3660 Camera
      Module Specification" PDF Seeed links is image-only Chinese text and
      was not read.
- [ ] The OV3660's reset default for `0x3A18` (`0x07`, with a Reserved bit
      set) looks like a datasheet typo in a preliminary spec; read the live
      register before `esp_camera_init()` to settle it, if it ever matters.
- [ ] Frame rates are datasheet targets at an unstated clock. The robot's
      achieved rate at QVGA/JPEG/20 MHz XCLK has not been measured here.
- [ ] Maximum PCLK and DVP timing (DS Table 5-3) not transcribed.

## Datasheets & References

- **OV3660 datasheet v1.3, May 2011, PRELIMINARY** (165 pages, SHA-256
  `5afc0efd12b27a58f87a4cfc4fab1edcdaf86e5e12ea25722df6bf1ab72c81bc`), as
  hosted by Seeed: <https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/OV3660_datasheet.pdf>.
  Byte-identical mirror: <https://github.com/developing-today/hardware-doc/blob/main/components/omnivision/ov3660/artifacts/ov3660-datasheet.pdf>
- **Seeed OV3660 Camera Module Specification** (Chinese, image-only, not
  read): <https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/new-res/OV3660_Camera_Module_Specification.pdf>
- **Seeed XIAO ESP32S3 Exp. Board v1.2 schematic** (the Sense camera/mic/SD
  board, 2026-08-10): <https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_Exp_Board_v1.2_SCH_PDF_260810.pdf>
- **Seeed XIAO ESP32-S3 getting-started wiki** (sensor transition note):
  <https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/>
- **espressif/esp32-camera** at commit `281d951f75b6559adec3cdb8cf076366da103880`:
  [`sensors/ov3660.c`](https://github.com/espressif/esp32-camera/blob/281d951f75b6559adec3cdb8cf076366da103880/sensors/ov3660.c),
  [`sensors/private_include/ov3660_settings.h`](https://github.com/espressif/esp32-camera/blob/281d951f75b6559adec3cdb8cf076366da103880/sensors/private_include/ov3660_settings.h),
  [`sensors/private_include/ov3660_regs.h`](https://github.com/espressif/esp32-camera/blob/281d951f75b6559adec3cdb8cf076366da103880/sensors/private_include/ov3660_regs.h),
  [`driver/include/sensor.h`](https://github.com/espressif/esp32-camera/blob/281d951f75b6559adec3cdb8cf076366da103880/driver/include/sensor.h),
  [`driver/sensor.c`](https://github.com/espressif/esp32-camera/blob/281d951f75b6559adec3cdb8cf076366da103880/driver/sensor.c),
  [`driver/esp_camera.c`](https://github.com/espressif/esp32-camera/blob/281d951f75b6559adec3cdb8cf076366da103880/driver/esp_camera.c)
- **Prior research note** (same datasheet and driver, cross-checked):
  <https://github.com/developing-today/hardware-doc/blob/main/components/omnivision/ov3660/README.md>
