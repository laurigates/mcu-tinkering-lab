# AI Thinker ESP32-CAM

A 27 × 40.5 mm camera board from Ai-Thinker: an ESP-32S module (ESP32, 4 MB
flash), an external PSRAM chip, a 24-pin camera connector for an OV2640, a
microSD slot, a white flash LED and a small status LED, on two 8-pin headers
with no USB. It is the board named by `cam-i2s-audio`, `cam-webserver` and
`llm-telegram` (each project's `flasher.json` `board` field). The silicon and
the module summary are on their own pages:
[ESP32-CAM module summary](../datasheets/board--esp32-cam-ai-thinker.md),
[ESP32](../datasheets/mcu--esp32.md), [OV2640](../datasheets/camera--ov2640.md).

## The board documented here

This page documents Ai-Thinker's own design, from two vendor documents:

- the *ESP32-CAM Wi-Fi+BT SoC 模组 V1.0* specification (Ai-Thinker, 2017, 4
  pages, Chinese), as hosted by LCSC for part C277946, and
- the *ESP32_CAM* schematic, revision 1.6, dated 2018/1/30 (Altium output,
  1 page), as mirrored in Seeed's `SeeedDocument/forum_doc` repository.

Ai-Thinker's documentation site no longer carries an ESP32-CAM page: on
2026-11-14 `docs.ai-thinker.com/en/esp32-cam` returned the site's landing
page, and the site's route table lists no ESP32-CAM entry. Both documents
above are copies held by third parties; their titles, revision and date
fields name Ai-Thinker.

Which board Lauri owns has not been checked. Do not confuse it with:

| Board | How it differs | Source |
|---|---|---|
| Unbranded ESP32-CAM clones | Sold under the same name. Clones can differ in the flash LED circuit, the PSRAM fitted and the antenna; none of them is documented here | No clone schematic fetched |
| ESP32-CAM-MB | A separate programmer base the ESP32-CAM plugs into, with a USB-serial chip. Not part of the camera board | No vendor document found; listings only |
| M5Stack ESP32CAM | A different board: ESP32 with 4 MB flash, OV2640, USB Type-C and a Grove port | M5Stack's product page, as mirrored at static.chipdip.ru (page 1) |

## Specifications

| Feature | Value | Source |
|---|---|---|
| Module | Ai-Thinker ESP-32S (M1) | Schematic V1.6, M1 |
| MCU | ESP32, dual-core 32-bit CPU, up to 240 MHz | Spec V1.0 p.1 (特性) |
| SPI flash | 32 Mbit (4 MB) by default | Spec V1.0 p.2 (SPI Flash: 默认 32Mbit) |
| RAM | 520 KB internal + external "4M PSRAM" | Spec V1.0 p.2 (RAM); PSRAM chip U5 in schematic V1.6 |
| Camera | OV2640 or OV7670 supported; JPEG output only with OV2640 | Spec V1.0 p.1 and p.2 (图像输出格式) |
| microSD | "TF card, up to 4G" | Spec V1.0 p.2 (支持 TF 卡) |
| Header I/O | "9 I/O"; the pin table below has 10 GPIO pads, counting UART0's two | Spec V1.0 p.2 (IO 口); spec p.4 |
| Package, size | DIP-16, 27 × 40.5 × 4.5 (±0.2) mm | Spec V1.0 p.2 |
| Antenna | On-board PCB antenna, 2 dBi | Spec V1.0 p.2 (天线形式) |
| Supply | 5 V | Spec V1.0 p.2 (供电范围) |
| Current at 5 V | 180 mA flash LED off; 310 mA flash LED at full brightness | Spec V1.0 p.2 (功耗) |
| Sleep current at 5 V | Deep-sleep 6 mA; modem-sleep 20 mA; light-sleep 6.7 mA | Spec V1.0 p.2 (功耗) |
| Default UART rate | 115200 bps | Spec V1.0 p.2 (串口速率) |
| Operating temperature | −20 to 85 °C | Spec V1.0 p.2 |
| Regulators | AMS1117 (5 V to 3V3); XC6206-2.8V and XC6206-1.2V for the camera | Schematic V1.6, U2, U1, U3 |
| USB | None | Spec V1.0 lists no USB; the schematic has no USB or USB-serial part |

## Pin mapping

The two 8-pin headers, numbered as the vendor's minimal-system diagram numbers
them (spec V1.0 p.4, module M1 pins 1–16), with the nets from schematic V1.6
(P2 is pins 1–8, P1 is pins 9–16, in the order the spec's diagram gives).
`Pin` is the vendor's module pin number, because three pads are all printed
`GND`. There are no `Side`/`Pos` columns: the order comes from the vendor
drawings, and no board file is published, so which face the drawing on spec
p.1 shows is unconfirmed (see the checklist).

| Pin | Silkscreen | GPIO | Function | Also wired to on the board | Source |
|---|---|---|---|---|---|
| 1 | IO4 | GPIO4 | GPIO | microSD DATA1 (R11 47K pull-up); flash LED driver Q1 base through R13 1K | Spec p.4; schematic P2-1 (HS2_DATA1) |
| 2 | IO2 | GPIO2 | strapping (boot mode) | microSD DATA0 (R10 47K pull-up) | Spec p.4; schematic P2-2 (HS2_DATA0) |
| 3 | IO14 | GPIO14 | GPIO | microSD CLK | Spec p.4; schematic P2-3 (HS2_CLK) |
| 4 | IO15 | GPIO15 | strapping (MTDO: boot log on U0TXD) | microSD CMD (R9 47K pull-up) | Spec p.4; schematic P2-4 (HS2_CMD) |
| 5 | IO13 | GPIO13 | GPIO | microSD DATA3 (R8 47K pull-up) | Spec p.4; schematic P2-5 (HS2_DATA3) |
| 6 | IO12 | GPIO12 | strapping (MTDI: VDD_SDIO voltage) | microSD DATA2 (pull-up R7 marked NC, not fitted) | Spec p.4; schematic P2-6 (HS2_DATA2) |
| 7 | GND | — | Ground | — | Spec p.4; schematic P2-7 |
| 8 | 5V | — | 5 V supply in | AMS1117 input | Spec p.4; schematic P2-8, U2 pin 3 |
| 9 | 3V3 | — | 3.3 V rail | AMS1117 output | Spec p.4; schematic P1-8, U2 pin 2 |
| 10 | IO16 | GPIO16 | GPIO (UART2 RX) | PSRAM chip select, U5 CS# (R5 10K pull-up) | Spec p.4; schematic P1-7 (U2RXD / IO16) |
| 11 | IO0 | GPIO0 | strapping (boot mode) | Camera XCLK (CSI_MCLK); R19 10K pull-up to 3V3 | Spec p.4 (IO0 悬空, "left floating"); schematic P1-6 |
| 12 | GND | — | Ground | — | Spec p.4; schematic P1-5 |
| 13 | VCC | — | Supply out: 3V3 through R1 0R or 5V through R2 0R | — | Spec p.4; schematic P1-4, R1, R2 |
| 14 | U0R | GPIO3 | UART0 RX (U0RXD) | — | Spec p.4; schematic P1-3; ESP32 datasheet Table 2-1 (U0RXD = GPIO3) |
| 15 | U0T | GPIO1 | UART0 TX (U0TXD) | — | Spec p.4; schematic P1-2; ESP32 datasheet Table 2-1 (U0TXD = GPIO1) |
| 16 | GND | — | Ground | — | Spec p.4; schematic P1-1 |

### Strapping pins

| GPIO | Header pin | What it selects at reset | Internal default | External on this board | Source |
|---|---|---|---|---|---|
| GPIO0 | 11 | 1 = SPI flash boot; 0 (with GPIO2 = 0) = download boot | Pull-up | R19 10K pull-up to 3V3 | ESP32 datasheet v5.3 Table 3-1, Table 3-3; schematic R19 |
| GPIO2 | 2 | Must be 0 for download boot; ignored for flash boot | Pull-down | R10 47K pull-up to 3V3 | Table 3-1, Table 3-3; schematic R10 |
| GPIO12 | 6 | MTDI: 0 = VDD_SDIO from VDD3P3_RTC (3.3 V); 1 = internal 1.8 V LDO | Pull-down | None (R7 not fitted) | Table 3-1, §3.2; schematic R7 |
| GPIO15 | 4 | MTDO: 1 = boot log printed on U0TXD; 0 = silent | Pull-up | R9 47K pull-up to 3V3 | Table 3-1, Table 3-4; schematic R9 |
| GPIO5 | — (camera D0) | With MTDO: SDIO slave timing | Pull-up | Camera data line | Table 3-1; ESP-32S spec V1 p.6 (Strapping 管脚) |

The ESP-32S module specification (V1, 2019, p.6) gives the same strapping
table as the ESP32 datasheet.

### Camera connector (Cam1)

All three sources agree on every line: the vendor specification's pin table,
the V1.6 schematic, and the `CAMERA_MODEL_AI_THINKER` block of Espressif's
Arduino example `camera_pins.h` (`espressif/arduino-esp32` at
`3ed7b779f2b4c797b3512f3fc10a7a978a78fb18`,
`libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h` lines
160–180). `espressif/esp32-camera` itself carries the same map as
`BOARD_ESP32CAM_AITHINKER` in `test/test_camera.c` lines 45–64 (at
`b3f7fe802f41f17fd7a104d517c53f1f62c8414d`); it has no board header.

| GPIO | Camera signal | Connector pin | Net (schematic) | esp32-camera name | Source |
|---|---|---|---|---|---|
| GPIO5 | Y2 (D0) | 6 | CSI_D0 | Y2_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.174 |
| GPIO18 | Y3 (D1) | 4 | CSI_D1 | Y3_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.173 |
| GPIO19 | Y4 (D2) | 3 | CSI_D2 | Y4_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.172 |
| GPIO21 | Y5 (D3) | 5 | CSI_D3 | Y5_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.171 |
| GPIO36 | Y6 (D4) | 7 | CSI_D4 (SENSOR_VP) | Y6_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.170 |
| GPIO39 | Y7 (D5) | 9 | CSI_D5 (SENSOR_VN) | Y7_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.169 |
| GPIO34 | Y8 (D6) | 11 | CSI_D6 | Y8_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.168 |
| GPIO35 | Y9 (D7) | 13 | CSI_D7 | Y9_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.167 |
| GPIO0 | XCLK | 12 | CSI_MCLK | XCLK_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.163 |
| GPIO22 | PCLK | 8 | CSI_PCLK | PCLK_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.177 |
| GPIO25 | VSYNC | 18 | CSI_VSYNC | VSYNC_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.175 |
| GPIO23 | HREF | 16 | CSI_HSYNC | HREF_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.176 |
| GPIO26 | SIO_D (SCCB data) | 22 | TWI_SDA, R18 4.7K pull-up | SIOD_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.164 |
| GPIO27 | SIO_C (SCCB clock) | 20 | TWI_SCK, R17 4.7K pull-up | SIOC_GPIO_NUM | Spec p.3; schematic; camera_pins.h l.165 |
| GPIO32 | Camera power switch | — | CAM_PWR: gate of P-MOS Q2 through R15 1K; R14 10K pulls the gate to 3V3 | PWDN_GPIO_NUM | Spec p.3 ("POWER PIN"); schematic; camera_pins.h l.161 |

GPIO32 does not reach the sensor's PWDN pin. On the V1.6 schematic, Q2
switches 3V3 into the two camera regulators (U1, U3), so driving GPIO32 low
powers the camera and the R14 pull-up leaves it off. The sensor's PWDN pin
(connector pin 17) goes to ground through R3 1K, and its RESET pin (pin 19,
CAM_RST) is an RC reset (R16 4.7K to 3V3, C12 0.1 µF to ground), not a GPIO;
`camera_pins.h` sets `RESET_GPIO_NUM -1` accordingly (line 162).

### Other on-board connections

| GPIO | Connected to | Behaviour | Source |
|---|---|---|---|
| GPIO4 | Flash LED (LED_FLASH1) through NPN Q1 (S8050); base from GPIO4 through R13 1K, R12 10K base to ground | High turns the LED on. The LED's anode is on 3V3; the schematic shows no series resistor | Schematic V1.6; spec p.3 (DATA1/闪光灯); camera_pins.h l.179–180 ("4 for flash led") |
| GPIO33 | LED1, cathode side; anode to 3V3 through R4 1K | Low turns the LED on | Schematic V1.6 (net LED); camera_pins.h l.179 ("33 for normal led") |
| GPIO14, GPIO15, GPIO2, GPIO4, GPIO12, GPIO13 | microSD CLK, CMD, DATA0, DATA1, DATA2, DATA3 | The 4-bit SD bus takes six of the header's GPIO pads | Spec p.3 (SD table); schematic SD1 |
| GPIO16 | PSRAM U5 CS# (pin 1), R5 10K pull-up | Not free while the PSRAM is used | Schematic V1.6 |
| GPIO17 | PSRAM U5 SCLK (pin 6) | Not on a header | Schematic V1.6 (PSRAM_CLK) |
| GPIO6–GPIO11 | Module flash (SCK, SDO, SDI, SHD, SWP, SCS); SD0–SD3 also run to PSRAM U5 | Not usable | ESP-32S spec V1 table 2.2 pins 17–22; schematic U5 |
| GPIO1, GPIO3 | UART0 on header pins 15 and 14 | The chip prints its boot log on U0TXD unless MTDO is low at reset | Schematic P1; ESP32 datasheet Table 2-1, §3.3 |

## Power

The vendor gives a single 5 V supply (spec p.2). On the V1.6 schematic the
`5V` pin feeds the AMS1117 regulator (U2), whose output is the `3V3` rail on
pin 9, the ESP-32S module, the PSRAM, the microSD slot and the flash LED.
The camera's 2.8 V and 1.2 V come from two XC6206 regulators behind the
GPIO32 power switch. Pin 13 (`VCC`) is an output whose voltage depends on which
of R1 (to 3V3) or R2 (to 5V), both 0 Ω, is fitted; the vendor's back-side
render on spec p.1 prints `5V` and `3V3` beside a jumper next to that pin.

The vendor rates the board at 180 mA (flash LED off) to 310 mA (flash LED at
full brightness) from 5 V (spec p.2) and says nothing about brownout.
`cam-webserver` treats brownout as a known problem: its
[`sdkconfig.defaults`](../../../packages/camera-vision/cam-webserver/sdkconfig.defaults)
enables the brownout detector at level 5 (lines 4–7) and caps Wi-Fi TX power
at 15 dBm (lines 23–28), its
[`main/main.c`](../../../packages/camera-vision/cam-webserver/main/main.c)
waits 1 s between camera and Wi-Fi start (lines 307–310), and its
[README](../../../packages/camera-vision/cam-webserver/README.md) lists both
measures (lines 15–16). Level 5 is 2.70 V ± 0.05 V in the ESP-IDF v5.5.1
Kconfig reference (`CONFIG_ESP_BROWNOUT_DET_LVL_SEL`), whose own note calls
the levels estimates. That is one project's working configuration, not a
vendor figure.

## Flashing

The board has no USB. Programming goes through UART0 on the header: a 3.3 V
USB-serial adapter's TX to `U0R` (pin 14, GPIO3), RX to `U0T` (pin 15, GPIO1),
GND to GND, and 5 V into the `5V` pin. The ESP32 operates from 2.3 V to
3.6 V (ESP32 datasheet §2.5, notes on power supply) and the board runs it from
the 3V3 rail, so the adapter's logic level must be 3.3 V. An ESP32-CAM-MB base does the same through its own USB-serial chip;
no vendor document for it was found.

To enter the download boot mode, hold GPIO0 (pin 11) at GND while the chip
comes out of reset (ESP32 datasheet Table 3-3), using the reset button K1 on
CHIP_PU (schematic V1.6, K1 with R6 10K and C7) or a power cycle. Remove the
link and reset again to run the firmware: R19 pulls GPIO0 high for flash
boot, which the vendor's diagram marks as "IO0 悬空" (left floating). The three
projects' justfiles print the same procedure, and their `flasher.json` files
set `"flashMode": "gpio0"`.

## Known issues and quirks

- **The microSD slot takes most of the header.** GPIO2, GPIO4, GPIO12, GPIO13,
  GPIO14 and GPIO15 are the 4-bit SD bus (spec p.3), leaving GPIO0 (camera
  XCLK), GPIO16 (PSRAM chip select) and UART0 as the other header I/O.
- **The flash LED follows SD DATA1.** Q1's base is driven from the HS2_DATA1
  net through R13, so traffic on SD DATA1 switches the flash LED too
  (schematic V1.6).
- **GPIO12 must not be high at reset.** MTDI = 1 selects the 1.8 V LDO for
  VDD_SDIO (ESP32 datasheet §3.2), and §2.5 notes that flash and PSRAM run
  from VDD_SDIO. The schematic powers the PSRAM from 3V3, so this is a 3.3 V
  flash and PSRAM design. Anything on header pin 6 that pulls it up at power-on
  breaks boot.
- **GPIO2 carries a 47K pull-up.** Download boot needs GPIO2 = 0 (ESP32
  datasheet Table 3-3), and the V1.6 schematic fits R10 47K from GPIO2 to
  3V3. How the board still enters download mode with only GPIO0 grounded is
  not explained by these sources; see the checklist.
- **Camera power is inverted.** GPIO32 low powers the camera (see the camera
  table).
- **GPIO16 belongs to the PSRAM.** Header pin 10 is wired to the PSRAM's
  chip select (schematic V1.6, U5 pin 1).
- **Sensor identity is not proven by the board name.** The vendor lists
  OV2640 and OV7670 as supported (spec p.1). Read the PID that
  `esp_camera_init()` logs before reasoning about any sensor register
  (`.claude/rules/camera-sensor-identity.md`).

## In this repository

| Project | Where | GPIOs used | Agrees with the vendor sources |
|---|---|---|---|
| [cam-webserver](../../../packages/camera-vision/cam-webserver/) | `main/main.c` lines 21–38 (`CAM_PIN_*`) | Camera: PWDN 32, RESET −1, XCLK 0, SIOD 26, SIOC 27, D7–D0 35, 34, 39, 36, 21, 19, 18, 5, VSYNC 25, HREF 23, PCLK 22 | Yes, every line |
| [llm-telegram](../../../packages/camera-vision/llm-telegram/) | `main/camera_handler.c` lines 13–29 | Camera: the same map as cam-webserver | Yes, every line |
| [llm-telegram](../../../packages/camera-vision/llm-telegram/) | `main/config.h` lines 31–34; `main/motor_controller.c` lines 68–71 | Motor PWM on GPIO12, GPIO13, GPIO14, GPIO15 | These are the microSD lines, and GPIO12 and GPIO15 are strapping pins; the project's README (line 159) says no motor pins are driven, while the code configures LEDC channels on them |
| [cam-i2s-audio](../../../packages/camera-vision/cam-i2s-audio/) | `main/main.c` | None yet: camera and I2S are placeholders | — |

All three projects flash with GPIO0 to GND (their justfiles' `flash` and
`info` recipes, `flasher.json` `"flashMode": "gpio0"`). No project has a
`hardware.toml` that names this page.

## Source

- **Header pin numbers and labels:** spec V1.0 p.4 (minimal-system diagram)
  and p.1 (dimension drawing and back-side render), cross-checked against
  headers P1 and P2 of schematic V1.6. Every pin agrees.
- **Camera, microSD, LED, PSRAM and power nets:** schematic V1.6, read from a
  600 dpi render of the PDF; camera and SD pins also from spec V1.0 p.3.
- **Camera pin map in working code:** `espressif/arduino-esp32`
  `camera_pins.h` and `espressif/esp32-camera` `test/test_camera.c` at the
  commits named above.
- **Strapping, UART0 and VDD_SDIO:** Espressif *ESP32 Series Datasheet*
  v5.3, Table 2-1, §2.5, §3 (Tables 3-1, 3-3, 3-4) and §3.2; Ai-Thinker
  *ESP32-S 规格书* V1 (2019), table 2.2 and p.6.
- **Brownout:** `cam-webserver` as the only witness, with the level voltage
  from the ESP-IDF v5.5.1 Kconfig reference. The repo builds with ESP-IDF
  v5.5.6 (`Dockerfile`).

**Not checked on a fitted board:** which board Lauri has (Ai-Thinker or a
clone), the header order as seen on it, which of R1/R2 sets `VCC`, the PSRAM
size, the sensor fitted, the LED colours, and the brownout behaviour.

## Check on the physical board

- [ ] Identify the board: Ai-Thinker or clone, the shield marking on the
      module (spec p.1 photo shows `ESP32-S`), and whether it came with an
      ESP32-CAM-MB base.
- [ ] Read the header silkscreen on the fitted board and confirm it matches
      the pin table (IO4 … 5V on one row, 3V3 … GND on the other), then note
      which face and orientation the spec p.1 drawing shows before adding
      `Side`/`Pos`.
- [ ] Measure pin 13 (`VCC`) against GND with 5 V applied: 3.3 V means R1 is
      fitted, 5 V means R2.
- [ ] Read the PSRAM size from the U5 chip marking or the boot log and compare
      with the spec's "4M PSRAM".
- [ ] Read the camera PID from the `esp_camera_init()` log (`OV2640_PID` is
      `0x2640`, per `.claude/rules/camera-sensor-identity.md`).
- [ ] Confirm the status LED on GPIO33 lights when GPIO33 is driven low, and
      note its colour.
- [ ] Confirm the flash LED's circuit: whether a series resistor is fitted
      between 3V3 and the LED, and its current at full on.
- [ ] Enter download mode with only GPIO0 grounded and record whether it
      works every time, given R10's 47K pull-up on GPIO2.
- [ ] Measure the 5 V and 3V3 rails with a scope during Wi-Fi association,
      with and without cam-webserver's TX power cap, to see whether the
      brownout settings are needed on this board and supply.

## References

- [Ai-Thinker ESP32-CAM specification V1.0 (LCSC copy, C277946)](https://datasheet.lcsc.com/datasheet/pdf/a6b8e236d770ab01c85fedd0de00a6bb.pdf)
- [Ai-Thinker ESP32_CAM schematic V1.6 (Seeed mirror)](https://github.com/SeeedDocument/forum_doc/blob/master/reg/ESP32_CAM_V1.6.pdf)
- [Ai-Thinker ESP32-S module specification V1 (LCSC copy, C277944)](https://datasheet.lcsc.com/datasheet/pdf/64c809fa198202a6fe4f560f9dddd0e6.pdf)
- [Espressif ESP32 Series Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf) (v5.3 read)
- [`camera_pins.h`, espressif/arduino-esp32](https://github.com/espressif/arduino-esp32/blob/3ed7b779f2b4c797b3512f3fc10a7a978a78fb18/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h#L160-L180)
- [`test/test_camera.c`, espressif/esp32-camera](https://github.com/espressif/esp32-camera/blob/b3f7fe802f41f17fd7a104d517c53f1f62c8414d/test/test_camera.c#L45-L64)
- [ESP-IDF v5.5.1 Kconfig reference, `CONFIG_ESP_BROWNOUT_DET_LVL_SEL`](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32/api-reference/kconfig-reference.html#config-esp-brownout-det-lvl-sel)
- [M5Stack ESP32CAM product page (chipdip mirror)](https://static.chipdip.ru/lib/849/DOC041849252.pdf)
