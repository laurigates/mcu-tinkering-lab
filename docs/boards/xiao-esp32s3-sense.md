# Seeed Studio XIAO ESP32-S3 Sense

The XIAO ESP32-S3 with an expansion board that adds an OV2640 camera (1600x1200), PDM digital microphone, and microSD card slot. All the AI/ML capabilities of the [base XIAO ESP32-S3](xiao-esp32s3.md) — dual-core LX7, 8 MB PSRAM, USB OTG — plus vision and audio input in a 21 x 17.8 x 15 mm stack. The smallest ESP32 camera board available at ~$14–18.

## Why XIAO ESP32-S3 Sense

- **Camera + AI in a thumb** — OV2640 camera with 8 MB PSRAM for frame buffers. Run TensorFlow Lite Micro face detection, person detection, or image classification on-device without cloud API calls.
- **Digital microphone** — MSM261D3526H1CPM PDM mic for wake word detection (ESP-SR), audio classification, or voice recording to SD card.
- **MicroSD card** — Up to 32 GB FAT32 for data logging, image capture, or audio recording.
- **Detachable expansion** — The camera/mic/SD board connects via B2B connector. Remove it and you have a standard XIAO ESP32-S3. Reattach when you need sensors.
- **Single-board robocar** — Selected as the target for unifying the dual-board robocar into one board ([ADR-013](../blueprint/adrs/ADR-013-single-board-xiao-esp32s3.md)).

### When to pick something else

| Need | Better choice |
|------|---------------|
| Higher resolution camera | ESP32-S3-EYE (2MP + LCD) or Linux SBC with CSI |
| More GPIOs (>11) | [XIAO ESP32-S3 Plus](xiao-esp32s3-plus.md) (no camera, but 18 GPIOs) |
| No camera/mic needed | [XIAO ESP32-S3](xiao-esp32s3.md) (same board, ~$8) |
| Wi-Fi 6 / Thread / Zigbee | [XIAO ESP32-C6](xiao-esp32c6.md) |
| LoRa radio | [TTGO LoRa32](ttgo-lora32-v2.md) |
| On-device NPU inference | [Luckfox Pico Ultra](luckfox-pico-ultra.md) (1 TOPS NPU) |

## Specifications

| Feature | Details |
|---------|---------|
| MCU | ESP32-S3R8 — Xtensa LX7 dual-core @ 240 MHz |
| SRAM | 512 KB |
| Flash | 8 MB (quad SPI) |
| PSRAM | 8 MB (octal SPI, separate bus) |
| Wi-Fi | 802.11 b/g/n (Wi-Fi 4, 2.4 GHz) |
| Bluetooth | Bluetooth 5.0 LE (no Classic BT) |
| USB | USB-C (USB 1.1 OTG + USB-Serial-JTAG) |
| Camera | OV2640 (1600x1200) or OV3660 (2048x1536) via DVP |
| Microphone | MSM261D3526H1CPM PDM digital MEMS |
| SD card | MicroSD slot, up to 32 GB FAT32 |
| GPIO | 11 usable pins (D0–D10), 2 bottom pads (D11/D12) |
| ADC | 9 channels (12-bit SAR) |
| Touch | 9 capacitive touch pins |
| PWM | 11 channels (LEDC, any GPIO) |
| SPI | 1× user-accessible |
| I2C | 1× (SDA: GPIO5/D4, SCL: GPIO6/D5) |
| UART | 1× (TX: GPIO43/D6, RX: GPIO44/D7) |
| Antenna | U.FL connector with detachable external antenna |
| Battery | 3.7V LiPo via bottom pads, 50–100 mA charge IC |
| Dimensions | 21 x 17.8 x 15 mm (with expansion board) |
| Operating temp | -40°C to 65°C |

## Pinout

Same external pinout as the [base XIAO ESP32-S3](xiao-esp32s3.md). The expansion board connects via the B2B connector on the bottom, using internal GPIOs that are not broken out on the side headers.

```
           ┌──────────────────────┐
           │  XIAO ESP32-S3 Sense │
           │  ┌──────────────┐    │
           │  │  OV2640       │    │
           │  │  Camera       │    │
           │  └──────────────┘    │
           │  [RST]    [BOOT]     │
           ├──┬───────────────┬───┤
    D0/A0  │● │               │ ●│  5V
    D1/A1  │● │               │ ●│  GND
    D2/A2  │● │               │ ●│  3V3
    D3/A3  │● │               │ ●│  D10/MOSI
    D4/SDA │● │               │ ●│  D9/MISO
    D5/SCL │● │               │ ●│  D8/SCK
    D6/TX  │● │               │ ●│  D7/RX
           ├──┴───────────────┴───┤
           │      [USB-C]         │
           └──────────────────────┘
```

### Expansion Board Internal GPIO Usage

The camera, microphone, and SD card use GPIOs that are not on the side headers. These are routed through the B2B connector on the bottom of the main board.

#### Camera (OV2640/OV3660) — 14 GPIOs

| Function | GPIO |
|----------|------|
| XMCLK (clock out) | GPIO10 |
| DVP_Y2 | GPIO15 |
| DVP_Y3 | GPIO17 |
| DVP_Y4 | GPIO18 |
| DVP_Y5 | GPIO16 |
| DVP_Y6 | GPIO14 |
| DVP_Y7 | GPIO12 |
| DVP_Y8 | GPIO11 |
| DVP_Y9 | GPIO48 |
| PCLK | GPIO13 |
| VSYNC | GPIO38 |
| HREF | GPIO47 |
| SIOC (I2C SCL) | GPIO39 |
| SIOD (I2C SDA) | GPIO40 |

#### Microphone — 2 GPIOs

| Function | GPIO |
|----------|------|
| PDM CLK | GPIO42 (D12) |
| PDM DATA | GPIO41 (D11) |

D11 and D12 are on the bottom of the expansion board and are reserved for the microphone by default. To reclaim them for other uses, cut the solder jumpers J1/J2 on the back of the expansion board.

#### SD Card — 1 GPIO

| Function | GPIO |
|----------|------|
| CS (chip select) | GPIO21 |

The SD card shares the SPI bus with D8/D9/D10 (SCK/MISO/MOSI). GPIO21 is the dedicated chip select, so SPI peripherals on D8–D10 coexist with the SD card if you manage CS lines.

### Pull-up Resistors

The Sense expansion board includes three pull-up resistors (R4–R6) connected to the SD card slot. These can cause conflicts if you also attach SPI peripherals that have their own pull-ups. Desolder R4–R6 if needed.

## Camera Usage

### ESP-IDF Camera Pin Configuration

```c
#define CAMERA_PIN_PWDN    -1   // Not connected
#define CAMERA_PIN_RESET   -1   // Not connected
#define CAMERA_PIN_XCLK    10
#define CAMERA_PIN_SIOD    40
#define CAMERA_PIN_SIOC    39
#define CAMERA_PIN_D7      48
#define CAMERA_PIN_D6      11
#define CAMERA_PIN_D5      12
#define CAMERA_PIN_D4      14
#define CAMERA_PIN_D3      16
#define CAMERA_PIN_D2      18
#define CAMERA_PIN_D1      17
#define CAMERA_PIN_D0      15
#define CAMERA_PIN_VSYNC   38
#define CAMERA_PIN_HREF    47
#define CAMERA_PIN_PCLK    13
```

### Camera Notes

- The OV2640 has been discontinued on newer production runs; replacement boards ship with the OV3660 (2048x1536). Both use the same DVP interface and pin assignments.
- With 8 MB PSRAM, you can buffer multiple frames for AI inference at a few FPS.
- Camera DMA and PSRAM use separate SPI buses from user-facing SPI, so there is no resource contention with I2C or SPI peripherals on D4–D10.

## Microphone Usage

The PDM microphone is configured via I2S in PDM RX mode:

```c
#include <driver/i2s_pdm.h>

i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000),  // 16 kHz sample rate
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
        .clk = GPIO_NUM_42,
        .din = GPIO_NUM_41,
    },
};
```

## Power Consumption

| State | Current | Notes |
|-------|---------|-------|
| Active (Wi-Fi TX) | ~310 mA peak | Depends on TX power level |
| Active (camera streaming) | ~200 mA | Continuous capture |
| Modem sleep | ~30 mA | CPU active, radio off |
| Light sleep | ~5 mA | |
| Deep sleep | ~14 µA | Camera/mic powered down |

## ESP-IDF Configuration

### sdkconfig.defaults for XIAO ESP32-S3 Sense

```ini
# Target
CONFIG_IDF_TARGET="esp32s3"

# PSRAM (8 MB octal — required for camera buffers)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y

# Camera
CONFIG_CAMERA_MODEL_XIAO_ESP32S3=y

# CPU frequency
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240=y

# Stack size (camera + WiFi + AI needs headroom)
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
```

### CMake target

```bash
idf.py set-target esp32s3
```

## In This Repository

The XIAO ESP32-S3 Sense is the target for:

- **[Unified Robocar](../../packages/esp32-projects/robocar-unified/)** — Single-board replacement for the dual Heltec + ESP32-CAM architecture ([ADR-013](../blueprint/adrs/ADR-013-single-board-xiao-esp32s3.md)). Camera for vision, I2C to TCA9548A/PCA9685 for motor control, WiFi for MQTT/OTA.

Future candidates:
- Standalone AI doorbell / security camera with on-device inference
- Voice-controlled device with wake word detection (ESP-SR)
- Environmental audio classifier (bird songs, baby cry detection)

## References

- [XIAO ESP32-S3 Sense Getting Started (Seeed Studio Wiki)](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [XIAO ESP32-S3 Camera Usage (Seeed Studio Wiki)](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/)
- [XIAO ESP32-S3 Microphone Usage (Seeed Studio Wiki)](https://wiki.seeedstudio.com/xiao_esp32s3_sense_mic/)
- [XIAO ESP32-S3 Sense Product Page](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)
- [XIAO ESP32-S3 Sense Pinout & Specs (espboards.dev)](https://www.espboards.dev/esp32/xiao-esp32s3-sense/)
- [XIAO ESP32-S3 Sense — DroneBot Workshop](https://dronebotworkshop.com/xiao-esp32s3-sense/)
- [ESP32-S3 Series Datasheet (Espressif)](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
