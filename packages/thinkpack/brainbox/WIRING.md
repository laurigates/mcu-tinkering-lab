# ThinkPack Brainbox — Wiring Reference

## GPIO Pinout

| GPIO | Signal | Direction | Notes |
|------|--------|-----------|-------|
| 4 | WS2812 status LED data | Output | Single pixel; RMT driver, DMA disabled |
| 5 | Piezo buzzer | Output | LEDC channel 0, timer 0; passive piezo |
| 9 | Tactile button | Input | Internal pull-up enabled; active LOW; FALLING edge ISR |
| SDA (21) | OLED I2C data | Bidirectional | SSD1306 — driver stubbed in v0.1 |
| SCL (22) | OLED I2C clock | Output | SSD1306 — driver stubbed in v0.1 |

> Note: SDA/SCL pin numbers above are the ESP32-S3 WROOM defaults. Verify against
> your specific board schematic; adjust `I2C_NUM_0` configuration when the OLED
> driver is implemented.

---

## ASCII Wiring Diagram

```
                    ESP32-S3 WROOM N8R8
                   ┌─────────────────────┐
                   │                     │
        3V3 ───────┤ 3V3           GPIO4 ├──────── WS2812 DIN
        GND ───────┤ GND           GPIO5 ├──────── Piezo (+)
                   │               GPIO9 ├──────── Button (one side)
                   │                     │
                   │              GPIO21 ├──────── OLED SDA  (v0.2+)
                   │              GPIO22 ├──────── OLED SCL  (v0.2+)
                   │                     │
                   │              USB/JTAG│──────── Host (monitor/flash)
                   └─────────────────────┘

  WS2812 single pixel:
    VCC ──── 3V3
    GND ──── GND
    DIN ──── GPIO4

  Passive piezo:
    (+) ──── GPIO5
    (-) ──── GND
    (No transistor needed for a passive piezo driven from LEDC at 50 % duty)

  Tactile button:
    One leg ──── GPIO9
    Other leg ── GND
    (Internal pull-up keeps GPIO9 HIGH at rest; press pulls LOW)

  SSD1306 OLED (stubbed — connect when driver is implemented):
    VCC ──── 3V3
    GND ──── GND
    SDA ──── GPIO21
    SCL ──── GPIO22
```

---

## Power Notes

- The ESP32-S3 WROOM N8R8 draws ~250 mA peak during WiFi association and HTTP
  requests. Power via USB-C or a regulated 3.3 V rail capable of ≥500 mA.
- The WS2812 pixel is brightness-capped to 60 % (153/255) in firmware, limiting
  its draw to ~15 mA per channel maximum.
- The passive piezo draws negligible current at the 50 % LEDC duty cap.
