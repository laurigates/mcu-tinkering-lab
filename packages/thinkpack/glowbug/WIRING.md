# Glowbug Wiring

## Pinout Table

| GPIO | Label        | Connected to                          | Notes                        |
|------|--------------|---------------------------------------|------------------------------|
| 1    | LDR_ADC      | LDR/10kΩ divider midpoint            | ADC1_CH0; 0V=dark, 3.3V=bright |
| 4    | LED_DATA     | WS2812B DIN                           | 3.3V logic; 330Ω series R recommended |
| 6    | I2C_SDA      | MPU6050 SDA                           | Internal pull-up enabled      |
| 7    | I2C_SCL      | MPU6050 SCL                           | Internal pull-up enabled      |
| 9    | BUTTON       | Momentary switch → GND               | Internal pull-up; active LOW  |
| 3V3  | 3V3 rail     | MPU6050 VCC, LDR top, button one leg  |                               |
| GND  | GND rail     | MPU6050 GND, 10kΩ bottom, button GND |                               |
| 5V   | 5V rail      | WS2812B power (external supply)       | Do NOT power ring from USB 5V if > 8 LEDs |

## ASCII Schematic

```
  ESP32-S3 SuperMini
  ┌─────────────────────┐
  │                     │
  │  3V3 ───────────────┼──┬── MPU6050 VCC
  │                     │  ├── LDR (top leg)
  │                     │  └── Button (one leg)
  │                     │
  │  GND ───────────────┼──┬── MPU6050 GND
  │                     │  ├── 10kΩ (bottom leg → GND)
  │                     │  └── Button (other leg)
  │                     │
  │  GPIO6 (SDA) ───────┼────── MPU6050 SDA
  │  GPIO7 (SCL) ───────┼────── MPU6050 SCL
  │                     │
  │  GPIO1 ─────────────┼────── LDR ──── 3V3
  │           (ADC)     │   │
  │                     │  10kΩ
  │                     │   │
  │                     │  GND
  │                     │
  │  GPIO4 ─────[330Ω]──┼────── WS2812B DIN
  │                     │
  │  GPIO9 ─────────────┼────── Button ── GND
  │         (pull-up)   │
  │                     │
  │  5V (ext) ──────────┼────── WS2812B 5V power rail
  └─────────────────────┘
```

## Notes

- **WS2812B power**: For a 12-LED ring at full brightness each LED draws up
  to 60 mA.  The firmware caps brightness at 60%, so worst-case current is
  ~430 mA.  Use a dedicated 5V/1A supply and connect GND to the ESP32-S3 GND.

- **Series resistor on data line**: A 330Ω resistor in series on GPIO4 reduces
  ringing on the data line at cable lengths > 10 cm.  Short traces (< 5 cm)
  can omit it.

- **MPU6050 pull-ups**: The ESP32-S3 internal pull-ups (~45kΩ) work for short
  traces.  For cables > 20 cm, add external 4.7kΩ pull-ups from SDA/SCL to
  3V3 for reliable I2C at 100 kHz.

- **LDR voltage divider**: Use a 10kΩ LDR matched to a 10kΩ fixed resistor for
  best mid-range sensitivity.  Higher LDR resistance gives higher ADC reading.
  Swap LDR and fixed resistor positions if the reading direction is inverted.

- **Button**: Any momentary N.O. (normally open) push-button.  No external
  pull-up needed; the internal GPIO pull-up is enabled in firmware.
