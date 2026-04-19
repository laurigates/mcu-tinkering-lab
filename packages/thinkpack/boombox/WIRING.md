# Boombox Wiring

## Pinout Table

| Signal           | ESP32-S3 Pin | Component          | Notes                              |
|------------------|-------------|--------------------|------------------------------------|
| Piezo+           | GPIO 5      | Passive piezo (+)  | LEDC PWM; 50 % duty cap            |
| Piezo−           | GND         | Passive piezo (−)  | Any GND pin                        |
| LED Data         | GPIO 4      | WS2812 DIN         | Add 300–500 Ω series resistor      |
| LED VCC          | 3V3         | WS2812 VCC         | 3.3 V sufficient for single LED    |
| LED GND          | GND         | WS2812 GND         |                                    |
| Pot 1 wiper      | GPIO 1      | Tempo pot centre   | ADC1_CH0                           |
| Pot 1 high       | 3V3         | Tempo pot end      |                                    |
| Pot 1 low        | GND         | Tempo pot end      |                                    |
| Pot 2 wiper      | GPIO 2      | Pitch pot centre   | ADC1_CH1                           |
| Pot 2 high       | 3V3         | Pitch pot end      |                                    |
| Pot 2 low        | GND         | Pitch pot end      |                                    |
| Button pin A     | GPIO 9      | Tactile button     | Internal pull-up enabled; active LOW |
| Button pin B     | GND         | Tactile button     |                                    |

## ASCII Diagram

```
ESP32-S3 SuperMini
┌─────────────────────────┐
│ 3V3 ──────────────────┬─┼──→ Pot1-high, Pot2-high, WS2812-VCC
│ GND ────────────────┬─┼─┼──→ Pot1-low, Pot2-low, Piezo−, BTN-B, WS2812-GND
│                     │ │ │
│ GPIO 1 (ADC1_CH0) ──┼─┴─┼──→ Pot1 wiper  (tempo)
│ GPIO 2 (ADC1_CH1) ──┼───┴──→ Pot2 wiper  (pitch)
│                     │
│ GPIO 4 ─────────────┼──[330Ω]──→ WS2812 DIN
│ GPIO 5 ─────────────┼──────────→ Piezo (+)
│ GPIO 9 ─────────────┴──────────→ Button pin A  (other pin → GND)
└─────────────────────────┘

Passive Piezo:
  (+) ── GPIO 5
  (−) ── GND

WS2812 Single LED:
  VCC ── 3V3
  GND ── GND
  DIN ── [330 Ω] ── GPIO 4

Tempo Pot (10 kΩ linear):
  Pin 1 (high) ── 3V3
  Pin 2 (wiper) ── GPIO 1
  Pin 3 (low)  ── GND

Pitch Pot (10 kΩ linear):
  Pin 1 (high) ── 3V3
  Pin 2 (wiper) ── GPIO 2
  Pin 3 (low)  ── GND

Button (tactile, SPST):
  Pin A ── GPIO 9
  Pin B ── GND
  (Internal pull-up on GPIO 9; button press pulls line LOW)
```

## Notes

- The ESP32-S3 SuperMini runs at 3.3 V. All signals are 3.3 V logic.
- A single WS2812 draws up to ~60 mA at full white; the onboard 3.3 V LDO
  on the SuperMini can supply this comfortably.
- Passive piezos are not polarised but require a square wave — LEDC PWM
  provides this directly. Do not use an active buzzer (it has its own
  oscillator and will ignore the frequency control).
- Keep pot wiring short to minimise ADC noise. A 100 nF capacitor from
  each wiper pin to GND improves stability.
