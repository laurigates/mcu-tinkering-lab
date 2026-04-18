# Wiring — robocar-unified (XIAO ESP32-S3 Sense)

Single-board wiring for the consolidated robocar. All pin assignments are authoritative in [`main/pin_config.h`](main/pin_config.h); this document mirrors them for human reference.

**All components must share a common ground (GND).**

## GPIO assignments (XIAO ESP32-S3 Sense)

The XIAO exposes only 11 GPIOs on its headers. Camera pins are internal to the Sense module and do not conflict with header pins.

| XIAO Pin | GPIO | Function | Notes |
|----------|------|----------|-------|
| D0 | GPIO1 | TB6612FNG STBY | HIGH = motors enabled |
| D1 | GPIO2 | Piezo buzzer | LEDC PWM |
| D2 | GPIO3 | Ultrasonic TRIG | 10 µs pulse output |
| D3 | GPIO4 | Ultrasonic ECHO | Pulse width input (RMT RX) |
| D4 | GPIO5 | **I2C SDA** | to TCA9548A |
| D5 | GPIO6 | **I2C SCL** | to TCA9548A |
| D6 | GPIO43 | USB Serial TX | debug console |
| D7 | GPIO44 | USB Serial RX | debug console |
| D8–D10 | GPIO7–9 | *spare* | future: SPI or expansion |

I2C runs at **400 kHz**.

## I2C topology (TCA9548A multiplexer @ 0x70)

```mermaid
graph LR
    ESP32[XIAO ESP32-S3<br/>GPIO5/6] -->|I2C 400kHz| TCA[TCA9548A<br/>0x70]
    TCA -->|ch0| PCA[PCA9685<br/>0x40 @ 200Hz]
    TCA -->|ch1| OLED[SSD1306 OLED<br/>0x3C, 128x64]
    TCA -.->|ch2-7| Future[reserved:<br/>IMU / ToF / sensors]
```

## PCA9685 channel map (0x40, 200 Hz)

| Ch | Signal | Device |
|----|--------|--------|
| 0 | Left LED R | RGB LED (left) |
| 1 | Left LED G | |
| 2 | Left LED B | |
| 3 | Right LED R | RGB LED (right) |
| 4 | Right LED G | |
| 5 | Right LED B | |
| 6 | Pan PWM | SG90 servo |
| 7 | Tilt PWM | SG90 servo |
| 8 | Motor R IN1 | TB6612FNG (digital: 0 / 4096) |
| 9 | Motor R IN2 | TB6612FNG (digital) |
| 10 | Motor R PWM | TB6612FNG (PWM 0-4095) |
| 11 | Motor L IN1 | TB6612FNG (digital) |
| 12 | Motor L IN2 | TB6612FNG (digital) |
| 13 | Motor L PWM | TB6612FNG (PWM 0-4095) |
| 14-15 | *reserved* | |

200 Hz is a compromise between servo timing (ideal 50 Hz) and motor PWM smoothness — works well for SG90s and TB6612FNG.

## Power

```mermaid
graph TD
    Bat[2x 18650 battery pack] --> Boost[XL6009 boost → 5V]
    Boost -->|5V| XIAO[XIAO ESP32-S3 Sense<br/>5V pin]
    Boost -->|5V| MD[TB6612FNG<br/>VM + VCC]
    Boost -->|5V| PCA[PCA9685<br/>V+ + VCC]
    Boost -->|5V| Servos[SG90 servos]
    MD --> ML[Left motor]
    MD --> MR[Right motor]
    PCA --> LED_L[Left RGB LED<br/>common-anode]
    PCA --> LED_R[Right RGB LED<br/>common-anode]
    PCA --> Servos
    XIAO -->|GPIO2| Piezo[Piezo buzzer]
    XIAO -->|GPIO1| MD
    classDef gnd fill:#ccc,stroke:#333
```

**Common ground required across all components.**

## Ultrasonic rangefinder

A 3.3 V-compatible ultrasonic sensor (HC-SR04P, RCWL-1601, or US-100) provides distance readings for the reactive controller's obstacle reflex.

| Signal | Pin | Voltage | Function |
|--------|-----|---------|----------|
| TRIG | GPIO3 (D2) | 3.3 V | Output; 10 µs pulse triggers measurement |
| ECHO | GPIO4 (D3) | 3.3 V | Input; pulse width encodes distance (RMT RX) |
| VCC | 3.3 V | 3.3 V | **Must be 3.3 V variant** (HC-SR04P, not HC-SR04) |
| GND | any GND | – | shared ground |

The sensor samples at ~20 Hz. Obstacle reflex: if distance < 15 cm, the executor immediately stops and reverses, independent of planner goals. The specific module will be confirmed on first wiring; update this table if a different 3.3 V sensor is used.

## Full connection diagram

```mermaid
graph TD
    subgraph Board[XIAO ESP32-S3 Sense]
        ESP[ESP32-S3<br/>+ OV2640 camera<br/>+ 8MB PSRAM]
    end

    ESP -- GPIO5 SDA --> TCA[TCA9548A 0x70]
    ESP -- GPIO6 SCL --> TCA
    ESP -- GPIO1 STBY --> TB[TB6612FNG]
    ESP -- GPIO2 --> BUZ[Piezo]

    TCA -- ch0 --> PCA[PCA9685 0x40]
    TCA -- ch1 --> OLED[SSD1306 0x3C]

    PCA -- ch0-5 --> LEDS[2x RGB LEDs]
    PCA -- ch6-7 --> SRV[Pan/Tilt SG90]
    PCA -- ch8-13 --> TB
    TB --> M1[Left motor]
    TB --> M2[Right motor]
```

## Flashing

XIAO ESP32-S3 has native USB-Serial-JTAG (VID `0x303a`). Plug in USB-C and flash:

```bash
PORT=/dev/cu.usbmodem* just robocar-unified::flash
```

If the board won't enter download mode: hold **BOOT**, tap **RESET**, release BOOT. See [README.md](README.md) for full flash / monitor commands.
