# Wiring Guide - ESP32 Kids Audio Toy

This guide provides detailed wiring instructions for all configurations of the audio toy.

## Table of Contents
1. [Basic Setup (ESP32 Only)](#basic-setup-esp32-only)
2. [Advanced Setup (with 555 Modulation)](#advanced-setup-with-555-modulation)
3. [Dual-Voice Setup (555 + ESP32)](#dual-voice-setup-555--esp32)
4. [Full Featured Setup (Both 555 Modes)](#full-featured-setup-both-555-modes)
5. [Troubleshooting Tips](#troubleshooting-tips)

---

## Basic Setup (ESP32 Only)

This is the simplest configuration - just ESP32, potentiometers, and speaker.

### Parts List
- 1× ESP32 development board
- 3× 10kΩ linear potentiometers (B10K)
- 1× Piezo speaker or 8Ω speaker
- 1× LED (any color)
- 1× 220Ω resistor
- Breadboard and jumper wires

### Schematic

```
                    ESP32 Development Board
                    ┌─────────────────────┐
                    │                     │
   POT1 (Pitch)     │  GPIO34 (ADC1_CH6) ●─── POT1 wiper
      ┌─────┐       │                     │
   3.3V ─┤     ├─── wiper                │
   GND ──┤     │    │  GPIO35 (ADC1_CH7) ●─── POT2 wiper
      └─────┘       │                     │
                    │  GPIO32 (ADC1_CH4) ●─── POT3 wiper
   POT2 (Duration)  │                     │
      ┌─────┐       │  GPIO33 (ADC1_CH5) ●─── (unused for now)
   3.3V ─┤     ├─── wiper                │
   GND ──┤     │    │                     │
      └─────┘       │       GPIO25       ●─── Piezo (+)
                    │                     │
   POT3 (Interval)  │        GPIO2       ●─── LED (+) ─┬─ 220Ω ─── GND
      ┌─────┐       │                     │            │
   3.3V ─┤     ├─── wiper                │            └─ (LED cathode)
   GND ──┤     │    │         GND        ●─── Piezo (-)
      └─────┘       │                     │
                    │                     │
                    └─────────────────────┘

                        Speaker
                        ┌─────┐
              GPIO25 ───┤ (+) │
                        │     │
                GND ────┤ (-) │
                        └─────┘
```

### Connection Table

| Component | Pin | ESP32 GPIO | Notes |
|-----------|-----|------------|-------|
| Pitch Pot | Wiper | GPIO34 (ADC1_CH6) | Controls frequency |
| Duration Pot | Wiper | GPIO35 (ADC1_CH7) | Controls beep length |
| Interval Pot | Wiper | GPIO32 (ADC1_CH4) | Controls pause between beeps |
| All Pots | Side 1 | 3.3V | Power rail |
| All Pots | Side 2 | GND | Ground rail |
| Piezo/Speaker | (+) | GPIO25 | PWM output |
| Piezo/Speaker | (-) | GND | Ground |
| LED | Anode (+) | GPIO2 | Through 220Ω resistor |
| LED | Cathode (-) | GND | Ground |

### Breadboard Layout

```
    3.3V Rail  ─────────●───────●───────●─────────
                        │       │       │
                     ┌──┴──┐ ┌──┴──┐ ┌──┴──┐
                     │ POT1│ │ POT2│ │ POT3│  (Potentiometers)
                     └──┬──┘ └──┬──┘ └──┬──┘
                        │       │       │
                      GPIO34  GPIO35  GPIO32  (to ESP32)


    GND Rail   ─────────●───────●───────●─────────●──── Piezo (-)
                        │       │       │         │
                     ┌──┴──┐ ┌──┴──┐ ┌──┴──┐      │
                     │ POT1│ │ POT2│ │ POT3│      │
                     └─────┘ └─────┘ └─────┘      │
                                                   │
                            LED                    │
                             │                     │
                           GPIO2 ──► 220Ω ─────────┘

                         Piezo (+)
                             │
                           GPIO25
```

---

## Advanced Setup (with 555 Modulation)

Add a 555 timer to create pitch modulation effects (vibrato, warble).

### Additional Parts
- 1× NE555 timer IC
- 1× 10kΩ potentiometer (for 555 frequency control)
- 1× 100nF capacitor (C1)
- 1× 10µF capacitor (C2)
- 2× 10kΩ resistors (R1, R2)
- 1× 1kΩ resistor (R3 - voltage divider)
- 1× 2kΩ resistor (R4 - voltage divider)

### 555 Timer Circuit (Astable Mode)

```
                         +5V (VCC)
                          │
                          ├────────┐
                          │        │
                         ┌┴┐       │
                    R1   │ │ 10kΩ  │
                         │ │       │
                         └┬┘       │
                          │        │
                      ┌───┴───┐    │
                      │       │    │
           ┌──────────┤7 VCC  8├───┘
           │          │         │
           │     ┌────┤3  OUT  4├────┐ RESET (tied high)
           │     │    │         │    │
           │     │    │555      │    │
           │   ┌─┴──┐ │         │    │
           │   │ C1 │ │2  TRIG 1├────┤ (GND)
           │   │100n│ │         │    │
           │   └─┬──┘ │6 THRESH │    │
           │     │    └────┬────┘    │
           │     │         │         │
        POT4 ───┴─────────┴─────────┘
      (Freq)    │
       ┌──┐     │
    ───┤  ├─────┤
       └──┘     │
                │
               ┌┴┐
          R2   │ │ 10kΩ
               │ │
               └┬┘
                │
               ┌┴┐ C2
               ─── 10µF
               ─┬─
                │
               GND

        OUTPUT (Pin 3)
             │
             ├─── Voltage Divider ───► ESP32 GPIO33
             │      (R3 + R4)
             │      to scale 0-5V → 0-3.3V
```

### Voltage Divider for ESP32 Protection

**IMPORTANT**: The 555 outputs 0-5V, but ESP32 ADC is 3.3V max. Use a voltage divider:

```
        555 Output (Pin 3)
              │
              │
             ┌┴┐
        R3   │ │ 1kΩ
             │ │
             └┬┘
              ├────────► ESP32 GPIO33 (ADC1_CH5)
             ┌┴┐
        R4   │ │ 2kΩ
             │ │
             └┬┘
              │
             GND

    Output voltage = 5V × (2kΩ / (1kΩ + 2kΩ)) = 3.33V ✓
```

### 555 Frequency Calculation

The 555 frequency is determined by:

```
f = 1.44 / ((R1 + 2×R2) × C2)
```

With R1=10kΩ, R2=0-10kΩ (pot), C2=10µF:
- **Minimum**: f = 1.44 / ((10k + 20k) × 10µ) = 4.8 Hz
- **Maximum**: f = 1.44 / ((10k + 0) × 10µ) = 144 Hz

This creates a slow-to-fast warble effect!

---

## Dual-Voice Setup (555 + ESP32)

Mix both audio sources for harmony or counterpoint.

### Audio Mixer Circuit

```
        ESP32 (GPIO25)           555 Timer (Pin 3)
              │                        │
             ┌┴┐                      ┌┴┐
        R5   │ │ 1kΩ             R6   │ │ 1kΩ
             │ │                      │ │
             └┬┘                      └┬┘
              │                        │
              └────────┬───────────────┘
                       │
                       ├───────► Speaker (+)
                       │
                      ┌┴┐
                 C3   ─── 100µF (DC blocking)
                      ─┬─
                       │
                    Speaker (-)
                       │
                      GND
```

### How It Works

1. **R5 and R6** limit current and prevent the outputs from fighting each other
2. **C3** blocks DC component, passing only AC audio signal
3. Both sources mix acoustically in the speaker
4. ESP32 creates melody while 555 provides bass/drone

### Tuning Tips

- Start with 555 at low frequency (~10-30 Hz) for bass drone
- Adjust ESP32 pitch pot to create harmony
- Try 555 at ~2-5 Hz for rhythmic "thump-thump" effect
- For call-and-response, use ESP32 interval control

---

## Full Featured Setup (Both 555 Modes)

Use TWO 555 timers for maximum fun!

### Configuration

- **555 Timer #1**: Modulation source → ESP32 GPIO33 (with voltage divider)
- **555 Timer #2**: Audio output → Mixed with ESP32 → Speaker

### Block Diagram

```
    POT1 ──┐
    POT2 ──┤
    POT3 ──┤
           ▼
        ESP32 ───► PWM Audio ──┐
           ▲                   │
           │                   ▼
    555 #1 ┘              Audio Mixer ───► Speaker
    (Mod)                      ▲
                               │
                        555 #2 ┘
                        (Bass)
```

### Power Distribution

```
                    +5V USB Power
                         │
         ┌───────────────┼───────────────┐
         │               │               │
      ESP32          555 #1          555 #2
      (3.3V)         (Pin 8)         (Pin 8)
         │               │               │
         └───────────────┴───────────────┘
                        GND
```

**Note**: ESP32 has onboard 3.3V regulator. Use 5V rail for 555 timers.

---

## Troubleshooting Tips

### Problem: Potentiometers are "backwards"

If turning clockwise decreases the value instead of increasing:
- **Hardware fix**: Swap the outer two pins of the potentiometer
- **Software fix**: Invert in code: `value = 4095 - adc_reading`

### Problem: 555 output too noisy

- Add 100nF capacitor between VCC and GND (decoupling cap)
- Ensure power supply is stable (measure with multimeter)
- Keep wires short, especially around timing capacitor

### Problem: ESP32 ADC readings are jittery

- Increase `MOD_SMOOTHING` value in code (e.g., 0.9)
- Add 100nF capacitor across pot wiper and ground
- Use shielded cable for long pot wires
- Enable ESP32's built-in ADC averaging (see code)

### Problem: Speaker is quiet

- Piezos work best at their resonant frequency (~2-4kHz)
- If using magnetic speaker, try 100Ω-470Ω resistor in series
- For louder output, use a small amplifier (PAM8403 module)
- Check PWM duty cycle is 50% (not 0% or 100%)

### Problem: 555 won't oscillate

1. Check power: 4.5-5V between pins 8 and 1
2. Verify capacitor isn't shorted (use multimeter)
3. Check timing resistor values (10kΩ minimum for R1+R2)
4. Ensure RESET (pin 4) is tied to VCC
5. Test with LED on output (pin 3) before connecting to ESP32

### Problem: ESP32 damaged from 5V

If you connected 555 output directly to ESP32 ADC:
- **Prevention**: Always use voltage divider (1kΩ + 2kΩ)
- **Check**: Measure divided output with multimeter (<3.3V)
- ESP32 ADC pins are NOT 5V tolerant!

### Debugging Tips

1. **Test in stages**:
   - First: ESP32 only
   - Second: Add one 555
   - Third: Add second 555

2. **Use serial monitor**: Watch ADC values and frequency output
   ```bash
   idf.py monitor
   ```

3. **Visual testing**:
   - Use LED on 555 output to see oscillation
   - ESP32 LED already shows beeps

4. **Audio testing**:
   - Test piezo with multimeter in AC voltage mode
   - Should see voltage swinging during beeps

---

## Safety Reminders

✓ Always power off before changing wiring
✓ Double-check polarity of electrolytic capacitors
✓ Verify voltage divider before connecting to ESP32
✓ Don't exceed 3.3V on any ESP32 GPIO pin
✓ Keep liquids away from circuit
✓ Supervise children around electronics

---

## Component Substitutions

### Potentiometers
- **10kΩ** (specified) works well
- Can use 5kΩ to 100kΩ without code changes
- Linear (B) taper recommended (A/audio taper works too)

### Capacitors (555 timing)
- Change C2 to adjust 555 frequency range:
  - **1µF**: Higher frequencies (10-1440 Hz)
  - **10µF**: Medium frequencies (1-144 Hz) ← Default
  - **100µF**: Lower frequencies (0.1-14 Hz) - bass drone

### Speakers
- **Piezo buzzers**: Loudest, best for kid toy
- **8Ω speakers**: Fuller sound, need amplifier for volume
- **Magnetic buzzers**: Work but quieter than piezo

### ESP32 Variants
- Works with any ESP32 with ADC pins
- ESP32-S2/S3: Check pinout (ADC channels differ)
- ESP32-C3: Only 6 ADC channels, may need pin reassignment

---

## Expansion Ideas

### More Controls
- Add 4th pot on GPIO33 (if not using 555 modulation)
- Use for: waveform selection, effects depth, volume

### Sensors
- **Light sensor** (photoresistor): Light-controlled pitch
- **Distance sensor** (ultrasonic): Proximity-based effects
- **Tilt sensor**: Angle affects parameters

### Outputs
- **RGB LED**: Color changes with pitch
- **LED strip**: VU meter or pattern display
- **Second speaker**: Stereo effects

### Advanced
- **Buttons**: Mode selection, presets, record/playback
- **OLED display**: Show frequency, settings
- **SD card**: Save/load patterns
- **WiFi**: Networked toys play together!

---

**Questions?** Check the main README.md or open an issue!

Happy building! 🎵🔧
