# ESP32 Kids Audio Toy

An interactive audio exploration toy for young children, combining ESP32 microcontroller capabilities with classic 555 timer circuits to create a rich sonic playground.

## 🎵 What It Does

This project creates a hands-on audio toy where kids can:
- **Turn knobs to change sounds** - Three potentiometers control pitch (high/low), beep duration (short/long), and rhythm (fast/slow)
- **Create interesting effects** - 555 timer circuits add modulation and dual-voice capabilities
- **See their sounds** - LED blinks in sync with audio output
- **Explore cause and effect** - Immediate tactile and auditory feedback

Perfect for 4-year-olds and up!

## ✨ Features

### Core Features (ESP32)
- **Pitch Control**: Adjust frequency from 100 Hz to 2000 Hz
- **Duration Control**: Beep length from 50ms to 1 second
- **Interval Control**: Rhythm/spacing between beeps
- **Visual Feedback**: LED blinks with each beep
- **Startup Tune**: Plays a little melody when powered on

### Advanced Features (with 555 Timers)
1. **Dual-Voice Mode**:
   - ESP32 generates primary melody
   - 555 timer creates bass drone or rhythmic counterpoint
   - Mix both signals for harmonious or interesting sonic textures

2. **Modulation Effects**:
   - 555 timer output feeds into ESP32 ADC
   - Creates vibrato, warble, or "robot voice" effects
   - Adds dynamic movement to static tones

## 🔧 Hardware Requirements

### Required Components
- ESP32 development board (any variant with ADC pins)
- 3× 10kΩ linear potentiometers (for pitch, duration, interval)
- 1× Piezo speaker or small 8Ω speaker
- 1× LED (any color)
- 1× 220Ω resistor (for LED)
- Breadboard and jumper wires
- USB power supply (5V)

### Optional Components (for 555 Features)
- 1-2× NE555 timer ICs
- Various resistors and capacitors (see WIRING.md for circuits)
- 1× 10kΩ potentiometer (for 555 frequency control)

### Tools
- Multimeter (for troubleshooting)
- ESP-IDF toolchain (for building and flashing)

## 🔌 Quick Start

### 1. Basic Setup (ESP32 Only)

```
ESP32 Connections:
├─ GPIO34 (ADC1_CH6) ← Pitch Potentiometer
├─ GPIO35 (ADC1_CH7) ← Duration Potentiometer
├─ GPIO32 (ADC1_CH4) ← Interval Potentiometer
├─ GPIO33 (ADC1_CH5) ← 555 Modulation Input (or leave unconnected)
├─ GPIO25 ← Piezo Speaker (+)
├─ GPIO2  ← LED (+) → 220Ω → GND
└─ GND ← Piezo Speaker (-), Pot wipers
```

See [WIRING.md](WIRING.md) for detailed schematics.

### 2. Build and Flash

```bash
# Navigate to project directory
cd packages/esp32-projects/kids-audio-toy

# Build the project
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor output (optional)
idf.py monitor
```

### 3. Play!

1. **Power on** - ESP32 plays a startup tune
2. **Turn the pitch knob** - Hear the tone go higher/lower
3. **Adjust duration** - Make beeps shorter or longer
4. **Change interval** - Speed up or slow down the rhythm
5. **Watch the LED** - It blinks in sync with each beep!

## 🎓 Educational Value

### For 4-Year-Olds
- **Cause and effect**: Turn knob → sound changes
- **Fine motor skills**: Precise knob turning
- **Auditory discrimination**: High vs low, fast vs slow
- **Pattern recognition**: Rhythm and repetition
- **Multi-sensory**: Sound + sight (LED) + touch (knobs)

### For Parents/Educators
- Introduction to electronics without screens
- Safe, low-voltage experimentation platform
- Expandable design - add features as child grows
- Open-source and hackable

## 🧪 How It Works

### ESP32 Audio Generation

The ESP32 uses **PWM (Pulse Width Modulation)** to generate audio tones:

1. **ADC Inputs**: Read potentiometer positions (0-3.3V → 0-4095 values)
2. **Mapping**: Convert ADC values to audio parameters (frequency, duration, timing)
3. **PWM Output**: Generate square wave at desired frequency
4. **Speaker Driver**: Square wave drives piezo speaker directly

### 555 Timer Integration

#### Mode 1: Modulation
```
555 Timer → Voltage Output → ESP32 ADC → Pitch Modulation
```
The 555's varying voltage level is read by the ESP32 and used to shift the pitch up/down in real-time, creating vibrato or warble effects.

#### Mode 2: Dual-Voice
```
555 Timer → Speaker (Bass/Drone)
ESP32 PWM → Speaker (Melody)
```
Both signals can be mixed (using resistors) to create harmony or call-and-response patterns.

### Why This Works for Kids

- **Immediate feedback**: No delay between action and result
- **Forgiving**: Can't break it by turning knobs "wrong"
- **Exploratory**: No right or wrong sounds, just experimentation
- **Tactile**: Physical knobs are more intuitive than touchscreens

## 🎛️ Customization Ideas

### Easy Modifications
- **Add more LEDs**: Create light patterns that follow pitch/rhythm
- **Different speakers**: Try buzzers, small speakers, or even a tin can
- **Enclosure**: 3D print or build a wooden box with exposed knobs
- **Paint/decorate**: Let your kid personalize it!

### Advanced Additions
- **Buttons**: Add "preset" sounds or change between modes
- **Photoresistor**: Light-controlled pitch (theremin-style)
- **Second piezo**: Stereo output or left/right patterns
- **Record/playback**: Save and replay favorite sound sequences
- **OLED display**: Show frequency numbers or patterns visually

## 🐛 Troubleshooting

### No Sound
- Check piezo polarity (some piezos are polarized)
- Verify GPIO25 connection
- Check serial output: `idf.py monitor`
- Ensure speaker isn't faulty (test with multimeter in continuity mode)

### Erratic Behavior
- Check potentiometer connections (especially ground)
- Verify 3.3V rail is stable
- Try increasing `MOD_SMOOTHING` in code (reduces jitter)

### 555 Not Working
- Verify 555 power (4.5-5V between VCC and GND)
- Check timing capacitor isn't shorted
- Test 555 output with LED before connecting to ESP32

### Build Errors
- Ensure ESP-IDF is properly installed: `idf.py --version`
- Check you're in correct directory
- Try `idf.py fullclean` then rebuild

## 📚 Learning Resources

### For Understanding the Code
- [ESP32 ADC Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html)
- [ESP32 LEDC (PWM) Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html)

### For 555 Timer Circuits
- [555 Timer Basics](https://www.electronics-tutorials.ws/waveforms/555_timer.html)
- [555 Astable Calculator](https://www.555-timer-circuits.com/calculator.html)

### For Young Learners
- Talk about "high sounds" (birds) vs "low sounds" (drums)
- Compare fast beeps (like a cricket) to slow beeps (like a bell)
- Relate frequencies to familiar sounds

## 🔐 Safety Notes

- **Low voltage**: Uses 5V USB power - safe for supervised children
- **No sharp edges**: Check enclosure before letting child handle
- **Supervision**: Always supervise young children with electronics
- **Piezo volume**: Piezo speakers are loud but safe at these power levels
- **Battery safety**: If using batteries, ensure they're secured and inaccessible

## 📜 License

This project is open source - feel free to modify, share, and improve!

## 🙋 Contributing

Found a bug? Have an idea? Create an issue or submit a pull request!

## 🎉 Have Fun!

The goal is exploration and play. There are no wrong sounds - just interesting ones! Let your kid experiment freely and discover the joy of making their own electronic music.

---

**Project Structure:**
```
kids-audio-toy/
├── main/
│   ├── main.c              # Main application code
│   └── CMakeLists.txt
├── CMakeLists.txt
├── sdkconfig.defaults      # ESP32 configuration
├── README.md               # This file
├── WIRING.md              # Detailed wiring diagrams
└── Makefile               # Build shortcuts (optional)
```
