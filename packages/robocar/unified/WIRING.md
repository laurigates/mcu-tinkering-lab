# Wiring — robocar-unified (XIAO ESP32-S3 Sense)

Single-board wiring for the consolidated robocar. All pin assignments are authoritative in [`main/pin_config.h`](main/pin_config.h); this document mirrors them for human reference. The pin tables and the power diagram marked `GENERATED` are emitted from [`hardware.toml`](hardware.toml), the header and the board reference by `just hardware::gen` (ADR-021), and CI fails when they are stale — change those sources, not the generated blocks.

![Schematic](../../../docs/schematics/images/robocar_unified.png)

Schematic source: [`docs/schematics/circuits/robocar_unified.py`](../../../docs/schematics/circuits/robocar_unified.py). Re-render with `just schematics::render-one robocar_unified` after pin changes.

**All components must share a common ground (GND).**

## GPIO assignments (XIAO ESP32-S3 Sense)

The XIAO exposes only 11 GPIOs on its headers. Camera pins are internal to the Sense module and do not conflict with header pins.

<!-- BEGIN GENERATED: pin-table -->
<!-- Generated from hardware.toml, main/pin_config.h and the board reference by `just hardware::gen` — edit those, not this block. -->

| Pin | GPIO | Macro | Wired to | Notes |
|-----|------|-------|----------|-------|
| D0 | GPIO1 | `MOTOR_STBY_PIN` | TB6612FNG STBY | HIGH = motors enabled; the six control lines come from the PCA9685 |
| D1 | GPIO2 | `PIEZO_PIN` | Piezo buzzer + | LEDC PWM, through a series resistor; the other leg to GND |
| D2 | GPIO3 | `TCA9548A_RST_PIN` | TCA9548A RST | Active-low mux reset, pulsed after repeated failed channel selects; the breakout pulls it up |
| D3 | GPIO4 | — | *unassigned* |  |
| D4 | GPIO5 | `I2C_SDA_PIN` | TCA9548A SDA | Every I2C device sits behind the mux |
| D5 | GPIO6 | `I2C_SCL_PIN` | TCA9548A SCL |  |
| D6 | GPIO43 | `UART0_TX_PIN` | — | UART0 TX, spare: the serial console is USB-Serial-JTAG on the USB-C connector, not this pad. The ROM bootloader prints its boot log here at every reset |
| D7 | GPIO44 | `UART0_RX_PIN` | — | UART0 RX, spare: the serial console is USB-Serial-JTAG on the USB-C connector, not this pad |
| D8 | GPIO7 | `I2S_BCLK_PIN` | MAX98357A BCLK | Bit clock |
| D9 | GPIO8 | `I2S_LRCLK_PIN` | MAX98357A LRC | Word select / left-right clock |
| D10 | GPIO9 | `I2S_DIN_PIN` | MAX98357A DIN | Serial audio data |

<!-- END GENERATED -->

> **Two spare pads: D3 (GPIO4) and D7 (GPIO44, UART0 RX).** Nothing uses
> them; D3 was the ultrasonic ECHO until the rangefinders moved to I2C (#782).
> D6 (GPIO43, UART0 TX) is an output, not a spare: the ROM bootloader prints its
> boot log there at every reset, and the firmware then mirrors every `ESP_LOG`
> line to it at 115200 8N1 (`main/log_mirror.c`, #798). A USB-serial adapter's
> RX on D6 (J15 on the carrier) reads both. Beyond those, digital I/O goes
> through the MCP23017 on TCA9548A channel 2.

I2C runs at **400 kHz**.

## I2C topology (TCA9548A multiplexer @ 0x70)

Select the channel on the multiplexer **before** addressing any downstream
device — nothing talks to the primary bus directly.

| Channel | Device | Address |
|---------|--------|---------|
| ch0 | PCA9685 PWM driver (motors, servos, LEDs) | 0x40 @ 100 Hz |
| ch1 | SSD1306 OLED display (128x64) | 0x3C |
| ch2 | MCP23017 GPIO expander — **optional**, firmware boots without it | 0x20 |
| ch3 | RCWL-1601 rangefinder, front (fixed) | 0x57 |
| ch4 | RCWL-1601 rangefinder, rear | 0x57 |
| ch5-7 | *reserved* (IMU / ToF / future sensors) | — |

The multiplexer's own pins from the XIAO. `RST` is active low: the firmware
pulses it after several channel selects fail in a row, which deselects every
channel and frees the upstream bus from a segment held low (see the
rangefinder section).

<!-- BEGIN GENERATED: signals:mux -->
<!-- Generated from hardware.toml, main/pin_config.h and the board reference by `just hardware::gen` — edit those, not this block. -->

| Signal | Pin | Function |
|--------|-----|----------|
| SDA | GPIO5 (D4) | Every I2C device sits behind the mux |
| SCL | GPIO6 (D5) |  |
| RST | GPIO3 (D2) | Active-low mux reset, pulsed after repeated failed channel selects; the breakout pulls it up |

<!-- END GENERATED -->

## PCA9685 channel map (0x40, 100 Hz)

<!-- BEGIN GENERATED: channels:pwm -->
<!-- Generated from hardware.toml, main/pin_config.h and the board reference by `just hardware::gen` — edit those, not this block. -->

| Ch | Macro | Wired to | Notes |
|----|-------|----------|-------|
| 0 | `LED_LEFT_R_CHANNEL` | Left RGB LED **R** |  |
| 1 | `LED_LEFT_G_CHANNEL` | Left RGB LED **G** |  |
| 2 | `LED_LEFT_B_CHANNEL` | Left RGB LED **B** |  |
| 3 | `LED_RIGHT_R_CHANNEL` | Right RGB LED **R** |  |
| 4 | `LED_RIGHT_G_CHANNEL` | Right RGB LED **G** |  |
| 5 | `LED_RIGHT_B_CHANNEL` | Right RGB LED **B** |  |
| 6 | `SERVO_PAN_CHANNEL` | SG90 servos **PAN** | Signal lead of the pan servo |
| 7 | `SERVO_TILT_CHANNEL` | SG90 servos **TILT** | Signal lead of the tilt servo |
| 8 | `MOTOR_RIGHT_PWM_CHANNEL` | TB6612FNG **PWMA** | Right motor speed, PWM 0-4095 |
| 9 | `MOTOR_RIGHT_IN2_CHANNEL` | TB6612FNG **AIN2** | Right motor direction, full-on / full-off |
| 10 | `MOTOR_RIGHT_IN1_CHANNEL` | TB6612FNG **AIN1** | Right motor direction, full-on / full-off |
| 11 | `MOTOR_LEFT_IN1_CHANNEL` | TB6612FNG **BIN1** | Left motor direction, full-on / full-off |
| 12 | `MOTOR_LEFT_IN2_CHANNEL` | TB6612FNG **BIN2** | Left motor direction, full-on / full-off |
| 13 | `MOTOR_LEFT_PWM_CHANNEL` | TB6612FNG **PWMB** | Left motor speed, PWM 0-4095 |
| 14 | — | *reserved* |  |
| 15 | — | *reserved* |  |

<!-- END GENERATED -->

100 Hz is a chip-wide compromise — one prescaler serves the servos, the motors and the LEDs. Measured on this build (2026-09-18): the SG90s fitted track at 50, 100 and 125 Hz and **buzz at 200**, stalling against the pulse train instead of following it. 100 sits one rung below the highest rate that worked, because the bench test was unloaded and a loaded servo has less timing margin. `servo freq <hz>` retunes it live without a reflash.

**Channels 8-13 are in the motor driver's own pin order, not in a per-motor
order.** Read the TB6612FNG's control header top to bottom and it is PWMA,
AIN2, AIN1, STBY, BIN1, BIN2, PWMB — symmetric about STBY rather than repeated
— so following it makes the six jumpers run straight across with no crossings,
at the cost of the A side reading PWM-then-direction and the B side
direction-then-PWM. STBY is skipped here because it comes from GPIO1, not from
the PCA9685. `docs/wiring-card-motors.pdf` draws both boards; `main/pin_config.h`
is authoritative, and `set_motors()` places each value by channel rather than by
position so this ordering cannot silently mis-drive a pin.

## Power

Solid edges are supply rails, labelled with the rail and the pin it lands on;
dotted edges are the XIAO's signal nets, each with its GPIO and header pad;
unlabelled edges are loads a part drives from its own terminals. The diagram is
emitted from the `[[rails]]`, `[[nets]]` and `[[outputs]]` in
[`hardware.toml`](hardware.toml), so a pin or rail changed there or in
`main/pin_config.h` reaches it through `just hardware::gen`.

<!-- BEGIN GENERATED: power-diagram -->
<!-- Generated from hardware.toml, main/pin_config.h and the board reference by `just hardware::gen` — edit those, not this block. -->

```mermaid
graph TD
    pack["2x 18650 in series<br/>7.4 V nominal, 8.4 V charged"]
    buck["LM2596 buck module<br/>adjust to 5.0 V before connecting a load"]
    mcu["XIAO ESP32-S3 Sense"]
    motor_driver["TB6612FNG"]
    pwm["PCA9685"]
    amp["MAX98357A"]
    mux["TCA9548A"]
    ranger_front["RCWL-1601 (front)"]
    ranger_rear["RCWL-1601 (rear)"]
    oled["SSD1306 OLED"]
    expander["MCP23017<br/>optional"]
    buzzer["Piezo buzzer"]
    servos["SG90 servos<br/>pan, tilt"]
    led_left["Left RGB LED<br/>common-anode"]
    led_right["Right RGB LED<br/>common-anode"]
    motor_left["Left motor"]
    motor_right["Right motor"]
    speaker["Speaker<br/>4–8 Ω"]
    pack -->|"VBAT → IN+"| buck
    buck -->|"5V → 5V"| mcu
    buck -->|"5V → VM"| motor_driver
    buck -->|"5V → V+"| pwm
    buck -->|"5V → Vin"| amp
    mcu -->|"3V3 → VCC"| motor_driver
    mcu -->|"3V3 → VCC"| pwm
    mcu -->|"3V3 → VIN"| mux
    mcu -->|"3V3 → VCC"| ranger_front
    mcu -->|"3V3 → VCC"| ranger_rear
    mcu -->|"3V3 → VCC"| oled
    mcu -->|"3V3 → VCC"| expander
    mcu -.->|"GPIO5 (D4) → SDA<br/>GPIO6 (D5) → SCL<br/>GPIO3 (D2) → RST"| mux
    mcu -.->|"GPIO1 (D0) → STBY"| motor_driver
    mcu -.->|"GPIO2 (D1) → +"| buzzer
    mcu -.->|"GPIO7 (D8) → BCLK<br/>GPIO8 (D9) → LRC<br/>GPIO9 (D10) → DIN"| amp
    pwm --> servos
    pwm --> led_left
    pwm --> led_right
    motor_driver --> motor_left
    motor_driver --> motor_right
    amp --> speaker
```

<!-- END GENERATED -->

**Common ground required across all components.**

### Logic rails are 3.3 V, not 5 V

The TB6612FNG's **VCC** and the PCA9685's **VCC** are logic supplies and take
**3.3 V**; only the TB6612FNG's **VM** and the PCA9685's **V+** take 5 V. This
diagram fed 5 V to both VCC pins until 2026-09; that was wrong, and the
arithmetic is not close:

| Part | Threshold | At V_CC = 5 V | At V_CC = 3.3 V |
|---|---|---|---|
| PCA9685 SCL/SDA | V_IH = 0.7 x V_DD | 3.5 V — above what the XIAO's 3.3 V I2C can drive | 2.31 V |
| TB6612FNG IN1/IN2/PWM | V_IH = 0.7 x V_CC | 3.5 V — above what the PCA9685 would output | 2.31 V |
| TB6612FNG STBY | V_IH = 0.7 x V_CC | 3.5 V — a 3.3 V GPIO cannot reliably lift it, so the motors stay in standby | 2.31 V |

Both parts specify their input threshold as a fraction of *their own* supply, so
the rail is not a free choice. `docs/wiring-card-motors.typ` and
`docs/schematics/circuits/robocar_unified.py` carry the same 3.3 V assignment.

**Check before power-up:** continuity between TB6612FNG VM and TB6612FNG VCC
must read open. A beep means the 5 V and 3.3 V rails are shorted.

### The regulator is a step-DOWN converter, and its output is adjustable

The pack is **two 18650 cells in series** — 7.4 V nominal, 8.4 V fully charged —
regulated to 5 V by an **LM2596 buck module**. Series and buck go together: a
step-down converter needs its input above its output, so a parallel (3.7 V) pack
could not feed it.

The common LM2596 module is the **adjustable** variant with a multi-turn
trimpot, so it does not produce 5 V until somebody sets it there.

- **Set the output before connecting anything to it.** Power the module from the
  pack with its output unloaded, and adjust to **5.0 V** on a meter. It can be
  turned anywhere from 1.2 V to 37 V.
- **5.5 V is the ceiling that matters.** The MAX98357A's recommended maximum
  supply is 5.5 V (absolute maximum 6 V), and this rail also feeds the XIAO's
  5V pin into its onboard regulator. A trimpot left high damages parts; a
  trimpot left low gives weak servos and a clipping amplifier.

### The 5 V rail dies before the cells do

The LM2596 needs its input roughly **1.25 V above its output at 3 A**, and about
**0.95 V above at 1 A** (TI SNVS124G, Figure 7-6). At 5 V out that puts the
dropout point near **6.3 V of pack under heavy load** — about 3.15 V per cell,
which a 2S 18650 pack reaches while it still has usable charge left.

So the failure is not a clean shutdown. As the pack sags the 5 V rail follows it
down, and because `CONFIG_ESP_BROWNOUT_DET` is disabled (see below) nothing
announces it: the symptoms are weak or stalled servos, clipping or distorted
audio, and eventually random resets. **A distorted voice is a plausible
low-battery indication on this robot.** Measure the pack before diagnosing
anything else on this rail.

Nothing on the robot stops the discharge either. The LM2596 has an ON/OFF pin
(TI SNVS124G, § 6), but the common module ties it to ground on its own board,
so the regulator keeps drawing as the cells fall past their safe minimum. **Use
a pack with a BMS**: its cell undervoltage cutoff is the only thing that ends
the discharge. The carrier adds a 3 A hold PTC fuse (F1) at the pack input for
a short, not for a flat pack.

### The 5 V current budget

The LM2596's current limit is 3.4 A minimum over temperature (SNVS124G,
§ 7.9). Every 5 V load shares it: both motors through the TB6612FNG, the
servos, the amplifier's peaks and the XIAO. The physics simulation assumes
about 1.7 A per stalled motor, so two stalls alone reach the limit, and past it
the regulator folds back and the rail sags into the symptoms above. That figure
is an assumption: **measure each motor's stall current** at 5 V before relying
on the budget. If two stalls do exceed it, cap and ramp the motor PWM duty in
firmware.

### Star-wire the rail; do not daisy-chain

Every 5V load in the diagram takes its own feed from the regulator's output
terminal. This is load-bearing rather than tidiness: a servo's inrush flowing
through the amplifier's feed wire modulates the amplifier's local supply, which
is heard as distortion. The motor driver, the PCA9685/servos and the amplifier
are the three transient sources, and none of them should share a run.

### Amplifier supply — read before wiring

`CONFIG_ESP_BROWNOUT_DET` is **already disabled** in this project because motor
inrush was tripping it. The MAX98357A adds transient draw of up to ~1 A into a
4 Ω load, on the same rail, at moments uncorrelated with motor current. With
brownout detection off, an undersized rail will not warn you — it will present
as random resets or corrupt audio mid-sentence.

- Fit a **bulk capacitor (470 µF) at the amplifier's Vin** — C3 below. The
  breakout already carries the datasheet's 0.1 µF + 10 µF next to the chip, so
  nothing smaller is needed there. Fit the same at the PCA9685's **V+** (C1):
  the servos are the harsher transient source of the two.
- **Never power servos from the XIAO's 5V pin or from USB VBUS.** That pin is a
  regulator input, not a supply output, and a USB host port cannot source what
  two SG90s and a class-D amplifier draw. Servos that buzz without moving are
  the signature.
- An **8 Ω speaker roughly halves peak current** versus 4 Ω and is the safer
  first choice while validating the supply.

### Powering from the regulator with USB also connected

Both at once is the normal bench case — USB for the console, the pack for the
motors. Whether it is safe depends on whether the XIAO's 5V pad is isolated from
USB VBUS by a series diode, which **is worth measuring rather than assuming**:
with USB alone connected and the regulator off, read the 5V pad against GND.
Roughly 5.0 V means it is tied straight to VBUS and feeding the rail would
back-feed the host port; roughly 4.6–4.7 V means a diode is in the way and the
higher source simply wins.

The arrangement that avoids the question entirely, and the better one for
debugging anyway: **regulator → amplifier, PCA9685 and motor driver; USB → the
XIAO; grounds commoned.** The MCU then sits on a rail that servo and motor
inrush cannot sag.

Do not connect anything to the XIAO's BAT pads while feeding its 5V pin — the
onboard charger will try to charge whatever it finds there.

### Suggested capacitors

The schematic draws six capacitors that no breakout carries, labelled C1–C6.
C1–C3 are recommended; C4–C6 are optional. Each was chosen by reading what the
vendor's board file already fits (the Eagle file at the commit its
`docs/reference/boards/` page cites) against what the part's datasheet asks for.

| Ref | Part | Fit at | Status | Why, and the source |
|-----|------|--------|--------|---------------------|
| C1 | 470 µF electrolytic, 16 V | PCA9685 V+ | Recommended | The Adafruit board leaves a through-hole electrolytic footprint on the V+ net empty for the builder (vendor ref C2, 3.5 mm pitch). Adafruit's guide suggests n × 100 µF for n servos as a start — 200 µF for the two SG90s — and says the right value depends on the servos and the supply; 470 µF matches C3. |
| C2 | 470 µF electrolytic, 16 V | TB6612FNG VM | Recommended | Toshiba's typical application puts 10 µF + 0.1 µF on VM "as close as possible to the IC", and the SparkFun board fits both (vendor refs C3, C1). The bulk part is for motor start and stall current arriving over a jumper run instead of a short trace — the case C3's datasheet sentence describes. |
| C3 | 470 µF electrolytic, 16 V | MAX98357A Vin | Recommended | Maxim: "Bypass VDD with a 0.1 µF and 10 µF capacitor to GND" — both on the Adafruit board (vendor refs C1, C2) — and "apply additional bulk capacitance at the ICs if long input traces between VDD and the power source are used". A jumper from the LM2596 is a long input trace. |
| C4 | 100 nF ceramic | PCA9685 VCC | Optional | The Adafruit board fits only a 10 µF (vendor ref C1) on VCC. NXP's datasheet FAQ: about 50 pF of decoupling is on-chip, and whether to add external decoupling as close as possible to the device is left to the designer when many outputs switch together. |
| C5 | 100 nF ceramic | TCA9548A VIN | Optional | The Adafruit board fits only a 10 µF (vendor ref C1). TI's layout guidance (SCPS207H §8.4.1) pairs a larger capacitor for supply glitches with a smaller one for high-frequency ripple; this is the smaller one. |
| C6 | 100 nF ceramic | MCP23017 VCC | Optional | The module fitted is unidentified (#662), so whether it carries one is unknown. Microchip's datasheet (DS20001952) names no value; 100 nF is generic practice. Skip it if the module already has a capacitor beside the chip. |

Fit each one **at the pin it names**, across that pin and the nearest GND pad,
with short leads. A capacitor at the regulator's end of a jumper cannot supply a
transient at the far end — the jumper's own resistance and inductance sit
between them, which is the star-wiring argument above applied to the capacitor.

**16 V is margin, not a guess.** The rail is bucked from a pack that reaches
8.4 V, and a step-down converter cannot raise its output above its input, so
even a trimpot turned fully up leaves a 10 V part inside its rating. Fit the
electrolytics with the stripe (−) to GND; the schematic marks the + plate.

One datasheet recommendation is deliberately not drawn: Toshiba also asks for
10 µF on the TB6612FNG's **VCC**, and the SparkFun board fits only 0.1 µF there
(vendor ref C2). That pin draws 1.1 mA typical (datasheet Icc at 3 V) from the XIAO's 3V3
pad, so it is left out; it is the first part to add if STBY or the control
inputs ever misbehave under motor load.

Datasheets: [TB6612FNG](https://cdn.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf),
[PCA9685](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf) and
[Adafruit's PCA9685 guide](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf),
[MAX98357A](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX98357A-MAX98357B.pdf),
[TCA9548A](https://www.ti.com/lit/ds/symlink/tca9548a.pdf),
[MCP23017](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP23017-Data-Sheet-DS20001952.pdf).
`docs/schematics/circuits/robocar_unified.py` (`SUGGESTED_CAPS`) is the list
the drawing is made from, and its tests fail if this table or the build guide
drifts from it.

## Audio output (MAX98357A)

Mono I2S class-D amplifier providing the robot's voice. Audio is 24 kHz 16-bit
mono — the native output rate of the Gemini TTS model, carried through without
resampling.

<!-- BEGIN GENERATED: signals:amp -->
<!-- Generated from hardware.toml, main/pin_config.h and the board reference by `just hardware::gen` — edit those, not this block. -->

| Signal | Pin | Function |
|--------|-----|----------|
| BCLK | GPIO7 (D8) | Bit clock |
| LRC | GPIO8 (D9) | Word select / left-right clock |
| DIN | GPIO9 (D10) | Serial audio data |

<!-- END GENERATED -->

Power and configuration pins:

| Signal | Pin | Function |
|--------|-----|----------|
| Vin | 5 V | See supply note above |
| GND | any GND | Shared ground |
| SD_MODE | *(see below)* | Channel select / shutdown |
| GAIN | float | 9 dB default; tie to GND for 12 dB, Vin for 6 dB |

`SD_MODE` selects the channel: leave **floating** for (L+R)/2 — correct here,
since the firmware duplicates the mono sample into both slots. Tying it to GND
shuts the amplifier down.

> **Trade-off: this replaces the microSD slot.** On the Sense expansion board
> D8/D9/D10 are the microSD SPI bus. Wiring the amplifier here gives up the
> card reader permanently. There is no alternative — I2S needs a hardware
> peripheral, so it cannot be moved behind the PCA9685 or the MCP23017.

The I2S channel is disabled between utterances: the MAX98357A hisses faintly
whenever BCLK is running, so leaving it clocking silence is audible.

## Onboard microphone (PDM)

The Sense expansion board carries an MSM261D PDM microphone wired to the
ESP32-S3 directly. **Nothing to wire** — it is on the module — but it is live
hardware the firmware depends on, so it is recorded here. The schematic draws
it dashed to mark it as on-module rather than as a breakout to solder.

| Signal | GPIO | Direction | Function |
|--------|------|-----------|----------|
| PDM CLK | GPIO42 | output | Clock, driven by the ESP32-S3 |
| PDM DATA | GPIO41 | input | Serial PDM data |

16 kHz / 16-bit / mono — fixed by the microphone, and also what Gemini expects
for inline audio, so there is no resampling stage anywhere in the path. Neither
pin collides with the camera DVP group (GPIO10-18, 38-40, 47-48) or the D0-D10
headers (GPIO1-9, 43-44).

It feeds two features: the ambient-audio speech gate ([ADR-020](../../docs/decisions/ADR-020-ambient-audio-speech-gate.md)),
and the push-to-talk `listen` console command. On the ESP32-S3, PDM RX exists
only on I2S0 — the same controller the MAX98357A uses — but the RX channel
**must** be allocated in its own `i2s_new_channel()` call, or the driver goes
full-duplex and clocks the 16 kHz microphone off the 24 kHz amplifier. See the
comment at the allocation site in `main/mic_pdm.c`.

Use the `mic` console command to tell a dead microphone from a quiet room.

## Ultrasonic rangefinders (2x RCWL-1601, I2C)

Two RCWL-1601 modules, one fixed at the front and one at the rear, feed the
reactive controller. The front one drives the obstacle reflex: if one reading
drops below 15 cm the executor brakes, independent of planner goals, and it
releases once the running mean is back above it.
The rear one is reported in telemetry only. Module facts and sources:
[`rcwl-1601.md`](../../../docs/reference/boards/rcwl-1601.md) (pins and mode
pads) and
[`sensor--rcwl-1601.md`](../../../docs/reference/datasheets/sensor--rcwl-1601.md)
(the I2C protocol).

**Both run in I2C mode: fit a 10 kΩ resistor on `R4` (`IIC`) and leave `R5`
(`UART`) empty.** The modules ship in trigger/echo mode with both pads empty.
In I2C mode the header's `Trig` pin is SCL and `Echo` is SDA; the silkscreen
still prints the GPIO-mode names.

| Pin | Front (ch3) | Rear (ch4) |
|-----|-------------|------------|
| VCC | 3.3 V | 3.3 V |
| SCL (`Trig`) | TCA9548A SC3 | TCA9548A SC4 |
| SDA (`Echo`) | TCA9548A SD3 | TCA9548A SD4 |
| GND | any GND | any GND |

- **Each module has a mux channel to itself.** Both answer at 0x57, which
  cannot be changed, and Adafruit's I2C-mode RCWL-1601 (#4742) "doesn't like
  to share the I2C bus".
- **Fit pull-ups on SC3, SD3, SC4 and SD4: nothing else provides them.** The
  modules have none (Adafruit #4742: "there's no built-in pullup resistors"),
  and neither does the TCA9548A breakout on its downstream channels — its board
  file puts resistors on the upstream pair only (see
  [`adafruit-tca9548a.md`](../../../docs/reference/boards/adafruit-tca9548a.md)).
  2.2 kΩ to 3.3 V on each line: at 400 kHz the bus must rise within 300 ns,
  and 10 kΩ manages that only below about 35 pF, which a sensor lead can
  exceed. The carrier fits 2.2 kΩ on every channel (#810).
  Channels 0–2 work without them because those breakouts carry their own.
- **A reading takes ~120 ms.** The firmware triggers, releases the bus, and
  reads three bytes on a later loop tick, so each sensor delivers a reading
  roughly every 150 ms. The wait is not settled by any datasheet; the board
  page lists the bench measurement that would shorten it.
- **Bring-up:** `robocar-bringup`'s `sonar` check reports whether 0x57 answers
  on channels 3 and 4, then five readings from each. A module that does not
  answer is unplugged or still in GPIO mode (`R4` not fitted).

## Flashing

XIAO ESP32-S3 has native USB-Serial-JTAG (VID `0x303a`). Plug in USB-C and flash:

```bash
PORT=/dev/cu.usbmodem* just robocar-unified::flash
```

If the board won't enter download mode: hold **BOOT**, tap **RESET**, release BOOT. See [README.md](README.md) for full flash / monitor commands.
