# Servo Selection: Frame Rate, Analog vs Digital, and Size

What was learned in 2026-09 while diagnosing the robocar-unified pan/tilt head,
whose SG90 servos did not respond at the firmware's shipped 200 Hz. Per-part
specs live in the datasheet notes
([SG90](datasheets/actuator--sg90.md), [MG996R](datasheets/actuator--mg996r.md));
this page covers the choices those specs feed.

Every claim below is labelled with where it came from. Three kinds of source
appear, and they are not equally trustworthy:

| Label | Meaning |
|-------|---------|
| **datasheet** | Read from the manufacturer or distributor PDF, including its drawings |
| **vendor page** | Stated on a manufacturer product page (TowerPro) |
| **uncited** | From an article that cites no sources or measurements; plausible, not proven |

## Frame rate: what the pulse repeat rate means

A hobby servo reads **pulse width** (about 1–2 ms) to decide its position. The
**frame rate** is how often that pulse repeats. At 50 Hz a frame is 20 ms; at
200 Hz it is 5 ms, which still has room for a 2.5 ms pulse. So a faster frame
rate does not change the pulse width; it only changes how often the servo
receives one.

- The SG90 is specified for a **20 ms (50 Hz) PWM period**, 1–2 ms nominal
  pulse, 500–2400 µs range. (datasheet: both SG90 PDFs, timing diagram)
- The MG996R datasheet does **not** state a frame rate. (datasheet)
- TowerPro's own pages for the SG90 Analog, SG90 Digital, MG90S and MG90D state
  **no frame rate** for any of them. (vendor page)

## Analog vs digital

- **Analog** servos compare the incoming pulse against the position sensor once
  per frame. The article claims the SG90's analog comparator "may not reset
  cleanly between pulses" above about 60 Hz, which causes oscillation or
  twitching, and that driving an analog servo at 333 Hz heats it and can damage
  it. (uncited)
- **Digital** servos sample the pulse with a microcontroller and run the motor
  at their own internal rate, so they generally accept faster frame rates. The
  article gives the MG996R as rated to 333 Hz and says its idle current rises
  from ~10 mA at 50 Hz to ~25–35 mA at 333 Hz. (uncited)

### The model name does not tell you which one you have

- TowerPro sells the SG90 as **both** an Analog and a Digital variant, with
  identical 23 × 12.2 × 29 mm dimensions. (vendor page)
- The Handsontec SG90 guide lists the part as "Modulation: Analogue". (datasheet)
- TowerPro describes its current MG90S as digital; many resellers describe the
  MG90S as analog. (vendor page vs reseller listings)
- Clones copy the printed label, so the label identifies neither the maker nor
  the modulation.

The dependable test is to drive the servo at the frame rate you intend to use
and watch whether it tracks cleanly.

## Size and mounting compatibility

Pan/tilt brackets and servo mounts are made for a **size class**. The screws go
through the servo's **mounting tabs** (also called mounting ears or flanges), so
both the case size and the distance across the tabs have to match.

| Servo | Class | Case (mm) | Across tabs (mm) | Weight | Source |
|-------|-------|-----------|------------------|--------|--------|
| SG90 Analog | micro (9 g) | 23 × 12.2 × 29 | 32.3 | 9 g | vendor page |
| SG90 Digital | micro (9 g) | 23 × 12.2 × 29 | 32.3 | 9 g | vendor page |
| MG90S | micro | 22.8 × 12.2 × 28.5 | — | 13.4 g | vendor page |
| MG90D | micro | 22.8 × 12.2 × 28.5 | 31.5 | 13 g | vendor page |
| MG996R | **standard** | 40.7 × 19.7 × 42.9 | 53.6 | 55 g | datasheet |

"Across tabs" is TowerPro's dimension E for the micro servos and the 53.6 mm
figure on the MG996R drawing. TowerPro's MG90S page gave no lettered dimensions.

**The MG996R is not a drop-in replacement for the SG90.** Its case is about
1.75× longer and 1.6× wider, the tab span is 53.6 mm against 32.3 mm, and it
weighs about six times as much. It needs a standard-size bracket. Its 2.5 A
stall current at 6 V (datasheet) also calls for a much heavier servo supply
than a micro servo does.

### The robocar pan/tilt bracket

The bracket in use is AliExpress item
[1005006539573065](https://www.aliexpress.com/item/1005006539573065.html),
listed as "Mg90s SG90 9g Steering Gear Pan Tilt Two Axis PTZ … sg90 bracket
set". It is a 9 g micro-class bracket. It takes the SG90 (either variant), the
MG90S and, going by the published case dimensions, the MG90D. It does not take
an MG996R.

## What this means for robocar-unified

- The PCA9685 has **one frequency for all 16 outputs** (`PCA9685_FREQ_HZ` in
  `packages/robocar/unified/main/pin_config.h`, shipped at 200 Hz), so the
  servos share it with the motor PWM and the RGB LEDs. See
  [`driver--pca9685.md`](datasheets/driver--pca9685.md).
- **Test before changing anything.** On the console, run `servo freq 50`, then
  `servo exercise`. If the head moves, frame rate was the cause. If every step
  logs `ESP_OK` and nothing moves, check servo power (the PCA9685 V+ terminal;
  VCC powers only its logic) and the servo leads. The frequency change is not
  persisted, so a reboot restores 200 Hz.
- If frame rate is confirmed, there are two ways forward:

  | Option | Keeps | Costs |
  |--------|-------|-------|
  | Run the whole PCA9685 at 50 Hz | The fitted SG90s | Possible visible LED flicker; coarser motor PWM |
  | Fit digital micro servos | 200 Hz for motors and LEDs | Buying servos, and confirming the chosen model actually tracks at 200 Hz, since no vendor page above publishes a rating |

- The firmware maps pan ±90° onto 500–2500 µs (`SERVO_MIN_PULSE_US` /
  `SERVO_MAX_PULSE_US`), so pan +90° exceeds the SG90's 2400 µs maximum.

## Sources

- SG90 datasheet (Handsontec): <https://handsontec.com/dataspecs/motor_fan/SG90-Servo.pdf>
- SG90 datasheet (Imperial College mirror): <http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf>
- MG996R datasheet (Handsontec): <https://www.handsontec.com/dataspecs/motor_fan/MG996R.pdf>
- TowerPro SG90 Analog: <https://towerpro.com.tw/product/sg90-analog/>
- TowerPro SG90 Digital: <https://towerpro.com.tw/product/sg90-7/>
- TowerPro MG90S: <https://towerpro.com.tw/product/mg90s-3/>
- TowerPro MG90D: <https://towerpro.com.tw/product/mg90d-2/>
- Uncited article: <https://zbotic.in/servo-pwm-frequency-50hz-vs-333hz-vs-1khz-trade-offs-explained/>
