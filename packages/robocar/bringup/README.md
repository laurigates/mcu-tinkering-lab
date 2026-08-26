# robocar-bringup

Hardware self-test for the XIAO ESP32-S3 Sense robocar build. Power it up, it
exercises every peripheral once and tells you what answered — by ear, by LED, on
the OLED, and over serial.

No WiFi, no API key, no credentials, no cost. 310 kB against `robocar-unified`'s
1.29 MB, so it flashes in a fraction of the time.

## Why it exists

`robocar-unified` is the robot. It wants credentials and a Gemini key, and when
something is miswired the symptom arrives filtered through a planner, a TLS stack
and a 15-second loop. That is the wrong instrument for the half hour where wires
are being soldered one at a time.

This one answers a narrower question — *is the thing I just soldered working?* —
and answers it in about twelve seconds.

## Quick start

```
just robocar-bringup::build
```
```
PORT=/dev/cu.usbmodem101 just robocar-bringup::flash
```
```
just robocar-bringup::monitor
```

Attaching the monitor resets the board, which re-runs the sweep — so the normal
loop is: solder a wire, tap RESET (or re-attach the monitor), listen.

`just robocar-bringup::reset` re-runs it without a monitor attached.

## Reading the result

**SKIP is the normal result on a part-built board.** Hardware that is not fitted
is not a failure. A sweep on a bare board should sound calm and report eleven
SKIPs — if unfitted hardware sounded like a fault, you would learn to ignore the
sweep and it would stop being worth running.

### By ear (works with nothing but power and a piezo)

| Sound | Meaning |
|---|---|
| Two rising notes | Sweep starting |
| One short high blip | PASS |
| Two mid blips | WARN — present, but the reading looks off |
| One short mid-low blip | SKIP — not fitted |
| Two long low notes | FAIL |
| **Three quick high notes** | **The motor check is about to drive the wheels** |
| Rising triad | Finished, nothing failed |
| Falling triad | Finished, something failed |

Pitch carries the verdict and length carries severity, so a fault is
distinguishable without counting beeps across thirteen checks.

### By LED (needs the mux + PCA9685)

Both RGB LEDs show the last result: green PASS, orange WARN, dim blue SKIP, red
FAIL. When the sweep finishes they blink the overall verdict slowly — a blink
rather than a solid colour, because a solid LED and a hung board look identical.

### On the OLED (needs the mux + an SSD1306 on channel 1)

The best untethered indicator on the board: it names *which* check failed rather
than just that one did. Shows a rolling list while the sweep runs, then a summary
with the failing check names.

### Over serial

The full table, with the number behind every verdict — the distance, the dB, the
addresses found, the sample count. `ULTRASONIC PASS` is not actionable;
`ULTRASONIC PASS 5/5 med 34cm` is.

## What it checks

In order. The order is load-bearing — see the header comment in `main/checks.c`.

| Check | What it does |
|---|---|
| `psram` | Octal PSRAM present and sized. Zero here means `CONFIG_SPIRAM_MODE_OCT` is wrong |
| `flash` | Flash size and free internal heap |
| `buzzer` | Two tones. Runs early: it is the channel everything below reports on |
| `i2c-mux` | The PCA9548A/TCA9548A at 0x70, and the PCA9685 behind it |
| `i2c-scan` | **Every mux channel scanned, every address that answers listed.** The most useful line in the sweep while an iron is hot |
| `oled` | SSD1306 on channel 1, initialised and cleared |
| `leds` | Both RGB LEDs through red/green/blue/white |
| `servos` | Pan ±30°, tilt ±20°, then centre and release |
| `motors` | **Drives the wheels.** Four pulses: left fwd, left rev, right fwd, right rev |
| `mcp23017` | Expander on channel 2, pin 0 written and read back |
| `sonar` | Five HC-SR04 readings; reports the count and the median |
| `amp` | A locally generated tone through the MAX98357A — see below |
| `mic` | PDM microphone peak and RMS, and whether any frames arrive at all |

### ⚠ The motor check drives the wheels

Three quick high beeps sound first, then four short pulses at ~27% speed. A robot
with wheels on, sitting near the edge of a bench, will drive off it. Put it on
its back or clear the bench before running the sweep with a motor driver fitted.

### The amplifier tone is a diagnostic, not a jingle

`amp` synthesises 440 Hz, 880 Hz, and a 200 Hz → 2 kHz sweep on-device. That is
the control the TTS path in `robocar-unified` has never had.

A synthesised sine exercises exactly the same I2S channel, DMA descriptors, ring
buffer and amplifier as a spoken sentence — but the samples are generated far
faster than real time, so the ring cannot run dry and the DMA cannot tear. So:

- **Clean here, garbled on speech** → the fault is upstream of I2S, in the fetch
  or the decode.
- **Garbled here too** → the fault is the amplifier, its supply, or the wiring.

The frequency sweep is the informative part: a resonance, a loose ground or a
sagging rail shows up as a band that buzzes while its neighbours stay clean,
which no single fixed tone can reveal.

## Relationship to robocar-unified

The drivers are **shared, not copied**. `main/CMakeLists.txt` compiles the ten
hardware files straight out of `../../unified/main/` and puts that directory on
the include path, so there is exactly one copy of every pin assignment and every
peripheral driver. A wiring fix lands in both firmwares at once.

That works because all ten depend only on `pin_config.h` plus ESP-IDF and
i2cdev — none of them reaches for WiFi, HTTP, cJSON or credentials. Worth
preserving: if a driver over there ever needs a network header, it belongs on the
robot side of the line, not in that list.

New code here is only what `unified` does not have: the sweep itself
(`main/checks.c`), the cue vocabulary and tone generator (`main/cues.c`), and an
SSD1306 driver (`main/oled.c`). The OLED is declared in `unified`'s
`pin_config.h` and `i2c_bus.h` but no driver there has ever driven it — this is
the first code that does.

## Not included (yet)

Considered and deliberately left out of the first version. Each is a small,
self-contained addition to `g_checks[]` if it turns out to be wanted:

- **Camera probe** — log the sensor PID (this board is an **OV3660**, not the
  OV2640 the pin header claimed for years), take one capture, report JPEG size
  and luma mean/p5/p95. Would catch a dark or unseated camera at solder time.
  Costs the `esp32-camera` managed component in the build.
- **WiFi + mDNS** — associate with Improv-provisioned credentials, report RSSI
  and IP, advertise `robocar-bringup.local`. Verifies the antenna and the
  provisioning path, at the cost of dragging the WiFi and TLS stacks into an
  otherwise tiny binary.

## Notes

- The board is wired with a **PCA9548A** where the code says TCA9548A. They are
  register-identical — one control byte, one bit per channel, base address 0x70 —
  and the `tca9548` driver drives either unchanged. The one board-level
  difference worth knowing is that RESET is active-low on both and must be pulled
  high; not every breakout populates that pull-up, and a floating RESET shows up
  as intermittent channel-select failures rather than a clean absence.
- `i2c-scan` skips 0x70 per channel on purpose. The mux sits on the primary bus
  so it ACKs on every channel, and the PCA9685's ALLCALL address is 0x70 by
  default too — reporting it would be eight lines of ambiguous noise hiding the
  one device that matters.
- The flash recipe is inline rather than composed from the shared `_s3-flash`,
  which hardcodes `--flash-size 4MB`. This board has 8 MB. Verify any change to
  it with `just --dry-run robocar-bringup::flash` — CI builds firmware but never
  flashes it, so nothing else exercises that recipe.
