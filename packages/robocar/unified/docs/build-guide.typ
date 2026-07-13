// robocar-unified — Printable Build Guide
// Build with:  typst compile build-guide.typ
// Pin data mirrors main/pin_config.h (authoritative). Keep in sync on pin changes.

#let version = "0.1.2"
#let accent = rgb("#1f6feb")
#let accent-soft = rgb("#e8f0fe")
#let ink = rgb("#1a1a1a")
#let muted = rgb("#6a737d")
#let rule = rgb("#d0d7de")

#set document(title: "robocar-unified Build Guide", author: "MCU Tinkering Lab")
#set page(
  paper: "a4",
  margin: (top: 2.2cm, bottom: 2cm, x: 1.9cm),
  header: context {
    if counter(page).get().first() > 1 [
      #set text(9pt, fill: muted)
      #grid(columns: (1fr, 1fr),
        align(left)[robocar-unified — Build Guide],
        align(right)[XIAO ESP32-S3 Sense · v#version],
      )
      #line(length: 100%, stroke: 0.5pt + rule)
    ]
  },
  footer: context {
    set text(9pt, fill: muted)
    grid(columns: (1fr, 1fr),
      align(left)[MCU Tinkering Lab],
      align(right)[#counter(page).display("1 / 1", both: true)],
    )
  },
)

#set text(font: ("Libertinus Serif", "DejaVu Serif"), size: 10.5pt, fill: ink)
#set par(justify: true, leading: 0.62em)
#show heading: set text(fill: ink)
#show heading.where(level: 1): it => {
  v(0.4em)
  block(text(16pt, weight: "bold", fill: accent)[#it.body])
  v(0.15em)
  line(length: 100%, stroke: 1pt + accent)
  v(0.35em)
}
#show heading.where(level: 2): it => {
  v(0.35em)
  block(text(12.5pt, weight: "bold")[#it.body])
  v(0.1em)
}
#show heading.where(level: 3): it => block(text(11pt, weight: "bold", fill: accent.darken(10%))[#it.body])

#set table(stroke: none)
#show table.cell.where(y: 0): set text(weight: "bold", fill: white)

// --- helper for a soft callout box ---
#let callout(title, body, fill: accent-soft, bar: accent) = block(
  width: 100%,
  fill: fill,
  inset: (x: 10pt, y: 8pt),
  radius: 4pt,
  stroke: (left: 3pt + bar),
)[
  #text(weight: "bold", fill: bar.darken(15%))[#title] \
  #body
]

// --- helper for a header-styled table ---
#let htable(cols, header, ..rows, aligns: none) = {
  let n = header.len()
  table(
    columns: cols,
    align: if aligns == none { left } else { aligns },
    fill: (_, y) => if y == 0 { accent } else if calc.odd(y) { rgb("#f6f8fa") } else { white },
    inset: (x: 7pt, y: 5pt),
    table.header(..header),
    ..rows.pos().flatten(),
  )
}

// ============================================================
// TITLE PAGE
// ============================================================
#v(3cm)
#align(center)[
  #text(11pt, fill: muted, tracking: 3pt)[MCU TINKERING LAB]
  #v(0.6cm)
  #text(30pt, weight: "bold", fill: accent)[robocar-unified]
  #v(0.15cm)
  #text(16pt, weight: "bold")[Single-Board AI Robot Car]
  #v(0.3cm)
  #text(12pt, fill: muted)[Assembly, Wiring & Firmware Build Guide]
  #v(1.2cm)
  #box(width: 80%)[
    #set text(10.5pt)
    #set par(justify: false)
    A hands-on guide to assembling the consolidated robocar on a
    *Seeed Studio XIAO ESP32-S3 Sense*. Camera capture, Gemini AI planning,
    motor control, and peripherals all run on one module.
  ]
  #v(1cm)
  #box(fill: accent-soft, inset: 10pt, radius: 5pt)[
    #grid(columns: (auto, auto), column-gutter: 1.2cm, row-gutter: 5pt,
      align(right)[#text(fill: muted)[Firmware version]], [*v#version*],
      align(right)[#text(fill: muted)[Target MCU]], [ESP32-S3 (8 MB PSRAM / flash)],
      align(right)[#text(fill: muted)[Toolchain]], [ESP-IDF v5.4 (containerized)],
      align(right)[#text(fill: muted)[Difficulty]], [Intermediate · \~2–3 h],
    )
  ]
]
#v(1fr)
#align(center)[
  #text(9pt, fill: muted)[
    Pin assignments in this guide mirror `main/pin_config.h`, which is authoritative. \
    All components must share a common ground.
  ]
]
#pagebreak()

// ============================================================
#outline(title: [Contents], indent: 1em, depth: 2)
#pagebreak()

// ============================================================
= 1 · Overview

The *robocar-unified* project consolidates the original dual-ESP32 robocar
(a Heltec main controller plus an ESP32-CAM) onto a single *XIAO ESP32-S3 Sense*.
One module now handles camera capture, AI inference, motor control, and all
peripherals, using dual-core affinity to keep motor timing isolated from
bursty network and vision work.

The control system is *hierarchical*: a slow *planner* (\~1 Hz, Core 1) calls
Google's Gemini Robotics-ER to emit structured goals, and a fast *reactive
executor* (\~30 Hz, Core 0) drives the robot smoothly toward those goals while
an ultrasonic sensor provides an independent obstacle reflex.

#grid(columns: (1fr, 1fr), column-gutter: 12pt,
  callout("Core 0 — real-time")[
    Reactive executor (visual servo, heading hold, motor PWM), peripheral I/O,
    command console, and the ultrasonic obstacle reflex.
  ],
  callout("Core 1 — bursty I/O", bar: rgb("#8250df"), fill: rgb("#f3eefc"))[
    Planner (Gemini calls), OV2640 camera DMA, WiFi / MQTT / OTA.
  ],
)

#v(0.3em)
#callout("What you get", bar: rgb("#1a7f37"), fill: rgb("#eafbef"))[
  A two-wheel-drive car that captures frames, asks Gemini what to do, and
  drives toward goals — with pan/tilt camera, status LEDs, an OLED display,
  buzzer feedback, WiFi provisioning over Bluetooth, and over-the-air updates.
]

= 2 · Bill of Materials

#htable(
  (auto, 1fr, auto),
  ([*Qty*], [*Component*], [*Notes*]),
  ([1], [XIAO ESP32-S3 Sense], [MCU + OV2640 camera + 8 MB PSRAM, USB-C]),
  ([1], [TCA9548A I²C multiplexer], [Breakout, address 0x70]),
  ([1], [PCA9685 16-ch PWM driver], [Breakout, address 0x40]),
  ([1], [TB6612FNG dual motor driver], [Breakout]),
  ([1], [SSD1306 OLED display], [128×64, I²C, address 0x3C]),
  ([1], [Ultrasonic rangefinder], [*3.3 V variant*: HC-SR04P / RCWL-1601 / US-100]),
  ([2], [RGB LED], [Common-anode]),
  ([2], [SG90 micro servo], [Pan / tilt]),
  ([2], [DC gear motor + wheel], [\~3–6 V hobby motors]),
  ([1], [Piezo buzzer], [Passive]),
  ([1], [100 Ω resistor], [In series with buzzer]),
  ([2], [18650 Li-ion cell + holder], [Battery pack]),
  ([1], [XL6009 boost converter], [Regulated to 5 V]),
  ([—], [Chassis, wiring, headers], [2WD car chassis, jumper wires, standoffs]),
  aligns: (center, left, left),
)

== Tools required
Soldering iron + solder, wire strippers, small screwdriver set, multimeter
(for verifying 5 V rail and continuity), a USB-C cable, and a computer with
Docker (for the containerized firmware build).

#callout("Sensor voltage — read this", bar: rgb("#bf8700"), fill: rgb("#fff8e1"))[
  The ultrasonic sensor *must be a 3.3 V-compatible module* (HC-SR04P, not the
  classic 5 V HC-SR04). The XIAO's GPIOs are not 5 V-tolerant — a 5 V ECHO line
  can damage the board.
]

= 3 · System Architecture

#figure(
  image("../../../../docs/schematics/images/robocar_unified.png", width: 100%),
  caption: [Full wiring schematic. Source: `docs/schematics/circuits/robocar_unified.py`.],
)

Everything on the I²C bus hangs off the *TCA9548A multiplexer* — the ESP32-S3
never talks to the PCA9685 or OLED directly. This lets devices that would
otherwise share addresses coexist, and keeps the two 400 kHz devices on
separate channels. The ESP32-S3 itself drives only four things directly:
I²C (GPIO5/6), the motor-enable STBY line (GPIO1), the buzzer (GPIO2), and the
ultrasonic sensor (GPIO3/4).

= 4 · Wiring Reference

== 4.1 · XIAO ESP32-S3 GPIO assignments
Only 11 GPIOs are exposed on the XIAO headers. Camera pins are internal to the
Sense module and do not conflict.

#htable(
  (auto, auto, 1fr, 1.2fr),
  ([*XIAO Pin*], [*GPIO*], [*Function*], [*Notes*]),
  ([D0], [GPIO1], [TB6612FNG STBY], [HIGH = motors enabled]),
  ([D1], [GPIO2], [Piezo buzzer], [LEDC PWM · 100 Ω in series]),
  ([D2], [GPIO3], [Ultrasonic TRIG], [10 µs pulse output]),
  ([D3], [GPIO4], [Ultrasonic ECHO], [Pulse width in (RMT RX)]),
  ([D4], [GPIO5], [*I²C SDA*], [to TCA9548A]),
  ([D5], [GPIO6], [*I²C SCL*], [to TCA9548A]),
  ([D6], [GPIO43], [USB Serial TX], [Debug console]),
  ([D7], [GPIO44], [USB Serial RX], [Debug console]),
  ([D8–D10], [GPIO7–9], [_spare_], [Future SPI / expansion]),
  aligns: (center, center, left, left),
)
I²C runs at *400 kHz*.

== 4.2 · I²C topology (TCA9548A @ 0x70)
Select the channel on the multiplexer *before* addressing any downstream device.

#htable(
  (auto, 1fr, auto),
  ([*Channel*], [*Device*], [*Address*]),
  ([ch0], [PCA9685 PWM driver (motors, servos, LEDs)], [0x40 @ 200 Hz]),
  ([ch1], [SSD1306 OLED display (128×64)], [0x3C]),
  ([ch2–7], [_reserved — IMU / ToF / future sensors_], [—]),
  aligns: (center, left, center),
)

== 4.3 · PCA9685 channel map (0x40, 200 Hz)
All motor direction, motor PWM, servo, and LED outputs go through the PCA9685.
Motor direction pins use PCA9685 "full-on" (4096) / "full-off" (0); PWM pins
use the full 12-bit range (0–4095).

#grid(columns: (1fr, 1fr), column-gutter: 12pt,
  htable(
    (auto, 1fr),
    ([*Ch*], [*Signal*]),
    ([0], [Left LED — R]),
    ([1], [Left LED — G]),
    ([2], [Left LED — B]),
    ([3], [Right LED — R]),
    ([4], [Right LED — G]),
    ([5], [Right LED — B]),
    ([6], [Pan servo (SG90)]),
    ([7], [Tilt servo (SG90)]),
    aligns: (center, left),
  ),
  htable(
    (auto, 1fr),
    ([*Ch*], [*Signal*]),
    ([8], [Motor R — IN1 (dir)]),
    ([9], [Motor R — IN2 (dir)]),
    ([10], [Motor R — PWM]),
    ([11], [Motor L — IN1 (dir)]),
    ([12], [Motor L — IN2 (dir)]),
    ([13], [Motor L — PWM]),
    ([14–15], [_reserved_]),
    ([], []),
    aligns: (center, left),
  ),
)
#text(fill: muted)[200 Hz is a compromise between servo timing (ideal 50 Hz)
and motor PWM smoothness — it works well for SG90s and the TB6612FNG.]

== 4.4 · Ultrasonic rangefinder
#htable(
  (auto, auto, auto, 1fr),
  ([*Signal*], [*Pin*], [*Voltage*], [*Function*]),
  ([TRIG], [GPIO3 (D2)], [3.3 V], [10 µs pulse triggers a measurement]),
  ([ECHO], [GPIO4 (D3)], [3.3 V], [Pulse width encodes distance (RMT RX)]),
  ([VCC], [3.3 V], [3.3 V], [*3.3 V variant only*]),
  ([GND], [any GND], [—], [Shared ground]),
  aligns: (center, center, center, left),
)
The sensor samples at \~20 Hz. *Obstacle reflex:* if distance < 15 cm, the
executor immediately stops and reverses, independent of planner goals.

= 5 · Power

#grid(columns: (1.15fr, 1fr), column-gutter: 14pt,
[
  Power the car from a *2×18650 pack* through an *XL6009 boost converter set to
  5 V*. Distribute that 5 V rail to the XIAO 5 V pin, the TB6612FNG (VM + VCC),
  the PCA9685 (V+ and VCC), and the servos.

  The 3.3 V logic for the OLED, ultrasonic sensor, and TCA9548A comes from the
  XIAO's 3V3 pin. Keep motor/servo current (high, noisy) on the 5 V rail and
  logic on 3V3.
],
callout("Golden rule", bar: rgb("#cf222e"), fill: rgb("#ffebe9"))[
  *Common ground everywhere.* Every module — boost converter, XIAO, motor
  driver, PCA9685, servos, sensors — must share one GND. Missing grounds cause
  brown-outs, I²C lockups, and erratic motion.

  Brown-out detection is disabled in firmware because motor inrush was tripping
  it; a stiff 5 V supply and thick power wires matter.
],
)

= 6 · Assembly Steps

+ *Mount the mechanics.* Fit the two gear motors and wheels to the chassis, add
  the caster/third wheel, and mount the battery holder low and centered.
+ *Wire power first.* Connect the 18650 pack to the XL6009 input, set its output
  to *5.0 V with a multimeter before connecting anything else*, then run the 5 V
  and shared GND rails.
+ *Place the XIAO* and bring out I²C (GPIO5/6), STBY (GPIO1), buzzer (GPIO2),
  and the ultrasonic pins (GPIO3/4).
+ *Wire the I²C chain:* XIAO SDA/SCL → TCA9548A → PCA9685 (ch0) and OLED (ch1).
  Pull-ups on the breakouts are usually sufficient.
+ *Wire the motor driver:* PCA9685 ch8–13 → TB6612FNG inputs; STBY → GPIO1;
  motor outputs → the two DC motors; VM/VCC → 5 V.
+ *Add servos* (PCA9685 ch6/7) and *RGB LEDs* (ch0–5, common-anode).
+ *Add the buzzer* on GPIO2 through the 100 Ω resistor, and the ultrasonic
  sensor on GPIO3/4 (3.3 V power).
+ *Double-check the 3.3 V vs 5 V rails* and confirm common ground with a
  multimeter continuity test before first power-up.

= 7 · Build & Flash the Firmware

Builds run inside a container — *no local ESP-IDF install is required*. The
XIAO uses native USB-Serial-JTAG (USB VID `0x303a`), which `just` auto-detects.

```bash
# From the repo root
just robocar-unified::build                       # containerized ESP-IDF v5.4 build
PORT=/dev/cu.usbmodem* just robocar-unified::flash # flash over USB-C
just robocar-unified::monitor                      # serial console
# or, in one step:
just robocar-unified::flash-monitor
```

#callout("Can't enter download mode?", bar: rgb("#bf8700"), fill: rgb("#fff8e1"))[
  Hold *BOOT*, tap *RESET*, then release BOOT to force the bootloader, and
  re-run the flash command.
]

The flasher writes three images to an 8 MB, OTA-capable layout:

#htable(
  (auto, 1fr, auto),
  ([*Offset*], [*Image*], [*Partition*]),
  ([`0x0`], [`build/bootloader/bootloader.bin`], [bootloader]),
  ([`0x8000`], [`build/partition_table/partition-table.bin`], [partition table]),
  ([`0x12000`], [`build/robocar-unified.bin`], [ota_0 (app)]),
  aligns: (left, left, left),
)

= 8 · First Boot & Provisioning

== 8.1 · WiFi over Bluetooth (Improv)
No WiFi credentials are compiled in. On first boot the device advertises an
*Improv WiFi BLE service*. Use a browser-based Improv provisioner (Chrome on
desktop or Android) to send your SSID and password; they are stored in NVS and
reused on later boots.

For local development you can instead copy `main/credentials.h.example` to
`main/credentials.h` (gitignored) and hard-code credentials.

== 8.2 · Discovery & AI backend
After connecting, the car is reachable at *`robocar-unified.local`* via mDNS.
The planner uses *Gemini Robotics-ER 1.6* to emit goals — `drive()`, `track()`,
`rotate()`, and `stop()`.

== 8.3 · Over-the-air updates
OTA is enabled with app rollback. The updater pulls releases from the
`laurigates/mcu-tinkering-lab` GitHub repo; `version.txt` (currently *v#version*)
is the single source of truth and is managed by release-please.

= 9 · Functional Checkout

Work through these after first flash, watching the serial monitor:

#htable(
  (auto, 1.2fr, 1.3fr),
  ([*#sym.checkmark*], [*Check*], [*Expected result*]),
  ([☐], [Boot log], [No PSRAM / boot-loop errors; tasks pin to cores 0 & 1]),
  ([☐], [I²C scan], [TCA9548A, PCA9685, and OLED all detected]),
  ([☐], [OLED], [Status screen renders (128×64)]),
  ([☐], [LEDs], [Both RGB LEDs cycle / show status colors]),
  ([☐], [Buzzer], [Audible startup tone]),
  ([☐], [Servos], [Pan/tilt center, then sweep within limits]),
  ([☐], [Motors], [Both wheels drive forward and reverse; STBY HIGH]),
  ([☐], [Ultrasonic], [Distance readings track a hand moving closer/away]),
  ([☐], [Reflex], [Car stops/reverses when an obstacle is < 15 cm]),
  ([☐], [WiFi], [Provisions via Improv; `robocar-unified.local` resolves]),
  aligns: (center, left, left),
)

= 10 · Troubleshooting

#htable(
  (1fr, 1.4fr),
  ([*Symptom*], [*Likely cause & fix*]),
  ([Boot loop on power-up], [Wrong PSRAM mode. The Sense uses *octal* PSRAM (`CONFIG_SPIRAM_MODE_OCT=y`); don't change it.]),
  ([Random resets under motor load], [Weak 5 V rail / missing common ground. Use thicker power wires and verify the XL6009 holds 5 V under load.]),
  ([No I²C devices found], [Not selecting the TCA9548A channel first, or SDA/SCL swapped. Check GPIO5=SDA, GPIO6=SCL.]),
  ([OLED and PCA9685 conflict], [Both bypassing the mux. Route each through its own TCA9548A channel (ch1 / ch0).]),
  ([Motors don't move], [STBY (GPIO1) not HIGH, or VM not on 5 V. Confirm TB6612FNG power and enable line.]),
  ([Servos jitter], [Shared noisy rail. Keep servo power on 5 V with common ground; 200 Hz PWM is expected.]),
  ([Board won't flash], [Force download mode: hold BOOT, tap RESET, release BOOT.]),
  ([Damaged ECHO / no distance], [Used a 5 V HC-SR04. Replace with a 3.3 V module (HC-SR04P).]),
  aligns: (left, left),
)

#v(1fr)
#line(length: 100%, stroke: 0.5pt + rule)
#align(center)[
  #text(9pt, fill: muted)[
    Authoritative pin data: `packages/robocar/unified/main/pin_config.h` ·
    Design rationale: `docs/decisions/ADR-016-hierarchical-ai-controller.md` \
    Regenerate this PDF with `typst compile build-guide.typ`.
  ]
]
