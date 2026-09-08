// Bench card: PCA9685 → TB6612FNG motor wiring for robocar-unified.
//
// Scope is deliberately one job — the wires between the PWM driver and the
// motor driver — so the sheet can be printed and kept next to the iron while
// the rest of the build guide stays on the shelf.
//
// Channel numbers and the STBY GPIO are imported from docs/auto/pin_defs.typ,
// which is generated from main/pin_config.h. Never retype a channel number
// here: the drift guard regenerates that file and diffs this PDF, so a channel
// moved in the C header fails CI instead of leaving a wrong number on a printed
// card. Board layouts are transcribed from the vendors' published Eagle files
// (see the Sources section at the end), not from memory or a product photo.
//
// Regenerate:  just robocar-unified::build-guide

#import "../../../../tools/typst/build-guide.typ": card, callout, htable, theme
#import "auto/pin_defs.typ" as pins

#show: card.with(
  title: "Motor wiring card",
  subtitle: "PCA9685 → TB6612FNG · robocar-unified",
  header-right: "XIAO ESP32-S3 Sense",
  footer-note: [Channels from `main/pin_config.h` · layouts from vendor board files],
)

#let used = (
  pins.PCA_CH_MOTOR_R_IN1, pins.PCA_CH_MOTOR_R_IN2, pins.PCA_CH_MOTOR_R_PWM,
  pins.PCA_CH_MOTOR_L_IN1, pins.PCA_CH_MOTOR_L_IN2, pins.PCA_CH_MOTOR_L_PWM,
)

#callout("Six signal wires, plus one that does not come from the PCA9685", [
  Channels #used.at(0)–#used.at(5) carry direction and speed for both motors.
  *STBY* is the seventh wire and runs straight from the XIAO's
  *D0 / GPIO#pins.MOTOR_STBY_PIN* — it is not a PCA9685 channel. With STBY low
  the H-bridges stay disabled and the motors do nothing, however correct the
  other six wires are.
], kind: "info")

= 1 · Board layouts, viewed from the component side

== PCA9685 16-channel PWM driver — Adafruit \#815 and its clones

#let hdr6 = table(
  columns: 1, stroke: 0.4pt + theme.rule, inset: (x: 6pt, y: 3.1pt), align: center,
  ..("GND", "OE", "SCL", "SDA", "VCC", "V+").map(p => text(8pt, weight: "bold")[#p]),
)

// Roles are keyed off the generated channel numbers, so a motor moved to a
// different channel in pin_config.h relabels the strip instead of mislabelling it.
#let role = (
  str(pins.PCA_CH_MOTOR_R_IN1): "R IN1",
  str(pins.PCA_CH_MOTOR_R_IN2): "R IN2",
  str(pins.PCA_CH_MOTOR_R_PWM): "R PWM",
  str(pins.PCA_CH_MOTOR_L_IN1): "L IN1",
  str(pins.PCA_CH_MOTOR_L_IN2): "L IN2",
  str(pins.PCA_CH_MOTOR_L_PWM): "L PWM",
)

#let chcell(n, row) = {
  let hot = used.contains(n) and row == "PWM"
  box(
    width: 100%, inset: (x: 0pt, y: 2.6pt), radius: 1pt,
    fill: if hot { theme.accent-soft } else { white },
    stroke: 0.4pt + theme.rule,
    align(center, text(
      if hot { 5.8pt } else { 6.5pt },
      fill: if hot { theme.accent } else { theme.muted },
      weight: if hot { "bold" } else { "regular" },
    )[#if hot { role.at(str(n)) } else { row }]),
  )
}

#let chblock(nums) = table(
  columns: (1fr,) * 4, stroke: none, inset: 0.4pt, align: center,
  ..nums.map(n => text(7.5pt, weight: if used.contains(n) { "bold" } else { "regular" },
      fill: if used.contains(n) { theme.accent } else { theme.muted })[#n]),
  ..nums.map(n => chcell(n, "PWM")),
  ..nums.map(n => chcell(n, "V+")),
  ..nums.map(n => chcell(n, "GND")),
)

#block(width: 100%, stroke: 0.8pt + theme.ink, radius: 3pt, inset: 7pt)[
  #align(center)[
    #box(stroke: 0.7pt + theme.muted, radius: 2pt, inset: (x: 7pt, y: 3pt))[
      #text(8pt, weight: "bold")[V+] #h(10pt) #text(8pt, weight: "bold")[GND]
      #h(6pt) #text(7pt, fill: theme.muted)[screw terminal — servo/motor rail in]
    ]
  ]
  #v(4pt)
  #grid(columns: (auto, 1fr, auto), column-gutter: 7pt, align: horizon,
    hdr6,
    align(center)[
      #text(10pt, weight: "bold")[PCA9685] \
      #text(7.5pt, fill: theme.muted)[16 × 12-bit PWM · addr #pins.PCA9685_ADDR · #pins.PCA9685_FREQ_HZ Hz] \
      #v(2pt)
      #text(7pt, fill: theme.muted)[I²C address jumpers A0–A5 along this top edge]
    ],
    hdr6,
  )
  #v(5pt)
  #grid(columns: (1fr, 1fr, 1fr, 1fr), column-gutter: 7pt,
    chblock((0, 1, 2, 3)), chblock((4, 5, 6, 7)),
    chblock((8, 9, 10, 11)), chblock((12, 13, 14, 15)),
  )
  #v(3pt)
  #align(center)[#text(7pt, fill: theme.muted)[
    Rows, board edge inwards: *GND* (outermost) · *V+* · *PWM* (nearest the chip).
    Channels run 0 on the left to 15 on the right, in four blocks of four.
    Only the *PWM* row of the marked columns is wired to the motor driver.
  ]]
]

#callout("Both side headers carry the same six pins", [
  The 6-pin headers on the left and right edges are identical — GND, OE, SCL,
  SDA, VCC, V+ top to bottom — so either one can take the I²C feed and the other
  chains onward. Here the I²C side comes from *TCA9548A channel #pins.TCA_CH_PCA9685*,
  not from the XIAO directly.
  The board's own underside warns: *the screw terminal is reverse-polarity
  protected, the side breakout pins are not* — the protection MOSFET sits behind
  the terminal only, so V+ fed backwards through a side header kills the board.
  *OE* needs nothing: a 10 kΩ pulldown on the board already enables the outputs.
], kind: "warn")

== TB6612FNG dual motor driver — SparkFun ROB-14451 and its clones

// Pins are tinted by what feeds them, so the diagram carries the wiring rather
// than just the silkscreen: signal from the PCA9685, enable from the MCU,
// rails, motor outputs.
#let tbfill = (
  sig: (theme.accent-soft, theme.accent),
  stby: (theme.warn-soft, theme.warn),
  pwr: (theme.zebra, theme.ink),
  out: (theme.ok-soft, theme.ok),
  spare: (white, theme.muted),
)

#let tbpin(label, kind) = {
  let (bg, fg) = tbfill.at(kind)
  box(width: 100%, inset: (x: 6pt, y: 3.2pt), radius: 1pt, fill: bg,
    stroke: 0.4pt + theme.rule,
    align(center, text(8pt, weight: "bold", fill: fg)[#label]))
}

#let tbrow(left, lk, right, rk) = (tbpin(left, lk), [], tbpin(right, rk))

#block(width: 100%, stroke: 0.8pt + theme.ink, radius: 3pt, inset: 8pt)[
  #align(center)[#text(7.5pt, fill: theme.muted)[
    top of board — SparkFun flame logo at this end
  ]]
  #v(3pt)
  #grid(columns: (3.4cm, 1fr, 3.4cm), column-gutter: 8pt, row-gutter: 2.5pt, align: horizon,
    ..tbrow("VM", "pwr", "PWMA", "sig"),
    ..tbrow("VCC", "pwr", "AIN2", "sig"),
    ..tbrow("GND", "pwr", "AIN1", "sig"),
    ..tbrow("A01", "out", "STBY", "stby"),
    ..tbrow("A02", "out", "BIN1", "sig"),
    ..tbrow("B02", "out", "BIN2", "sig"),
    ..tbrow("B01", "out", "PWMB", "sig"),
    ..tbrow("GND", "spare", "GND", "spare"),
  )
  #place(center + horizon, dy: 0pt)[
    #box(fill: white, inset: (x: 6pt, y: 4pt))[
      #align(center)[
        #text(10pt, weight: "bold")[TB6612FNG] \
        #text(7.5pt, fill: theme.muted)[dual H-bridge · 1.2 A avg / 3.2 A peak]
      ]
    ]
  ]
  #v(3pt)
  #align(center)[
    #text(7pt)[
      #box(fill: theme.accent-soft, inset: (x: 3pt, y: 1.5pt), radius: 1pt)[from PCA9685]
      #h(5pt) #box(fill: theme.warn-soft, inset: (x: 3pt, y: 1.5pt), radius: 1pt)[from XIAO D0]
      #h(5pt) #box(fill: theme.zebra, inset: (x: 3pt, y: 1.5pt), radius: 1pt)[rails]
      #h(5pt) #box(fill: theme.ok-soft, inset: (x: 3pt, y: 1.5pt), radius: 1pt)[to motors]
      #h(5pt) #box(stroke: 0.4pt + theme.rule, inset: (x: 3pt, y: 1.5pt), radius: 1pt)[unused]
    ]
    #v(2pt)
    #text(7pt, fill: theme.muted)[
      Both 8-pin rows start at the same end: *VM* and *PWMA* are the top corners,
      both rows end in *GND*. Outputs are on the VM side, control on the PWMA side.
      Pins are labelled on both faces; the second GND is redundant on-board.
    ]
  ]
]

#callout("A01 / A02 / B01 / B02 are printed with a zero, not a letter O", [
  Channel *A drives the right motor* and *B the left*, matching
  `docs/schematics/circuits/robocar_unified.py` and the `MOTOR_RIGHT_*` /
  `MOTOR_LEFT_*` names in `main/pin_config.h`. Nothing in the firmware can tell
  which motor is which — that is set entirely by which output pair you solder to
  which wheel.
], kind: "info")

= 2 · The seven wires

#htable(
  (auto, auto, auto, 1fr),
  ([], [From], [To], [Carries]),
  ([☐], [PCA9685 ch #pins.PCA_CH_MOTOR_R_IN1 · PWM row], [TB6612 *AIN1*], [Right motor direction bit 1 — driven full-on (4096) or full-off (0), never a duty cycle]),
  ([☐], [PCA9685 ch #pins.PCA_CH_MOTOR_R_IN2 · PWM row], [TB6612 *AIN2*], [Right motor direction bit 2]),
  ([☐], [PCA9685 ch #pins.PCA_CH_MOTOR_R_PWM · PWM row], [TB6612 *PWMA*], [Right motor speed — the only modulated line of the three]),
  ([☐], [PCA9685 ch #pins.PCA_CH_MOTOR_L_IN1 · PWM row], [TB6612 *BIN1*], [Left motor direction bit 1]),
  ([☐], [PCA9685 ch #pins.PCA_CH_MOTOR_L_IN2 · PWM row], [TB6612 *BIN2*], [Left motor direction bit 2]),
  ([☐], [PCA9685 ch #pins.PCA_CH_MOTOR_L_PWM · PWM row], [TB6612 *PWMB*], [Left motor speed]),
  ([☐], [XIAO *D0 / GPIO#pins.MOTOR_STBY_PIN*], [TB6612 *STBY*], [Global enable, straight from the MCU — high enables both bridges]),
  aligns: (center, left, left, left),
)

#v(3pt)
#callout("Leave the V+ and GND rows of those six columns unconnected", [
  Each PCA9685 channel breaks out three pins, but the motor driver takes only
  the signal. The V+ and GND pins of columns
  #used.at(0)–#used.at(5) stay empty; the driver gets its power and its ground
  from the rails below, not from the PWM header.
], kind: "warn")

= 3 · Power and ground

#htable(
  (auto, auto, auto, 1fr),
  ([], [Pin], [Rail], [Note]),
  ([☐], [TB6612 *VM*], [5.0 V from the LM2596], [Motor supply. Star-wire it from the regulator's output terminal — do not daisy-chain off the amplifier or the servo feed]),
  ([☐], [TB6612 *VCC*], [3.3 V], [Logic supply. Must match the PCA9685's VCC — see §4]),
  ([☐], [PCA9685 *VCC*], [3.3 V], [Logic supply, via the side header]),
  ([☐], [PCA9685 *V+* (screw terminal)], [5.0 V from the LM2596], [Servo rail. 6 V absolute maximum, per the board's own silkscreen. Fit ≥470 µF here]),
  ([☐], [TB6612 *GND* (either pin)], [Common ground], [Both GND pins are joined on-board; one wire is enough]),
  ([☐], [PCA9685 *GND*], [Common ground], [Terminal-block GND and header GND are the same net]),
  aligns: (center, left, left, left),
)

#v(3pt)
#callout("A missing ground does not stop a chip from running", [
  A board whose GND is not connected finds a return path through the ESD clamp
  diodes on its signal pins and keeps working — badly, in a way that reads as a
  firmware bug. Buzz every ground to the regulator's negative terminal *before*
  applying power, not after something misbehaves.
], kind: "danger")

= 4 · Why both logic rails are 3.3 V

Both parts specify their input threshold as a fraction of *their own* supply,
so the rail is not a free choice:

#htable(
  (auto, auto, 1fr),
  ([Part], [Spec], [Consequence]),
  ([PCA9685 SCL/SDA], [V#sub[IH] = 0.7 × V#sub[DD]], [At V#sub[DD] = 5 V the threshold is 3.5 V, above what the XIAO's 3.3 V I²C can drive. At 3.3 V it is 2.31 V]),
  ([TB6612 IN1/IN2/PWM], [V#sub[IH] = 0.7 × V#sub[CC]], [At V#sub[CC] = 5 V the threshold is 3.5 V, above the 3.3 V the PCA9685 would output]),
  ([TB6612 STBY], [V#sub[IH] = 0.7 × V#sub[CC]], [At V#sub[CC] = 5 V a 3.3 V GPIO cannot reliably lift STBY — the motors would stay in standby]),
  aligns: (left, left, left),
)

#v(3pt)
#callout("WIRING.md currently says 5 V here — the schematic says 3.3 V", [
  `WIRING.md`'s power diagram feeds 5 V to both VCC pins; the schematic at
  `docs/schematics/circuits/robocar_unified.py` feeds 3.3 V. The arithmetic
  above settles it in favour of 3.3 V for both, with the 5 V rail reaching only
  TB6612 *VM* and PCA9685 *V+*. Wire it that way and treat the diagram as the
  stale copy.
], kind: "danger")

= 5 · Before power-up, and first motion

#htable(
  (auto, 1fr, 1fr),
  ([], [Check], [Expected]),
  ([☐], [Continuity, every GND pin to the regulator's negative terminal], [Beeps — all of them]),
  ([☐], [Continuity, TB6612 VM to TB6612 VCC], [*No* beep. A beep means the 5 V and 3.3 V rails are shorted]),
  ([☐], [Regulator output, unloaded, before anything is connected], [5.0 V — the LM2596 trimpot spans 1.2–37 V and arrives set at neither]),
  ([☐], [`servo` on the console], [Reports the live PWM frequency — proves the PCA9685 answers on TCA9548A ch #pins.TCA_CH_PCA9685 before any motor moves]),
  ([☐], [`F` on the console, wheels off the ground], [Both wheels forward for about a second, then stop]),
  ([☐], [`trace` on the console], [I²C op counter climbing while driving, near-idle when stopped]),
  aligns: (center, left, left),
)

#v(4pt)

#grid(columns: (1fr, 1fr), column-gutter: 10pt,
  [
    == Truth table, per channel
    #htable(
      (auto, auto, auto, 1fr),
      ([IN1], [IN2], [PWM], [Mode]),
      ([H], [L], [H], [CW]),
      ([L], [H], [H], [CCW]),
      ([H], [L], [L], [Short brake]),
      ([H], [H], [—], [Short brake]),
      ([L], [L], [H], [Stop (coast)]),
      aligns: (center, center, center, left),
    )
    #v(2pt)
    #text(8pt, fill: theme.muted)[
      STBY must be high for any of it. The firmware drives forward as
      IN1 = high, IN2 = low.
    ]
  ],
  [
    == If a wheel runs backwards
    Swap that motor's *two output wires* at A01/A02 or B01/B02. Do not swap the
    IN1/IN2 wires and do not change the firmware: the direction bits are shared
    with the reactive controller's turn logic, so inverting them there fixes
    driving straight and breaks turning.

    #v(3pt)
    == If nothing moves at all
    Measure STBY. Below 2.31 V with V#sub[CC] at 3.3 V, the bridges are held
    off and every other wire is irrelevant.
  ],
)

#v(6pt)
#line(length: 100%, stroke: 0.5pt + theme.rule)
#v(3pt)
#text(7.5pt, fill: theme.muted)[
  *Sources.* Channel numbers and the STBY GPIO: `main/pin_config.h` via
  `docs/auto/pin_defs.typ`, regenerated and diffed by
  `.github/workflows/build-guide-check.yml`. Board layouts are read out of the
  vendors' published Eagle files — Adafruit `Adafruit-16-Channel-PWM-Servo-Driver-PCB`
  (`Adafruit PCA9685 rev C.brd`) and SparkFun `Motor_Driver-Dual_TB6612FNG`
  (`SparkFun_Motor_Driver-TB6612FNG_v11.brd`) — not from a product photo. Thresholds: NXP PCA9685 datasheet
  rev. 4 and Toshiba TB6612FNG datasheet, both listed in
  `docs/reference/datasheets/`. Rails and decoupling: `WIRING.md` § Power.
]
