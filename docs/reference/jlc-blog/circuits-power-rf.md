# JLCPCB blog: circuit, power and RF design findings

Domain: board-level circuit design for the repo's MCU projects (not bare-board
fabrication, not assembly). I read 58 posts in full or skimmed them for numbers, out of about 240
whose titles or keywords match the topic. Re-fetch any with `tools/jlc-blog.py fetch <slug>`.

Repo context checked first: `.claude/agents/pcb-reviewer.md`,
`.claude/rules/power-stage-checklist.md` (11 rows), the `carrier-pcb` skill (46
traps), `tools/kicad/pcb/review.py`, `tools/kicad/measure.py`,
`tools/hardware/board.py`/`model.py`, `docs/reference/datasheets/*`,
`docs/reference/boards/*`, `docs/reference/parts/*`, and the three carriers'
`hardware.toml` and `board.toml`.

**Bottom line.** The repo's power-stage checklist and carrier tooling are already
deeper than any of these blog posts on regulators, protection, decoupling
distance and via fanout. The blog adds value in four places: via and track
ampacity numbers, which a script can check; ESP32 board-level and RF rules,
which the repo's thin silicon pages and the reviewer do not cover; a few
decoupling-layout rules; and conflicts worth flagging.

## 1. Relevant articles (ranked)

| # | URL | Title | Value | Target repo file / section | Why |
|---|---|---|---|---|---|
| 1 | https://jlcpcb.com/blog/bms-pcb-design | Reliable BMS Boards Start With Quality PCB Manufacturing | high | `tools/kicad/pcb/review.py`, power-stage checklist row 11, pcb-reviewer §3 | Correct IPC-2221 widths (2 A gives 0.78 mm on 1 oz at a 10 °C rise). A 0.3 mm via is about 1 A per 10 °C, so plan one via per amp. 1 oz copper is 0.5 mΩ/sq. Includes the formula |
| 2 | https://jlcpcb.com/blog/esp32-ble-guide | ESP32 BLE Guide: Choosing the Right Chip and Designing Your PCB | high | `docs/reference/datasheets/mcu--esp32*.md`, embedded-best-practices WiFi section | Coexistence: 50/50 time slices, and a SoftAP with clients is "unstable". Light-sleep current with a 32.768 kHz crystal vs the main crystal: S3 3.3 mA vs 230 µA, C3 2.3 mA vs 140 µA. The original ESP32 needs the external crystal for BLE in light sleep |
| 3 | https://jlcpcb.com/blog/how-to-design-an-esp32-s2-module-pcb | How to Design and Assemble a Reliable ESP32 Module PCB on a 2-Layer Board | high | pcb-reviewer (new RF item), mcu pages | Antenna off the edge, or no copper on any layer under it (15 mm). 10 µF + 0.1 µF at 3V3. Strap-pin check. Bring-up: VCC–GND > 1 kΩ, then a Wi-Fi RSSI scan |
| 4 | https://jlcpcb.com/blog/esp32-pinout-guide | ESP32 Pinout Reference | high | `tools/hardware` join (strapping), `mcu--esp32.md` | Strap pins GPIO0/2/5/12/15. GPIO34–39 are input-only with no pulls. ADC2 is unusable with Wi-Fi on. GPIO16 is PSRAM on -R2 modules |
| 5 | https://jlcpcb.com/blog/bypass-capacitor-in-pcb-layout | Bypass Capacitor Placement in PCB Layout | high | `review.py` decoupling section, pcb-reviewer §1 | About 1 nH per mm of trace; a 0.3 mm via in 1.6 mm board is about 1.3 nH. Put the via on the far side of the cap from the IC, give each cap its own via, use two vias per pad, keep value steps within 10× |
| 6 | https://jlcpcb.com/blog/voltage-regulator-pcb-practical-design-guide | Voltage Regulator PCB: Practical Design Guide | high | power-stage checklist, `parts/ams1117-3.3.toml` | AMS1117: 15 V abs max; SOT-223 about 90 °C/W in free air, 46 °C/W with a large pour; about 1.1 W in free air and at best 2.2 W; 22 µF tantalum, and an X7R can oscillate. This agrees with the repo |
| 7 | https://jlcpcb.com/blog/ldo-vs-switching-regulator-comparison-pcb-layout | LDO vs Switching Regulator: Comparison & PCB Layout | med | power-stage checklist row 8/10 | PSRR at 1 kHz says nothing at 2 MHz. Keep switch-node copper minimal (cites TI). The HF input cap goes at the switch cell, not at the connector. Avoids rules of thumb |
| 8 | https://jlcpcb.com/blog/esp32-power-consumption-explained | ESP32 Power Consumption Explained | med | `mcu--esp32.md`, thinkpack battery projects | Typical chip currents (TX 240 mA, light sleep 0.8 mA, deep sleep 10 µA). AMS1117 Iq is 5 mA typ and 11 mA max. A dev board's deep-sleep floor is about 6.5 mA. Espressif asks for a 3.3 V supply of at least 500 mA |
| 9 | https://jlcpcb.com/blog/esp32-cam-guide | What Is the ESP32-CAM, and How to Wire, Power, and Program It? | med | `docs/reference/datasheets/board--esp32-cam-ai-thinker.md` | AI-Thinker: 5 V, supply of at least 2 A; 180 mA (LED off) and 310 mA (flash LED on); a brownout shows as ripple in the image; the reset line is not on the headers |
| 10 | https://jlcpcb.com/blog/how-to-choose-capacitor-voltage-rating | How Engineers Choose Capacitor Voltage Ratings | med | parts registry / pcb-reviewer §1 | Derating: MnO2 tantalum to 50 % of VR, ceramic and polymer to 80 %, electrolytic to 50–80 %. An unclamped relay coil makes 50–150 V spikes |
| 11 | https://jlcpcb.com/blog/decoupling-capacitors-guide | How to Choose and Place Decoupling Capacitors | med | pcb-reviewer §1 | A via on the far side of the board adds 1–2 nH. Class-II MLCCs lose 60–80 % of their value near rated voltage. Per-vendor defaults (ST: 100 nF per VDD + 4.7 µF; Espressif: 100 nF + 10 µF) |
| 12 | https://jlcpcb.com/blog/smd-crystal-oscillator-guide | SMD Crystal & Crystal Oscillator Guide | med | parts registry README (future chip-down or RTC) | Each load cap is 2 × (CL − Cstray), so a 20 pF crystal with 3 pF stray takes about 33 pF. On a 4-pad crystal a 90° rotation shorts XIN to XOUT through the lid. The pad map is vendor-specific |
| 13 | https://jlcpcb.com/blog/diode-type-guide | Diode Type Guide | med | pcb-reviewer §2 (input protection) | A TVS 2 mm from the pin, not 15 mm. 10 mm of trace at a 1 ns rise lets the spike through. Never use a 1N4007 in a 100 kHz buck |
| 14 | https://jlcpcb.com/blog/zener-diode-vs-schottky-diode | Zener Diode vs Schottky Diode | med | `parts/b5819w.toml` notes, battery designs | Schottky leakage rises about 20× per 25 °C (SS14: 0.2 mA at 25 °C, 6 mA at 100 °C), which drains a LiPo or backup path |
| 15 | https://jlcpcb.com/blog/understanding-star-grounding-pcb-design-ultimate-guide | Understanding Star Grounding in PCB Design | med | pcb-reviewer §5 "How separate grounds join" | Star-ground only slow, high-current returns (motors, speakers, LEDs). Keep a solid plane for USB, RF and fast clocks. Choose the star point by current path |
| 16 | https://jlcpcb.com/blog/capacitor-esr-explained | A Comprehensive Guide to Capacitor ESR | med | power-stage checklist row 6 | Output ripple ΔV ≈ ΔI·ESR + ΔI/(8·f·C), with a worked table (100 µF at 100 kHz and 0.5 A: electrolytic 150 mV, MnO2 tantalum 50 mV, polymer 5 mV) |
| 17 | https://jlcpcb.com/blog/the-ultimate-guide-to-smd-inductor | SMD Inductor Selection Guide | low-med | power-stage checklist row 7 | Isat above peak + 30 %, Irms above load + 20 %, ripple 20–40 % of Iout. Row 7 already covers this more rigorously |
| 18 | https://jlcpcb.com/blog/rf-pcb-layout-mistakes-and-guidelines | RF PCB Layout: Common Design Mistakes | med | pcb-reviewer §5 | A via fence at λ/20 (6 mm at 2.4 GHz in free space). Keep a switcher about 20 mm from RF. A 50 Ω microstrip on 1.6 mm FR-4 is about 2.8 mm wide. Has errors, see §3 |
| 19 | https://jlcpcb.com/blog/how-to-design-an-esp32-s3-development-board | How to Design an ESP32-S3 Development Board from Scratch | med | `mcu--esp32-s3.md` | RC delay on EN, a separate 5.1 kΩ on each CC pin for native USB, ESD arrays behind VBUS/D+/D−, the CP2102 auto-boot circuit. Conflicts with the repo, see §2 |
| 20 | https://jlcpcb.com/blog/understanding-dc-5v-input-in-pcb-design | Understanding DC 5V Input in PCB Design | med | pcb-reviewer §2 | 5 V input: a 5.6 V TVS plus a PPTC. Ripple targets: analog < 20 mVpp, digital < 50 mVpp. Measure ripple at 20 MHz bandwidth with the tip-and-barrel method. Its trace-width table is wrong, see §3 |
| 21 | https://jlcpcb.com/blog/track-width-vs-current-capacity-pcb-layout-tips | Track Width vs Current Capacity | low-med | — | Says about 500 mA per via and about 1 mm per 2 A. Conflicts with #1 |
| 22 | https://jlcpcb.com/blog/high-current-pcb-design-guide-2026 | The Engineer's Guide to High Current PCB Design 2026 | low-med | pcb-reviewer §3 connectors | Derate IPC-2152 by 15–20 %. A 10 mΩ contact at 20 A burns 4 W, so check connector contact resistance and rating |
| 23 | https://jlcpcb.com/blog/pcb-connectors-types-applications | PCB Connector Types Explained | low-med | pcb-reviewer §3 "current over the ribbon cable" | Rough per-pin ratings: 2.54 mm header 1–3 A, JST wire-to-board 1–5 A, FFC 0.5 A per contact. Use only as a sanity check; the vendor datasheet governs |
| 24 | https://jlcpcb.com/blog/hi-fi-amplifier-pcb-design | Amplifier PCB Design: Building Noise-free Hi-Fi Boards | low-med | audio-static-debugging Step 4 | Class-D: output filter at the speaker terminal, short PWM-node traces, 4–10 vias under heavy pads. First power-up on a current-limited supply (100–500 mA) into an 8 Ω dummy load |
| 25 | https://jlcpcb.com/blog/mastering-split-planes-pcb | Mastering Split Planes | low-med | pcb-reviewer §5 | Never split ground. Split power only over solid ground. A 0.25 mm moat. Stitching caps (0.1 µF / 1 nF) where a signal must cross |
| 26 | https://jlcpcb.com/blog/guide-to-ground-bounce-in-pcb | Guide to Ground Bounce in PCB | low | — | L·di/dt table (2 nH at 200 A/µs gives 400 mV). Generic |
| 27 | https://jlcpcb.com/blog/frequency-response-of-decoupling-capacitors | Understanding the Frequency Response of Decoupling Capacitors | low-med | — | Anti-resonance between unlike caps; damp it with 0.1–1 Ω ESR |
| 28 | https://jlcpcb.com/blog/ceramic-capacitor-types | Ceramic Capacitor Types Explained | low-med | parts registry `c0805.toml` notes | C0G for anything that sets a frequency; Y5V drifts +22/−82 %. Covered by datasheets |
| 29 | https://jlcpcb.com/blog/pcb-antenna-design-guide | Designing Antennas on PCB | low | — | λ/4 at 2.4 GHz is 31 mm. Keep 5–10 mm clear. Its 50 Ω feed width is wrong, see §3 |
| 30 | https://jlcpcb.com/blog/designing-effective-pcb-antennas | Designing Effective PCB Antennas | low | — | Solder mask over an antenna lowers its resonance by 1–2 %. FR-4 Df is about 0.02 |
| 31 | https://jlcpcb.com/blog/relay-wiring-diagram | Relay Wiring Diagram | low | — | Coil driver, base resistor and flyback diode worked out (SRD-05VDC: 70 Ω, 71.4 mA) |
| 32 | https://jlcpcb.com/blog/8x8-rgb-led-matrix-jlcpcb | 8x8 RGB LED Matrix (case study) | low | `led--ws2812b.md` | 100 nF per WS2812B. 64 LEDs at full white draw about 4 A. Says nothing about the 3.3 V data level (gap noted in §2) |
| 33 | https://jlcpcb.com/blog/how-to-reduce-noise-and-emi-in-pcb-layout | How to Reduce Noise and EMI in PCB Layout | low | — | 3W rule, λ/20 shield apertures. Misstates the 20H rule, see §3 |
| 34 | https://jlcpcb.com/blog/emc-design-guidelines | Design Guidelines for better EMC Standards | low | — | Edge time about 20 % of the bit period. Keep fast circuits away from between connectors |
| 35 | https://jlcpcb.com/blog/thermal-vias-in-pcb-design-for-heat-management | Thermal Vias in PCB Design | low | — | 0.3 mm drill at 0.8 mm pitch, against 1.0–1.2 mm in #1, #6 and #20 |

**Also on topic, low value** (generic, marketing, or not applicable to 2-layer
hobby carriers): comprehensive-guide-to-the-esp32-microcontroller, esp32-c3-vs-s3,
pcb-layout-guidelines-for-usb-type-c, choosing-the-right-usb-connector,
100nf-capacitor-guide, battery-pcb-board-guide, what-is-a-power-management-ic,
electrostatic-discharge-threat, pcb-fuse-ensuring-circuit-protection,
understanding-analog-and-digital-ground-in-pcb-design, mixed-signal-pcb-design,
circuit-board-ground-plane-and-power-plane-guide-2026, rf-pcb-design-guidelines,
pcb-antenna-design, effective-pcb-layouts-for-switching-regulators,
electrical-schematic-design-mistakes, ic-board-design-a-technical-guide-to-pcb-layout,
basic-bridge-circuits, how-to-build-an-arduino-bluetooth-controlled-car,
pcb-board-design-beginners-guide (IP2312 3 A charger, few numbers),
critical-role-of-gpio-pinouts-selection-in-embedded-systems.

The blog has nothing usable on these topics: I2C pull-ups and bus capacitance,
level shifting, motor-driver flyback and back-EMF layout (stepper/DRV8833/L298N),
LiPo charger and BMS design for 1S hobby packs, USB-C CC details beyond "5.1 kΩ
on CC", mmWave, or NFC antenna matching. The repo's own pages
(`display--ssd1306.md`, `io--mcp23017.md`, power-stage checklist) are better.

## 2. Proposed changes

Status key: **new** = not in the repo; **covered** = file:line; **conflict** =
the blog and the repo disagree. Fit: skill / reviewer item / tool check / reference doc.

### P1. Via count and track width against current, scripted. Tool check plus reviewer item. New
- Rule: per IPC-2221 (outer layer, ΔT = 10 °C), I = 0.048·ΔT^0.44·A^0.725 (A in
  mil²). On 1 oz copper that gives 2 A → 0.78 mm, 5 A → 2.77 mm, 10 A → 7.2 mm
  (inner layer about 2.6× wider). One 0.3 mm via (18 µm plating) carries a little over 1 A
  at a 10 °C rise, so stitch one via per amp at every layer change on a power path.
  Derate IPC-2152 by 15–20 % (high-current guide).
- Source: https://jlcpcb.com/blog/bms-pcb-design,
  https://jlcpcb.com/blog/high-current-pcb-design-guide-2026
- Status: new as a check. The repo cites the IPC-2221 estimate only in prose
  (`.claude/rules/power-stage-checklist.md:30`, row 11). `review.py` reports the
  narrowest track per class (`tools/kicad/pcb/review.py:13-14`) but takes no
  current. `board.toml` `[[net_classes]]` has no current field
  (`packages/robocar/unified-integrated/hardware/carrier/board.toml:541-580`).
  There is no via-per-amp rule anywhere (grep for "per via" and "ampac" finds nothing).
- Fit: add an optional `current_a` to `[[net_classes]]`. `review.py`, or
  `kicad.measure path`, then flags (a) the narrowest track below the IPC-2221 width
  for that current and (b) any layer change on the net with fewer than
  ceil(current_a / via_amp(drill)) vias, scaling via_amp by barrel circumference
  against 0.3 mm. Add a reviewer bullet under §3 "Current paths".
- Conflict to flag, not resolve: track-width-vs-current-capacity says "about 500 mA
  per via", half the BMS figure. understanding-dc-5v-input's table gives 2 A →
  1.3 mm, about 1.7× the IPC-2221 external value. Use the formula and cite IPC.

### P2. Strapping pins in the hardware join. Tool check. New
- Rule: ESP32 strap pins (GPIO0/2/5/12/15; on the S3, GPIO0/3/45/46) are sampled at reset. Any
  pull-up, pull-down, LED, or bus with pull-ups on them must hold the boot level.
  On the ESP32, GPIO12 high at reset selects the 1.8 V flash supply and the module will not boot.
- Source: https://jlcpcb.com/blog/esp32-pinout-guide,
  https://jlcpcb.com/blog/how-to-design-an-esp32-s2-module-pcb (§1.4 checks 2)
- Status: half-built. `tools/hardware/board.py:39,140` parses
  `BoardPin.strapping` from the board page, and only the tests read it
  (`tools/hardware/test_hardware.py:172,265`). No join check uses it.
  The board pages already mark strap pins (`docs/reference/boards/xiao-esp32s3.md:94,115`,
  `ttgo-lora32-v2.md:99-101`, `esp32-c3-super-mini.md:92`).
- Fit: in `join()`, when a role lands on a pin with `strapping=True` and the
  net carries a pull resistor or a bus with pull-ups (`[[bus_nets]]`), raise an
  error unless the entry carries `strap_ok = "<why the level is safe>"`. This works
  like the existing `[[undrawn]]` acknowledgement (`tools/hardware/model.py:92`).

### P3. Antenna and RF checks for carriers and enclosures. Reviewer item. New
- Rule: a module's PCB or ceramic antenna overhangs the carrier edge, or has no
  copper on any layer under it and none within 15 mm (S2 post; Espressif's figure,
  so verify it in the Espressif HW design guidelines before citing). Keep a switcher
  at least about 20 mm from the antenna and RF parts. A U.FL antenna (XIAO ESP32S3,
  `docs/reference/boards/xiao-esp32s3.md:44`) is routed away from motors, the
  ground pour and metal. Bring-up check: an RSSI scan against the bare module.
- Source: https://jlcpcb.com/blog/how-to-design-an-esp32-s2-module-pcb,
  https://jlcpcb.com/blog/esp32-ble-guide,
  https://jlcpcb.com/blog/rf-pcb-layout-mistakes-and-guidelines
- Status: new. `.claude/agents/pcb-reviewer.md` §5 (lines ~100-103) has no RF
  or antenna item, and no carrier `board.toml` mentions an antenna. It matters most for
  the XIAO ESP32C6 (onboard ceramic antenna, `xiao-esp32c6.md:41`) and the
  C3 SuperMini (PCB antenna, `esp32-c3-super-mini.md:42,123-129`, already a known trouble spot).
- Fit: a reviewer bullet in §5 and §7 (mechanical: antenna clearance to the
  chassis and case). Possibly an `[[keepouts]]` template in the socket registry
  entry for antenna modules.

### P4. Decoupling via placement. Tool check plus reviewer item. New in part
- Rule: the via goes on the far side of the capacitor pad from the IC, never between
  them. Give each cap its own via and never share one; use two per pad where there is room (about
  1.3 nH per 0.3 mm via through 1.6 mm). About 1 nH/mm of trace. Put the smallest value nearest the
  pin, and keep adjacent values within 10×.
- Source: https://jlcpcb.com/blog/bypass-capacitor-in-pcb-layout,
  https://jlcpcb.com/blog/decoupling-capacitors-guide
- Status: distance is covered (`tools/kicad/pcb/review.py:15-17,46`,
  `DECAP_MM = 3.0`). Via presence within 2.5 mm of an SMD pad is covered
  (`review.py:11-12`, `REACH_MM`). The new parts are via order (via between cap
  and IC), shared vias between caps, and the order of values.
- Fit: extend the decoupling section of `review.py`. For each decap, find the
  GND via nearest its ground pad and flag it when it is shared with another decap or is
  farther than the cap-to-IC distance. Add a reviewer §1 sub-bullet.

### P5. ESP32 and ESP32-S3 silicon pages: a "board-level design" section. Reference doc. New
- Content, each item to be verified against Espressif's HW design guidelines and
  datasheet as the page's source (the blog only points to them):
  - 3V3 supply rated at 500 mA or more (Espressif, quoted in esp32-power-consumption).
    The S2 post's "320 mA" is a TX peak, not the supply rating.
  - 10 µF + 0.1 µF at the module 3V3 pin. An RC delay on EN/CHIP_PU (S3 dev-board post).
  - Typical currents: TX 240 mA, RX 95–100 mA, light sleep 0.8 mA, deep sleep 10–150 µA
    (ESP32).
  - BLE light sleep needs a 32.768 kHz crystal (500 ppm), and the table of figures
    (esp32-ble-guide). Coexistence limits: SoftAP with clients is "unstable"
    (relevant to `usb-tools/facedancer-espdancer-fw` SoftAP and to the BLE gamepad
    projects).
  - ESP32: GPIO34–39 are input-only with no pulls; ADC2 is unusable with Wi-Fi on; GPIO16 is
    taken on -R2 modules.
- Source: https://jlcpcb.com/blog/esp32-power-consumption-explained,
  https://jlcpcb.com/blog/esp32-ble-guide, https://jlcpcb.com/blog/esp32-pinout-guide,
  https://jlcpcb.com/blog/how-to-design-an-esp32-s3-development-board
- Status: new. `docs/reference/datasheets/mcu--esp32.md` and `mcu--esp32-s3.md`
  are 24 lines each: specs and links only, with no power, strapping or layout content.
- Fit: reference doc (`board-reference` skill §5 "silicon page").

### P6. ESP32-CAM supply. Reference doc. New
- Rule: AI-Thinker specifies a 5 V supply rated at least 2 A. The board draws 180 mA
  (flash LED off) and 310 mA (on), with TX peaks above that. A sagging supply shows as
  brownout resets or ripple bands in the image.
- Source: https://jlcpcb.com/blog/esp32-cam-guide (verify it against AI-Thinker's sheet)
- Status: new. `docs/reference/datasheets/board--esp32-cam-ai-thinker.md` has no
  supply, brownout or regulator content (grep finds nothing).
- Fit: reference doc. This is also an `esp32-debugging` skill hint: in camera
  projects, a brownout or striped image points at the supply before the firmware.

### P7. Tantalum derating and failure mode. Reviewer item. Checked; compliant
- Rule: MnO2 tantalum is derated to 50 % of its rated voltage; ceramic and polymer to 80 %.
- Source: https://jlcpcb.com/blog/how-to-choose-capacitor-voltage-rating
- Status: the integrated carrier's 22 µF 16 V tantalums sit on 5 V and 3V3 rails
  (`packages/robocar/unified-integrated/hardware.toml:631-683`), which is 31 % or less,
  so they comply. Add a reviewer §1 bullet: "capacitor working voltage against its
  dielectric's derating, especially MnO2 tantalum on VBAT (8.4 V on 16 V is 52 %, which
  fails the rule)". There is currently no check.

### P8. TVS and input-protection placement. Reviewer item. New in part
- Rule: put a TVS or ESD diode within about 2 mm of the connector pin, in the order
  connector → ESD → filter → load. 10 mm of trace defeats it at a 1 ns rise. A 5 V input
  takes a 5.6 V TVS plus a PPTC.
- Source: https://jlcpcb.com/blog/diode-type-guide,
  https://jlcpcb.com/blog/understanding-dc-5v-input-in-pcb-design,
  https://jlcpcb.com/blog/pcb-layout-guidelines-for-usb-type-c
- Status: the PTC and reverse-polarity P-FET are covered (power-stage checklist row 3;
  `packages/robocar/unified-integrated/hardware.toml:155-201`;
  `packages/robotics/balancebot/hardware.toml:232-260`). Neither carrier has a TVS
  on the pack input. There is no reviewer item for transient or ESD clamping, or for its
  distance to the connector.
- Fit: a reviewer §2 bullet: "a transient clamp on the pack input (motor
  inductive kick, hot-plug), and its distance to the connector". Leave it as an
  open question for the owner rather than a rule.

### P9. Output ripple from ESR. Power-stage checklist. New
- Rule: ΔV ≈ ΔI·ESR + ΔI/(8·f·C), using ESR at the switching frequency. Ripple
  targets: about 20 mVpp for analog or ADC references, about 50 mVpp for logic. On the bench,
  measure at 20 MHz bandwidth with a short ground spring.
- Source: https://jlcpcb.com/blog/capacitor-esr-explained,
  https://jlcpcb.com/blog/understanding-dc-5v-input-in-pcb-design
- Status: row 6 (`.claude/rules/power-stage-checklist.md`) checks capacitance against
  the loop's table, but neither ripple magnitude nor the measurement method.
- Fit: a 12th checklist row or a note on row 6. The measurement method also suits
  `analog-fault-isolation.md` ("confirm the instrument can see it").

### P10. Shared motor and amplifier rail, and Class-D board checks. Skill section. New in part
- Rule: keep motor, servo and LED returns off the amplifier's and sensors' reference copper
  (a star branch for slow, high-current returns, with the plane kept solid otherwise). Put the
  Class-D output filter or ferrites at the speaker terminal. First power-up on a
  current-limited supply into an 8 Ω dummy load.
- Source: https://jlcpcb.com/blog/understanding-star-grounding-pcb-design-ultimate-guide,
  https://jlcpcb.com/blog/hi-fi-amplifier-pcb-design
- Status: `.claude/skills/audio-static-debugging/SKILL.md:132-140` (Step 4) only
  covers SD-pin pops. A supply-coupled whine that follows motor PWM is not listed as a symptom.
  The carrier has a ferrite bead on the amplifier supply
  (`packages/robocar/unified-integrated/hardware.toml:476-480`).
- Fit: add a symptom row to the skill's Step 1 and Step 4: a whine that tracks motor
  PWM or servo moves means the supply or ground path, so scope the amplifier's VDD
  during a motor step.

### P11. Crystal load capacitors and 4-pad orientation. Parts registry README. New, latent
- Rule: C1 = C2 = 2 × (CL − Cstray), with Cstray about 3–5 pF. The 4-pad map is vendor-specific,
  and a 90° rotation shorts XIN to XOUT through the lid.
- Source: https://jlcpcb.com/blog/smd-crystal-oscillator-guide
- Status: new, but no carrier has a crystal yet (all seat modules). It becomes relevant at
  the first chip-down MCU or RTC. Fit: `docs/reference/parts/README.md` note
  for crystal entries.

### P12. AMS1117 output capacitor. Conflict
- The S3 dev-board post puts a 10 µF ceramic on the AMS1117-3.3 output
  (https://jlcpcb.com/blog/how-to-design-an-esp32-s3-development-board). The
  regulator post (https://jlcpcb.com/blog/voltage-regulator-pcb-practical-design-guide)
  and the repo (`packages/robocar/unified/hardware.toml:701-703`, citing the AMS1117
  datasheet) call for 22 µF tantalum and warn that a low-ESR X7R can oscillate. The repo
  follows the datasheet, so no change. If a pcb-reviewer bullet is added: "the LDO output
  cap's ESR range is part of the stability spec; do not copy blog reference designs."
- The same post's 1.1 W free-air limit for SOT-223 is a useful sanity number for the
  reviewer's "LDO dissipation" item (`.claude/agents/pcb-reviewer.md:91`).

### Gaps found with no usable blog source (for other research)
- WS2812B data from a 3.3 V GPIO with the LED at 5 V. `docs/reference/datasheets/led--ws2812b.md`
  has no VIH or level-shift note. The blog's matrix post skips the question.
- I2C pull-up sizing is covered by the repo (`display--ssd1306.md:100`,
  `io--mcp23017.md:33`). The blog has nothing.
- Battery-divider drain and ADC source impedance for `packages/components/thinkpack-power`
  (it samples Vbat via ADC1, `power_mgr.c:4`). The blog has nothing.

## 3. Noise notes

- **Share:** about 55–60 % of what I read is generic or marketing (definitions, "JLCPCB
  offers…", FAQ padding). About 15 posts have concrete numbers. Most "comprehensive
  guide", "vs", "symbol" and "types" posts give no design rule. The 2026 posts
  (bms-pcb-design, ldo-vs-switching, voltage-regulator, esp32-ble-guide, esp32-pinout,
  esp32-power-consumption, smd-crystal) are far better: they cite TI, ADI and Espressif
  and hedge their rules of thumb.
- **Claims that look wrong, or posts that contradict each other:**
  - understanding-dc-5v-input, trace-width table: 0.5 A → 0.3 mm and 2 A → 1.3 mm
    on 1 oz at a 10 °C rise. IPC-2221 external gives about 0.12 mm and 0.78 mm
    (bms-pcb-design agrees with IPC). The table is conservative, not an IPC value.
  - Via ampacity: "about 500 mA per via" (track-width-vs-current) against "about 1 A per
    10 °C for a 0.3 mm via" (bms-pcb-design).
  - 50 Ω feed width: "about 1.2 mm on 1.6 mm FR-4" (pcb-antenna-design-guide) against
    "about 2.8 mm" (rf-pcb-layout-mistakes). About 3 mm is right for microstrip; 1.2 mm
    fits only coplanar waveguide with ground.
  - rf-pcb-layout-mistakes: "ENIG… non-magnetic surface". ENIG's nickel is
    ferromagnetic and lossy at RF; this is wrong. Its λ/20 via-fence spacing of 6 mm at 2.4 GHz uses the
    free-space wavelength; in FR-4 (εeff about 3.3) λ/20 is about 3.4 mm.
  - how-to-reduce-noise-and-emi: the "20H rule" is defined as keeping traces one
    dielectric thickness from the edge. The 20H rule pulls the power plane back 20·H from
    the ground plane's edge.
  - ESP32-S3 dev-board post: calls native USB "high-speed". It is full-speed (12 Mbit/s),
    so 90 Ω matching matters far less than the post implies.
  - Grounding advice contradicts itself across posts: mixed-signal-pcb-design says
    "separate analog and digital ground planes, joined at a single point";
    star-grounding, split-planes and ground-bounce say never split ground. The
    newer posts (never split; star only for slow high-current returns) agree with TI/ADI.
  - effective-pcb-layouts-for-switching-regulators: "keep input and output
    capacitors 1–2 cm apart". This has no basis, and it contradicts every reference layout
    with shared-ground caps.
  - Thermal-via pitch varies by post: 0.8 mm (thermal-vias) against 1.0–1.2 mm
    (bms, voltage-regulator, dc-5v). Follow the part's datasheet land pattern.
  - "15 mm keep-out under the antenna" (S2 post): plausible and Espressif-derived, but
    no citation. Verify against Espressif's HW design guidelines before using it.
