# JLCPCB blog: bare-board fabrication, DFM and layout for the KiCad carriers

Scope: 2-layer FR-4 carriers (`tools/kicad`, ADR-027), three boards:
balancebot, robocar-unified (saw), robocar-unified-integrated (saw, Economic
PCBA, fiducials, MAX98357A TQFN-16 at 0.5 mm pitch). Read: the `carrier-pcb`
skill, `pcb-reviewer`, ADR-027, `jlcpcb-order`, `boardcheck.py`, `panel.py`,
`fab.py`, `pcb/review.py`, the three `board.toml`, `.kicad_pro`, `.kicad_dru`,
and the committed `.kicad_pcb` files (via sizes, THT rings, fiducials measured
with throwaway regexes, numbers below). Posts read in full: 52 (re-fetch:
`tools/jlc-blog.py fetch <slug>`). JLCPCB capabilities page (https://jlcpcb.com/capabilities/pcb-capabilities) fetched 2026-10-27
(referred to as **CAP** below).

## 1. Relevant articles (ranked)

| URL | Title | Value | Target | Why |
|---|---|---|---|---|
| https://jlcpcb.com/capabilities/pcb-capabilities (not blog) | Rigid PCB capabilities | high | `.kicad_pro` rules, `boardcheck`, `pcb-reviewer` §6 | The authority. Several of its figures differ from the blog's (section 3) |
| https://jlcpcb.com/blog/character-design-specifications | Technical Guidance: Character Design Specifications | high | `boardcheck.SILK_MIN_MM`, `pcb/review.py`, `.kicad_dru`, trap 31/41 | Silk 0.15 mm from pads or CAM **deletes** it. 1.0 mm / 0.15 mm standard; 0.8 mm only with "high-precision characters". 1:6 ratio, 0.15 mm gap between characters |
| https://jlcpcb.com/blog/jlcpcb-pcb-osp-finish | JLCPCB Launches OSP Starting from 2-Layer PCBs | high | `jlcpcb-order` §3 step 2, carrier READMEs | 2026-07-30: OSP is free on 2-layer boards and ENIG "no longer free". OSP: 3-month shelf life, 4 thermal cycles |
| https://jlcpcb.com/blog/jlcpcb-lead-free-osp-upgrade | JLCPCB Enhances Lead-Free OSP… | high | `jlcpcb-order` | 2026-10-08 chemistry. Keep peak ≤260 °C, no 300–400 °C hot plates or hot air, about 1 week in open air. Rules out leaving THT parts to hand-solder for weeks after PCBA |
| https://jlcpcb.com/blog/how-to-resolve-short-circuit-hasl | How to Resolve Short Circuit Defects Caused by HASL | high | `jlcpcb-order` step 2, `pcb-reviewer` §6 | HASL bridges at pitch ≤0.5 mm, so use ENIG there; HASL is fine above 0.65 mm. The integrated carrier's TQFN is 0.5 mm, but the skill's default is HASL |
| https://jlcpcb.com/blog/five-via-finishes | Five Via Finishes | high | `boardcheck` net-class validation, trap list | Tented vias: drill ≤0.4 mm ideal, ≤0.5 mm max. Above that, no complaint is accepted. The carriers' power vias are 1.0/0.5, exactly at the limit. Plugging is impossible within 0.35 mm of a mask opening. Component holes +0.13/−0.08 mm; via holes have no tolerance |
| https://jlcpcb.com/blog/how-to-set-pcb-via-tenting | How to Set PCB Via Tenting | med | `jlcpcb-order`, trap list | The Gerber decides tenting and the order-page option is secondary. Same 0.4/0.5 mm limit. Check it in JLC's Gerber viewer |
| https://jlcpcb.com/blog/basic-design-of-solder-mask | Basic Design of Solder Mask | high | `.kicad_pro`, `pcb/review.py` | Mask dam needs a pad gap ≥0.10 mm (green, 1 oz) or ≥0.13 mm (black/white). Narrower gaps get a gang opening. The post says 2-layer boards get a 0.05 mm expansion, which **conflicts** with CAP's 1:1 (section 3). Says "do not modify original solder mask size" |
| https://jlcpcb.com/blog/mouse-bite-panelization-guide | Technical Guidance: Mouse Bite Panelization Guide | high (already used) | `panel.py`, ADR-027 addendum | Source of the repo's 0.6 mm holes, 0.35 mm web, 5 holes, 50–60 mm spacing and 1.2 mm minimum gap. Also: CAM silently widens a web to 0.3 mm. Panel's long side preferably <250 mm |
| https://jlcpcb.com/blog/v-cut-panelization-standards | Technical Guidance: V-Cut Panelization Standards | med | ADR-027 addendum, `panel.py` (a future `vcut` mode) | Copper ≥0.4 mm from the centreline (1.0 recommended, 2.0 for tall parts). Straight edge-to-edge lines only, double-sided, 25°. Panel 70–475 mm, ±0.4 mm tolerance. Fiducial centre ≥3.85 mm from the edge. Connecting edge ≥3 mm |
| https://jlcpcb.com/blog/how-to-avoid-damaged-traces-and-pads | How to Avoid Damaged Traces and Pads | low (covered) | trap 22 | Milling ±0.2 mm, so keep copper ≥0.3 mm from a milled edge. The repo uses 0.5 mm |
| https://jlcpcb.com/blog/thermal-relief-pad-design | Thermal Relief Pad Design | med | `[pour]` spoke settings, `pcb-reviewer` §3 | IPC-2221B spoke floor 0.2 mm. A 4×0.25 mm relief is 1–3 mΩ, so above 3 A continuous use a solid connection. Design spokes 30% over the fab minimum |
| https://jlcpcb.com/blog/the-importance-of-copper-pour-in-empty-areas | The Importance of Copper Pour in Empty Areas | med | `[pour]`, `pcb-reviewer` §6 | Spoke and gap >0.25 mm. Pour ≥0.2 mm from traces (0.5 mm ideal). No hatched pours. An unpoured area plates thicker, so on a sparse board size spacing as for 2 oz with 8 mil minimum |
| https://jlcpcb.com/blog/castellated-pcbs-introduction-and-design-requirements | Castellated PCBs: Design Requirements | low | `derive_xiao` note | Castellated holes ≥0.6 mm, 1 mm from the edge, ring 0.25 (0.18 abs). Its own mouse-bite figures (1.0–1.2 mm c-c, 4–6 holes) conflict with the guide's. The carriers mount no castellated part |
| https://jlcpcb.com/blog/reliable-pcb-slot-holes-design | Reliable PCB Slot Holes | low | `pcb-reviewer` §6 | Plated slot ≥0.5 mm, non-plated ≥1.0 mm, length ≥2.5× width. CAP says 2× and no rectangular slots |
| https://jlcpcb.com/blog/annular-rings-pcb-design | Annular Rings in PCB Design | med | `pcb/review.py` | Position ±0.075 mm; a nominal ring shrinks by the tolerance stack. IPC Class 2 outer ring 0.15 mm |
| https://jlcpcb.com/blog/pcb-panel-size | Choosing the Right Panel Size | med | `jlcpcb-order`, ADR-027 | JLC SMT panel: 5 mm rails, 2 mm NPTH tooling holes, 1 mm fiducials at 3.85 mm from the edge. Routed gap 1.6–2.4 mm. Parts ≥1 mm from a V-score (2 mm if tall) |
| https://jlcpcb.com/blog/pcb-fiducials | Fiducial Markers… | med | `pcb-reviewer` §6, integrated board.toml | ≥2 global fiducials, diagonal (3 in an L is better). 1 mm pad, 2 mm mask, a 3 mm radius free of copper, silk and vias |
| https://jlcpcb.com/blog/how-to-avoid-incomplete-milling | Avoid Incomplete Milling of Internal Corners | low | `panel.py` | A round cutter leaves spurs at internal corners between abutting boards. Fixes: a drilled corner, a gap with mouse bites, or a sacrificial strip |
| https://jlcpcb.com/blog/border-lines-3d-footprint-lines | Border Lines on Mechanical Layer 1 | low | `fab.py` | JLC takes the lowest-numbered mechanical or outline layer and ignores the others. `fab.py` exports Edge.Cuts only, so it is covered |
| https://jlcpcb.com/blog/kicad-mcp-how-to-design-a-pcb-with-claude-code | KiCad MCP with Claude Code | med (validates) | `carrier-pcb` traps 22, 14 | Same failures: the router used 0.35 mm against a 0.5 mm edge rule, and overlapping labels merged nets that ERC only warned about. "0 DRC is not manufacturing approval" |
| https://jlcpcb.com/blog/gpt-6-astra-pcb-design-in-kicad | GPT-6 Astra PCB Design in KiCad | low-med | `carrier-pcb` | Ran ERC, DRC and parity again on a copy in a separate directory (portability), with no exclusions. Its "0 DRC" still had 5 categories ignored, so read the rule severities |
| https://jlcpcb.com/blog/how-to-design-an-esp32-s2-module-pcb | How to Design an ESP32-S2 Module PCB | med | `carrier-pcb` (new trap), `pcb-reviewer` §5 | Clear copper on all layers under a module's PCB antenna ("≥15 mm"). 1 mm placement clearance around the module |
| https://jlcpcb.com/blog/esp32-ble-guide | ESP32 BLE Guide | med | same | Antenna at a corner, beyond the edge where possible. No copper, traces or parts under it on any layer, or cut the board away |
| https://jlcpcb.com/blog/how-to-design-an-esp32-s3-development-board | ESP32-S3 Development Board | low-med | same | Module at one end. Keep-out on all layers |
| https://jlcpcb.com/blog/pcb-antenna-design-guide | PCB Antenna Design Guide | low | same | No ground under the antenna element, 5–10 mm clearance |
| https://jlcpcb.com/blog/copper-weight-and-trace-width-balance | Copper Weight and Trace Width | low | `pcb-reviewer` §3 | IPC-2152: 3 A at a 10 °C rise is 1.27 mm external on 1 oz, 0.64 mm on 2 oz |
| https://jlcpcb.com/blog/pcb-design-rules-best-practices | PCB Design Rules and Guidelines | low | `pcb-reviewer` §3 | Current table, 1 oz external at 10 °C: 1 A ≈10 mil, 2 A ≈30 mil, 3 A ≈50 mil, 5 A ≈100 mil |
| https://jlcpcb.com/blog/pcb-trace-width-guide | PCB Trace Width Guide | low | `measure resistance` cross-check | 1 oz ≈0.49 mΩ/sq. 2 A at 10 °C is 0.78 mm |
| https://jlcpcb.com/blog/pcb-copper-balancing | Copper Balancing in PCBs | low | — | Copper density 40–60%. IPC-6012 bow and twist ≤0.75% for SMT |
| https://jlcpcb.com/blog/standard-pcb-thickness | Standard PCB Thickness | low | `enclosure-cad` | 1.6 mm standard. CAP: ±10% at ≥1.0 mm (1.44–1.76 mm) |
| https://jlcpcb.com/blog/how-to-avoid-copper-peel-off | How to Avoid Copper Peel Off | low | — | HASL bath 275–300 °C, above Tg. Small single-sided pads peel; rings ≥0.3 mm or ENIG help |
| https://jlcpcb.com/blog/a-beginners-guide-to-tented-vs-untented-vias | Tented vs Untented Vias | low | — | Repeats five-via-finishes |
| https://jlcpcb.com/blog/via-in-pad-design-deep-dive | Via-in-Pad Design Deep Dive | low | trap 37 | POFV via holes 0.25–0.35 mm (0.20–0.50 abs). Not used: the carriers forbid via-in-pad |
| https://jlcpcb.com/blog/dfm-analysis-in-pcb-design | Understanding the Importance of DFM Analysis | low | `jlcpcb-order` (one line) | The user flagged this one. It is generic, with no numbers. Its only usable pointer is JLCPCB's own online DFM ("JLCDFM", DFM-Analysis-Result help article) as a pre-order check |
| https://jlcpcb.com/blog/free-dfm-tool | Free DFM Tool | low | same | Same, a JLCDFM advert |

Also on topic, low value (generic, marketing or other-process): `pcb-mouse-bites`
(its figures are wrong, see section 3), `pcb-design-efficiency-mouse-bites`,
`panelization-with-issues-irregular-shapes`, `partially-flipped-panelization`
(Altium), `pcb-solder-mask-expansion-guide`,
`pcb-silkscreen-design-manufacturing-guide`, `critical-aspect-of-pcb-panel-size`,
`pcb-warpage-control`, `pcb-voltage-clearance-guide` (IPC-2221 table; carriers
≤12 V: 0.1 mm), `right-tg-value-for-pcb`, `hasl-vs-enig-surface-finishes`,
`prepare-perfect-gerber-files`, `5-pcb-board-layout-design-rules-for-manufacturing-success`,
`common-pcb-design-mistakes`, `pcb-design-mistakes-of-beginners`,
`pcb-copper-pour-basics` (duplicates the copper-pour post),
`design-rule-check`, `via-stitching-in-pcb-design`, `pcb-antenna-design`,
`designing-effective-pcb-antennas`, `rf-pcb-layout-mistakes-and-guidelines`,
`jlcpcb-fr4-pcb-price-adjustment-august2026` (volume surcharges only),
`esp32-smart-rc-control-board` and the other "Case Studies" / "PCB Fabrication
Case" posts (`flight-computer-of-model-rockets`, `autonomous-exploration-vehicle`,
`guitar-effect-pedal-pcb`, `precision-72-led-watch-jlcpcb-case`,
`pibrick-cm5-jlcpcb-case-study`, `athena-6-layer-pcb`, …): testimonials, no
DFM rules.

## 2. Proposed changes

Status key: **new**, **covered** (file:line), **conflict** (file:line).
"Carriers" = the three committed `.kicad_pro` files (identical rule sections).

### A. Silkscreen

A1. **CAM deletes silkscreen within 0.15 mm of a pad or on exposed copper.**
JLC does this without asking unless the order remarks say otherwise. Polarity
marks and pin-1 dots are the usual victims. CAP: "Pad to silkscreen 0.15 mm".
Recommended ≥0.25 mm. Sources: character-design-specifications, CAP Legend.
- Status: **new**. The carriers set `"min_silk_clearance": 0.0` (balancebot
  `.kicad_pro:152`) and `"silk_over_copper": "warning"` (`:118`).
  `pcb/review.py:21-22` and `:255` measure only text height and stroke, not
  distance to pads.
- Risk: trap 41's widening (`pcb/silk.py`, 0.12 → 0.15 mm) moves each stroke
  edge 0.015 mm closer to the pads.
- Fit, an automated check:
  - a `.kicad_dru` rule per carrier, `(rule "silk to pads" (condition
    "A.Layer == '?.Silkscreen' && B.Type == 'Pad'") (constraint silk_clearance
    (min 0.15mm)))`;
  - `silk_over_copper` stays as it is: as a warning it already fails the
    gate under `--severity-all`, but it catches overlap only, not the
    0.15 mm gap;
  - a `pcb/review.py` section "nearest silkscreen to a pad", flagged under
    0.15 mm;
  - a skill trap: the marks the polarity tripwire relies on (trap 20, #803)
    can be erased at the fab while every gate passes.
- Verify that KiCad 10 applies `silk_clearance` to pads. Use the repo's
  "verify the gate can fail" step: move a "+" text onto a pad.

A2. **0.8 mm text is a JLC option, not the standard.**
- character-design-specifications:
  - standard characters ≥1.0 mm tall and ≥0.15 mm stroke;
  - 0.8 mm / 0.1 mm only with "high-precision characters";
  - recommended stroke ≥0.2 mm, ratio 1:6, ≥0.15 mm between characters.
- CAP: "Characters height less than 40 mil (1.0 mm) will be unidentifiable."
- Status: **conflict**:
  - `boardcheck.py:256-259` (`SILK_MIN_MM = 0.8`, "the floor");
  - ADR-027:224;
  - trap 31 (SKILL.md:273-276);
  - `pcb-reviewer` §6 ("0.8 mm is the floor");
  - the integrated `board.toml:226` sets `C9`, `C10`, `C11` to 0.8.
- Fit: change the skill and reviewer wording to "1.0 mm unless the order
  selects high-precision silkscreen". Keep 0.8 as the hard floor in
  `boardcheck` and print the sub-1.0 refs in `carrier-review` (it already
  does, `review.py:246-251`).
- Also: 1:6 at 1.0 mm is a 0.167 mm stroke, and build_board.py uses 0.16 mm.
  Marginal; note only.

### B. Surface finish (process, `jlcpcb-order`)

B1. **Pick the finish by pitch and by who solders.**
- Facts:
  - HASL bridges at pitch ≤0.5 mm; above 0.65 mm it is fine
    (how-to-resolve-short-circuit-hasl).
  - OSP has been free on 2-layer boards since 2026-07-30, and ENIG is now an
    extra charge (jlcpcb-pcb-osp-finish).
  - OSP shelf life is 3 months and its open-air life is about 1 week. It
    takes no 300–400 °C hot plate or hot air, peak ≤260 °C
    (jlcpcb-lead-free-osp-upgrade).
  - HASL keeps 6–12 months. CAP lists HASL leaded or lead-free, ENIG and OSP.
- Status: **new**. `jlcpcb-order/SKILL.md:73` defaults to "FR-4, 1.6 mm,
  HASL, 1 oz" for every board. `grep -w OSP` and `grep ENIG` over the repo's
  Markdown find nothing. The integrated carrier
  (`docs/reference/parts/max98357a-tqfn16.toml:9`, 0.5 mm-pitch TQFN, Economic
  PCBA) gets HASL by that default.
- Fit: a decision rule in `jlcpcb-order` §3 step 2:
  - HASL or lead-free HASL for hand-soldered, socketed carriers;
  - ENIG (paid) or OSP (free) when any SMD pitch is ≤0.5 mm;
  - OSP only when the THT parts left for hand soldering go on within days,
    with an iron and not hot air.
  - Optionally a `pcb-reviewer` §6 item: "finish vs finest pitch".

### C. Vias, holes, rings

C1. **Tented-via drill limit.** Drill ≤0.4 mm is reliable and ≤0.5 mm is the
maximum. Above 0.5 mm the tent and plug can fail, and JLC accepts no
complaint (five-via-finishes, how-to-set-pcb-via-tenting).
- Status: **at the limit, unchecked**:
  - the power class is `via_um = [1000, 500]` in all three `board.toml`
    (e.g. balancebot `:213`, `:253`);
  - the committed boards carry 17 / 7 / 9 vias of 1.0/0.5 mm, measured;
  - the boards tent both faces (`.kicad_pcb` `(tenting (front yes) (back yes))`).
- Fit, an automated check: `boardcheck` net-class validation (`validate_config`,
  around `boardcheck.py:686`) rejects a via drill >0.5 mm while tenting is on,
  and warns over 0.4 mm. Add a one-line skill trap.

C2. **PTH annular ring on 2-layer 1 oz:** 0.25 mm recommended, 0.18 mm
absolute minimum (CAP; castellated post).
- Status: **new, one footprint below the recommendation.** On both robocar
  carriers, J3 `Connector_AMASS:AMASS_XT30PW-M` has 1.4 mm pads on 1.0 mm
  drills, a 0.20 mm ring. It is the battery connector.
- With hole tolerance +0.13/−0.08 and position ±0.075 (CAP), the worst-case
  ring is under 0.1 mm. Every other THT pad is ≥0.275 mm.
- `.kicad_pro` `min_via_annular_width 0.1` covers vias only, as the name says.
  Verify whether KiCad 10's `annular_width` also judges pads.
- Fit, an automated check: `pcb/review.py` reports the thinnest PTH-pad ring
  with ref and footprint, flagged under 0.25 mm. Add a `pcb-reviewer` §6
  item. The fix would be a derived XT30 footprint with 1.6–1.7 mm pads; the
  owner decides.

C3. **Hole-to-hole for pads is 0.45 mm; for vias 0.2 mm** (CAP).
- Status: **looser**: `.kicad_pro:148` `min_hole_to_hole 0.25`, severity
  `"warning"` (`:96`).
- Tightening to 0.45 would flag the mouse-bite NPTH rows: 0.95 − 0.6 =
  0.35 mm. Those are a JLC-sanctioned feature, so exempt the MB footprints
  with a `.kicad_dru` rule.
- Fit: a `.kicad_dru` rule. Low priority; no current pad pair is near it.

C4. **Hole to track:** PTH 0.28 mm (0.35 recommended), via 0.2 mm, NPTH
0.2 mm (CAP).
- Status: **covered in effect**. `min_hole_clearance 0.25` plus copper
  clearance ≥0.2 mm from the pad ring (≥0.275) keeps tracks ≥0.47 mm from any
  PTH hole. No change.

C5. **Via geometry floor:** the via diameter must be 0.1 mm larger than the
hole (0.15 preferred). The preferred minimum hole is 0.2 mm. A 0.2/0.25 mm
hole with a diameter <0.45 mm costs extra (CAP).
- Status: **covered**. The smallest via is 0.6/0.3, and `jlcpcb-order:74`
  checks the drill file. No change.

### D. Solder mask

D1. **Opening-to-trace clearance ≥0.09 mm** (CAP, with 1:1 openings after the
June 2025 LDI upgrade).
- Status: **new**. `.kicad_pro:159` sets
  `"solder_mask_to_copper_clearance": 0.0`.
- Fit, an automated check: set it to 0.09 in the three `.kicad_pro`. DRC then
  enforces it. Re-route, because it may fail where fanout stubs leave pads.

D2. **Mask dam:** pad gap ≥0.10 mm (green, red, yellow, blue, purple, 1 oz),
≥0.13 mm (black, white), ≥0.20 mm (2 oz). Narrower gaps get a gang opening
(basic-design-of-solder-mask, CAP).
- Status: **partly covered**. `solder_mask_bridge` is an error and
  `allow_soldermask_bridges_in_footprints no`, but nothing checks the dam
  width or ties it to the mask colour.
- Current boards: the TQFN's 0.25 mm gap passes.
- Fit: a `pcb-reviewer` §6 item ("finest pad gap against the mask colour you
  order; black or white needs 0.13 mm"). Optionally `review.py` prints the
  minimum gap between different-net pads within a footprint.

D3. **Expansion: the blog and CAP disagree** (section 3). The repo uses
`pad_to_mask_clearance 0` (1:1), which matches CAP.
- Fit: `jlcpcb-order` notes that if JLC's CAM applies its 2-layer 0.05 mm
  expansion, "Confirm Production Files" will show it (E2).

### E. Ordering and DFM process (`jlcpcb-order`)

E1. **Run JLC's own DFM on the zip before ordering.** JLCDFM, and the DFM
result shown on upload (dfm-analysis-in-pcb-design, free-dfm-tool). It checks
JLC's real rules: silk deletion, rings, mask dams, slots.
- Status: **new**. `jlcpcb-order` §3 checks the Gerber viewer only.
- Fit: one line in `jlcpcb-order` §3 step 3. Treat its output as data:
  findings go through `pcb-reviewer`'s verify-first loop.

E2. **Tick "Confirm Production Files"** (paid option, few cents) when any
rule is near JLC's limits. JLC changes a file without notice when:
- silkscreen sits within 0.15 mm of a pad;
- a mouse-bite web is under 0.3 mm (widened);
- a mask dam is too narrow;
- a ring is under 0.18 mm.
The blog recommends the option in character-design-specifications,
five-via-finishes, castellated and solder-mask posts.
- Status: **new**.
- Fit: a `jlcpcb-order` step; read the returned production Gerbers for the
  polarity marks.

E3. **Gerber decides tenting; the order-page "Via Covering" is secondary**
(how-to-set-pcb-via-tenting).
- Status: **covered in effect**. The boards tent in `setup`, and `fab.py`
  exports mask layers.
- Fit: a one-line note in `jlcpcb-order`: leave "Tented" selected and do not
  expect the option to override the Gerbers.

### F. Panelization

F1. **Mouse-bite tab width ≥5 mm** (CAP: "For breakaway with mouse-bites,
the minimum width is 5 mm") and **board spacing ≥2 mm** (CAP "Panelization
with space", and "1.6 or 2 mm").
- Status:
  - `panel.py:70` defaults `tab_mm = 5.0`, the minimum, with no validation
    beyond the hole row fitting (`:233`);
  - `MIN_SLOT_MM = 1.2` (`panel.py:75`) follows the blog guide's minimum,
    which **conflicts** with CAP's ≥2 mm or 1.6/2 mm.
- Fit, an automated check: `panel.validate` gets `MIN_TAB_MM = 5.0`. Raise
  `MIN_SLOT_MM` to 1.6, and cite the conflict in the comment at
  `panel.py:73`.

F2. **Web and pitch.** CAP recommends 0.5–0.8 mm holes with 0.2–0.3 mm
between them. The blog guide says 0.35–0.4 mm, minimum 0.3.
- Status: **conflict between JLC sources**. The repo's 0.35 mm web
  (`panel.py:14`, ADR-027:244-247) satisfies the guide and exceeds CAP's
  range. ADR-027:246-247 calls 0.25 mm "under JLCPCB's 0.3 mm minimum", but
  CAP would accept 0.25.
- Fit: no change. A footnote in ADR-027 that CAP and the guide differ.

F3. **V-cut as a third option for the saw carriers.**
- Rules (CAP, v-cut-panelization-standards):
  - straight, edge-to-edge lines only;
  - copper ≥0.4 mm from the centreline (1.0 recommended);
  - panel 70–475 mm, ≥0.6 mm thick;
  - lines ≥2 mm apart (3 recommended);
  - ±0.4 mm tolerance;
  - a hybrid (V-cut one axis, route the other) is allowed.
- Status: **new**. The unified carrier is 100×100 mm and its x = 70.5 line
  runs edge to edge (`board.toml:216`), so it can be V-cut. The head lines
  stop at it (`:217-220`) and could not be: hybrid only.
- The existing ≥3 mm copper-free bands (README:20) already meet the 0.4/1.0 mm
  clearance.
- Same pricing caveat as mouse bites: JLC counts V-cut pieces as separate
  designs (`jlcpcb-order:35-38`).
- Fit: an ADR-027 addendum option, not urgent.

F4. **SMT panel features:**
- 5 mm rails, 2 mm tooling holes;
- 1 mm fiducials centred 3.85 mm from the panel edge (CAP mouse-bite and
  V-cut sections, pcb-panel-size);
- ≥2 fiducials diagonal, 3 in an L preferred, with a 3 mm radius free of
  copper, silk and vias (pcb-fiducials).

Status: **mostly covered**. The integrated carrier has FID1–3 in an L,
`board.toml:218-222`, at 3.5 mm from the edge, and the order uses "Added by
JLCPCB" rails (`jlcpcb-order:79`), which move them inward.
- **SUSPECT**: ordered without rails, 3.5 < 3.85 mm.
- Fit: a `pcb-reviewer` §6 item: fiducials ≥3.85 mm from the board edge, or
  rails added; a 3 mm clear radius.

### G. Copper, pour, thermal

G1. **Thermal reliefs on high-current pads.** Above about 3 A continuous,
connect the pad solid; a 4×0.25 mm relief is 1–3 mΩ (thermal-relief-pad-design).
- Status: **partly covered**. Every GND pad also has a routed track
  (`.kicad_dru` comment), and spokes are 0.35/0.3 mm (balancebot `[pour]`).
  On boards where `min_resolved_spokes 1` applies, a battery-return or motor
  pad can still meet the plane through one 0.35 mm spoke plus its track.
- Fit: a `pcb-reviewer` §3 item: for pads on a >3 A path (XT30, JST battery,
  VMOT bulk), check the route, not the relief. Optional: `[pour] solid =
  ["J1:2", ...]` setting a per-pad `ZONE_CONNECTION_FULL` in `route_board.py`.

G2. **Pour clearance and spokes:**
- spoke and gap >0.25 mm; pour ≥0.2 mm from traces (0.5 ideal); no hatched
  fills (the-importance-of-copper-pour-in-empty-areas);
- IPC-2221B spoke floor 0.2 mm (thermal-relief-pad-design).

Status: **covered**. `[pour] clearance_mm 0.4`, spoke 0.35, gap 0.3, solid
fill. No change.

G3. **Copper to a routed edge ≥0.2 mm** (CAP) or ≥0.3 mm (damaged-traces
post).
- Status: **covered and stricter**: `min_copper_edge_clearance 0.5` and
  trap 22.

### H. Antenna keep-out (new trap and reviewer item)

H1. **Modules with an on-board antenna:**
- remove copper on both layers under the antenna and around it;
- put the antenna at an edge or corner, beyond the edge if possible;
- keep 1 mm placement clearance round the module.

The posts give "≥15 mm" (esp32-s2-module-pcb, which looks generic) and
"5–10 mm" (pcb-antenna-design-guide). Take the module vendor's own figure
over either.
- Status: **new**. No "antenna" in `carrier-pcb`, `pcb-reviewer`, ADR-027 or
  any carrier README.
- The current carriers use XIAO ESP32-S3 (Sense), U.FL plus an external FPC
  antenna (`docs/reference/boards/xiao-esp32s3.md:44`), and the RP2350 (no
  radio). None needs it today.
- The XIAO ESP32-C6 has an on-board ceramic antenna
  (`xiao-esp32c6.md:41`), and so does the ESP32-C3 SuperMini.
- Fit:
  - a `carrier-pcb` trap and a `pcb-reviewer` §5 item: a socketed module's
    antenna end overhangs the edge, or sits over a `[[keepouts]]` rect
    (copper-free, both layers) taken from the module's own drawing;
  - for U.FL modules, the FPC antenna mounts off the carrier, away from the
    GND pour;
  - optional: the board's reference page names its antenna region, and
    `boardcheck` checks that a keepout covers it.

### I. Smaller notes

- **Board thickness ±10% at ≥1.0 mm** (1.44–1.76 mm for 1.6) (CAP,
  standard-pcb-thickness). `tools/enclosure/enclosure/board.py:168` uses the
  nominal 1.6. Note for `enclosure-cad`: slot or snap fits on the PCB edge
  need ±0.16 mm.
- **NPTH ≥0.5 mm, non-plated slot ≥1.0 mm, plated slot ≥0.5 mm with length
  ≥2× width, no square-cornered slots** (CAP). Covered for mouse bites
  (`MIN_HOLE_MM 0.5`). A `pcb-reviewer` §6 item if a slotted footprint
  arrives.
- **Min track/space on 2-layer 1 oz is 0.10/0.10 mm; SMD pad to pad 0.15 mm**
  (CAP). The repo's 0.2 mm `min_track_width` and clearance are 2× margin.
  Trap 24 mentions lowering `min_track_width` for necking; the JLC floor is
  0.1 mm, and keep ≥30% above it (thermal-relief post).
- **Current widths:** 1 oz external at a 10 °C rise is ~1.27 mm for 3 A and
  ~0.78 mm for 2 A (IPC-2152, three posts agree). Useful as the `pcb-reviewer`
  §3 yardstick. The repo's power class is 1.0 mm and VBAT 1.5 mm.
- **DRC severities: covered.** `tools/kicad/check.py:88-90` runs DRC with
  `--severity-all --exit-code-violations`, so the rules set to `warning` in
  `.kicad_pro:96-125` already fail the gate. This includes `hole_to_hole`,
  `silk_over_copper` and `text_height`. Raising them to `error` (A1, C3) is
  cosmetic. The thresholds are the gap: `min_silk_clearance 0`,
  `solder_mask_to_copper_clearance 0`, `min_hole_to_hole 0.25`. The GPT-6
  KiCad post's "5 categories ignored" lesson: only `ignore` severities hide
  rules. Grep `.kicad_pro` for `"ignore"` before trusting a pass.

## 3. Noise notes

- About 70% of the ~250 domain posts are generic SEO text:
  - "What is X", tool adverts (Altium, Valor), rewrites of the same table;
  - "Case Studies" and "PCB Fabrication Case" posts are testimonials with no
    DFM numbers.
- The useful content is the JLC-authored "Technical Guidance" and DFM series
  (character spec, mouse-bite, V-cut, five via finishes, solder mask,
  castellated, HASL shorts, damaged traces) and the 2026 product-update posts
  (OSP).
- The user-flagged `dfm-analysis-in-pcb-design` has no figures.

**Conflicts between JLC sources (flag, do not choose):**
1. Mouse-bite web and pitch: four figures.
   - Guide: 0.35–0.4 mm edge to edge, min 0.3, 5–8 holes.
   - CAP: 0.5–0.8 mm holes, 0.2–0.3 mm between them.
   - Castellated post: 1.0–1.2 mm centre to centre, 4–6 holes.
   - `pcb-mouse-bites` (a 2026 student-written post): 0.7–0.8 mm centre to
     centre with 0.5–0.6 mm holes, "practically touching", which leaves a
     0.1–0.3 mm web.

   The last looks wrong next to JLC's own 0.3 mm minimum.
2. Mouse-bite board gap: the guide says 1.6 or 2 mm, minimum 1.2. CAP says
   "Panelization with space ≥2 mm" and also "1.6 or 2 mm". `panel.py:75` uses
   1.2.
3. Solder-mask expansion:
   - `basic-design-of-solder-mask` (updated 2026-09): 1- and 2-layer boards
     get 0.05 mm per side, and 1:1 LDI applies to multilayer only.
   - CAP: 1:1 since the June 2025 LDI upgrade, with no layer restriction.
   - `pcb-solder-mask-expansion-guide`: 0.05–0.1 mm "typical".
4. Silk minimum height:
   - character-design-specifications: 0.8 mm absolute, 1.0 recommended.
   - CAP: under 1.0 mm "unidentifiable".
   - `pcb-silkscreen-design-manufacturing-guide`: an absolute 25 mil
     (0.64 mm), which is generic and wrong for JLC.
5. Recommended line width: character spec says ≥0.2 mm, CAP ≥0.15 mm.
6. Slot length: the slot-holes post says ≥2.5× width, CAP ≥2×.
7. `border-lines-3d-footprint-lines` recommends an annular ring "≥0.1 mm".
   CAP's 2-layer PTH figure is 0.25 (0.18 absolute), so the blog figure
   applies to vias at most.
8. `castellated-pcbs…` gives a 0.6 mm minimum castellated hole; CAP gives
   0.5 mm.

**Claims that looked wrong or misleading:**
- "≥15 mm" antenna keep-out on all layers (ESP32-S2 post) is not Espressif's
  figure. Take it from the module datasheet.
- `pcb-copper-balancing` says "warpage under 0.5%" and "20–30% plating
  variation" with no source.
- `pcb-design-rules-best-practices` says 2 oz copper carries 0.5 A at
  "~2 mil". That is below any fab minimum, a table artefact.
