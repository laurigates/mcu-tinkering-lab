# JLCPCB blog: PCBA, sourcing, stencils, ordering, cost

Domain agent: PCB assembly, parts sourcing, stencils, ordering and cost.
Read 2026-10-27. 64 posts fetched with `tools/jlc-blog.py` and read (fully for the top
ten, the rest grepped for figures); about 230 more triaged from title and preview
(the "PCB Assembly", "Components & Sourcing", "SMT Stencil*", "Stencil
Selection", "Solder Paste", "Hand Soldering & Stencils", "Assembly
Techniques", "Quality*", "Cost Management", "Case Studies" categories, plus
the uncategorised posts matched by keyword). The repo was not edited.

Repo baseline it was checked against: `.claude/skills/jlcpcb-order/SKILL.md`
(JO), `.claude/skills/aliexpress-parts/SKILL.md` (AE),
`.claude/skills/carrier-pcb/SKILL.md` traps 32-33 (CP), `tools/kicad/jlc.py`,
`fab.py`, `panel.py`, `docs/reference/jlcpcb-rotations.toml`,
`.claude/agents/pcb-reviewer.md` (PR), the three carrier READMEs.

## 1. Relevant articles

| # | URL | Title | Value | Target | Why |
|---|---|---|---|---|---|
| 1 | https://jlcpcb.com/blog/reduce-pcb-assembly-cost-prototype | How to Reduce Prototype PCB Assembly Cost (With Real Quotes), 2026-10-13 | high | JO § 3 step 4-7, § 4; AE "Comparing with JLCPCB PCBA" | Current Economic vs Standard fee table, the full list of what forces Standard PCBA, THT per-joint price, Confirm Parts Placement ($0.45), Global Sourcing MOQ/attrition, file-change rules |
| 2 | https://jlcpcb.com/blog/files-needed-for-pcb-assembly | Files Needed for PCB Assembly: A Checklist | high | `tools/kicad/jlc.py`, `fab.py`, JO § 1 | BOM/CPL columns, uppercase designators, no range grouping, Gerber/CPL common origin, paste layers required |
| 3 | https://jlcpcb.com/blog/mouse-bite-panelization-guide | Technical Guidance: Mouse Bite Panelization | high (already applied) | `tools/kicad/panel.py` | JLC's numbers (0.6 mm holes, 0.35-0.4 mm web, 1.6-2 mm gap, a set every 50-60 mm, <250 mm side); JLC silently widens a web <0.3 mm; won't add tooling holes/fiducials to a customer Gerber panel |
| 4 | https://jlcpcb.com/blog/fiducial-marks | Fiducial Marks in PCB and SMT Assembly | med | PR § 6; integrated README "Fiducials"; possible `kicad.boardcheck` check | 1 mm pad, mask opening ≥2× pad, copper keep-out = opening, ≥3.35 mm from edge "including clearance", 3 in asymmetric L, no silkscreen |
| 5 | https://jlcpcb.com/blog/pcb-panel-size | Get More From Every PCB Panel: Choosing the Right Size | med | `panel.py` docstring, JO § 1 | JLC process edge 5 mm, tooling hole 2 mm NPTH asymmetric, rail fiducial centre 3.85 mm from edge, routed gap 1.6-2.4 mm, V-cut array 70×70 to 475×475 mm |
| 6 | https://jlcpcb.com/blog/how-to-choose-a-solder-paste-stencil | SMT Stencil Thickness Guide 2026 | med | AE (self-assembly), integrated README "Assembly" | JLC's free thicknesses (0.10/0.12/0.13/0.15/0.18/0.20 mm) and the pitch → thickness table; frameless is enough for hand printing |
| 7 | https://jlcpcb.com/blog/stencil-aperture-design-geometry-area-ratio-tips | Stencil Aperture Design: Geometry, Area Ratio & Tips | med | PR § 6 "Exposed-pad vias and paste" | Area ratio ≥0.66, aspect ratio ≥1.5, worked example; aperture width per pitch (0.5 mm → 0.24 mm, 0.65 mm → 0.30-0.33 mm) |
| 8 | https://jlcpcb.com/blog/dfn-package-guide-footprint-soldering-stencil | DFN Package Guide: Footprint, Soldering, Stencil | med | PR § 6; integrated U8 exposed pad | EP paste 50-70 % window-pane; EP vias 1.0-1.2 mm pitch, tented or plugged so paste does not wick |
| 9 | https://jlcpcb.com/blog/how-to-solder-qfn-components-a-step-by-step-guide | How to Solder QFN Components | med | integrated README "Assembly" (MAX98357A hand build) | 0.10-0.12 mm stencil for 0.5 mm QFN, EP 50-70 %, magnification, hot plate / hot air procedure |
| 10 | https://jlcpcb.com/blog/moisture-sensitivity-level-msl-baking-guide | Moisture Sensitivity Level (MSL): Baking Guide | med | AE traps (leftover chips) | Floor-life table (MSL 3 = 168 h at ≤30 °C/60 % RH), cumulative exposure, bake tiers 125/90/40 °C and tape/reel limited to ~40 °C (~192 h) |
| 11 | https://jlcpcb.com/blog/solder-paste-storage | Solder Paste Storage | med | AE trap 6 | 2-10 °C, never freeze, shelf life ~6 months from manufacture (4-12), warm sealed ≥2 h (Indium) to 4-6 h (AIM), on-stencil life ~8 h |
| 12 | https://jlcpcb.com/blog/solder-paste-types-viscosity-mesh-size | Solder Paste Types, Viscosity and Mesh Size | med | AE trap 6 | Type 3 25-45 µm, Type 4 20-38 µm; five-ball rule (aperture ≥5× max particle: T3 ≈225 µm, T4 ≈190 µm) |
| 13 | https://jlcpcb.com/blog/smd-rework-guide | SMD Rework Guide | med | integrated README "Repair" | SAC305 iron 340-370 °C, hot air 320-400 °C by package, airflow 40-50 % (30 % for ≤0402), bottom preheat 80-150 °C, nozzle sizes |
| 14 | https://jlcpcb.com/blog/hot-air-surface-mount-soldering-step-by-step-guide | Hot Air Surface Mount Soldering | med | same | 60-120 s preheat at 3-5 cm, "silver flash", do not move 30 s; SAC305 300-330 °C at 1.5 cm |
| 15 | https://jlcpcb.com/blog/reflow-soldering-profile-explained | Reflow Soldering Profile | med | same (hot-plate / oven build) | SAC305 profile: ramp 1-3 °C/s, soak 150-200 °C 60-120 s, peak 230-250 °C, TAL 45-90 s, cool 2-4 °C/s |
| 16 | https://jlcpcb.com/blog/pcb-tombstone | What Is Tombstoning? | low-med | PR § 6 | Asymmetric pad heating causes it; 0603+ rarely tombstones; thermal-relief the plane-side pad |
| 17 | https://jlcpcb.com/blog/pin-in-paste-through-hole-reflow-stencil-guide | Pin-in-Paste: Through-Hole Reflow With a Stencil | low-med | JO § 4 (THT option) | JLC opens a THT paste aperture only if the pad is on the paste layer; stencil 0.15-0.2 mm; 75 % barrel fill target |
| 18 | https://jlcpcb.com/blog/pcb-test-points-and-smart-dft | Maximizing Circuit Reliability: PCB Test Points and Smart DFT | low-med | PR § 7/8; integrated README "Debug" | Probe-pad sizes (signal 1.0-1.27 mm, power 1.5-2.0 mm), 2.54 mm spacing, 3.2-3.8 mm from edge |
| 19 | https://jlcpcb.com/blog/jlcpcb-launches-compliant-overseas-component-consignment-service | JLCPCB Launches Compliant Overseas Component Consignment Service, 2026-08-12 | low-med | JO new "Consigned parts" note | Parts from outside mainland China go through JLC as import agent; HS code, origin, manufacturer, use declared; customs runs 2-3×/week; shipped to the HK warehouse |
| 20 | https://jlcpcb.com/blog/smt-stencil-fiducial-types-design-rules-placement | SMT Stencil Fiducial: Types, Design Rules, and Placement | low | PR § 6 | 3 fiducials needed for scaling compensation (2 give X/Y/θ only); quiet zone = fiducial diameter |
| 21 | https://jlcpcb.com/blog/diy-pcb-solder-paste-stencil | DIY PCB Solder Paste Stencil | low | AE | 0.5 mm pitch is the DIY (Kapton/Mylar) boundary; Kapton ~20-30 prints, Mylar 5-10 |
| 22 | https://jlcpcb.com/blog/step-stencil-design-complete-technical-guide | Step Stencil Design | low | none now | Steps 0.025-0.05 mm; 1.5-2 mm aperture clearance to a step edge; first try 5-20 % aperture reduction |
| 23 | https://jlcpcb.com/blog/castellated-pcbs-introduction-and-design-requirements | Castellated PCBs | low | JO § 4 (Standard trigger) | Castellation forces Standard PCBA (see #1); ≥0.6 mm hole, ≥1 mm pad to edge |
| 24 | https://jlcpcb.com/blog/pcb-x-ray-inspection-explained | PCB X-Ray Inspection Explained | low | none | QFN EP voids <30 % (Class 2) / <20 % (Class 3) |
| 25 | https://jlcpcb.com/blog/solder-paste-inspection-guide | Solder Paste Inspection | low | JO § 4 | SPI tolerances (height ±20 %, volume ±15 %, offset ≤50 µm). #1 says SPI is Standard-only, not Economic |
| 26 | https://jlcpcb.com/blog/jlcpcb-jlcone-desktop-app | JLCPCB Unveils JLCONE, 2026-08-22 | low | JO § 2, § 5 | $2 PCB and special shipping are JLCONE-only; random $1-20 checkout discount stacks with one coupon; $104 coupon pack on install |
| 27 | https://jlcpcb.com/blog/how-to-desolder-smd-components | How to Desolder SMD Components | low | rework | Hot air 300-350 °C lead-free start; low-melt removal alloy (ChipQuik 58 °C leaded / 79-91 °C lead-free) |

Also on topic, low value (generic, marketing, or outside this repo's
two-layer, 0805, top-side scope): `pcba-cost-breakdown`,
`reducing-pcb-assembly-costs`, `low-cost-and-rapid-prototype-pcb-assembly-service`,
`turnkey-pcb-assembly`, `low-volume-pcb-assembly`, `build-first-custom-pcba`,
`pcb-assembly-workflow-beginners`, `smt-process-in-pcb-assembly`,
`pcba-testing`, `the-guide-to-flying-probe-testing`, `smt-tests-in-pcb-assembly`,
`automated-optical-inspection`, `pcb-panelization`, `design-for-manufacturing-and-assembly`,
`ultimate-guide-to-qfn-package`, `mixed-technology-pcb-assembly-combining-smt-and-tht`,
`order-custom-stencils-for-pcb-assembly`, `stencil-in-pcb-fabrication-jlcpcb-solution`,
`solder-paste-layer-how-to-create-and-optimize-it`, `paste-mask-vs-solder-mask`,
`smd-stencils-in-prototyping-vs-mass-production`,
`top-5-circuit-board-stencil-mistakes-how-to-fix-them-for-higher-smt-yield`,
`the-role-of-fiducials-in-smt-stencil-alignment`, `how-to-use-solder-paste-step-by-step-guide`,
`complete-stencil-guide-lead-free-solder-paste-printing`, `low-temperature-solder-paste-sn-bi-guide`,
`how-to-solder-fine-pitch-ic-pins-using-flux`, `how-to-solder-pin-headers`,
`manual-pick-and-place-cost-effective-smt-alternative`, `smd-assembly-vs-tht-assembly-practical-guide`,
`capacitor-polarity-guide`, `smd-diode-polarity-guide`, `pcb-pricing-breakdown`,
`demystifying-conformal-coatings`, `free-assembly-1-6-layer-pcb-discount-8-20-layer-pcba`,
`jlcpcb-september-october-2026-holiday-schedule`, `in-circuit-testing`,
`bed-of-nails-test-fixture`, `pcb-encapsulation-guide`, the BGA set
(`bga-*`, `the-complete-guide-to-bga-rework`), the Case Studies (all
product showcases, no process figures).

## 2. Proposed changes

Figures are the blog's, dated as given. Each needs a quote-page check before
it goes into a skill as a calibration, as JO already says of its own numbers.

### A. Ordering and cost (jlcpcb-order)

A1. **What pushes an order out of Economic into Standard PCBA.** Any of:
components on both sides (a bottom-side THT part counts); a package below
0402; IC pitch <0.4 mm or BGA pitch <0.5 mm; >50 boards (>30 for some
thickness/colour/finish combinations); V-cut panels (Economic takes single
boards and mouse-bite panels up to 250×250 mm); thickness outside 0.8-1.6 mm
or a layer count other than 2/4/6; gold fingers, castellated holes, edge
plating. Standard also needs edge rails and fiducials, and boards under
70×70 mm must be panelised or railed. Source: #1 (citing JLCPCB's PCBA
capabilities page). Target: JO § 3 step 4, as a "stays Economic if" line.
Status: **new** (JO only says "Economic → Top Side", JO:78). Fit: skill
step, plus a cheap automated check in `tools/kicad/jlc.py`: fail if a placed
LCSC footprint name matches `0201|01005` or a pitch `_P0\.[0-3]\d*mm` under
0.4 mm. Bottom-side LCSC parts already fail, because `fab.py` exports
`--side front` and `jlc.write` raises "LCSC parts with no SMD placement".

A2. **Fee table, 2026-10-09 (USD).** Economic: setup $8.18, stencil $1.53,
$3.07 per unique Extended line (Basic and Preferred free), 1-3 day build.
Standard: setup $25.56 single / $51.12 double-sided, stencil $8.21 / $16.42,
$1.53 per unique line (Basic or Extended), 4+ days, and a handling fee so
Standard is at least $14.93 over Economic for a design that qualifies for
both. Source: #1. Status: **matches** JO:94 (€7.33 + €1.38) within the
exchange rate; JO:95's "≈€2.75 (USD 3)" is **slightly below** the blog's
$3.07. Fit: update the calibration row with date and source. On the
integrated carrier's 14 Extended lines (README:205) that is 14 × $3.07 ≈ $43.

A3. **JLCPCB can solder the through-hole parts.** $0.0164 per joint plus
$3.58 hand-soldering labour per order, about one extra build day, and no
feeder fee. The part must be listed for THT assembly in the parts library,
and shell/mounting tabs count as joints. Source: #1. Count from the committed
boards (`(pad … thru_hole` in each `.kicad_pcb`): integrated carrier 170
joints, socketed unified 247, balancebot 68. Integrated ×5 is 850 joints ≈
$13.9 + $3.58 ≈ $17.5 before the THT parts' own prices; ×2 is ≈ $9.2.
Status: **new**. JO § 4 does not list it, and `jlc.py:7-9` and the READMEs
treat sockets and connectors as hand-soldered by design. Fit: a JO § 4 lever
row, and a line in AE "Comparing with JLCPCB PCBA". Making it work needs
`fab.py` to drop `--smd-only` for THT parts that carry an LCSC number, and a
rotation entry for each. **Trap:** the socketed carrier's back-face IDC
headers (CP consumers list, "Two head IDC headers sit on the back face")
would make the order double-sided, which forces Standard, if JLCPCB solders
them. Keep those hand-soldered.

A4. **Tick "Confirm Parts Placement" ($0.45).** The placement preview on the
order page is generated and "for reference only". Assembly follows a
production file that JLCPCB engineers prepare from your files and the
silkscreen. Confirm Parts Placement lets you approve that file before
assembly starts. Source: #1, #2. Status: **new**. JO:83 treats the preview as
the final check, and nothing in the repo mentions this option (grepped).
Fit: a JO § 3 step between 6 and 7. It is the one gate after
`jlcpcb-rotations.toml` that sees JLCPCB's own rotation decisions.

A5. **Global Sourcing / not-in-stock parts.** They are bought at the
distributor's MOQ plus an attrition allowance (one example: MOQ 443 for 10
used). Leftovers stay in your private parts library for later PCBA orders
and cannot be shipped to you. The parts must arrive before assembly starts.
Source: #1. Status: **partly covered**. The integrated README:204 says
"Attrition parts come on top", but JO has no MOQ note. Fit: a JO § 4 row.
It also matters to the AE "consumed vs cash" comparison: JLCPCB's leftovers
have no resale or reuse value outside JLCPCB, which is not true of
AliExpress leftovers.

A6. **File changes after ordering.** Support can enable file replacement
until the PCB enters production. The replacement creates a new order and
refunds the old one to the account balance. After that nothing can change.
Source: #1. Status: **new**. Fit: a JO § 6 hand-off line ("freeze the zip;
the sha256 in the cart CSV is the one that will be built").

A7. **PCBA quantity counts panels, not boards**, when panelised. Source: #1.
Status: **not applicable today**. The integrated carrier is one outline in
`mode = "saw"` (board.toml:664). Keep it as a JO trap for a future
`[panel] mode = "mousebites"` order.

A8. **Standard PCBA stencil storage**: $4.93 keeps it 3 months. Economic has
no storage option, and its stencil is $1.53. Source: #1. Low; note only.

A9. **Consigning parts from outside mainland China** (e.g. a module bought
in the EU). It goes through JLCPCB's Overseas Component Consignment service:
declare the manufacturer, use, country of origin and HS code; JLCPCB imports
as agent (duty + import VAT) into the HK warehouse, clearing 2-3 times a
week, and a sale-and-purchase agreement is formed after count. Help page:
https://jlcpcb.com/help/article/how-to-consign-parts-to-jlcpcb. Source: #19.
Status: **new** (no "consign" anywhere in the repo). Fit: a short JO note.
It is not needed for the current carriers, whose modules are socketed and
hand-fitted.

A10. **JLCONE specifics.** The $2 PCB deal and the special shipping rates are
app-only since 2026-08. The app adds a random $1-20 checkout discount that
stacks with the one coupon, and a $104 coupon pack on first install. Source:
#26. Status: **consistent** with JO § 2 and § 5 (one coupon per order, the
app's special PCB price). Fit: one line in JO § 5 on the stacking random
discount. Low.

### B. Assembly files (tools/kicad/jlc.py, fab.py)

B1. **Designators uppercase and unique ignoring case; no range grouping
("R1-R8").** Source: #2. Status: ranges are **covered** (`fab.py` passes
`--ref-range-delimiter ""`, fab.py:126-127). Case is **not checked**. Fit:
a one-line automated check in `jlc.write` (fail if
`len({r.upper()}) != len(refs)`). Low risk today: refs are generated.

B2. **Gerber and CPL must share an origin.** Source: #2. Status: **covered
implicitly**. Neither the `gerbers` nor the `pos` export in `fab.py` passes
`--use-drill-file-origin`, so both use the absolute origin. Fit: a comment
in `fab.py` so nobody adds the flag to one export only.

B3. **Paste layers are what the stencil is cut from.** JLCPCB's CAM applies
its "opening-process standard" rather than copying pads 1:1, and opens a
THT pad only if it is on the paste layer. Source: #2, #17, `solder-paste-layer-*`.
Status: **covered** (`fab.py` GERBER_LAYERS includes F.Paste/B.Paste;
the vendored KiCad TQFN-16 EP already has a 4-window paste pattern). No
change.

### C. Panel, fiducials, tooling (panel.py, pcb-reviewer)

C1. **Mouse-bite numbers.** Status: **already covered** at `tools/kicad/panel.py`
docstring and constants (MIN_SLOT_MM 1.2, MIN_WEB_MM 0.3, MIN_HOLES 5,
defaults 2.0/0.6/0.95/5/50). Two details not in it: JLCPCB "will adjust it
to a minimum of 0.3 mm … without notifying the customer", and "for panel
designs provided as gerber files, if they require tooling holes or
fiducials, please add them yourself". Source: #3. Fit: one docstring line.

C2. **Fiducial edge distance: conflict to resolve, not choose.**
`fiducial-marks` (#4): "at least 3.35 mm between the board edge and the
fiducial mark, including its clearance area", which puts a 1 mm pad's centre
at ≥4.35 mm from the edge. `pcb-panel-size` (#5): rail fiducial centre 3.85
mm from the panel edge. Integrated carrier: centres 3.5 mm from the edge
(board.toml:218-222, README:170), so the 2 mm opening's edge is 2.5 mm in.
Status: **conflicts** with #4. JLCPCB's help/capabilities page should
settle it. It only matters for Standard PCBA, which requires rails and
fiducials; README:170 already says Economic does not require them at this
size. Fit: PR § 6 checklist item ("fiducial ≥ N mm from Edge.Cuts once the
figure is confirmed").

C3. **Fiducial geometry check (automatable).** 1.0 mm copper, a mask opening
≥2× the pad, a copper keep-out the size of the opening (no tracks, pour,
vias or silk), three in an asymmetric L, and a set on each side that is
assembled. Source: #4, #20. Status: geometry **covered** by the registry
footprint (`docs/reference/parts/fiducial-1mm.toml`, Fiducial_1mm_Mask2mm).
The keep-out against the GND pour is **not checked**: is the pour kept 1 mm
off each FID? Fit: an automated check in `tools/kicad/boardcheck.py` (or
`carrier-review`): no copper of another item within 1.0 mm of a FID pad's
centre, ≥3 FIDs, not collinear and not symmetric.

C4. **Tooling holes.** JLCPCB rails use a 2 mm NPTH, placed asymmetrically.
An ICT fixture wants two 3.2 mm NPTH in opposite corners. Source: #5, #18.
JO:79 picks "Added by JLCPCB". What JLCPCB adds (on the board or on a rail)
is **not stated** in any post read. Open question for the help page;
README-level only.

### D. Paste, stencil and hand assembly (aliexpress-parts, integrated README)

D1. **A stencil for self-assembly.** A frameless stencil is enough for hand
printing. JLCPCB's thickness choices at no extra cost are 0.10, 0.12, 0.13,
0.15, 0.18 and 0.20 mm. The table: 0.5 mm pitch → 0.12 mm (0.12-0.15);
0.65 mm → 0.15 mm (0.15-0.18); 0402 → 0.12 mm. For the integrated carrier
(TQFN-16 0.5 mm, TSSOP/SSOP 0.65 mm, 0805) that makes **0.12 mm** the one
foil. Stencils start at "$3" (marketing figure) with a 12 h build. Source:
#6, #9. Status: **new** (the repo has no stencil guidance; JO § 5 mentions
only a Stencil coupon category). Fit: AE "Comparing with JLCPCB PCBA" (add
the stencil to the self-assembly side of the comparison), and the integrated
README "Assembly" bullet (README:98-100).

D2. **Solder paste.** Type 3 (25-45 µm) suits >0.5 mm pitch. Type 4
(20-38 µm) suits 0.4-0.5 mm pitch: the MAX98357A's 0.24 mm apertures sit at
the five-ball limit for Type 3 (≈225 µm). Store at 2-10 °C and never freeze.
Shelf life is about 6 months **from manufacture** (4-12 by product). Warm
the sealed jar ≥2-4 h before opening. Source: #11, #12. Status: **extends**
AE trap 6 (AE:72, "ships slowly … wants refrigeration"). Fit: AE trap 6.
With 24-46 day AliExpress shipping, check the manufacture date in the
listing and buy Type 4 for the TQFN.

D3. **MSL and leftover chips.** Leftover chips (AE: "15 chips … left over")
sit in opened bags with unknown exposure. MSL 3 (common for QFN/TSSOP)
allows 168 h cumulative at ≤30 °C/60 % RH. Unknown history means bake per
J-STD-033 before reflow: in tape or reel only at about 40 °C (days, ~192 h);
125 °C only out of the carrier, if the part allows it. Check the part's MSL
on its datasheet or the LCSC page. Source: #10. Status: **new** (no MSL
anywhere in the repo). Fit: AE trap ("leftover chips are only worth their
price if their MSL floor life is kept or they are baked").

D4. **Hand-reflow and rework settings, SAC305.** Profile: ramp 1-3 °C/s,
soak 150-200 °C for 60-120 s, peak 230-250 °C, TAL 45-90 s. Hot air:
preheat 60-120 s at 3-5 cm, then 1-2 cm; 300-330 °C (blog #14) or
350-400 °C for QFN (blog #13); airflow 10-20 % (#14) or 30 % (#13) for
≤0402. Iron 340-370 °C. Bottom preheat 80-150 °C for plane-heavy boards.
Source: #13, #14, #15. Status: **new** (the repo says only "needs hot air or
JLCPCB assembly", README:99). Fit: a short "Hand assembly" subsection in the
integrated README, or a future self-assembly skill. The ranges disagree
across posts (see § 3), so quote them as ranges.

D5. **DIY stencils stop at 0.5 mm pitch.** Kapton lasts about 20-30 prints,
Mylar 5-10. Source: #21. Fit: AE note ("a cut-film stencil will not do the
TQFN; buy steel").

### E. Layout review (pcb-reviewer)

E1. **Exposed-pad via wicking.** EP vias should be tented or plugged so
paste does not wick through during reflow (#8). On 2 layers the via in
U8's 1.23 mm EP (`[fanout] in_pad = ["U8:17"]`, board.toml:627; 0.6/0.3 mm)
can only be tented from B.Cu, and JLCPCB's via-covering order option decides
that. Status: **partly covered**. `route_board.py:404-410` forbids vias in
pads except `in_pad` because "the hole drains the pad's paste", but says
nothing about the allowed one, and PR § 6 lists "Exposed-pad vias and paste"
without a criterion. Fit: PR § 6 item. Check the EP via is tented on
B.Mask (or the order's via covering is "tented"), and that the EP paste
stays a window-pane of 50-70 % coverage.

E2. **Tombstoning from asymmetric pad heating.** A pad on the plane heats
later than a pad on a thin track. Thermal-relief the plane side and keep
both pads equal. 0603 and up rarely tombstone. Source: #16. Status:
**covered**. Pours use `ZONE_CONNECTION_THERMAL` (route_board.py:64) and
the passives are 0805. No change, apart from an optional PR note for any
future 0402.

E3. **Bring-up probe pads.** A flat mask-open pad of 1.0-1.27 mm (signal) or
1.5-2.0 mm (power), on 2.54 mm spacing where possible, ≥0.5-1.0 mm from part
bodies. Source: #18. Status: the integrated README:102-104 records "no test
point on the motor-driver inputs or the I2S lines" as a known gap. Fit: PR
§ 7/8 item. Low.

## 3. Noise notes

- About 65-70 % of the posts read are generic or marketing. Every stencil
  post carries the same "$3, ±0.003 mm, 12 h, $11 coupon" block, and several
  cost posts restate "use our parts library". The posts with original,
  current, JLCPCB-specific figures: #1 (by far the best, with real quotes and
  dated fees), #2, #3, #5, #6, #19. The Case Studies hold no process data.
- The "60-70 % of SMT defects come from paste printing" claim is repeated in
  six posts with no source.
- Reflow and hand-solder figures disagree from post to post. TAL: 45-90 s
  (#15, #16) vs 45-75 s (`smd-assembly-vs-tht-*`). Cooling: 2-4 °C/s (#15) vs
  3-6 °C/s (same). SAC305 peak: 230-250 / 235-245 / 240-250 °C. Hot-air
  temperature: 300-330 °C (#14) vs 320-400 °C (#13). Paste warm-up: 2 h /
  3-4 h / 4-8 h. Quote ranges and the paste datasheet, not one post.
- `fiducial-marks` says FR-4 distortion of "100 ppm" gives "a 0.01 mm error
  at a component far from the global fiducials". 100 ppm over 100 mm is
  0.01 mm, which is negligible, so the argument does not support the
  local-fiducial point it makes. It also says "a minimum of two, but three
  strongly recommended", while `the-role-of-fiducials-*` says the edge
  distance "shall not be greater than the clearance + 7.62 mm". The posts
  give different edge rules (see C2).
- `pcb-pricing-breakdown`: "All-SMD assembly can cost up to ~50% more
  through-hole assembly" is garbled and contradicts #1's per-joint figures.
  Ignore it.
- `low-cost-and-rapid-prototype-*` advertises "Standard AOI and X-ray
  Inspection" and "24-hour turnaround" for prototypes. #1 says Economic has
  no SPI and a 1-3 day build, so the generic posts overstate what Economic
  includes.
- `pcba-cost-breakdown` and `files-needed-*` advise against consignment
  ("turnkey is generally more cost-effective"). That is JLCPCB's commercial
  position, not data.
- The JLC-specific fees in #1 are dated 2026-10-09/13 and say "prices vary
  slightly with exchange rates". JO's own observations (2026-10-22) agree
  within about 10 %.
