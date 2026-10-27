# JLC3DP blog → enclosure / printed-mechanism improvements

Scope: 3D-printed carrier cases (`tools/enclosure`, `enclosure-cad` skill) and servo
mechanisms (`tools/mechanism`, `printed-mechanism` skill, pan-tilt `print/`) ordered at
JLC3DP. 54 posts fetched (re-fetch: `tools/jlc-blog.py fetch <slug> --site 3dp`), from the 419-post index; the jlcpcb.com index had
nothing on enclosures beyond EasyEDA/marketing. Repo read-only; line refs are to HEAD.

Repo numbers the findings are checked against:
- `tools/enclosure/enclosure/case.py`: `MIN_WALL = 2.0` (l.27); `SCREWS` M3 pilot 2.5 / M2
  pilot 1.6 (l.33–35); `CHASSIS_CLEARANCE`/`LID_CLEARANCE` 3.4 (l.37–38); `SKIN = MIN_WALL`
  (l.40); `MIN_THREAD = 4.0` (l.41); `Params.gap = 0.5` board-to-wall (l.48); lid lip
  slide fit `t = 0.3` (l.311). No fillets anywhere in case.py.
- `tools/mechanism/mechanism/check.py`: clash = intersection volume > `CLASH_TOL_MM3 = 0.01`
  (l.43, l.102). It has **no minimum-clearance (distance) check**.
- pan-tilt `print/design.py`: `FIT 0.3`, `FIT_END 0.35`, `GAP = 0.5` "running clearance
  between parts that move" (l.132), `KEY_FIT 0.3`, `PILOT 1.6`, `INSERT_HOLE = 3.2` for an
  M2×3 insert OD 3.5 (l.167), `TB_POCKET_CRUSH 0.15`, crush ribs 0.1 mm; no fillets.
- Skills: `enclosure-cad` trap 4 (l.68–72: JLC upload check "asks only for over 1.2 mm");
  `printed-mechanism` checklist 6/8 (l.72–80: "MJF ±0.3 mm", "MJF holds about 1.0 mm") and
  "Materials at JLC3DP" (l.82–92). No skill mentions heat-deflection temperature, UV/creep,
  escape holes, emboss limits, fillets or JLC's threaded-insert service.

## 1. Relevant articles (ranked)

| # | URL | Title | Value | Target | Why |
|---|---|---|---|---|---|
| 1 | https://jlc3dp.com/blog/sls-vs-mjf | SLS vs MJF | high | printed-mechanism "Materials"/checklist; mechanism check.py | Only post carrying JLC's own MJF/SLS rule table: wall 1.0, emboss/engrave 0.5×0.5 (MJF) / 0.8×0.8 (SLS), assembled clearance 0.2–0.4, **moving clearance 0.6**, escape hole ≥2.5, hole ≥1.5, column ≥2.0, min part 5×5×5, max build sizes |
| 2 | https://jlc3dp.com/blog/3d-printed-articulated-animals-design | 3D Printed Articulated Animals | high | mechanism check.py; checklist | Quotes JLC guideline "min clearance between moving parts": SLA 0.5, MJF 0.6, SLS 0.6, FDM 0.5, SLM/BJ 1.0; "check clearance over the whole motion path"; "never scale functional gaps" |
| 3 | https://jlc3dp.com/blog/mjf-tolerances-and-dimensional-accuracy | MJF Tolerances & Dimensional Accuracy | high | printed-mechanism checklist 6–8 | ±0.2 mm or ±0.2 % (greater); per-feature table; holes <5 mm print 0.2–0.3 undersize; press fits "not reliable as-printed", Ra 10–15 µm; walls <1.5 mm ±0.3 or worse; bosses >5 mm need base fillets |
| 4 | https://jlc3dp.com/blog/threaded-inserts-3d-printing | Threaded Inserts: MJF & SLA Guide | high | enclosure-cad design + trap; pan-tilt insert hole | JLC **Threaded Insert Service** (8 types, M3/M4/M5 brass); insert holes per process (M3 MJF 4.2–4.3, SLA 4.1–4.2); boss M3 6 / M4 8 / M5 10 mm; resin: press-fit/adhesive not heat; printed threads M2–M2.5 "too small", M4+ reliable; iron 180–220 °C |
| 5 | https://jlc3dp.com/blog/design-for-additive-manufacturing | DfAM guide | high | both skills; checklist | Process table (wall, accuracy, min hole, clearance, overhang, Ra); fillet internal corners 0.5–1 mm; escape holes 3–5 mm; cost scales with Z-height; validate with a coupon |
| 6 | https://jlc3dp.com/blog/3d-print-wall-thickness | Appropriate Wall Thickness | high | enclosure-cad trap 4; printed-mechanism checklist 8 | JLC recommended walls: SLA 0.8, MJF/SLS 1.0, FDM 1.2, SLM 1.5; at most 0.2 under, non-critical only |
| 7 | https://jlc3dp.com/blog/repair-stl-files | Repair STL files | high | enclosure-cad trap 4; jlcpcb-order §7 | JLC automated screen: **wall >1.2 mm and no feature <0.8 mm**, then human engineer review in 4–6 business h before payment; STL unitless (25.4× error); powder escape not flagged by repair tools |
| 8 | https://jlc3dp.com/blog/mjf-for-production-parts | MJF for Production Parts | high | printed-mechanism Materials; snap-fit checklist | Snap-fit catch 0.5–1.0 mm, beams 1.0–2.0, living hinge 0.3–0.5 (PA12 only, not GB), ribs 1.0–1.2, inside radii ≥0.5; enclosure wall 1.5–2.5; PA12 UV: coat beyond 12–24 months outdoors |
| 9 | https://jlc3dp.com/blog/3d-printing-design-considerations | Common 3D Printing Problems | high | enclosure-cad (resin cases) | JLC case notes: resin wall >1.5, **resin holes >1.5, shrink ~0.3**; nylon >1 mm, length:thickness ≤10:1, pillar D:H 1:3; FDM ≥1.6 (≥2.4 large); metal >1.5 |
| 10 | https://jlc3dp.com/blog/pa12-vs-pa12s | PA12 vs PA12S | med-high | printed-mechanism Materials | PA12 elong. 15–25 % vs PA12S 6–10 %; modulus 1.6–1.8 vs 2.0–2.3 GPa; Ra 10–12 vs 6–8 µm; PA12S dyes evenly, creeps less, holds holes nearer nominal; PA12S worse for snap/flex |
| 11 | https://jlc3dp.com/blog/mjf | MJF guide | med-high | printed-mechanism Materials | HDT table PA12/PA12S/PA11 ~175 °C @0.45 MPa, ~100 °C high load; TPU 75; PP 100; PA11 for snap-fits/hinges; PP for living hinges |
| 12 | https://jlc3dp.com/blog/engineering-resin-9000he | 9000HE resin | med-high | enclosure-cad (material choice near heat) | 9000HE HDT 51–60 °C @0.45 MPa; Izod 36–55 J/m vs standard 10–20; std resin elong. 1–3 %; post-cure drift can shrink a bore 0.3 mm |
| 13 | https://jlc3dp.com/blog/abs-like-vs-standard-resin | ABS-like vs Standard Resin | med | enclosure-cad material | HDT standard 50–60 °C, ABS-like (CBY) 55–65 °C; elong. 5–15 vs 20–50 %; JLC Black Resin HDT 58–69 °C; walls ≥~1 mm in any resin |
| 14 | https://jlc3dp.com/blog/jlc-black-resin | JLC Black Resin | med | enclosure-cad material | Black resin HDT 65 °C, no dyeing needed, low reflection (camera mounts) |
| 15 | https://jlc3dp.com/blog/drastic-material-price-cuts-and-adjusted-post-processing-service-fees | Pricing strategy Aug 2025 | med | printed-mechanism price line; pan-tilt README | Source of MJF PA12 $0.35→**$0.275/g**; SLM 316L $0.24→0.21/g; black-resin sanding +15–20 % |
| 16 | https://jlc3dp.com/blog/materials-finishing-pricing-update-july2026 | Pricing update Jul 2026 | med | jlcpcb-order §7; printed-mechanism price | PAC-HP −70 %, TC4 −47 %, PA12-CF −24 %, ABS −9 %; spray painting +16.7 %, oil spraying +14.3 % |
| 17 | https://jlc3dp.com/blog/threaded-inserts-3d-prints | Threaded inserts for PCB housings (2024) | med | enclosure-cad | Resin: M3 insert OD 5 → 5.0 mm hole, press in cold with CA; PLA 4.8 mm heat-set |
| 18 | https://jlc3dp.com/blog/glass-filled-nylon-3d-printing-guide | Glass-filled nylon | med | printed-mechanism Materials | GF: creep-resistant, bolted joints stay tight; no snap-fits/living hinges; dyes blotchy |
| 19 | https://jlc3dp.com/blog/3d-metal-printing-design-rules | Metal design rules | med | printed-mechanism (SLM brow upgrade line) | SLM wall 0.4, feature 0.6, overhang 50°; BJ wall 1.0, feature 2.0 |
| 20 | https://jlc3dp.com/blog/3mf-vs-stl | 3MF vs STL | low-med | enclosure-cad / jlcpcb-order §7 | STL unitless, 3MF explicit units |
| 21 | https://jlc3dp.com/blog/what-is-nylon | What is nylon | low-med | printed-mechanism Materials | PA12 low moisture vs PA6/66; "parts can drift enough to fail an assembly" for high-absorption grades |
| 22 | https://jlc3dp.com/blog/split-large-prints | Split large prints | low-med | enclosure-cad (multi-piece) | Alignment pin/key gap 0.1–0.2 mm (FDM) |
| 23 | https://jlc3dp.com/blog/strongest-resin-for-3d-printing | Strongest resin | low-med | enclosure-cad | 9600 for brackets/housings, 8228 rigid fixtures; tough/ABS-like for snap-fits; over-cure → brittle |
| 24 | https://jlc3dp.com/blog/mjf-nylon-strength-stiffness-surface-finish | MJF nylon strength/stiffness/finish | low-med | printed-mechanism | Wall <1.5 "risky for structural", ≥2.5 "safe baseline"; dye after finishing; blasting/tumbling soften features |
| 25 | https://jlc3dp.com/blog/resin-yellowing | Resin yellowing | low-med | enclosure-cad (white 9600 cases) | UV/oxidation yellows resin; JLC lists 8228, 9000R, LEDO 6060 as low-yellowing |
| 26 | https://jlc3dp.com/blog/easyeda-stl-export | EasyEDA STL export | low | enclosure-cad | EasyEDA case generator recommends shell ≥1.5 mm |
| 27 | https://jlc3dp.com/blog/white-vs-black-nylon | White vs black nylon | low | enclosure/mechanism colour | Black: better UV, absorbs more heat; white translucent (LED bleed) |

Also on topic, low value (generic, FDM-hobby or marketing): `comprehensive-guide-3d-printing-tolerances`,
`overhangs-tips`, `3d-printing-supports-guide`, `modeling-tips`, `3d-print-electronic-enclosures`,
`pcb-enclosure-printing`, `assemble-parts`, `considerations-for-3d-printing-threads`, `3d-printed-gear-guide`,
`3d-printed-springs-manufacturing`, `heat-resistant-3d-printing-plastics`, `reduce-3d-printing-cost`,
`estimate-3d-printing-cost`, `make-watertight-prints`, `3d-print-spray-paint-guide`,
`3d-print-nylon-post-processing`, `3d-print-post-processing-overview`, `building-3d-printed-robot`,
`jlc3dp-retsetman-mech-design`, `3d-printed-lego-key-considerations`, `elongation-at-break`,
`best-outdoor-3d-printing-materials`, `transparent-resin`, `fdm-vs-sla-vs-sls`, `3201pa-f-nylon-sls`,
`add-color-text` (FDM emboss 0.4–0.6 mm only), `resin-9600-vs-8111x` (its TDS table is an image; read
JLC's 9600 help page for HDT instead).

## 2. Proposed changes

### A. Gates (tools/mechanism, tools/enclosure)

A1. **Minimum running clearance over the sweep** — new gate in `tools/mechanism/mechanism/check.py`.
- Rule: parts that move relative to each other need ≥0.6 mm (MJF/SLS), ≥0.5 mm (SLA, FDM),
  ≥1.0 mm (SLM/BJ) per JLC's guideline as quoted in [2] and [1] ("between two moving or
  connecting parts"); assembled static fits 0.2–0.4 mm MJF [1]. [2] also: "check clearance
  around the entire moving path", which is exactly the sweep.
- Status: **new**. `check.py` only fails on intersection volume > `CLASH_TOL_MM3` (l.43, l.102),
  so a 0.05 mm near-miss over travel passes. Pan-tilt `GAP = 0.5` (design.py l.132) is under
  the 0.6 MJF figure → **conflicts with the blog figure**, flagged not resolved: the 0.6 is
  JLC's print-in-place number (parts printed together); separately printed, assembled parts
  may only need the assembled-fit value plus ±0.2 mm/part [3]. Decide which applies.
- Fit: gate parameter, e.g. `MIN_RUNNING_CLEARANCE_MM` (per process, default 0.5) measured
  with BRepExtrema distance between bodies of different groups at each pose, reported in
  `check.json` as `near_misses` (pair, pose, distance); `allow` excuses it like a clash.

A2. **Process-aware wall floor** in `tools/enclosure/enclosure/thickness.py` callers.
- Rule: JLC recommended minimum walls SLA 0.8, MJF/SLS 1.0, FDM 1.2, SLM 1.5, "at most 0.2 under,
  non-critical only" [6]; JLC upload screen rejects <1.2 mm wall / <0.8 mm feature [7].
- Status: `MIN_WALL = 2.0` (case.py l.27) is stricter than all; covered for cases. For mechanisms
  the 1.0 mm MJF floor is prose only (printed-mechanism l.77–80; pan-tilt README "Walls").
- Fit: a `--min-wall` / `design.py MIN_WALL` override for `mechanism::check` (1.0 for MJF), so the
  1.0 mm readings are gated rather than just printed. Note [3] says MJF walls <1.5 mm hold only
  ±0.3 mm "or worse" and [8]/[24] call <1.5 risky for structure; the pan-tilt 1.0 mm tab webs
  are already on the coupon (README "coupon.stl") — keep that.

A3. **JLC upload-screen pre-check** (warn, not fail) in `tools/enclosure` `report.json`.
- Rule: any reading <1.2 mm wall or feature <0.8 mm will trip JLC's automated screen and wait
  4–6 business h for an engineer [7].
- Status: trap 4 (enclosure-cad l.68–72) already says the flag is a heuristic and quotes 1.2 mm;
  the 0.8 mm feature figure and the human-review delay are **new**.
- Fit: checklist line in jlcpcb-order §7 ("expect engineer review before payment for MJF parts
  with 1.0 mm webs; say so in the product note") plus an optional `jlc_screen_warnings` list in
  `check.json`/`report.json`.

A4. **Powder escape check** for MJF/SLS parts.
- Rule: enclosed cavities need an escape hole ≥2.5 mm (JLC MJF/SLS table [1]); DfAM says 3–5 mm
  [5]. Repair tools don't flag it [7].
- Status: pan-tilt README asserts "every part is an open shell, so powder clears" (order sheet) —
  asserted, not checked. **New** as a gate.
- Fit: in `check.py`, count shells/voids of each printed part (a solid with >1 shell = internal
  void) and fail if any for process MJF/SLS/SLA.

### B. printed-mechanism skill

B1. **Tolerance figure** — checklist item 6 (l.72–74) says "MJF ±0.3 mm twice". JLC states
±0.2 mm or ±0.2 % of length, whichever greater; ±0.15–0.30 through holes, ±0.20–0.40 blind holes,
±0.20–0.35 bosses, ±0.40+ long slender features [3]; [8] "±0.2 mm + 0.2 %". Status: roughly
consistent; refine the item to cite the per-feature table and source. Skill text edit.

B2. **Holes print undersize** — holes <5 mm come out 0.2–0.3 mm small (>60 % of JLC holes);
oversize functional holes 0.2–0.3 mm or leave 0.5 mm for reaming [3]. Status: pan-tilt drills
Ø1.6 and taps (README BOM) and sands the Ø8.25 pin (checklist 7) — practice covered, the rule is
**new** in the skill. Also MJF min hole 1.5 mm, min column 2.0 mm [1]. Checklist item.

B3. **Press fits / crush ribs** — "Press-fit features: not reliable as-printed"; Ra ≈ 10–15 µm
"can overwhelm small interference values"; add 0.1 mm extra for sliding fits [3].
Status: **conflicts in spirit** with pan-tilt design (crush ribs 0.1 mm on the horn, 0.15 mm/side
on MR128ZZ and 6705ZZ pockets; design.py l.150, README review table). The repo already hedges
(Loctite 638 fallback; coupon). Add to checklist 6/8: "crush ribs ≤0.15 mm are inside MJF's
±0.2 mm and surface texture; prove on the coupon, keep the adhesive fallback".

B4. **Snap fits / living hinges** — PA12 snap beams 1.0–2.0 mm, catch 0.5–1.0 mm, radius at the
root, inside radii ≥0.5, living hinges 0.3–0.5 mm PA12 only (not PA12 GB) [8]; PA11 for snap
fits/hinges, PP for living hinges [11]; PA12S elong. 6–10 % "worse for snap/flex" [10]; glass-
filled: no snap fits [18]; resin snap fits need tough/ABS-like resin (elong. 20–50 %) [13][23].
Status: Materials (l.84–87) says PA11 for flex, resin cracks — consistent; numbers **new**.
**Conflict inside the blog**: [3] "snap features: low repeatability, avoid high-cycle" vs [8]
"thousands of flex cycles". Flag; keep the repo's PA11 rule. Skill Materials + checklist item.

B5. **PA12 vs PA12S** — teuvo-face entry specs PA12S-HP (entries/teuvo-face/README l.46–49) for
parts that self-tap and clamp; PA12S is stiffer, 6–10 % elongation, holds holes nearer nominal,
dyes more evenly [10]. Add one line to Materials: "PA12S for cosmetic/dyed/flat parts; PA12 where
screws self-tap, ribs crush or parts flex". New.

B6. **Fillets** — internal corners 0.5–1 mm [5], ≥0.5 mm at hinge/snap roots [8]; MJF bosses
>5 mm tall: ~15 % need rework without base fillets/smooth transitions [3]. Status: **new**; no
`fillet` in pan-tilt design.py or case.py. Checklist item ("fillet boss roots and rib roots ≥0.5
mm"), with the caveat that fillets change the wall gate's readings.

B7. **Heat** — MJF PA12/PA12S/PA11 HDT ~175 °C @0.45 MPa, ~100 °C high load [11]; resin
50–65 °C [12][13][14]. **Conflict**: [10] gives PA12 "HDT @0.45 MPa ~95–100 °C" — that is HP's
1.82 MPa value; flag, quote [11]. Status: new (no HDT anywhere in skills). Materials line: "any
part touching a servo case, motor, stepper or regulator: MJF, not resin".

B8. **Price line** — $0.275/g MJF PA12 (l.89–90, "an agent's estimate") is JLC's published rate
from 13 Aug 2025 (was $0.35) [15]; Jul 2026 cuts elsewhere (PAC-HP −70 %), spray painting +16.7 %
[16]. Status: covered; replace "agent's estimate" with the source and date. Skill text.

B9. **Metal upgrade line** (pan-tilt brows in SLM AlSi10Mg): SLM wall ≥0.4, feature ≥0.6,
overhang ≤50° from vertical needs supports [19]; JLC SLM moving clearance 1.0 mm [2]; SLM wall
1.5 mm "recommended" [6] and ">1.5 mm" [9] — **conflict** between 0.4 [19] and 1.5 [6][9];
flag. Brows read 1.2 mm in the gate (README "Walls") → under JLC's 1.5 for SLM. Note in the
pan-tilt order sheet's upgrade row. New.

B10. **"Never scale functional geometry"** — scaling a design shrinks gaps below process limits
[2]. Checklist item for designs that are parameterised by board size. New, low.

### C. enclosure-cad skill

C1. **Resin case heat** — 9600/standard resin HDT ~50–60 °C @0.45 MPa (9000HE 51–60 [12];
standard 50–60, ABS-like 55–65 [13]; JLC Black 58–69/65 [13][14]). Status: **new**; no HDT in
the skill, and enclosure.toml has no material field. Balancebot carrier carries Pololu stepper-
driver breakouts (trap 2) and the robocar carriers carry regulators; a heatsink or a stepper can
exceed 60 °C. Fit: trap ("resin cases soften at 50–65 °C; keep a gap and vents over driver
heatsinks and regulators, or order MJF"), plus a "measure the hottest part under load" line.

C2. **Inserts** — enclosure-cad l.31–32 "Resin takes a thread; it does not take a heat-set
insert": **consistent** with [4] ("heat can cause cracking"), but [4] and [17] give the resin
alternative (press-fit or epoxy/CA-bonded insert; [17]: M3 OD 5 insert in a 5.0 mm hole with CA;
[4]: SLA M3 hole 4.1–4.2), and JLC now sells a **Threaded Insert Service** (M3/M4/M5 brass,
8 types) [4]. Fit: add to "The design" as an option; check JLC's help page
(`/help/article/threaded-insert-service`) for which materials it covers before relying on it.
Note conflict: [4] SLA M3 hole 4.1–4.2 vs [17] 5.0 (different insert ODs; the post doesn't say).

C3. **Self-tap pilots in resin** — `SCREWS` M2 pilot 1.6 mm (case.py l.35) is just above JLC's
resin minimum hole 1.5 mm with ~0.3 mm shrink [9]; blind small holes can also retain uncured
resin. Status: **new risk**, the M3 2.5 mm pilot is fine. Fit: trap / or raise M2 pilot to the
tap-drill + 0.2 compensation; verify on a coupon.

C4. **Printed threads** — M2–M2.5 "usually too small", M3 borderline, M4+ reliable [4]; MJF/SLS
min thread pitch 0.6 mm [1]. Status: repo taps or self-taps, never prints threads — consistent.
Checklist line only ("never model M2/M3 threads; pilot and tap").

C5. **Lid/lip fit** — case.py lip slide fit `t = 0.3` (l.311) vs JLC SLA clearance 0.2 [5],
MJF assembled 0.2–0.4 [1], resin hole shrink ~0.3 [9]. Status: within range; no change, cite
source in the comment.

C6. **UV / yellowing** — white 9600 resin yellows under UV/oxidation [25]; black better for UV,
but runs hotter in sun [27]; PA12 outdoors: coat beyond 12–24 months [8]. Status: new, low.
Trap only if a case goes on the robot outdoors.

C7. **Emboss / engrave text** — MJF min 0.5 mm deep × 0.5 wide, SLS 0.8 × 0.8 [1]; FDM 0.4–0.6
[add-color-text]. Status: no text in cases today; checklist line for when labels are added.

### D. jlcpcb-order §7 (JLC3DP ordering)

D1. Upload screen + engineer review: wall >1.2 mm, feature ≥0.8 mm, engineer reply in 4–6
business hours, before payment [7]. Covered partly (l.24–25); add the 0.8 mm and the delay. New.
D2. Units: STL is unitless; the existing volume check (l.17–18, quoted volume = `report.json`)
already catches a 25.4× scale error — say so in one clause. Covered.
D3. Process limits worth one line: min part 5×5×5 mm; MJF PA12 max 370×276×360 mm, SLS 400×350×350
[1]. New, low.
D4. Finishing prices: spray painting +16.7 %, oil spraying +14.3 % (Jul 2026) [16]; black-resin
sanding +15–20 % (Aug 2025) [15]; JLC Black Resin needs no dye [14]. Low.
D5. MJF cost is volume × rate with a $1 part minimum [15] and [3] ("MJF parts start from $1");
DfAM: print time/cost scales with Z height [5] — for SLA cases, orient low. Low.

## 3. Noise notes

- About 60 % of the fetched text is generic or marketing (CTA blocks, coupon lines, repeated
  keyword phrases). Posts written for the JLC3DP site's own process (sls-vs-mjf table, wall-
  thickness, design-considerations, repair-stl, threaded-inserts 2026, mjf-tolerances) carry
  nearly all the usable numbers; the 2023–2024 "guides" (tolerances, enclosures, threads,
  assemble-parts, modeling-tips) carry almost none.
- Contradictions to flag rather than resolve:
  - MJF min wall: 1.0 mm [1][6] vs 0.5 mm [5]; SLA 0.8 [6] vs 0.5 [5] vs 1.5 [9]; SLM 1.5
    [6][9] vs 0.4 [19]. Use JLC's guideline page; the repo's 1.0 MJF citation (pan-tilt README
    "Walls") matches [1][6].
  - MJF clearance: 0.2 mm [5] vs 0.2–0.4 assembled / 0.6 moving [1][2].
  - PA12 HDT @0.45 MPa: ~175 °C [11] vs ~95–100 °C [10] ([10] appears to give the 1.82 MPa value).
  - MJF snap fits: "avoid high-cycle" [3] vs "thousands of cycles" [8].
  - 9000HE tensile: 41–61 MPa in body vs "65–75 MPa" in its own FAQ [12].
  - JLC Black resin HDT: 65 °C [14] vs 58–69 °C [13].
  - Insert boss: "wall ≥1.5× insert diameter" would give an M3 boss ≥ ~17 mm, yet the same
    post's table says 6 mm [4]; the rule is mis-stated (probably boss OD ≈ 1.5–2× insert OD).
  - SLA M3 insert hole 4.1–4.2 [4] vs 5.0 [17] (insert OD not stated in [4]).
- Unverifiable "internal data" claims ([3]: ">60 % of holes below 5 mm", "15 % of bosses",
  "20 % of large flat parts warp") read as marketing; the direction is plausible, the
  percentages are not citable.
- Several images are AI-generated (labelled in [4]); the 9600 vs 8111X comparison is image-only.
