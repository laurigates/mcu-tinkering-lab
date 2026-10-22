# PRD-013: Rapid Hardware Prototyping Before a Fab Order

**Status**: draft
**Created**: 2026-10-22
**Confidence**: 6/10

---

## Overview

This PRD covers the step between a carrier design that passes its checks and a
PCB order. In that step a prototype is built by hand to confirm the circuit
works. The repo already lays a carrier out on perfboard from the same netlist as
the PCB (`tools/kicad/perfboard/`, [ADR-027](../decisions/ADR-027-kicad-carrier-generation.md)).
This PRD records which prototyping methods are worth adding for that step, and
which ones were evaluated and dropped.

The available equipment is an Epson EcoTank ET-3750 inkjet printer and a
Silhouette Portrait 2 craft cutter. The bench is in Helsinki, which sets the fab
lead times below.

## Problem

The carriers in this repo are module carriers. A Seeed XIAO, Pololu DRV8825
breakouts, a GY-521 and an LM2596 module plug into 2.54 mm headers, and the
power nets carry a few amps. The balancebot perfboard has 19 parts, 25 nets and
about 60 wires on an 18 × 24-hole board.

Fabricating the board is not what makes this slow. Wiring is: placing about 60
wires correctly, and finding the one that is wrong before power is applied. A
JLCPCB order costs about €30 and arrives in 6–8 days by DHL, or about €8 and
2–4 weeks by the untracked service (estimates; JLCPCB shows prices only in its
calculator). A prototype method has to beat that on time or effort, and still
carry motor current.

## Goals

- Make the hand-wired prototype faster to build and harder to get wrong, from
  the files the perfboard generator already writes
- Catch a wiring fault before first power-up, with a checklist generated from
  the netlist rather than written by hand
- Run one cheap physical test of knife-cut copper foil before investing in
  tooling for it
- Keep each method's evaluation, with its sources, so it is not repeated

## Non-Goals

- No home fabrication of double-sided boards or of plated through-holes
- No CNC mill or fiber laser purchase
- No change to the carrier PCB pipeline itself. Foil boards would need their
  own single-layer layout, not a transformed carrier board (see Findings)

## Findings

Four parallel research passes evaluated a brainstorm of ideas. Three searched
the web: Silhouette methods, print-based methods, and fabs and Helsinki
makerspaces. The fourth read this repo's pipeline. Figures marked *est.* are
estimates that no source confirmed.

### Ranked

| Rank | Method | Verdict | Effort / cost | Main risk |
|---|---|---|---|---|
| 1 | 1:1 printable perfboard drawing with corner marks, printed on the ET-3750 | Do (FR-020) | Small code change; a sheet of paper | Printer scaling; a printed 50 mm bar checks it |
| 2 | Continuity checklist from the netlist, per build step, plus rail isolation pairs | Do (FR-021) | Small code change | A bulk capacitor reads as a short for a moment |
| 3 | Knife-cut single-layer copper foil, one test board first | Experiment (FR-022) | €1–3 foil per board; about 1–1.5 h per board (est.) | Foil lifts at headers; through-hole parts have no holes |
| 4 | Stripboard mode for the perfboard generator, with a breadboard preset | Plan (FR-023) | Medium–large: a new router | Strip assignment is a different routing problem |
| 5 | Aisler Blitz (Germany) as the fab when speed matters | Get one quote | About €30 for 5 boards (est.), 2 working days + UPS | Unconfirmed; may beat JLCPCB + DHL by 1–3 days |

### Marginal

| Method | Why it is marginal |
|---|---|
| Etching through an inkjet-printed UV mask with dry-film photoresist | The ET-3750's black (Epson 102) is pigment. Makers report that one pass is not UV-opaque, and dry film is negative-working, so the whole background prints solid black. Hand-drilling about 100 holes is the slow step. Try a test strip before buying the rest |
| Vinyl (Oramask 813) etch mask cut on the Portrait 2 | Gives the most robust home result (copper bonded, holes drillable), at 2–3 h per board with etching and drilling |
| Self-fluxing enamelled wire or a Verowire pen for signal nets | No stripping and daisy-chained nets, but the enamel needs 400–480 °C and gives off TDI fumes. Too thin for power nets |
| SMD-grid and ground-plane proto boards (BusBoard SMT, about €12) | Help only with SMD passives. A plane carries no amps without bus wire |
| Mylar paste stencil cut on the Portrait 2 | 0.5 mm pitch is credible with multi-pass cuts and pads shrunk by about 2 mil. Only matters for SMD-heavy boards, not module carriers |

### Dropped

| Method | Why |
|---|---|
| Multi-layer copper foil + Kapton stack with solder vias | Registration across 3–4 laminated layers, vias on adhesive that softens. No report of anyone doing it with a craft cutter |
| Paint-marker etch resist plotted by the cutter | The 0.8 mm marker tip is the minimum trace, and filled areas get pinholes. Vinyl resist beats it on every count |
| Wire-wrap | Standard header pins are too short to wrap, and 30 AWG Kynar cannot carry the power nets |
| Conductive ink on perfboard | High resistance, not solderable |
| Home CNC (3018) or fiber laser | Single-sided, unplated holes, no solder mask. Weeks of tuning, or €1,000+ |
| OSH Park | Ships from the USA; the slowest option to Finland |
| Converting the routed carrier boards to foil cut files | The carriers are two-layer with GND poured on both sides, 124–153 vias and 0.25–0.3 mm signal tracks. Foil needs ≥ 0.8 mm tracks on one layer |

### Helsinki access

Aalto Fablab is the only confirmed PCB mill nearby (Roland SRM-20). It is open
to the public on open days only, and a machine permit needs an induction. Oodi's
Urban Workshop has a laser cutter for wood and plastics, but no mill. Helsinki
Hacklab's tools are for members only, and no current machine list was found.

### Corrections to the brainstorm

| The brainstorm said | What the sources say |
|---|---|
| Portrait 2 has a 210 gf carriage | 210 gf is the published figure for the Portrait 3 and 4; none was found for the Portrait 2. Copper foil needs about 70–90 g |
| `gerber2graphtec --border … > /dev/usb/lp0` | The tool is now SMTCut and has had no commits since January 2020. It has no `--border` option, and `/dev/usb/lp0` exists only on Linux. On macOS, `inkscape-silhouette` lists the Portrait 2 as confirmed working and is maintained |
| "Linux CLI toolchain" | Both tools document macOS |
| Export `F.Paste` with −10 % clearance | The SMTCut author shrinks pads by about 2 mil, an absolute amount |
| 3M 1181 rated to 130 °C continuous | Not in the datasheet read; unverified |
| Verowire 38–30 AWG | Vero lists 34 AWG only |
| Insulation vaporizes at 380–400 °C | 400 °C or more; 400–480 °C on an older Farnell sheet |
| ET-3750 black is UV-opaque | It is pigment; single-pass opacity is contradicted by maker reports |

### Sources

- inkscape-silhouette: <https://github.com/fablabnbg/inkscape-silhouette>
- SMTCut (formerly gerber2graphtec): <https://github.com/GatCode/gerber2graphtec>,
  original README <https://raw.githubusercontent.com/pmonta/gerber2graphtec/master/README>
- Vinyl-cut copper circuits: <https://www.makehaven.org/blog/vinyl-cutting-diy-circuitry>,
  <https://fab.cba.mit.edu/content/archive/processes/PCB/vinylcut.html>,
  <https://makezine.com/projects/vinyl-cut-pcb-resist/>
- 3M 1181 datasheet: <https://www.mouser.com/datasheet/2/1/78-8127-9953-0%201181%20DS-771379.pdf>
- Verowire datasheet: <https://static.chipdip.ru/lib/477/DOC021477720.pdf>;
  solderable wire: <https://www.farnell.com/datasheets/3179336.pdf>
- Inkjet UV masks: <https://www.eevblog.com/forum/beginners/good-printer-for-transparencies-for-uv-pcb/msg1110610/>
- JLCPCB shipping: <https://JLCPCB.com/help/article/65-Shipping-Info>
- Aalto Fablab: <https://fablabs.io/labs/aaltofablab>; Oodi:
  <https://oodihelsinki.fi/en/urban-workshops-konehuone-opens-on-18-december>
- Etchant disposal (HSY): <https://hsy.fi/en/waste-and-recycling/sorting/hazardous-waste/>

## Features

### FR-020: 1:1 printable perfboard drawing

`python3 -m kicad.perfboard` also writes `perfboard-print.svg`, sized in
millimetres at true scale:

- The component side as placed, and a mirrored solder side with every wire
- A cut line on the board outline, for kiss-cutting a placement sticker
- Crosshairs on the four corner holes, to align the print with a real board
- A 50 mm scale bar for checking the printer's scaling

`--check` covers it, like the other outputs.

### FR-021: Continuity checklist

`python3 -m kicad.perfboard` also writes `continuity.md`, grouped by build step
like the wire list. For each net it names one probe pad and every pad that must
beep to it. A pre-power section then lists the rail pairs that must read open.
The rails are the nets that `perfboard.toml [continuity] isolated` names.

### FR-022: Copper-foil test board

A generated cut file for one single-layer test board, cut on the Portrait 2. The
board carries:

- A ladder of trace widths and clearances
- 45° and filleted corners
- SMD 2.54 mm header pads
- A 3 mm power trace
- A crossover gap with its Kapton patch

It answers four questions on the bench: the smallest trace that survives
weeding, whether header pads hold under cable pull, how much heat the adhesive
takes, and whether the crossover holds. Only a pass leads to foil layouts for
real boards.

### FR-023: Stripboard mode (planned)

A strip-assignment router for the perfboard generator. It writes a track-cut map
that the Portrait 2 can mark with a pen; a blade would break in the holes. A
breadboard is a preset with fixed strips. Planned only: it waits on FR-020 and
FR-021 being used on a real build.

## Constraints

- KiCad-free, stdlib Python, like the rest of `kicad.perfboard`: the outputs
  regenerate in CI and in pre-commit
- Generated files carry the do-not-edit banner, and `--check` fails when they
  are stale
- Nothing physical is claimed to work until it has been built: FR-022's results
  go into this PRD as measurements, not estimates

## Acceptance Criteria

- [ ] `just balancebot::perfboard` writes `perfboard-print.svg` and
  `continuity.md`, and `just balancebot::perfboard-check` passes on the
  committed files
- [ ] A test asserts the print's hole pitch is exactly 2.54 mm in SVG user
  units, with the document sized in mm, and that the solder side mirrors
- [ ] The checklist lists every net of every step, and every pair of the
  configured rails
- [ ] A printout of `perfboard-print.svg` at 100 % measures 50.0 ± 0.5 mm on the
  scale bar (bench check)
- [ ] The FR-022 test board has been cut and soldered, and its results are
  recorded here

## Open Questions

- Aisler Blitz vs. JLCPCB + DHL for a 50 × 70 mm 2-layer board: a live quote
  from each
- Does a double-density print on ET-3750 transparency film block enough UV to
  expose dry film? This needs a test strip before buying the rest of the chain

## Related Documents

- [ADR-021: Hardware Source of Truth](../decisions/ADR-021-hardware-source-of-truth.md)
- [ADR-027: KiCad Carrier Generation](../decisions/ADR-027-kicad-carrier-generation.md)
- [PRD-012: Balancebot](PRD-012-balancebot.md), whose perfboard is the first user
