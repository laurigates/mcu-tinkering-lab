---
paths:
  - 'packages/**/hardware/**'
  - '**/*.kicad_*'
---
# Scripting KiCad 10: Traps That Read as Success

`packages/robotics/balancebot/hardware/carrier/` builds its board with KiCad's
`pcbnew` Python module, `kicad-cli` and Freerouting rather than in the GUI. Every
trap below was hit building it (PR #749, KiCad 10.0.6), and most of them did not
fail at the point of the mistake. Issue #750 plans a shared `carrier-pcb` skill
for `tools/kicad/`; move this content there when it lands.

## 1. `board.Remove()` corrupts SWIG for the rest of the process

After removing a loaded board's footprints with `board.Remove()`, **every later
`pcbnew` call** returns an untyped `SwigPyObject`. That includes
`FootprintLoad()`, `LoadBoard()` and `PCB_IO_KICAD_SEXPR().FootprintLoad()`. The
error lands on whichever call comes next
(`'SwigPyObject' object has no attribute 'SetFPID'`), so it reads as a bug in
that call.

Confirmed by elimination: 200 consecutive `pcbnew.FootprintLoad()` calls with no
`Remove()` all returned `FOOTPRINT`. The first misdiagnosis blamed
`FootprintLoad()`'s per-call plugin lookup. That was wrong, and the reader
`build_board.py` reuses is a harmless leftover of it.

**Do instead:** build a fresh board on every run. `pcbnew.NewBoard(path)` also
writes a `.kicad_pro` beside `path`, so create it in a scratch directory and
`board.Save()` to the real board file. Then the project's own `.kicad_pro` is
not overwritten.

## 2. A vendor footprint can carry board-shape geometry

Seeed's `XIAO-*-DIP` footprints are for soldering the module flat. They include
an **Edge.Cuts** cutout under the module and plated holes for the bottom pads
along it. Placed on a carrier that sockets the XIAO, that cutout would be milled
out of the board. DRC reports it only indirectly, as `copper_edge_clearance`
against pads 15–24.

**Check:** `grep -c 'Edge.Cuts' <footprint>.kicad_mod` before using a vendor
footprint on a different board. `hardware/carrier/scripts/derive_xiao_socket.py`
derives a 14-pad socket footprint and the matching symbol, and leaves Seeed's
files unmodified.

## 3. Pad 1 is not "+", and the silkscreen says so

KiCad's `Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal` prints **"−"
beside pad 1** and "+" beside pad 2. The first carrier schematic put VBAT on
pin 1. ERC and DRC were both clean, and every net check passed, because the
error was consistent everywhere. The only evidence was the "−" next to the
square pad in a board render.

**Check:** for every polarised connector, read the footprint's `fp_text` "+" and
"−" positions against the pads (`(at x y)` of each), and map nets to what the
silkscreen will say. Then confirm against the physical part's own moulding: the
footprint is the library's claim, not the part's (`board-layout-from-vendor-files.md`).

## 4. `kicad-cli` ERC JSON positions are in units of 100 mm

`kicad-cli sch erc --format json` reports `pos` values like `{"x": 1.7653,
"y": 0.6096}` for a pin at (176.53 mm, 60.96 mm), so multiply by 100 before
placing anything there. The tell is that ×100 lands on the 1.27 mm schematic
grid. The report's own `coordinate_units` field says `mm`, so it cannot be
trusted for this.

## 5. Freerouting does not reproduce a route

The same DSN routes differently on every run, and which pad ends up crowded
moves with it (a starved thermal appeared on J3 pad 7, then on U1 pad 13).
Consequences:

- Commit the board that was checked. A re-run is a different board and needs
  checking again.
- Compare **netlists**, never board files, when proving a refactor changed
  nothing.
- A fix tuned to one run's geometry (a spoke width, a single DRC exclusion) can
  fail on the next run. Fix the class, not the instance.

## 6. Verify the gate can fail

Each check here was proven able to fail before being trusted:

- DRC exits 5 on a copy of the board with one track segment deleted.
- `check_pins.py` fails with `PIN_DIR_LEFT` moved to GPIO27.
- The netlist comparison fails with two pins swapped in the expected plan.

Restore a mutated `pin_config.h` from a `cp` snapshot, never `git checkout --`.

## Related

- `board-layout-from-vendor-files.md`: the physical pin order of a breakout
  comes from its vendor's board file. Traps 2 and 3 are the KiCad-library
  version of the same rule.
- `schematic-layout.md` and ADR-023: the repo's own schematic renderer, which
  this does not replace.
- `packages/robotics/balancebot/hardware/carrier/README.md`: the carrier, its
  recipes and its pre-order checks (#748).
