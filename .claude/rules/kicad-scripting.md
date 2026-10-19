---
paths:
  - 'packages/**/hardware/**'
  - '**/*.kicad_*'
  - 'tools/kicad/**'
  - 'tools/kicad.just'
---
# KiCad Carriers: Use the `carrier-pcb` Skill

Carrier PCBs are generated from `hardware.toml` by `tools/kicad`
([ADR-027](../../docs/decisions/ADR-027-kicad-carrier-generation.md)). The
workflow, the parts registry and 35 traps that read as success are in
the `carrier-pcb` skill (`.claude/skills/carrier-pcb/`). Three matter at edit time:

1. **Never call `board.Remove()` in a pcbnew script.** Every later pcbnew call
   returns an untyped `SwigPyObject`; build a fresh `NewBoard()` instead.
2. **Check polarity against the footprint's own text.** XT30 pad 1 is "−", and
   ERC, DRC and every net check pass with it reversed.
3. **Compare netlists, never board files.** Freerouting does not route the same
   way twice; commit the board you checked.
