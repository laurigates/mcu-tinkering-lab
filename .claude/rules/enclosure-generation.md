---
reviewed: 2026-11-01
paths:
  - 'tools/enclosure/**'
  - 'packages/**/hardware/enclosure/**'
---
# Carrier Cases: Use the `enclosure-cad` Skill

Cases are generated from the committed board by `tools/enclosure`
(build123d). The workflow, the two gates and the traps are in the
`enclosure-cad` skill (`.claude/skills/enclosure-cad/`). Three matter at edit
time:

1. **Never import build123d under `tools/kicad`.** Its tests run on
   stdlib-only Python in CI and pre-commit.
2. **Read geometry from the board, not from a copy.** Positions and holes come
   from the `.kicad_pcb`, heights from the STEP; only parts with no 3D model
   get an `enclosure.toml` height, with its `source`.
3. **`just enclosure::gen` must exit 0.** It fails on a clash with the board, a
   wall under 2.0 mm, a board that no lift or tilt gets into the case, or a
   board screw no screwdriver can reach; JLC3DP's own upload check is not a
   substitute. `just enclosure::sequence` animates the motions it found.
