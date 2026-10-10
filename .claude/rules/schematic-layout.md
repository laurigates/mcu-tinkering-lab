# Schematic Layout: What Is Hand-Authored, and What a Symbol Change Touches

Placement of components in `docs/schematics/circuits/*.py` is hand-authored on
purpose. The router draws wires; it does not place parts. What is generated is
the pin labels, the physical pad order and the net endpoints, read from the
hardware join at render time. The split is ADR-021 § "What stays
hand-authored" (`docs/decisions/ADR-021-hardware-source-of-truth.md`); the
router and its tests are in `docs/schematics/README.md`.

Symbol geometry in `docs/schematics/components.py` (size, pin positions,
padding) is coupled to the test fixtures in `test_routing.py` and to the
`total_length` pin on `robocar_unified` in the same file. After touching
`components.py`, run both:

```sh
just schematics::test
just schematics::metrics
```

A local render rewrites `balancebot.png` and `gamepad_synth.png` even when the
circuit's SVG is unchanged (host PNG encoder, checked at ed86f09). Stage only
images whose SVG changed, restore the others with `git restore <file>`, and use
`just schematics::render-one <name>` for single-circuit work. CI diffs only the
SVGs.
