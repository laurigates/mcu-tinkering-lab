# Schematics

Text-defined circuit schematics for MCU Tinkering Lab projects, rendered with
[Schemdraw](https://schemdraw.readthedocs.io/).

Each circuit lives in `circuits/` as a small Python module that imports
reusable component blocks from `components.py` and returns a
`schemdraw.Drawing` from a `draw()` function. `render.py` discovers them and
writes SVG + PNG to `images/`.

## Layout

```
docs/schematics/
├── circuits/              # One .py per circuit (defines draw())
│   └── gamepad_synth.py
├── components.py          # Reusable chip/breakout factories (ESP32, MAX98357A, ...)
├── images/                # Generated SVG + PNG (committed so GitHub renders them)
├── render.py              # Batch-render every circuit in circuits/
├── justfile               # `just schematics::render`, `::clean`, ...
└── pyproject.toml         # uv-managed deps: schemdraw, cairosvg
```

## Quick start

```bash
# From repo root
just schematics::render              # renders every circuit
just schematics::render-one gamepad_synth
just schematics::clean
```

Or directly:

```bash
cd docs/schematics
uv sync
uv run python render.py
```

## Adding a new circuit

1. Add any missing component factories to `components.py`. Keep pin sets
   minimal and name pins as the firmware does.
2. Create `circuits/<name>.py` with a `draw() -> schemdraw.Drawing` function.
   Reference `circuits/gamepad_synth.py` as a template.
3. Run `just schematics::render`. The SVG + PNG land in `images/`.
4. Link the rendered PNG from the project's README or WIRING.md:
   ```markdown
   ![Wiring](../../docs/schematics/images/<name>.png)
   ```

## Conventions

- **Pin ordering**: schemdraw renders L/R side pins *bottom-to-top* in the
  order given. Component factories list pins bottom-first so visual order
  matches the intuitive top-to-bottom reading order.
- **Alignment**: chips that connect to each other keep matching pin counts on
  facing sides so default spacing aligns them when placed at the same
  y-center. E.g. both the ESP32-S3-Zero and MAX98357A factories expose 4 pins
  on the left and 3 on the right.
- **Labels**: factories don't set a center label — individual circuits add
  `.label('Name', loc='bot', ofst=0.4)` to avoid collisions with pin labels.
- **Colors**: signal buses use `steelblue`; power/ground use default black.
- **Output format**: SVG is the source of truth (diff-friendly, crisp);
  PNG is generated alongside for easier embedding in READMEs.

## Source of truth

Each circuit file cites the authoritative wiring document at the top (usually
the project's `WIRING.md`). Keep the schematic and that document in sync when
pins change.
