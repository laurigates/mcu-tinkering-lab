# Documentation

Project documentation, architecture decisions, and development planning for
MCU Tinkering Lab.

## Structure

```
docs/
├── decisions/            # Architecture Decision Records (ADR-NNN-*.md)
├── requirements/         # Product Requirements Documents (PRD-NNN-*.md)
├── prompts/              # Product Requirement Prompts + implementation plans
├── reference/
│   ├── boards/           # Board profiles (ESP32 variants, STM32, Orange Pi, etc.)
│   └── datasheets/       # Vendor datasheets for components
├── flasher/              # Browser-based ESP Web Tools firmware flasher (GitHub Pages)
├── manifest.json         # Blueprint manifest
├── manifest.md           # Human-readable manifest
└── feature-tracker.{json,md}  # Feature tracking
```

## Document types

| Type | Directory | Filename convention |
|------|-----------|---------------------|
| ADR  | `docs/decisions/`    | `ADR-NNN-slug.md` |
| PRD  | `docs/requirements/` | `PRD-NNN-slug.md` |
| PRP  | `docs/prompts/`      | `feature-slug.md` |

## ADRs (Architecture Decision Records)

See `decisions/` for the full list. Use sequential numbering: `ADR-001`,
`ADR-002`, etc. Current highest is ADR-016 (hierarchical AI controller).

## PRDs (Product Requirements Documents)

See `requirements/` for the full list. Covers robocar, IT troubleshooter,
Xbox bridge, NFC scavenger hunt, audiobook player, gamepad synth, thinkpack.

## PRPs (Product Requirement Prompts)

See `prompts/` for active/planned implementation prompts: HIL testing,
host-based unit tests, SLAM, hierarchical AI controller, OTA plan.

## Cross-references

Link related documents with relative paths, e.g.:

```markdown
See [ADR-001: Monorepo Structure](../decisions/ADR-001-monorepo-structure.md)
```
