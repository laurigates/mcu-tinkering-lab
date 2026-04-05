# Document Management

Rules for organizing and managing project documentation.

## Document Types and Locations

| Document Type | Directory | Naming Convention | Purpose |
|--------------|-----------|-------------------|---------|
| PRD | `docs/prds/` | `{feature-name}.md` | Product Requirements Documents |
| ADR | `docs/adrs/` | `{number}-{title}.md` | Architecture Decision Records |
| PRP | `docs/prps/` | `{feature-name}.md` | Product Requirement Prompts |

### Examples

```
docs/
├── prds/
│   ├── nfc-game-mechanics.md
│   └── audio-feedback.md
├── adrs/
│   ├── 0001-rc522-spi-interface.md
│   └── 0002-gemini-tts-backend.md
└── prps/
    └── multi-player-mode.md
```

## Root Directory Policy

The repository root should contain only:

### Allowed Files
- `README.md` - Project overview and quick start
- `CHANGELOG.md` - Release notes (often auto-generated)
- Configuration files (CMakeLists.txt, sdkconfig.defaults, etc.)

### Not Allowed in Root
- Requirements documents (move to `docs/prds/`)
- Architecture documents (move to `docs/adrs/`)
- Design documents (move to `docs/` or appropriate subdirectory)

## Automatic Detection

Claude will proactively prompt to create documentation when conversations contain:

| Conversation Type | Document Suggested |
|-------------------|-------------------|
| Feature requirements discussed | PRD |
| Architecture decisions debated | ADR |
| Implementation approach planned | PRP |

## ADR Numbering

ADRs use sequential numbering with zero-padded 4-digit prefixes:

```
0001-rc522-spi-interface.md
0002-gemini-tts-backend.md
```

## Cross-Referencing

Link related documents using relative paths:
```markdown
See [ADR-0001: RC522 SPI Interface](../adrs/0001-rc522-spi-interface.md)
```
