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
│   ├── usb-composite.md
│   ├── wifi-connectivity.md
│   └── claude-api-integration.md
├── adrs/
│   ├── 0001-usb-composite-architecture.md
│   └── 0002-phased-development-approach.md
└── prps/
    ├── cdc-ncm-usb-ethernet.md
    └── wifi-hotspot-connection.md
```

## Root Directory Policy

The repository root should contain only standard files (README, CLAUDE.md, CMakeLists.txt, justfile, sdkconfig.defaults, etc.). Move documentation into `docs/`.

## Document Lifecycle

### 1. Detection
Claude identifies documentation opportunities during conversation:
- Feature discussions trigger PRD detection
- Architecture debates trigger ADR detection
- Implementation planning triggers PRP detection

### 2. Confirmation
User approves document creation via AskUserQuestion prompt:
- "Yes, create [document type]" - Proceed with creation
- "Not now, remind me later" - Defer, re-prompt if topic expands
- "No, just continue" - Skip documentation

### 3. Generation
Document created in appropriate location.

### 4. Tracking
Manifest updated with document metadata; feature-tracker.json updated if FR codes are referenced.

## Automatic Detection

When `has_document_detection` is enabled, Claude will proactively prompt to create:

| Conversation Type | Document Suggested |
|-------------------|-------------------|
| Feature requirements discussed | PRD |
| Architecture decisions debated | ADR |
| Implementation approach planned | PRP |

## ADR Numbering

ADRs use sequential 4-digit zero-padded prefixes: `0001-title.md`, `0002-title.md`, etc.

## Cross-Referencing

Link related documents using relative paths:
- PRD → ADR: `See [ADR-0001](../adrs/0001-title.md) for rationale`
- PRP → PRD: `Implements requirements from [PRD](../prds/feature.md)`
