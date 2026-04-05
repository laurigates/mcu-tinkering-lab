# Document Management

Rules for organizing and managing project documentation.

## Document Types and Locations

| Document Type | Directory | Naming Convention | Purpose |
|--------------|-----------|-------------------|---------|
| PRD | `docs/blueprint/prds/` | `PRD-{NNN}-{title}.md` | Product Requirements Documents |
| ADR | `docs/blueprint/adrs/` | `ADR-{NNN}-{title}.md` | Architecture Decision Records |
| PRP | `docs/prps/` | `{feature-name}.md` | Product Requirement Prompts |

## Root Directory Policy

The repository root should contain only:

### Allowed Files
- `README.md` - Project overview and quick start
- `CHANGELOG.md` - Release notes (often auto-generated)
- `LICENSE` or `LICENSE.md` - License file
- `CONTRIBUTING.md` - Contribution guidelines
- Configuration files (justfile, Makefile, .clang-format, etc.)
- Build/CI configuration (.github/, etc.)

### Not Allowed in Root
- Requirements documents (move to `docs/blueprint/prds/`)
- Architecture documents (move to `docs/blueprint/adrs/`)
- Design documents (move to `docs/` or appropriate subdirectory)

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
Document created in appropriate location with proper naming convention.

### 4. Tracking
Manifest updated with document metadata in `docs/blueprint/manifest.json`.

## Automatic Detection

When conversations contain:

| Conversation Type | Document Suggested |
|-------------------|-------------------|
| Feature requirements discussed | PRD |
| Architecture decisions debated | ADR |
| Implementation approach planned | PRP |

## ADR Numbering

ADRs use sequential numbering: `ADR-001`, `ADR-002`, etc.
Current highest: ADR-006 (USB Composite Architecture)

## Cross-Referencing

Link related documents using relative paths:
```markdown
See [ADR-001: Monorepo Structure](../adrs/ADR-001-monorepo-structure.md)
```
