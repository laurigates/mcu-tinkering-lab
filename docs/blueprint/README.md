# Blueprint Development

Blueprint manages project documentation, architecture decisions, and development planning for the MCU Tinkering Lab.

## Structure

```
docs/blueprint/
├── manifest.json           # Configuration and version tracking
├── feature-tracker.json    # Feature implementation progress
├── feature-tracker.md      # Human-readable feature overview
├── manifest.md             # Human-readable manifest (legacy)
├── prds/                   # Product Requirements Documents
├── adrs/                   # Architecture Decision Records
├── work-orders/            # Task packages for subagents
│   ├── completed/
│   └── archived/
├── ai_docs/                # Curated documentation (on-demand)
│   ├── libraries/
│   └── project/
└── README.md               # This file
```

## Commands

| Command | Purpose |
|---------|---------|
| `/blueprint:status` | Check version and configuration |
| `/blueprint:derive-prd` | Derive PRD from existing documentation |
| `/blueprint:derive-adr` | Derive ADRs from codebase analysis |
| `/blueprint:derive-plans` | Derive docs from git history |
| `/blueprint:derive-rules` | Derive rules from git commit decisions |
| `/blueprint:prp-create` | Create a Product Requirement Prompt |
| `/blueprint:generate-rules` | Generate rules from PRDs |
| `/blueprint:sync` | Check for stale generated content |
| `/blueprint:feature-tracker-status` | View feature completion stats |
| `/blueprint:feature-tracker-sync` | Sync tracker with project files |
| `/blueprint:claude-md` | Update CLAUDE.md |

## Documents

### PRDs (Product Requirements Documents)
- PRD-001: AI-Powered Robot Car System
- PRD-002: ESP32-CAM Vision and AI Backend
- PRD-003: Robot Car Physics Simulation
- PRD-004: IT Troubleshooter
- PRD-005: Xbox-to-Switch Bridge
- PRD-006: NFC Scavenger Hunt
- PRD-007: Audiobook Player
- PRD-008: Gamepad Synth

### ADRs (Architecture Decision Records)
- ADR-001: Monorepo Structure for Multi-Platform MCU Projects
- ADR-002: Dual ESP32 Architecture for Robot Car
- ADR-003: Pluggable AI Backends (Claude API vs Ollama)
- ADR-004: OTA Update Architecture
- ADR-005: IT Troubleshooter Hardware
- ADR-006: USB Composite Architecture
- ADR-007: Shared I2C Protocol Component
- ADR-008: Gemini Robotics-ER 1.5 as Third AI Backend
- ADR-009: Switch Pro Controller On-Device Protocol
- ADR-010: MQTT Logging Architecture
- ADR-011: Gamepad Synth I2S Audio
- ADR-012: Browser-Based Web Flasher Architecture

### PRPs (Product Requirement Prompts)
Located in `docs/prps/`:
- host-based-unit-tests: Expand ESP32 host-based unit tests (in-progress, P2)
- hardware-in-the-loop-testing: HIL testing via simulation bridge (planned, P2)
- slam-autonomous-navigation: SLAM and autonomous navigation (planned, P3)
