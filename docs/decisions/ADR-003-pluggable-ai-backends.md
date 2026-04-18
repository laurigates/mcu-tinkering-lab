---
id: ADR-003
title: Pluggable AI Backends (Claude API vs Ollama Self-Hosted)
status: accepted
created: 2026-03-05
---

# ADR-003: Pluggable AI Backends (Claude API vs Ollama Self-Hosted)

## Context

The ESP32-CAM vision system needs an AI model to analyze images and generate navigation
commands. Two viable options exist:

- **Claude API** (Anthropic): high-quality vision, requires internet, incurs per-token
  cost, subject to rate limits and API availability.
- **Ollama** (self-hosted): runs locally on a developer's machine or home server (e.g.,
  `llava-phi3`), no internet required, free after hardware cost, lower latency on LAN.

Neither option is universally preferable: development without internet access favors
Ollama; production quality and model capability favor Claude. The team wanted to support
both without maintaining two separate firmware branches.

## Decision

The AI backend is selected at **compile time** via a preprocessor macro in
`main/config.h`:

```c
#define CONFIG_AI_BACKEND_CLAUDE
// #define CONFIG_AI_BACKEND_OLLAMA
```

Each backend is implemented behind a common function interface. The build system
conditionally compiles only the selected backend. Credentials and endpoint URLs are
stored in `main/credentials.h` (excluded from version control via `.gitignore` and a
pre-commit hook).

Backend-specific configuration:

| Setting | Claude | Ollama |
|---------|--------|--------|
| API key | `CLAUDE_API_KEY` in `credentials.h` | not required |
| Endpoint | Anthropic API (HTTPS) | `OLLAMA_API_URL` in `config.h` |
| Model | `claude-3-haiku-20240307` (configurable) | `llava-phi3` (configurable) |
| Discovery | static HTTPS endpoint | mDNS or static IP |

## Consequences

**Positive**
- A single `#define` change and rebuild switches backends; no hardware changes needed.
- Credentials for each backend are isolated in a single file that is never committed.
- The interface can be extended to add future backends (Gemini, local llama.cpp server)
  without touching motor control or MQTT code.
- Ollama backend enables fully offline development and demos.

**Negative**
- Compile-time selection means both backends cannot be active simultaneously or
  switched at runtime without reflashing.
- Each developer must maintain their own `credentials.h`; there is no runtime
  configuration UI.
- CI cannot test AI inference end-to-end (no API keys in CI); build verification only.

## Alternatives Considered

- **Runtime selection via NVS / config file**: rejected for initial implementation due
  to added complexity on a resource-constrained device; can be revisited if runtime
  switching becomes a hard requirement.
- **Single backend only (Claude)**: rejected because offline development and
  cost-sensitive use cases require a local option.
- **HTTP abstraction layer with dynamic dispatch**: over-engineered for two backends;
  compile-time selection is simpler and eliminates dead code from the flash image.
