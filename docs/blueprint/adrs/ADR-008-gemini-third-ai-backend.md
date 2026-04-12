# ADR-008: Gemini Robotics-ER 1.5 as Third AI Backend

**Status**: accepted
**Date**: 2026-03-22
**Source commit**: feat: add Gemini Robotics-ER 1.5 as third AI backend for robocar-camera (#140, 9bc9fa8)
**Confidence**: 9/10

---

## Context

The robocar-camera AI backend supported two inference providers at compile time: Anthropic Claude API (cloud, highest quality) and Ollama (self-hosted, lowest latency). Google released Gemini Robotics-ER 1.5, a model specifically optimized for spatial reasoning and robotics task planning. This model is a natural fit for robot navigation inference and offers a third option between the cloud-quality/cost of Claude and the local-latency of Ollama.

See ADR-003 for the original pluggable AI backend decision.

## Decision

Add `gemini_backend.h/c` implementing the Gemini Robotics-ER 1.5 API as a third compile-time-selectable backend via `#define USE_GEMINI_API`. The backend sends base64-encoded camera frames inline in the JSON payload to the Gemini `generateContent` endpoint. The existing backend selection pattern (`#ifdef USE_CLAUDE_API / USE_OLLAMA`) is extended with a third branch.

## Consequences

**Positive:**
- Third inference option: cloud-quality spatial reasoning with Google's robotics-specialized model
- No protocol changes required on the main controller side
- Zero runtime overhead for users not selecting Gemini

**Negative:**
- Requires a Google AI API key (additional credential management)
- Base64-encoding large camera frames increases request payload size vs. Claude's multipart approach
- Three backends increases maintenance surface for protocol changes

## Alternatives Considered

1. **OpenAI Vision API** — Not robotics-specialized; Claude already covers general vision
2. **Runtime backend selection** — More flexible but increases RAM usage and binary size; compile-time selection sufficient given typical deployment model
3. **Wait for on-device Gemini Nano** — Not available for ESP32-CAM class hardware

## Implementation

- `USE_GEMINI_API` compile flag in `sdkconfig.defaults`
- Backend: `packages/esp32-projects/robocar-camera/main/gemini_backend.h/c`
- API: `generativelanguage.googleapis.com/v1beta/models/gemini-robotics-er-1.5:generateContent`
- Auth: `x-goog-api-key` header

## Related

- ADR-003: Pluggable AI Backends (Claude API vs Ollama)
