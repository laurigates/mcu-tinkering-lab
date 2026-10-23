# Feature Tracker

| ID | Feature | PRD | Status | Priority | Notes |
|----|---------|-----|--------|----------|-------|
| FR-001 | Main controller firmware (motor, LED, servo, OLED) | PRD-001 | active | P0 | Heltec WiFi LoRa 32 V1; ESP-IDF v5.4 |
| FR-002 | ESP32-CAM firmware with AI vision | PRD-002 | active | P0 | OV2640; Claude API + Ollama backends |
| FR-003 | MQTT telemetry and logging | PRD-002 | active | P1 | JSON messages, mDNS service discovery |
| FR-004 | Pluggable AI backend (Claude / Ollama) | PRD-002 | active | P1 | Compile-time selection via config.h |
| FR-005 | Pymunk 2D physics simulation | PRD-003 | active | P1 | Python 3.11; 18/24 tests passing |
| FR-006 | WebSocket communication bridge | PRD-003 | active | P2 | Port 8765; bi-directional motor/sensor |
| FR-007 | Hardware-in-the-loop (HIL) testing | PRD-003 | planned | P2 | Real ESP32 + virtual environment |
| FR-008 | OTA firmware updates | PRD-001 | active | P1 | Dual-partition; size limits enforced in CI |
| FR-009 | ESP32 build CI pipeline | PRD-001 | active | P0 | Parallel matrix; 4 projects; artifact archive |
| FR-010 | Python simulation CI pipeline | PRD-003 | active | P1 | uv + pytest with coverage |
| FR-011 | Pre-commit code quality hooks | — | active | P1 | clang-format, ruff, mypy, gitleaks |
| FR-012 | Docker development environment | — | active | P2 | ESP-IDF containerized; reproducible builds |
| FR-013 | ESP32-CAM LLM Telegram bot | — | active | P2 | Separate project; Claude/Ollama vision |
| FR-014 | ESP32-CAM web video streaming | — | active | P3 | Live MJPEG server |
| — | Arduino platform support | — | planned | P3 | No projects yet; no FR-ID in feature-tracker.json (drift) |
| — | STM32 platform support | — | planned | P3 | No projects yet; no FR-ID in feature-tracker.json (drift) |
| — | Automated project scaffolding tool | — | planned | P3 | Mentioned in README as coming soon; no FR-ID in feature-tracker.json (drift) |
| FR-015 | ESP32 host-based unit tests | — | planned | P2 | Blocked; no tests exist yet |
| FR-016 | SLAM / autonomous navigation | — | planned | P3 | Listed as future enhancement |
| — | Multi-robot swarm coordination | — | planned | P3 | Listed as future enhancement; no FR-ID in feature-tracker.json (drift) |
| FR-017 | Melody detector firmware (CV + ESP-DL + I2S) | PRD-011 | planned | P2 | XIAO ESP32-S3 Sense + MAX98357A; ROI classifier with INT8 CNN |
| FR-018 | Melody detector training pipeline | PRD-011 | planned | P2 | PyTorch + ESP-PPQ; programmatic + diffusion synthetic data |
| FR-019 | Melody detector validation harness | PRD-011 | planned | P3 | Gemini Robotics-ER 1.6 oracle benchmark |
| FR-020 | 1:1 printable perfboard drawing | PRD-013 | planned | P1 | perfboard-print.svg: mm-true, corner marks, 50 mm scale bar |
| FR-021 | Perfboard continuity checklist | PRD-013 | planned | P1 | continuity.md from the netlist; rail isolation pairs |
| FR-022 | Copper-foil test board | PRD-013 | planned | P2 | Portrait 2 knife-cut single-layer test board; bench results go in PRD-013 |
| FR-023 | Stripboard mode for the perfboard generator | PRD-013 | planned | P3 | Strip router, pen-plotted track-cut map, breadboard preset |
| FR-024 | Teuvo upgrade-request capture in the voice turn | PRD-014 | planned | P2 | __REQUEST__ sentinel + summary to teuvo/<id>/upgrade/request; desk off by default |
| FR-025 | Order facts downlink and gated phrasing | PRD-014 | planned | P2 | Facts JSON with seq and concierge status LWT; model sees codes only; __ORDER__ templates; sentence filter; truncation fixes first |
| FR-026 | Teuvo concierge service | PRD-014 | planned | P2 | packages/robocar/concierge: order state, fee table, phone gate, dispatch, order page |
| FR-027 | Upgrade payment rail | PRD-014 | planned | P2 | Stripe sandbox, then per-order Payment Link; PAID only by Lauri or a verified webhook |
| FR-028 | Gated planning workflow | PRD-014 | planned | P2 | teuvo-plan.yml: environment approval on main, OIDC federation, path-scoped Claude job + fixed PR job, draft PR |
| FR-029 | Upgrade-plan template | PRD-014 | planned | P2 | packages/robocar/upgrades/<code>-<slug>/: parts, cost.csv, gate table, renders |
| FR-030 | Upgrade link hand-off | PRD-014 | planned | P3 | Printed per-robot QR sticker + spoken order code; OLED QR later |
| FR-031 | Lead-time calibration | PRD-014 | planned | P3 | lead-times.csv of real order dates; quoted bands until then |
| FR-032 | Persona overlay in NVS | PRD-015 | planned | P2 | Name (nom + gen), preset personality, voice; prompt-budget host test |
| FR-033 | Per-device identity | PRD-015 | planned | P2 | MQTT client id, topic root and mDNS from the device id |
| FR-034 | Teuvo backend (Gemini proxy) | PRD-015 | planned | P3 | Paid key server-side; per-device token, monthly cap, kill switch, model remap |
| FR-035 | Owner onboarding and notices | PRD-015 | planned | P3 | Adults only, privacy, AI disclosure, WiFi setup, reset |
| FR-036 | Open-hardware build pack | PRD-015 | planned | P3 | Route A: build guide, BOM, fab files, own-key setup guide |
| FR-037 | Teuvo network list with scan-and-pick | ADR-028 | planned | P1 | Up to 8 networks in NVS; all-channel scan; serial wifi commands; home flag gates MQTT |
| FR-038 | Network services armed on link-up | ADR-028 | planned | P0 | Non-blocking connect; MQTT, OTA and rollback-confirm on GOT_IP; Improv re-arm; skip requests offline |
| FR-039 | Runtime Gemini credential in NVS | ADR-028 | planned | P0 | gemini auth; fixes key dropped after Improv; 402/403 handling |
| FR-040 | Offline voice clips and help request | ADR-028 | planned | P1 | Pre-rendered ADPCM persona lines embedded in the app image |
| FR-041 | WiFi join from a QR code | ADR-028 | planned | P2 | quirc on JPEG luma; both WIFI: dialects; QR entries never home; console confirmation while a known network is reachable |
| FR-042 | WiFi hunt game | ADR-028 | planned | P3 | Opt-in, supervised: QR sweep, known-AP RSSI climb, ask for help |
| FR-043 | TTS migration off gemini-3.1-flash-tts-preview | ADR-028 | planned | P0 | Earliest shutdown 2026-11-17; probe gemini-3.8-flash-tts (Finnish; not Flash-Lite) for streaming and style prefix |

## Status Key

| Status | Meaning |
|--------|---------|
| active | Implemented and in use |
| planned | Scoped but not yet started |
| in-progress | Work has begun |
| blocked | Depends on something unresolved |
| deprecated | No longer maintained |
