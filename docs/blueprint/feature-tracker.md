# Feature Tracker

| ID | Feature | PRD | Status | Priority | Notes |
|----|---------|-----|--------|----------|-------|
| FR-001 | Robocar firmware on one XIAO ESP32-S3 Sense (robocar) | PRD-001 | active | P0 | Motors, servos, LEDs via PCA9685 behind a TCA9548A; OLED; the Heltec + ESP32-CAM pair is removed (ADR-013) |
| FR-002 | Camera capture and Gemini vision planner | PRD-002 | active | P0 | OV3660 on the XIAO; Gemini Robotics-ER 2 planner (ADR-016); the ESP32-CAM board is removed (ADR-013) |
| FR-003 | MQTT telemetry and logging | PRD-001 | active | P1 | robocar/status, robocar/events (ADR-033), remote commands; log forwarding open (#956); mDNS broker discovery proposed (ADR-034 §3) |
| FR-004 | Pluggable AI backend (Claude / Ollama) | PRD-002 | deprecated | P1 | The claude_*/ollama_* backends are removed; Gemini only. A second backend would be a newly designed abstraction |
| FR-005 | Pymunk 2D physics simulation | PRD-003 | active | P1 | Python 3.12; import package robocar_sim. The plant (robocar_sim.plant: motors, contacts, two rangefinders, two floor sensors) for the behaviour twin, plus the legacy demo (ADR-036). 194 passed, 6 skipped on the core and dev dependencies (uv run pytest tests/, 2026-11-13) |
| FR-006 | WebSocket communication bridge | PRD-003 | deprecated | P2 | Removed by ADR-036, which withdraws PRD-003 R6-R8. The JSONL stepper python -m robocar_sim.plant is the simulation's external interface |
| FR-007 | Hardware-in-the-loop (HIL) testing | PRD-003 | planned | P2 | Real ESP32 + virtual environment. No bridge to build on since ADR-036 removed the WebSocket and serial bridge (FR-006) |
| FR-008 | OTA firmware updates | PRD-001 | active | P1 | Manifest polling, build-SHA rule, rollback; no board has completed an OTA yet (#1005). Notify subscription dropped (#1054); first check after 360 min (#1055) |
| FR-009 | ESP32 build CI pipeline | PRD-001 | active | P0 | Parallel matrix; 4 projects; artifact archive |
| FR-010 | Python simulation CI pipeline | PRD-003 | active | P1 | uv + pytest with coverage (floor 25 %) on numpy, pymunk, pyyaml and the dev extra; the viz extra (Genesis, torch) is not installed in CI (ADR-036) |
| FR-011 | Pre-commit code quality hooks | — | active | P1 | clang-format, ruff, mypy, gitleaks |
| FR-012 | Docker development environment | — | active | P2 | ESP-IDF containerized; reproducible builds |
| FR-013 | ESP32-CAM LLM Telegram bot | — | active | P2 | Separate project; Claude/Ollama vision |
| FR-014 | ESP32-CAM web video streaming | — | active | P3 | Live MJPEG server |
| — | Arduino platform support | — | planned | P3 | No projects yet; no FR-ID in feature-tracker.json (drift) |
| — | STM32 platform support | — | planned | P3 | No projects yet; no FR-ID in feature-tracker.json (drift) |
| — | Automated project scaffolding tool | — | planned | P3 | Mentioned in README as coming soon; no FR-ID in feature-tracker.json (drift) |
| FR-015 | ESP32 host-based unit tests | — | active | P2 | Host suites under packages/robotics/robocar/test, run by just robocar::test (54 ctest suites on 2026-11-09) and in CI |
| FR-016 | SLAM / autonomous navigation | — | planned | P3 | Listed as future enhancement |
| — | Multi-robot swarm coordination | — | planned | P3 | Listed as future enhancement; no FR-ID in feature-tracker.json (drift) |
| FR-017 | Melody detector firmware (CV + ESP-DL + I2S) | PRD-011 | planned | P2 | XIAO ESP32-S3 Sense + MAX98357A; ROI classifier with INT8 CNN |
| FR-018 | Melody detector training pipeline | PRD-011 | planned | P2 | PyTorch + ESP-PPQ; programmatic + diffusion synthetic data |
| FR-019 | Melody detector validation harness | PRD-011 | planned | P3 | Gemini Robotics-ER 1.6 oracle benchmark |
| FR-020 | 1:1 printable perfboard drawing | PRD-013 | planned | P1 | perfboard-print.svg: mm-true, corner marks, 50 mm scale bar |
| FR-021 | Perfboard continuity checklist | PRD-013 | planned | P1 | continuity.md from the netlist; rail isolation pairs |
| FR-022 | Copper-foil test board | PRD-013 | planned | P2 | Portrait 2 knife-cut single-layer test board; bench results go in PRD-013 |
| FR-023 | Stripboard mode for the perfboard generator | PRD-013 | planned | P3 | Strip router, pen-plotted track-cut map, breadboard preset |
| FR-024 | Teuvo upgrade-request capture in the voice turn | PRD-014 | planned | P2 | __REQUEST__ sentinel + summary to teuvo/<id>/upgrade/request; desk off by default (#853) |
| FR-025 | Order facts downlink and gated phrasing | PRD-014 | planned | P2 | Facts JSON with seq and concierge status LWT; model sees codes only; __ORDER__ templates; sentence filter; truncation fixes first (#853) |
| FR-026 | Teuvo concierge service | PRD-014 | planned | P2 | packages/robocar/concierge: order state, fee table, phone gate, dispatch, order page (#854) |
| FR-027 | Upgrade payment rail | PRD-014 | planned | P2 | Stripe sandbox, then per-order Payment Link; PAID only by Lauri or a verified webhook (#854) |
| FR-028 | Gated planning workflow | PRD-014 | planned | P2 | teuvo-plan.yml: environment approval on main, OIDC federation, path-scoped Claude job + fixed PR job, draft PR (#855) |
| FR-029 | Upgrade-plan template | PRD-014 | planned | P2 | packages/robocar/upgrades/<code>-<slug>/: parts, cost.csv, gate table, renders (#855) |
| FR-030 | Upgrade link hand-off | PRD-014 | planned | P3 | Printed per-robot QR sticker + spoken order code; OLED QR later |
| FR-031 | Lead-time calibration | PRD-014 | planned | P3 | lead-times.csv of real order dates; quoted bands until then |
| FR-032 | Persona overlay in NVS | PRD-015 | planned | P2 | Name (nom + gen), preset personality, voice; prompt-budget host test (#852) |
| FR-033 | Per-device identity | PRD-015 | planned | P2 | MQTT client id, topic root and mDNS from the device id (#851) |
| FR-034 | Teuvo backend (Gemini proxy) | PRD-015 | planned | P3 | Paid key server-side; per-device token, monthly cap, kill switch, model remap |
| FR-035 | Owner onboarding and notices | PRD-015 | planned | P3 | Adults only, privacy, AI disclosure, WiFi setup, reset |
| FR-036 | Open-hardware build pack | PRD-015 | planned | P3 | Route A: build guide, BOM, fab files, own-key setup guide |
| FR-037 | Teuvo network list with scan-and-pick | ADR-028 | planned | P1 | Up to 8 networks in NVS; all-channel scan; serial wifi commands; home flag gates MQTT (#848) |
| FR-038 | Network services armed on link-up | ADR-028 | in-progress | P0 | Done: non-blocking connect, OTA/MQTT start on link-up, Improv re-arm, requests skipped offline (#846). Open: MQTT client at boot (#1054), first OTA check (#1055), home-flag gating needs FR-037 |
| FR-039 | Runtime Gemini credential in NVS | ADR-028 | active | P0 | gemini auth; fixes key dropped after Improv; 402/403 handling |
| FR-040 | Offline voice clips and help request | ADR-028 | planned | P1 | Pre-rendered ADPCM persona lines embedded in the app image (#849) |
| FR-041 | WiFi join from a QR code | ADR-028 | planned | P2 | quirc on JPEG luma; both WIFI: dialects; QR entries never home; console confirmation while a known network is reachable (#850) |
| FR-042 | WiFi hunt game | ADR-028 | planned | P3 | Opt-in, supervised: QR sweep, known-AP RSSI climb, ask for help |
| FR-043 | TTS migration off gemini-3.1-flash-tts-preview | ADR-028 | active | P0 | Earliest shutdown 2026-11-17; probe gemini-3.8-flash-tts (Finnish; not Flash-Lite) for streaming and style prefix (#844) |
| FR-044 | Robot needs from hardware.toml and the parts registry | PRD-016 | planned | P1 | One list per robot; [[consumables]] for off-board parts |
| FR-045 | Source records with tariff code and origin | PRD-016 | planned | P1 | Reuses the registry buy field; currency, quote date, lead time |
| FR-046 | Delivered-cost rules R1-R10 as tested functions | PRD-016 | planned | P0 | EU flat duty, DHL fee, AliExpress per-code charge, JLCPCB VAT/shipping/coupons |
| FR-047 | Parcel planner minimising cash | PRD-016 | planned | P1 | Pack sizes, owned stock from one-page order history; consumed cost reported |
| FR-048 | Cost per robot at N with labour | PRD-016 | planned | P1 | Hours in docs/reference/build-times.toml, estimates replaced by measurements |
| FR-049 | JLCPCB PCBA vs bare PCB plus hand assembly | PRD-016 | planned | P2 | IOSS and DDP both priced |
| FR-050 | Calibration fixtures against real orders | PRD-016 | planned | P0 | Fails when a prediction misses its observed total by more than EUR 1; fitted vs held-out fixtures |
| FR-051 | Plan table and CSV output | PRD-016 | planned | P2 | tmp/quotes/plan-<date>.csv, one row per line |
| FR-052 | Rule validity windows | PRD-016 | planned | P2 | Warns outside R1's 2026-07-01..2028-07-01 and the VAT rate |

## Status Key

| Status | Meaning |
|--------|---------|
| active | Implemented and in use |
| planned | Scoped but not yet started |
| in-progress | Work has begun |
| blocked | Depends on something unresolved |
| deprecated | No longer maintained |
