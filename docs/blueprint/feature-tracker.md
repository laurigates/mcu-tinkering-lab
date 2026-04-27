# Feature Tracker

| Feature | PRD | Status | Priority | Notes |
|---------|-----|--------|----------|-------|
| Main controller firmware (motor, LED, servo, OLED) | PRD-001 | active | P0 | Heltec WiFi LoRa 32 V1; ESP-IDF v5.4 |
| ESP32-CAM firmware with AI vision | PRD-002 | active | P0 | OV2640; Claude API + Ollama backends |
| MQTT telemetry and logging | PRD-002 | active | P1 | JSON messages, mDNS service discovery |
| Pluggable AI backend (Claude / Ollama) | PRD-002 | active | P1 | Compile-time selection via config.h |
| Pymunk 2D physics simulation | PRD-003 | active | P1 | Python 3.11; 18/24 tests passing |
| WebSocket communication bridge | PRD-003 | active | P2 | Port 8765; bi-directional motor/sensor |
| Hardware-in-the-loop (HIL) testing | PRD-003 | planned | P2 | Real ESP32 + virtual environment |
| OTA firmware updates | PRD-001 | active | P1 | Dual-partition; size limits enforced in CI |
| ESP32 build CI pipeline | PRD-001 | active | P0 | Parallel matrix; 4 projects; artifact archive |
| Python simulation CI pipeline | PRD-003 | active | P1 | uv + pytest with coverage |
| Pre-commit code quality hooks | — | active | P1 | clang-format, ruff, mypy, gitleaks |
| Docker development environment | — | active | P2 | ESP-IDF containerized; reproducible builds |
| ESP32-CAM LLM Telegram bot | — | active | P2 | Separate project; Claude/Ollama vision |
| ESP32-CAM web video streaming | — | active | P3 | Live MJPEG server |
| Arduino platform support | — | planned | P3 | No projects yet |
| STM32 platform support | — | planned | P3 | No projects yet |
| Automated project scaffolding tool | — | planned | P3 | Mentioned in README as coming soon |
| ESP32 host-based unit tests | — | planned | P2 | Blocked; no tests exist yet |
| SLAM / autonomous navigation | — | planned | P3 | Listed as future enhancement |
| Multi-robot swarm coordination | — | planned | P3 | Listed as future enhancement |
| Melody detector firmware (CV + ESP-DL + I2S) | PRD-011 | planned | P2 | XIAO ESP32-S3 Sense + MAX98357A; ROI classifier with INT8 CNN |
| Melody detector training pipeline | PRD-011 | planned | P2 | PyTorch + ESP-PPQ; programmatic + diffusion synthetic data |
| Melody detector validation harness | PRD-011 | planned | P3 | Gemini Robotics-ER 1.6 oracle benchmark |

## Status Key

| Status | Meaning |
|--------|---------|
| active | Implemented and in use |
| planned | Scoped but not yet started |
| in-progress | Work has begun |
| blocked | Depends on something unresolved |
| deprecated | No longer maintained |
