# PRP: Claude API Integration (Phase 3)

**Source**: [IT Troubleshooter PRD — Phase 3](../prds/it-troubleshooter.md#phase-3--claude-api-integration-planned)
**Confidence**: 6/10
**Status**: Planned

## Goal

Integrate the Claude API so the device can analyze command output captured from the target and autonomously decide what commands to run next — closing the human-in-the-loop and enabling AI-driven troubleshooting.

## Requirements

From PRD Phase 3:
- FR3.1: Capture command output from target (via CDC or screen-scraping simulation)
- FR3.2: Forward output to Claude API for analysis
- FR3.3: Execute Claude-suggested next commands via HID injection
- FR3.4: Configurable API key and model via `credentials.h`

## Design Notes

### Output Capture Challenge

The target machine's command output is not automatically sent to the ESP32 — the device emulates a keyboard, not a terminal. Options:

1. **CDC serial relay**: Operator copies output to CDC serial manually (simplest, Phase 3a)
2. **Screen reader**: Not feasible with HID keyboard only (would need USB HID mouse + display capture)
3. **SSH output capture**: With Phase 2 WiFi, SSH to target and capture output natively (preferred, Phase 3b)

Phase 3a (CDC relay) should be implemented first; Phase 3b upgrades the capture mechanism.

### New Component: `claude_client`

```
components/claude_client/
├── CMakeLists.txt
├── idf_component.yml
├── include/claude_client.h
└── claude_client.c
```

Uses `esp_http_client` to POST to `api.anthropic.com/v1/messages`.

Key considerations:
- TLS: Use `esp_tls` with bundled certificates (`CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y`)
- Payload size: Command output may be large — consider chunking or truncation
- Response parsing: Parse JSON response to extract `content[0].text`
- Model: Default to `claude-haiku-4-5-20251001` for low latency/cost; configurable via `credentials.h`

### Main Flow Update (Phase 3)

```
diagnostic_loop():
  output = read_from_cdc_or_ssh()
  analysis = claude_client_analyze(output, context)
  if analysis.has_command:
    usb_keyboard_type_string(analysis.next_command)
    status_led_set_mode(BIOS_MODE)
  else if analysis.complete:
    status_led_set_mode(COMPLETE)
  else:
    status_led_set_mode(ERROR)
```

## Configuration

Add to `credentials.h`:
```c
#define CLAUDE_API_KEY "sk-ant-..."
#define CLAUDE_MODEL   "claude-haiku-4-5-20251001"
```

## sdkconfig.defaults additions

```
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y
CONFIG_ESP_TLS_INSECURE=n
CONFIG_HEAP_TRACING=n
```

## Test Plan

1. Paste sample command output to CDC serial; verify Claude API call is made
2. Verify suggested command is typed via HID on target
3. Test with invalid API key; verify error LED and graceful fallback
4. Test with truncated/oversized output; verify no heap exhaustion

## Related

- [ADR-0008: Phased Delivery](../adrs/0008-phased-delivery.md) — Phase 3 scope
- [PRP: WiFi Connectivity](wifi-connectivity.md) — prerequisite
