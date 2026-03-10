# PRP: WiFi Connectivity (Phase 2)

**Source**: [IT Troubleshooter PRD — Phase 2](../prds/it-troubleshooter.md#phase-2--wifi--command-execution-planned)
**Confidence**: 7/10
**Status**: Planned

## Goal

Add WiFi connectivity to the IT Troubleshooter so it can connect to a mobile hotspot on boot, enabling operator-driven command execution on remote hosts.

## Requirements

From PRD Phase 2:
- FR2.1: Connect to pre-configured WiFi hotspot on boot
- FR2.2: Accept operator input via CDC serial
- FR2.3: Inject commands to target via HID keyboard
- FR2.4: NVS-persisted state machine for multi-step workflows
- FR2.5: LED feedback during WiFi and execution phases

## Implementation Approach

### New Component: `wifi_manager`

```
components/wifi_manager/
├── CMakeLists.txt
├── idf_component.yml
├── include/wifi_manager.h
└── wifi_manager.c
```

Responsibilities:
- Connect to SSID/password from `credentials.h`
- Retry with exponential backoff
- Expose `wifi_manager_is_connected()` for polling
- Call `status_led_set_mode(STATUS_LED_WIFI_CONNECTING)` during connection

### NVS State Machine

Store device state in NVS namespace `"it-ts"`:
- `"phase"` (uint8): current phase (1=USB only, 2=WiFi connected, 3=command running)
- `"step"` (uint8): current step within phase
- Persist across power cycles for multi-step workflows

### Main Flow Update

```
app_main():
  nvs_flash_init()
  status_led_init()
  usb_composite_init()
  wait_for_usb_mount()
  wifi_manager_init()
  wait_for_wifi_connected()   ← NEW
  start_command_loop_task()   ← NEW (core 1, replaces CDC echo)
```

### Command Loop Task (core 1)

```
command_loop_task():
  while(true):
    status_led_update()
    if cdc_data_available():
      cmd = usb_cdc_read()
      usb_keyboard_type_string(cmd)   ← inject to target
    vTaskDelay(10ms)
```

## Hardware/Config

- WiFi credentials in `credentials.h`: `WIFI_SSID`, `WIFI_PASS`
- `sdkconfig.defaults` additions needed:
  - `CONFIG_ESP_WIFI_ENABLED=y`
  - `CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=10`
  - `CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM=32`
- Power management must stay disabled (`CONFIG_PM_ENABLE=n`) for USB + WiFi low latency

## Test Plan

1. Build and flash; observe LED transitions: BOOT → USB_READY → WIFI_CONNECTING → DIAGNOSTIC
2. Connect terminal to CDC serial; type a command; verify it appears on target screen via HID
3. Power cycle; verify NVS state restores correctly
4. Disconnect WiFi hotspot; verify reconnection retry and LED feedback

## Related

- [ADR-0007: Core Assignment Strategy](../adrs/0007-core-assignment-strategy.md) — WiFi tasks on core 1
- [ADR-0008: Phased Delivery](../adrs/0008-phased-delivery.md) — Phase 2 scope
