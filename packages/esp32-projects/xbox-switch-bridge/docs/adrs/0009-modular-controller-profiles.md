# ADR-0009: Modular Controller Profile Abstraction

**Status**: Accepted
**Date**: 2026-03-15
**Confidence**: High

## Context

ADR-0003 established Pro Controller emulation with hardcoded type bytes (`0x03`), connection info (`0x8E`), SPI color data, and MAC address throughout `switch_pro_usb.c`. This works for a single controller but blocks FR12 (multi-controller support), which requires:

1. **Multiple controller identities** on one USB device (e.g., Joy-Con L + R on a Charging Grip PID)
2. **Per-interface protocol responses** — each HID interface must report its own type byte, MAC, colors, and firmware version
3. **Easy experimentation** — swapping controller types should be a one-line change, not a scattershot edit across protocol handlers

Additionally, we want to explore optimistic capability reporting: declaring buttons and features that a real controller of that type wouldn't have, to see if the Switch processes them anyway.

## Decision

Introduce a **`controller_profile`** component that encapsulates all type-dependent protocol values in a single struct:

```c
typedef struct {
    const char *name;
    controller_type_t type;      // 0x01 JC-L, 0x02 JC-R, 0x03 Pro
    connection_info_t conn_info; // Byte 1 of 0x30/0x21 reports
    uint8_t mac[6];              // Unique per interface
    uint8_t fw_major, fw_minor;
    uint32_t caps;               // Bitmask of controller_cap_t flags
    rgb_color_t body_color, button_color, left_grip_color, right_grip_color;
} controller_profile_t;
```

A `composite_config_t` ties together the USB device-level identity (PID, product string) with per-interface profiles:

```c
typedef struct {
    const char *name;
    uint16_t usb_pid;
    const char *product_string;
    uint8_t interface_count;
    const controller_profile_t *interfaces[4];
} composite_config_t;
```

### Preset profiles

| Profile | Type byte | Capabilities | Purpose |
|---------|-----------|-------------|---------|
| `PROFILE_PRO_CONTROLLER` | 0x03 | Real Pro set | Backward compatible baseline |
| `PROFILE_JOYCON_LEFT` | 0x01 | Real JC-L set | Safe Charging Grip emulation |
| `PROFILE_JOYCON_RIGHT` | 0x02 | Real JC-R set | Safe Charging Grip emulation |
| `PROFILE_JOYCON_LEFT_FULL` | 0x01 | ALL capabilities | Optimistic experiment |
| `PROFILE_JOYCON_RIGHT_FULL` | 0x02 | ALL capabilities | Optimistic experiment |

### Preset composite configs

| Config | PID | Interfaces | Experiment |
|--------|-----|-----------|------------|
| `COMPOSITE_SINGLE_PRO` | 0x2009 | 1× Pro | Current behavior |
| `COMPOSITE_GRIP_JOYCON` | 0x200E | JC-L + JC-R | Real Charging Grip |
| `COMPOSITE_GRIP_FULL` | 0x200E | JC-L(full) + JC-R(full) | Full-input Joy-Cons |
| `COMPOSITE_DUAL_PRO` | 0x200E | Pro + Pro | Two Pro Controllers |

### Optimistic capability design

The capability flags (`controller_cap_t`) are intentionally decoupled from the type byte. A profile can claim to be a Joy-Con L (type `0x01`) while declaring `CAPS_ALL` — both sticks, all face buttons, NFC, IR, home light, rumble. The emulation layer ACKs the corresponding sub-commands rather than NACKing them.

Rationale: the Switch may process inputs it receives regardless of what controller type it believes is connected. If it does, we get two fully independent controllers from a single Charging Grip PID. If it doesn't, we fall back to the safe presets with no risk.

## Consequences

- **Every hardcoded type-dependent value** in `switch_pro_usb.c` (type bytes, connection info, MAC, colors, firmware version) becomes a `profile->` dereference — next step in FR12 integration.
- **New experiments** require only defining a new `controller_profile_t` and `composite_config_t` — no protocol code changes.
- **Capability flags** can gate sub-command ACK behavior (e.g., don't ACK NFC sub-commands if `CAP_NFC` is not set).
- **Per-interface MAC addresses** prevent the Switch from treating two interfaces as the same controller.
- **No runtime overhead** — profiles are `const` structs, resolved at link time.

## Alternatives Considered

- **`#define` / `Kconfig` compile-time selection**: Only allows one type per build. Rejected — we need per-interface selection for multi-controller.
- **Runtime JSON/NVS configuration**: Over-engineered for the experiment phase. Can be added later if needed. Rejected for now.
- **Subclassing with function pointers**: More flexible but adds indirection and complexity for what is fundamentally a data-driven difference. Rejected.
