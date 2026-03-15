/**
 * @file controller_profile.h
 * @brief Swappable controller identity profiles for Switch emulation.
 *
 * Every type-dependent value in the Switch protocol (type byte, connection
 * info, SPI colors, button capabilities, USB PID) is encapsulated in a
 * controller_profile_t.  Swap profiles to experiment with how the Switch
 * treats different controller identities.
 *
 * Usage:
 *   const controller_profile_t *p = &PROFILE_PRO_CONTROLLER;
 *   // or try: &PROFILE_JOYCON_LEFT, &PROFILE_JOYCON_RIGHT
 *
 * For multi-controller (FR12), use composite_config_t to define a
 * USB device with multiple HID interfaces, each driven by its own profile.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Controller type bytes (SPI 0x6012, sub-command 0x02, USB status) ── */

typedef enum {
    CONTROLLER_TYPE_JOYCON_L = 0x01,
    CONTROLLER_TYPE_JOYCON_R = 0x02,
    CONTROLLER_TYPE_PRO = 0x03,
} controller_type_t;

/* ── Connection info nibbles (byte 1 of 0x30 / 0x21 reports) ──
 *
 * Upper nibble: power/transport
 *   0x0 = BT, no charge     0x8 = USB powered
 *   0x1 = BT, charging      0xE = USB powered (alternate, seen on Pro Con)
 *
 * Lower nibble: controller type
 *   0xE = Pro Controller    0x1 = Joy-Con Left    0x2 = Joy-Con Right
 *
 * Real Pro Controller over USB sends 0x8E.  Joy-Con in Grip send 0x81/0x82.
 */
typedef enum {
    CONN_INFO_PRO_USB = 0x8E,
    CONN_INFO_JOYCON_L_USB = 0x81,
    CONN_INFO_JOYCON_R_USB = 0x82,
} connection_info_t;

/* ── Capability flags ──
 *
 * Declare what the profile *claims* to support.  The Switch may or may not
 * query/use each capability — that's the experiment.  Setting a flag means
 * the emulator will ACK the relevant sub-commands rather than NACK them.
 */
typedef enum {
    CAP_LEFT_STICK = (1 << 0),
    CAP_RIGHT_STICK = (1 << 1),
    CAP_DPAD = (1 << 2),
    CAP_FACE_BUTTONS_LEFT = (1 << 3),  /* Y, X, B, A (right-side byte) */
    CAP_FACE_BUTTONS_RIGHT = (1 << 4), /* unused for now, reserved */
    CAP_SHOULDER_L = (1 << 5),
    CAP_SHOULDER_R = (1 << 6),
    CAP_TRIGGER_ZL = (1 << 7),
    CAP_TRIGGER_ZR = (1 << 8),
    CAP_MINUS = (1 << 9),
    CAP_PLUS = (1 << 10),
    CAP_HOME = (1 << 11),
    CAP_CAPTURE = (1 << 12),
    CAP_LSTICK_CLICK = (1 << 13),
    CAP_RSTICK_CLICK = (1 << 14),
    CAP_SL_SR = (1 << 15), /* Joy-Con side buttons */
    CAP_IMU = (1 << 16),
    CAP_NFC = (1 << 17),
    CAP_IR = (1 << 18),
    CAP_HOME_LIGHT = (1 << 19),
    CAP_RUMBLE = (1 << 20),
} controller_cap_t;

/* Convenience: all capabilities a real Pro Controller has */
#define CAPS_PRO_CONTROLLER                                                                   \
    (CAP_LEFT_STICK | CAP_RIGHT_STICK | CAP_DPAD | CAP_FACE_BUTTONS_LEFT | CAP_SHOULDER_L |  \
     CAP_SHOULDER_R | CAP_TRIGGER_ZL | CAP_TRIGGER_ZR | CAP_MINUS | CAP_PLUS | CAP_HOME |    \
     CAP_CAPTURE | CAP_LSTICK_CLICK | CAP_RSTICK_CLICK | CAP_IMU | CAP_NFC | CAP_HOME_LIGHT | \
     CAP_RUMBLE)

/* Real Joy-Con L capabilities */
#define CAPS_JOYCON_L                                                                      \
    (CAP_LEFT_STICK | CAP_DPAD | CAP_SHOULDER_L | CAP_TRIGGER_ZL | CAP_MINUS | CAP_CAPTURE | \
     CAP_LSTICK_CLICK | CAP_SL_SR | CAP_IMU | CAP_RUMBLE)

/* Real Joy-Con R capabilities */
#define CAPS_JOYCON_R                                                                        \
    (CAP_RIGHT_STICK | CAP_FACE_BUTTONS_LEFT | CAP_SHOULDER_R | CAP_TRIGGER_ZR | CAP_PLUS | \
     CAP_HOME | CAP_RSTICK_CLICK | CAP_SL_SR | CAP_IMU | CAP_NFC | CAP_IR |                \
     CAP_HOME_LIGHT | CAP_RUMBLE)

/* Optimistic: Joy-Con type byte but with ALL input capabilities.
 * "Happy little surprises" — the Switch might just accept it. */
#define CAPS_ALL                                                                              \
    (CAP_LEFT_STICK | CAP_RIGHT_STICK | CAP_DPAD | CAP_FACE_BUTTONS_LEFT | CAP_SHOULDER_L |  \
     CAP_SHOULDER_R | CAP_TRIGGER_ZL | CAP_TRIGGER_ZR | CAP_MINUS | CAP_PLUS | CAP_HOME |    \
     CAP_CAPTURE | CAP_LSTICK_CLICK | CAP_RSTICK_CLICK | CAP_SL_SR | CAP_IMU | CAP_NFC |    \
     CAP_IR | CAP_HOME_LIGHT | CAP_RUMBLE)

/* ── RGB color (body/button/grip) ── */

typedef struct {
    uint8_t r, g, b;
} rgb_color_t;

/* ── Controller profile ── */

typedef struct {
    const char *name; /**< Human-readable label for logging */

    /* Identity */
    controller_type_t type;     /**< Type byte for SPI 0x6012 + sub-cmd 0x02 */
    connection_info_t conn_info; /**< Byte 1 of 0x30/0x21 reports */
    uint8_t mac[6];             /**< Unique MAC per interface */
    uint8_t fw_major;           /**< Firmware version (sub-cmd 0x02) */
    uint8_t fw_minor;

    /* Capabilities */
    uint32_t caps; /**< Bitmask of controller_cap_t flags */

    /* SPI flash colors (address 0x6050 + 0x6056) */
    rgb_color_t body_color;
    rgb_color_t button_color;
    rgb_color_t left_grip_color;
    rgb_color_t right_grip_color;
} controller_profile_t;

/* ── Preset profiles ── */

/** Standard Pro Controller — matches current single-controller behavior. */
extern const controller_profile_t PROFILE_PRO_CONTROLLER;

/** Joy-Con Left — real capability set. */
extern const controller_profile_t PROFILE_JOYCON_LEFT;

/** Joy-Con Right — real capability set. */
extern const controller_profile_t PROFILE_JOYCON_RIGHT;

/** Joy-Con Left with full input — optimistic experiment. */
extern const controller_profile_t PROFILE_JOYCON_LEFT_FULL;

/** Joy-Con Right with full input — optimistic experiment. */
extern const controller_profile_t PROFILE_JOYCON_RIGHT_FULL;

/* ── Composite device config (FR12 multi-controller) ── */

/** Maximum HID interfaces in a composite device. */
#define COMPOSITE_MAX_INTERFACES 4

/**
 * @brief USB-level composite device configuration.
 *
 * Ties together the device-level USB identity (PID, product string) with
 * per-interface controller profiles.  Each interface gets its own handshake
 * state, input reports, and protocol handling driven by its profile.
 */
typedef struct {
    const char *name;           /**< Config name for logging/selection */
    uint16_t usb_pid;           /**< USB Product ID */
    const char *product_string; /**< USB product string descriptor */

    uint8_t interface_count;                                  /**< Number of HID interfaces (1-4) */
    const controller_profile_t *interfaces[COMPOSITE_MAX_INTERFACES]; /**< Profile per interface */
} composite_config_t;

/* ── Preset composite configs ── */

/** Single Pro Controller — current behavior, backward compatible. */
extern const composite_config_t COMPOSITE_SINGLE_PRO;

/** Charging Grip with real Joy-Con types (L + R). */
extern const composite_config_t COMPOSITE_GRIP_JOYCON;

/** Charging Grip with full-input Joy-Cons — optimistic experiment. */
extern const composite_config_t COMPOSITE_GRIP_FULL;

/** Two Pro Controllers on one USB device — wild experiment. */
extern const composite_config_t COMPOSITE_DUAL_PRO;

/* ── Helpers ── */

/**
 * @brief Check if a profile claims a specific capability.
 */
static inline bool profile_has_cap(const controller_profile_t *p, controller_cap_t cap)
{
    return (p->caps & (uint32_t)cap) != 0;
}

/**
 * @brief Get a profile's type byte for SPI reads and sub-command replies.
 */
static inline uint8_t profile_type_byte(const controller_profile_t *p)
{
    return (uint8_t)p->type;
}

/**
 * @brief Get a profile's connection info byte for input/sub-command reports.
 */
static inline uint8_t profile_conn_info(const controller_profile_t *p)
{
    return (uint8_t)p->conn_info;
}

#ifdef __cplusplus
}
#endif
