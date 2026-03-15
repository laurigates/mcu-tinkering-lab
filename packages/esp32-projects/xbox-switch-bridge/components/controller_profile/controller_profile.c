/**
 * @file controller_profile.c
 * @brief Preset controller profiles and composite device configurations.
 *
 * Each profile captures every type-dependent protocol value so the
 * Switch emulation layer can be driven entirely by swapping a pointer.
 *
 * MAC addresses use RFC 5737 documentation range (00:00:5E:00:53:xx)
 * with unique last bytes per interface to avoid collisions.
 */

#include "controller_profile.h"

/* ── Standard presets (real capability sets) ── */

const controller_profile_t PROFILE_PRO_CONTROLLER = {
    .name = "Pro Controller",
    .type = CONTROLLER_TYPE_PRO,
    .conn_info = CONN_INFO_PRO_USB,
    .mac = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x01},
    .fw_major = 0x04,
    .fw_minor = 0x33, /* 4.51 — matches real Pro Controller */
    .caps = CAPS_PRO_CONTROLLER,
    .body_color = {0x32, 0x32, 0x32},       /* Dark gray */
    .button_color = {0xFF, 0xFF, 0xFF},     /* White */
    .left_grip_color = {0x32, 0x32, 0x32},  /* Dark gray */
    .right_grip_color = {0x32, 0x32, 0x32}, /* Dark gray */
};

const controller_profile_t PROFILE_JOYCON_LEFT = {
    .name = "Joy-Con (L)",
    .type = CONTROLLER_TYPE_JOYCON_L,
    .conn_info = CONN_INFO_JOYCON_L_USB,
    .mac = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x10},
    .fw_major = 0x04,
    .fw_minor = 0x33,
    .caps = CAPS_JOYCON_L,
    .body_color = {0x00, 0x82, 0xE6},       /* Neon blue */
    .button_color = {0x0A, 0x1E, 0x0A},     /* Dark */
    .left_grip_color = {0x00, 0x82, 0xE6},  /* Neon blue */
    .right_grip_color = {0x00, 0x82, 0xE6}, /* Neon blue */
};

const controller_profile_t PROFILE_JOYCON_RIGHT = {
    .name = "Joy-Con (R)",
    .type = CONTROLLER_TYPE_JOYCON_R,
    .conn_info = CONN_INFO_JOYCON_R_USB,
    .mac = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x20},
    .fw_major = 0x04,
    .fw_minor = 0x33,
    .caps = CAPS_JOYCON_R,
    .body_color = {0xFF, 0x34, 0x00},       /* Neon red */
    .button_color = {0x0A, 0x1E, 0x0A},     /* Dark */
    .left_grip_color = {0xFF, 0x34, 0x00},  /* Neon red */
    .right_grip_color = {0xFF, 0x34, 0x00}, /* Neon red */
};

/* ── Optimistic presets (Joy-Con type byte + full capabilities) ──
 *
 * These report as Joy-Con to the Switch but claim ALL input capabilities.
 * If the Switch honors the extra buttons/sticks, we get full-input
 * Joy-Cons in a Grip — two complete controllers from one USB device.
 *
 * If it doesn't work, we lose nothing — just switch back to the
 * standard presets.  Happy little experiments.
 */

const controller_profile_t PROFILE_JOYCON_LEFT_FULL = {
    .name = "Joy-Con (L) [Full]",
    .type = CONTROLLER_TYPE_JOYCON_L,
    .conn_info = CONN_INFO_JOYCON_L_USB,
    .mac = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x11},
    .fw_major = 0x04,
    .fw_minor = 0x33,
    .caps = CAPS_ALL,
    .body_color = {0x00, 0xA0, 0xFF},       /* Bright blue */
    .button_color = {0xFF, 0xFF, 0xFF},     /* White */
    .left_grip_color = {0x00, 0xA0, 0xFF},
    .right_grip_color = {0x00, 0xA0, 0xFF},
};

const controller_profile_t PROFILE_JOYCON_RIGHT_FULL = {
    .name = "Joy-Con (R) [Full]",
    .type = CONTROLLER_TYPE_JOYCON_R,
    .conn_info = CONN_INFO_JOYCON_R_USB,
    .mac = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x21},
    .fw_major = 0x04,
    .fw_minor = 0x33,
    .caps = CAPS_ALL,
    .body_color = {0xFF, 0x50, 0x00},       /* Bright red-orange */
    .button_color = {0xFF, 0xFF, 0xFF},     /* White */
    .left_grip_color = {0xFF, 0x50, 0x00},
    .right_grip_color = {0xFF, 0x50, 0x00},
};

/* ── Composite device configurations ── */

const composite_config_t COMPOSITE_SINGLE_PRO = {
    .name = "Single Pro Controller",
    .usb_pid = 0x2009, /* Pro Controller */
    .product_string = "Pro Controller",
    .interface_count = 1,
    .interfaces = {&PROFILE_PRO_CONTROLLER},
};

const composite_config_t COMPOSITE_GRIP_JOYCON = {
    .name = "Charging Grip (Joy-Con L + R)",
    .usb_pid = 0x200E, /* Charging Grip */
    .product_string = "Joy-Con Charging Grip",
    .interface_count = 2,
    .interfaces = {&PROFILE_JOYCON_LEFT, &PROFILE_JOYCON_RIGHT},
};

const composite_config_t COMPOSITE_GRIP_FULL = {
    .name = "Charging Grip (Full-input Joy-Cons)",
    .usb_pid = 0x200E, /* Charging Grip */
    .product_string = "Joy-Con Charging Grip",
    .interface_count = 2,
    .interfaces = {&PROFILE_JOYCON_LEFT_FULL, &PROFILE_JOYCON_RIGHT_FULL},
};

const composite_config_t COMPOSITE_DUAL_PRO = {
    .name = "Dual Pro Controllers (experimental)",
    .usb_pid = 0x200E, /* Grip PID, but with Pro type bytes */
    .product_string = "Joy-Con Charging Grip",
    .interface_count = 2,
    .interfaces = {&PROFILE_PRO_CONTROLLER, &PROFILE_PRO_CONTROLLER},
};
