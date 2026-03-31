/**
 * @file switch_pro_usb.c
 * @brief Nintendo Switch Pro Controller USB HID emulation.
 *
 * Implements the wired Pro Controller protocol using TinyUSB on ESP32-S3.
 * Reference: https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 *
 * Protocol overview:
 *   1. Switch sends 0x80 USB commands for handshake (status, baud, force USB)
 *   2. Controller responds with 0x81 replies
 *   3. Switch sends 0x01 sub-commands (device info, SPI reads, LED config)
 *   4. Controller responds with 0x21 sub-command replies
 *   5. After handshake, controller sends 0x30 input reports at ~125 Hz
 */

#include "switch_pro_usb.h"

#include <string.h>

#include "class/hid/hid_device.h"
#include "device/usbd.h"
#include "esp_log.h"
#include "tinyusb.h"

static const char *TAG = "switch_pro_usb";

/* Nintendo Switch Pro Controller identifiers */
#define SWITCH_PRO_VID 0x057E
#define SWITCH_PRO_PID 0x2009

/* Report IDs: Controller → Switch (input) */
#define REPORT_ID_INPUT 0x30        /* Standard input report */
#define REPORT_ID_SUBCMD_REPLY 0x21 /* Sub-command reply */
#define REPORT_ID_USB_REPLY 0x81    /* USB command reply */

/* Report IDs: Switch → Controller (output) */
#define REPORT_ID_SUBCMD 0x01  /* Sub-command request */
#define REPORT_ID_RUMBLE 0x10  /* Rumble-only */
#define REPORT_ID_USB_CMD 0x80 /* USB command */

/* USB 0x80 sub-commands (Switch → Controller) */
#define USB_CMD_STATUS 0x01
#define USB_CMD_HANDSHAKE 0x02
#define USB_CMD_HIGH_SPEED 0x03
#define USB_CMD_FORCE_USB 0x04
#define USB_CMD_DISABLE_USB 0x05

/* Fake MAC address for the emulated controller */
static const uint8_t fake_mac[6] = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x01};

/* Handshake state */
static volatile bool s_handshake_complete = false;
/* Setup phase: Switch sends subcmds (device info, SPI reads, set mode, player lights)
 * after the 0x80 handshake. Don't send 0x30 input reports until setup is done —
 * they steal the HID IN endpoint from 0x21 sub-command replies. */
static volatile bool s_setup_complete = false;
/* Accessed from core 1 only: bridge_task (send_report) and TinyUSB task (set_report_cb).
 * Both are pinned to core 1, so no cross-core synchronization needed. */
static uint8_t s_timer_counter = 0;

/**
 * HID Report Descriptor matching the real Switch Pro Controller.
 *
 * Captured from an actual Pro Controller. The Switch largely ignores this
 * descriptor and uses its own vendor protocol, but it must be present
 * and structurally valid for enumeration to succeed.
 *
 * Source: https://gist.github.com/spacemeowx2/22171913a36721501e42f14f1fd81633
 */
static const uint8_t s_hid_report_descriptor[] = {
    0x05, 0x01, /* Usage Page (Generic Desktop) */
    0x15, 0x00, /* Logical Minimum (0) */
    0x09, 0x04, /* Usage (Joystick) */
    0xA1, 0x01, /* Collection (Application) */
    /* Report 0x30: Standard input */
    0x85, 0x30,                   /*   Report ID (0x30) */
    0x05, 0x01,                   /*   Usage Page (Generic Desktop) */
    0x05, 0x09,                   /*   Usage Page (Button) */
    0x19, 0x01,                   /*   Usage Minimum (1) */
    0x29, 0x0A,                   /*   Usage Maximum (10) */
    0x15, 0x00,                   /*   Logical Minimum (0) */
    0x25, 0x01,                   /*   Logical Maximum (1) */
    0x75, 0x01,                   /*   Report Size (1) */
    0x95, 0x0A,                   /*   Report Count (10) */
    0x55, 0x00,                   /*   Unit Exponent (0) */
    0x65, 0x00,                   /*   Unit (None) */
    0x81, 0x02,                   /*   Input (Data, Variable, Absolute) */
    0x05, 0x09,                   /*   Usage Page (Button) */
    0x19, 0x0B,                   /*   Usage Minimum (11) */
    0x29, 0x0E,                   /*   Usage Maximum (14) */
    0x15, 0x00,                   /*   Logical Minimum (0) */
    0x25, 0x01,                   /*   Logical Maximum (1) */
    0x75, 0x01,                   /*   Report Size (1) */
    0x95, 0x04,                   /*   Report Count (4) */
    0x81, 0x02,                   /*   Input (Data, Variable, Absolute) */
    0x75, 0x01,                   /*   Report Size (1) */
    0x95, 0x02,                   /*   Report Count (2) */
    0x81, 0x03,                   /*   Input (Constant) - padding */
    0x0B, 0x01, 0x00, 0x01, 0x00, /*   Usage (Generic Desktop:Pointer) */
    0xA1, 0x00,                   /*   Collection (Physical) */
    0x0B, 0x30, 0x00, 0x01, 0x00, /*     Usage (X) */
    0x0B, 0x31, 0x00, 0x01, 0x00, /*     Usage (Y) */
    0x0B, 0x32, 0x00, 0x01, 0x00, /*     Usage (Z) */
    0x0B, 0x35, 0x00, 0x01, 0x00, /*     Usage (Rz) */
    0x15, 0x00,                   /*     Logical Minimum (0) */
    0x27, 0xFF, 0xFF, 0x00, 0x00, /*     Logical Maximum (65535) */
    0x75, 0x10,                   /*     Report Size (16) */
    0x95, 0x04,                   /*     Report Count (4) */
    0x81, 0x02,                   /*     Input (Data, Variable, Absolute) */
    0xC0,                         /*   End Collection (Physical) */
    0x0B, 0x39, 0x00, 0x01, 0x00, /*   Usage (Hat Switch) */
    0x15, 0x00,                   /*   Logical Minimum (0) */
    0x25, 0x07,                   /*   Logical Maximum (7) */
    0x35, 0x00,                   /*   Physical Minimum (0) */
    0x46, 0x3B, 0x01,             /*   Physical Maximum (315) */
    0x65, 0x14,                   /*   Unit (Degrees) */
    0x75, 0x04,                   /*   Report Size (4) */
    0x95, 0x01,                   /*   Report Count (1) */
    0x81, 0x02,                   /*   Input (Data, Variable, Absolute) */
    0x05, 0x09,                   /*   Usage Page (Button) */
    0x19, 0x0F,                   /*   Usage Minimum (15) */
    0x29, 0x12,                   /*   Usage Maximum (18) */
    0x15, 0x00,                   /*   Logical Minimum (0) */
    0x25, 0x01,                   /*   Logical Maximum (1) */
    0x75, 0x01,                   /*   Report Size (1) */
    0x95, 0x04,                   /*   Report Count (4) */
    0x81, 0x02,                   /*   Input (Data, Variable, Absolute) */
    0x75, 0x08,                   /*   Report Size (8) */
    0x95, 0x34,                   /*   Report Count (52) */
    0x81, 0x03,                   /*   Input (Constant) - padding */
    /* Report 0x21: Sub-command reply (input) */
    0x06, 0x00, 0xFF, /*   Usage Page (Vendor Defined) */
    0x85, 0x21,       /*   Report ID (0x21) */
    0x09, 0x01,       /*   Usage (Vendor 1) */
    0x75, 0x08,       /*   Report Size (8) */
    0x95, 0x3F,       /*   Report Count (63) */
    0x81, 0x03,       /*   Input (Constant) */
    /* Report 0x81: USB command reply (input) */
    0x85, 0x81, /*   Report ID (0x81) */
    0x09, 0x02, /*   Usage (Vendor 2) */
    0x75, 0x08, /*   Report Size (8) */
    0x95, 0x3F, /*   Report Count (63) */
    0x81, 0x03, /*   Input (Constant) */
    /* Report 0x01: Sub-command (output) */
    0x85, 0x01, /*   Report ID (0x01) */
    0x09, 0x03, /*   Usage (Vendor 3) */
    0x75, 0x08, /*   Report Size (8) */
    0x95, 0x3F, /*   Report Count (63) */
    0x91, 0x83, /*   Output (Constant, Variable, Volatile) */
    /* Report 0x10: Rumble only (output) */
    0x85, 0x10, /*   Report ID (0x10) */
    0x09, 0x04, /*   Usage (Vendor 4) */
    0x75, 0x08, /*   Report Size (8) */
    0x95, 0x3F, /*   Report Count (63) */
    0x91, 0x83, /*   Output (Constant, Variable, Volatile) */
    /* Report 0x80: USB command (output) */
    0x85, 0x80, /*   Report ID (0x80) */
    0x09, 0x05, /*   Usage (Vendor 5) */
    0x75, 0x08, /*   Report Size (8) */
    0x95, 0x3F, /*   Report Count (63) */
    0x91, 0x83, /*   Output (Constant, Variable, Volatile) */
    /* Report 0x82: (output - unused but present in real descriptor) */
    0x85, 0x82, /*   Report ID (0x82) */
    0x09, 0x06, /*   Usage (Vendor 6) */
    0x75, 0x08, /*   Report Size (8) */
    0x95, 0x3F, /*   Report Count (63) */
    0x91, 0x83, /*   Output (Constant, Variable, Volatile) */
    0xC0,       /* End Collection */
};

/**
 * USB Configuration Descriptor for HID with IN + OUT endpoints.
 *
 * esp_tinyusb only provides default config descriptors for CDC/MSC/NCM,
 * so HID devices must supply their own.
 */
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

static const uint8_t s_config_descriptor[] = {
    /* Configuration descriptor */
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),
    /* HID interface with IN (0x81) and OUT (0x02) interrupt endpoints */
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(s_hid_report_descriptor), 0x02,
                             0x81, 64, 8),
};

/**
 * @brief Fill neutral stick data into a report buffer at the given offset.
 *
 * Center = 0x800 (2048), packed as 12-bit values.
 */
static void fill_neutral_sticks(uint8_t *buf, int offset)
{
    /* Left stick neutral: X=0x800, Y=0x800 */
    buf[offset + 0] = 0x00;
    buf[offset + 1] = 0x08;
    buf[offset + 2] = 0x80;
    /* Right stick neutral: X=0x800, Y=0x800 */
    buf[offset + 3] = 0x00;
    buf[offset + 4] = 0x08;
    buf[offset + 5] = 0x80;
}

/**
 * @brief Handle SPI flash read sub-command (0x10).
 *
 * The Switch reads calibration data and colors from SPI flash via this
 * sub-command. We return sensible defaults so sticks work correctly.
 */
static void handle_spi_read(const uint8_t *data, uint16_t len, uint8_t *reply)
{
    if (len < 16)
        return;

    /* Extract SPI address (little-endian 32-bit) and read length */
    uint32_t addr = data[11] | ((uint32_t)data[12] << 8) | ((uint32_t)data[13] << 16) |
                    ((uint32_t)data[14] << 24);
    uint8_t read_len = data[15];

    ESP_LOGI(TAG, "SPI read: addr=0x%04lX len=%d", (unsigned long)addr, read_len);

    /* Echo address and length in reply */
    memcpy(&reply[14], &data[11], 5);

    /* Fill in known calibration addresses with defaults */
    uint8_t *spi_data = &reply[19];
    if (read_len > 44)
        read_len = 44; /* reply buffer is 63 bytes; spi_data starts at offset 19 */

    switch (addr) {
        case 0x6012: /* Device type (1 byte) */
            if (read_len >= 1) {
                spi_data[0] = 0x03; /* Pro Controller */
            }
            break;

        case 0x6020: /* Factory IMU calibration (24 bytes) */
            if (read_len >= 24) {
                /* Accelerometer origin (6 bytes: X, Y, Z as int16 LE) — zeros = level */
                memset(&spi_data[0], 0x00, 6);
                /* Accelerometer sensitivity (6 bytes: X, Y, Z as int16 LE)
                 * 0x4000 = 16384 = 1g at ±8g range (default) */
                spi_data[6] = 0x00;
                spi_data[7] = 0x40;
                spi_data[8] = 0x00;
                spi_data[9] = 0x40;
                spi_data[10] = 0x00;
                spi_data[11] = 0x40;
                /* Gyroscope origin (6 bytes) — zeros = no drift */
                memset(&spi_data[12], 0x00, 6);
                /* Gyroscope sensitivity (6 bytes: X, Y, Z as int16 LE)
                 * 0x343B = 13371 ≈ sensitivity coefficient for ±2000 dps */
                spi_data[18] = 0x3B;
                spi_data[19] = 0x34;
                spi_data[20] = 0x3B;
                spi_data[21] = 0x34;
                spi_data[22] = 0x3B;
                spi_data[23] = 0x34;
            }
            break;

        case 0x603D: /* Factory left stick calibration (9 bytes) */
            if (read_len >= 9) {
                /* Max above center, center, min below center */
                /* Each triplet: X_high|X_low, Y_high|Y_low packed 12-bit */
                spi_data[0] = 0x00;
                spi_data[1] = 0x07;
                spi_data[2] = 0x70;
                spi_data[3] = 0x00;
                spi_data[4] = 0x08;
                spi_data[5] = 0x80;
                spi_data[6] = 0x00;
                spi_data[7] = 0x07;
                spi_data[8] = 0x70;
            }
            break;

        case 0x6046: /* Factory right stick calibration (9 bytes) */
            if (read_len >= 9) {
                spi_data[0] = 0x00;
                spi_data[1] = 0x08;
                spi_data[2] = 0x80;
                spi_data[3] = 0x00;
                spi_data[4] = 0x07;
                spi_data[5] = 0x70;
                spi_data[6] = 0x00;
                spi_data[7] = 0x07;
                spi_data[8] = 0x70;
            }
            break;

        case 0x6050: /* Body + button color (6 bytes) */
            if (read_len >= 6) {
                /* Pro Controller default: dark gray body, dark buttons */
                spi_data[0] = 0x32;
                spi_data[1] = 0x32;
                spi_data[2] = 0x32;
                spi_data[3] = 0xFF;
                spi_data[4] = 0xFF;
                spi_data[5] = 0xFF;
            }
            break;

        case 0x8010: /* User left stick calibration */
            /* Magic byte 0xB2 at offset 0 indicates valid user cal.
             * Return 0xFF (no user cal) so the Switch uses factory cal. */
            if (read_len >= 1) {
                spi_data[0] = 0xFF;
            }
            break;

        case 0x801B: /* User right stick calibration */
            /* Same magic byte convention as left stick */
            if (read_len >= 1) {
                spi_data[0] = 0xFF;
            }
            break;

        case 0x8026: /* User IMU calibration */
            if (read_len >= 1) {
                spi_data[0] = 0xFF;
            }
            break;

        default:
            /* Return all zeros for unknown addresses */
            break;
    }
}

/**
 * @brief Build a 0x81 response for the USB handshake.
 *
 * The Switch sends 0x80 commands and expects 0x81 responses:
 * - 0x80 0x01 (Status): Reply with MAC + controller type
 * - 0x80 0x02 (Handshake): Reply with ack
 * - 0x80 0x03 (High speed): Reply with ack (UART baud switch, N/A for USB)
 * - 0x80 0x04 (Force USB): No response needed; marks handshake done
 */
static void handle_usb_cmd(const uint8_t *data, uint16_t len)
{
    if (len < 2)
        return;

    uint8_t subcmd = data[1];
    uint8_t response[63] = {0};

    switch (subcmd) {
        case USB_CMD_STATUS:
            ESP_LOGI(TAG, "USB CMD: Status request");
            response[0] = 0x01;
            response[1] = 0x00; /* Padding */
            response[2] = 0x03; /* Pro Controller type */
            memcpy(&response[3], fake_mac, 6);
            tud_hid_report(REPORT_ID_USB_REPLY, response, sizeof(response));
            break;

        case USB_CMD_HANDSHAKE:
            ESP_LOGI(TAG, "USB CMD: Handshake");
            response[0] = 0x02;
            tud_hid_report(REPORT_ID_USB_REPLY, response, sizeof(response));
            break;

        case USB_CMD_HIGH_SPEED:
            ESP_LOGI(TAG, "USB CMD: High speed mode");
            response[0] = 0x03;
            tud_hid_report(REPORT_ID_USB_REPLY, response, sizeof(response));
            break;

        case USB_CMD_FORCE_USB:
            ESP_LOGI(TAG, "USB CMD: Force USB - handshake complete!");
            s_handshake_complete = true;
            break;

        case USB_CMD_DISABLE_USB:
            /* Switch sends 0x05 early in enumeration. No response expected —
             * sending one stalls the handshake. Just log and continue. */
            ESP_LOGI(TAG, "USB CMD: Disable USB timeout (no reply)");
            break;

        default:
            ESP_LOGW(TAG, "USB CMD: Unknown subcmd 0x%02X", subcmd);
            break;
    }
}

/**
 * @brief Handle sub-command requests (report ID 0x01/0x10).
 *
 * The Switch sends sub-commands to configure the controller (LEDs, IMU, etc).
 * We reply with a 0x21 report containing button/stick state + sub-command ACK.
 */
static void handle_subcommand(const uint8_t *data, uint16_t len)
{
    if (len < 11)
        return;

    uint8_t subcmd_id = data[10];
    ESP_LOGI(TAG, "Subcommand: 0x%02X", subcmd_id);

    /* Build a 0x21 sub-command reply */
    uint8_t reply[63] = {0};
    reply[0] = s_timer_counter++; /* Timer byte */
    reply[1] = 0x8E;              /* Connection info: USB, Pro Controller */
    /* Bytes 2-4: Button state (neutral) */
    reply[2] = 0x00; /* buttons_right */
    reply[3] = 0x00; /* buttons_shared */
    reply[4] = 0x00; /* buttons_left */
    /* Bytes 5-10: Stick data (neutral center) */
    fill_neutral_sticks(reply, 5);
    reply[11] = 0x00;      /* Vibrator input report */
    reply[12] = 0x80;      /* Sub-command ACK: 0x80 = success */
    reply[13] = subcmd_id; /* Echo back the sub-command ID */

    /* Some sub-commands need specific data in the reply */
    switch (subcmd_id) {
        case 0x02:            /* Request device info */
            reply[14] = 0x04; /* Firmware version major */
            reply[15] = 0x33; /* Firmware version minor */
            reply[16] = 0x03; /* Controller type: Pro Controller */
            reply[17] = 0x02; /* Unknown */
            memcpy(&reply[18], fake_mac, 6);
            reply[24] = 0x01; /* Unknown */
            reply[25] = 0x02; /* Use SPI colors */
            break;

        case 0x03: /* Set input report mode */
            /* ACK is sufficient */
            break;

        case 0x04: /* Trigger buttons elapsed time */
            /* Return zeros for all trigger timers */
            break;

        case 0x08: /* Set shipment low power state */
            break;

        case 0x10:            /* SPI flash read */
            reply[12] = 0x90; /* SPI read ACK uses 0x90, not generic 0x80 */
            handle_spi_read(data, len, reply);
            break;

        case 0x30: /* Set player lights — last setup command */
            ESP_LOGI(TAG, "Player lights: 0x%02X", data[11]);
            if (!s_setup_complete) {
                s_setup_complete = true;
                ESP_LOGI(TAG, "Setup complete — starting 0x30 input reports");
            }
            break;

        case 0x38: /* Set HOME light */
            break;

        case 0x40: /* Enable IMU (6-axis) */
            ESP_LOGI(TAG, "IMU enable: %d", data[11]);
            break;

        case 0x41: /* Set IMU sensitivity */
            break;

        case 0x48: /* Enable vibration */
            ESP_LOGI(TAG, "Vibration enable: %d", data[11]);
            break;

        default:
            ESP_LOGI(TAG, "Unhandled subcommand 0x%02X", subcmd_id);
            reply[14] = 0x03; /* NACK: unknown subcommand */
            break;
    }

    tud_hid_report(REPORT_ID_SUBCMD_REPLY, reply, sizeof(reply));
}

/*--- TinyUSB Callbacks ---*/
// Note: tud_mount_cb and tud_umount_cb are provided by esp_tinyusb with
// strong symbols — do not redefine them here.

/**
 * @brief TinyUSB HID report descriptor callback.
 */
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_hid_report_descriptor;
}

/**
 * @brief TinyUSB HID GET_REPORT callback.
 */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_type;
    ESP_LOGI(TAG, "GET_REPORT id=0x%02X len=%d", report_id, reqlen);
    memset(buffer, 0, reqlen);
    return reqlen;
}

/**
 * @brief TinyUSB HID SET_REPORT callback - handles incoming commands.
 *
 * Called from two paths with different conventions:
 *   - Control pipe (SET_REPORT): report_id is extracted from wValue,
 *     buffer starts AFTER the report ID byte.
 *   - Interrupt OUT endpoint: report_id is always 0, buffer contains
 *     the full packet INCLUDING the report ID as the first byte.
 */
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_type;

    uint8_t full_pkt[64];
    uint16_t total_len;

    if (report_id == 0 && bufsize > 0) {
        /* Interrupt OUT: report ID is first byte of buffer */
        report_id = buffer[0];
        total_len = (bufsize < 64) ? bufsize : 64;
        memcpy(full_pkt, buffer, total_len);
    } else {
        /* Control pipe: report ID already extracted, prepend it */
        full_pkt[0] = report_id;
        uint16_t copy_len = (bufsize < 63) ? bufsize : 63;
        memcpy(&full_pkt[1], buffer, copy_len);
        total_len = copy_len + 1;
    }

    ESP_LOGI(TAG, "SET_REPORT id=0x%02X len=%d", report_id, (int)total_len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, full_pkt, total_len, ESP_LOG_DEBUG);

    if (report_id == REPORT_ID_USB_CMD) {
        handle_usb_cmd(full_pkt, total_len);
    } else if (report_id == REPORT_ID_SUBCMD || report_id == REPORT_ID_RUMBLE) {
        handle_subcommand(full_pkt, total_len);
    } else {
        ESP_LOGW(TAG, "Unknown report_id 0x%02X", report_id);
    }
}

/*--- Public API ---*/

esp_err_t switch_pro_usb_init(void)
{
    ESP_LOGI(TAG, "Initializing Switch Pro Controller USB HID");

    static const tusb_desc_device_t device_desc = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = SWITCH_PRO_VID,
        .idProduct = SWITCH_PRO_PID,
        .bcdDevice = 0x0200,
        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,
        .bNumConfigurations = 0x01,
    };

    static const char *string_desc[] = {
        "\x09\x04",           /* 0: Language (English) */
        "Nintendo Co., Ltd.", /* 1: Manufacturer */
        "Pro Controller",     /* 2: Product */
        "000000000001",       /* 3: Serial */
    };

    const tinyusb_config_t tusb_cfg = {
        .task =
            {
                .size = 4096,
                .priority = 5,
                .xCoreID = 1, /* Core 0 reserved for BTstack; USB HID runs on core 1 */
            },
        .descriptor =
            {
                .device = &device_desc,
                .full_speed_config = s_config_descriptor,
                .string = string_desc,
                .string_count = 4,
            },
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "USB HID initialized, waiting for Switch handshake...");
    return ESP_OK;
}

bool switch_pro_usb_send_report(const switch_pro_input_t *input)
{
    if (!tud_mounted() || !s_handshake_complete || !s_setup_complete)
        return false;

    if (!tud_hid_ready())
        return false; /* IN endpoint busy (likely sending 0x21 reply) — skip this cycle */

    /*
     * Standard input report (0x30), 63 bytes after report ID:
     *   Byte 0:    Timer (increments each report)
     *   Byte 1:    Connection info (0x8E = USB, Pro Controller)
     *   Byte 2:    buttons_right (Y, X, B, A, R, ZR)
     *   Byte 3:    buttons_shared (-, +, RStick, LStick, Home, Capture)
     *   Byte 4:    buttons_left (DPad, L, ZL)
     *   Bytes 5-7: Left stick (12-bit X, 12-bit Y, packed)
     *   Bytes 8-10: Right stick (12-bit X, 12-bit Y, packed)
     *   Byte 11:   Vibrator input report
     *   Bytes 12+: IMU data (zeros when IMU disabled)
     */
    uint8_t report[63] = {0};
    report[0] = s_timer_counter++;
    report[1] = 0x8E; /* USB powered, Pro Controller, full report */
    report[2] = input->buttons_right;
    report[3] = input->buttons_shared;
    report[4] = input->buttons_left;

    /* Pack left stick: 12-bit X and Y into 3 bytes
     * Byte 5: X[7:0]
     * Byte 6: Y[3:0] | X[11:8]
     * Byte 7: Y[11:4]
     */
    report[5] = input->lx & 0xFF;
    report[6] = ((input->ly & 0x0F) << 4) | ((input->lx >> 8) & 0x0F);
    report[7] = (input->ly >> 4) & 0xFF;

    /* Pack right stick: same 12-bit packing */
    report[8] = input->rx & 0xFF;
    report[9] = ((input->ry & 0x0F) << 4) | ((input->rx >> 8) & 0x0F);
    report[10] = (input->ry >> 4) & 0xFF;

    return tud_hid_report(REPORT_ID_INPUT, report, sizeof(report));
}

bool switch_pro_usb_is_mounted(void)
{
    return tud_mounted();
}

bool switch_pro_usb_is_ready(void)
{
    return s_handshake_complete && s_setup_complete && tud_mounted();
}
