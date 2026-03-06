/**
 * @file switch_pro_usb.c
 * @brief Nintendo Switch Pro Controller USB HID emulation.
 *
 * Implements the wired Pro Controller protocol using TinyUSB on ESP32-S3.
 * Reference: https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 */

#include "switch_pro_usb.h"

#include <string.h>

#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

static const char *TAG = "switch_pro_usb";

/* Nintendo Switch Pro Controller identifiers */
#define SWITCH_PRO_VID 0x057E
#define SWITCH_PRO_PID 0x2009

/* Report IDs used in the Pro Controller protocol */
#define REPORT_ID_INPUT 0x30
#define REPORT_ID_USB_CMD 0x80
#define REPORT_ID_SUBCOMMAND 0x21

/* USB 0x80 sub-commands (Switch → Controller) */
#define USB_CMD_STATUS 0x01
#define USB_CMD_HANDSHAKE 0x02
#define USB_CMD_HIGH_SPEED 0x03
#define USB_CMD_FORCE_USB 0x04

/* Fake MAC address for the emulated controller */
static const uint8_t fake_mac[6] = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x01};

/* Handshake state */
static volatile bool s_handshake_complete = false;
static uint8_t s_timer_counter = 0;

/**
 * HID Report Descriptor for the Switch Pro Controller.
 *
 * This is a vendor-specific descriptor that the Switch expects. It defines
 * a 63-byte input report and a 63-byte output report, both vendor-defined.
 */
static const uint8_t s_hid_report_descriptor[] = {
    0x05, 0x01,        /* Usage Page (Generic Desktop) */
    0x15, 0x00,        /* Logical Minimum (0) */
    0x09, 0x04,        /* Usage (Joystick) */
    0xA1, 0x01,        /* Collection (Application) */
    0x85, 0x30,        /*   Report ID (0x30) - Standard input */
    0x05, 0x01,        /*   Usage Page (Generic Desktop) */
    0x05, 0x09,        /*   Usage Page (Button) */
    0x19, 0x01,        /*   Usage Minimum (1) */
    0x29, 0x0A,        /*   Usage Maximum (10) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x01,        /*   Logical Maximum (1) */
    0x75, 0x01,        /*   Report Size (1) */
    0x95, 0x0A,        /*   Report Count (10) */
    0x81, 0x02,        /*   Input (Data, Variable, Absolute) */
    0x05, 0x01,        /*   Usage Page (Generic Desktop) */
    0x09, 0x39,        /*   Usage (Hat Switch) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x07,        /*   Logical Maximum (7) */
    0x75, 0x04,        /*   Report Size (4) */
    0x95, 0x01,        /*   Report Count (1) */
    0x81, 0x42,        /*   Input (Data, Variable, Null) */
    0x05, 0x01,        /*   Usage Page (Generic Desktop) */
    0x09, 0x30,        /*   Usage (X) */
    0x09, 0x31,        /*   Usage (Y) */
    0x09, 0x33,        /*   Usage (Rx) */
    0x09, 0x34,        /*   Usage (Ry) */
    0x16, 0x00, 0x00,  /*   Logical Minimum (0) */
    0x27, 0xFF, 0x0F, 0x00, 0x00, /* Logical Maximum (4095) */
    0x75, 0x10,        /*   Report Size (16) */
    0x95, 0x04,        /*   Report Count (4) */
    0x81, 0x02,        /*   Input (Data, Variable, Absolute) */
    /* Vendor-defined output for 0x80 commands and subcommands */
    0x85, 0x21,        /*   Report ID (0x21) */
    0x09, 0x01,        /*   Usage (Vendor Defined) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x3F,        /*   Report Count (63) */
    0x81, 0x02,        /*   Input (Data, Variable, Absolute) */
    0x85, 0x80,        /*   Report ID (0x80) */
    0x09, 0x01,        /*   Usage (Vendor Defined) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x3F,        /*   Report Count (63) */
    0x81, 0x02,        /*   Input (Data, Variable, Absolute) */
    0x85, 0x01,        /*   Report ID (0x01) */
    0x09, 0x01,        /*   Usage (Vendor Defined) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x3F,        /*   Report Count (63) */
    0x91, 0x02,        /*   Output (Data, Variable, Absolute) */
    0x85, 0x10,        /*   Report ID (0x10) */
    0x09, 0x01,        /*   Usage (Vendor Defined) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x3F,        /*   Report Count (63) */
    0x91, 0x02,        /*   Output (Data, Variable, Absolute) */
    0x85, 0x80,        /*   Report ID (0x80) */
    0x09, 0x01,        /*   Usage (Vendor Defined) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x3F,        /*   Report Count (63) */
    0x91, 0x02,        /*   Output (Data, Variable, Absolute) */
    0xC0,              /* End Collection */
};

/**
 * @brief Build a 0x80 response for the USB handshake.
 *
 * The Switch sends 0x80 commands and expects specific responses:
 * - 0x80 0x01 (Status): Reply with MAC + status info
 * - 0x80 0x02 (Handshake): Reply with ack
 * - 0x80 0x03 (High speed): Reply with ack
 * - 0x80 0x04 (Force USB): Reply with ack, marks handshake done
 */
static void handle_usb_cmd(const uint8_t *data, uint16_t len) {
    if (len < 2) return;

    uint8_t subcmd = data[1];
    uint8_t response[64] = {0};
    response[0] = REPORT_ID_USB_CMD;

    switch (subcmd) {
        case USB_CMD_STATUS:
            /* Reply: 0x80 0x01 <padding> <type> <mac[6]> */
            ESP_LOGI(TAG, "USB CMD: Status request");
            response[1] = 0x01;
            response[2] = 0x00;  /* Padding */
            response[3] = 0x03;  /* Pro Controller type */
            memcpy(&response[4], fake_mac, 6);
            tud_hid_report(REPORT_ID_USB_CMD, &response[1], 63);
            break;

        case USB_CMD_HANDSHAKE:
            ESP_LOGI(TAG, "USB CMD: Handshake");
            response[1] = 0x02;
            tud_hid_report(REPORT_ID_USB_CMD, &response[1], 63);
            break;

        case USB_CMD_HIGH_SPEED:
            ESP_LOGI(TAG, "USB CMD: High speed mode");
            response[1] = 0x03;
            tud_hid_report(REPORT_ID_USB_CMD, &response[1], 63);
            break;

        case USB_CMD_FORCE_USB:
            ESP_LOGI(TAG, "USB CMD: Force USB - handshake complete!");
            response[1] = 0x04;
            tud_hid_report(REPORT_ID_USB_CMD, &response[1], 63);
            s_handshake_complete = true;
            break;

        default:
            ESP_LOGW(TAG, "USB CMD: Unknown subcmd 0x%02X", subcmd);
            response[1] = subcmd;
            tud_hid_report(REPORT_ID_USB_CMD, &response[1], 63);
            break;
    }
}

/**
 * @brief Handle subcommand requests (report ID 0x01/0x10).
 *
 * The Switch sends subcommands to configure the controller (LEDs, IMU, etc).
 * We reply with a minimal 0x21 acknowledgment.
 */
static void handle_subcommand(const uint8_t *data, uint16_t len) {
    if (len < 11) return;

    uint8_t subcmd_id = data[10];
    ESP_LOGI(TAG, "Subcommand: 0x%02X", subcmd_id);

    /* Build a 0x21 subcommand reply */
    uint8_t reply[63] = {0};
    reply[0] = s_timer_counter++;   /* Timer byte */
    reply[1] = 0x81;               /* Connection info: USB powered */
    /* Bytes 2-12: Stick + button data (zeros = neutral) */
    reply[2] = 0x00;               /* buttons_right */
    reply[3] = 0x00;               /* buttons_shared */
    reply[4] = 0x00;               /* buttons_left */
    /* Left stick neutral (center = 0x800 = 2048) */
    reply[5] = 0x00;               /* LX low */
    reply[6] = 0x08;               /* LX high + LY low nibble */
    reply[7] = 0x80;               /* LY */
    /* Right stick neutral */
    reply[8] = 0x00;               /* RX low */
    reply[9] = 0x08;               /* RX high + RY low nibble */
    reply[10] = 0x80;              /* RY */
    reply[11] = 0x00;              /* Vibrator input report */
    reply[12] = 0x80;              /* Subcommand ACK: 0x80 = success */
    reply[13] = subcmd_id;         /* Echo back the subcommand ID */

    /* Some subcommands need specific data in the reply */
    switch (subcmd_id) {
        case 0x02:  /* Request device info */
            reply[14] = 0x04;  /* Firmware version major */
            reply[15] = 0x00;  /* Firmware version minor */
            reply[16] = 0x03;  /* Pro Controller type */
            reply[17] = 0x02;  /* Unknown */
            memcpy(&reply[18], fake_mac, 6);
            reply[24] = 0x03;  /* Unknown */
            reply[25] = 0x02;  /* Color - use SPI colors */
            break;

        case 0x10:  /* SPI flash read */
            /* Return zeros for any SPI read - we have no flash data */
            memcpy(&reply[14], &data[11], 5);  /* Echo address + length */
            break;

        case 0x03:  /* Set input report mode */
            /* ACK is sufficient */
            break;

        case 0x04:  /* Trigger buttons elapsed time */
            break;

        case 0x08:  /* Set shipment low power state */
            break;

        case 0x30:  /* Set player lights */
            ESP_LOGI(TAG, "Player lights: 0x%02X", data[11]);
            break;

        case 0x38:  /* Set HOME light */
            break;

        case 0x40:  /* Enable IMU */
            ESP_LOGI(TAG, "IMU enable: %d", data[11]);
            break;

        case 0x41:  /* Set IMU sensitivity */
            break;

        case 0x48:  /* Enable vibration */
            ESP_LOGI(TAG, "Vibration enable: %d", data[11]);
            break;

        default:
            ESP_LOGD(TAG, "Unhandled subcommand 0x%02X", subcmd_id);
            break;
    }

    tud_hid_report(REPORT_ID_SUBCOMMAND, reply, sizeof(reply));
}

/*--- TinyUSB Callbacks ---*/

/**
 * @brief TinyUSB HID GET_REPORT callback.
 */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type, uint8_t *buffer,
                                uint16_t reqlen) {
    (void)instance;
    (void)report_type;
    ESP_LOGD(TAG, "GET_REPORT id=0x%02X len=%d", report_id, reqlen);
    memset(buffer, 0, reqlen);
    return reqlen;
}

/**
 * @brief TinyUSB HID SET_REPORT callback - handles incoming commands.
 */
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize) {
    (void)instance;
    (void)report_type;

    ESP_LOGD(TAG, "SET_REPORT id=0x%02X len=%d", report_id, bufsize);

    if (report_id == REPORT_ID_USB_CMD) {
        /* Reconstruct the full packet with report ID */
        uint8_t full_pkt[64];
        full_pkt[0] = report_id;
        memcpy(&full_pkt[1], buffer, (bufsize < 63) ? bufsize : 63);
        handle_usb_cmd(full_pkt, bufsize + 1);
    } else if (report_id == 0x01 || report_id == 0x10) {
        uint8_t full_pkt[64];
        full_pkt[0] = report_id;
        memcpy(&full_pkt[1], buffer, (bufsize < 63) ? bufsize : 63);
        handle_subcommand(full_pkt, bufsize + 1);
    } else {
        ESP_LOGW(TAG, "Unknown report_id 0x%02X", report_id);
    }
}

/*--- Public API ---*/

esp_err_t switch_pro_usb_init(void) {
    ESP_LOGI(TAG, "Initializing Switch Pro Controller USB HID");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &(const tusb_desc_device_t){
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
        },
        .string_descriptor = (const char *[]){
            "\x09\x04",          /* 0: Language (English) */
            "Nintendo Co., Ltd.", /* 1: Manufacturer */
            "Pro Controller",    /* 2: Product */
            "000000000001",      /* 3: Serial */
        },
        .string_descriptor_count = 4,
        .external_phy = false,
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB driver install failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "USB HID initialized, waiting for Switch handshake...");
    return ESP_OK;
}

bool switch_pro_usb_send_report(const switch_pro_input_t *input) {
    if (!tud_mounted()) return false;

    /*
     * Standard input report (0x30):
     *   Byte 0:    Timer (increments each report)
     *   Byte 1:    Connection info
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
    report[1] = 0x81;  /* USB powered, Pro Controller */
    report[2] = input->buttons_right;
    report[3] = input->buttons_shared;
    report[4] = input->buttons_left;

    /* Pack left stick: 12-bit X and Y
     * Byte 5: X[7:0]
     * Byte 6: Y[3:0] | X[11:8]
     * Byte 7: Y[11:4]
     */
    report[5] = input->lx & 0xFF;
    report[6] = ((input->ly & 0x0F) << 4) | ((input->lx >> 8) & 0x0F);
    report[7] = (input->ly >> 4) & 0xFF;

    /* Pack right stick: same format */
    report[8] = input->rx & 0xFF;
    report[9] = ((input->ry & 0x0F) << 4) | ((input->rx >> 8) & 0x0F);
    report[10] = (input->ry >> 4) & 0xFF;

    return tud_hid_report(REPORT_ID_INPUT, report, sizeof(report));
}

bool switch_pro_usb_is_ready(void) {
    return s_handshake_complete && tud_mounted();
}
