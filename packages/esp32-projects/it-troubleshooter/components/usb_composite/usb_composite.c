/**
 * @file usb_composite.c
 * @brief USB composite device: HID boot-protocol keyboard + CDC-ACM serial.
 *
 * The HID keyboard uses boot protocol (subclass 1, protocol 1) so BIOS
 * firmware can recognize it without parsing arbitrary HID report descriptors.
 *
 * CDC-ACM provides a virtual serial port for OS-level communication.
 * Phase 1 uses CDC-ACM; Phase 4 will replace it with CDC-NCM (USB Ethernet).
 */

#include "usb_composite.h"

#include <string.h>

#include "class/cdc/cdc_device.h"
#include "class/hid/hid_device.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"

static const char *TAG = "usb_composite";

/* Key press/release timing */
#define KEY_PRESS_MS 20
#define KEY_RELEASE_MS 20
#define CHAR_DELAY_MS 10
#define HID_READY_TIMEOUT_MS 50
#define HID_READY_POLL_MS 5

/* HID modifier key bitmask values (USB HID Usage Table, Usage Page 0x07) */
#define HID_MOD_LEFT_SHIFT 0x02

/* ─── USB Descriptors ──────────────────────────────────────────────────────── */

/* Device descriptor: composite device using IAD */
static const tusb_desc_device_t s_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210, /* USB 2.1 for BOS descriptor support */
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,  /* Espressif VID */
    .idProduct = 0x8004, /* Custom PID for IT troubleshooter */
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/*
 * String descriptors. Index 0 is the raw 2-byte USB LANGID (0x0409 = English US),
 * NOT a full USB string descriptor with bLength/bDescriptorType prefix.
 * TinyUSB wraps it into a proper descriptor internally.
 */
static const char *s_string_desc[] = {
    [0] = "\x09\x04", /* LANGID 0x0409: English (US) */
    [1] = "MCU Tinkering Lab",
    [2] = "IT Troubleshooter",
    [3] = "000001",
};

/* HID boot keyboard report descriptor.
 * The paired-byte layout below encodes meaning (each line is one HID item);
 * do not reflow. */
/* clang-format off */
static const uint8_t s_hid_report_desc[] = {
    0x05, 0x01, /* Usage Page (Generic Desktop) */
    0x09, 0x06, /* Usage (Keyboard) */
    0xA1, 0x01, /* Collection (Application) */

    /* Modifier keys (8 bits) */
    0x05, 0x07, /*   Usage Page (Key Codes) */
    0x19, 0xE0, /*   Usage Minimum (Left Control) */
    0x29, 0xE7, /*   Usage Maximum (Right GUI) */
    0x15, 0x00, /*   Logical Minimum (0) */
    0x25, 0x01, /*   Logical Maximum (1) */
    0x75, 0x01, /*   Report Size (1) */
    0x95, 0x08, /*   Report Count (8) */
    0x81, 0x02, /*   Input (Data, Variable, Absolute) */

    /* Reserved byte */
    0x95, 0x01, /*   Report Count (1) */
    0x75, 0x08, /*   Report Size (8) */
    0x81, 0x01, /*   Input (Constant) */

    /* LED output (5 bits: Num/Caps/Scroll/Compose/Kana) */
    0x95, 0x05, /*   Report Count (5) */
    0x75, 0x01, /*   Report Size (1) */
    0x05, 0x08, /*   Usage Page (LEDs) */
    0x19, 0x01, /*   Usage Minimum (Num Lock) */
    0x29, 0x05, /*   Usage Maximum (Kana) */
    0x91, 0x02, /*   Output (Data, Variable, Absolute) */

    /* LED padding (3 bits) */
    0x95, 0x01, /*   Report Count (1) */
    0x75, 0x03, /*   Report Size (3) */
    0x91, 0x01, /*   Output (Constant) */

    /* Key codes (6 bytes) */
    0x95, 0x06, /*   Report Count (6) */
    0x75, 0x08, /*   Report Size (8) */
    0x15, 0x00, /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00, /* Logical Maximum (255) */
    0x05, 0x07, /*   Usage Page (Key Codes) */
    0x19, 0x00, /*   Usage Minimum (0) */
    0x2A, 0xFF, 0x00, /* Usage Maximum (255) */
    0x81, 0x00, /*   Input (Data, Array) */

    0xC0, /* End Collection */
};
/* clang-format on */

/* ─── Configuration Descriptor ─────────────────────────────────────────────── */

/*
 * Configuration descriptor layout:
 *   Interface 0: HID Keyboard (boot protocol)
 *     - IN endpoint (interrupt, 8 bytes, 10ms)
 *   Interface 1-2: CDC-ACM (via IAD)
 *     - Interface 1: CDC Communication (notification endpoint)
 *     - Interface 2: CDC Data (bulk IN + OUT)
 */

#define ITF_NUM_HID 0
#define ITF_NUM_CDC 1
#define ITF_NUM_CDC_DATA 2
#define ITF_NUM_TOTAL 3

#define EPNUM_HID 0x81
#define EPNUM_CDC_NOTIF 0x82
#define EPNUM_CDC_OUT 0x03
#define EPNUM_CDC_IN 0x83

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t s_config_desc[] = {
    /* Configuration descriptor */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
                          100),

    /* HID Keyboard: boot protocol, subclass 1, protocol 1 (keyboard) */
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_KEYBOARD, sizeof(s_hid_report_desc),
                       EPNUM_HID, 8, 10),

    /* CDC-ACM: communication + data interfaces */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

/* ─── TinyUSB Callbacks ────────────────────────────────────────────────────── */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_hid_report_desc;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
    /* Accept LED state reports from host, but no action needed */
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

/* ─── Public API: Init ─────────────────────────────────────────────────────── */

esp_err_t usb_composite_init(void)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &s_device_desc,
        .string_descriptor = s_string_desc,
        .string_descriptor_count = sizeof(s_string_desc) / sizeof(s_string_desc[0]),
        .external_phy = false,
        .configuration_descriptor = s_config_desc,
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "USB composite initialized (HID keyboard + CDC serial)");
    return ESP_OK;
}

bool usb_composite_is_mounted(void)
{
    return tud_mounted();
}

/* ─── Public API: HID Keyboard ─────────────────────────────────────────────── */

esp_err_t usb_keyboard_send_report(uint8_t modifier, const uint8_t keycodes[6])
{
    if (!tud_mounted()) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Wait until previous report is sent, with retry loop */
    int retries = HID_READY_TIMEOUT_MS / HID_READY_POLL_MS;
    while (!tud_hid_ready() && retries-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(HID_READY_POLL_MS));
    }
    if (!tud_hid_ready()) {
        ESP_LOGW(TAG, "HID not ready after %d ms", HID_READY_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    tud_hid_keyboard_report(0, modifier, (uint8_t *)keycodes);
    return ESP_OK;
}

esp_err_t usb_keyboard_press(uint8_t keycode)
{
    return usb_keyboard_press_with_modifier(0, keycode);
}

esp_err_t usb_keyboard_press_with_modifier(uint8_t modifier, uint8_t keycode)
{
    uint8_t keycodes[6] = {keycode, 0, 0, 0, 0, 0};

    /* Key down */
    esp_err_t ret = usb_keyboard_send_report(modifier, keycodes);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(KEY_PRESS_MS));

    /* Key up (empty report) */
    uint8_t empty[6] = {0};
    ret = usb_keyboard_send_report(0, empty);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(KEY_RELEASE_MS));
    return ESP_OK;
}

/**
 * @brief Map ASCII character to HID keycode + modifier.
 * @return true if mapping found, false otherwise.
 */
static bool ascii_to_hid(char c, uint8_t *keycode, uint8_t *modifier)
{
    *modifier = 0;

    if (c >= 'a' && c <= 'z') {
        *keycode = 0x04 + (c - 'a'); /* HID_KEY_A = 0x04 */
        return true;
    }
    if (c >= 'A' && c <= 'Z') {
        *keycode = 0x04 + (c - 'A');
        *modifier = HID_MOD_LEFT_SHIFT;
        return true;
    }
    if (c >= '1' && c <= '9') {
        *keycode = 0x1E + (c - '1'); /* HID_KEY_1 = 0x1E */
        return true;
    }
    if (c == '0') {
        *keycode = 0x27; /* HID_KEY_0 */
        return true;
    }

    /* Special characters */
    switch (c) {
        case '\n':
            *keycode = 0x28;
            return true; /* Enter */
        case '\t':
            *keycode = 0x2B;
            return true; /* Tab */
        case ' ':
            *keycode = 0x2C;
            return true; /* Space */
        case '-':
            *keycode = 0x2D;
            return true;
        case '=':
            *keycode = 0x2E;
            return true;
        case '[':
            *keycode = 0x2F;
            return true;
        case ']':
            *keycode = 0x30;
            return true;
        case '\\':
            *keycode = 0x31;
            return true;
        case ';':
            *keycode = 0x33;
            return true;
        case '\'':
            *keycode = 0x34;
            return true;
        case '`':
            *keycode = 0x35;
            return true;
        case ',':
            *keycode = 0x36;
            return true;
        case '.':
            *keycode = 0x37;
            return true;
        case '/':
            *keycode = 0x38;
            return true;

        /* Shifted characters */
        case '!':
            *keycode = 0x1E;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '@':
            *keycode = 0x1F;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '#':
            *keycode = 0x20;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '$':
            *keycode = 0x21;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '%':
            *keycode = 0x22;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '^':
            *keycode = 0x23;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '&':
            *keycode = 0x24;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '*':
            *keycode = 0x25;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '(':
            *keycode = 0x26;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case ')':
            *keycode = 0x27;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '_':
            *keycode = 0x2D;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '+':
            *keycode = 0x2E;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '{':
            *keycode = 0x2F;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '}':
            *keycode = 0x30;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '|':
            *keycode = 0x31;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case ':':
            *keycode = 0x33;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '"':
            *keycode = 0x34;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '~':
            *keycode = 0x35;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '<':
            *keycode = 0x36;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '>':
            *keycode = 0x37;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;
        case '?':
            *keycode = 0x38;
            *modifier = HID_MOD_LEFT_SHIFT;
            return true;

        default:
            return false;
    }
}

esp_err_t usb_keyboard_type_string(const char *str)
{
    if (!str)
        return ESP_ERR_INVALID_ARG;

    for (const char *p = str; *p; p++) {
        uint8_t keycode, modifier;
        if (ascii_to_hid(*p, &keycode, &modifier)) {
            esp_err_t ret = usb_keyboard_press_with_modifier(modifier, keycode);
            if (ret != ESP_OK)
                return ret;
            vTaskDelay(pdMS_TO_TICKS(CHAR_DELAY_MS));
        } else {
            ESP_LOGW(TAG, "Unmapped char: 0x%02x '%c'", *p, *p);
        }
    }
    return ESP_OK;
}

/* ─── Public API: CDC Serial ───────────────────────────────────────────────── */

int usb_cdc_write(const uint8_t *data, size_t len)
{
    if (!tud_mounted() || !tud_cdc_connected()) {
        return -1;
    }
    uint32_t written = tud_cdc_write(data, len);
    tud_cdc_write_flush();
    return (int)written;
}

int usb_cdc_read(uint8_t *buf, size_t buf_size)
{
    if (!tud_mounted() || !tud_cdc_available()) {
        return 0;
    }
    return (int)tud_cdc_read(buf, buf_size);
}
