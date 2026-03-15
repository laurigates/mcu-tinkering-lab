/**
 * @file main.c
 * @brief Switch USB Proxy — thin USB HID ↔ UART bridge.
 *
 * Flash this once to the ESP32-S3. It presents as a Nintendo Switch
 * Pro Controller (VID 0x057E / PID 0x2009) over USB, and forwards
 * all HID traffic to/from the PC over UART.
 *
 * The PC runs Python code that handles the actual protocol logic,
 * so you can iterate on responses without reflashing.
 *
 * UART frame format (both directions):
 *   [0xAA] [len] [payload...] [checksum]
 *
 *   0xAA     = sync byte
 *   len      = payload length (1 byte, max 65)
 *   payload  = [direction] [report_data...]
 *     direction: 'S' (0x53) = from Switch to PC
 *                'P' (0x50) = from PC to Switch (response)
 *     report_data: raw HID report including report ID (max 64 bytes)
 *   checksum = XOR of all payload bytes (direction + report_data)
 *
 * Hardware:
 *   - USB-C port → connects to Switch dock
 *   - UART1 on GPIO43 (TX) / GPIO44 (RX) → connects to PC via USB-UART adapter
 *   - Baud: 921600 (fast enough for 125 Hz × 64 bytes)
 */

#include <string.h>

#include "class/hid/hid_device.h"
#include "device/usbd.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"

static const char *TAG = "usb_proxy";

/* --- Configuration --- */

#define PROXY_UART_NUM   UART_NUM_1
#define PROXY_UART_TX    GPIO_NUM_43
#define PROXY_UART_RX    GPIO_NUM_44
#define PROXY_UART_BAUD  921600
#define PROXY_UART_BUF   (1024)

#define FRAME_SYNC       0xAA
#define DIR_FROM_SWITCH   'S'  /* 0x53: Switch → PC */
#define DIR_FROM_PC       'P'  /* 0x50: PC → Switch */

#define MAX_REPORT_SIZE  64

/* Nintendo Switch Pro Controller identifiers */
#define SWITCH_PRO_VID   0x057E
#define SWITCH_PRO_PID   0x2009

/* Report IDs */
#define REPORT_ID_INPUT         0x30
#define REPORT_ID_SUBCMD_REPLY  0x21
#define REPORT_ID_USB_REPLY     0x81
#define REPORT_ID_SUBCMD        0x01
#define REPORT_ID_RUMBLE        0x10
#define REPORT_ID_USB_CMD       0x80

/* HID Report Descriptor — identical to the real Pro Controller */
static const uint8_t s_hid_report_descriptor[] = {
    0x05, 0x01, 0x15, 0x00, 0x09, 0x04, 0xA1, 0x01,
    0x85, 0x30, 0x05, 0x01, 0x05, 0x09, 0x19, 0x01,
    0x29, 0x0A, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
    0x95, 0x0A, 0x55, 0x00, 0x65, 0x00, 0x81, 0x02,
    0x05, 0x09, 0x19, 0x0B, 0x29, 0x0E, 0x15, 0x00,
    0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81, 0x02,
    0x75, 0x01, 0x95, 0x02, 0x81, 0x03,
    0x0B, 0x01, 0x00, 0x01, 0x00, 0xA1, 0x00,
    0x0B, 0x30, 0x00, 0x01, 0x00,
    0x0B, 0x31, 0x00, 0x01, 0x00,
    0x0B, 0x32, 0x00, 0x01, 0x00,
    0x0B, 0x35, 0x00, 0x01, 0x00,
    0x15, 0x00, 0x27, 0xFF, 0xFF, 0x00, 0x00,
    0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0xC0,
    0x0B, 0x39, 0x00, 0x01, 0x00, 0x15, 0x00, 0x25, 0x07,
    0x35, 0x00, 0x46, 0x3B, 0x01, 0x65, 0x14, 0x75, 0x04,
    0x95, 0x01, 0x81, 0x02,
    0x05, 0x09, 0x19, 0x0F, 0x29, 0x12, 0x15, 0x00,
    0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81, 0x02,
    0x75, 0x08, 0x95, 0x34, 0x81, 0x03,
    0x06, 0x00, 0xFF, 0x85, 0x21, 0x09, 0x01,
    0x75, 0x08, 0x95, 0x3F, 0x81, 0x03,
    0x85, 0x81, 0x09, 0x02, 0x75, 0x08, 0x95, 0x3F, 0x81, 0x03,
    0x85, 0x01, 0x09, 0x03, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
    0x85, 0x10, 0x09, 0x04, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
    0x85, 0x80, 0x09, 0x05, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
    0x85, 0x82, 0x09, 0x06, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
    0xC0,
};

/* USB Configuration Descriptor */
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

static const uint8_t s_config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE,
                             sizeof(s_hid_report_descriptor),
                             0x02, 0x81, 64, 8),
};

/* --- UART Framing --- */

static uint8_t calc_checksum(const uint8_t *data, size_t len)
{
    uint8_t xor = 0;
    for (size_t i = 0; i < len; i++) {
        xor ^= data[i];
    }
    return xor;
}

/**
 * @brief Send a framed packet to the PC over UART.
 *
 * Frame: [0xAA] [len] [direction] [report_data...] [checksum]
 */
static void uart_send_frame(uint8_t direction, const uint8_t *report, size_t report_len)
{
    uint8_t frame[3 + MAX_REPORT_SIZE + 1]; /* sync + len + payload + checksum */
    size_t payload_len = 1 + report_len;     /* direction + report */

    if (payload_len > MAX_REPORT_SIZE + 1) {
        return;
    }

    frame[0] = FRAME_SYNC;
    frame[1] = (uint8_t)payload_len;
    frame[2] = direction;
    memcpy(&frame[3], report, report_len);

    /* Checksum over payload (direction + report) */
    frame[2 + payload_len] = calc_checksum(&frame[2], payload_len);

    uart_write_bytes(PROXY_UART_NUM, frame, 2 + payload_len + 1);
}

/**
 * @brief Read a framed packet from the PC over UART.
 *
 * Returns the number of report bytes (excluding direction byte),
 * or 0 if no valid frame available.
 */
static size_t uart_read_frame(uint8_t *direction, uint8_t *report, size_t max_len)
{
    uint8_t byte;
    int ret;

    /* Look for sync byte */
    ret = uart_read_bytes(PROXY_UART_NUM, &byte, 1, 0);
    if (ret != 1 || byte != FRAME_SYNC) {
        return 0;
    }

    /* Read length */
    uint8_t len;
    ret = uart_read_bytes(PROXY_UART_NUM, &len, 1, pdMS_TO_TICKS(10));
    if (ret != 1 || len < 2 || len > MAX_REPORT_SIZE + 1) {
        return 0;
    }

    /* Read payload + checksum */
    uint8_t buf[MAX_REPORT_SIZE + 2]; /* payload + checksum */
    ret = uart_read_bytes(PROXY_UART_NUM, buf, len + 1, pdMS_TO_TICKS(20));
    if (ret != (int)(len + 1)) {
        return 0;
    }

    /* Verify checksum */
    uint8_t expected_checksum = calc_checksum(buf, len);
    if (buf[len] != expected_checksum) {
        ESP_LOGW(TAG, "Checksum mismatch: got 0x%02X, expected 0x%02X", buf[len],
                 expected_checksum);
        return 0;
    }

    *direction = buf[0];
    size_t report_len = len - 1;
    if (report_len > max_len) {
        report_len = max_len;
    }
    memcpy(report, &buf[1], report_len);
    return report_len;
}

/* --- TinyUSB Callbacks --- */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_type;
    ESP_LOGI(TAG, "GET_REPORT id=0x%02X", report_id);
    memset(buffer, 0, reqlen);
    return reqlen;
}

/**
 * @brief TinyUSB SET_REPORT callback — forwards Switch output to PC via UART.
 */
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_type;

    /* Normalize: ensure report_id is the first byte of full_pkt */
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

    /* Forward to PC over UART */
    uart_send_frame(DIR_FROM_SWITCH, full_pkt, total_len);
}

/* --- USB Init --- */

static esp_err_t usb_init(void)
{
    ESP_LOGI(TAG, "Initializing USB HID (Pro Controller proxy)");

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
        "\x09\x04",
        "Nintendo Co., Ltd.",
        "Pro Controller",
        "000000000001",
    };

    const tinyusb_config_t tusb_cfg = {
        .task =
            {
                .size = 4096,
                .priority = 5,
                .xCoreID = 0,
            },
        .descriptor =
            {
                .device = &device_desc,
                .full_speed_config = s_config_descriptor,
                .string = string_desc,
                .string_count = 4,
            },
    };

    return tinyusb_driver_install(&tusb_cfg);
}

/* --- UART Init --- */

static esp_err_t uart_proxy_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = PROXY_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(PROXY_UART_NUM, PROXY_UART_BUF * 2,
                                         PROXY_UART_BUF, 0, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = uart_param_config(PROXY_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }

    return uart_set_pin(PROXY_UART_NUM, PROXY_UART_TX, PROXY_UART_RX,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/* --- Main Bridge Task --- */

/**
 * @brief Bridge task: reads UART frames from PC and sends as USB HID reports.
 */
static void bridge_task(void *arg)
{
    (void)arg;
    uint8_t direction;
    uint8_t report[MAX_REPORT_SIZE];

    ESP_LOGI(TAG, "Bridge task started — forwarding PC→Switch");

    while (1) {
        size_t len = uart_read_frame(&direction, report, sizeof(report));
        if (len > 0 && direction == DIR_FROM_PC) {
            /* report[0] = report_id, report[1..] = report data */
            if (len > 1 && tud_mounted() && tud_hid_ready()) {
                uint8_t report_id = report[0];
                tud_hid_report(report_id, &report[1], len - 1);
            }
        }

        /* Small yield — don't spin at 100% CPU */
        vTaskDelay(1);
    }
}

/* --- Entry Point --- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Switch USB Proxy ===");
    ESP_LOGI(TAG, "UART: %d baud on GPIO%d(TX) / GPIO%d(RX)",
             PROXY_UART_BAUD, PROXY_UART_TX, PROXY_UART_RX);

    /* Initialize UART first (for status messages to PC) */
    esp_err_t ret = uart_proxy_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "UART initialized");

    /* Send a boot message to the PC */
    const char *boot_msg = "\r\n[PROXY] Switch USB Proxy ready\r\n";
    uart_write_bytes(PROXY_UART_NUM, boot_msg, strlen(boot_msg));

    /* Initialize USB HID */
    ret = usb_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB init failed: %s", esp_err_to_name(ret));
        const char *err_msg = "[PROXY] USB init failed\r\n";
        uart_write_bytes(PROXY_UART_NUM, err_msg, strlen(err_msg));
        return;
    }
    ESP_LOGI(TAG, "USB HID initialized — waiting for Switch connection");

    /* Start the PC→Switch bridge task */
    xTaskCreatePinnedToCore(bridge_task, "bridge", 4096, NULL, 4, NULL, 0);

    /* Main loop: just monitor USB mount status and notify PC */
    bool was_mounted = false;
    while (1) {
        bool mounted = tud_mounted();
        if (mounted && !was_mounted) {
            ESP_LOGI(TAG, "USB mounted (Switch connected)");
            const char *msg = "[PROXY] USB mounted\r\n";
            uart_write_bytes(PROXY_UART_NUM, msg, strlen(msg));
        } else if (!mounted && was_mounted) {
            ESP_LOGI(TAG, "USB unmounted (Switch disconnected)");
            const char *msg = "[PROXY] USB unmounted\r\n";
            uart_write_bytes(PROXY_UART_NUM, msg, strlen(msg));
        }
        was_mounted = mounted;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
