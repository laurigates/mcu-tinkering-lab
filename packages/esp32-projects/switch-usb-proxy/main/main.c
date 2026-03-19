/**
 * @file main.c
 * @brief Switch USB Proxy — on-device Pro Controller emulator with UART logging.
 *
 * Handles the full Nintendo Switch Pro Controller USB protocol on-device
 * (no UART round-trip latency). UART is used only for debug logging.
 *
 * Architecture: FreeRTOS queue + single-writer to HID IN endpoint.
 *   - TinyUSB callback (priority 5) receives commands, enqueues replies
 *     via xQueueSend (atomic struct copy — no torn data)
 *   - input_report_task (priority 4) dequeues replies and sends ALL
 *     reports: queued replies first, then 0x30 keepalive reports
 *   - s_timer_counter is owned exclusively by input_report_task
 *     (single-writer — no synchronization needed)
 *   - Never call tud_hid_report() from the callback — the TinyUSB task
 *     can preempt the report task mid-send, causing concurrent calls
 *     and corrupted state (was the root cause of Switch error 2162-0002)
 *
 * Hardware:
 *   - USB-C port → connects to Switch (direct or via dock)
 *   - UART1 on GPIO43 (TX) / GPIO44 (RX) → debug logging to PC
 */

#include <stdio.h>
#include <string.h>

#include "class/hid/hid_device.h"
#include "device/usbd.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tinyusb.h"

static const char *TAG __attribute__((unused)) = "switch_emu";

/* --- Configuration --- */

#define PROXY_UART_NUM UART_NUM_1
#define PROXY_UART_TX GPIO_NUM_43
#define PROXY_UART_RX GPIO_NUM_44
#define PROXY_UART_BAUD 921600
#define PROXY_UART_BUF (1024)

/* Debug log over UART (ESP_LOG goes to USB-Serial-JTAG which is unavailable) */
static void uart_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void uart_log(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf) - 2, fmt, args);
    va_end(args);
    if (len > 0) {
        buf[len] = '\n';
        uart_write_bytes(PROXY_UART_NUM, buf, len + 1);
    }
}

/* Nintendo Switch Pro Controller identifiers */
#define SWITCH_PRO_VID 0x057E
#define SWITCH_PRO_PID 0x2009

/* Report IDs: Controller → Switch (input) */
#define REPORT_ID_INPUT 0x30
#define REPORT_ID_SUBCMD_REPLY 0x21
#define REPORT_ID_USB_REPLY 0x81

/* Report IDs: Switch → Controller (output) */
#define REPORT_ID_SUBCMD 0x01
#define REPORT_ID_RUMBLE 0x10
#define REPORT_ID_USB_CMD 0x80

/* USB handshake sub-commands */
#define USB_CMD_STATUS 0x01
#define USB_CMD_HANDSHAKE 0x02
#define USB_CMD_HIGH_SPEED 0x03
#define USB_CMD_FORCE_USB 0x04
#define USB_CMD_DISABLE_USB 0x05

/* Fake MAC address */
static const uint8_t s_fake_mac[6] = {0x00, 0x00, 0x5E, 0x00, 0x53, 0x01};

/* Protocol state */
static volatile bool s_handshake_complete = false;
static volatile bool s_setup_complete = false;

/* Reply queue: callback enqueues, input_report_task dequeues.
 * FreeRTOS queue performs atomic struct copy — eliminates the torn-data
 * race that occurred with the old volatile flag approach. */
typedef struct {
    uint8_t report_id;
    uint8_t len;
    uint8_t data[63];
} reply_msg_t;

static QueueHandle_t s_reply_queue = NULL;

/* Timer counter — owned exclusively by input_report_task (single writer). */
static uint8_t s_timer_counter = 0;

/* --- SPI Flash Emulation --- */

typedef struct {
    uint32_t addr;
    const uint8_t *data;
    uint8_t len;
} spi_entry_t;

static const uint8_t spi_device_type[] = {0x03};
static const uint8_t spi_imu_cal[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Accel origin */
    0x00, 0x40, 0x00, 0x40, 0x00, 0x40, /* Accel sensitivity */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Gyro origin */
    0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34, /* Gyro sensitivity */
};
static const uint8_t spi_lstick_cal[] = {0x00, 0x07, 0x70, 0x00, 0x08, 0x80, 0x00, 0x07, 0x70};
static const uint8_t spi_rstick_cal[] = {0x00, 0x08, 0x80, 0x00, 0x07, 0x70, 0x00, 0x07, 0x70};
static const uint8_t spi_colors[] = {0x32, 0x32, 0x32, 0xFF, 0xFF, 0xFF};
static const uint8_t spi_grip_l[] = {0x32, 0x32, 0x32};
static const uint8_t spi_grip_r[] = {0x32, 0x32, 0x32};
static const uint8_t spi_serial[16] = {[0 ... 15] = 0xFF};
static const uint8_t spi_user_none[] = {0xFF};
static const uint8_t spi_shipment[] = {0x00};

static const spi_entry_t s_spi_flash[] = {
    {0x6012, spi_device_type, sizeof(spi_device_type)},
    {0x6020, spi_imu_cal, sizeof(spi_imu_cal)},
    {0x603D, spi_lstick_cal, sizeof(spi_lstick_cal)},
    {0x6046, spi_rstick_cal, sizeof(spi_rstick_cal)},
    {0x6050, spi_colors, sizeof(spi_colors)},
    {0x6056, spi_grip_l, sizeof(spi_grip_l)},
    {0x6059, spi_grip_r, sizeof(spi_grip_r)},
    {0x6000, spi_serial, sizeof(spi_serial)},
    {0x8010, spi_user_none, 1},
    {0x801B, spi_user_none, 1},
    {0x8026, spi_user_none, 1},
    {0x5000, spi_shipment, sizeof(spi_shipment)},
};
#define SPI_FLASH_COUNT (sizeof(s_spi_flash) / sizeof(s_spi_flash[0]))

static void spi_read(uint32_t addr, uint8_t read_len, uint8_t *out)
{
    memset(out, 0, read_len);

    for (size_t i = 0; i < SPI_FLASH_COUNT; i++) {
        const spi_entry_t *e = &s_spi_flash[i];
        if (addr >= e->addr && addr < e->addr + e->len) {
            uint32_t offset = addr - e->addr;
            uint8_t avail = e->len - offset;
            uint8_t copy = (read_len < avail) ? read_len : avail;
            memcpy(out, e->data + offset, copy);
            return;
        }
    }
}

/* --- HID Report Descriptor — identical to the real Pro Controller --- */

static const uint8_t s_hid_report_descriptor[] = {
    0x05, 0x01, 0x15, 0x00, 0x09, 0x04, 0xA1, 0x01, 0x85, 0x30, 0x05, 0x01, 0x05, 0x09, 0x19,
    0x01, 0x29, 0x0A, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0A, 0x55, 0x00, 0x65, 0x00,
    0x81, 0x02, 0x05, 0x09, 0x19, 0x0B, 0x29, 0x0E, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95,
    0x04, 0x81, 0x02, 0x75, 0x01, 0x95, 0x02, 0x81, 0x03, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xA1,
    0x00, 0x0B, 0x30, 0x00, 0x01, 0x00, 0x0B, 0x31, 0x00, 0x01, 0x00, 0x0B, 0x32, 0x00, 0x01,
    0x00, 0x0B, 0x35, 0x00, 0x01, 0x00, 0x15, 0x00, 0x27, 0xFF, 0xFF, 0x00, 0x00, 0x75, 0x10,
    0x95, 0x04, 0x81, 0x02, 0xC0, 0x0B, 0x39, 0x00, 0x01, 0x00, 0x15, 0x00, 0x25, 0x07, 0x35,
    0x00, 0x46, 0x3B, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x02, 0x05, 0x09, 0x19,
    0x0F, 0x29, 0x12, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81, 0x02, 0x75, 0x08,
    0x95, 0x34, 0x81, 0x03, 0x06, 0x00, 0xFF, 0x85, 0x21, 0x09, 0x01, 0x75, 0x08, 0x95, 0x3F,
    0x81, 0x03, 0x85, 0x81, 0x09, 0x02, 0x75, 0x08, 0x95, 0x3F, 0x81, 0x03, 0x85, 0x01, 0x09,
    0x03, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83, 0x85, 0x10, 0x09, 0x04, 0x75, 0x08, 0x95, 0x3F,
    0x91, 0x83, 0x85, 0x80, 0x09, 0x05, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83, 0x85, 0x82, 0x09,
    0x06, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83, 0xC0,
};

/* USB Configuration Descriptor */
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

static const uint8_t s_config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),
    TUD_HID_INOUT_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(s_hid_report_descriptor), 0x02,
                             0x81, 64, 8),
};

/* --- Neutral stick data (center position 0x800) --- */

static void fill_neutral_sticks(uint8_t *buf, size_t offset)
{
    /* 12-bit packed: X=0x800, Y=0x800 for both sticks */
    buf[offset + 0] = 0x00;
    buf[offset + 1] = 0x08;
    buf[offset + 2] = 0x80;
    buf[offset + 3] = 0x00;
    buf[offset + 4] = 0x08;
    buf[offset + 5] = 0x80;
}

/* --- Reply Queue --- */

/**
 * @brief Enqueue a reply for the input_report_task to send.
 *
 * Uses xQueueSend which performs an atomic struct copy into the queue,
 * eliminating the torn-data race that occurred with volatile flags.
 * Called from TinyUSB callback context (priority 5).
 */
static void queue_reply(uint8_t report_id, const uint8_t *data, uint8_t len)
{
    reply_msg_t msg = {0};
    msg.report_id = report_id;
    msg.len = len;
    memcpy(msg.data, data, len);

    if (xQueueSend(s_reply_queue, &msg, 0) != pdTRUE) {
        uart_log("WARN: reply queue full — dropping 0x%02X", report_id);
    }
}

/* --- Protocol Handlers --- */

static void handle_usb_cmd(const uint8_t *data, uint16_t len)
{
    if (len < 2) {
        return;
    }

    uint8_t subcmd = data[1];
    uint8_t response[63] = {0};

    switch (subcmd) {
        case USB_CMD_STATUS:
            uart_log("USB CMD: Status request");
            response[0] = 0x01;
            response[1] = 0x00;
            response[2] = 0x03; /* Pro Controller type */
            memcpy(&response[3], s_fake_mac, 6);
            queue_reply(REPORT_ID_USB_REPLY, response, sizeof(response));
            break;

        case USB_CMD_HANDSHAKE:
            uart_log("USB CMD: Handshake");
            response[0] = 0x02;
            queue_reply(REPORT_ID_USB_REPLY, response, sizeof(response));
            break;

        case USB_CMD_HIGH_SPEED:
            uart_log("USB CMD: High speed");
            response[0] = 0x03;
            queue_reply(REPORT_ID_USB_REPLY, response, sizeof(response));
            break;

        case USB_CMD_FORCE_USB:
            uart_log("USB CMD: Force USB — handshake complete!");
            s_handshake_complete = true;
            break;

        case USB_CMD_DISABLE_USB:
            uart_log("USB CMD: Disable USB timeout (no reply)");
            break;

        default:
            uart_log("USB CMD: Unknown 0x%02X", subcmd);
            break;
    }
}

static void handle_subcommand(const uint8_t *data, uint16_t len)
{
    if (len < 11) {
        return;
    }

    uint8_t subcmd_id = data[10];
    uart_log("Subcmd: 0x%02X", subcmd_id);

    /* Build 0x21 reply: 63 bytes after report ID.
     * Timer byte (reply[0]) is set to 0 here — input_report_task stamps
     * the actual s_timer_counter value right before sending. */
    uint8_t reply[63] = {0};
    reply[0] = 0; /* placeholder — stamped by report task */
    reply[1] = 0x8E; /* USB, Pro Controller */
    fill_neutral_sticks(reply, 5);
    reply[12] = 0x80; /* ACK */
    reply[13] = subcmd_id;

    switch (subcmd_id) {
        case 0x02: /* Device info */
            reply[14] = 0x04; /* FW major */
            reply[15] = 0x33; /* FW minor */
            reply[16] = 0x03; /* Pro Controller */
            reply[17] = 0x02;
            memcpy(&reply[18], s_fake_mac, 6);
            reply[24] = 0x01;
            reply[25] = 0x02; /* SPI colors */
            break;

        case 0x03: /* Set input report mode */
            break;

        case 0x04: /* Trigger buttons elapsed time */
            break;

        case 0x08: /* Set shipment low power state */
            break;

        case 0x10: { /* SPI flash read */
            reply[12] = 0x90; /* SPI read uses 0x90 ACK */
            if (len >= 16) {
                uint32_t addr = data[11] | (data[12] << 8) | (data[13] << 16) | (data[14] << 24);
                uint8_t read_len = data[15];
                if (read_len > 29) {
                    read_len = 29;
                }
                reply[14] = data[11]; /* addr echo LE */
                reply[15] = data[12];
                reply[16] = data[13];
                reply[17] = data[14];
                reply[18] = read_len;
                spi_read(addr, read_len, &reply[19]);
                uart_log("  SPI 0x%04lX [%dB]", (unsigned long)addr, read_len);
            }
            break;
        }

        case 0x30: /* Player lights */
            uart_log("Player lights: 0x%02X", data[11]);
            if (!s_setup_complete) {
                s_setup_complete = true;
                uart_log("Setup complete — controller is live");
            }
            break;

        case 0x38: /* HOME light */
            break;

        case 0x40: /* Enable IMU */
            uart_log("IMU enable: %d", data[11]);
            break;

        case 0x41: /* IMU sensitivity */
            break;

        case 0x48: /* Enable vibration */
            uart_log("Vibration enable: %d", data[11]);
            break;

        default:
            uart_log("Unknown subcmd 0x%02X — ACK", subcmd_id);
            break;
    }

    queue_reply(REPORT_ID_SUBCMD_REPLY, reply, sizeof(reply));
}

/* --- TinyUSB Callbacks --- */

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_type;
    uart_log("GET_REPORT id=0x%02X", report_id);
    memset(buffer, 0, reqlen);
    return reqlen;
}

/**
 * @brief TinyUSB SET_REPORT callback — handles all Switch commands on-device.
 *
 * Enqueues replies via xQueueSend for the input_report_task to send.
 * Never sends directly to avoid concurrent tud_hid_report() calls
 * (this callback runs at higher priority than the report task).
 */
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_type;

    uint8_t full_pkt[64];
    uint16_t total_len;

    if (report_id == 0 && bufsize > 0) {
        report_id = buffer[0];
        total_len = (bufsize < 64) ? bufsize : 64;
        memcpy(full_pkt, buffer, total_len);
    } else {
        full_pkt[0] = report_id;
        uint16_t copy_len = (bufsize < 63) ? bufsize : 63;
        memcpy(&full_pkt[1], buffer, copy_len);
        total_len = copy_len + 1;
    }

    uart_log("CB: id=0x%02X len=%d", report_id, (int)total_len);

    if (report_id == REPORT_ID_USB_CMD) {
        handle_usb_cmd(full_pkt, total_len);
    } else if (report_id == REPORT_ID_SUBCMD || report_id == REPORT_ID_RUMBLE) {
        handle_subcommand(full_pkt, total_len);
    }
}

/* --- USB Init --- */

static esp_err_t usb_init(void)
{
    uart_log("Initializing USB HID (Pro Controller)");

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

/* --- UART Init (debug logging only) --- */

static esp_err_t uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = PROXY_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret =
        uart_driver_install(PROXY_UART_NUM, PROXY_UART_BUF * 2, PROXY_UART_BUF, 0, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = uart_param_config(PROXY_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }
    return uart_set_pin(PROXY_UART_NUM, PROXY_UART_TX, PROXY_UART_RX, UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
}

/* --- Input Report Task --- */

/**
 * @brief Single writer to the HID IN endpoint.
 *
 * Drains the reply queue (0x81/0x21) with priority, retries on endpoint
 * busy, then sends 0x30 keepalive reports after handshake. This is the
 * ONLY task that calls tud_hid_report(), preventing concurrent access
 * from the TinyUSB callback task.
 *
 * s_timer_counter is incremented here immediately before each send —
 * single-writer ownership eliminates the need for synchronization.
 */
static void input_report_task(void *arg)
{
    (void)arg;

    uart_log("Input report task started");

    bool lr_sent = false;
    uint32_t lr_press_tick = 0;

    /* Local retry buffer — holds a reply that failed to send (endpoint busy) */
    reply_msg_t pending_reply;
    bool have_pending_reply = false;

    while (1) {
        /* Priority 1: retry a previously failed reply send */
        if (have_pending_reply && tud_hid_ready()) {
            /* Only stamp timer for 0x21 reports — 0x81 replies use data[0]
             * as the sub-command echo byte and must not be overwritten. */
            if (pending_reply.report_id == REPORT_ID_SUBCMD_REPLY) {
                pending_reply.data[0] = s_timer_counter++;
            }
            bool ok = tud_hid_report(pending_reply.report_id, pending_reply.data,
                                     pending_reply.len);
            if (ok) {
                uart_log("SENT 0x%02X ok (retry)", pending_reply.report_id);
                have_pending_reply = false;
            } else {
                uart_log("BUSY 0x%02X — will retry", pending_reply.report_id);
                vTaskDelay(1);
                continue;
            }
            vTaskDelay(1);
            continue;
        }

        /* Priority 2: drain the reply queue */
        if (!have_pending_reply && xQueueReceive(s_reply_queue, &pending_reply, 0) == pdTRUE) {
            if (!tud_hid_ready()) {
                have_pending_reply = true;
                vTaskDelay(1);
                continue;
            }
            if (pending_reply.report_id == REPORT_ID_SUBCMD_REPLY) {
                pending_reply.data[0] = s_timer_counter++;
            }
            bool ok = tud_hid_report(pending_reply.report_id, pending_reply.data,
                                     pending_reply.len);
            if (ok) {
                uart_log("SENT 0x%02X ok", pending_reply.report_id);
            } else {
                uart_log("BUSY 0x%02X — will retry", pending_reply.report_id);
                have_pending_reply = true;
            }
            vTaskDelay(1);
            continue;
        }

        /* Priority 3: wait for handshake before sending 0x30 reports.
         * The Switch needs continuous 0x30 keepalive reports to proceed
         * with setup (SPI reads, IMU, player lights). The reply queue
         * priority above ensures 0x21 replies are sent first. */
        if (!tud_mounted() || !s_handshake_complete) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (!tud_hid_ready()) {
            vTaskDelay(1);
            continue;
        }

        /* Priority 4: send 0x30 input report (only when queue empty and no pending retry) */
        uint8_t report[63] = {0};
        report[0] = s_timer_counter++;
        report[1] = 0x8E; /* USB, Pro Controller */

        /* Press L+R to register the controller (only after setup) */
        if (s_setup_complete && !lr_sent) {
            lr_press_tick = xTaskGetTickCount();
            lr_sent = true;
            uart_log("Pressing L+R to register controller");
        }
        if (lr_sent && (xTaskGetTickCount() - lr_press_tick) < pdMS_TO_TICKS(200)) {
            report[2] = 0x40; /* R button (buttons_right byte) */
            report[4] = 0x40; /* L button (buttons_left byte) */
        }

        fill_neutral_sticks(report, 5);

        tud_hid_report(REPORT_ID_INPUT, report, sizeof(report));

        /* 8ms = 125 Hz */
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

/* --- Entry Point --- */

void app_main(void)
{
    /* Initialize UART first (needed for uart_log) */
    esp_err_t ret = uart_init();
    if (ret != ESP_OK) {
        return;
    }

    uart_log("=== Switch Pro Controller Emulator ===");
    const char *boot_msg = "\r\n[EMU] Switch Pro Controller Emulator ready\r\n";
    uart_write_bytes(PROXY_UART_NUM, boot_msg, strlen(boot_msg));

    /* Create reply queue before USB init — callbacks may fire immediately */
    s_reply_queue = xQueueCreate(2, sizeof(reply_msg_t));
    if (s_reply_queue == NULL) {
        uart_log("FATAL: failed to create reply queue");
        return;
    }

    /* Initialize USB HID */
    ret = usb_init();
    if (ret != ESP_OK) {
        uart_log("USB init failed: %s", esp_err_to_name(ret));
        return;
    }
    uart_log("USB HID initialized — waiting for Switch");

    /* Start input report task on same core as TinyUSB */
    xTaskCreatePinnedToCore(input_report_task, "input_rpt", 4096, NULL, 4, NULL, 0);

    /* Main loop: monitor USB mount status */
    bool was_mounted = false;
    while (1) {
        bool mounted = tud_mounted();
        if (mounted && !was_mounted) {
            uart_log("USB mounted (Switch connected)");
            const char *msg = "[EMU] USB mounted\r\n";
            uart_write_bytes(PROXY_UART_NUM, msg, strlen(msg));
        } else if (!mounted && was_mounted) {
            uart_log("USB unmounted (Switch disconnected)");
            const char *msg = "[EMU] USB unmounted\r\n";
            uart_write_bytes(PROXY_UART_NUM, msg, strlen(msg));
            /* Reset state for reconnection */
            s_handshake_complete = false;
            s_setup_complete = false;
            /* Flush stale replies from previous session */
            xQueueReset(s_reply_queue);
        }
        was_mounted = mounted;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
