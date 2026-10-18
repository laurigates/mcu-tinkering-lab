/**
 * @file log_mirror.c
 * @brief UART0 mirror of the application log (issue #798).
 *
 * See log_mirror.h for what is and is not mirrored. The line rendering is
 * pure C in log_mirror_core.c, host-tested in test/test_log_mirror_core.c;
 * this file is only the ESP-IDF plumbing around it.
 *
 * Why a queue and a drain task rather than uart_write_bytes() in the hook:
 * the hook runs on whichever task logged — the 30 Hz executor included — and
 * at 115200 baud a burst of lines outruns the wire. uart_write_bytes() blocks
 * once the driver's TX ring is full, and the driver offers no reliable way to
 * ask beforehand: uart_get_tx_buffer_free_size() reports the ring size minus
 * the item the ISR is sending now, not minus what is queued behind it
 * (esp_driver_uart/src/uart.c, ESP-IDF v5.4). A NOSPLIT ring buffer sent with
 * a zero timeout either takes the whole line or refuses it, from any number of
 * tasks, so the hook never waits; the drain task is the only caller that ever
 * blocks on the UART.
 *
 * Nothing in this file may call ESP_LOG once the hook is installed except
 * log_mirror_init() itself, after it has finished: a log line from inside the
 * hook would re-enter it.
 */

#include "log_mirror.h"

#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>

#include "config.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "log_mirror_core.h"
#include "pin_config.h"

#if LOG_MIRROR_UART0_ENABLED

#define LOG_MIRROR_UART UART_NUM_0

/* The driver refuses an RX ring no larger than the 128-byte hardware FIFO, so
 * one must be given although RX is never read (and its interrupt is turned
 * off below). TX ring 0: the drain task is the only writer and may block. */
#define LOG_MIRROR_UART_RX_BYTES 256

static const char *TAG = "log_mirror";

static RingbufHandle_t s_queue;
static vprintf_like_t s_prev_vprintf;
static atomic_bool s_ready;
static atomic_uint_fast32_t s_dropped;

static void mirror_enqueue(const char *fmt, va_list args)
{
    char line[LOG_MIRROR_LINE_MAX];
    const size_t len = log_mirror_format(line, sizeof(line), fmt, args);
    if (len == 0) {
        return;
    }
    if (xRingbufferSend(s_queue, line, len, 0) != pdTRUE) {
        atomic_fetch_add(&s_dropped, 1);
    }
}

static int mirror_vprintf(const char *fmt, va_list args)
{
    /* Before init finishes there is no queue; from an ISR the hook must not
     * format into a stack buffer or touch the ring. ESP_LOG is not ISR-safe
     * anyway (ESP_DRAM_LOG is, and does not come through here). */
    if (atomic_load(&s_ready) && !xPortInIsrContext()) {
        va_list copy;
        va_copy(copy, args);
        mirror_enqueue(fmt, copy);
        va_end(copy);
    }
    /* esp_log_set_vprintf() installs this hook before it returns the previous
     * one, so a task can log in between with s_prev_vprintf still unset. The
     * previous handler is the default, vprintf, in that window. */
    const vprintf_like_t prev = s_prev_vprintf;
    return (prev != NULL ? prev : vprintf)(fmt, args);
}

static void drain_task(void *arg)
{
    (void)arg;
    for (;;) {
        size_t len = 0;
        void *item = xRingbufferReceive(s_queue, &len, portMAX_DELAY);
        if (item == NULL) {
            continue;
        }
        (void)uart_write_bytes(LOG_MIRROR_UART, item, len);
        vRingbufferReturnItem(s_queue, item);

        /* Report drops on the wire that lost them, between whole lines.
         * snprintf, not ESP_LOG: a log line from here would be queued behind
         * the very backlog it is reporting. */
        const uint_fast32_t dropped = atomic_exchange(&s_dropped, 0);
        if (dropped > 0) {
            char note[64];
            const int n = snprintf(note, sizeof(note), "[log_mirror: %u line(s) dropped]\r\n",
                                   (unsigned)dropped);
            if (n > 0) {
                (void)uart_write_bytes(LOG_MIRROR_UART, note,
                                       (size_t)n < sizeof(note) ? (size_t)n : sizeof(note) - 1);
            }
        }
    }
}

static esp_err_t uart_setup(void)
{
    const uart_config_t cfg = {
        .baud_rate = LOG_MIRROR_UART0_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(LOG_MIRROR_UART, LOG_MIRROR_UART_RX_BYTES, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_param_config(LOG_MIRROR_UART, &cfg);
    if (err == ESP_OK) {
        /* TX only. RX stays where it is (UART_PIN_NO_CHANGE): D7 is not wired
         * on the carrier, and nothing here claims GPIO44. */
        err = uart_set_pin(LOG_MIRROR_UART, UART0_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        /* Nothing reads UART0, and a floating RX pad would otherwise keep the
         * driver's ISR busy filling a buffer nobody drains. */
        err = uart_disable_rx_intr(LOG_MIRROR_UART);
    }
    if (err != ESP_OK) {
        (void)uart_driver_delete(LOG_MIRROR_UART);
    }
    return err;
}

esp_err_t log_mirror_init(void)
{
    if (atomic_load(&s_ready)) {
        return ESP_OK;
    }

    s_queue = xRingbufferCreateWithCaps(LOG_MIRROR_QUEUE_BYTES, RINGBUF_TYPE_NOSPLIT,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = uart_setup();
    if (err != ESP_OK) {
        vRingbufferDeleteWithCaps(s_queue);
        s_queue = NULL;
        return err;
    }

    const BaseType_t ok =
        xTaskCreatePinnedToCore(drain_task, "log_mirror", LOG_MIRROR_TASK_STACK_SIZE, NULL,
                                LOG_MIRROR_TASK_PRIORITY, NULL, LOG_MIRROR_TASK_CORE);
    if (ok != pdPASS) {
        (void)uart_driver_delete(LOG_MIRROR_UART);
        vRingbufferDeleteWithCaps(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* The queue and the drain task exist before the hook can see them. */
    atomic_store(&s_ready, true);
    s_prev_vprintf = esp_log_set_vprintf(mirror_vprintf);
    if (s_prev_vprintf == NULL) {
        s_prev_vprintf = vprintf;
    }

    ESP_LOGI(TAG, "mirroring the log to UART0 TX (GPIO%d, %d 8N1); console stays on USB",
             (int)UART0_TX_PIN, LOG_MIRROR_UART0_BAUD);
    return ESP_OK;
}

#else  // !LOG_MIRROR_UART0_ENABLED

esp_err_t log_mirror_init(void)
{
    return ESP_OK;
}

#endif  // LOG_MIRROR_UART0_ENABLED
