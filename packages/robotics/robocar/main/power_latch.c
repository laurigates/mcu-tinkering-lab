/**
 * @file power_latch.c
 * @brief HOLD output of the carrier's soft latch (see power_latch.h).
 */

#include "power_latch.h"

#include <stdatomic.h>

#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "pin_config.h"

static const char *TAG = "power_latch";

static atomic_bool s_early;

/** Level first, then direction, so the pad goes straight to HIGH without a
 *  low pulse. GPIO44 is UART0 RX by default; gpio_config() hands it to the
 *  GPIO matrix. No logging: this also runs before app_main. */
static esp_err_t drive_hold(int level)
{
    gpio_set_level(POWER_HOLD_PIN, level);
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << POWER_HOLD_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io);
}

/** Runs from the startup code's static constructors, before app_main and the
 *  scheduler. Priority 101 is the first one open to applications. */
__attribute__((constructor(101))) static void power_latch_early(void)
{
    if (!FEATURE_POWER_LATCH_ENABLED) {
        return;
    }
    if (drive_hold(1) == ESP_OK) {
        atomic_store(&s_early, true);
    }
}

esp_err_t power_latch_init(void)
{
    if (!FEATURE_POWER_LATCH_ENABLED) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* No logging: app_main calls this ahead of event_report_boot_capture(), and
     * the first log line overwrites the previous boot's breadcrumbs (ADR-033).
     * app_main logs the outcome from power_latch_asserted_early() later. */
    return drive_hold(1);
}

void power_latch_release(void)
{
    if (!FEATURE_POWER_LATCH_ENABLED) {
        return;
    }
    (void)drive_hold(0);
    ESP_LOGW(TAG, "HOLD (GPIO%d) released", (int)POWER_HOLD_PIN);
}

bool power_latch_asserted_early(void)
{
    return atomic_load(&s_early);
}
