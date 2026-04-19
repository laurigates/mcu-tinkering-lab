/**
 * @file power_mgr.c
 * @brief ESP-IDF orchestration for thinkpack-power.
 *
 * Periodically samples Vbat via ADC1 oneshot, runs the pure-logic
 * classifier, and logs state transitions. When thinkpack-mesh gains a
 * runtime beacon-interval setter this file will also poke that API; for
 * now we only log the recommended interval — see TODO below.
 *
 * Host builds compile to a shim that returns ESP_OK and does no work.
 */

#include "thinkpack_power.h"

#ifdef ESP_PLATFORM

#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

static const char *TAG = "tp_power";

typedef struct {
    bool initialised;
    power_config_t config;
    adc_oneshot_unit_handle_t adc;
    adc_cali_handle_t cali;
    adc_channel_t channel;
    esp_timer_handle_t tick_timer;
    uint16_t last_mv;
    thinkpack_power_battery_state_t last_state;
} power_mgr_ctx_t;

static power_mgr_ctx_t s_ctx;

static uint16_t read_vbat_mv(void)
{
    if (!s_ctx.initialised || !s_ctx.adc) {
        return 0;
    }

    int raw = 0;
    if (adc_oneshot_read(s_ctx.adc, s_ctx.channel, &raw) != ESP_OK) {
        return s_ctx.last_mv;
    }

    int mv_at_adc = 0;
    if (s_ctx.cali && adc_cali_raw_to_voltage(s_ctx.cali, raw, &mv_at_adc) != ESP_OK) {
        return s_ctx.last_mv;
    }

    uint16_t ratio_x10 = s_ctx.config.divider_ratio_x10 ? s_ctx.config.divider_ratio_x10 : 20u;
    uint32_t vbat_mv = ((uint32_t)mv_at_adc * ratio_x10) / 10u;
    if (vbat_mv > 0xFFFFu) {
        vbat_mv = 0xFFFFu;
    }
    return (uint16_t)vbat_mv;
}

static void periodic_tick(void *arg)
{
    (void)arg;
    uint16_t mv = read_vbat_mv();
    s_ctx.last_mv = mv;

    thinkpack_power_battery_state_t state = thinkpack_power_classify(mv);
    if (state != s_ctx.last_state) {
        uint16_t new_interval = thinkpack_power_adaptive_beacon_interval_ms(state);
        ESP_LOGI(TAG, "battery state %d -> %d (%u mV) — recommended beacon interval %u ms",
                 (int)s_ctx.last_state, (int)state, (unsigned)mv, (unsigned)new_interval);
        s_ctx.last_state = state;
        /* TODO(thinkpack-mesh): once thinkpack_mesh_set_beacon_interval_ms()
         * lands, invoke it here so the mesh actually reduces beacon rate when
         * the battery is low. For now the classification is logged only. */
    }
}

esp_err_t thinkpack_power_init(const power_config_t *config)
{
    if (!config || config->adc_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.initialised) {
        return ESP_OK;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.config = *config;
    if (s_ctx.config.tick_interval_ms == 0) {
        s_ctx.config.tick_interval_ms = 5000u;
    }
    if (s_ctx.config.divider_ratio_x10 == 0) {
        s_ctx.config.divider_ratio_x10 = 20u; /* 1:2 divider by default */
    }

    adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_1};
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_ctx.adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %d", err);
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    /* ADC1 channel from GPIO lookup is target-specific; for the ESP32-S3
     * SuperMini we default to the GPIO1/CH0 wiring documented in WIRING.md.
     * Caller supplies adc_gpio in power_config_t; we trust the pinout. */
    s_ctx.channel = (adc_channel_t)(config->adc_gpio - 1); /* GPIO1 -> CH0, GPIO2 -> CH1, ... */
    err = adc_oneshot_config_channel(s_ctx.adc, s_ctx.channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %d", err);
        return err;
    }

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = s_ctx.channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_ctx.cali) != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable — readings may be less accurate");
        s_ctx.cali = NULL;
    }

    esp_timer_create_args_t timer_args = {
        .callback = periodic_tick,
        .name = "tp_power_tick",
    };
    err = esp_timer_create(&timer_args, &s_ctx.tick_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %d", err);
        return err;
    }

    err = esp_timer_start_periodic(s_ctx.tick_timer,
                                   (uint64_t)s_ctx.config.tick_interval_ms * 1000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
        return err;
    }

    s_ctx.initialised = true;
    s_ctx.last_state = THINKPACK_POWER_OK;
    ESP_LOGI(TAG, "power monitor started — gpio=%d tick=%u ms", config->adc_gpio,
             (unsigned)s_ctx.config.tick_interval_ms);
    return ESP_OK;
}

uint16_t thinkpack_power_read_voltage_mv(void)
{
    return s_ctx.last_mv;
}

void thinkpack_power_deep_sleep(uint32_t wake_ms)
{
    ESP_LOGI(TAG, "entering deep sleep for %u ms", (unsigned)wake_ms);
    esp_sleep_enable_timer_wakeup((uint64_t)wake_ms * 1000ULL);
    esp_deep_sleep_start();
}

#else /* !ESP_PLATFORM */

/* Host build — ESP-IDF APIs unavailable. Provide harmless stubs so a
 * firmware main.c that calls thinkpack_power_init() still links on the
 * host if ever desired. */

esp_err_t thinkpack_power_init(const power_config_t *config)
{
    (void)config;
    return ESP_OK;
}

uint16_t thinkpack_power_read_voltage_mv(void)
{
    return 0;
}

void thinkpack_power_deep_sleep(uint32_t wake_ms)
{
    (void)wake_ms;
}

#endif /* ESP_PLATFORM */
