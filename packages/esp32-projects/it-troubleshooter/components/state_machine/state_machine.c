/**
 * @file state_machine.c
 * @brief NVS-persisted device state machine for IT Troubleshooter.
 */

#include "state_machine.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "state_machine";

#define NVS_NAMESPACE "it-ts"
#define NVS_KEY_PHASE "phase"
#define NVS_KEY_STEP  "step"

static sm_phase_t s_phase = SM_PHASE_USB_ONLY;
static uint8_t s_step = 0;

/** Write current s_phase and s_step to NVS. */
static esp_err_t persist(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u8(handle, NVS_KEY_PHASE, (uint8_t)s_phase);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_u8 phase failed: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_set_u8(handle, NVS_KEY_STEP, s_step);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_u8 step failed: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(ret));
    }

    nvs_close(handle);
    return ret;
}

esp_err_t sm_init(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        /* No state persisted yet — start at defaults */
        ESP_LOGI(TAG, "No persisted state found — starting fresh (phase=1, step=0)");
        s_phase = SM_PHASE_USB_ONLY;
        s_step = 0;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s — using defaults", esp_err_to_name(ret));
        return ret;
    }

    uint8_t phase_raw = (uint8_t)SM_PHASE_USB_ONLY;
    uint8_t step_raw = 0;

    /* Ignore read errors: fall back to default values */
    nvs_get_u8(handle, NVS_KEY_PHASE, &phase_raw);
    nvs_get_u8(handle, NVS_KEY_STEP, &step_raw);
    nvs_close(handle);

    /* Validate phase is in range; reset to default if corrupted */
    if (phase_raw < SM_PHASE_USB_ONLY || phase_raw > SM_PHASE_CMD_RUNNING) {
        ESP_LOGW(TAG, "Stored phase %u out of range — resetting", phase_raw);
        phase_raw = (uint8_t)SM_PHASE_USB_ONLY;
        step_raw = 0;
    }

    s_phase = (sm_phase_t)phase_raw;
    s_step = step_raw;

    ESP_LOGI(TAG, "Restored state: phase=%u step=%u", (unsigned)s_phase, (unsigned)s_step);
    return ESP_OK;
}

sm_phase_t sm_get_phase(void)
{
    return s_phase;
}

uint8_t sm_get_step(void)
{
    return s_step;
}

esp_err_t sm_set_phase(sm_phase_t phase)
{
    if (phase == s_phase) {
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Phase %u → %u", (unsigned)s_phase, (unsigned)phase);
    s_phase = phase;
    s_step = 0;
    return persist();
}

esp_err_t sm_set_step(uint8_t step)
{
    if (step == s_step) {
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Step %u → %u (phase=%u)", (unsigned)s_step, (unsigned)step,
             (unsigned)s_phase);
    s_step = step;
    return persist();
}

esp_err_t sm_reset(void)
{
    ESP_LOGI(TAG, "Resetting state to defaults");
    s_phase = SM_PHASE_USB_ONLY;
    s_step = 0;
    return persist();
}
