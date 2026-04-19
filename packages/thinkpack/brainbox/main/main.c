/**
 * @file main.c
 * @brief ThinkPack Brainbox — application entry point.
 *
 * Wires together credential loading, local peripherals, WiFi, the ESP-NOW
 * mesh, the group-mode LLM coordinator, and the selected AI backend.
 * A periodic status task renders the display and fires surprise-mode LLM
 * queries every 5 minutes when peers are present.
 */

#include <string.h>

#include "credentials_loader.h"
#include "display_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "group_mode.h"
#include "ota_handler.h"
#include "sdkconfig.h"
#include "standalone_mode.h"
#include "thinkpack_ai.h"
#include "thinkpack_power.h"
#include "thinkpack_protocol.h"
#include "wifi_manager.h"

static const char *TAG = "brainbox_main";

/* ------------------------------------------------------------------ */
/* AI backend name from Kconfig                                        */
/* ------------------------------------------------------------------ */

#if defined(CONFIG_AI_BACKEND_CLAUDE)
#define AI_BACKEND_NAME "Claude"
#elif defined(CONFIG_AI_BACKEND_GEMINI)
#define AI_BACKEND_NAME "Gemini"
#else
#define AI_BACKEND_NAME "Ollama"
#endif

/* ------------------------------------------------------------------ */
/* Surprise-mode period                                                */
/* ------------------------------------------------------------------ */

/** Fire a periodic surprise LLM query every 5 minutes (FR-T20). */
#define SURPRISE_INTERVAL_MS (5u * 60u * 1000u)

/* ------------------------------------------------------------------ */
/* Status task                                                         */
/* ------------------------------------------------------------------ */

/**
 * Renders the display every 5 s. Every SURPRISE_INTERVAL_MS, fires a
 * "periodic surprise" LLM query if at least one peer is present.
 */
static void status_task(void *pvParam)
{
    const TickType_t render_period = pdMS_TO_TICKS(5000);
    uint32_t last_surprise_ms = 0;

    for (;;) {
        vTaskDelay(render_period);

        /* Keep display peer count up to date. */
        display_manager_set_peer_count(thinkpack_mesh_peer_count());

        /* Update WiFi status. */
        if (wifi_manager_is_connected()) {
            wifi_info_t info = {0};
            if (wifi_manager_get_info(&info) == ESP_OK) {
                display_manager_set_wifi_status(true, info.ip_address);
            } else {
                display_manager_set_wifi_status(true, "");
            }
        } else {
            display_manager_set_wifi_status(false, "");
        }

        display_manager_render();

        /* Periodic surprise mode (FR-T20). */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (thinkpack_mesh_peer_count() > 0 &&
            (last_surprise_ms == 0 || (now_ms - last_surprise_ms) >= SURPRISE_INTERVAL_MS)) {
            last_surprise_ms = now_ms;
            group_mode_trigger_llm("periodic surprise");
        }
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "ThinkPack Brainbox starting (backend: %s)", AI_BACKEND_NAME);

    /* --- Credentials ------------------------------------------------ */
    /* Stage 1 exposes are_credentials_available() as the unified loader.
     * Calling it forces NVS → env → credentials.h load + validation.
     * Individual getters (get_wifi_ssid, get_claude_api_key, etc.) are
     * used later when passing values to subsystems.                     */
    bool creds_ok = are_credentials_available();
    if (!creds_ok) {
        ESP_LOGW(TAG, "Credentials not fully loaded — WiFi/LLM may fall back to Improv");
    }

    /* --- Local peripherals ----------------------------------------- */
    ESP_ERROR_CHECK(standalone_mode_init());
    ESP_ERROR_CHECK(display_manager_init());
    display_manager_set_backend(AI_BACKEND_NAME);

    /* --- WiFi ------------------------------------------------------- */
    ESP_ERROR_CHECK(wifi_manager_init());

    /* Improv WiFi fallback. If nothing is stored in NVS, block on UART0
     * until the browser provides credentials (up to 10 minutes). When
     * credentials already exist this returns ESP_OK immediately so we
     * don't re-prompt on every boot. */
    if (!creds_ok) {
        esp_err_t improv_ret =
            wifi_manager_start_improv_provisioning(10u * 60u * 1000u); /* 10 minute timeout */
        if (improv_ret != ESP_OK) {
            ESP_LOGW(TAG,
                     "Improv provisioning did not complete (%s) — Brainbox will operate in "
                     "offline mode",
                     esp_err_to_name(improv_ret));
        }
    }

    /* Empty strings → wifi_manager loads credentials from NVS / credentials.h. */
    esp_err_t wifi_ret = wifi_manager_connect("", "");
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi connect returned %s — continuing (Improv may provision later)",
                 esp_err_to_name(wifi_ret));
    }

    /* --- Mesh ------------------------------------------------------- */
    thinkpack_mesh_config_t mesh_cfg = {
        .box_type = (uint8_t)BOX_BRAINBOX,
        .capabilities = (uint16_t)(CAP_WIFI | CAP_LLM | CAP_DISPLAY | CAP_STORAGE | CAP_AUDIO_OUT),
        .channel = 1,
        .battery_level = 100,
        .beacon_interval_ms = 500,
    };
    strncpy(mesh_cfg.name, "brainbox", THINKPACK_BOX_NAME_LEN - 1);

    ESP_ERROR_CHECK(thinkpack_mesh_init(&mesh_cfg));

    /* group_mode must be initialised before setting the callback so that
     * the worker task is ready to receive triggers from the event handler. */
    ESP_ERROR_CHECK(group_mode_init());
    ESP_ERROR_CHECK(thinkpack_mesh_set_event_callback(group_mode_on_event, NULL));
    ESP_ERROR_CHECK(thinkpack_mesh_start());

    /* --- OTA handler (PR F) ---------------------------------------- */
    /* Initialised AFTER mesh so the ESP-NOW broadcaster has peers to
     * push to. Failure is non-fatal — Brainbox still operates. */
    (void)ota_handler_init();

    /* --- Power monitor ---------------------------------------------- */
    /* After mesh/OTA so the classifier can eventually feed a
     * thinkpack_mesh_set_beacon_interval_ms() hook without race.
     * GPIO1/ADC1_CH0 by default: verify against WIRING.md. */
    (void)thinkpack_power_init(
        &(power_config_t){.adc_gpio = 1, .tick_interval_ms = 5000, .divider_ratio_x10 = 20});

    /* --- AI backend ------------------------------------------------- */
    const thinkpack_ai_backend_t *ai = thinkpack_ai_get_current();
    ai_config_t ai_cfg = {
    /* Stage 1 credential API:
     *   get_claude_api_key()  — returns the Claude key (or NULL)
     *   get_gemini_api_key()  — returns the Gemini key (or NULL)
     * Neither Stage 1 header exposes a generic get_api_key() or
     * get_api_url(); we select per-backend below.                  */
#if defined(CONFIG_AI_BACKEND_CLAUDE)
        .api_key = get_claude_api_key(),
#elif defined(CONFIG_AI_BACKEND_GEMINI)
        .api_key = get_gemini_api_key(),
#else
        .api_key = NULL, /* Ollama does not use an API key */
#endif
        .api_url = NULL, /* each backend uses its compiled-in default URL */
        .model = NULL,   /* each backend uses its compiled-in default model */
    };

    esp_err_t ai_ret = ai->init(&ai_cfg);
    if (ai_ret != ESP_OK) {
        ESP_LOGW(TAG, "AI backend '%s' init failed: %s — continuing without LLM", AI_BACKEND_NAME,
                 esp_err_to_name(ai_ret));
    } else {
        ESP_LOGI(TAG, "AI backend '%s' initialised", AI_BACKEND_NAME);
    }

    /* --- Status task ------------------------------------------------ */
    xTaskCreatePinnedToCore(status_task, "brain_status", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "Brainbox initialisation complete");
}
