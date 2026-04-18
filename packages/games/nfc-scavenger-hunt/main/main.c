#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "game_logic.h"
#include "gemini_tts.h"
#include "i2s_audio.h"
#include "local_sounds.h"
#include "mdns.h"
#include "prompt_builder.h"
#include "rc522_driver.h"
#include "status_led.h"
#include "tag_config.h"
#include "wifi_manager.h"

static const char *TAG = "main";

/* Reset button — same GPIO on both boards */
#define PIN_RESET_BUTTON 3
#define BUTTON_HOLD_MS 2000

/* NFC polling interval */
#define NFC_POLL_INTERVAL_MS 300

/* Debounce: ignore same tag for this long after a scan */
#define TAG_DEBOUNCE_MS 3000

/* Prompt buffer size */
#define PROMPT_BUF_SIZE 1024

static uint8_t last_uid[MAX_UID_LEN];
static uint8_t last_uid_len = 0;
static TickType_t last_scan_time = 0;

static void log_uid(const char *prefix, const uint8_t *uid, uint8_t uid_len)
{
    char buf[MAX_UID_LEN * 3 + 1];
    for (int i = 0; i < uid_len; i++) {
        sprintf(&buf[i * 3], "%02X%s", uid[i], (i < uid_len - 1) ? ":" : "");
    }
    ESP_LOGI(TAG, "%s %s", prefix, buf);
}

static bool is_same_tag(const uint8_t *uid, uint8_t uid_len)
{
    if (uid_len != last_uid_len) {
        return false;
    }
    return memcmp(uid, last_uid, uid_len) == 0;
}

static void play_local_sound(sound_id_t sound)
{
    const int16_t *pcm;
    size_t len;
    local_sounds_get(sound, &pcm, &len);
    if (pcm && len > 0) {
        i2s_audio_play_mono(pcm, len, 24000);
    }
}

static void handle_new_find(const tag_entry_t *entry)
{
    /* Immediate audio feedback */
    play_local_sound(SOUND_HAPPY_CHIME);

    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "WiFi not connected — playing error tone");
        play_local_sound(SOUND_ERROR);
        return;
    }

    /* Build TTS prompt */
    char prompt[PROMPT_BUF_SIZE];
    int found = game_logic_found_count();
    int total = game_logic_total_count();

    if (game_logic_all_found()) {
        prompt_builder_celebration(prompt, sizeof(prompt));
    } else {
        prompt_builder_clue(prompt, sizeof(prompt), entry->name, found, total, entry->hint);
    }

    ESP_LOGI(TAG, "Requesting TTS: %s", prompt);

    /* Fetch TTS audio from Gemini */
    status_led_set_mode(LED_RAPID_BLINK);

    tts_result_t tts;
    esp_err_t ret = gemini_tts_request(prompt, &tts);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TTS request failed: %s", esp_err_to_name(ret));
        status_led_set_mode(LED_SLOW_BLINK);
        play_local_sound(SOUND_ERROR);
        return;
    }

    /* Play the AI-generated voice clue */
    status_led_set_mode(LED_SOLID_ON);

    ret = i2s_audio_queue_playback(tts.pcm_data, tts.pcm_len, tts.sample_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio queue failed");
        heap_caps_free(tts.pcm_data);
        status_led_set_mode(LED_SLOW_BLINK);
        return;
    }

    /* Wait for playback to finish (up to 30 seconds) */
    i2s_audio_wait_done(30000);

    /* Celebration mode if all found */
    if (game_logic_all_found()) {
        status_led_set_mode(LED_PULSE);
        ESP_LOGI(TAG, "ALL TOYS FOUND! Scavenger hunt complete!");
    } else {
        status_led_set_mode(LED_SLOW_BLINK);
    }
}

static bool check_reset_button(void)
{
    if (gpio_get_level(PIN_RESET_BUTTON) == 0) {
        /* Button pressed — wait to see if it's a long hold */
        TickType_t press_start = xTaskGetTickCount();
        while (gpio_get_level(PIN_RESET_BUTTON) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if ((xTaskGetTickCount() - press_start) * portTICK_PERIOD_MS >= BUTTON_HOLD_MS) {
                return true;
            }
        }
    }
    return false;
}

static void scan_task(void *arg)
{
    uint8_t uid[MAX_UID_LEN];
    uint8_t uid_len;

    while (true) {
        /* Check reset button */
        if (check_reset_button()) {
            ESP_LOGI(TAG, "Reset button held — clearing progress");
            game_logic_reset();
            play_local_sound(SOUND_STARTUP);
            status_led_set_mode(LED_SLOW_BLINK);
            last_uid_len = 0;
        }

        /* Poll NFC reader */
        if (rc522_poll_tag(uid, &uid_len)) {
            /* Debounce: skip if same tag was just scanned */
            TickType_t now = xTaskGetTickCount();
            if (is_same_tag(uid, uid_len) &&
                (now - last_scan_time) * portTICK_PERIOD_MS < TAG_DEBOUNCE_MS) {
                vTaskDelay(pdMS_TO_TICKS(NFC_POLL_INTERVAL_MS));
                continue;
            }

            /* Remember this tag for debouncing */
            memcpy(last_uid, uid, uid_len);
            last_uid_len = uid_len;
            last_scan_time = now;

            log_uid("Tag detected:", uid, uid_len);

            /* Process through game logic */
            const tag_entry_t *entry = NULL;
            tag_result_t result = game_logic_process_tag(uid, uid_len, &entry);

            switch (result) {
                case TAG_RESULT_UNKNOWN:
                    log_uid("Unknown tag:", uid, uid_len);
                    play_local_sound(SOUND_UNKNOWN_TAG);
                    break;

                case TAG_RESULT_ALREADY_FOUND:
                    ESP_LOGI(TAG, "Already found: %s", entry->name);
                    play_local_sound(SOUND_ALREADY_FOUND);
                    break;

                case TAG_RESULT_NEW_FIND:
                    ESP_LOGI(TAG, "New find: %s!", entry->name);
                    handle_new_find(entry);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(NFC_POLL_INTERVAL_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== NFC Scavenger Hunt ===");
    ESP_LOGI(TAG, "Free DRAM: %lu KB",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    ESP_LOGI(TAG, "Free PSRAM: %lu KB",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);

    /* Initialize status LED first for visual feedback */
    ESP_ERROR_CHECK(status_led_init());
    ESP_ERROR_CHECK(status_led_start_task());
    status_led_set_mode(LED_RAPID_BLINK);

    /* Generate local sound effects */
    local_sounds_init();

    /* Initialize I2S audio output */
    ESP_ERROR_CHECK(i2s_audio_init());
    ESP_ERROR_CHECK(i2s_audio_start_task());

    /* Play startup jingle */
    play_local_sound(SOUND_STARTUP);

    /* Initialize game logic (NVS) */
    ESP_ERROR_CHECK(game_logic_init());

    /* Initialize NFC reader */
    ESP_ERROR_CHECK(rc522_init());

    /* Configure reset button with internal pull-up */
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << PIN_RESET_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    /* Connect to WiFi */
    ESP_LOGI(TAG, "Connecting to WiFi...");
    esp_err_t wifi_ret = wifi_manager_init();
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi failed — device will work with local sounds only");
        play_local_sound(SOUND_ERROR);
    } else {
        /* Initialize mDNS for nfc-scavenger-hunt.local hostname */
        ESP_ERROR_CHECK(mdns_init());
        ESP_ERROR_CHECK(mdns_hostname_set("nfc-scavenger-hunt"));
        ESP_ERROR_CHECK(mdns_instance_name_set("NFC Scavenger Hunt"));
        ESP_LOGI(TAG, "mDNS initialized: nfc-scavenger-hunt.local");
    }

    /* Ready to scan */
    status_led_set_mode(LED_SLOW_BLINK);
    ESP_LOGI(TAG, "Ready! Tap a toy to start the hunt.");
    ESP_LOGI(TAG, "Progress: %d/%d found", game_logic_found_count(), game_logic_total_count());

    /* Start the main scan task on Core 0 */
    xTaskCreatePinnedToCore(scan_task, "scan", 8192, NULL, 4, NULL, 0);
}
