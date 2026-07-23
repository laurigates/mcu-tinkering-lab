/**
 * @file self_report.c
 * @brief Spoken self-introduction and status self-diagnostic. See the header.
 */

#include "self_report.h"

#include <string.h>

#include "audio_player.h"
#include "credentials_loader.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gemini_backend.h"
#include "gpio_expander.h"
#include "i2c_bus.h"
#include "mqtt_logger.h"
#include "speech_queue.h"
#include "wifi_manager.h"

static const char *TAG = "self_report";

/* -------------------------------------------------------------------------- */
/* Task tuning                                                                 */
/* -------------------------------------------------------------------------- */

#define STATUS_MONITOR_TASK_STACK_SIZE 4096U
#define STATUS_MONITOR_TASK_PRIORITY 2U
#define STATUS_MONITOR_TASK_CORE 1  //!< bursty + network-bound, like the planner

/** How often the monitor samples subsystem health. */
#define STATUS_POLL_MS 3000U

/** Minimum gap between spoken announcements — guards against chatter when a
 *  flaky link toggles repeatedly. */
#define ANNOUNCE_RATE_LIMIT_US (15 * 1000 * 1000)

/* -------------------------------------------------------------------------- */
/* Boot-recorded results                                                       */
/* -------------------------------------------------------------------------- */

/* Only the camera is stored: it has no live "is ready" accessor, so its boot
 * result is the sole source for camera_ok. The I2C bus and audio have live
 * accessors (i2c_bus_is_ready / audio_player_is_ready) that collect() reads, so
 * recording them here would be dead state — they are only logged. */
static bool s_camera_ok = false;

static TaskHandle_t s_monitor_task = NULL;

void self_report_note_init(self_report_subsystem_t subsystem, bool ok)
{
    if (subsystem == SELF_REPORT_SUBSYS_CAMERA) {
        s_camera_ok = ok;
    }
    ESP_LOGI(TAG, "init: subsystem=%d ok=%d", (int)subsystem, (int)ok);
}

/* -------------------------------------------------------------------------- */
/* Snapshot + facts                                                            */
/* -------------------------------------------------------------------------- */

void self_report_collect(robocar_status_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));

    /* Boot-recorded (the camera has no live "is ready" accessor). */
    out->camera_ok = s_camera_ok;

    /* Live accessors — authoritative, and what makes change-detection work. */
    out->i2c_bus_ok = i2c_bus_is_ready();
    out->audio_ok = audio_player_is_ready();
    out->mcp23017_present = gpio_expander_available();
    out->wifi_up = wifi_is_connected();

    const char *key = get_gemini_api_key();
    out->key_present = (key != NULL && key[0] != '\0');

    const char *ssid = get_wifi_ssid();
    strlcpy(out->ssid, (ssid != NULL) ? ssid : "", sizeof(out->ssid));

    const esp_app_desc_t *desc = esp_app_get_description();
    strlcpy(out->version, (desc != NULL) ? desc->version : "?", sizeof(out->version));
}

bool self_report_voice_able(const robocar_status_t *status)
{
    return status != NULL && status->wifi_up && status->key_present && status->audio_ok;
}

/** Required-subsystem state word: fault reads differently from "ok". */
static const char *req_state(bool ok)
{
    return ok ? "ok" : "not-responding";
}

size_t self_report_format_facts(const robocar_status_t *status, char *buf, size_t len)
{
    if (!buf || len == 0) {
        return 0;
    }
    if (!status) {
        buf[0] = '\0';
        return 0;
    }

    int n = snprintf(buf, len,
                     "robot=robocar version=%s wifi=%s ssid=%s camera=%s i2c_peripherals=%s "
                     "audio=%s mcp23017=%s gemini_key=%s",
                     status->version, req_state(status->wifi_up),
                     status->ssid[0] ? status->ssid : "none", req_state(status->camera_ok),
                     req_state(status->i2c_bus_ok), req_state(status->audio_ok),
                     status->mcp23017_present ? "present" : "absent(optional)",
                     status->key_present ? "present" : "absent");

    if (n < 0) {
        buf[0] = '\0';
        return 0;
    }
    return (size_t)n < len ? (size_t)n : len - 1;
}

/* -------------------------------------------------------------------------- */
/* Change detection                                                            */
/* -------------------------------------------------------------------------- */

/** FNV-1a over the health booleans + ssid. A change in any one flips it. */
static uint32_t status_signature(const robocar_status_t *s)
{
    uint32_t h = 2166136261u;
#define MIX_BYTE(b)        \
    do {                   \
        h ^= (uint8_t)(b); \
        h *= 16777619u;    \
    } while (0)
    MIX_BYTE(s->wifi_up);
    MIX_BYTE(s->camera_ok);
    MIX_BYTE(s->i2c_bus_ok);
    MIX_BYTE(s->mcp23017_present);
    MIX_BYTE(s->audio_ok);
    MIX_BYTE(s->key_present);
    for (const char *p = s->ssid; *p != '\0'; ++p) {
        MIX_BYTE(*p);
    }
#undef MIX_BYTE
    return h;
}

/* -------------------------------------------------------------------------- */
/* Template fallback                                                           */
/* -------------------------------------------------------------------------- */

/* Spoken when the Gemini text call fails. Deliberately plain; truncated to the
 * speech buffer. Names only what is wrong so a healthy robot stays brief. */
static void template_line(const robocar_status_t *s, char *out, size_t len)
{
    char faults[128];
    faults[0] = '\0';
    if (!s->wifi_up) {
        strlcat(faults, " WiFi", sizeof(faults));
    }
    if (!s->camera_ok) {
        strlcat(faults, " camera", sizeof(faults));
    }
    if (!s->i2c_bus_ok) {
        strlcat(faults, " motors", sizeof(faults));
    }
    if (!s->audio_ok) {
        strlcat(faults, " audio", sizeof(faults));
    }

    if (faults[0] == '\0') {
        strlcpy(out, "Hi, I'm Robocar and all my systems are online.", len);
    } else {
        snprintf(out, len, "Hi, I'm Robocar. These parts are not responding:%s.", faults);
    }
}

/* -------------------------------------------------------------------------- */
/* Narrate + post                                                              */
/* -------------------------------------------------------------------------- */

static void narrate_and_post(const robocar_status_t *status, const char *facts, bool is_update)
{
    char line[SPEECH_TEXT_MAX];
    line[0] = '\0';

    esp_err_t ret = gemini_backend_narrate(facts, is_update, line, sizeof(line));
    if (ret != ESP_OK || line[0] == '\0') {
        ESP_LOGW(TAG, "narrate failed (%s) — speaking template line", esp_err_to_name(ret));
        template_line(status, line, sizeof(line));
    }

    esp_err_t sp = speech_queue_post(line);
    if (sp == ESP_OK) {
        ESP_LOGI(TAG, "self-report%s: \"%s\"", is_update ? " (update)" : "", line);
    } else if (sp == ESP_ERR_NO_MEM) {
        ESP_LOGD(TAG, "speech queue full — dropped self-report: \"%s\"", line);
    } else {
        ESP_LOGW(TAG, "speech_queue_post failed: %s", esp_err_to_name(sp));
    }
}

/* -------------------------------------------------------------------------- */
/* Monitor task                                                                */
/* -------------------------------------------------------------------------- */

static void status_monitor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "status monitor started on core %d, poll %u ms", xPortGetCoreID(),
             (unsigned)STATUS_POLL_MS);

    robocar_status_t status;
    char facts[SELF_REPORT_FACTS_MAX];

    bool announced = false;       /* has the first announcement fired? */
    uint32_t announced_sig = 0;   /* signature of the last spoken state */
    uint32_t pending_sig = 0;     /* a change seen once, awaiting confirm */
    bool have_pending = false;    /* one-cycle debounce armed */
    int64_t last_announce_us = 0; /* for the rate limit */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_POLL_MS));

        self_report_collect(&status);
        const uint32_t sig = status_signature(&status);

        /* The diagnostic is always logged (and published) so it is observable
         * even while the robot cannot talk. */
        self_report_format_facts(&status, facts, sizeof(facts));
        ESP_LOGI(TAG, "status: %s", facts);
        if (mqtt_logger_is_connected()) {
            mqtt_logger_publish_status(facts);
        }

        if (!self_report_voice_able(&status)) {
            /* Not able to speak yet. Drop any armed change so we don't fire a
             * stale "update" the instant voice comes up — the first-announce
             * path already covers "here is my status" at that moment. */
            have_pending = false;
            continue;
        }

        const int64_t now = esp_timer_get_time();

        /* First announcement, the moment the robot becomes voice-able. */
        if (!announced) {
            narrate_and_post(&status, facts, false);
            announced = true;
            announced_sig = sig;
            last_announce_us = now;
            have_pending = false;
            continue;
        }

        /* Re-announce on health change: debounce one cycle, then rate-limit. */
        if (sig != announced_sig) {
            if (!have_pending || pending_sig != sig) {
                pending_sig = sig;
                have_pending = true;
                continue;  // confirm it is stable next cycle
            }
            if (now - last_announce_us < ANNOUNCE_RATE_LIMIT_US) {
                continue;  // stable, but too soon — retry next cycle
            }
            narrate_and_post(&status, facts, true);
            announced_sig = sig;
            last_announce_us = now;
            have_pending = false;
        } else {
            have_pending = false;  // reverted before it could be confirmed
        }
    }
}

esp_err_t self_report_start(void)
{
    if (s_monitor_task != NULL) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        status_monitor_task, "self_report", STATUS_MONITOR_TASK_STACK_SIZE, NULL,
        STATUS_MONITOR_TASK_PRIORITY, &s_monitor_task, STATUS_MONITOR_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create status monitor task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
