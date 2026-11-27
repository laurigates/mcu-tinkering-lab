/**
 * @file wall_clock.c
 * @brief SNTP start and wall-time helpers (ADR-039 §1).
 */

#include "wall_clock.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"

static const char *TAG = "wall_clock";

/* The robot lives in Finland. POSIX form, so no tz database is needed. */
#define WALL_CLOCK_TZ "EET-2EEST,M3.5.0/3,M10.5.0/4"
#define WALL_CLOCK_NTP_SERVER "pool.ntp.org"

static bool s_started;

static void on_sync(struct timeval *tv)
{
    (void)tv;
    ESP_LOGI(TAG, "clock synced by SNTP");
}

void wall_clock_start(void)
{
    if (s_started) {
        return;
    }
    s_started = true;
    setenv("TZ", WALL_CLOCK_TZ, 1);
    tzset();

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(WALL_CLOCK_NTP_SERVER);
    cfg.sync_cb = on_sync;
    const esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SNTP not started (%s) — timestamps stay boot-relative",
                 esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "SNTP started (%s)", WALL_CLOCK_NTP_SERVER);
}

quest_time_t wall_clock_now(void)
{
    return quest_time_from((int64_t)time(NULL), esp_timer_get_time() / 1000000);
}

bool wall_clock_synced(void)
{
    return wall_clock_now().synced;
}

size_t wall_clock_format(quest_time_t t, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return 0;
    }
    if (!t.synced) {
        const int n = snprintf(out, out_len, "boot+%llds", (long long)t.s);
        return n < 0 ? 0 : (size_t)n;
    }
    const time_t tt = (time_t)t.s;
    struct tm tm_local;
    localtime_r(&tt, &tm_local);
    return strftime(out, out_len, "%Y-%m-%d %H:%M", &tm_local);
}
