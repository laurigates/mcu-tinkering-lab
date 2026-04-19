/**
 * @file display_manager.c
 * @brief Serial-only display manager (SSD1306 OLED driver TBD).
 *
 * All state is held in a static struct updated by the setter functions.
 * display_manager_render() logs a formatted status line via ESP_LOGI.
 * Real OLED output is a follow-up task tracked in issue #196.
 */

#include "display_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "display_manager";

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    bool wifi_connected;
    char ip[16];
    size_t peer_count;
    char backend[16];
    uint32_t last_llm_ms;
} display_state_t;

static display_state_t s_state = {
    .wifi_connected = false,
    .ip = "",
    .peer_count = 0,
    .backend = "none",
    .last_llm_ms = 0,
};

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t display_manager_init(void)
{
    ESP_LOGI(TAG, "Display manager init (serial-only stub; OLED driver TBD)");
    memset(&s_state, 0, sizeof(s_state));
    strncpy(s_state.backend, "none", sizeof(s_state.backend) - 1);
    return ESP_OK;
}

void display_manager_set_wifi_status(bool connected, const char *ip)
{
    s_state.wifi_connected = connected;
    if (ip && ip[0] != '\0') {
        strncpy(s_state.ip, ip, sizeof(s_state.ip) - 1);
        s_state.ip[sizeof(s_state.ip) - 1] = '\0';
    } else {
        s_state.ip[0] = '\0';
    }
}

void display_manager_set_peer_count(size_t count)
{
    s_state.peer_count = count;
}

void display_manager_set_backend(const char *backend_name)
{
    if (!backend_name) {
        return;
    }
    strncpy(s_state.backend, backend_name, sizeof(s_state.backend) - 1);
    s_state.backend[sizeof(s_state.backend) - 1] = '\0';
}

void display_manager_set_last_llm_time_ms(uint32_t ms)
{
    s_state.last_llm_ms = ms;
}

void display_manager_render(void)
{
    ESP_LOGI(TAG, "[status] wifi:%s ip:%s peers:%zu backend:%s llm_ms:%" PRIu32,
             s_state.wifi_connected ? "up" : "down", s_state.ip[0] ? s_state.ip : "-",
             s_state.peer_count, s_state.backend, s_state.last_llm_ms);
}
