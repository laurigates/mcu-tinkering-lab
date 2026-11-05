/**
 * @file oled_status.c
 * @brief Gathers the status screen's facts on the device (see oled_status.h).
 */

#include "oled_status.h"

#include <string.h>

#include "esp_netif.h"
#include "i2c_bus.h"
#include "led_controller.h"
#include "motor_controller.h"
#include "oled.h"
#include "oled_status_core.h"
#include "ota_manager.h"
#include "plan_activity.h"
#include "plan_budget.h"
#include "reactive_controller.h"
#include "servo_controller.h"
#include "ultrasonic_core.h"
#include "wifi_manager.h"

_Static_assert(OLED_STATUS_ROWS <= OLED_ROWS, "oled_status: more rows than the panel has");
_Static_assert(OLED_STATUS_COLS <= OLED_COLS, "oled_status: rows wider than the panel");
_Static_assert(OLED_HEAD_ROWS * 2 <= OLED_ROWS, "oled_status: head view needs two pages a row");
_Static_assert(OLED_HEAD_COLS * 12 <= OLED_WIDTH, "oled_status: head view wider than the panel");

/** The STA interface's IPv4 address, most significant octet first. */
static bool read_ip(uint8_t ip[4])
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info;
    if (netif == NULL || esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == 0) {
        return false;
    }
    /* esp_ip4_addr_t holds the address in network byte order; byte 0 is the
     * first octet, as IP2STR reads it. */
    const uint8_t *b = (const uint8_t *)&info.ip.addr;
    memcpy(ip, b, 4);
    return true;
}

static void collect(oled_status_t *s)
{
    memset(s, 0, sizeof(*s));

    s->wifi_up = wifi_is_connected() && read_ip(s->ip);

    s->i2c_bus_ok = i2c_bus_is_ready();
    s->motors_ok = motor_is_initialized();
    s->servos_ok = servo_is_initialized();
    s->leds_ok = led_is_initialized();

    s->plan_gate_on = plan_activity_enabled();
    s->plan_dormant = plan_activity_dormant();
    s->plan_fused = !plan_budget_allows();
    s->plan_requests = plan_budget_requests();
    plan_budget_get(&s->plan_max_requests, NULL);

    reactive_telemetry_t tele;
    if (reactive_controller_get_telemetry(&tele) == ESP_OK) {
        s->sonar_known = true;
        s->reflex_active = tele.reflex_active;
        s->front_failed = tele.sensor_failed;
        s->rear_failed = tele.rear_failed || tele.distance_rear_cm == ULTRASONIC_DIST_ERROR;
        s->front_cm = tele.distance_cm;
        s->rear_cm = tele.distance_rear_cm;
    }
}

esp_err_t oled_status_init(void)
{
    const esp_err_t err = oled_init();
    if (err != ESP_OK) {
        return err;
    }
    oled_clear();
    oled_text(0, 0, "ROBOCAR");
    oled_text(0, 1, ota_manager_get_version());
    oled_text(0, 2, "BOOTING");
    return oled_flush();
}

static volatile bool s_head_view = false;

void oled_status_set_head_view(bool on)
{
    s_head_view = on;
}

bool oled_status_head_view(void)
{
    return s_head_view;
}

/** The commanded head angles at double size. servo_get_position() is the last
 *  angle the driver accepted, which is what a calibration reads off. */
static void draw_head_view(void)
{
    servo_position_t pos = {0};
    char rows[OLED_HEAD_ROWS][OLED_HEAD_COLS + 1];
    oled_clear();
    if (servo_is_initialized() && servo_get_position(&pos) == ESP_OK) {
        oled_head_format(pos.pan_angle, pos.tilt_angle, rows);
        for (uint8_t r = 0; r < OLED_HEAD_ROWS; r++) {
            oled_text_2x(0, r, rows[r]);
        }
    } else {
        oled_text_2x(0, 0, "NO SERVOS");
    }
    (void)oled_flush();
}

void oled_status_refresh(void)
{
    if (!oled_available()) {
        return;
    }
    if (s_head_view) {
        draw_head_view();
        return;
    }
    oled_status_t s;
    collect(&s);

    char rows[OLED_STATUS_ROWS][OLED_STATUS_COLS + 1];
    oled_status_format(&s, rows);

    oled_clear();
    for (uint8_t r = 0; r < OLED_STATUS_ROWS; r++) {
        oled_text(0, r, rows[r]);
    }
    (void)oled_flush();
}
