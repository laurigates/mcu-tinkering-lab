/**
 * @file net_supervisor_core.c
 * @brief See net_supervisor_core.h.
 */

#include "net_supervisor_core.h"

#include <string.h>

void net_supervisor_init(net_supervisor_t *s, bool has_credentials, uint32_t now_ms)
{
    memset(s, 0, sizeof(*s));
    s->initialised = true;
    s->has_credentials = has_credentials;
    s->down_since_ms = now_ms;
}

void net_supervisor_set_credentials(net_supervisor_t *s, bool has_credentials)
{
    s->has_credentials = has_credentials;
}

net_actions_t net_supervisor_step(net_supervisor_t *s, bool link_up, uint32_t now_ms)
{
    net_actions_t a = {0};
    if (!s->initialised) {
        return a;
    }

    if (link_up && !s->link_up) {
        s->link_up = true;
        if (!s->ota_started) {
            s->ota_started = true;
            a.start_ota = true;
        }
    } else if (!link_up && s->link_up) {
        s->link_up = false;
        s->down_since_ms = now_ms;
    }

    /* MQTT follows the link on every step, so a missed edge is corrected on
     * the next one rather than leaving MQTT in the wrong state for the boot. */
    if (s->link_up && !s->mqtt_running) {
        s->mqtt_running = true;
        a.start_mqtt = true;
    } else if (!s->link_up && s->mqtt_running) {
        s->mqtt_running = false;
        a.stop_mqtt = true;
    }

    /* Unsigned subtraction: correct across the uint32 wrap at day 49. */
    const bool down_long = !s->link_up && (now_ms - s->down_since_ms) >= NET_IMPROV_REARM_MS;
    const bool want_improv = !s->link_up && (!s->has_credentials || down_long);
    if (want_improv && !s->improv_armed) {
        s->improv_armed = true;
        a.arm_improv = true;
    } else if (s->link_up && s->improv_armed) {
        s->improv_armed = false;
        a.disarm_improv = true;
    }

    return a;
}
