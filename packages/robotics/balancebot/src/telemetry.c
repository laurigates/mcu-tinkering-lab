#include "telemetry.h"

#include <stdio.h>

#include "balance_control.h"
#include "config.h"
#include "pico/stdlib.h"

static bool enabled;
static uint32_t interval_ms = 1000 / TELEMETRY_DEFAULT_HZ;
static absolute_time_t next_due;
static bool header_pending;

void telemetry_init(void)
{
    enabled = false;
    next_due = get_absolute_time();
}

void telemetry_set_enabled(bool en, int hz)
{
    if (hz > 0) {
        if (hz > 200) {
            hz = 200;
        }
        interval_ms = 1000u / (uint32_t)hz;
    }
    enabled = en;
    header_pending = en;
    next_due = get_absolute_time();
}

bool telemetry_is_enabled(void)
{
    return enabled;
}

void telemetry_poll(void)
{
    if (!enabled || absolute_time_diff_us(get_absolute_time(), next_due) > 0) {
        return;
    }
    next_due = make_timeout_time_ms(interval_ms);

    if (header_pending) {
        printf("t_ms,state,angle_deg,gyro_dps,setpoint_deg,rate_sps,loop_us\n");
        header_pending = false;
    }

    balance_snapshot_t s;
    balance_control_get_snapshot(&s);
    printf("%lu,%u,%.3f,%.2f,%.3f,%.1f,%.1f\n",
           (unsigned long)to_ms_since_boot(get_absolute_time()), s.state, (double)s.pitch_deg,
           (double)s.gyro_dps, (double)s.angle_setpoint, (double)s.rate_cmd_sps, (double)s.loop_us);
}
