#include "log.h"

#include "balance_control.h"
#include "config.h"
#include "hardware/flash.h"
#include "log_record.h"
#include "pack.h"
#include "pico/stdlib.h"

// Longest wait for core 1 to apply the auto-arm hold-off before a save: a few
// ticks at 2 ms, or the 3 ms fallback with no IMU interrupt.
#define LOG_HOLDOFF_SYNC_TIMEOUT_US 20000

static log_row_t rows[LOG_CAPACITY_ROWS];
static log_session_t session;
static bool enabled;
static uint32_t rate_hz;
static uint32_t interval_us;
static absolute_time_t next_due;
static uint32_t next_seq;
static uint32_t saves;
static log_save_result_t last;
static uint32_t lost_runs;

static uint32_t tail_rows(void)
{
    uint32_t n = (uint32_t)(LOG_TAIL_S * (float)rate_hz + 0.5f);
    return n == 0 ? 1u : n;
}

void log_init(void)
{
    enabled = true;
    rate_hz = LOG_DEFAULT_HZ;
    interval_us = 1000000u / rate_hz;
    log_session_init(&session, rows, LOG_CAPACITY_ROWS, tail_rows());
    log_slot_info_t slots[LOG_SLOTS];
    log_store_scan(slots);
    next_seq = log_slots_next_seq(slots, LOG_SLOTS);
    next_due = get_absolute_time();
}

void log_set_enabled(bool en, int hz)
{
    if (hz > 0) {
        rate_hz = hz > LOG_MAX_HZ ? LOG_MAX_HZ : (uint32_t)hz;
        interval_us = 1000000u / rate_hz;
        session.tail_rows = tail_rows();
    }
    if (!en) {
        session.phase = LOG_PHASE_IDLE;
    }
    enabled = en;
    next_due = get_absolute_time();
}

log_status_t log_get_status(void)
{
    log_status_t st = {
        .enabled = enabled,
        .rate_hz = rate_hz,
        .phase = session.phase,
        .rows = session.ring.count,
        .saves = saves,
        .last = last,
        .lost_runs = lost_runs,
        .fits = log_store_fits(),
    };
    return st;
}

static bool robot_armed(void)
{
    balance_snapshot_t s;
    balance_control_get_snapshot(&s);
    return s.state == BAL_STATE_RUN;
}

// Hold off the auto-arm gesture for the length of the write, and wait until
// core 1 has applied it: core 0 is busy until the save ends, so `arm` cannot
// arrive meanwhile, and with the gesture held off nothing else enters RUN.
static bool hold_off_arm_for_save(void)
{
    const uint32_t per_sector = FLASH_SECTOR_SIZE / LOG_ROW_SIZE;
    const uint32_t sectors = (session.ring.count + per_sector - 1u) / per_sector + 1u;  // + header
    balance_cmd_t cmd = {.cmd = CMD_HOLD_OFF_ARM,
                         .value = 1.0f + LOG_SAVE_HOLDOFF_S_PER_SECTOR * (float)sectors};
    if (!balance_control_send_cmd(&cmd)) {
        return false;  // queue full
    }
    absolute_time_t deadline = make_timeout_time_us(LOG_HOLDOFF_SYNC_TIMEOUT_US);
    for (;;) {
        balance_snapshot_t s;
        balance_control_get_snapshot(&s);
        if (s.state == BAL_STATE_RUN) {
            return false;
        }
        if (s.arm_holdoff_ticks != 0) {
            return true;
        }
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
        sleep_us(500);
    }
}

static void save_run(void)
{
    saves++;
    if (!hold_off_arm_for_save()) {
        last = LOG_SAVE_ABORTED;
        return;
    }
    log_header_t h = {
        .rate_hz = rate_hz,
        .overwritten = session.ring.pushed - session.ring.count,
        .save_seq = next_seq,
        .end_state = session.end_state,
    };
    last = log_store_save(&session.ring, &h, robot_armed, NULL);
    if (last == LOG_SAVE_OK) {
        next_seq++;
    }
}

void log_poll(void)
{
    if (!enabled) {
        return;
    }
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(now, next_due) > 0) {
        return;
    }
    // Keep the nominal rate; after a stall (a CLI command, a save) resync
    // instead of catching up with a burst of identical snapshots.
    next_due = delayed_by_us(next_due, interval_us);
    if (absolute_time_diff_us(now, next_due) <= 0) {
        next_due = delayed_by_us(now, interval_us);
    }

    balance_snapshot_t s;
    balance_control_get_snapshot(&s);
    log_row_t row = log_row_from_snapshot(&s, to_ms_since_boot(now), pack_get_centivolts());
    switch (log_session_sample(&session, &row)) {
        case LOG_ACTION_SAVE:
            save_run();
            break;
        case LOG_ACTION_LOST:
            lost_runs++;
            break;
        case LOG_ACTION_NONE:
        default:
            break;
    }
}
