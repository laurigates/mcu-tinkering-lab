// Untethered run log (ADR-037), driven from core 0. Samples the core-1
// snapshot into a RAM ring while the robot balances, and copies each finished
// run (with LOG_TAIL_S after it) to flash, where `log dump` reads it.
#ifndef BALANCEBOT_LOG_H
#define BALANCEBOT_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "log_store.h"

typedef struct {
    bool enabled;
    uint32_t rate_hz;
    uint8_t phase;           // log_phase_t
    uint32_t rows;           // rows in the RAM ring
    uint32_t saves;          // saves attempted since boot
    log_save_result_t last;  // result of the last one (saves > 0)
    uint32_t lost_runs;      // runs dropped because the robot re-armed during the tail
    bool fits;               // log_store_fits()
} log_status_t;

void log_init(void);

// Call frequently from the core-0 loop. Blocks for the flash write (about
// 0.06 s per 128 rows) when a run has ended.
void log_poll(void);

// hz <= 0 keeps the current rate; above LOG_MAX_HZ is capped. Turning the
// log off drops a run in progress.
void log_set_enabled(bool enabled, int hz);

log_status_t log_get_status(void);

#endif  // BALANCEBOT_LOG_H
