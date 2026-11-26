// The run log's flash region (ADR-037): LOG_SLOTS slots at LOG_FLASH_OFFSET,
// each a header sector then the rows of one run, sector by sector (#1277).
// Writes go through flash_safe_execute(), one sector erase or program per call
// with the watchdog fed before each, and MUST only happen while the motors are
// disarmed. Called from core 0.
#ifndef BALANCEBOT_LOG_STORE_H
#define BALANCEBOT_LOG_STORE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "log_record.h"

typedef enum {
    LOG_SAVE_OK = 0,
    LOG_SAVE_ABORTED,        // the robot armed (or was found armed) before the save finished
    LOG_SAVE_FLASH_ERROR,    // flash_safe_execute() failed
    LOG_SAVE_VERIFY_FAILED,  // the read-back header or rows did not match
    LOG_SAVE_NO_ROOM,        // the firmware image reaches into the log region
} log_save_result_t;

// Called before every flash operation; returning true stops the save.
typedef bool (*log_store_abort_fn)(void);

// Write the ring's rows and then `h` into the slot log_slots_next() picks
// (an empty slot, else the oldest run's). The caller fills in rate_hz,
// overwritten, save_seq and end_state; row_count and the CRCs are set here.
// That slot's old run is invalidated first, so an interrupted save leaves it
// empty and the other slots untouched. *slot_out, when not NULL, is the slot
// used.
log_save_result_t log_store_save(const log_ring_t *ring, log_header_t *h,
                                 log_store_abort_fn abort_fn, unsigned *slot_out);

// Read every slot's header (no row CRC).
void log_store_scan(log_slot_info_t slots[LOG_SLOTS]);

// Slot `slot`'s header, if it holds a valid one.
bool log_store_header(unsigned slot, log_header_t *out);

// Slot `slot`'s rows (XIP), valid for the row_count of a header that
// log_store_rows_ok() accepts.
const log_row_t *log_store_rows(unsigned slot);

// True when slot `slot`'s rows match the header's CRC.
bool log_store_rows_ok(unsigned slot, const log_header_t *h);

// False when the firmware image reaches into the log region.
bool log_store_fits(void);

#endif  // BALANCEBOT_LOG_STORE_H
