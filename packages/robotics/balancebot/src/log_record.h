// Untethered run log (ADR-037): the row format, the RAM ring, the session that
// decides when a run is over, and the flash header that frames a saved run.
// Pure C with no SDK includes, so the host tests compile it; log.c records
// and log_store.c moves the bytes to and from flash.
#ifndef BALANCEBOT_LOG_RECORD_H
#define BALANCEBOT_LOG_RECORD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ipc.h"

// The CSV columns of `stream` and `log dump`, append-only: plots and
// tools/analyze_capture.py index them by name or position.
#define LOG_CSV_HEADER \
    "t_ms,state,angle_deg,gyro_dps,setpoint_deg,rate_sps,loop_us,ctrl,pos_m,pack_v"

// Longest CSV line log_row_format_csv() writes, newline and NUL included.
#define LOG_CSV_LINE_MAX 128

// One sample: the telemetry row in binary. 32 bytes, all fields naturally
// aligned, so 128 rows fill a 4 KB flash sector exactly.
typedef struct {
    uint32_t t_ms;     // ms since boot
    uint8_t state;     // balance_state_t
    uint8_t ctrl;      // ctrl_mode_t
    uint16_t pack_cv;  // pack voltage in 10 mV steps; 0 = no pack on D0 (pack_sense.h).
                       // Rows saved before this field read 0 here, so they read as absent.
    float angle_deg;
    float gyro_dps;
    float setpoint_deg;
    float rate_sps;
    float loop_us;
    float pos_m;
} log_row_t;

#define LOG_ROW_SIZE 32u
_Static_assert(sizeof(log_row_t) == LOG_ROW_SIZE, "log_row_t must stay 32 bytes");

// The row a snapshot read at t_ms gives. The pack voltage is core 0's reading,
// not part of core 1's snapshot.
log_row_t log_row_from_snapshot(const balance_snapshot_t *s, uint32_t t_ms, uint16_t pack_cv);

// One CSV line (with '\n') in the telemetry format. Returns the length
// written, or 0 if `len` is too short.
size_t log_row_format_csv(const log_row_t *row, char *buf, size_t len);

// ---- RAM ring: keeps the newest `capacity` rows.
typedef struct {
    log_row_t *rows;
    uint32_t capacity;
    uint32_t head;    // index the next push writes
    uint32_t count;   // rows held, <= capacity
    uint32_t pushed;  // rows pushed since the last clear (count + overwritten)
} log_ring_t;

void log_ring_init(log_ring_t *r, log_row_t *storage, uint32_t capacity);
void log_ring_clear(log_ring_t *r);
void log_ring_push(log_ring_t *r, const log_row_t *row);
// Row i, oldest first (0 <= i < count).
const log_row_t *log_ring_at(const log_ring_t *r, uint32_t i);

// ---- Session: records from the sample that first reads RUN until
// `tail_rows` samples after the first one that does not, so the fall itself
// is in the log. The ring is cleared when the next run starts, not when one
// ends, so a finished run stays readable until then.
typedef enum {
    LOG_PHASE_IDLE = 0,       // not recording; the ring holds the last run, if any
    LOG_PHASE_RECORDING = 1,  // state RUN
    LOG_PHASE_TAIL = 2,       // RUN ended; recording the tail
} log_phase_t;

typedef enum {
    LOG_ACTION_NONE = 0,
    LOG_ACTION_SAVE = 1,  // a run and its tail are complete: save the ring now
    LOG_ACTION_LOST = 2,  // re-armed during the tail: the run before was dropped
} log_action_t;

typedef struct {
    log_ring_t ring;
    uint8_t phase;      // log_phase_t
    uint8_t end_state;  // the state that ended the last run (FAULT or IDLE)
    uint32_t tail_rows;
    uint32_t tail_left;
} log_session_t;

void log_session_init(log_session_t *s, log_row_t *storage, uint32_t capacity, uint32_t tail_rows);
// Feed one sample. The caller saves on LOG_ACTION_SAVE, while the state is
// still not RUN.
log_action_t log_session_sample(log_session_t *s, const log_row_t *row);

// ---- Flash header: the first page of the log region. Written after the
// rows, so a save that never finished leaves no valid header.
#define LOG_MAGIC 0x474F4C42u  // "BLOG", little-endian
#define LOG_VERSION 1u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t row_size;
    uint32_t row_count;
    uint32_t rate_hz;      // the sampling rate when the run was recorded
    uint32_t overwritten;  // rows the ring dropped because the run outlasted it
    uint32_t save_seq;     // 1 for the first save after a boot with no log, then +1
    uint8_t end_state;     // balance_state_t that ended the run
    uint8_t reserved[3];
    uint32_t rows_crc;    // crc32 over row_count * row_size bytes of rows
    uint32_t header_crc;  // crc32 over every field above
} log_header_t;

#define LOG_HEADER_SIZE 36u
_Static_assert(sizeof(log_header_t) == LOG_HEADER_SIZE, "log_header_t layout changed");

// Fill in magic, version, row_size and header_crc, and write the header.
// Returns the bytes written, or 0 if `len` is too short.
size_t log_header_encode(log_header_t *h, uint8_t *buf, size_t len);

// True when buf holds a header of this version whose CRC matches and whose
// row_count fits max_rows. The rows' CRC is the caller's to check.
bool log_header_decode(const uint8_t *buf, size_t len, uint32_t max_rows, log_header_t *out);

// ---- Slots (#1277): the flash keeps several runs, one per slot, each a
// header sector and its rows. What a slot holds is read from its header; the
// newest run is the one with the highest save_seq.
typedef struct {
    bool valid;         // the header decodes (the rows' CRC is checked on read)
    uint32_t save_seq;  // meaningful when valid
} log_slot_info_t;

// The slot of the newest run, or -1 when no slot holds one.
int log_slots_newest(const log_slot_info_t *slots, unsigned n);

// The slot the next save goes to: the first that holds no run, otherwise the
// one with the oldest run, so each save overwrites the oldest.
unsigned log_slots_next(const log_slot_info_t *slots, unsigned n);

// The save_seq the next save takes: one above the newest, or 1.
uint32_t log_slots_next_seq(const log_slot_info_t *slots, unsigned n);

// The slot holding run `save_seq`, or -1.
int log_slots_find(const log_slot_info_t *slots, unsigned n, uint32_t save_seq);

// Fill `order` with the valid slots, newest first. Returns how many.
unsigned log_slots_order(const log_slot_info_t *slots, unsigned n, unsigned *order);

#endif  // BALANCEBOT_LOG_RECORD_H
