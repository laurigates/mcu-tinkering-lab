#include "log_store.h"

#include <string.h>

#include "config.h"
#include "crc32.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

#define ROWS_PER_SECTOR (FLASH_SECTOR_SIZE / LOG_ROW_SIZE)

_Static_assert(LOG_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0, "log region must be sector-aligned");
_Static_assert(LOG_SLOT_SIZE == FLASH_SECTOR_SIZE + LOG_CAPACITY_ROWS * LOG_ROW_SIZE,
               "a slot must hold the header sector and every row");
_Static_assert(LOG_FLASH_SIZE == LOG_SLOTS * LOG_SLOT_SIZE, "the region must hold every slot");
_Static_assert(LOG_CAPACITY_ROWS % ROWS_PER_SECTOR == 0, "rows must fill whole sectors");
_Static_assert(LOG_HEADER_SIZE <= FLASH_PAGE_SIZE, "header must fit one page");

extern char __flash_binary_end;

static uint32_t header_offset(unsigned slot)
{
    return LOG_FLASH_OFFSET + slot * LOG_SLOT_SIZE;
}

static uint32_t rows_offset(unsigned slot)
{
    return header_offset(slot) + FLASH_SECTOR_SIZE;
}

typedef struct {
    uint32_t offset;
    const uint8_t *data;
    size_t len;
} flash_op_t;

static void do_erase(void *param)
{
    const flash_op_t *op = (const flash_op_t *)param;
    flash_range_erase(op->offset, op->len);
}

static void do_program(void *param)
{
    const flash_op_t *op = (const flash_op_t *)param;
    flash_range_program(op->offset, op->data, op->len);
}

// One erase or program, with a full watchdog budget: core 1 is locked out for
// its duration and cannot feed the watchdog itself (no-op if not running yet).
static bool flash_op(void (*fn)(void *), uint32_t offset, const uint8_t *data, size_t len)
{
    flash_op_t op = {.offset = offset, .data = data, .len = len};
    watchdog_update();
    return flash_safe_execute(fn, &op, 100) == PICO_OK;
}

bool log_store_fits(void)
{
    return (uintptr_t)&__flash_binary_end - XIP_BASE <= LOG_FLASH_OFFSET;
}

bool log_store_header(unsigned slot, log_header_t *out)
{
    if (slot >= LOG_SLOTS) {
        return false;
    }
    const uint8_t *hdr = (const uint8_t *)(XIP_BASE + header_offset(slot));
    return log_header_decode(hdr, FLASH_PAGE_SIZE, LOG_CAPACITY_ROWS, out);
}

void log_store_scan(log_slot_info_t slots[LOG_SLOTS])
{
    for (unsigned k = 0; k < LOG_SLOTS; k++) {
        log_header_t h;
        slots[k].valid = log_store_header(k, &h);
        slots[k].save_seq = slots[k].valid ? h.save_seq : 0u;
    }
}

const log_row_t *log_store_rows(unsigned slot)
{
    return (const log_row_t *)(XIP_BASE + rows_offset(slot));
}

bool log_store_rows_ok(unsigned slot, const log_header_t *h)
{
    return crc32_calc(log_store_rows(slot), (size_t)h->row_count * LOG_ROW_SIZE) == h->rows_crc;
}

log_save_result_t log_store_save(const log_ring_t *ring, log_header_t *h,
                                 log_store_abort_fn abort_fn, unsigned *slot_out)
{
    static uint8_t buf[FLASH_SECTOR_SIZE];

    if (!log_store_fits()) {
        return LOG_SAVE_NO_ROOM;
    }
    if (abort_fn()) {
        return LOG_SAVE_ABORTED;
    }
    log_slot_info_t slots[LOG_SLOTS];
    log_store_scan(slots);
    const unsigned slot = log_slots_next(slots, LOG_SLOTS);
    if (slot_out != NULL) {
        *slot_out = slot;
    }

    // Invalidate that slot's old run first: from here until the new header
    // lands, a power cut leaves the slot empty rather than a mix of two runs.
    if (!flash_op(do_erase, header_offset(slot), NULL, FLASH_SECTOR_SIZE)) {
        return LOG_SAVE_FLASH_ERROR;
    }

    const uint32_t n = ring->count;
    uint32_t crc = 0;
    for (uint32_t first = 0; first < n; first += ROWS_PER_SECTOR) {
        if (abort_fn()) {
            return LOG_SAVE_ABORTED;
        }
        uint32_t k = n - first < ROWS_PER_SECTOR ? n - first : ROWS_PER_SECTOR;
        memset(buf, 0xFF, sizeof buf);
        for (uint32_t i = 0; i < k; i++) {
            memcpy(buf + i * LOG_ROW_SIZE, log_ring_at(ring, first + i), LOG_ROW_SIZE);
        }
        crc = crc32_update(crc, buf, k * LOG_ROW_SIZE);

        const uint32_t offset = rows_offset(slot) + (first / ROWS_PER_SECTOR) * FLASH_SECTOR_SIZE;
        const size_t pages = (k * LOG_ROW_SIZE + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE;
        if (!flash_op(do_erase, offset, NULL, FLASH_SECTOR_SIZE) ||
            !flash_op(do_program, offset, buf, pages * FLASH_PAGE_SIZE)) {
            return LOG_SAVE_FLASH_ERROR;
        }
    }

    h->row_count = n;
    h->rows_crc = crc;
    memset(buf, 0xFF, FLASH_PAGE_SIZE);
    if (log_header_encode(h, buf, FLASH_PAGE_SIZE) == 0) {
        return LOG_SAVE_FLASH_ERROR;
    }
    if (abort_fn()) {
        return LOG_SAVE_ABORTED;
    }
    if (!flash_op(do_program, header_offset(slot), buf, FLASH_PAGE_SIZE)) {
        return LOG_SAVE_FLASH_ERROR;
    }

    log_header_t check;
    if (!log_store_header(slot, &check) || check.row_count != n || check.rows_crc != crc ||
        !log_store_rows_ok(slot, &check)) {
        return LOG_SAVE_VERIFY_FAILED;
    }
    return LOG_SAVE_OK;
}
