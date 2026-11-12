/**
 * @file checks.h
 * @brief The individual hardware checks the bringup sweep runs.
 *
 * Every check follows the same three rules, and they are what make the sweep
 * usable while soldering rather than only once the board is finished:
 *
 * 1. NON-FATAL. A check may not abort, panic, or ESP_ERROR_CHECK. A board with
 *    one wire soldered must complete the whole sweep and report thirteen SKIPs,
 *    or the tool is useless during exactly the phase it exists for.
 *
 * 2. ABSENT IS NOT BROKEN. Hardware that does not answer is CHECK_SKIP, not
 *    CHECK_FAIL. Reporting "not fitted yet" as a failure trains you to ignore
 *    failures, which is worse than having no indicator at all.
 *
 * 3. A READING, NOT A VERDICT. Every result carries a `detail` string with the
 *    number behind it — the distance, the dB, the address, the byte count.
 *    "ULTRASONIC PASS" is not actionable; "ULTRASONIC PASS 5/5 med 34cm" is.
 *    This is also what stops an unread sensor being reported as a measurement.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHECK_PASS = 0, /**< Present, and the reading looks right. */
    CHECK_WARN,     /**< Present, but the reading is suspect. Worth a look. */
    CHECK_SKIP,     /**< Not fitted, or a prerequisite was not met. Normal. */
    CHECK_FAIL,     /**< Present and misbehaving, or a hard error. */
} check_status_t;

#define CHECK_DETAIL_MAX 48

typedef struct {
    check_status_t status;
    char detail[CHECK_DETAIL_MAX];
} check_result_t;

typedef check_result_t (*check_fn_t)(void);

typedef struct {
    const char *name; /**< <= 10 chars: it has to fit an OLED row beside a verdict. */
    check_fn_t run;
} check_t;

/** The sweep, in execution order. Order is load-bearing — see checks.c. */
extern const check_t g_checks[];
extern const size_t g_check_count;

/** Short uppercase label for @p status, for the OLED and the serial table. */
const char *check_status_label(check_status_t status);

#ifdef __cplusplus
}
#endif
