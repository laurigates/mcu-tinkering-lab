/**
 * @file log_mirror.h
 * @brief Mirror every ESP_LOG line to UART0 TX (XIAO D6, carrier header J15)
 *        (issue #798).
 *
 * The console stays on USB-Serial-JTAG: stdin, stdout, command_task's input
 * and Improv provisioning are untouched. What changes is that each ESP_LOG
 * line is ALSO queued for UART0, so J15 carries the application log and not
 * only the ROM boot log. Two reasons the USB console alone is not enough:
 * it silently drops TX once the host stops reading for 50 ms, and it is not
 * there at all off the tether.
 *
 * Only ESP_LOG output is mirrored. Anything written to stdout directly —
 * console command replies, the frame and mic dumps' printf() lines, Improv
 * packets (improv_console_write() calls write() on stdout) — goes to the USB
 * console only, because it never passes through esp_log's vprintf hook.
 *
 * Best effort by design: a line that does not fit the queue is dropped and
 * counted, never waited for, so a slow 115200-baud wire cannot stall the task
 * that logged. The drop count is printed on UART0 itself once there is room.
 */

#ifndef LOG_MIRROR_H
#define LOG_MIRROR_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Install the UART0 driver (TX only) and the esp_log vprintf hook.
 *
 * Call once, early in app_main. Idempotent. Non-fatal for the caller: on
 * failure nothing is hooked, the log stays on the USB console alone, and the
 * error is returned for the caller to log.
 *
 * @return ESP_OK, ESP_ERR_NO_MEM, or the UART driver's error.
 */
esp_err_t log_mirror_init(void);

#ifdef __cplusplus
}
#endif

#endif  // LOG_MIRROR_H
