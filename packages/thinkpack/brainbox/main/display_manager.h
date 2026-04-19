/**
 * @file display_manager.h
 * @brief OLED display management for the ThinkPack Brainbox.
 *
 * Provides a thin facade over the SSD1306 display.  The current implementation
 * is serial-only (ESP_LOGI) and logs a status line every 5 s when
 * display_manager_render() is called.  Real OLED rendering (SSD1306 I2C driver)
 * is deferred to a follow-up task; see issue #196.
 *
 * Callers update individual fields via the setter functions and call
 * display_manager_render() from a periodic task to flush any changes.
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the display subsystem.
 *
 * Currently a stub that logs initialisation and prepares internal state.
 * A future revision will configure I2C and the SSD1306 driver here.
 *
 * @return ESP_OK always (stub implementation).
 */
esp_err_t display_manager_init(void);

/**
 * @brief Update the displayed WiFi connection status.
 *
 * @param connected  true if the device has an IP address.
 * @param ip         NUL-terminated IP string, or NULL / "" when disconnected.
 */
void display_manager_set_wifi_status(bool connected, const char *ip);

/**
 * @brief Update the displayed peer count.
 *
 * @param count  Number of mesh peers currently tracked.
 */
void display_manager_set_peer_count(size_t count);

/**
 * @brief Update the displayed AI backend name.
 *
 * @param backend_name  Short string identifying the active backend,
 *                      e.g. "Claude", "Gemini", or "Ollama".
 */
void display_manager_set_backend(const char *backend_name);

/**
 * @brief Record the last LLM round-trip duration for display.
 *
 * @param ms  Round-trip time in milliseconds.
 */
void display_manager_set_last_llm_time_ms(uint32_t ms);

/**
 * @brief Render the current status to the display (or serial log).
 *
 * Call this from a periodic task (e.g. every 5 s).  The serial-only
 * implementation logs one status line per call.  A future OLED revision will
 * refresh the SSD1306 framebuffer here.
 */
void display_manager_render(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_MANAGER_H */
