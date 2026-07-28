/**
 * @file camera.h
 * @brief ESP32-CAM camera interface
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"
#include "esp_err.h"

/** Frames discarded after esp_camera_init() so the AEC/AGC loops converge
 *  before the planner sees anything. Capture starts inside esp_camera_init(),
 *  i.e. while the SCCB tuning writes are still going out. */
#define CAMERA_WARMUP_FRAMES 5

/** Live exposure state read back over SCCB — the cheap instrument that
 *  separates "the sensor is exposure-starved" (gain pegged at the ceiling and
 *  exposure at maximum) from "the room really is dark" (plenty of headroom
 *  left). The cached sensor_t status struct cannot answer this: it holds the
 *  values last WRITTEN, not what the sensor's own AEC/AGC loops have since
 *  chosen.
 *
 *  Units are SENSOR-DEPENDENT, which is why @ref pid travels with them — see
 *  camera_format_exposure() for the rendering, and never interpret these
 *  without checking which sensor produced them. */
typedef struct {
    uint16_t pid;         /**< sensor PID: 0x3660 = OV3660, 0x2640 = OV2640 */
    uint32_t gain;        /**< OV3660: gain x16. OV2640: GAIN[7:0]+REG45[7:6] */
    uint32_t exposure;    /**< OV3660: rows x16 (20-bit). OV2640: 16-bit AEC */
    uint16_t gainceiling; /**< OV3660: max gain x16. OV2640: enum 0=2x..6=128x */
    bool valid;           /**< false if the sensor could not be read */
} camera_exposure_t;

/**
 * @brief Initialize the camera
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_init(void);

/**
 * @brief Read the sensor's live gain / exposure / gain-ceiling registers.
 *
 * Supports OV3660 and OV2640; returns false (with @p out zeroed) for any other
 * sensor rather than reading someone else's register map and reporting the
 * result as fact.
 *
 * @return true if all reads succeeded (@p out->valid is set to match).
 */
bool camera_read_exposure(camera_exposure_t *out);

/**
 * @brief Render an exposure reading, in the units of the sensor that produced it.
 *
 * Writes "sensor=UNREADABLE" when @p exp is invalid — an unread sensor must
 * never render as "gain=0 exp=0", which is itself a specific and actionable
 * diagnosis ("auto-exposure is not converging").
 */
void camera_format_exposure(char *out, size_t n, const camera_exposure_t *exp);

/**
 * @brief Set the AGC gain ceiling at runtime. UNITS ARE SENSOR-DEPENDENT.
 *
 * OV3660 takes a RAW ceiling in 1/16 gain steps, 0..1023 — its power-on default
 * is 248 (15.5x). OV2640 takes an enum index, 0=2x .. 6=128x. Passing an
 * OV2640-shaped 0..6 to an OV3660 sets a ceiling of essentially zero gain,
 * which is exactly the bug that made this robot's frames dark.
 */
esp_err_t camera_set_gainceiling(int ceiling);

/** @brief Largest value camera_set_gainceiling() accepts for the fitted sensor. */
int camera_gainceiling_max(void);

/** @brief Bias the auto-exposure target (-2 .. +2) at runtime. */
esp_err_t camera_set_ae_level(int level);

/** @brief Set post-processing brightness (-2 .. +2) at runtime. */
esp_err_t camera_set_brightness(int value);

/**
 * @brief Capture an image
 * @return Pointer to camera frame buffer, NULL on error
 */
camera_fb_t *camera_capture(void);

/**
 * @brief Return the camera frame buffer
 * @param fb Frame buffer to return
 */
void camera_return_fb(camera_fb_t *fb);

/**
 * @brief Deinitialize the camera
 */
void camera_deinit(void);

#endif  // CAMERA_H
