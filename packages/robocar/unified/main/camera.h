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

/** AGC gain ceiling applied at init: 0=2x 1=4x 2=8x 3=16x 4=32x 5=64x 6=128x.
 *  2x is what the esp32-camera driver itself forces for every OV2640, and it
 *  is the LOWEST of the seven — a strong suspect for dim indoor frames, but
 *  unmeasured. Kept at the historical value so the first measurements describe
 *  the camera as it has been behaving; raise it live with `cam gainceiling N`
 *  and compare the luma the planner logs. */
#define CAMERA_DEFAULT_GAINCEILING ((gainceiling_t)0)

/** Live exposure state read back over SCCB — the cheap instrument that
 *  separates "the sensor is exposure-starved" (gain pegged at the ceiling and
 *  exposure at maximum) from "the room really is dark" (plenty of headroom
 *  left). The cached sensor_t status struct cannot answer this: it holds the
 *  values last WRITTEN, not what the sensor's own AEC/AGC loops have since
 *  chosen. */
typedef struct {
    uint16_t gain;       /**< analog gain register, GAIN[7:0] + REG45[7:6] */
    uint16_t exposure;   /**< AEC integration time, REG45[5:0]:AEC[7:0]:REG04[1:0] */
    uint8_t gainceiling; /**< COM9[7:5]: 0=2x .. 6=128x */
    bool valid;          /**< false if the sensor could not be read */
} camera_exposure_t;

/**
 * @brief Initialize the camera
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_init(void);

/**
 * @brief Read the sensor's live gain / exposure / gain-ceiling registers.
 * @return true if all reads succeeded (@p out->valid is set to match).
 */
bool camera_read_exposure(camera_exposure_t *out);

/** @brief Set the AGC gain ceiling (0=2x .. 6=128x) at runtime. */
esp_err_t camera_set_gainceiling(int ceiling);

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
