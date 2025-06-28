/**
 * @file camera.h
 * @brief ESP32-CAM camera interface
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "esp_camera.h"
#include "esp_err.h"

/**
 * @brief Initialize the camera
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_init(void);

/**
 * @brief Capture an image
 * @return Pointer to camera frame buffer, NULL on error
 */
camera_fb_t* camera_capture(void);

/**
 * @brief Return the camera frame buffer
 * @param fb Frame buffer to return
 */
void camera_return_fb(camera_fb_t* fb);

/**
 * @brief Deinitialize the camera
 */
void camera_deinit(void);

#endif // CAMERA_H