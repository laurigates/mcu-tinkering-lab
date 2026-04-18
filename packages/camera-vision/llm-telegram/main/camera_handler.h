#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_camera.h"
#include "esp_err.h"

// Camera status structure
typedef struct {
    bool is_initialized;
    bool is_capturing;
    uint32_t capture_count;
    uint32_t error_count;
    size_t last_frame_size;
} camera_status_t;

// Camera capture callback
typedef void (*camera_capture_cb)(const uint8_t *data, size_t size, void *user_data);

// Initialize camera
esp_err_t camera_init(void);

// Deinitialize camera
void camera_deinit(void);

// Capture single frame
esp_err_t camera_capture_frame(uint8_t **buffer, size_t *size);

// Capture and encode to JPEG
esp_err_t camera_capture_jpeg(uint8_t **buffer, size_t *size);

// Start continuous capture with callback
esp_err_t camera_start_capture(uint32_t interval_ms, camera_capture_cb callback, void *user_data);

// Stop continuous capture
esp_err_t camera_stop_capture(void);

// Get camera status
camera_status_t camera_get_status(void);

// Adjust camera settings
esp_err_t camera_set_quality(int quality);
esp_err_t camera_set_brightness(int brightness);
esp_err_t camera_set_contrast(int contrast);
esp_err_t camera_set_saturation(int saturation);

// Take snapshot and save to buffer
esp_err_t camera_take_snapshot(uint8_t *buffer, size_t buffer_size, size_t *actual_size);

// Free camera buffer
void camera_free_buffer(uint8_t *buffer);

#endif  // CAMERA_HANDLER_H
