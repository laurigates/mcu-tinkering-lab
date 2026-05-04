#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <stddef.h>

#include "esp_camera.h"
#include "esp_err.h"

esp_err_t camera_capture_init(void);

// Take a single JPEG snapshot. Caller must release with esp_camera_fb_return().
camera_fb_t *camera_capture_snapshot(void);

#endif  // CAMERA_CAPTURE_H
