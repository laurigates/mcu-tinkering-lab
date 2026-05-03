#include "camera_capture.h"

#include "camera_pins.h"
#include "esp_log.h"

static const char *TAG = "cam";

esp_err_t camera_capture_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        // VGA gives ~640x480 — enough resolution for fiducial detection at
        // typical sheet distances. Phase 2 may bump this to SVGA if the ROI
        // grid needs more pixels per cell.
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init: 0x%x", err);
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
        s->set_lenc(s, 1);
    }
    ESP_LOGI(TAG, "camera ready (VGA JPEG, q=12, fb in PSRAM)");
    return ESP_OK;
}

camera_fb_t *camera_capture_snapshot(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "esp_camera_fb_get returned NULL");
        return NULL;
    }
    ESP_LOGI(TAG, "snapshot: %ux%u, %u bytes", fb->width, fb->height, (unsigned)fb->len);
    return fb;
}
