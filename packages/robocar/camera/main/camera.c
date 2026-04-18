/**
 * @file camera.c
 * @brief ESP32-CAM camera implementation
 */

#include "camera.h"
#include "camera_pins.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "camera";

esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP32-CAM");

    camera_config_t config = {
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
        .frame_size = FRAMESIZE_QVGA,  // 320x240 for faster processing
        .jpeg_quality = 15,            // Lower number = higher quality
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

    // Power up the camera if PWDN pin is defined
    if (config.pin_pwdn != -1) {
        gpio_config_t conf = {
            .pin_bit_mask = 1LL << config.pin_pwdn,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&conf);
        gpio_set_level(config.pin_pwdn, 0);
    }

    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    // Adjust camera settings for better AI analysis
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        // Adjust settings for outdoor robotics
        s->set_brightness(s, 0);      // -2 to 2
        s->set_contrast(s, 0);        // -2 to 2
        s->set_saturation(s, 0);      // -2 to 2
        s->set_special_effect(s, 0);  // 0 to 6 (0-No Effect, 1-Negative, 2-Grayscale, 3-Red Tint,
                                      // 4-Green Tint, 5-Blue Tint, 6-Sepia)
        s->set_whitebal(s, 1);        // 0 = disable , 1 = enable
        s->set_awb_gain(s, 1);        // 0 = disable , 1 = enable
        s->set_wb_mode(s, 0);  // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 -
                               // Office, 4 - Home)
        s->set_exposure_ctrl(s, 1);               // 0 = disable , 1 = enable
        s->set_aec2(s, 0);                        // 0 = disable , 1 = enable
        s->set_ae_level(s, 0);                    // -2 to 2
        s->set_aec_value(s, 300);                 // 0 to 1200
        s->set_gain_ctrl(s, 1);                   // 0 = disable , 1 = enable
        s->set_agc_gain(s, 0);                    // 0 to 30
        s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        s->set_bpc(s, 0);                         // 0 = disable , 1 = enable
        s->set_wpc(s, 1);                         // 0 = disable , 1 = enable
        s->set_raw_gma(s, 1);                     // 0 = disable , 1 = enable
        s->set_lenc(s, 1);                        // 0 = disable , 1 = enable
        s->set_hmirror(s, 0);                     // 0 = disable , 1 = enable
        s->set_vflip(s, 0);                       // 0 = disable , 1 = enable
        s->set_dcw(s, 1);                         // 0 = disable , 1 = enable
        s->set_colorbar(s, 0);                    // 0 = disable , 1 = enable
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

camera_fb_t *camera_capture(void)
{
    ESP_LOGI(TAG, "Capturing image...");

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return NULL;
    }

    ESP_LOGI(TAG, "Image captured: %zu bytes", fb->len);
    return fb;
}

void camera_return_fb(camera_fb_t *fb)
{
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

void camera_deinit(void)
{
    esp_camera_deinit();
    ESP_LOGI(TAG, "Camera deinitialized");
}
