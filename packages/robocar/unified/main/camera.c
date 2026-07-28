/**
 * @file camera.c
 * @brief ESP32-CAM camera implementation
 */

#include "camera.h"
#include "camera_pins.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

        /* fb_count MUST be > 1, and grab_mode is inert below that.
         *
         * With a single buffer the driver stops capturing the moment the
         * application takes the frame (cam_hal.c: frames[n].en = 0, then
         * cam_start_frame() finds nothing enabled and drops to CAM_STATE_IDLE)
         * and only resumes on esp_camera_fb_return(). The planner holds its
         * frame across the whole Gemini call and returns it at the end of the
         * loop, so the frame handed to the NEXT call was captured ~2 VSYNCs
         * after the previous return — i.e. a full planner period (15 s) old.
         * The robot was planning motion from a 15-second-old view.
         *
         * CAMERA_GRAB_LATEST additionally drops the stale queued frame rather
         * than serving it; cam_hal.c only consults grab_mode when frame_cnt > 1,
         * so the previous CAMERA_GRAB_WHEN_EMPTY did nothing at all. */
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
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
        s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
        s->set_aec2(s, 0);           // 0 = disable , 1 = enable
        s->set_ae_level(s, 0);       // -2 to 2
        s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable

        /* set_aec_value() and set_agc_gain() used to be called here and were
         * DEAD: with set_exposure_ctrl(1) / set_gain_ctrl(1) the sensor's own
         * AEC/AGC loops rewrite REG04/AEC/REG45 and GAIN every frame, so a
         * manual value survives at most one frame. Worse, set_agc_gain(s, 0)
         * writes agc_gain_tbl[0] — the MINIMUM gain — so the frame captured
         * during this tuning block was pinned at 1x. They read like deliberate
         * exposure tuning and explained nothing; steer brightness with
         * set_ae_level() and the gain ceiling instead, or turn auto off first.
         *
         * The gain ceiling is left at the driver's own post-init default
         * (esp_camera.c forces GAINCEILING_2X for every OV2640) so the first
         * measurements describe the camera as it has been behaving. It is
         * runtime-adjustable — see camera_set_gainceiling() and the `cam`
         * console command — because only a human looking at the room can judge
         * the result. */
        s->set_gainceiling(s, CAMERA_DEFAULT_GAINCEILING);
        s->set_bpc(s, 0);       // 0 = disable , 1 = enable
        s->set_wpc(s, 1);       // 0 = disable , 1 = enable
        s->set_raw_gma(s, 1);   // 0 = disable , 1 = enable
        s->set_lenc(s, 1);      // 0 = disable , 1 = enable
        s->set_hmirror(s, 0);   // 0 = disable , 1 = enable
        s->set_vflip(s, 0);     // 0 = disable , 1 = enable
        s->set_dcw(s, 1);       // 0 = disable , 1 = enable
        s->set_colorbar(s, 0);  // 0 = disable , 1 = enable

        /* Which sensor is actually fitted decides whether any of the OV2640
         * register reasoning above applies. Nobody had ever checked. */
        ESP_LOGI(TAG, "Sensor PID=0x%04x VER=0x%02x MIDL=0x%02x MIDH=0x%02x", s->id.PID, s->id.VER,
                 s->id.MIDL, s->id.MIDH);
    }

    /* Discard warm-up frames.
     *
     * esp_camera_init() starts capture as its last step, so the first frame is
     * exposed while the SCCB tuning writes above are still going out and before
     * the sensor's AEC/AGC loops have converged — the classic near-black OV2640
     * boot frame. Without this the very first Gemini call describes it. */
    int warmed = 0;
    for (int i = 0; i < CAMERA_WARMUP_FRAMES; i++) {
        camera_fb_t *warm = esp_camera_fb_get();
        if (!warm) {
            /* Bail on the first miss rather than paying esp_camera_fb_get()'s
             * 4 s FB_GET_TIMEOUT five times over. A sensor that probes on SCCB
             * but never completes a capture (a DVP or VSYNC/HREF fault on the
             * Sense module's flex) would otherwise add ~20 s to every boot,
             * ahead of WiFi provisioning and the console. */
            ESP_LOGW(TAG, "Warm-up capture %d/%d returned no frame — skipping the rest", i + 1,
                     CAMERA_WARMUP_FRAMES);
            break;
        }
        esp_camera_fb_return(warm);
        warmed++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Camera initialized successfully (%d/%d warm-up frames discarded)", warmed,
             CAMERA_WARMUP_FRAMES);
    return ESP_OK;
}

bool camera_read_exposure(camera_exposure_t *out)
{
    if (!out) {
        return false;
    }
    *out = (camera_exposure_t){0};

    sensor_t *s = esp_camera_sensor_get();
    if (!s || !s->get_reg) {
        return false;
    }

    /* ov2640's get_reg() encodes the register bank in bit 8 of `reg`
     * (read_reg(sensor, (reg >> 8) & 0x01, reg & 0xFF); BANK_SENSOR == 1), so
     * sensor-bank registers are addressed as 0x1xx. */
    const int gain = s->get_reg(s, 0x100, 0xFF);   // GAIN[7:0]
    const int aec = s->get_reg(s, 0x110, 0xFF);    // AEC[7:0]   — exposure bits 9:2
    const int reg04 = s->get_reg(s, 0x104, 0x03);  // REG04[1:0] — exposure bits 1:0
    const int reg45 = s->get_reg(s, 0x145, 0xFF);  // [5:0] exposure 15:10, [7:6] gain 9:8
    const int com9 = s->get_reg(s, 0x114, 0xFF);   // COM9[7:5] = AGC gain ceiling

    if (gain < 0 || aec < 0 || reg04 < 0 || reg45 < 0 || com9 < 0) {
        return false;
    }

    out->gain = (uint16_t)(((reg45 & 0xC0) << 2) | gain);
    out->exposure = (uint16_t)(((reg45 & 0x3F) << 10) | (aec << 2) | (reg04 & 0x03));
    out->gainceiling = (uint8_t)((com9 >> 5) & 0x07);
    out->valid = true;
    return true;
}

esp_err_t camera_set_gainceiling(int ceiling)
{
    if (ceiling < 0 || ceiling > 6) {
        return ESP_ERR_INVALID_ARG;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (!s || !s->set_gainceiling) {
        return ESP_ERR_INVALID_STATE;
    }
    return (s->set_gainceiling(s, (gainceiling_t)ceiling) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t camera_set_ae_level(int level)
{
    if (level < -2 || level > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (!s || !s->set_ae_level) {
        return ESP_ERR_INVALID_STATE;
    }
    return (s->set_ae_level(s, level) == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t camera_set_brightness(int value)
{
    if (value < -2 || value > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (!s || !s->set_brightness) {
        return ESP_ERR_INVALID_STATE;
    }
    return (s->set_brightness(s, value) == 0) ? ESP_OK : ESP_FAIL;
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
