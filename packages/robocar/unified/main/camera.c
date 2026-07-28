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
         * AEC/AGC loops rewrite the exposure and gain registers every frame, so
         * a manual value survives at most one frame. They read like deliberate
         * exposure tuning and explained nothing; steer brightness with
         * set_ae_level() and the gain ceiling instead, or turn auto off first.
         *
         * set_gainceiling() is likewise NOT called here any more, and that is
         * the fix for the dark frames. It used to pass (gainceiling_t)0 —
         * meaningful on an OV2640, where the argument is an enum index and 0
         * selects 2x. This board is an OV3660 (PID 0x3660, logged below), whose
         * driver writes the argument as a RAW 10-bit ceiling into 0x3A18/0x3A19.
         * Zero therefore clamped the AGC ceiling to *no gain at all*, against
         * the sensor's own power-on default of 0x00F8 = 248 = 15.5x. Leaving the
         * driver's default in place is both correct and sensor-agnostic; tune it
         * live with `cam gainceiling N` if a room really needs it. */
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

/** OV3660: 16-bit register addresses, read one byte at a time (mask <= 0xFF
 *  selects the driver's single-byte path). Layouts are from ov3660_regs.h:
 *    exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]} / 16 rows
 *    gain     = {0x350A[1:0], 0x350B[7:0]} / 16
 *    ceiling  = {0x3A18[1:0], 0x3A19[7:0]} / 16   (power-on default 248 = 15.5x)
 */
static bool read_exposure_ov3660(sensor_t *s, camera_exposure_t *out)
{
    const int e2 = s->get_reg(s, 0x3500, 0x0F);
    const int e1 = s->get_reg(s, 0x3501, 0xFF);
    const int e0 = s->get_reg(s, 0x3502, 0xFF);
    const int g1 = s->get_reg(s, 0x350A, 0x03);
    const int g0 = s->get_reg(s, 0x350B, 0xFF);
    const int c1 = s->get_reg(s, 0x3A18, 0x03);
    const int c0 = s->get_reg(s, 0x3A19, 0xFF);

    if (e2 < 0 || e1 < 0 || e0 < 0 || g1 < 0 || g0 < 0 || c1 < 0 || c0 < 0) {
        return false;
    }

    out->exposure = ((uint32_t)(e2 & 0x0F) << 16) | ((uint32_t)e1 << 8) | (uint32_t)e0;
    out->gain = ((uint32_t)(g1 & 0x03) << 8) | (uint32_t)g0;
    out->gainceiling = (uint16_t)(((c1 & 0x03) << 8) | c0);
    return true;
}

/** OV2640: get_reg() encodes the register bank in bit 8 of `reg`
 *  (read_reg(sensor, (reg >> 8) & 0x01, reg & 0xFF); BANK_SENSOR == 1), so
 *  sensor-bank registers are addressed as 0x1xx. */
static bool read_exposure_ov2640(sensor_t *s, camera_exposure_t *out)
{
    const int gain = s->get_reg(s, 0x100, 0xFF);   // GAIN[7:0]
    const int aec = s->get_reg(s, 0x110, 0xFF);    // AEC[7:0]   — exposure bits 9:2
    const int reg04 = s->get_reg(s, 0x104, 0x03);  // REG04[1:0] — exposure bits 1:0
    const int reg45 = s->get_reg(s, 0x145, 0xFF);  // [5:0] exposure 15:10, [7:6] gain 9:8
    const int com9 = s->get_reg(s, 0x114, 0xFF);   // COM9[7:5] = AGC gain ceiling

    if (gain < 0 || aec < 0 || reg04 < 0 || reg45 < 0 || com9 < 0) {
        return false;
    }

    out->gain = (uint32_t)(((reg45 & 0xC0) << 2) | gain);
    out->exposure = (uint32_t)(((reg45 & 0x3F) << 10) | (aec << 2) | (reg04 & 0x03));
    out->gainceiling = (uint16_t)((com9 >> 5) & 0x07);
    return true;
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
    out->pid = s->id.PID;

    /* Branch on the fitted sensor rather than assuming. Reading one sensor's
     * register map through another's driver returns plausible-looking numbers
     * that mean nothing — this instrument exists to end that class of mistake,
     * not to add to it. */
    bool ok;
    switch (s->id.PID) {
        case OV3660_PID:
            ok = read_exposure_ov3660(s, out);
            break;
        case OV2640_PID:
            ok = read_exposure_ov2640(s, out);
            break;
        default:
            return false; /* unknown register map — say nothing rather than guess */
    }

    out->valid = ok;
    return ok;
}

void camera_format_exposure(char *out, size_t n, const camera_exposure_t *exp)
{
    if (!out || n == 0) {
        return;
    }
    if (!exp || !exp->valid) {
        snprintf(out, n, "sensor=UNREADABLE");
        return;
    }

    if (exp->pid == OV3660_PID) {
        /* Gain and ceiling are 1/16 steps; print the multiplier, which is the
         * number worth acting on ("pegged at the ceiling" is the diagnosis). */
        snprintf(out, n, "gain=%u.%02ux exp=%u ceil=%u.%02ux", (unsigned)(exp->gain / 16),
                 (unsigned)((exp->gain % 16) * 100 / 16), (unsigned)(exp->exposure / 16),
                 (unsigned)(exp->gainceiling / 16), (unsigned)((exp->gainceiling % 16) * 100 / 16));
    } else {
        snprintf(out, n, "gain=%u exp=%u ceil=%u(%ux)", (unsigned)exp->gain,
                 (unsigned)exp->exposure, exp->gainceiling, 1u << (exp->gainceiling + 1));
    }
}

int camera_gainceiling_max(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return 0;
    }
    /* OV3660 takes a raw 10-bit ceiling; OV2640 an enum index. Getting this
     * wrong is not a range error, it is a silent exposure change. */
    return (s->id.PID == OV3660_PID) ? 1023 : 6;
}

esp_err_t camera_set_gainceiling(int ceiling)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s || !s->set_gainceiling) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ceiling < 0 || ceiling > camera_gainceiling_max()) {
        return ESP_ERR_INVALID_ARG;
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
