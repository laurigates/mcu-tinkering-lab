#include "camera_handler.h"
#include <string.h>
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "CAMERA_HANDLER";

// Camera pins for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Camera state
static camera_status_t camera_status = {0};
static SemaphoreHandle_t camera_mutex = NULL;
static TaskHandle_t capture_task_handle = NULL;
static camera_capture_cb capture_callback = NULL;
static void *capture_user_data = NULL;
static uint32_t capture_interval = 0;
static bool capture_running = false;

// Capture task
static void camera_capture_task(void *pvParameters)
{
    uint8_t *buffer = NULL;
    size_t size = 0;

    while (capture_running) {
        if (camera_capture_jpeg(&buffer, &size) == ESP_OK) {
            if (capture_callback) {
                capture_callback(buffer, size, capture_user_data);
            }
            camera_free_buffer(buffer);
            camera_status.capture_count++;
        } else {
            camera_status.error_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(capture_interval));
    }

    vTaskDelete(NULL);
}

// Initialize camera
esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "Initializing camera");

    if (camera_status.is_initialized) {
        ESP_LOGW(TAG, "Camera already initialized");
        return ESP_OK;
    }

    // Create mutex
    if (camera_mutex == NULL) {
        camera_mutex = xSemaphoreCreateMutex();
        if (camera_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create camera mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    // Camera configuration
    camera_config_t config = {.pin_pwdn = PWDN_GPIO_NUM,
                              .pin_reset = RESET_GPIO_NUM,
                              .pin_xclk = XCLK_GPIO_NUM,
                              .pin_sscb_sda = SIOD_GPIO_NUM,
                              .pin_sscb_scl = SIOC_GPIO_NUM,
                              .pin_d7 = Y9_GPIO_NUM,
                              .pin_d6 = Y8_GPIO_NUM,
                              .pin_d5 = Y7_GPIO_NUM,
                              .pin_d4 = Y6_GPIO_NUM,
                              .pin_d3 = Y5_GPIO_NUM,
                              .pin_d2 = Y4_GPIO_NUM,
                              .pin_d1 = Y3_GPIO_NUM,
                              .pin_d0 = Y2_GPIO_NUM,
                              .pin_vsync = VSYNC_GPIO_NUM,
                              .pin_href = HREF_GPIO_NUM,
                              .pin_pclk = PCLK_GPIO_NUM,

                              .xclk_freq_hz = 20000000,
                              .ledc_timer = LEDC_TIMER_0,
                              .ledc_channel = LEDC_CHANNEL_0,

                              .pixel_format = PIXFORMAT_JPEG,
                              .frame_size = CAMERA_FRAME_SIZE,
                              .jpeg_quality = CAMERA_JPEG_QUALITY,
                              .fb_count = 1,
                              .grab_mode = CAMERA_GRAB_LATEST};

    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    camera_status.is_initialized = true;
    ESP_LOGI(TAG, "Camera initialized successfully");

    // Get initial sensor settings for adjustment
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // Initial settings
        s->set_brightness(s, 0);                  // -2 to 2
        s->set_contrast(s, 0);                    // -2 to 2
        s->set_saturation(s, 0);                  // -2 to 2
        s->set_special_effect(s, 0);              // 0 to 6 (0 - No Effect)
        s->set_whitebal(s, 1);                    // 0 = disable , 1 = enable
        s->set_awb_gain(s, 1);                    // 0 = disable , 1 = enable
        s->set_wb_mode(s, 0);                     // 0 to 4 - if awb_gain enabled (0 - Auto)
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

    return ESP_OK;
}

// Deinitialize camera
void camera_deinit(void)
{
    if (!camera_status.is_initialized) {
        return;
    }

    // Stop capture if running
    camera_stop_capture();

    // Deinit camera
    esp_camera_deinit();

    // Delete mutex
    if (camera_mutex != NULL) {
        vSemaphoreDelete(camera_mutex);
        camera_mutex = NULL;
    }

    camera_status.is_initialized = false;
    ESP_LOGI(TAG, "Camera deinitialized");
}

// Capture single frame
esp_err_t camera_capture_frame(uint8_t **buffer, size_t *size)
{
    if (!camera_status.is_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(camera_mutex, portMAX_DELAY);

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        xSemaphoreGive(camera_mutex);
        return ESP_FAIL;
    }

    *buffer = (uint8_t *)malloc(fb->len);
    if (*buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for frame");
        esp_camera_fb_return(fb);
        xSemaphoreGive(camera_mutex);
        return ESP_ERR_NO_MEM;
    }

    memcpy(*buffer, fb->buf, fb->len);
    *size = fb->len;
    camera_status.last_frame_size = fb->len;

    esp_camera_fb_return(fb);
    xSemaphoreGive(camera_mutex);

    return ESP_OK;
}

// Capture and encode to JPEG
esp_err_t camera_capture_jpeg(uint8_t **buffer, size_t *size)
{
    // Since we configured camera for JPEG format, this is same as capture_frame
    return camera_capture_frame(buffer, size);
}

// Start continuous capture
esp_err_t camera_start_capture(uint32_t interval_ms, camera_capture_cb callback, void *user_data)
{
    if (!camera_status.is_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (capture_running) {
        ESP_LOGW(TAG, "Capture already running");
        return ESP_ERR_INVALID_STATE;
    }

    capture_interval = interval_ms;
    capture_callback = callback;
    capture_user_data = user_data;
    capture_running = true;
    camera_status.is_capturing = true;

    xTaskCreate(camera_capture_task, "camera_capture", 4096, NULL, CAMERA_TASK_PRIORITY,
                &capture_task_handle);

    ESP_LOGI(TAG, "Started continuous capture with interval %d ms", interval_ms);
    return ESP_OK;
}

// Stop continuous capture
esp_err_t camera_stop_capture(void)
{
    if (!capture_running) {
        return ESP_OK;
    }

    capture_running = false;
    camera_status.is_capturing = false;

    // Wait for task to finish
    if (capture_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        capture_task_handle = NULL;
    }

    ESP_LOGI(TAG, "Stopped continuous capture");
    return ESP_OK;
}

// Get camera status
camera_status_t camera_get_status(void)
{
    return camera_status;
}

// Adjust camera settings
esp_err_t camera_set_quality(int quality)
{
    if (!camera_status.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_quality(s, quality);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t camera_set_brightness(int brightness)
{
    if (!camera_status.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, brightness);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t camera_set_contrast(int contrast)
{
    if (!camera_status.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_contrast(s, contrast);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t camera_set_saturation(int saturation)
{
    if (!camera_status.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_saturation(s, saturation);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Take snapshot
esp_err_t camera_take_snapshot(uint8_t *buffer, size_t buffer_size, size_t *actual_size)
{
    if (!camera_status.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t *temp_buffer = NULL;
    size_t temp_size = 0;

    esp_err_t err = camera_capture_jpeg(&temp_buffer, &temp_size);
    if (err != ESP_OK) {
        return err;
    }

    if (temp_size > buffer_size) {
        camera_free_buffer(temp_buffer);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(buffer, temp_buffer, temp_size);
    *actual_size = temp_size;

    camera_free_buffer(temp_buffer);
    return ESP_OK;
}

// Free camera buffer
void camera_free_buffer(uint8_t *buffer)
{
    if (buffer) {
        free(buffer);
    }
}
