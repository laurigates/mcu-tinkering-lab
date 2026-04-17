/**
 * @file main.c
 * @brief esp32s3-gemini-vision: XIAO ESP32-S3 Sense + Gemini Robotics-ER object detection.
 *
 * Flow:
 *   app_main
 *     -> NVS + WiFi STA
 *     -> mDNS (xiao-vision.local)
 *     -> OV2640 camera init (XIAO Sense pins)
 *     -> Gemini worker task (consumes trigger queue)
 *     -> Auto-detect timer (configurable interval)
 *     -> HTTP server on :80 with / /stream /snapshot /detect /metadata /config
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "credentials.h"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "gemini_client.h"
#include "mdns.h"
#include "nvs_flash.h"

static const char *TAG = "gemini-vision";

// ---------------------------------------------------------------- Camera ----
// XIAO ESP32-S3 Sense OV2640 internal pin map
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 10
#define CAM_PIN_SIOD 40
#define CAM_PIN_SIOC 39
#define CAM_PIN_D7 48
#define CAM_PIN_D6 11
#define CAM_PIN_D5 12
#define CAM_PIN_D4 14
#define CAM_PIN_D3 16
#define CAM_PIN_D2 18
#define CAM_PIN_D1 17
#define CAM_PIN_D0 15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF 47
#define CAM_PIN_PCLK 13

static esp_err_t camera_init(void)
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
        .frame_size = FRAMESIZE_VGA,  // 640x480 — balances bandwidth vs Gemini input
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        // LATEST drops stale frames under backpressure — for live streaming this
        // keeps the feed fresh and lets the camera run at its natural rate. With
        // WHEN_EMPTY the camera stalls for the slowest consumer.
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
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
    ESP_LOGI(TAG, "Camera initialized (VGA JPEG)");
    return ESP_OK;
}

// ------------------------------------------------------------------ WiFi ----
// Canonical STA setup — see .claude/skills/wifi-sta-setup/SKILL.md
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAX_RETRY 5

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "WiFi disconnected. Reason: %d (%s)", disconnected->reason,
                 disconnected->reason == WIFI_REASON_NO_AP_FOUND         ? "AP not found"
                 : disconnected->reason == WIFI_REASON_AUTH_FAIL         ? "Auth failed"
                 : disconnected->reason == WIFI_REASON_ASSOC_FAIL        ? "Assoc failed"
                 : disconnected->reason == WIFI_REASON_HANDSHAKE_TIMEOUT ? "Handshake timeout"
                                                                         : "Other");
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry %d/%d to connect to the AP", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to AP after %d retries", WIFI_MAX_RETRY);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_sta_start(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    // Regulatory: allow channels 1–13 (default clips to 1–11 and hides APs on 12/13)
    wifi_country_t country = {
        .cc = "FI", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_AUTO};
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_cfg = {
        .sta =
            {
                // Mixed-mode tolerant: accepts APs that advertise WPA or WPA2
                .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
                // PMF capable-but-not-required fixes 4-way handshake timeouts
                // against APs that advertise PMF (common on modern WiFi 6).
                .pmf_cfg = {.capable = true, .required = false},
                .scan_method = WIFI_FAST_SCAN,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            },
    };
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

// ------------------------------------------------------ Detection store ----
#define DETECTION_BUF_SIZE 4096

static SemaphoreHandle_t s_detection_mutex;
static char s_detection_json[DETECTION_BUF_SIZE] = "[]";
static uint32_t s_detection_latency_ms = 0;
static int64_t s_detection_timestamp_us = 0;

static void detection_store_update(const char *json, uint32_t latency_ms)
{
    if (xSemaphoreTake(s_detection_mutex, portMAX_DELAY) == pdTRUE) {
        strlcpy(s_detection_json, json, sizeof(s_detection_json));
        s_detection_latency_ms = latency_ms;
        s_detection_timestamp_us = esp_timer_get_time();
        xSemaphoreGive(s_detection_mutex);
    }
}

// ---------------------------------------------------- Gemini worker task ----
static QueueHandle_t s_trigger_queue;  // messages: uint8_t (ignored; presence is the signal)
static volatile bool s_inference_busy = false;
static volatile uint32_t s_auto_interval_s = 0;  // 0 = disabled

static void gemini_worker_task(void *arg)
{
    (void)arg;
    char *resp = malloc(DETECTION_BUF_SIZE);
    if (!resp) {
        ESP_LOGE(TAG, "worker alloc failed");
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        uint8_t trigger;
        if (xQueueReceive(s_trigger_queue, &trigger, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        // Drain any additional queued triggers to coalesce bursts
        while (xQueueReceive(s_trigger_queue, &trigger, 0) == pdTRUE) {
        }

        s_inference_busy = true;
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "frame capture failed");
            detection_store_update("{\"error\":\"frame capture failed\"}", 0);
            s_inference_busy = false;
            continue;
        }
        ESP_LOGI(TAG, "Detecting on JPEG %zu bytes", fb->len);
        uint32_t latency = 0;
        esp_err_t err = gemini_detect(fb->buf, fb->len, resp, DETECTION_BUF_SIZE, &latency);
        esp_camera_fb_return(fb);

        if (err == ESP_OK) {
            detection_store_update(resp, latency);
        } else {
            ESP_LOGW(TAG, "detection failed: %s", esp_err_to_name(err));
            detection_store_update(resp, latency);  // resp holds an {"error":...} object
        }
        s_inference_busy = false;
    }
}

static void enqueue_trigger(void)
{
    uint8_t msg = 1;
    // Non-blocking; if queue is full the worker will coalesce anyway.
    xQueueSend(s_trigger_queue, &msg, 0);
}

static void auto_timer_cb(TimerHandle_t t)
{
    (void)t;
    static uint32_t elapsed = 0;
    if (s_auto_interval_s == 0) {
        elapsed = 0;
        return;
    }
    elapsed++;
    if (elapsed >= s_auto_interval_s) {
        elapsed = 0;
        if (!s_inference_busy) {
            enqueue_trigger();
        }
    }
}

// --------------------------------------------------------- HTTP handlers ----
// cppcheck-suppress syntaxError // GCC asm() rename extension (embedded html blob)
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
// cppcheck-suppress syntaxError
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
}

static const char *STREAM_CT = "multipart/x-mixed-replace;boundary=frame";

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, STREAM_CT);
    char part[96];
    uint32_t frames = 0;
    size_t bytes = 0;
    int64_t last_log_us = esp_timer_get_time();
    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            return ESP_FAIL;
        }
        size_t hlen =
            snprintf(part, sizeof(part),
                     "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     (unsigned)fb->len);
        if (httpd_resp_send_chunk(req, part, hlen) != ESP_OK ||
            httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
            esp_camera_fb_return(fb);
            return ESP_FAIL;
        }
        frames++;
        bytes += fb->len;
        esp_camera_fb_return(fb);

        int64_t now = esp_timer_get_time();
        if (now - last_log_us >= 2000000) {
            float secs = (now - last_log_us) / 1000000.0f;
            ESP_LOGI(TAG, "stream: %.1f fps, avg %u bytes/frame, %.1f kB/s", frames / secs,
                     (unsigned)(bytes / frames), (bytes / 1024.0f) / secs);
            frames = 0;
            bytes = 0;
            last_log_us = now;
        }
    }
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return r;
}

static esp_err_t detect_handler(httpd_req_t *req)
{
    enqueue_trigger();
    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"queued\":true}", 15);
}

static esp_err_t metadata_handler(httpd_req_t *req)
{
    char *out = malloc(DETECTION_BUF_SIZE + 256);
    if (!out) {
        return httpd_resp_send_500(req);
    }
    if (xSemaphoreTake(s_detection_mutex, portMAX_DELAY) == pdTRUE) {
        int n = snprintf(out, DETECTION_BUF_SIZE + 256,
                         "{\"timestamp_ms\":%lld,\"latency_ms\":%u,\"busy\":%s,\"auto_interval_s\":"
                         "%u,\"objects\":%s}",
                         (long long)(s_detection_timestamp_us / 1000),
                         (unsigned)s_detection_latency_ms, s_inference_busy ? "true" : "false",
                         (unsigned)s_auto_interval_s, s_detection_json);
        xSemaphoreGive(s_detection_mutex);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, out, n);
    } else {
        httpd_resp_send_500(req);
    }
    free(out);
    return ESP_OK;
}

static esp_err_t config_handler(httpd_req_t *req)
{
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "interval", val, sizeof(val)) == ESP_OK) {
            int v = atoi(val);
            if (v < 0)
                v = 0;
            if (v > 300)
                v = 300;
            s_auto_interval_s = v;
            ESP_LOGI(TAG, "auto_interval_s = %d", v);
        }
    }
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "{\"auto_interval_s\":%u}", (unsigned)s_auto_interval_s);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

static void http_start(void)
{
    // Primary server on :80 — short-lived handlers (UI, JSON API, one-shot snapshot)
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 10;
    httpd_handle_t api = NULL;
    ESP_ERROR_CHECK(httpd_start(&api, &cfg));
    httpd_uri_t api_routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_handler},
        {.uri = "/snapshot", .method = HTTP_GET, .handler = snapshot_handler},
        {.uri = "/detect", .method = HTTP_POST, .handler = detect_handler},
        {.uri = "/detect", .method = HTTP_GET, .handler = detect_handler},
        {.uri = "/metadata", .method = HTTP_GET, .handler = metadata_handler},
        {.uri = "/config", .method = HTTP_GET, .handler = config_handler},
    };
    for (size_t i = 0; i < sizeof(api_routes) / sizeof(api_routes[0]); i++) {
        httpd_register_uri_handler(api, &api_routes[i]);
    }

    // Secondary server on :81 — MJPEG stream only (its handler blocks forever)
    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port = 32769;
    scfg.stack_size = 8192;
    httpd_handle_t stream = NULL;
    ESP_ERROR_CHECK(httpd_start(&stream, &scfg));
    httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
    httpd_register_uri_handler(stream, &stream_uri);

    ESP_LOGI(TAG, "HTTP: api on :80, stream on :81");
}

// ---------------------------------------------------------------- main ----
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "esp32s3-gemini-vision starting");

    ESP_ERROR_CHECK(camera_init());
    // Let the camera's power rail settle before WiFi init (USB power can dip otherwise)
    vTaskDelay(pdMS_TO_TICKS(500));

    if (wifi_sta_start() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi failed to connect — check credentials.h");
        return;
    }

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("xiao-vision"));
    ESP_ERROR_CHECK(mdns_instance_name_set("XIAO Gemini Vision"));
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS: xiao-vision.local");

    s_detection_mutex = xSemaphoreCreateMutex();
    s_trigger_queue = xQueueCreate(4, sizeof(uint8_t));
    xTaskCreatePinnedToCore(gemini_worker_task, "gemini", 8192, NULL, 5, NULL, 1);

    TimerHandle_t auto_timer =
        xTimerCreate("auto", pdMS_TO_TICKS(1000), pdTRUE, NULL, auto_timer_cb);
    xTimerStart(auto_timer, 0);

    http_start();

    ESP_LOGI(TAG, "Ready — open http://xiao-vision.local/");
}
