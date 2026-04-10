/**
 * @file ollama_backend.c
 * @brief Ollama backend implementation for the AI interface.
 */

#include "ollama_backend.h"
#include <stdlib.h>
#include <string.h>
#include "base64.h"
#include "cJSON.h"
#include "config.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "ollama_discovery.h"

#define MAX_RESPONSE_SIZE 4096

static const char *TAG = "ollama_backend";

// Static configuration and response buffer
static char s_api_url[512] = {0};  // Dynamic URL from service discovery
static const char *s_model = NULL;
static char *s_response_buffer = NULL;
static size_t s_response_len = 0;
static bool s_discovery_enabled = false;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (s_response_buffer && evt->data && evt->data_len > 0) {
                // Ensure we don't overflow the buffer (MAX_RESPONSE_SIZE total, -1 for null
                // terminator)
                size_t available_space = MAX_RESPONSE_SIZE - 1 - s_response_len;
                size_t bytes_to_copy =
                    (evt->data_len <= available_space) ? evt->data_len : available_space;

                if (bytes_to_copy > 0) {
                    memcpy(s_response_buffer + s_response_len, evt->data, bytes_to_copy);
                    s_response_len += bytes_to_copy;
                    s_response_buffer[s_response_len] = '\0';

                    if (bytes_to_copy < evt->data_len) {
                        ESP_LOGW(
                            TAG,
                            "HTTP response truncated: received %d bytes, only %zu bytes available",
                            evt->data_len, available_space);
                    }
                } else {
                    ESP_LOGW(TAG, "HTTP response buffer full, discarding %d bytes", evt->data_len);
                }
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
    return ESP_OK;
}

/**
 * @brief Test connectivity to Ollama server using /api/tags endpoint
 */
static esp_err_t test_ollama_connectivity(const char *base_url)
{
    if (!base_url || strlen(base_url) == 0) {
        ESP_LOGE(TAG, "Invalid base URL for connectivity test");
        return ESP_ERR_INVALID_ARG;
    }

    // Build tags URL from base URL
    char tags_url[512];
    const char *api_generate_suffix = "/api/generate";
    size_t base_len = strlen(base_url);
    size_t suffix_len = strlen(api_generate_suffix);

    if (base_len > suffix_len &&
        strcmp(base_url + base_len - suffix_len, api_generate_suffix) == 0) {
        // Remove /api/generate suffix and append /api/tags
        size_t base_only_len = base_len - suffix_len;
        snprintf(tags_url, sizeof(tags_url), "%.*s/api/tags", (int)base_only_len, base_url);
    } else {
        // Assume base_url is just the server, add /api/tags
        snprintf(tags_url, sizeof(tags_url), "%s/api/tags", base_url);
    }

    ESP_LOGI(TAG, "Testing connectivity to: %s", tags_url);

    esp_http_client_config_t config = {
        .url = tags_url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 10000,  // 10 second timeout
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client for connectivity test");
        return ESP_FAIL;
    }

    // Clear response buffer for this test
    s_response_len = 0;
    if (s_response_buffer)
        s_response_buffer[0] = '\0';

    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "Connectivity test: HTTP Status %d, Response length: %d", status_code,
             s_response_len);

    esp_http_client_cleanup(client);

    if (err == ESP_OK && status_code == 200) {
        ESP_LOGI(TAG, "Ollama server connectivity test PASSED");
        ESP_LOGD(TAG, "Server response: %.100s", s_response_buffer ? s_response_buffer : "empty");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Ollama server connectivity test FAILED");
        ESP_LOGE(TAG, "HTTP error: %s, status code: %d", esp_err_to_name(err), status_code);
        if (s_response_len > 0 && s_response_buffer) {
            ESP_LOGE(TAG, "Error response: %.200s", s_response_buffer);
        }
        return ESP_FAIL;
    }
}

static esp_err_t ollama_init(const ai_config_t *config)
{
    if (!config || !config->model) {
        ESP_LOGE(TAG, "Ollama model is required");
        return ESP_ERR_INVALID_ARG;
    }

    s_model = config->model;

#if OLLAMA_USE_SERVICE_DISCOVERY
    // Initialize service discovery
    ollama_discovery_config_t discovery_config = {.srv_record = OLLAMA_SRV_RECORD,
                                                  .fallback_url = OLLAMA_FALLBACK_URL,
                                                  .timeout_ms = OLLAMA_DISCOVERY_TIMEOUT_MS,
                                                  .use_mdns = OLLAMA_USE_MDNS};

    ESP_LOGI(TAG, "Initializing service discovery for SRV record: %s", OLLAMA_SRV_RECORD);
    esp_err_t discovery_err = ollama_discovery_init(&discovery_config);
    if (discovery_err == ESP_OK) {
        ESP_LOGI(TAG, "Service discovery initialized successfully");

        // Try to discover services
        ollama_service_info_t services[3];
        size_t num_found = 0;

        ESP_LOGI(TAG, "Searching for Ollama services...");
        esp_err_t find_err = ollama_discovery_find_services(services, 3, &num_found);
        if (find_err == ESP_OK && num_found > 0) {
            ESP_LOGI(TAG, "Found %d Ollama service(s)", num_found);

            // Log all discovered services
            for (size_t i = 0; i < num_found; i++) {
                ESP_LOGI(TAG, "Service %d: %s:%d (%s) priority=%d weight=%d", i,
                         services[i].hostname, services[i].port, services[i].ip_addr,
                         services[i].priority, services[i].weight);
            }

            // Get the best available service
            ollama_service_info_t best_service;
            esp_err_t best_err =
                ollama_discovery_get_best_service(services, num_found, &best_service);
            if (best_err == ESP_OK) {
                ESP_LOGI(TAG, "Selected best service: %s:%d", best_service.hostname,
                         best_service.port);

                // Build the API URL
                esp_err_t url_err = ollama_discovery_build_url(&best_service, "/api/generate",
                                                               s_api_url, sizeof(s_api_url));
                if (url_err == ESP_OK) {
                    s_discovery_enabled = true;
                    ESP_LOGI(TAG, "Using discovered Ollama service: %s", s_api_url);
                } else {
                    ESP_LOGE(TAG, "Failed to build URL from discovered service: %s",
                             esp_err_to_name(url_err));
                }
            } else {
                ESP_LOGE(TAG, "Failed to select best service: %s", esp_err_to_name(best_err));
            }
        } else {
            ESP_LOGW(TAG, "No Ollama services discovered: %s", esp_err_to_name(find_err));
            ESP_LOGW(TAG, "Service discovery troubleshooting:");
            ESP_LOGW(TAG, "  1. Check if Ollama mDNS service is published");
            ESP_LOGW(TAG, "  2. Verify both devices are on the same network");
            ESP_LOGW(TAG, "  3. Test from your computer: dns-sd -B _ollama._tcp");
            ESP_LOGW(TAG, "  4. Publish service: dns-sd -R \"Ollama AI\" _ollama._tcp . 11434");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize service discovery: %s", esp_err_to_name(discovery_err));
        ESP_LOGE(TAG, "mDNS initialization failed - check network configuration");
    }

    // Fallback to configured URL if discovery failed
    if (!s_discovery_enabled) {
        if (config->api_url) {
            strncpy(s_api_url, config->api_url, sizeof(s_api_url) - 1);
            s_api_url[sizeof(s_api_url) - 1] = '\0';
            ESP_LOGI(TAG, "Using configured API URL: %s", s_api_url);
        } else {
            ESP_LOGE(TAG, "No API URL available - neither discovery nor config provided a URL");
            return ESP_ERR_INVALID_ARG;
        }
    }
#else
    // Service discovery disabled, use configured URL
    if (!config->api_url) {
        ESP_LOGE(TAG, "Ollama API URL is required when service discovery is disabled");
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(s_api_url, config->api_url, sizeof(s_api_url) - 1);
    s_api_url[sizeof(s_api_url) - 1] = '\0';
    ESP_LOGI(TAG, "Using configured API URL: %s", s_api_url);
#endif

    s_response_buffer = malloc(MAX_RESPONSE_SIZE);
    if (!s_response_buffer) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        return ESP_ERR_NO_MEM;
    }

    // Test connectivity to Ollama server
    ESP_LOGI(TAG, "Testing connectivity to Ollama server...");
    esp_err_t connectivity_result = test_ollama_connectivity(s_api_url);
    if (connectivity_result != ESP_OK) {
        ESP_LOGE(TAG, "Ollama server connectivity test failed!");
        ESP_LOGE(TAG, "Troubleshooting steps:");
        ESP_LOGE(TAG, "  1. Verify Ollama is running: curl %s", s_api_url);
        ESP_LOGE(TAG, "  2. Check network connectivity from ESP32-CAM to server");
        ESP_LOGE(TAG, "  3. Verify firewall allows HTTP traffic on port 11434");
        ESP_LOGE(TAG, "  4. Test tags endpoint: curl %s/../tags", s_api_url);

        // Free resources before returning error
        if (s_response_buffer) {
            free(s_response_buffer);
            s_response_buffer = NULL;
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Ollama backend initialized successfully with model %s", s_model);
    ESP_LOGI(TAG, "Server connectivity verified - ready for image analysis");
    return ESP_OK;
}

static esp_err_t ollama_analyze_image(const uint8_t *image_data, size_t image_size,
                                      ai_response_t *response)
{
    if (!image_data || !response || strlen(s_api_url) == 0 || !s_model) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Analyzing image with Ollama (size: %zu bytes)", image_size);

    char *base64_image = base64_encode_alloc(image_data, image_size);
    if (!base64_image) {
        ESP_LOGE(TAG, "Failed to encode image to base64");
        return ESP_ERR_NO_MEM;
    }

    cJSON *json = cJSON_CreateObject();
    cJSON *images = cJSON_CreateArray();
    cJSON_AddItemToArray(images, cJSON_CreateString(base64_image));

    cJSON_AddItemToObject(json, "model", cJSON_CreateString(s_model));
    cJSON_AddItemToObject(
        json, "prompt",
        cJSON_CreateString(
            "You control a robot with a camera. Analyze this image and help the robot navigate its "
            "environment. "
            "Identify objects and their positions (left, center, right) in the scene. "
            "Determine if there are obstacles to avoid or clear paths. "
            "Respond with ONLY a simple JSON object with these keys:\n"
            "- objects: array of detected objects with name and position (e.g., "
            "[{\"name\":\"cup\", \"position\":\"left\"}])\n"
            "- recommendation: one of [forward, backward, left, right, rotate_cw, rotate_ccw, "
            "stop]\n"
            "- sound: optional sound command [beep, melody, alert, custom:freq:duration, "
            "morse:TEXT:message, rtttl:name]\n"
            "- pan: optional camera pan angle (0-180, default 90)\n"
            "- tilt: optional camera tilt angle (0-180, default 90)\n"
            "Respond ONLY with the JSON object, no explanations."));
    cJSON_AddItemToObject(json, "images", images);
    cJSON_AddItemToObject(json, "stream", cJSON_CreateFalse());
    cJSON_AddItemToObject(json, "format", cJSON_CreateString("json"));

    char *json_string = cJSON_PrintUnformatted(json);
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to create JSON request");
        free(base64_image);
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "JSON request size: %d bytes", strlen(json_string));

    esp_http_client_config_t config = {
        .url = s_api_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 45000,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        free(base64_image);
        free(json_string);
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");

    s_response_len = 0;
    if (s_response_buffer)
        s_response_buffer[0] = '\0';

    esp_http_client_set_post_field(client, json_string, strlen(json_string));

    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP Status: %d, Response length: %d", status_code, s_response_len);

    if (err == ESP_OK && status_code == 200) {
        cJSON *root = cJSON_Parse(s_response_buffer);
        if (root) {
            cJSON *response_json = cJSON_GetObjectItem(root, "response");
            if (cJSON_IsString(response_json)) {
                const char *response_str = cJSON_GetStringValue(response_json);
                size_t len = strlen(response_str);
                response->response_text = malloc(len + 1);
                if (response->response_text) {
                    strcpy(response->response_text, response_str);
                    response->response_length = len;
                    ESP_LOGI(TAG, "Successfully extracted Ollama response");
                    err = ESP_OK;
                } else {
                    ESP_LOGE(TAG, "Failed to allocate memory for response text");
                    err = ESP_ERR_NO_MEM;
                }
            } else {
                ESP_LOGE(TAG, "Failed to parse 'response' field from Ollama JSON");
                err = ESP_FAIL;
            }
            cJSON_Delete(root);
        } else {
            ESP_LOGE(TAG, "Failed to parse Ollama JSON response");
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s, status: %d", esp_err_to_name(err), status_code);
        if (s_response_len > 0) {
            ESP_LOGE(TAG, "Error response: %.*s", (int)s_response_len, s_response_buffer);
        }
        err = ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    free(base64_image);
    free(json_string);
    cJSON_Delete(json);

    return err;
}

static void ollama_free_response(ai_response_t *response)
{
    if (response && response->response_text) {
        free(response->response_text);
        response->response_text = NULL;
        response->response_length = 0;
    }
}

static void ollama_deinit(void)
{
    memset(s_api_url, 0, sizeof(s_api_url));
    s_model = NULL;
    s_discovery_enabled = false;

    if (s_response_buffer) {
        free(s_response_buffer);
        s_response_buffer = NULL;
    }

#if OLLAMA_USE_SERVICE_DISCOVERY
    ollama_discovery_deinit();
#endif

    ESP_LOGI(TAG, "Ollama backend deinitialized");
}

static const ai_backend_t s_ollama_backend = {
    .init = ollama_init,
    .analyze_image = ollama_analyze_image,
    .free_response = ollama_free_response,
    .deinit = ollama_deinit,
};

const ai_backend_t *ollama_backend_get(void)
{
    return &s_ollama_backend;
}
