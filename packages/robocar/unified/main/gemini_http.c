/**
 * @file gemini_http.c
 * @brief Shared Gemini HTTPS POST helper. See gemini_http.h.
 */

#include "gemini_http.h"

#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_log.h"

static const char *TAG = "gemini_http";

esp_err_t gemini_http_post(const char *url, const char *api_key, const char *body, int timeout_ms,
                           http_event_handle_cb handler, void *user_ctx, int *status_out)
{
    if (status_out) {
        *status_out = 0;
    }
    if (!url || !api_key || !body) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = timeout_ms,
        .event_handler = handler,
        .user_data = user_ctx,
        /* TLS via the bundled roots. Note that this build leaves
         * CONFIG_MBEDTLS_HAVE_TIME_DATE unset, so certificate dates are not
         * validated and no SNTP client is needed — see
         * .claude/rules/freertos-task-gotchas.md before "fixing" a TLS failure
         * by adding a clock. */
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "esp_http_client_init() failed");
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "x-goog-api-key", api_key);
    esp_http_client_set_post_field(client, body, strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    if (status_out) {
        *status_out = status;
    }

    if (err == ESP_OK && status != 200) {
        err = ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    return err;
}
