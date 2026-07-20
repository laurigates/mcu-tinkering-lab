/**
 * @file speech_queue.c
 * @brief Utterance queue implementation. See speech_queue.h for the rationale.
 */

#include "speech_queue.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "speech_queue";

static QueueHandle_t s_queue = NULL;

esp_err_t speech_queue_init(void)
{
    if (s_queue) {
        return ESP_OK;
    }

    s_queue = xQueueCreate(SPEECH_QUEUE_DEPTH, sizeof(speech_request_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "failed to create queue (%d x %u bytes)", SPEECH_QUEUE_DEPTH,
                 (unsigned)sizeof(speech_request_t));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t speech_queue_post(const char *text)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!text || text[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    speech_request_t req;
    strlcpy(req.text, text, sizeof(req.text));

    // Zero timeout: posting happens from the planner task, which must never
    // stall on a busy speaker.
    if (xQueueSend(s_queue, &req, 0) != pdTRUE) {
        ESP_LOGD(TAG, "queue full — dropped: \"%s\"", req.text);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t speech_queue_receive(speech_request_t *out, uint32_t timeout_ms)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    const TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    return (xQueueReceive(s_queue, out, ticks) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void speech_queue_flush(void)
{
    if (s_queue) {
        xQueueReset(s_queue);
    }
}
