#include "i2s_audio.h"

#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "i2s_audio";

#if defined(BOARD_SUPERMINI)
#define PIN_I2S_BCLK 4
#define PIN_I2S_LRCLK 5
#define PIN_I2S_DOUT 6
#else  // XIAO ESP32-S3
#define PIN_I2S_BCLK 6
#define PIN_I2S_LRCLK 43
#define PIN_I2S_DOUT 44
#endif

#define I2S_DMA_BUF_COUNT 4
#define I2S_DMA_BUF_LEN 1024
#define AUDIO_TASK_STACK 4096
#define STEREO_CHUNK_SAMPLES 512

typedef struct {
    int16_t *pcm_data;
    size_t pcm_len;
    uint32_t sample_rate;
} audio_msg_t;

static i2s_chan_handle_t tx_handle;
static QueueHandle_t audio_queue;
static SemaphoreHandle_t playback_done;

static void audio_play_buffer(const int16_t *mono, size_t mono_len, uint32_t sample_rate)
{
    size_t mono_samples = mono_len / sizeof(int16_t);
    int16_t stereo_chunk[STEREO_CHUNK_SAMPLES * 2];
    size_t offset = 0;

    while (offset < mono_samples) {
        size_t chunk = mono_samples - offset;
        if (chunk > STEREO_CHUNK_SAMPLES) {
            chunk = STEREO_CHUNK_SAMPLES;
        }

        /* Mono → stereo duplication */
        for (size_t i = 0; i < chunk; i++) {
            stereo_chunk[i * 2] = mono[offset + i];
            stereo_chunk[i * 2 + 1] = mono[offset + i];
        }

        size_t bytes_written = 0;
        size_t write_bytes = chunk * 2 * sizeof(int16_t);
        i2s_channel_write(tx_handle, stereo_chunk, write_bytes, &bytes_written,
                          pdMS_TO_TICKS(1000));

        offset += chunk;
    }
}

static void audio_task(void *arg)
{
    audio_msg_t msg;

    while (true) {
        if (xQueueReceive(audio_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Playing %u bytes @ %lu Hz", (unsigned)msg.pcm_len,
                     (unsigned long)msg.sample_rate);

            i2s_channel_enable(tx_handle);
            audio_play_buffer(msg.pcm_data, msg.pcm_len, msg.sample_rate);
            i2s_channel_disable(tx_handle);

            /* Free the PSRAM buffer */
            if (msg.pcm_data) {
                free(msg.pcm_data);
            }

            xSemaphoreGive(playback_done);
        }
    }
}

esp_err_t i2s_audio_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = I2S_DMA_BUF_LEN;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S channel create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(24000),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = PIN_I2S_BCLK,
                .ws = PIN_I2S_LRCLK,
                .dout = PIN_I2S_DOUT,
                .din = I2S_GPIO_UNUSED,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };

    ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S std mode init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S initialized (BCLK=%d, LRCLK=%d, DOUT=%d)", PIN_I2S_BCLK, PIN_I2S_LRCLK,
             PIN_I2S_DOUT);
    return ESP_OK;
}

esp_err_t i2s_audio_start_task(void)
{
    audio_queue = xQueueCreate(2, sizeof(audio_msg_t));
    if (!audio_queue) {
        return ESP_ERR_NO_MEM;
    }

    playback_done = xSemaphoreCreateBinary();
    if (!playback_done) {
        return ESP_ERR_NO_MEM;
    }

    /* Pin audio task to Core 1 for real-time I2S output */
    BaseType_t ret = xTaskCreatePinnedToCore(audio_task, "audio", AUDIO_TASK_STACK, NULL, 5, NULL,
                                             1);  // Core 1
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Audio task started on Core 1");
    return ESP_OK;
}

esp_err_t i2s_audio_play_mono(const int16_t *pcm_data, size_t pcm_len, uint32_t sample_rate)
{
    i2s_channel_enable(tx_handle);
    audio_play_buffer(pcm_data, pcm_len, sample_rate);
    i2s_channel_disable(tx_handle);
    return ESP_OK;
}

esp_err_t i2s_audio_queue_playback(int16_t *pcm_data, size_t pcm_len, uint32_t sample_rate)
{
    audio_msg_t msg = {
        .pcm_data = pcm_data,
        .pcm_len = pcm_len,
        .sample_rate = sample_rate,
    };

    /* Reset the done semaphore before queuing */
    xSemaphoreTake(playback_done, 0);

    if (xQueueSend(audio_queue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Audio queue full");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t i2s_audio_wait_done(uint32_t timeout_ms)
{
    if (xSemaphoreTake(playback_done, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}
