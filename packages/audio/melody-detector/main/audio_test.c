#include "audio_test.h"

#include <math.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "pin_config.h"

static const char *TAG = "audio";

#define I2S_DMA_BUF_COUNT 4
#define I2S_DMA_BUF_LEN 1024
#define WRITE_CHUNK_FRAMES 256  // stereo frames per i2s_channel_write

static i2s_chan_handle_t s_tx;

esp_err_t audio_test_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = I2S_DMA_BUF_LEN;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = I2S_BCLK_PIN,
                .ws = I2S_LRC_PIN,
                .dout = I2S_DIN_PIN,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {0},
            },
    };
    err = i2s_channel_init_std_mode(s_tx, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2S ready (BCLK=%d LRC=%d DIN=%d @ %d Hz)", I2S_BCLK_PIN, I2S_LRC_PIN,
             I2S_DIN_PIN, AUDIO_SAMPLE_RATE_HZ);
    return ESP_OK;
}

esp_err_t audio_test_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (!s_tx) {
        return ESP_ERR_INVALID_STATE;
    }

    const float two_pi = 6.2831853f;
    const float phase_inc = two_pi * (float)freq_hz / (float)AUDIO_SAMPLE_RATE_HZ;
    // ~30% of full scale — comfortable on a small speaker without clipping
    const int16_t amplitude = 10000;

    float phase = 0.0f;
    int16_t buf[WRITE_CHUNK_FRAMES * 2];

    const uint32_t total_frames = (uint32_t)((uint64_t)duration_ms * AUDIO_SAMPLE_RATE_HZ / 1000);
    uint32_t written_frames = 0;

    esp_err_t err = i2s_channel_enable(s_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        return err;
    }

    while (written_frames < total_frames) {
        uint32_t chunk = total_frames - written_frames;
        if (chunk > WRITE_CHUNK_FRAMES) {
            chunk = WRITE_CHUNK_FRAMES;
        }
        // Apply a 5ms attack and release ramp to avoid speaker clicks
        const uint32_t ramp_frames = AUDIO_SAMPLE_RATE_HZ / 200;  // 5 ms
        for (uint32_t i = 0; i < chunk; i++) {
            float gain = 1.0f;
            uint32_t f = written_frames + i;
            if (f < ramp_frames) {
                gain = (float)f / (float)ramp_frames;
            } else if (f + ramp_frames > total_frames) {
                uint32_t remaining = total_frames - f;
                gain = (float)remaining / (float)ramp_frames;
            }
            int16_t s = (int16_t)(sinf(phase) * amplitude * gain);
            buf[i * 2] = s;
            buf[i * 2 + 1] = s;
            phase += phase_inc;
            if (phase >= two_pi) {
                phase -= two_pi;
            }
        }
        size_t bytes_written = 0;
        err = i2s_channel_write(s_tx, buf, chunk * 2 * sizeof(int16_t), &bytes_written,
                                pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_write: %s", esp_err_to_name(err));
            i2s_channel_disable(s_tx);
            return err;
        }
        written_frames += chunk;
    }

    return i2s_channel_disable(s_tx);
}
