/**
 * @file audio_engine.c
 * @brief I2S RX (INMP441) + TX (MAX98357A) driver implementation.
 *
 * Hardware-unverified — the configuration is modelled after the
 * nfc-scavenger-hunt I2S playback path (MAX98357A mono-to-stereo duplication)
 * and a standard I2S PDM/Philips INMP441 capture channel.
 */

#include "audio_engine.h"

#include <string.h>

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "thinkpack_audio.h"

static const char *TAG = "audio_engine";

/* ------------------------------------------------------------------ */
/* Pin map — ESP32-S3 SuperMini. See WIRING.md.                        */
/* ------------------------------------------------------------------ */

#define PIN_TX_BCLK 4
#define PIN_TX_LRCLK 5
#define PIN_TX_DOUT 6

#define PIN_RX_BCLK 10
#define PIN_RX_LRCLK 11
#define PIN_RX_DIN 12

/* ------------------------------------------------------------------ */
/* I2S DMA parameters                                                  */
/* ------------------------------------------------------------------ */

#define I2S_DMA_DESC_NUM 4
#define I2S_DMA_FRAME_NUM 512
#define TX_CHUNK_SAMPLES 256
#define RX_CHUNK_SAMPLES 256

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static bool s_initialised;
static bool s_recording;

static i2s_chan_handle_t s_rx_chan;
static i2s_chan_handle_t s_tx_chan;

static int16_t *s_clip_buf; /* PSRAM */
static size_t s_clip_samples;

static int16_t *s_shifted_buf; /* PSRAM scratch for pitch-shift playback */

/* ------------------------------------------------------------------ */
/* I2S channel configuration helpers                                   */
/* ------------------------------------------------------------------ */

static esp_err_t configure_tx_channel(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (err != ESP_OK) {
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(THINKPACK_AUDIO_SAMPLE_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = PIN_TX_BCLK,
                .ws = PIN_TX_LRCLK,
                .dout = PIN_TX_DOUT,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {0},
            },
    };
    return i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
}

static esp_err_t configure_rx_channel(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_chan);
    if (err != ESP_OK) {
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(THINKPACK_AUDIO_SAMPLE_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = PIN_RX_BCLK,
                .ws = PIN_RX_LRCLK,
                .dout = I2S_GPIO_UNUSED,
                .din = PIN_RX_DIN,
                .invert_flags = {0},
            },
    };
    return i2s_channel_init_std_mode(s_rx_chan, &std_cfg);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t audio_engine_init(void)
{
    if (s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = configure_tx_channel();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX channel init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = configure_rx_channel();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RX channel init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_clip_buf = heap_caps_malloc(THINKPACK_AUDIO_CLIP_MAX_SAMPLES * sizeof(int16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_shifted_buf = heap_caps_malloc(THINKPACK_AUDIO_CLIP_MAX_SAMPLES * sizeof(int16_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_clip_buf == NULL || s_shifted_buf == NULL) {
        ESP_LOGE(TAG, "PSRAM clip alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_clip_samples = 0;
    s_initialised = true;

    ESP_LOGI(TAG, "audio_engine ready: %d samples of PSRAM reserved",
             (int)THINKPACK_AUDIO_CLIP_MAX_SAMPLES);
    return ESP_OK;
}

esp_err_t audio_engine_record_start(void)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_recording) {
        return ESP_ERR_INVALID_STATE;
    }
    s_clip_samples = 0;
    esp_err_t err = i2s_channel_enable(s_rx_chan);
    if (err != ESP_OK) {
        return err;
    }
    s_recording = true;
    return ESP_OK;
}

esp_err_t audio_engine_record_tick(bool *reached_limit)
{
    if (!s_recording) {
        return ESP_ERR_INVALID_STATE;
    }
    if (reached_limit != NULL) {
        *reached_limit = false;
    }

    size_t remaining = THINKPACK_AUDIO_CLIP_MAX_SAMPLES - s_clip_samples;
    if (remaining == 0) {
        if (reached_limit != NULL) {
            *reached_limit = true;
        }
        return ESP_OK;
    }

    size_t chunk = remaining < RX_CHUNK_SAMPLES ? remaining : RX_CHUNK_SAMPLES;
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(s_rx_chan, &s_clip_buf[s_clip_samples],
                                     chunk * sizeof(int16_t), &bytes_read, pdMS_TO_TICKS(10));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        return err;
    }

    s_clip_samples += bytes_read / sizeof(int16_t);
    if (s_clip_samples >= THINKPACK_AUDIO_CLIP_MAX_SAMPLES && reached_limit != NULL) {
        *reached_limit = true;
    }
    return ESP_OK;
}

esp_err_t audio_engine_record_stop(size_t *out_samples)
{
    if (!s_recording) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = i2s_channel_disable(s_rx_chan);
    s_recording = false;
    if (out_samples != NULL) {
        *out_samples = s_clip_samples;
    }
    /* Apply toddler-safe volume cap once, at the end of capture. */
    thinkpack_audio_apply_volume_cap(s_clip_buf, s_clip_samples);
    return err;
}

/** Blocking mono-to-stereo write helper for MAX98357A. */
static esp_err_t play_mono(const int16_t *mono, size_t mono_samples)
{
    if (mono == NULL || mono_samples == 0) {
        return ESP_OK;
    }

    int16_t stereo[TX_CHUNK_SAMPLES * 2];
    esp_err_t err = i2s_channel_enable(s_tx_chan);
    if (err != ESP_OK) {
        return err;
    }

    size_t offset = 0;
    while (offset < mono_samples) {
        size_t chunk = mono_samples - offset;
        if (chunk > TX_CHUNK_SAMPLES) {
            chunk = TX_CHUNK_SAMPLES;
        }
        for (size_t i = 0; i < chunk; i++) {
            stereo[i * 2] = mono[offset + i];
            stereo[i * 2 + 1] = mono[offset + i];
        }
        size_t written = 0;
        err = i2s_channel_write(s_tx_chan, stereo, chunk * 2 * sizeof(int16_t), &written,
                                pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            break;
        }
        offset += chunk;
    }

    i2s_channel_disable(s_tx_chan);
    return err;
}

esp_err_t audio_engine_playback(void)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    return play_mono(s_clip_buf, s_clip_samples);
}

esp_err_t audio_engine_playback_with_pitch_shift(int8_t semitone_shift)
{
    if (!s_initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (semitone_shift == 0) {
        return audio_engine_playback();
    }
    size_t produced = thinkpack_audio_pitch_shift(s_clip_buf, s_clip_samples, semitone_shift,
                                                  s_shifted_buf, THINKPACK_AUDIO_CLIP_MAX_SAMPLES);
    return play_mono(s_shifted_buf, produced);
}

const int16_t *audio_engine_clip_samples(void)
{
    return s_clip_buf;
}

size_t audio_engine_clip_sample_count(void)
{
    return s_clip_samples;
}
