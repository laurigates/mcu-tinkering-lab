/**
 * @file mic_pdm.c
 * @brief PDM RX driver implementation. See mic_pdm.h for the design.
 */

#include "mic_pdm.h"

#include "driver/i2s_pdm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "mic_pdm";

/** RX DMA sizing — 4 descriptors x 512 frames = 2048 samples = 128 ms at
 *  16 kHz, which is what MIC_PDM_BACKLOG_MS states.
 *
 *  Kept SHALLOW on purpose, the opposite of the TX side's reasoning. Depth on
 *  TX buys slack against a stalled writer; depth on RX buys nothing but
 *  latency, because every buffered descriptor is a frame that will be handed
 *  to a reader describing a moment further in the past. Two to three frames of
 *  slack is enough to survive the listener losing its core briefly; more only
 *  widens the window in which the robot can hear its own voice after the
 *  amplifier has gone quiet. */
#define MIC_DMA_DESC_NUM 4
#define MIC_DMA_FRAME_NUM 512

/** Scratch for mic_pdm_flush(). Static rather than stack: the flusher may be a
 *  4 kB task and this costs nothing to keep. */
#define MIC_FLUSH_CHUNK_SAMPLES 256

static i2s_chan_handle_t s_rx_chan = NULL;
static SemaphoreHandle_t s_lock = NULL;
static int16_t s_flush_scratch[MIC_FLUSH_CHUNK_SAMPLES];

esp_err_t mic_pdm_init(void)
{
    if (s_rx_chan != NULL) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "mutex allocation failed");
        return ESP_ERR_NO_MEM;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = MIC_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = MIC_DMA_FRAME_NUM;

    /* ================================================================== *
     * DO NOT MERGE THIS CALL WITH audio_player.c's TX ALLOCATION.
     * ================================================================== *
     *
     * This is a SEPARATE, RX-ONLY i2s_new_channel() on a controller that
     * audio_player.c has already acquired for the amplifier. It looks like an
     * obvious candidate for "simplification" into one call requesting both
     * handles. That change would silently break the microphone, and the
     * mechanism is entirely invisible from this file:
     *
     *   - i2s_common.c:998 sets `controller->full_duplex = true` IF AND ONLY IF
     *     a SINGLE i2s_new_channel() call asks for both a TX and an RX handle.
     *     Two separate one-direction calls leave it false.
     *   - i2s_pdm.c:389 then calls i2s_ll_share_bck_ws(dev, true) when
     *     full_duplex is set, and :397 computes
     *         is_slave = (role == I2S_ROLE_SLAVE) | full_duplex
     *     So a "full duplex" PDM RX channel is SLAVED to the transmitter's
     *     clocks — this 16 kHz microphone would be clocked by the MAX98357A's
     *     24 kHz BCLK/WS and return garbage.
     *   - Worse, it would die intermittently rather than outright: the player
     *     calls channel_set_active(false) to power the amp down after its
     *     300 ms idle grace, at which point the mic's borrowed clock stops
     *     altogether. The symptom would be a mic that works only while the
     *     robot is talking, which is precisely backwards.
     *
     * With full_duplex false, i2s_take_available_channel() simply grabs the
     * free RX slot on the already-acquired controller. ESP32-S3 is
     * SOC_I2S_HW_VERSION_2, where TX and RX have independent clock dividers, so
     * 24 kHz TX and 16 kHz RX coexist with no interaction.
     *
     * One controller-wide register IS shared and this init does write it:
     * i2s_pdm_rx_set_gpio() unconditionally calls i2s_ll_mclk_bind_to_rx_clk()
     * under SOC_I2S_HW_VERSION_2, setting rx_clkm_conf.mclk_sel for the whole
     * controller. That is harmless here only because audio_player's STD master
     * path never binds MCLK and its .mclk is I2S_GPIO_UNUSED, so nothing
     * consumes the MCLK output. If the amplifier ever needs a real MCLK, this
     * is the line that will fight it.
     * ================================================================== */
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel (rx) failed: %s", esp_err_to_name(err));
        s_rx_chan = NULL;
        return err;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE_HZ),
        // PDM is 16-bit only; the onboard mic is mono.
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .clk = MIC_PDM_CLK_PIN,
                // .din aliases dins[0] in the driver's union; the default mono
                // slot mask selects line 0 only, so dins[1..3] are never read.
                .din = MIC_PDM_DATA_PIN,
                .invert_flags = {false},
            },
    };
    /* The default clock config is the correct one: it sets
     * dn_sample_mode = I2S_PDM_DSR_8S and mclk_multiple = I2S_MCLK_MULTIPLE_256,
     * so the delivered rate really is MIC_SAMPLE_RATE_HZ. Do not hand-fill this
     * struct — a wrong downsampling ratio halves the rate with no error. */

    err = i2s_channel_init_pdm_rx_mode(s_rx_chan, &pdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_pdm_rx_mode failed: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return err;
    }

    err = i2s_channel_enable(s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable (rx) failed: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return err;
    }

    ESP_LOGI(TAG, "PDM mic ready: clk=%d data=%d %d Hz mono 16-bit, backlog<=%d ms",
             (int)MIC_PDM_CLK_PIN, (int)MIC_PDM_DATA_PIN, (int)MIC_SAMPLE_RATE_HZ,
             (int)MIC_PDM_BACKLOG_MS);
    return ESP_OK;
}

bool mic_pdm_is_ready(void)
{
    return s_rx_chan != NULL;
}

esp_err_t mic_pdm_lock(uint32_t timeout_ms)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void mic_pdm_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

esp_err_t mic_pdm_read(int16_t *dst, size_t max_samples, size_t *out_samples, uint32_t timeout_ms)
{
    if (out_samples != NULL) {
        *out_samples = 0;
    }
    if (dst == NULL || max_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_rx_chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t bytes_read = 0;
    const esp_err_t err =
        i2s_channel_read(s_rx_chan, dst, max_samples * sizeof(int16_t), &bytes_read, timeout_ms);
    if (out_samples != NULL) {
        *out_samples = bytes_read / sizeof(int16_t);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_read failed: %s (%u/%u samples)", esp_err_to_name(err),
                 (unsigned)(bytes_read / sizeof(int16_t)), (unsigned)max_samples);
    }
    return err;
}

void mic_pdm_flush(void)
{
    if (s_rx_chan == NULL) {
        return;
    }
    /* Drain with a zero timeout until the DMA has nothing queued. Bounded by
     * the descriptor count so a mic producing data faster than this loop can
     * consume it cannot spin here forever. */
    for (int i = 0; i < MIC_DMA_DESC_NUM * (MIC_DMA_FRAME_NUM / MIC_FLUSH_CHUNK_SAMPLES) + 1; i++) {
        size_t bytes_read = 0;
        if (i2s_channel_read(s_rx_chan, s_flush_scratch, sizeof(s_flush_scratch), &bytes_read, 0) !=
                ESP_OK ||
            bytes_read == 0) {
            return;
        }
    }
}
