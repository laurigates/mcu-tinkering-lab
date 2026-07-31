/**
 * @file audio_player.c
 * @brief I2S playback implementation. See audio_player.h for the design.
 */

#include "audio_player.h"

#include <string.h>

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "audio_player";

/** DMA sizing — 8 descriptors x 256 frames ~= 85 ms of slack at 24 kHz.
 *
 *  Doubled from 4 for margin against the ESP-IDF TX ISR's queue-overflow
 *  behaviour, which is what turns an underrun into audible NOISE rather than a
 *  clean gap. i2s_common.c creates msg_queue at `desc_num - 1` and, when it is
 *  full, the ISR DROPS THE OLDEST entry (xQueueIsQueueFullFromISR ->
 *  xQueueReceiveFromISR into a dummy) before pushing the newly finished
 *  descriptor. So after any writer stall longer than desc_num-1 descriptor
 *  periods, i2s_channel_write() is handed the OLDEST free descriptor — which is
 *  the very next one the DMA will transmit. The writer's lead collapses to
 *  roughly one descriptor period and its memcpy can land inside a buffer the
 *  DMA is actively reading, producing torn samples. More descriptors widen both
 *  the stall a writer can absorb and the lead it retains afterwards. */
#define I2S_DMA_DESC_NUM 8
#define I2S_DMA_FRAME_NUM 256

/** Mono samples converted per I2S write. Keep the stereo scratch off the
 *  stack budget of a 4 kB task: 128 frames -> 512 bytes. */
#define PLAYER_CHUNK_SAMPLES 128

/** Idle grace before the I2S channel is powered down. The MAX98357A emits a
 *  faint hiss whenever BCLK is running, so it is disabled between utterances
 *  rather than left clocking silence.
 *
 *  This is a poll interval, not a verdict: an empty ring means "power down"
 *  only once no fetch is in flight (s_fetch_active). A dry ring while the
 *  download is still running is a stall, and tearing the channel down there
 *  used to inject a further ~85 ms of preloaded silence on re-enable, in the
 *  middle of a word. Measured 4 such disables inside a single sentence. */
#define PLAYER_IDLE_TIMEOUT_MS 300

/** Safety net on the preroll wait. Notifications are counting and cleared on
 *  take, so none can be lost — this only bounds the damage from a future logic
 *  bug that forgets to notify. */
#define PLAYER_GATE_POLL_MS 500

/** Bound on a single DMA write. NOT portMAX_DELAY: i2s_channel_write() takes
 *  MILLISECONDS and feeds the value through pdMS_TO_TICKS() twice
 *  (i2s_common.c, for the binary semaphore and the descriptor queue), so
 *  portMAX_DELAY overflows TickType_t into ~72 minutes rather than meaning
 *  "block forever". A real bound makes a wedged DMA visible in the log. */
#define PLAYER_I2S_WRITE_TIMEOUT_MS 1000

static i2s_chan_handle_t s_tx_chan = NULL;
static RingbufHandle_t s_ring = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_channel_active = false;

/* Preroll gate state.
 *
 * The gate is a property of the RING, not of an utterance. That distinction is
 * load-bearing: an earlier version tracked "bytes banked for this utterance"
 * and "this utterance's fetch finished", which broke as soon as two utterances
 * were queued back to back (SPEECH_QUEUE_DEPTH is 2, and three producers post
 * into it). Utterance B's begin_utterance() would clear the completion flag
 * while A was still draining, and B would inherit A's already-open gate and
 * play its first bytes with no preroll at all — silently reintroducing exactly
 * the underrun this gate exists to prevent, and only in the back-to-back case,
 * so it would have looked intermittent.
 *
 * Occupancy is derived from two MONOTONIC counters with a single writer each,
 * so there is no shared read-modify-write to tear: the producer only ever adds
 * to s_written_total, the player only ever adds to s_played_total. Unsigned
 * wraparound keeps the difference correct. Both tasks are pinned to core 1
 * (AUDIO_PLAYER_TASK_CORE / TTS_FETCH_TASK_CORE) and a 32-bit aligned load is
 * atomic, so volatile is sufficient — no lock is needed or wanted on this path.
 */
static volatile size_t s_written_total = 0;  /**< producer (TTS fetch task) only */
static volatile size_t s_played_total = 0;   /**< player task only */
static volatile bool s_fetch_active = false; /**< producer only: a fetch is in flight */
static volatile bool s_armed = false;        /**< gate open: playback may start */

/** Bytes currently sitting in the ring. */
static inline size_t ring_pending(void)
{
    return s_written_total - s_played_total;
}

/* Odd trailing byte held back from the previous audio_player_write so only
 * 16-bit-aligned runs enter the ring. See audio_player_write. */
static uint8_t s_carry = 0;
static bool s_have_carry = false;

/* ------------------------------------------------------------------ */
/* I2S setup                                                           */
/* ------------------------------------------------------------------ */

static esp_err_t i2s_setup(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
    chan_cfg.auto_clear = true;  // emit zeros on underrun instead of stale DMA

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        // Stereo slots with a duplicated mono sample: the MAX98357A expects a
        // standard Philips frame and averages L+R when SD_MODE is floating.
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,  // MAX98357A derives its own clock
                .bclk = I2S_BCLK_PIN,
                .ws = I2S_LRCLK_PIN,
                .dout = I2S_DIN_PIN,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {false, false, false},
            },
    };

    err = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
    }
    return err;
}

/* ------------------------------------------------------------------ */
/* Player task                                                         */
/* ------------------------------------------------------------------ */

/** Scale to AUDIO_VOLUME_PCT and duplicate each mono sample into L and R. */
static void mono_to_stereo(const int16_t *mono, size_t count, int16_t *stereo)
{
    for (size_t i = 0; i < count; i++) {
        const int32_t scaled = ((int32_t)mono[i] * AUDIO_VOLUME_PCT) / 100;
        const int16_t s = (int16_t)scaled;  // in-range by construction (pct <= 100)
        stereo[2 * i] = s;
        stereo[2 * i + 1] = s;
    }
}

/** Fill the TX DMA buffers with silence while the channel is still in READY
 *  state. i2s_channel_enable() starts clocking the DMA buffers out immediately,
 *  before the first i2s_channel_write() can land — so without this the amp
 *  reproduces whatever those buffers happen to hold: uninitialised memory on
 *  the first utterance, and the stale tail of the previous clip on every one
 *  after (the channel is disabled mid-buffer at end of utterance). That is a
 *  burst of full-scale static in front of every clip. chan_cfg.auto_clear does
 *  not cover this: it zeroes on *underrun*, not at enable time.
 *  i2s_channel_preload_data() is only valid before enable, which is why this
 *  runs here rather than after. */
static void preload_silence(void)
{
    static const int16_t silence[PLAYER_CHUNK_SAMPLES * 2] = {0};
    size_t loaded = 0;
    do {
        loaded = 0;
        if (i2s_channel_preload_data(s_tx_chan, silence, sizeof(silence), &loaded) != ESP_OK) {
            return;
        }
        // A short load means the DMA buffers are full.
    } while (loaded == sizeof(silence));
}

static void channel_set_active(bool active)
{
    if (active == s_channel_active) {
        return;
    }
    if (active) {
        preload_silence();
    }
    const esp_err_t err = active ? i2s_channel_enable(s_tx_chan) : i2s_channel_disable(s_tx_chan);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_%s failed: %s", active ? "enable" : "disable",
                 esp_err_to_name(err));
        return;
    }
    s_channel_active = active;
}

static void player_task(void *arg)
{
    (void)arg;
    int16_t stereo[PLAYER_CHUNK_SAMPLES * 2];

    for (;;) {
        /* ---- Preroll gate ----
         * Draining a ring that the network cannot keep filled is what produces
         * the glitching: once the DMA runs dry the ISR's descriptor queue
         * overflows and the writer starts landing in a buffer the DMA is
         * reading (see I2S_DMA_DESC_NUM). Waiting until AUDIO_PREROLL_BYTES is
         * banked — or until the fetch has finished, whichever comes first —
         * means a short utterance is played from a complete buffer and cannot
         * underrun at all, while a long one still overlaps its download. */
        if (!s_armed) {
            if (s_channel_active) {
                channel_set_active(false);
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PLAYER_GATE_POLL_MS));
            continue;
        }

        size_t got = 0;
        void *item = xRingbufferReceiveUpTo(s_ring, &got, pdMS_TO_TICKS(PLAYER_IDLE_TIMEOUT_MS),
                                            PLAYER_CHUNK_SAMPLES * sizeof(int16_t));

        if (!item) {
            /* Ring dry. With no fetch in flight nothing more is coming, so the
             * utterance is genuinely over — close the gate and let the branch
             * above power the amp's clock down. While a fetch IS running this
             * is a mid-download stall instead, and the channel keeps clocking
             * so the rest of the sentence resumes without a teardown in the
             * middle of a word. */
            if (!s_fetch_active) {
                s_armed = false;
            }
            continue;
        }

        channel_set_active(true);

        const size_t samples = got / sizeof(int16_t);
        mono_to_stereo((const int16_t *)item, samples, stereo);
        vRingbufferReturnItem(s_ring, item);
        s_played_total += got;

        size_t written = 0;
        const esp_err_t err = i2s_channel_write(s_tx_chan, stereo, samples * 2 * sizeof(int16_t),
                                                &written, PLAYER_I2S_WRITE_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_write failed: %s (%u/%u bytes)", esp_err_to_name(err),
                     (unsigned)written, (unsigned)(samples * 2 * sizeof(int16_t)));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t audio_player_init(void)
{
    if (s_task) {
        return ESP_OK;
    }

    // The ring lives in PSRAM: 96 kB of internal RAM would meaningfully cut
    // into the WiFi/TLS and camera framebuffer budget.
    s_ring = xRingbufferCreateWithCaps(AUDIO_RING_BYTES, RINGBUF_TYPE_BYTEBUF,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring) {
        ESP_LOGE(TAG, "failed to allocate %d-byte PSRAM ring", AUDIO_RING_BYTES);
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t err = i2s_setup();
    if (err != ESP_OK) {
        vRingbufferDeleteWithCaps(s_ring);
        s_ring = NULL;
        return err;
    }

    const BaseType_t ok =
        xTaskCreatePinnedToCore(player_task, "audio_player", AUDIO_PLAYER_TASK_STACK_SIZE, NULL,
                                AUDIO_PLAYER_TASK_PRIORITY, &s_task, AUDIO_PLAYER_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create player task");
        // Mirror the i2s_setup() failure path: without this the 96 kB PSRAM
        // ring and the I2S channel stay allocated with nothing able to use or
        // release them, since s_task stays NULL and a retry re-allocates.
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        vRingbufferDeleteWithCaps(s_ring);
        s_ring = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ready — %d Hz mono, BCLK=%d LRCLK=%d DIN=%d, %d kB PSRAM ring",
             AUDIO_SAMPLE_RATE_HZ, I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DIN_PIN,
             AUDIO_RING_BYTES / 1024);
    return ESP_OK;
}

static esp_err_t ring_send(const uint8_t *p, size_t n, uint32_t timeout_ms)
{
    if (n == 0) {
        return ESP_OK;
    }
    // Blocking send is deliberate backpressure: the download must not outrun
    // real-time playback, or a long utterance would need unbounded PSRAM.
    if (xRingbufferSend(s_ring, p, n, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "ring full for %ums — dropping %u bytes", (unsigned)timeout_ms, (unsigned)n);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t audio_player_write(const uint8_t *pcm, size_t bytes, uint32_t timeout_ms)
{
    if (!s_ring) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!pcm || bytes == 0) {
        return ESP_OK;
    }

    /* Keep the byte ring 16-bit aligned. The base64 decoder emits 3-byte
     * groups, so `bytes` is frequently odd; sending odd-length runs into a
     * RINGBUF_TYPE_BYTEBUF lets a 16-bit sample straddle a read boundary, and
     * the player (got / sizeof(int16_t), then returns the whole item) drops the
     * straggler byte and shifts every following sample by one — loud static.
     * Because the shift is cumulative, the stream oscillates between aligned
     * (clean) and misaligned (static) as the drops accumulate. Holding the odd
     * trailing byte back until the next write keeps only even runs in the ring,
     * so no sample is ever split. Producer-side (TTS task) only — the player
     * merely reads — so s_carry needs no lock. */
    esp_err_t err;
    if (s_have_carry) {
        const uint8_t splice[2] = {s_carry, pcm[0]};  // complete the straddled sample
        if ((err = ring_send(splice, 2, timeout_ms)) != ESP_OK) {
            return err;
        }
        s_have_carry = false;
        pcm++;
        bytes--;
    }
    const size_t even = bytes & ~(size_t)1;
    if ((err = ring_send(pcm, even, timeout_ms)) != ESP_OK) {
        return err;
    }
    if (bytes & 1) {
        s_carry = pcm[even];
        s_have_carry = true;
    }

    /* Open the preroll gate on the way past the threshold — once only, so a
     * long utterance is not notifying the player on every write. */
    s_written_total += bytes;
    if (!s_armed && ring_pending() >= (size_t)AUDIO_PREROLL_BYTES) {
        s_armed = true;
        if (s_task) {
            xTaskNotifyGive(s_task);
        }
    }
    return ESP_OK;
}

void audio_player_begin_utterance(void)
{
    s_fetch_active = true;
    s_have_carry = false;

    /* Make this utterance earn its own preroll — but only if the ring is
     * actually empty. If the previous utterance is still draining, the gate
     * must stay open (closing it would cut that audio off mid-word) and this
     * utterance simply inherits the buffer already in front of it, which is
     * exactly as much protection as a fresh preroll would have bought. */
    if (ring_pending() == 0) {
        s_armed = false;
    }
}

void audio_player_end_utterance(void)
{
    // Drop a dangling half-sample (≤1 byte, inaudible) so it cannot prepend to,
    // and misalign, the next utterance.
    s_have_carry = false;
    s_fetch_active = false;

    /* Everything that is coming has arrived, so release the gate even if the
     * preroll threshold was never reached — a short utterance never reaches it,
     * and waiting for audio that will never arrive would simply never play.
     * Conditional on there being something to play: a failed or empty request
     * must NOT leave the gate open, or the next utterance skips its preroll. */
    if (ring_pending() > 0) {
        s_armed = true;
    }
    if (s_task) {
        xTaskNotifyGive(s_task); /* always: the player must re-evaluate */
    }
}

void audio_player_abort(void)
{
    if (!s_ring) {
        return;
    }

    s_have_carry = false;  // discard any half-sample so the next utterance starts aligned

    // Drain whatever is queued without playing it.
    for (;;) {
        size_t got = 0;
        void *item = xRingbufferReceiveUpTo(s_ring, &got, 0, AUDIO_RING_BYTES);
        if (!item) {
            break;
        }
        vRingbufferReturnItem(s_ring, item);
    }

    /* Close the gate and wake the player so it powers the channel down. The
     * player, not this task, disables I2S: it may be inside i2s_channel_write,
     * and disabling the channel underneath it is not safe. Draining the ring
     * from here IS safe — the player runs at a higher priority on the same
     * core, so it cannot be holding an outstanding item while we run, and for
     * the same reason this task may settle the played counter on its behalf. */
    s_played_total = s_written_total;
    s_fetch_active = false;
    s_armed = false;
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

bool audio_player_is_ready(void)
{
    return s_task != NULL && s_ring != NULL;
}

bool audio_player_is_active(void)
{
    /* No new flag on purpose — see audio_player.h. Every term here is already
     * maintained by begin_utterance / the player task / abort, so there is
     * nothing extra for those paths to forget. Over-inclusive by design: the
     * mic gate that consumes this must fail toward silence. */
    return s_fetch_active || s_armed || s_channel_active || ring_pending() > 0;
}
