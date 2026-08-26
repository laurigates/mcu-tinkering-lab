/**
 * @file driver/i2s_std.h — host-test shim.
 *
 * The I2S peripheral has no host analogue and nothing here is under test: the
 * accounting tests never start the player task, so no channel is ever enabled
 * or written. Every entry point returns ESP_OK and records nothing. The types
 * exist only so audio_player.c's i2s_setup() compiles unmodified — which is the
 * point of shimming rather than #ifdef-ing the shipped file: the code the test
 * links is the code the device runs.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_I2S_STD_H
#define ROBOCAR_UNIFIED_HOST_TEST_I2S_STD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef void *i2s_chan_handle_t;

typedef enum { I2S_NUM_0 = 0, I2S_NUM_1 = 1 } i2s_port_t;
typedef enum { I2S_ROLE_MASTER = 0, I2S_ROLE_SLAVE } i2s_role_t;
typedef enum { I2S_DATA_BIT_WIDTH_8BIT = 8, I2S_DATA_BIT_WIDTH_16BIT = 16 } i2s_data_bit_width_t;
typedef enum { I2S_SLOT_MODE_MONO = 0, I2S_SLOT_MODE_STEREO } i2s_slot_mode_t;

#define I2S_GPIO_UNUSED (-1)

typedef struct {
    i2s_port_t id;
    i2s_role_t role;
    uint32_t dma_desc_num;
    uint32_t dma_frame_num;
    bool auto_clear;
} i2s_chan_config_t;

typedef struct {
    uint32_t sample_rate_hz;
} i2s_std_clk_config_t;

typedef struct {
    i2s_data_bit_width_t data_bit_width;
    i2s_slot_mode_t slot_mode;
} i2s_std_slot_config_t;

typedef struct {
    bool mclk_inv;
    bool bclk_inv;
    bool ws_inv;
} i2s_std_gpio_invert_t;

typedef struct {
    int mclk;
    int bclk;
    int ws;
    int dout;
    int din;
    i2s_std_gpio_invert_t invert_flags;
} i2s_std_gpio_config_t;

typedef struct {
    i2s_std_clk_config_t clk_cfg;
    i2s_std_slot_config_t slot_cfg;
    i2s_std_gpio_config_t gpio_cfg;
} i2s_std_config_t;

#define I2S_CHANNEL_DEFAULT_CONFIG(port, task_role) \
    ((i2s_chan_config_t){                           \
        .id = (port),                               \
        .role = (task_role),                        \
        .dma_desc_num = 6,                          \
        .dma_frame_num = 240,                       \
        .auto_clear = false,                        \
    })

#define I2S_STD_CLK_DEFAULT_CONFIG(rate) ((i2s_std_clk_config_t){.sample_rate_hz = (rate)})

#define I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits, mode) \
    ((i2s_std_slot_config_t){.data_bit_width = (bits), .slot_mode = (mode)})

esp_err_t i2s_new_channel(const i2s_chan_config_t *cfg, i2s_chan_handle_t *tx,
                          i2s_chan_handle_t *rx);
esp_err_t i2s_del_channel(i2s_chan_handle_t chan);
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t chan, const i2s_std_config_t *cfg);
esp_err_t i2s_channel_enable(i2s_chan_handle_t chan);
esp_err_t i2s_channel_disable(i2s_chan_handle_t chan);
esp_err_t i2s_channel_write(i2s_chan_handle_t chan, const void *src, size_t size, size_t *written,
                            uint32_t timeout_ms);
esp_err_t i2s_channel_preload_data(i2s_chan_handle_t chan, const void *src, size_t size,
                                   size_t *loaded);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_I2S_STD_H */
