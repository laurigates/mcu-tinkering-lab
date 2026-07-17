/**
 * @file gpio_expander.c
 * @brief MCP23017 16-bit GPIO expander via TCA9548A channel 2
 */

#include "gpio_expander.h"

#include <esp_log.h>
#include <mcp23x17.h>

#include "i2c_bus.h"
#include "pin_config.h"

static const char *TAG = "gpio_expander";

static mcp23x17_t s_mcp23017;
static bool s_available = false;

// Select the expander's TCA9548A channel and run one driver call while
// holding the bus mutex. Returns ESP_ERR_INVALID_STATE when no device
// was detected at init so callers can treat the expander as optional.
#define EXPANDER_OP(call)                                                    \
    do {                                                                     \
        if (!s_available)                                                    \
            return ESP_ERR_INVALID_STATE;                                    \
        esp_err_t op_ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_MCP23017); \
        if (op_ret != ESP_OK)                                                \
            return op_ret;                                                   \
        op_ret = (call);                                                     \
        i2c_bus_release();                                                   \
        return op_ret;                                                       \
    } while (0)

esp_err_t gpio_expander_init(void)
{
    if (s_available) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    esp_err_t ret =
        mcp23x17_init_desc(&s_mcp23017, MCP23017_ADDR, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017 init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Probe: read the IODIR registers. A missing board is expected — the
    // expander is optional hardware, so absence downgrades to a warning.
    ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_MCP23017);
    if (ret != ESP_OK) {
        mcp23x17_free_desc(&s_mcp23017);
        return ret;
    }

    uint16_t mode = 0;
    ret = mcp23x17_port_get_mode(&s_mcp23017, &mode);
    i2c_bus_release();

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No MCP23017 at 0x%02X on TCA9548A ch%d (%s) — expander unavailable",
                 MCP23017_ADDR, I2C_BUS_CHANNEL_MCP23017, esp_err_to_name(ret));
        mcp23x17_free_desc(&s_mcp23017);
        return ESP_OK;
    }

    s_available = true;
    ESP_LOGI(TAG, "MCP23017 initialized at 0x%02X on TCA9548A ch%d (IODIR=0x%04X)", MCP23017_ADDR,
             I2C_BUS_CHANNEL_MCP23017, mode);
    return ESP_OK;
}

bool gpio_expander_available(void)
{
    return s_available;
}

static esp_err_t set_mode_locked(uint8_t pin, gpio_expander_mode_t mode)
{
    esp_err_t ret = mcp23x17_set_mode(&s_mcp23017, pin,
                                      mode == GPIO_EXPANDER_OUTPUT ? MCP23X17_GPIO_OUTPUT
                                                                   : MCP23X17_GPIO_INPUT);
    if (ret != ESP_OK)
        return ret;
    return mcp23x17_set_pullup(&s_mcp23017, pin, mode == GPIO_EXPANDER_INPUT_PULLUP);
}

esp_err_t gpio_expander_set_mode(uint8_t pin, gpio_expander_mode_t mode)
{
    if (pin > 15)
        return ESP_ERR_INVALID_ARG;
    EXPANDER_OP(set_mode_locked(pin, mode));
}

esp_err_t gpio_expander_write(uint8_t pin, bool level)
{
    if (pin > 15)
        return ESP_ERR_INVALID_ARG;
    EXPANDER_OP(mcp23x17_set_level(&s_mcp23017, pin, level ? 1 : 0));
}

static esp_err_t read_locked(uint8_t pin, bool *level)
{
    uint32_t val = 0;
    esp_err_t ret = mcp23x17_get_level(&s_mcp23017, pin, &val);
    if (ret == ESP_OK)
        *level = (val != 0);
    return ret;
}

esp_err_t gpio_expander_read(uint8_t pin, bool *level)
{
    if (pin > 15)
        return ESP_ERR_INVALID_ARG;
    if (!level)
        return ESP_ERR_INVALID_ARG;
    EXPANDER_OP(read_locked(pin, level));
}

esp_err_t gpio_expander_read_port(uint16_t *value)
{
    if (!value)
        return ESP_ERR_INVALID_ARG;
    EXPANDER_OP(mcp23x17_port_read(&s_mcp23017, value));
}
