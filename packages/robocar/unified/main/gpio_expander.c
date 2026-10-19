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
static uint8_t s_addr = 0;  // the address that answered at init

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

/* GPA7 and GPB7 are output-only on the Rev D part (DS20001952D p. 1, p. 11,
 * § 3.5.1 note), but IODIR powers up 0xFFFF, which leaves both as inputs with
 * nothing behind them. Make them what they can only be: outputs, latched low
 * before the direction flips so the pin never drives high. Bus held by the
 * caller. */
static esp_err_t drive_output_only_pins_low_locked(void)
{
    for (uint8_t pin = 0; pin < GPIO_EXPANDER_PIN_COUNT; pin++) {
        if (!gpio_expander_pin_output_only(pin))
            continue;
        esp_err_t ret = mcp23x17_set_level(&s_mcp23017, pin, 0);
        if (ret != ESP_OK)
            return ret;
        ret = mcp23x17_set_mode(&s_mcp23017, pin, MCP23X17_GPIO_OUTPUT);
        if (ret != ESP_OK)
            return ret;
    }
    return ESP_OK;
}

/* Try one address: a descriptor, the bus clock, and an IODIR read on the
 * expander's channel. When the chip answers, *found is set, the descriptor is
 * kept and GPA7/GPB7 are driven low as outputs (*drive_ret says whether that
 * worked). When the read fails, the descriptor is freed and the error left in
 * *no_answer. A non-ESP_OK return is a bus or driver fault, for the caller to
 * pass on. */
static esp_err_t probe_at(uint8_t addr, bool *found, uint16_t *iodir, esp_err_t *drive_ret,
                          esp_err_t *no_answer)
{
    *found = false;
    esp_err_t ret = mcp23x17_init_desc(&s_mcp23017, addr, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017 init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* mcp23x17_init_desc() hardcodes 1 MHz, which differs from the rest of the
     * bus and would make i2cdev delete and reinstall the I2C driver on every
     * alternation between this device and the mux. Match i2c_bus.c's
     * pin_bus_clock(). */
    s_mcp23017.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_MCP23017);
    if (ret != ESP_OK) {
        mcp23x17_free_desc(&s_mcp23017);
        return ret;
    }
    ret = mcp23x17_port_get_mode(&s_mcp23017, iodir);
    if (ret == ESP_OK)
        *drive_ret = drive_output_only_pins_low_locked();
    i2c_bus_release();

    if (ret != ESP_OK) {
        *no_answer = ret;
        mcp23x17_free_desc(&s_mcp23017);
        return ESP_OK;
    }
    *found = true;
    return ESP_OK;
}

esp_err_t gpio_expander_init(void)
{
    if (s_available) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* MCP23017_ADDR first, then the rest of 0x20-0x27: the carriers strap
     * A0-A2 to GND, but a CJMCU-2317 on a harness biases them through its own
     * resistor array, possibly high (0x27, #781). A missing board is expected:
     * the expander is optional hardware, so absence downgrades to a warning. */
    uint8_t order[GPIO_EXPANDER_ADDR_COUNT];
    size_t n = gpio_expander_probe_order(MCP23017_ADDR, order);
    esp_err_t no_answer = ESP_ERR_NOT_FOUND;
    for (size_t i = 0; i < n; i++) {
        bool found = false;
        uint16_t iodir = 0;
        esp_err_t drive_ret = ESP_OK;
        esp_err_t ret = probe_at(order[i], &found, &iodir, &drive_ret, &no_answer);
        if (ret != ESP_OK)
            return ret;
        if (!found)
            continue;

        s_available = true;
        s_addr = order[i];
        ESP_LOGI(TAG, "MCP23017 initialized at 0x%02X on TCA9548A ch%d (IODIR at probe=0x%04X)",
                 s_addr, I2C_BUS_CHANNEL_MCP23017, iodir);
        if (s_addr != MCP23017_ADDR)
            ESP_LOGW(TAG,
                     "MCP23017 answered at 0x%02X, not MCP23017_ADDR 0x%02X: its A0-A2 are "
                     "not all low; strap them or set MCP23017_ADDR",
                     s_addr, MCP23017_ADDR);
        if (drive_ret == ESP_OK)
            ESP_LOGI(TAG, "GPA7/GPB7 are output-only: set to outputs, driven low");
        else
            ESP_LOGW(TAG, "Could not set output-only GPA7/GPB7 to outputs (%s) — left as inputs",
                     esp_err_to_name(drive_ret));
        return ESP_OK;
    }

    ESP_LOGW(TAG, "No MCP23017 at 0x%02X-0x%02X on TCA9548A ch%d (%s) — expander unavailable",
             GPIO_EXPANDER_ADDR_BASE, GPIO_EXPANDER_ADDR_BASE + GPIO_EXPANDER_ADDR_COUNT - 1,
             I2C_BUS_CHANNEL_MCP23017, esp_err_to_name(no_answer));
    return ESP_OK;
}

uint8_t gpio_expander_address(void)
{
    return s_available ? s_addr : 0;
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
    switch (gpio_expander_check_mode(pin, mode)) {
        case GPIO_EXPANDER_CHECK_OK:
            break;
        case GPIO_EXPANDER_CHECK_OUTPUT_ONLY:
            return ESP_ERR_NOT_SUPPORTED;
        case GPIO_EXPANDER_CHECK_BAD_PIN:
        case GPIO_EXPANDER_CHECK_BAD_MODE:
        default:
            return ESP_ERR_INVALID_ARG;
    }
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
