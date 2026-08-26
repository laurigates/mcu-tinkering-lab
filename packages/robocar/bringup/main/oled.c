/**
 * @file oled.c
 * @brief SSD1306 over i2cdev, behind TCA9548A/PCA9548A channel I2C_BUS_CHANNEL_OLED.
 */

#include "oled.h"

#include <ctype.h>
#include <string.h>

#include "esp_log.h"
#include "i2c_bus.h"
#include "i2cdev.h"
#include "pin_config.h"

static const char *TAG = "oled";

#define OLED_PAGES (OLED_HEIGHT / 8)
#define OLED_FB_BYTES (OLED_WIDTH * OLED_PAGES)

/** SSD1306 I2C framing: the first byte of every transfer selects the stream. */
#define SSD1306_CTRL_CMD 0x00
#define SSD1306_CTRL_DATA 0x40

/** Bytes of framebuffer per I2C transfer. The panel auto-increments across
 *  transfers in horizontal addressing mode, so this only bounds the transaction
 *  size — it does not need to align to a page. */
#define OLED_FLUSH_CHUNK 128

static i2c_dev_t s_dev;
static bool s_available = false;
static uint8_t s_fb[OLED_FB_BYTES];

/* -------------------------------------------------------------------------- */
/* 5x7 font, column-major, LSB = topmost pixel                                  */
/*                                                                              */
/* Uppercase, digits and six punctuation marks only — everything this sweep      */
/* prints. A full ASCII table would be four times the size to render text that   */
/* is never drawn.                                                               */
/* -------------------------------------------------------------------------- */

static const uint8_t k_font_digits[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 9 */
};

static const uint8_t k_font_upper[26][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* Z */
};

static const uint8_t k_font_blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t k_font_dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
static const uint8_t k_font_dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
static const uint8_t k_font_slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
static const uint8_t k_font_colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
static const uint8_t k_font_query[5] = {0x02, 0x01, 0x51, 0x09, 0x06};

/** Glyph for @p c, folding case and substituting `?` for anything absent. */
static const uint8_t *glyph(char c)
{
    const unsigned char u = (unsigned char)toupper((unsigned char)c);

    if (u >= 'A' && u <= 'Z') {
        return k_font_upper[u - 'A'];
    }
    if (u >= '0' && u <= '9') {
        return k_font_digits[u - '0'];
    }
    switch (u) {
        case ' ':
            return k_font_blank;
        case '-':
        case '_':
            return k_font_dash;
        case '.':
            return k_font_dot;
        case '/':
            return k_font_slash;
        case ':':
            return k_font_colon;
        default:
            return k_font_query;
    }
}

/* -------------------------------------------------------------------------- */
/* Panel I/O                                                                    */
/* -------------------------------------------------------------------------- */

/** Send a command run. The caller owns the mux channel and the bus mutex. */
static esp_err_t send_cmds(const uint8_t *cmds, size_t len)
{
    const uint8_t ctrl = SSD1306_CTRL_CMD;
    return i2c_dev_write(&s_dev, &ctrl, 1, cmds, len);
}

esp_err_t oled_init(void)
{
    if (s_available) {
        return ESP_OK;
    }
    if (!i2c_bus_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_dev, 0, sizeof(s_dev));
    s_dev.port = I2C_NUM_0;
    s_dev.addr = OLED_I2C_ADDR;
    s_dev.cfg.sda_io_num = I2C_SDA_PIN;
    s_dev.cfg.scl_io_num = I2C_SCL_PIN;
    s_dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_dev_create_mutex(&s_dev);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_bus_select_channel(I2C_BUS_CHANNEL_OLED);
    if (err != ESP_OK) {
        i2c_dev_delete_mutex(&s_dev);
        return err;
    }

    /* Probe before initialising, so "no panel fitted" is reported as
     * ESP_ERR_NOT_FOUND rather than as a write error from the first command —
     * the sweep renders those two states very differently (SKIP vs FAIL). */
    err = i2c_dev_probe(&s_dev, I2C_DEV_WRITE);
    if (err != ESP_OK) {
        i2c_bus_release();
        i2c_dev_delete_mutex(&s_dev);
        return ESP_ERR_NOT_FOUND;
    }

    static const uint8_t init_seq[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* clock divide / oscillator frequency */
        0xA8, 0x3F, /* multiplex ratio = 63 (64-row panel) */
        0xD3, 0x00, /* display offset = 0 */
        0x40,       /* start line = 0 */
        0x8D, 0x14, /* charge pump on (panel has no external Vcc) */
        0x20, 0x00, /* horizontal addressing mode */
        0xA1,       /* segment remap: column 127 -> SEG0 */
        0xC8,       /* COM scan direction remapped */
        0xDA, 0x12, /* COM pins: alternative, no left/right remap */
        0x81, 0x7F, /* contrast */
        0xD9, 0xF1, /* pre-charge period */
        0xDB, 0x40, /* VCOMH deselect level */
        0xA4,       /* resume from RAM (not all-pixels-on) */
        0xA6,       /* normal, not inverted */
        0xAF,       /* display on */
    };

    err = send_cmds(init_seq, sizeof(init_seq));
    i2c_bus_release();

    if (err != ESP_OK) {
        i2c_dev_delete_mutex(&s_dev);
        return err;
    }

    s_available = true;
    memset(s_fb, 0, sizeof(s_fb));
    ESP_LOGI(TAG, "SSD1306 %dx%d up at 0x%02X on mux ch%d", OLED_WIDTH, OLED_HEIGHT, OLED_I2C_ADDR,
             I2C_BUS_CHANNEL_OLED);
    return oled_flush();
}

bool oled_available(void)
{
    return s_available;
}

void oled_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void oled_text(uint8_t col, uint8_t row, const char *text)
{
    if (!s_available || text == NULL || row >= OLED_ROWS) {
        return;
    }

    for (const char *p = text; *p != '\0' && col < OLED_COLS; p++, col++) {
        const uint8_t *g = glyph(*p);
        const size_t x = (size_t)col * 6u;
        uint8_t *cell = &s_fb[(size_t)row * OLED_WIDTH + x];
        memcpy(cell, g, 5);
        cell[5] = 0x00; /* inter-character gap */
    }
}

esp_err_t oled_flush(void)
{
    if (!s_available) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_bus_select_channel(I2C_BUS_CHANNEL_OLED);
    if (err != ESP_OK) {
        return err;
    }

    static const uint8_t window[] = {
        0x21, 0x00, OLED_WIDTH - 1, /* column range */
        0x22, 0x00, OLED_PAGES - 1, /* page range */
    };
    err = send_cmds(window, sizeof(window));

    const uint8_t ctrl = SSD1306_CTRL_DATA;
    for (size_t off = 0; err == ESP_OK && off < sizeof(s_fb); off += OLED_FLUSH_CHUNK) {
        const size_t n =
            (sizeof(s_fb) - off < OLED_FLUSH_CHUNK) ? (sizeof(s_fb) - off) : OLED_FLUSH_CHUNK;
        err = i2c_dev_write(&s_dev, &ctrl, 1, &s_fb[off], n);
    }

    i2c_bus_release();
    return err;
}
