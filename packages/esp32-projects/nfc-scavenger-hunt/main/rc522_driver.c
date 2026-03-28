#include "rc522_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rc522";

/*
 * Pin definitions — change these for your board variant.
 * XIAO ESP32-S3 defaults shown. See tag_config.h header for SuperMini pins.
 */
#if defined(BOARD_SUPERMINI)
#define RC522_SPI_HOST SPI2_HOST
#define PIN_SCK 12
#define PIN_MISO 13
#define PIN_MOSI 11
#define PIN_CS 10
#define PIN_RST 14
#else  // XIAO ESP32-S3 (default)
#define RC522_SPI_HOST SPI2_HOST
#define PIN_SCK 7
#define PIN_MISO 8
#define PIN_MOSI 9
#define PIN_CS 5
#define PIN_RST 4
#endif

/* RC522 register addresses */
#define REG_COMMAND 0x01
#define REG_COM_IEN 0x02
#define REG_DIV_IEN 0x03
#define REG_COM_IRQ 0x04
#define REG_DIV_IRQ 0x05
#define REG_ERROR 0x06
#define REG_STATUS1 0x07
#define REG_STATUS2 0x08
#define REG_FIFO_DATA 0x09
#define REG_FIFO_LEVEL 0x0A
#define REG_CONTROL 0x0C
#define REG_BIT_FRAMING 0x0D
#define REG_COLL 0x0E
#define REG_MODE 0x11
#define REG_TX_CONTROL 0x14
#define REG_TX_ASK 0x15
#define REG_CRC_RESULT_H 0x21
#define REG_CRC_RESULT_L 0x22
#define REG_MOD_WIDTH 0x24
#define REG_T_MODE 0x2A
#define REG_T_PRESCALER 0x2B
#define REG_T_RELOAD_H 0x2C
#define REG_T_RELOAD_L 0x2D
#define REG_AUTO_TEST 0x36
#define REG_VERSION 0x37

/* RC522 commands */
#define CMD_IDLE 0x00
#define CMD_CALC_CRC 0x03
#define CMD_TRANSCEIVE 0x0C
#define CMD_MF_AUTHENT 0x0E
#define CMD_SOFT_RESET 0x0F

/* PICC commands */
#define PICC_REQA 0x26
#define PICC_ANTICOLL_CL1 0x93
#define PICC_ANTICOLL_CL2 0x95
#define PICC_SELECT_CL1 0x93
#define PICC_SELECT_CL2 0x95

/* Error/IRQ bits */
#define IRQ_IDLE (1 << 4)
#define IRQ_RX (1 << 5)
#define IRQ_TIMER (1 << 0)
#define ERR_BUFFER_OVFL (1 << 4)
#define ERR_COLL (1 << 3)
#define ERR_CRC (1 << 2)
#define ERR_PARITY (1 << 1)
#define ERR_PROTOCOL (1 << 0)

static spi_device_handle_t spi_handle;

static void rc522_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(reg << 1) & 0x7E, val};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_transmit(spi_handle, &t);
}

static uint8_t rc522_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {((reg << 1) & 0x7E) | 0x80, 0x00};
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi_handle, &t);
    return rx[1];
}

static void rc522_set_bit_mask(uint8_t reg, uint8_t mask)
{
    uint8_t val = rc522_read_reg(reg);
    rc522_write_reg(reg, val | mask);
}

static void rc522_clear_bit_mask(uint8_t reg, uint8_t mask)
{
    uint8_t val = rc522_read_reg(reg);
    rc522_write_reg(reg, val & ~mask);
}

static void rc522_antenna_on(void)
{
    uint8_t val = rc522_read_reg(REG_TX_CONTROL);
    if ((val & 0x03) != 0x03) {
        rc522_set_bit_mask(REG_TX_CONTROL, 0x03);
    }
}

static void rc522_reset(void)
{
    rc522_write_reg(REG_COMMAND, CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Wait for reset to complete */
    int timeout = 100;
    while (timeout-- > 0) {
        if (!(rc522_read_reg(REG_COMMAND) & (1 << 4))) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static esp_err_t rc522_communicate(uint8_t cmd, uint8_t *send_data, uint8_t send_len,
                                   uint8_t *recv_data, uint8_t *recv_len, uint8_t *valid_bits)
{
    uint8_t irq_en = 0x00;
    uint8_t wait_irq = 0x00;

    if (cmd == CMD_TRANSCEIVE) {
        irq_en = 0x77;
        wait_irq = IRQ_RX | IRQ_IDLE;
    }

    rc522_write_reg(REG_COM_IEN, irq_en | 0x80);
    rc522_clear_bit_mask(REG_COM_IRQ, 0x80);
    rc522_set_bit_mask(REG_FIFO_LEVEL, 0x80);  // FlushBuffer
    rc522_write_reg(REG_COMMAND, CMD_IDLE);

    /* Write data to FIFO */
    for (uint8_t i = 0; i < send_len; i++) {
        rc522_write_reg(REG_FIFO_DATA, send_data[i]);
    }

    rc522_write_reg(REG_COMMAND, cmd);

    if (cmd == CMD_TRANSCEIVE) {
        rc522_set_bit_mask(REG_BIT_FRAMING, 0x80);  // StartSend
    }

    /* Wait for completion */
    int timeout = 2000;
    uint8_t irq;
    do {
        irq = rc522_read_reg(REG_COM_IRQ);
        timeout--;
    } while (timeout > 0 && !(irq & wait_irq) && !(irq & IRQ_TIMER));

    rc522_clear_bit_mask(REG_BIT_FRAMING, 0x80);

    if (timeout == 0) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t error = rc522_read_reg(REG_ERROR);
    if (error & (ERR_BUFFER_OVFL | ERR_CRC | ERR_PARITY | ERR_PROTOCOL)) {
        return ESP_FAIL;
    }

    if (recv_data && recv_len) {
        uint8_t n = rc522_read_reg(REG_FIFO_LEVEL);
        if (n > *recv_len) {
            n = *recv_len;
        }
        *recv_len = n;
        for (uint8_t i = 0; i < n; i++) {
            recv_data[i] = rc522_read_reg(REG_FIFO_DATA);
        }
        if (valid_bits) {
            *valid_bits = rc522_read_reg(REG_CONTROL) & 0x07;
        }
    }

    /* Check for collision */
    if (error & ERR_COLL) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t rc522_request(uint8_t *atqa)
{
    uint8_t cmd = PICC_REQA;
    uint8_t recv_len = 2;
    uint8_t valid_bits = 7;

    rc522_write_reg(REG_BIT_FRAMING, 0x07);

    esp_err_t ret = rc522_communicate(CMD_TRANSCEIVE, &cmd, 1, atqa, &recv_len, &valid_bits);
    if (ret != ESP_OK) {
        return ret;
    }

    return (recv_len == 2) ? ESP_OK : ESP_FAIL;
}

static esp_err_t rc522_anticoll_select(uint8_t cascade_level, uint8_t *uid_part, uint8_t *sak)
{
    uint8_t cmd_buf[9];
    uint8_t recv_buf[5];
    uint8_t recv_len;

    /* Anticollision */
    cmd_buf[0] = cascade_level;
    cmd_buf[1] = 0x20;  // NVB: 2 bytes sent

    rc522_write_reg(REG_BIT_FRAMING, 0x00);
    recv_len = 5;
    esp_err_t ret = rc522_communicate(CMD_TRANSCEIVE, cmd_buf, 2, recv_buf, &recv_len, NULL);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Verify BCC */
    uint8_t bcc = 0;
    for (int i = 0; i < 4; i++) {
        bcc ^= recv_buf[i];
    }
    if (bcc != recv_buf[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    memcpy(uid_part, recv_buf, 4);

    /* Select */
    cmd_buf[0] = cascade_level;
    cmd_buf[1] = 0x70;  // NVB: 7 bytes sent
    memcpy(&cmd_buf[2], recv_buf, 5);

    /* Calculate CRC for select command */
    rc522_write_reg(REG_COMMAND, CMD_IDLE);
    rc522_clear_bit_mask(REG_DIV_IRQ, 0x04);
    rc522_set_bit_mask(REG_FIFO_LEVEL, 0x80);
    for (int i = 0; i < 7; i++) {
        rc522_write_reg(REG_FIFO_DATA, cmd_buf[i]);
    }
    rc522_write_reg(REG_COMMAND, CMD_CALC_CRC);

    int timeout = 1000;
    while (timeout-- > 0) {
        uint8_t div_irq = rc522_read_reg(REG_DIV_IRQ);
        if (div_irq & 0x04) {
            break;
        }
    }

    cmd_buf[7] = rc522_read_reg(REG_CRC_RESULT_L);
    cmd_buf[8] = rc522_read_reg(REG_CRC_RESULT_H);

    recv_len = 3;
    ret = rc522_communicate(CMD_TRANSCEIVE, cmd_buf, 9, recv_buf, &recv_len, NULL);
    if (ret != ESP_OK) {
        return ret;
    }

    if (sak) {
        *sak = recv_buf[0];
    }

    return ESP_OK;
}

esp_err_t rc522_init(void)
{
    /* Configure RST pin */
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_conf);
    gpio_set_level(PIN_RST, 1);

    /* Configure SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t ret = spi_bus_initialize(RC522_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 5000000,  // 5 MHz
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    ret = spi_bus_add_device(RC522_SPI_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Hardware reset */
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    rc522_reset();

    /* Timer: TPrescaler = 0x0D3E → ~25ms timeout */
    rc522_write_reg(REG_T_MODE, 0x8D);
    rc522_write_reg(REG_T_PRESCALER, 0x3E);
    rc522_write_reg(REG_T_RELOAD_H, 0x00);
    rc522_write_reg(REG_T_RELOAD_L, 0x1E);

    rc522_write_reg(REG_TX_ASK, 0x40);
    rc522_write_reg(REG_MODE, 0x3D);

    rc522_antenna_on();

    uint8_t version = rc522_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "RC522 initialized, firmware version: 0x%02X", version);

    if (version == 0x00 || version == 0xFF) {
        ESP_LOGE(TAG, "RC522 not detected — check wiring");
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

bool rc522_poll_tag(uint8_t *uid, uint8_t *uid_len)
{
    uint8_t atqa[2];
    if (rc522_request(atqa) != ESP_OK) {
        return false;
    }

    /* Cascade level 1 */
    uint8_t uid_part[4];
    uint8_t sak = 0;
    if (rc522_anticoll_select(PICC_ANTICOLL_CL1, uid_part, &sak) != ESP_OK) {
        return false;
    }

    if (uid_part[0] == 0x88) {
        /* Cascade tag — UID is 7 bytes, need level 2 */
        memcpy(uid, &uid_part[1], 3);

        uint8_t uid_part2[4];
        if (rc522_anticoll_select(PICC_ANTICOLL_CL2, uid_part2, &sak) != ESP_OK) {
            return false;
        }
        memcpy(&uid[3], uid_part2, 4);
        *uid_len = 7;
    } else {
        /* Single-size UID (4 bytes) */
        memcpy(uid, uid_part, 4);
        *uid_len = 4;
    }

    return true;
}
