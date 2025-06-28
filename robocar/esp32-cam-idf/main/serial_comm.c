/**
 * @file serial_comm.c
 * @brief Serial communication implementation
 */

#include "serial_comm.h"
#include "camera_pins.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "serial_comm";

#define UART_PORT UART_NUM_2
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024

esp_err_t serial_init(void) {
    ESP_LOGI(TAG, "Initializing serial communication");

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Serial communication initialized on UART%d (TX:%d, RX:%d, %d baud)", 
             UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
    return ESP_OK;
}

esp_err_t serial_send_command(const char* command) {
    if (!command) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Sending command: %s", command);

    size_t command_len = strlen(command);
    int bytes_written = uart_write_bytes(UART_PORT, command, command_len);
    uart_write_bytes(UART_PORT, "\n", 1);  // Add newline

    if (bytes_written < 0) {
        ESP_LOGE(TAG, "Failed to send command");
        return ESP_FAIL;
    }

    return ESP_OK;
}

int serial_read_response(char* buffer, size_t buffer_size, int timeout_ms) {
    if (!buffer || buffer_size == 0) {
        return -1;
    }

    int bytes_read = uart_read_bytes(UART_PORT, (uint8_t*)buffer, buffer_size - 1, 
                                   timeout_ms / portTICK_PERIOD_MS);
    
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        ESP_LOGD(TAG, "Received response: %.*s", bytes_read, buffer);
    }

    return bytes_read;
}

esp_err_t serial_send_display_message(int line, const char* message) {
    if (!message || line < 0 || line > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    // Format: DISP:line:message
    char command[128];
    snprintf(command, sizeof(command), "DISP:%d:%s", line, message);
    
    ESP_LOGI(TAG, "Sending display message to line %d: %s", line, message);
    return serial_send_command(command);
}

void serial_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing serial communication");
    uart_driver_delete(UART_PORT);
}