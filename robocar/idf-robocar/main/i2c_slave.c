/**
 * @file i2c_slave.c
 * @brief I2C slave implementation for main board
 */

#include "i2c_slave.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "i2c_slave";

// Global state
static bool i2c_initialized = false;
static bool task_running = false;
static TaskHandle_t i2c_task_handle = NULL;
static SemaphoreHandle_t status_mutex = NULL;
static status_data_t current_status = {0};

// External function declarations (implemented in main.c)
extern void process_i2c_movement_command(movement_command_t movement, uint8_t speed);
extern void process_i2c_sound_command(sound_command_t sound);
extern void process_i2c_servo_command(servo_command_t servo, uint8_t angle);
extern void process_i2c_display_command(uint8_t line, const char* message);

// Internal functions
static void i2c_slave_task(void *pvParameters);
static void process_i2c_command(const i2c_command_packet_t* command, i2c_response_packet_t* response);
static void prepare_response(i2c_response_packet_t* response, uint8_t status, const uint8_t* data, uint8_t data_len);

esp_err_t i2c_slave_init(void) {
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C slave already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2C slave");

    // Create mutex for status data
    status_mutex = xSemaphoreCreateMutex();
    if (!status_mutex) {
        ESP_LOGE(TAG, "Failed to create status mutex");
        return ESP_ERR_NO_MEM;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_COMM_SDA_IO,
        .scl_io_num = I2C_COMM_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Pins 26/27 support internal pull-ups
        .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Internal pull-ups enabled
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDRESS,
        .slave.maximum_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_SLAVE_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        vSemaphoreDelete(status_mutex);
        return err;
    }

    err = i2c_driver_install(I2C_SLAVE_NUM, conf.mode, 
                            I2C_SLAVE_RX_BUF_LEN, 
                            I2C_SLAVE_TX_BUF_LEN, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
        vSemaphoreDelete(status_mutex);
        return err;
    }

    // Initialize status data
    memset(&current_status, 0, sizeof(current_status));
    current_status.wifi_connected = 1; // Assume connected for now

    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C slave initialized successfully (Address: 0x%02X, SDA:%d, SCL:%d)", 
             I2C_SLAVE_ADDRESS, I2C_COMM_SDA_IO, I2C_COMM_SCL_IO);

    return ESP_OK;
}

void i2c_slave_deinit(void) {
    if (i2c_initialized) {
        i2c_slave_stop_task();
        i2c_driver_delete(I2C_SLAVE_NUM);
        
        if (status_mutex) {
            vSemaphoreDelete(status_mutex);
            status_mutex = NULL;
        }
        
        i2c_initialized = false;
        ESP_LOGI(TAG, "I2C slave deinitialized");
    }
}

void i2c_slave_start_task(void) {
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C slave not initialized");
        return;
    }

    if (task_running) {
        ESP_LOGW(TAG, "I2C slave task already running");
        return;
    }

    BaseType_t result = xTaskCreate(i2c_slave_task, "i2c_slave", 4096, NULL, 5, &i2c_task_handle);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create I2C slave task");
        return;
    }

    task_running = true;
    ESP_LOGI(TAG, "I2C slave task started");
}

void i2c_slave_stop_task(void) {
    if (task_running && i2c_task_handle) {
        task_running = false;
        vTaskDelete(i2c_task_handle);
        i2c_task_handle = NULL;
        ESP_LOGI(TAG, "I2C slave task stopped");
    }
}

void i2c_slave_set_status(const status_data_t* status) {
    if (!status || !status_mutex) return;

    if (xSemaphoreTake(status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&current_status, status, sizeof(status_data_t));
        xSemaphoreGive(status_mutex);
    }
}

void i2c_slave_get_status(status_data_t* status) {
    if (!status || !status_mutex) return;

    if (xSemaphoreTake(status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(status, &current_status, sizeof(status_data_t));
        xSemaphoreGive(status_mutex);
    }
}

bool i2c_slave_is_ready(void) {
    return i2c_initialized && task_running;
}

static void i2c_slave_task(void *pvParameters) {
    ESP_LOGI(TAG, "I2C slave task started");
    
    uint8_t rx_buffer[I2C_SLAVE_RX_BUF_LEN];
    uint8_t tx_buffer[I2C_SLAVE_TX_BUF_LEN];
    
    while (task_running) {
        // Read data from master
        int rx_len = i2c_slave_read_buffer(I2C_SLAVE_NUM, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(100));
        
        if (rx_len > 0) {
            ESP_LOGD(TAG, "Received %d bytes from master", rx_len);
            
            if (rx_len == sizeof(i2c_command_packet_t)) {
                i2c_command_packet_t* command = (i2c_command_packet_t*)rx_buffer;
                i2c_response_packet_t* response = (i2c_response_packet_t*)tx_buffer;
                
                // Verify checksum
                if (verify_checksum((uint8_t*)command, sizeof(i2c_command_packet_t) - 1, command->checksum)) {
                    process_i2c_command(command, response);
                    
                    // Send response
                    int tx_len = i2c_slave_write_buffer(I2C_SLAVE_NUM, tx_buffer, sizeof(i2c_response_packet_t), pdMS_TO_TICKS(100));
                    if (tx_len != sizeof(i2c_response_packet_t)) {
                        ESP_LOGW(TAG, "Failed to send complete response");
                    }
                } else {
                    ESP_LOGE(TAG, "Command checksum verification failed");
                    
                    // Send error response
                    prepare_response(response, 0x01, NULL, 0); // 0x01 = ERROR
                    i2c_slave_write_buffer(I2C_SLAVE_NUM, tx_buffer, sizeof(i2c_response_packet_t), pdMS_TO_TICKS(100));
                }
            } else {
                ESP_LOGW(TAG, "Invalid command packet size: %d", rx_len);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent tight loop
    }
    
    ESP_LOGI(TAG, "I2C slave task ended");
    vTaskDelete(NULL);
}

static void process_i2c_command(const i2c_command_packet_t* command, i2c_response_packet_t* response) {
    ESP_LOGD(TAG, "Processing command type: 0x%02X", command->command_type);
    
    switch (command->command_type) {
        case CMD_TYPE_MOVEMENT: {
            if (command->data_length == sizeof(movement_data_t)) {
                movement_data_t* move_data = (movement_data_t*)command->data;
                process_i2c_movement_command(move_data->movement, move_data->speed);
                prepare_response(response, 0x00, NULL, 0); // 0x00 = OK
            } else {
                prepare_response(response, 0x01, NULL, 0); // 0x01 = ERROR
            }
            break;
        }
        
        case CMD_TYPE_SOUND: {
            if (command->data_length == sizeof(sound_data_t)) {
                sound_data_t* sound_data = (sound_data_t*)command->data;
                process_i2c_sound_command(sound_data->sound_type);
                prepare_response(response, 0x00, NULL, 0);
            } else {
                prepare_response(response, 0x01, NULL, 0);
            }
            break;
        }
        
        case CMD_TYPE_SERVO: {
            if (command->data_length == sizeof(servo_data_t)) {
                servo_data_t* servo_data = (servo_data_t*)command->data;
                process_i2c_servo_command(servo_data->servo_type, servo_data->angle);
                prepare_response(response, 0x00, NULL, 0);
            } else {
                prepare_response(response, 0x01, NULL, 0);
            }
            break;
        }
        
        case CMD_TYPE_DISPLAY: {
            if (command->data_length == sizeof(display_data_t)) {
                display_data_t* display_data = (display_data_t*)command->data;
                process_i2c_display_command(display_data->line, display_data->message);
                prepare_response(response, 0x00, NULL, 0);
            } else {
                prepare_response(response, 0x01, NULL, 0);
            }
            break;
        }
        
        case CMD_TYPE_STATUS: {
            status_data_t status;
            i2c_slave_get_status(&status);
            prepare_response(response, 0x00, (uint8_t*)&status, sizeof(status_data_t));
            break;
        }
        
        case CMD_TYPE_PING: {
            prepare_response(response, 0x00, NULL, 0);
            ESP_LOGD(TAG, "Ping received and acknowledged");
            break;
        }
        
        default: {
            ESP_LOGW(TAG, "Unknown command type: 0x%02X", command->command_type);
            prepare_response(response, 0x01, NULL, 0);
            break;
        }
    }
}

static void prepare_response(i2c_response_packet_t* response, uint8_t status, const uint8_t* data, uint8_t data_len) {
    if (!response) return;
    
    response->status = status;
    response->data_length = (data_len > sizeof(response->data)) ? sizeof(response->data) : data_len;
    
    if (data && response->data_length > 0) {
        memcpy(response->data, data, response->data_length);
    } else {
        memset(response->data, 0, sizeof(response->data));
    }
    
    response->checksum = calculate_checksum((uint8_t*)response, sizeof(i2c_response_packet_t) - 1);
}