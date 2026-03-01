/**
 * @file i2c_master.c
 * @brief I2C master implementation for ESP32-CAM (New Driver API)
 */

#include "i2c_master.h"
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "i2c_master";
static bool i2c_initialized = false;
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_dev_handle = NULL;
static uint8_t current_sequence_number = 0;

// Configuration for retries
#define I2C_MAX_RETRIES 3
#define I2C_RETRY_DELAY_MS 100

esp_err_t i2c_master_init(void)
{
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C master already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2C master with new driver API");

    // Configure I2C master bus
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_COMM_SCL_IO,
        .sda_io_num = I2C_COMM_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    // Configure I2C device (slave)
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_SLAVE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
        return err;
    }

    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C master initialized successfully (SDA:%d, SCL:%d, %dHz)", I2C_COMM_SDA_IO,
             I2C_COMM_SCL_IO, I2C_MASTER_FREQ_HZ);

    return ESP_OK;
}

void i2c_master_deinit(void)
{
    if (i2c_initialized) {
        if (i2c_dev_handle) {
            i2c_master_bus_rm_device(i2c_dev_handle);
            i2c_dev_handle = NULL;
        }
        if (i2c_bus_handle) {
            i2c_del_master_bus(i2c_bus_handle);
            i2c_bus_handle = NULL;
        }
        i2c_initialized = false;
        ESP_LOGI(TAG, "I2C master deinitialized");
    }
}

esp_err_t i2c_master_send_command(const i2c_command_packet_t *command,
                                  i2c_response_packet_t *response, uint32_t timeout_ms)
{
    if (!i2c_initialized || !i2c_dev_handle) {
        ESP_LOGE(TAG, "I2C master not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!command) {
        ESP_LOGE(TAG, "Invalid command pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Verify command checksum
    if (!verify_checksum((uint8_t *)command, sizeof(i2c_command_packet_t) - 1, command->checksum)) {
        ESP_LOGE(TAG, "Command checksum verification failed");
        return ESP_ERR_INVALID_CRC;
    }

    esp_err_t err = ESP_FAIL;
    int retries = 0;

    // Retry loop
    while (retries < I2C_MAX_RETRIES) {
        // Send command using new I2C driver API
        err = i2c_master_transmit(i2c_dev_handle, (uint8_t *)command, sizeof(i2c_command_packet_t),
                                  timeout_ms);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2C transmit failed (attempt %d/%d): %s", retries + 1, I2C_MAX_RETRIES,
                     esp_err_to_name(err));
            retries++;
            if (retries < I2C_MAX_RETRIES) {
                vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
                continue;
            }
            return err;
        }

        // Read response if requested
        if (response) {
            err = i2c_master_receive(i2c_dev_handle, (uint8_t *)response,
                                     sizeof(i2c_response_packet_t), timeout_ms);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "I2C receive failed (attempt %d/%d): %s", retries + 1,
                         I2C_MAX_RETRIES, esp_err_to_name(err));
                retries++;
                if (retries < I2C_MAX_RETRIES) {
                    vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
                    continue;
                }
                return err;
            }

            // Verify response checksum
            if (!verify_checksum((uint8_t *)response, sizeof(i2c_response_packet_t) - 1,
                                 response->checksum)) {
                ESP_LOGW(TAG, "Response checksum verification failed (attempt %d/%d)", retries + 1,
                         I2C_MAX_RETRIES);
                retries++;
                if (retries < I2C_MAX_RETRIES) {
                    vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
                    continue;
                }
                return ESP_ERR_INVALID_CRC;
            }

            // Verify sequence number matches
            if (response->sequence_number != command->sequence_number) {
                ESP_LOGW(TAG, "Sequence number mismatch: sent %d, received %d",
                         command->sequence_number, response->sequence_number);
                retries++;
                if (retries < I2C_MAX_RETRIES) {
                    vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
                    continue;
                }
                return ESP_ERR_INVALID_RESPONSE;
            }

            ESP_LOGD(TAG, "Command sent successfully, status: 0x%02X, seq: %d", response->status,
                     response->sequence_number);
        } else {
            ESP_LOGD(TAG, "Command sent successfully (no response)");
        }

        return ESP_OK;
    }

    return err;
}

// Helper to get next sequence number
static uint8_t get_next_sequence_number(void)
{
    return ++current_sequence_number;
}

esp_err_t i2c_send_movement_command(movement_command_t movement, uint8_t speed)
{
    i2c_command_packet_t command;
    uint8_t seq = get_next_sequence_number();
    prepare_movement_command(&command, movement, speed, seq);

    ESP_LOGI(TAG, "Sending movement command: %d, speed: %d, seq: %d", movement, speed, seq);
    return i2c_master_send_command(&command, NULL, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t i2c_send_sound_command(sound_command_t sound)
{
    i2c_command_packet_t command;
    uint8_t seq = get_next_sequence_number();
    prepare_sound_command(&command, sound, seq);

    ESP_LOGI(TAG, "Sending sound command: %d, seq: %d", sound, seq);
    return i2c_master_send_command(&command, NULL, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t i2c_send_servo_command(servo_command_t servo, uint8_t angle)
{
    i2c_command_packet_t command;
    uint8_t seq = get_next_sequence_number();
    prepare_servo_command(&command, servo, angle, seq);

    ESP_LOGI(TAG, "Sending servo command: %d, angle: %d, seq: %d", servo, angle, seq);
    return i2c_master_send_command(&command, NULL, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t i2c_send_display_command(uint8_t line, const char *message)
{
    if (!message) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_command_packet_t command;
    uint8_t seq = get_next_sequence_number();
    prepare_display_command(&command, line, message, seq);

    ESP_LOGI(TAG, "Sending display command: line %d, message: %.20s, seq: %d", line, message, seq);
    return i2c_master_send_command(&command, NULL, I2C_MASTER_TIMEOUT_MS);
}

esp_err_t i2c_ping_slave(void)
{
    i2c_command_packet_t command;
    i2c_response_packet_t response;
    uint8_t seq = get_next_sequence_number();

    prepare_ping_command(&command, seq);

    ESP_LOGD(TAG, "Pinging slave device, seq: %d", seq);
    esp_err_t err = i2c_master_send_command(&command, &response, I2C_MASTER_TIMEOUT_MS);

    if (err == ESP_OK && response.status == 0x00) {
        ESP_LOGD(TAG, "Slave ping successful");
    } else {
        ESP_LOGW(TAG, "Slave ping failed or returned error status");
    }

    return err;
}

esp_err_t i2c_get_status(status_data_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_command_packet_t command;
    i2c_response_packet_t response;
    uint8_t seq = get_next_sequence_number();

    command.command_type = CMD_TYPE_STATUS;
    command.sequence_number = seq;
    command.data_length = 0;
    command.checksum = calculate_checksum((uint8_t *)&command, sizeof(i2c_command_packet_t) - 1);

    esp_err_t err = i2c_master_send_command(&command, &response, I2C_MASTER_TIMEOUT_MS);

    if (err == ESP_OK && response.status == 0x00) {
        if (response.data_length == sizeof(status_data_t)) {
            memcpy(status, response.data, sizeof(status_data_t));
            ESP_LOGD(TAG, "Status retrieved successfully");
        } else {
            ESP_LOGE(TAG, "Invalid status data length: %d", response.data_length);
            err = ESP_ERR_INVALID_SIZE;
        }
    }

    return err;
}

// OTA command functions
esp_err_t i2c_send_enter_maintenance_mode(void)
{
    i2c_command_packet_t command;
    i2c_response_packet_t response;
    uint8_t seq = get_next_sequence_number();

    prepare_enter_maintenance_command(&command, seq);

    ESP_LOGI(TAG, "Sending enter maintenance mode command, seq: %d", seq);
    esp_err_t err = i2c_master_send_command(&command, &response, I2C_MASTER_TIMEOUT_MS);

    if (err == ESP_OK && response.status == 0x00) {
        ESP_LOGI(TAG, "Main controller entered maintenance mode");
    } else {
        ESP_LOGW(TAG, "Failed to enter maintenance mode");
    }

    return err;
}

esp_err_t i2c_send_begin_ota(const char *url, const uint8_t *hash)
{
    if (!url) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_command_packet_t command;
    i2c_response_packet_t response;
    uint8_t seq = get_next_sequence_number();

    prepare_begin_ota_command(&command, url, hash, seq);

    ESP_LOGI(TAG, "Sending begin OTA command: %s, seq: %d", url, seq);
    esp_err_t err = i2c_master_send_command(&command, &response, I2C_MASTER_TIMEOUT_MS);

    if (err == ESP_OK && response.status == 0x00) {
        ESP_LOGI(TAG, "OTA update started on main controller");
    } else {
        ESP_LOGW(TAG, "Failed to start OTA update");
    }

    return err;
}

esp_err_t i2c_get_ota_status(ota_status_response_t *ota_status)
{
    if (!ota_status) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_command_packet_t command;
    i2c_response_packet_t response;
    uint8_t seq = get_next_sequence_number();

    prepare_get_ota_status_command(&command, seq);

    ESP_LOGD(TAG, "Getting OTA status, seq: %d", seq);
    esp_err_t err = i2c_master_send_command(&command, &response, I2C_MASTER_TIMEOUT_MS);

    if (err == ESP_OK && response.status == 0x00) {
        if (response.data_length == sizeof(ota_status_response_t)) {
            memcpy(ota_status, response.data, sizeof(ota_status_response_t));
            ESP_LOGD(TAG, "OTA status: %d, progress: %d%%, error: %d", ota_status->status,
                     ota_status->progress, ota_status->error_code);
        } else {
            ESP_LOGE(TAG, "Invalid OTA status data length: %d", response.data_length);
            err = ESP_ERR_INVALID_SIZE;
        }
    }

    return err;
}

esp_err_t i2c_get_version(char *version, size_t version_len)
{
    if (!version || version_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_command_packet_t command;
    i2c_response_packet_t response;
    uint8_t seq = get_next_sequence_number();

    prepare_get_version_command(&command, seq);

    ESP_LOGI(TAG, "Getting firmware version, seq: %d", seq);
    esp_err_t err = i2c_master_send_command(&command, &response, I2C_MASTER_TIMEOUT_MS);

    if (err == ESP_OK && response.status == 0x00) {
        if (response.data_length == sizeof(version_response_t)) {
            version_response_t *ver = (version_response_t *)response.data;
            strncpy(version, ver->version, version_len);
            version[version_len - 1] = '\0';
            ESP_LOGI(TAG, "Firmware version: %s", version);
        } else {
            ESP_LOGE(TAG, "Invalid version data length: %d", response.data_length);
            err = ESP_ERR_INVALID_SIZE;
        }
    }

    return err;
}

esp_err_t i2c_send_reboot(void)
{
    i2c_command_packet_t command;
    i2c_response_packet_t response;
    uint8_t seq = get_next_sequence_number();

    prepare_reboot_command(&command, seq);

    ESP_LOGW(TAG, "Sending reboot command to main controller, seq: %d", seq);
    esp_err_t err = i2c_master_send_command(&command, &response, I2C_MASTER_TIMEOUT_MS);

    if (err == ESP_OK && response.status == 0x00) {
        ESP_LOGI(TAG, "Reboot command accepted");
    } else {
        ESP_LOGW(TAG, "Failed to send reboot command");
    }

    return err;
}
