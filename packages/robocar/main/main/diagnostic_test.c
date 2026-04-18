#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config_idf.h"

static const char *TAG = "diagnostic";

// Test individual pin functionality
void test_gpio_pins(void)
{
    ESP_LOGI(TAG, "=== GPIO PIN DIAGNOSTIC TEST ===");

    // List of pins to test
    int test_pins[] = {RIGHT_PWMA_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN, LEFT_PWMB_PIN,
                       LEFT_IN1_PIN,   LEFT_IN2_PIN,  STBY_PIN};
    char *pin_names[] = {"RIGHT_PWMA", "RIGHT_IN1", "RIGHT_IN2", "LEFT_PWMB",
                         "LEFT_IN1",   "LEFT_IN2",  "STBY"};
    int num_pins = sizeof(test_pins) / sizeof(test_pins[0]);

    for (int i = 0; i < num_pins; i++) {
        int pin = test_pins[i];
        ESP_LOGI(TAG, "Testing pin %d (%s)", pin, pin_names[i]);

        // Configure as output
        gpio_config_t io_conf = {.pin_bit_mask = (1ULL << pin),
                                 .mode = GPIO_MODE_OUTPUT,
                                 .pull_up_en = GPIO_PULLUP_DISABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_DISABLE};

        esp_err_t result = gpio_config(&io_conf);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure pin %d: %s", pin, esp_err_to_name(result));
            continue;
        }

        // Test HIGH and LOW states
        gpio_set_level(pin, 1);
        ESP_LOGI(TAG, "Pin %d set HIGH", pin);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gpio_set_level(pin, 0);
        ESP_LOGI(TAG, "Pin %d set LOW", pin);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "GPIO pin test completed");
}

// Test PWM functionality
void test_pwm_channels(void)
{
    ESP_LOGI(TAG, "=== PWM DIAGNOSTIC TEST ===");

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t result = ledc_timer_config(&ledc_timer);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "PWM timer config failed: %s", esp_err_to_name(result));
        return;
    }
    ESP_LOGI(TAG, "PWM timer configured successfully");

    // Test right motor PWM
    ledc_channel_config_t right_channel = {.channel = PWM_RIGHT_CHANNEL,
                                           .duty = 0,
                                           .gpio_num = RIGHT_PWMA_PIN,
                                           .speed_mode = PWM_MODE,
                                           .hpoint = 0,
                                           .timer_sel = PWM_TIMER};

    result = ledc_channel_config(&right_channel);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Right PWM channel config failed: %s", esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "Right PWM channel (pin %d) configured", RIGHT_PWMA_PIN);

        // Test different duty cycles
        for (int duty = 0; duty <= 255; duty += 64) {
            uint32_t ledc_duty = (duty * ((1 << PWM_RESOLUTION) - 1)) / 255;
            ledc_set_duty(PWM_MODE, PWM_RIGHT_CHANNEL, ledc_duty);
            ledc_update_duty(PWM_MODE, PWM_RIGHT_CHANNEL);
            ESP_LOGI(TAG, "Right PWM duty: %d (LEDC: %lu)", duty, ledc_duty);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    // Test left motor PWM
    ledc_channel_config_t left_channel = {.channel = PWM_LEFT_CHANNEL,
                                          .duty = 0,
                                          .gpio_num = LEFT_PWMB_PIN,
                                          .speed_mode = PWM_MODE,
                                          .hpoint = 0,
                                          .timer_sel = PWM_TIMER};

    result = ledc_channel_config(&left_channel);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Left PWM channel config failed: %s", esp_err_to_name(result));
    } else {
        ESP_LOGI(TAG, "Left PWM channel (pin %d) configured", LEFT_PWMB_PIN);

        // Test different duty cycles
        for (int duty = 0; duty <= 255; duty += 64) {
            uint32_t ledc_duty = (duty * ((1 << PWM_RESOLUTION) - 1)) / 255;
            ledc_set_duty(PWM_MODE, PWM_LEFT_CHANNEL, ledc_duty);
            ledc_update_duty(PWM_MODE, PWM_LEFT_CHANNEL);
            ESP_LOGI(TAG, "Left PWM duty: %d (LEDC: %lu)", duty, ledc_duty);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    ESP_LOGI(TAG, "PWM test completed");
}

// Test motor driver enable/standby
void test_motor_driver_standby(void)
{
    ESP_LOGI(TAG, "=== MOTOR DRIVER STANDBY TEST ===");

    // Configure STBY pin
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << STBY_PIN),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};

    esp_err_t result = gpio_config(&io_conf);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "STBY pin config failed: %s", esp_err_to_name(result));
        return;
    }

    ESP_LOGI(TAG, "Testing STBY pin %d", STBY_PIN);

    // Test standby disable (motor driver off)
    gpio_set_level(STBY_PIN, 0);
    ESP_LOGI(TAG, "Motor driver DISABLED (STBY=LOW)");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Test standby enable (motor driver on)
    gpio_set_level(STBY_PIN, 1);
    ESP_LOGI(TAG, "Motor driver ENABLED (STBY=HIGH)");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Motor driver standby test completed");
}

// Simple motor test without complex logic
void test_basic_motor_control(void)
{
    ESP_LOGI(TAG, "=== BASIC MOTOR CONTROL TEST ===");

    // Initialize all pins
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << RIGHT_IN1_PIN) | (1ULL << RIGHT_IN2_PIN) |
                                             (1ULL << LEFT_IN1_PIN) | (1ULL << LEFT_IN2_PIN) |
                                             (1ULL << STBY_PIN),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    // Enable motor driver
    gpio_set_level(STBY_PIN, 1);
    ESP_LOGI(TAG, "Motor driver enabled");

    // Test right motor forward (without PWM)
    ESP_LOGI(TAG, "Testing RIGHT motor forward (no PWM)");
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Stop right motor
    gpio_set_level(RIGHT_IN1_PIN, 0);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    ESP_LOGI(TAG, "Right motor stopped");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Test left motor forward (without PWM)
    ESP_LOGI(TAG, "Testing LEFT motor forward (no PWM)");
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Stop left motor
    gpio_set_level(LEFT_IN1_PIN, 0);
    gpio_set_level(LEFT_IN2_PIN, 0);
    ESP_LOGI(TAG, "Left motor stopped");

    ESP_LOGI(TAG, "Basic motor control test completed");
}

void diagnostic_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting comprehensive diagnostic tests");

    while (1) {
        test_gpio_pins();
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        test_motor_driver_standby();
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        test_pwm_channels();
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        test_basic_motor_control();
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "=== DIAGNOSTIC CYCLE COMPLETE ===");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP-IDF RoboCar Diagnostic Tool");
    ESP_LOGI(TAG, "Pin configuration:");
    ESP_LOGI(TAG, "  RIGHT_PWMA: %d, RIGHT_IN1: %d, RIGHT_IN2: %d", RIGHT_PWMA_PIN, RIGHT_IN1_PIN,
             RIGHT_IN2_PIN);
    ESP_LOGI(TAG, "  LEFT_PWMB: %d, LEFT_IN1: %d, LEFT_IN2: %d", LEFT_PWMB_PIN, LEFT_IN1_PIN,
             LEFT_IN2_PIN);
    ESP_LOGI(TAG, "  STBY: %d", STBY_PIN);

    xTaskCreate(diagnostic_task, "diagnostic_task", 4096, NULL, 5, NULL);
}
