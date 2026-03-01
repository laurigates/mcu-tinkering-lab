#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_console.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_protocol.h"
#include "i2c_slave.h"
#include "pca9685.h"
#include "system_config.h"
#include "wifi_manager.h"

static const char *TAG = "idf-robocar";

// Command and state definitions now centralized in system_config.h

// Global variables (using centralized configuration)
system_state_t current_state = STATE_STOPPED;
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT = SYSTEM_COMMAND_TIMEOUT_MS;

// Action tracking for display
char last_action[ACTION_STRING_LENGTH] = "STARTUP";
char last_command_source[COMMAND_SOURCE_LENGTH] = "INIT";
unsigned long action_counter = 0;

// Buffer for serial communication
char command_buffer[MAX_COMMAND_LENGTH];
int buffer_pos = 0;

// Mutex protecting all global state above
static SemaphoreHandle_t g_state_mutex = NULL;

// Safe string copy macro: copies src into dst[size], always null-terminates
#define SAFE_STRCPY(dst, src, size)      \
    do {                                 \
        strncpy((dst), (src), (size)-1); \
        (dst)[(size)-1] = '\0';          \
    } while (0)

// PCA9685 and I2C variables
i2c_dev_t pca9685_dev;
bool pca9685_initialized = false;
int current_pan_angle = SERVO_DEFAULT_ANGLE;
int current_tilt_angle = SERVO_DEFAULT_ANGLE;

// OLED Display variables
esp_lcd_panel_handle_t oled_panel = NULL;
esp_lcd_panel_io_handle_t oled_io_handle = NULL;
bool oled_initialized = false;

// Function declarations
void set_leds(int red, int green, int blue);
void set_left_led(int red, int green, int blue);
void set_right_led(int red, int green, int blue);
void update_leds(void);
void set_pan_angle(int angle);
void set_tilt_angle(int angle);
void init_oled(void);
void update_oled_status(void);
void oled_show_startup(void);
void oled_show_command(const char *cmd);
void oled_show_state(const char *state);
void oled_draw_text(int x, int y, const char *text, bool clear_line);
void oled_show_claude_message(int line, const char *message);
void oled_show_action_debug(void);
void track_action(const char *action, const char *source);

// Initialize console for serial communication
void init_console(void)
{
    // Configure UART for console
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    // Tell VFS to use UART driver
    uart_vfs_dev_use_driver(UART_NUM_0);

    ESP_LOGI(TAG, "Console initialized at 115200 baud");
}

// Initialize I2C master bus
esp_err_t init_i2c(void)
{
    ESP_LOGI(TAG, "Initializing I2C master bus");

    // Initialize I2C device descriptor for PCA9685
    memset(&pca9685_dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(i2cdev_init());

    esp_err_t ret =
        pca9685_init_desc(&pca9685_dev, PCA9685_I2C_ADDR, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685 descriptor: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

// Initialize PCA9685 PWM controller
esp_err_t init_pca9685(void)
{
    if (!PCA9685_ENABLED) {
        ESP_LOGI(TAG, "PCA9685 disabled in configuration");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing PCA9685");

    esp_err_t ret = pca9685_init(&pca9685_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685: %s", esp_err_to_name(ret));
        pca9685_initialized = false;
        return ret;
    }

    // Set PWM frequency for LEDs initially
    ret = pca9685_set_pwm_frequency(&pca9685_dev, PCA9685_FREQ_LED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCA9685 frequency: %s", esp_err_to_name(ret));
        pca9685_initialized = false;
        return ret;
    }

    pca9685_initialized = true;
    ESP_LOGI(TAG, "PCA9685 initialized successfully at %dHz", PCA9685_FREQ_LED);

    // Initialize LEDs to red (stopped state)
    set_leds(255, 0, 0);

    // Set frequency for servos and initialize servo positions
    ret = pca9685_set_pwm_frequency(&pca9685_dev, PCA9685_FREQ_SERVO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo frequency: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PCA9685 frequency set to %dHz for servos", PCA9685_FREQ_SERVO);

    // Initialize servos to center position
    set_pan_angle(SERVO_DEFAULT_ANGLE);
    set_tilt_angle(SERVO_DEFAULT_ANGLE);

    return ESP_OK;
}

// Initialize OLED display
void init_oled(void)
{
    if (!OLED_ENABLED) {
        ESP_LOGI(TAG, "OLED disabled in configuration");
        return;
    }

    ESP_LOGI(TAG, "Initializing OLED display using native ESP-IDF LCD driver");

    // Configure I2C master
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,  // 400 KHz
    };

    esp_err_t ret = i2c_param_config(I2C_NUM_1, &i2c_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C for OLED: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver for OLED: %s", esp_err_to_name(ret));
        return;
    }

    // Create LCD panel I/O handle
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,
    };

    ret =
        esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_1, &io_config, &oled_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LCD panel I/O: %s", esp_err_to_name(ret));
        return;
    }

    // Create LCD panel handle
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = OLED_RST_PIN,
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,  // Monochrome display
    };

    ret = esp_lcd_new_panel_ssd1306(oled_io_handle, &panel_config, &oled_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SSD1306 panel: %s", esp_err_to_name(ret));
        return;
    }

    // Reset and initialize the panel
    esp_lcd_panel_reset(oled_panel);
    esp_lcd_panel_init(oled_panel);
    esp_lcd_panel_disp_on_off(oled_panel, true);

    oled_initialized = true;
    ESP_LOGI(TAG,
             "OLED initialized successfully using native ESP-IDF driver on SDA=%d, SCL=%d, RST=%d",
             OLED_SDA_PIN, OLED_SCL_PIN, OLED_RST_PIN);

    // Clear display and show startup message
    uint8_t clear_buffer[OLED_WIDTH * OLED_HEIGHT / 8] = {0};
    esp_lcd_panel_draw_bitmap(oled_panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, clear_buffer);
    oled_show_startup();
}

void init_gpio(void)
{
    // Configure motor control pins as outputs
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << RIGHT_IN1_PIN) | (1ULL << RIGHT_IN2_PIN) |
                                             (1ULL << LEFT_IN1_PIN) | (1ULL << LEFT_IN2_PIN) |
                                             (1ULL << STBY_PIN) | (1ULL << PIEZO_PIN),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set initial pin states
    ESP_ERROR_CHECK(gpio_set_level(RIGHT_IN1_PIN, 0));
    ESP_ERROR_CHECK(gpio_set_level(RIGHT_IN2_PIN, 0));
    ESP_ERROR_CHECK(gpio_set_level(LEFT_IN1_PIN, 0));
    ESP_ERROR_CHECK(gpio_set_level(LEFT_IN2_PIN, 0));
    ESP_ERROR_CHECK(gpio_set_level(STBY_PIN, 1));   // Enable motor driver
    ESP_ERROR_CHECK(gpio_set_level(PIEZO_PIN, 0));  // Piezo off initially
}

// Initialize PWM for motor control
void init_pwm(void)
{
    // Timer configuration
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Channel configuration for right motor
    ledc_channel_config_t right_motor = {.channel = PWM_RIGHT_CHANNEL,
                                         .duty = 0,
                                         .gpio_num = RIGHT_PWMA_PIN,
                                         .speed_mode = PWM_MODE,
                                         .hpoint = 0,
                                         .timer_sel = PWM_TIMER};
    ESP_ERROR_CHECK(ledc_channel_config(&right_motor));

    // Channel configuration for left motor
    ledc_channel_config_t left_motor = {.channel = PWM_LEFT_CHANNEL,
                                        .duty = 0,
                                        .gpio_num = LEFT_PWMB_PIN,
                                        .speed_mode = PWM_MODE,
                                        .hpoint = 0,
                                        .timer_sel = PWM_TIMER};
    ESP_ERROR_CHECK(ledc_channel_config(&left_motor));
}

// Set motor speed (0-255)
void set_motor_speed(int channel, int speed)
{
    // Convert 8-bit speed (0-255) to duty cycle based on resolution
    uint32_t duty = (speed * ((1 << PWM_RESOLUTION) - 1)) / 255;
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

// Motor control functions
void move_forward(int speed)
{
    // Standard TB6612FNG logic: RIGHT motor forward
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);

    // Standard TB6612FNG logic: LEFT motor forward
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    set_motor_speed(PWM_LEFT_CHANNEL, speed);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_state = STATE_FORWARD;
        xSemaphoreGive(g_state_mutex);
    }
    update_leds();

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "FWD SPD:%d", speed);
    track_action(action_text, "MOTOR");

    ESP_LOGI(TAG, "Moving forward at speed %d", speed);
}

void move_backward(int speed)
{
    // Standard TB6612FNG logic: RIGHT motor backward
    gpio_set_level(RIGHT_IN1_PIN, 0);
    gpio_set_level(RIGHT_IN2_PIN, 1);

    // Standard TB6612FNG logic: LEFT motor backward
    gpio_set_level(LEFT_IN1_PIN, 0);
    gpio_set_level(LEFT_IN2_PIN, 1);

    set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    set_motor_speed(PWM_LEFT_CHANNEL, speed);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_state = STATE_BACKWARD;
        xSemaphoreGive(g_state_mutex);
    }
    update_leds();

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "BACK SPD:%d", speed);
    track_action(action_text, "MOTOR");

    ESP_LOGI(TAG, "Moving backward at speed %d", speed);
}

void turn_left(int speed)
{
    // Standard TB6612FNG logic: RIGHT motor full speed forward
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);

    // Standard TB6612FNG logic: LEFT motor slower forward
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    set_motor_speed(PWM_LEFT_CHANNEL, speed / 2);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_state = STATE_LEFT;
        xSemaphoreGive(g_state_mutex);
    }
    update_leds();

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "LEFT SPD:%d", speed);
    track_action(action_text, "MOTOR");

    ESP_LOGI(TAG, "Turning left at speed %d", speed);
}

void turn_right(int speed)
{
    // Standard TB6612FNG logic: RIGHT motor slower forward
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);

    // Standard TB6612FNG logic: LEFT motor full speed forward
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    set_motor_speed(PWM_RIGHT_CHANNEL, speed / 2);
    set_motor_speed(PWM_LEFT_CHANNEL, speed);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_state = STATE_RIGHT;
        xSemaphoreGive(g_state_mutex);
    }
    update_leds();

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "RIGHT SPD:%d", speed);
    track_action(action_text, "MOTOR");

    ESP_LOGI(TAG, "Turning right at speed %d", speed);
}

void rotate_cw(int speed)
{
    // Standard TB6612FNG logic: RIGHT motor backward
    gpio_set_level(RIGHT_IN1_PIN, 0);
    gpio_set_level(RIGHT_IN2_PIN, 1);

    // Standard TB6612FNG logic: LEFT motor forward
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    set_motor_speed(PWM_LEFT_CHANNEL, speed);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_state = STATE_ROTATE_CW;
        xSemaphoreGive(g_state_mutex);
    }
    update_leds();

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "ROT_CW SPD:%d", speed);
    track_action(action_text, "MOTOR");

    ESP_LOGI(TAG, "Rotating clockwise at speed %d", speed);
}

void rotate_ccw(int speed)
{
    // Standard TB6612FNG logic: RIGHT motor forward
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);

    // Standard TB6612FNG logic: LEFT motor backward
    gpio_set_level(LEFT_IN1_PIN, 0);
    gpio_set_level(LEFT_IN2_PIN, 1);

    set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    set_motor_speed(PWM_LEFT_CHANNEL, speed);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_state = STATE_ROTATE_CCW;
        xSemaphoreGive(g_state_mutex);
    }
    update_leds();

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "ROT_CCW SPD:%d", speed);
    track_action(action_text, "MOTOR");

    ESP_LOGI(TAG, "Rotating counter-clockwise at speed %d", speed);
}

void stop_motors(void)
{
    gpio_set_level(RIGHT_IN1_PIN, 0);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    gpio_set_level(LEFT_IN1_PIN, 0);
    gpio_set_level(LEFT_IN2_PIN, 0);

    set_motor_speed(PWM_RIGHT_CHANNEL, 0);
    set_motor_speed(PWM_LEFT_CHANNEL, 0);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_state = STATE_STOPPED;
        xSemaphoreGive(g_state_mutex);
    }
    ESP_LOGI(TAG, "Motors stopped");

    // Update LEDs and display to reflect stopped state
    update_leds();
    track_action("STOP", "MOTOR");
}

// Get current time in milliseconds
unsigned long millis(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// LED Control Functions
void set_leds(int red, int green, int blue)
{
    set_left_led(red, green, blue);
    set_right_led(red, green, blue);
}

void set_left_led(int red, int green, int blue)
{
    if (!pca9685_initialized)
        return;

    // Scale 0-255 to 0-4095 and invert for common anode LEDs
    uint16_t r_pwm = 4095 - ((red * 4095) / 255);
    uint16_t g_pwm = 4095 - ((green * 4095) / 255);
    uint16_t b_pwm = 4095 - ((blue * 4095) / 255);

    pca9685_set_pwm_value(&pca9685_dev, LED_LEFT_R_CHANNEL, r_pwm);
    pca9685_set_pwm_value(&pca9685_dev, LED_LEFT_G_CHANNEL, g_pwm);
    pca9685_set_pwm_value(&pca9685_dev, LED_LEFT_B_CHANNEL, b_pwm);
}

void set_right_led(int red, int green, int blue)
{
    if (!pca9685_initialized)
        return;

    // Scale 0-255 to 0-4095 and invert for common anode LEDs
    uint16_t r_pwm = 4095 - ((red * 4095) / 255);
    uint16_t g_pwm = 4095 - ((green * 4095) / 255);
    uint16_t b_pwm = 4095 - ((blue * 4095) / 255);

    pca9685_set_pwm_value(&pca9685_dev, LED_RIGHT_R_CHANNEL, r_pwm);
    pca9685_set_pwm_value(&pca9685_dev, LED_RIGHT_G_CHANNEL, g_pwm);
    pca9685_set_pwm_value(&pca9685_dev, LED_RIGHT_B_CHANNEL, b_pwm);
}

void update_leds(void)
{
    if (!pca9685_initialized)
        return;

    // Set RGB LED colors based on current state
    switch (current_state) {
        case STATE_STOPPED:
            set_leds(255, 0, 0);  // Red
            break;
        case STATE_FORWARD:
            set_leds(0, 255, 0);  // Green
            break;
        case STATE_BACKWARD:
            set_leds(255, 0, 0);  // Red
            break;
        case STATE_LEFT:
            set_left_led(255, 165, 0);  // Amber
            set_right_led(0, 255, 0);   // Green
            break;
        case STATE_RIGHT:
            set_left_led(0, 255, 0);     // Green
            set_right_led(255, 165, 0);  // Amber
            break;
        case STATE_ROTATE_CW:
            set_left_led(0, 0, 255);     // Blue
            set_right_led(255, 165, 0);  // Amber
            break;
        case STATE_ROTATE_CCW:
            set_left_led(255, 165, 0);  // Amber
            set_right_led(0, 0, 255);   // Blue
            break;
    }
}

// Servo Control Functions
void set_servo_angle(int channel, int angle)
{
    if (!pca9685_initialized)
        return;

    // Constrain angle to limits
    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    // Map angle (0-180) to pulse length (SERVO_MIN_PULSE-SERVO_MAX_PULSE)
    uint16_t pulse_length =
        SERVO_MIN_PULSE + ((angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / SERVO_MAX_ANGLE);

    esp_err_t ret = pca9685_set_pwm_value(&pca9685_dev, channel, pulse_length);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo angle: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Servo channel %d set to angle: %d (pulse: %d)", channel, angle, pulse_length);
}

void set_pan_angle(int angle)
{
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_pan_angle = angle;
        xSemaphoreGive(g_state_mutex);
    }
    set_servo_angle(SERVO_PAN_CHANNEL, angle);

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "PAN:%d", angle);
    track_action(action_text, "SERVO");
}

void set_tilt_angle(int angle)
{
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        current_tilt_angle = angle;
        xSemaphoreGive(g_state_mutex);
    }
    set_servo_angle(SERVO_TILT_CHANNEL, angle);

    char action_text[32];
    snprintf(action_text, sizeof(action_text), "TILT:%d", angle);
    track_action(action_text, "SERVO");
}

// Simple 8x8 bitmap font for basic characters (A-Z, 0-9, space, colon)
static const uint8_t font_8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // Space (32)
    {0x18, 0x3C, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00},  // A (65)
    {0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00},  // B
    {0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00},  // C
    {0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00},  // D
    {0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x7E, 0x00},  // E
    {0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x60, 0x00},  // F
    {0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00},  // G
    {0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00},  // H
    {0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00},  // I
    {0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x6C, 0x38, 0x00},  // J
    {0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x00},  // K
    {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00},  // L
    {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63, 0x00},  // M
    {0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x00},  // N
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00},  // O
    {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00},  // P
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x0E, 0x00},  // Q
    {0x7C, 0x66, 0x66, 0x7C, 0x78, 0x6C, 0x66, 0x00},  // R
    {0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00},  // S
    {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00},  // T
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00},  // U
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00},  // V
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00},  // W
    {0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00},  // X
    {0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00},  // Y
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00},  // Z
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // ... (skipping some)
    {0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00},  // 0 (48)
    {0x18, 0x18, 0x38, 0x18, 0x18, 0x18, 0x7E, 0x00},  // 1
    {0x3C, 0x66, 0x06, 0x0C, 0x30, 0x60, 0x7E, 0x00},  // 2
    {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00},  // 3
    {0x06, 0x0E, 0x1E, 0x66, 0x7F, 0x06, 0x06, 0x00},  // 4
    {0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00},  // 5
    {0x3C, 0x66, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00},  // 6
    {0x7E, 0x66, 0x0C, 0x18, 0x18, 0x18, 0x18, 0x00},  // 7
    {0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00},  // 8
    {0x3C, 0x66, 0x66, 0x3E, 0x06, 0x66, 0x3C, 0x00},  // 9
    {0x00, 0x00, 0x18, 0x00, 0x00, 0x18, 0x00, 0x00},  // : (58)
};

// Get font bitmap for character
static const uint8_t *get_char_bitmap(char c)
{
    if (c == ' ')
        return font_8x8[0];
    if (c >= 'A' && c <= 'Z')
        return font_8x8[c - 'A' + 1];
    if (c >= 'a' && c <= 'z')
        return font_8x8[c - 'a' + 1];  // Use uppercase
    if (c >= '0' && c <= '9')
        return font_8x8[c - '0' + 27];
    if (c == ':')
        return font_8x8[38];
    return font_8x8[0];  // Default to space
}

// Draw text at specified position
void oled_draw_text(int x, int y, const char *text, bool clear_line)
{
    if (!oled_initialized || !text)
        return;

    static uint8_t display_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

    // Clear the line if requested
    if (clear_line) {
        int line_start = (y / 8) * OLED_WIDTH;
        memset(&display_buffer[line_start], 0, OLED_WIDTH);
    }

    int char_x = x;
    for (int i = 0; text[i] && char_x < OLED_WIDTH - 8; i++) {
        const uint8_t *bitmap = get_char_bitmap(text[i]);

        // Draw character bitmap - font data is row-based, 8 bytes per character
        for (int row = 0; row < 8; row++) {
            if (y + row < OLED_HEIGHT) {
                uint8_t font_row = bitmap[row];

                // Extract each bit from the row and place it in the correct column
                for (int col = 0; col < 8; col++) {
                    int pixel_x = char_x + col;
                    int pixel_y = y + row;

                    if (pixel_x < OLED_WIDTH && pixel_y < OLED_HEIGHT) {
                        int byte_index = (pixel_y / 8) * OLED_WIDTH + pixel_x;
                        int bit_offset = pixel_y % 8;

                        if (byte_index < sizeof(display_buffer)) {
                            if (font_row & (1 << (7 - col))) {
                                display_buffer[byte_index] |= (1 << bit_offset);
                            } else {
                                display_buffer[byte_index] &= ~(1 << bit_offset);
                            }
                        }
                    }
                }
            }
        }
        char_x += 8;  // Move to next character position
    }

    // Update the display
    esp_lcd_panel_draw_bitmap(oled_panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, display_buffer);
}

// OLED Display Functions
void oled_show_startup(void)
{
    if (!oled_initialized)
        return;

    oled_draw_text(0, 0, "ROBOCAR ESP-IDF", true);
    oled_draw_text(0, 8, "INITIALIZING...", true);
    oled_draw_text(0, 16, "STARTING DEBUG", true);
    oled_draw_text(0, 24, "DISPLAY MODE", true);
    oled_draw_text(0, 32, "READY FOR CMDS", true);

    // Switch to debug display after a short delay
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    track_action("SYSTEM_READY", "INIT");
}

void oled_show_command(const char *cmd)
{
    if (!oled_initialized)
        return;

    char cmd_text[32];
    snprintf(cmd_text, sizeof(cmd_text), "CMD: %s", cmd);
    oled_draw_text(0, 48, cmd_text, true);
}

void oled_show_state(const char *state)
{
    if (!oled_initialized)
        return;

    char state_text[32];
    snprintf(state_text, sizeof(state_text), "STATE: %s", state);
    oled_draw_text(0, 56, state_text, true);
}

void update_oled_status(void)
{
    if (!oled_initialized)
        return;

    // Show current state
    const char *state_name;
    switch (current_state) {
        case STATE_STOPPED:
            state_name = "STOPPED";
            break;
        case STATE_FORWARD:
            state_name = "FORWARD";
            break;
        case STATE_BACKWARD:
            state_name = "BACKWARD";
            break;
        case STATE_LEFT:
            state_name = "LEFT";
            break;
        case STATE_RIGHT:
            state_name = "RIGHT";
            break;
        case STATE_ROTATE_CW:
            state_name = "ROTATE CW";
            break;
        case STATE_ROTATE_CCW:
            state_name = "ROTATE CCW";
            break;
        default:
            state_name = "UNKNOWN";
            break;
    }

    char status_text[32];
    snprintf(status_text, sizeof(status_text), "STATE: %s", state_name);
    oled_draw_text(0, 16, status_text, true);

    // Show servo positions (protected read)
    int pan, tilt;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        pan = current_pan_angle;
        tilt = current_tilt_angle;
        xSemaphoreGive(g_state_mutex);
    } else {
        pan = current_pan_angle;
        tilt = current_tilt_angle;
    }
    snprintf(status_text, sizeof(status_text), "PAN:%d TILT:%d", pan, tilt);
    oled_draw_text(0, 40, status_text, true);
}

void oled_show_action_debug(void)
{
    if (!oled_initialized)
        return;

    // Line 0: System status with action counter
    char line_text[17];  // 16 chars + null terminator for display width
    snprintf(line_text, sizeof(line_text), "ROBO #%lu", action_counter);
    oled_draw_text(0, 0, line_text, true);

    // Line 1: Current state with command source (truncated to fit)
    const char *state_name;
    switch (current_state) {
        case STATE_STOPPED:
            state_name = "STOP";
            break;
        case STATE_FORWARD:
            state_name = "FWD";
            break;
        case STATE_BACKWARD:
            state_name = "BACK";
            break;
        case STATE_LEFT:
            state_name = "LEFT";
            break;
        case STATE_RIGHT:
            state_name = "RGHT";
            break;
        case STATE_ROTATE_CW:
            state_name = "CW";
            break;
        case STATE_ROTATE_CCW:
            state_name = "CCW";
            break;
        default:
            state_name = "UNK";
            break;
    }
    // Truncate command source to 4 chars to fit in 16 char display
    char short_source[5];
    SAFE_STRCPY(short_source, last_command_source, sizeof(short_source));
    snprintf(line_text, sizeof(line_text), "%s [%s]", state_name, short_source);
    oled_draw_text(0, 8, line_text, true);

    // Line 2: Last action taken (truncated to fit)
    char short_action[12];
    SAFE_STRCPY(short_action, last_action, sizeof(short_action));
    snprintf(line_text, sizeof(line_text), "ACT:%s", short_action);
    oled_draw_text(0, 16, line_text, true);

    // Line 3: Servo positions (protected read)
    int pan_disp, tilt_disp;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        pan_disp = current_pan_angle;
        tilt_disp = current_tilt_angle;
        xSemaphoreGive(g_state_mutex);
    } else {
        pan_disp = current_pan_angle;
        tilt_disp = current_tilt_angle;
    }
    snprintf(line_text, sizeof(line_text), "PAN:%d TLT:%d", pan_disp, tilt_disp);
    oled_draw_text(0, 24, line_text, true);

    // Line 4: Time since last command
    unsigned long time_since_cmd = millis() - last_command_time;
    if (time_since_cmd < 10000) {  // Show if less than 10 seconds
        snprintf(line_text, sizeof(line_text), "CMD:%lums ago", time_since_cmd);
    } else {
        snprintf(line_text, sizeof(line_text), "CMD:>10s ago");
    }
    oled_draw_text(0, 32, line_text, true);

    // Line 5-7: Reserved for dynamic messages or status
    // These can be used by oled_show_claude_message or other functions
}

void track_action(const char *action, const char *source)
{
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        action_counter++;
        SAFE_STRCPY(last_action, action, sizeof(last_action));
        SAFE_STRCPY(last_command_source, source, sizeof(last_command_source));
        xSemaphoreGive(g_state_mutex);
    } else {
        ESP_LOGE(TAG, "track_action: failed to acquire state mutex");
    }

    // Update display with new action (read-only access to last_action/last_command_source is safe
    // after mutex release)
    oled_show_action_debug();

    ESP_LOGI(TAG, "Action: %s from %s", action, source);
}

void oled_show_claude_message(int line, const char *message)
{
    if (!oled_initialized || !message)
        return;

    // Calculate Y position based on line number (8 pixels per line)
    int y_pos = line * 8;

    // Ensure line is within display bounds (64 pixels height = 8 lines max)
    if (y_pos < 0 || y_pos >= 64) {
        ESP_LOGW(TAG, "Invalid display line: %d (0-7 valid)", line);
        return;
    }

    // Display the message with truncation if too long
    char display_msg[17];  // 16 chars + null terminator (128px / 8px per char)
    strncpy(display_msg, message, sizeof(display_msg) - 1);
    display_msg[sizeof(display_msg) - 1] = '\0';

    oled_draw_text(0, y_pos, display_msg, true);
    ESP_LOGI(TAG, "Claude message displayed on line %d: %s", line, display_msg);
}

// Basic sound generation using GPIO toggle
void play_sound(int frequency, int duration_ms)
{
    if (frequency <= 0 || duration_ms <= 0)
        return;

    unsigned long period_us = 1000000 / frequency;  // Period in microseconds
    unsigned long half_period_us = period_us / 2;
    unsigned long end_time = millis() + duration_ms;

    ESP_LOGI(TAG, "Playing sound: %dHz for %dms", frequency, duration_ms);

    while (millis() < end_time) {
        gpio_set_level(PIEZO_PIN, 1);
        esp_rom_delay_us(half_period_us);
        gpio_set_level(PIEZO_PIN, 0);
        esp_rom_delay_us(half_period_us);
        vTaskDelay(pdMS_TO_TICKS(1));  // yield to prevent task starvation
    }

    gpio_set_level(PIEZO_PIN, 0);  // Ensure piezo is off
}

// Sound effect functions
void sound_beep(void)
{
    track_action("BEEP", "SOUND");
    play_sound(1000, 200);  // 1kHz for 200ms
}

void sound_melody(void)
{
    track_action("MELODY", "SOUND");
    // Simple melody: C, E, G, C
    play_sound(262, 200);  // C4
    vTaskDelay(50 / portTICK_PERIOD_MS);
    play_sound(330, 200);  // E4
    vTaskDelay(50 / portTICK_PERIOD_MS);
    play_sound(392, 200);  // G4
    vTaskDelay(50 / portTICK_PERIOD_MS);
    play_sound(523, 400);  // C5
}

void sound_alert(void)
{
    track_action("ALERT", "SOUND");
    // Alert sound: alternating high-low
    for (int i = 0; i < 5; i++) {
        play_sound(1500, 100);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        play_sound(800, 100);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// Print help commands
void print_help_commands(void)
{
    printf("RoboCar Available Commands:\n");
    printf("--------------------------\n");
    printf("Movement Commands:\n");
    printf("  F - Move Forward\n");
    printf("  B - Move Backward\n");
    printf("  L - Turn Left\n");
    printf("  R - Turn Right\n");
    printf("  C - Rotate Clockwise\n");
    printf("  W - Rotate Counter-Clockwise\n");
    printf("  S - Stop\n");
    printf("Sound Commands:\n");
    printf("  SB - Sound Beep\n");
    printf("  SM - Sound Melody\n");
    printf("  SA - Sound Alert\n");
    printf("Servo Commands:\n");
    printf("  PAN:angle - Set Pan Servo Angle (0-180, e.g., PAN:90)\n");
    printf("  TILT:angle - Set Tilt Servo Angle (0-180, e.g., TILT:45)\n");
    printf("Display Commands:\n");
    printf("  DISP:line:message - Show message on OLED line (e.g., DISP:0:Hello)\n");
    printf("Utility Commands:\n");
    printf("  HELP - Display this help information\n");
    printf("--------------------------\n");
}

// Process received commands
void process_command(const char *command)
{
    ESP_LOGI(TAG, "Command received: %s", command);

    // Update command timestamp and reset timeout (protected)
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        last_command_time = millis();
        xSemaphoreGive(g_state_mutex);
    }

    // Track serial command reception
    char action_text[32];
    snprintf(action_text, sizeof(action_text), "SERIAL:%s", command);
    track_action(action_text, "UART");

    // Process movement commands
    if (strcmp(command, CMD_FORWARD) == 0) {
        move_forward(DEFAULT_SPEED);
    } else if (strcmp(command, CMD_BACKWARD) == 0) {
        move_backward(DEFAULT_SPEED);
    } else if (strcmp(command, CMD_LEFT) == 0) {
        turn_left(DEFAULT_SPEED);
    } else if (strcmp(command, CMD_RIGHT) == 0) {
        turn_right(DEFAULT_SPEED);
    } else if (strcmp(command, CMD_ROTATE_CW) == 0) {
        rotate_cw(DEFAULT_SPEED);
    } else if (strcmp(command, CMD_ROTATE_CCW) == 0) {
        rotate_ccw(DEFAULT_SPEED);
    } else if (strcmp(command, CMD_STOP) == 0) {
        stop_motors();
    }
    // Process sound commands
    else if (strcmp(command, CMD_SOUND_BEEP) == 0) {
        sound_beep();
    } else if (strcmp(command, CMD_SOUND_MELODY) == 0) {
        sound_melody();
    } else if (strcmp(command, CMD_SOUND_ALERT) == 0) {
        sound_alert();
    }
    // Process servo commands
    else if (strncmp(command, CMD_SERVO_PAN, strlen(CMD_SERVO_PAN)) == 0) {
        const char *angle_str = strchr(command, ':');
        if (angle_str != NULL) {
            angle_str++;  // Skip the colon
            int angle = atoi(angle_str);
            set_pan_angle(angle);
            update_oled_status();
        }
    } else if (strncmp(command, CMD_SERVO_TILT, strlen(CMD_SERVO_TILT)) == 0) {
        const char *angle_str = strchr(command, ':');
        if (angle_str != NULL) {
            angle_str++;  // Skip the colon
            int angle = atoi(angle_str);
            set_tilt_angle(angle);
            update_oled_status();
        }
    }
    // Process display commands
    else if (strncmp(command, CMD_DISPLAY, strlen(CMD_DISPLAY)) == 0) {
        // Format: DISP:line:message — work on a local copy to avoid modifying input buffer
        char disp_buf[MAX_COMMAND_LENGTH];
        SAFE_STRCPY(disp_buf, command, sizeof(disp_buf));
        char *line_str = strchr(disp_buf, ':');
        if (line_str != NULL) {
            line_str++;  // Skip first colon
            char *message_str = strchr(line_str, ':');
            if (message_str != NULL) {
                *message_str = '\0';  // Null terminate line number string in local copy
                message_str++;        // Move to message
                int line = atoi(line_str);
                oled_show_claude_message(line, message_str);
                ESP_LOGI(TAG, "Display message on line %d: %s", line, message_str);
            }
        }
    }
    // Process utility commands
    else if (strcmp(command, CMD_HELP) == 0) {
        print_help_commands();
    } else {
        ESP_LOGW(TAG, "Unknown command: %s. Type HELP for options.", command);
    }
}

// Serial command processing task
void command_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting command processing task");

    // Initialize command timestamp
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
        last_command_time = millis();
        xSemaphoreGive(g_state_mutex);
    }

    while (1) {
        // Check for command timeout (protected read of shared state)
        bool timed_out = false;
        if (g_state_mutex &&
            xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
            timed_out = (current_state != STATE_STOPPED) &&
                        ((millis() - last_command_time) > COMMAND_TIMEOUT);
            xSemaphoreGive(g_state_mutex);
        }
        if (timed_out) {
            ESP_LOGW(TAG, "Command timeout - stopping motors");
            stop_motors();
        }

        // Read from UART (USB serial) — 100ms timeout reduces busy-waiting
        uint8_t data[1];
        int len = uart_read_bytes(UART_NUM_0, data, 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            char c = (char)data[0];

            // Echo the character back for user feedback
            printf("%c", c);
            fflush(stdout);

            if (c == '\n' || c == '\r') {
                if (buffer_pos > 0) {
                    command_buffer[buffer_pos] = '\0';

                    // Convert to uppercase for consistency
                    for (int i = 0; i < buffer_pos; i++) {
                        command_buffer[i] = toupper(command_buffer[i]);
                    }

                    printf("\n");  // Add newline for clean output

                    // Process the command
                    process_command(command_buffer);

                    // Reset buffer (protected)
                    if (g_state_mutex &&
                        xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                        buffer_pos = 0;
                        xSemaphoreGive(g_state_mutex);
                    }
                }
            } else if (c >= 32 && c <= 126) {
                // Only accept printable characters — protect buffer_pos increment
                if (g_state_mutex &&
                    xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE) {
                    if (buffer_pos < MAX_COMMAND_LENGTH - 1) {
                        command_buffer[buffer_pos++] = c;
                    }
                    xSemaphoreGive(g_state_mutex);
                }
            }
        }

        // Small delay to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// I2C command processing functions
void process_i2c_movement_command(movement_command_t movement, uint8_t speed)
{
    ESP_LOGI(TAG, "Processing I2C movement command: %d, speed: %d", movement, speed);

    // Track I2C command reception
    char action_text[32];
    snprintf(action_text, sizeof(action_text), "I2C_CMD:%d", movement);
    track_action(action_text, "I2C");

    switch (movement) {
        case MOVE_FORWARD:
            move_forward(speed);
            break;
        case MOVE_BACKWARD:
            move_backward(speed);
            break;
        case MOVE_LEFT:
            turn_left(speed);
            break;
        case MOVE_RIGHT:
            turn_right(speed);
            break;
        case MOVE_ROTATE_CW:
            rotate_cw(speed);
            break;
        case MOVE_ROTATE_CCW:
            rotate_ccw(speed);
            break;
        case MOVE_STOP:
        default:
            stop_motors();
            break;
    }
}

void process_i2c_sound_command(sound_command_t sound)
{
    ESP_LOGI(TAG, "Processing I2C sound command: %d", sound);

    switch (sound) {
        case SOUND_BEEP:
            sound_beep();
            break;
        case SOUND_MELODY:
            sound_melody();
            break;
        case SOUND_ALERT:
            sound_alert();
            break;
        default:
            ESP_LOGW(TAG, "Unknown sound command: %d", sound);
            break;
    }
}

void process_i2c_servo_command(servo_command_t servo, uint8_t angle)
{
    ESP_LOGI(TAG, "Processing I2C servo command: %d, angle: %d", servo, angle);

    switch (servo) {
        case SERVO_PAN:
            set_pan_angle(angle);
            break;
        case SERVO_TILT:
            set_tilt_angle(angle);
            break;
        default:
            ESP_LOGW(TAG, "Unknown servo command: %d", servo);
            break;
    }

    update_oled_status();
}

void process_i2c_display_command(uint8_t line, const char *message)
{
    ESP_LOGI(TAG, "Processing I2C display command: line %d, message: %s", line, message);
    oled_show_claude_message(line, message);
}

// Forward declarations for main controller init functions
static esp_err_t init_communication_interfaces(void);
static esp_err_t init_hardware_controllers(void);
static esp_err_t init_display_and_feedback(void);
static esp_err_t init_inter_board_communication(void);
static void finalize_system_startup(void);

void app_main(void)
{
    ESP_LOGI(TAG, "RoboCar ESP-IDF starting up");

    // Phase 1: Communication interfaces (console, WiFi)
    if (init_communication_interfaces() != ESP_OK) {
        ESP_LOGE(TAG, "Communication initialization failed");
        return;
    }

    // Phase 2: Hardware controllers (GPIO, PWM, I2C, PCA9685)
    if (init_hardware_controllers() != ESP_OK) {
        ESP_LOGE(TAG, "Hardware initialization failed");
        return;
    }

    // Phase 3: Display and feedback systems
    if (init_display_and_feedback() != ESP_OK) {
        ESP_LOGE(TAG, "Display initialization failed");
        return;
    }

    // Phase 4: Inter-board communication (I2C slave)
    if (init_inter_board_communication() != ESP_OK) {
        ESP_LOGE(TAG, "Inter-board communication initialization failed");
        return;
    }

    // Phase 5: System ready
    finalize_system_startup();

    ESP_LOGI(TAG, "Command processing task started. Ready for commands!");
}

static esp_err_t init_communication_interfaces(void)
{
    // Initialize console first for proper serial communication
    init_console();

    // Initialize WiFi (requires NVS)
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (wifi_manager_init() == ESP_OK) {
        ESP_LOGI(TAG, "WiFi initialized successfully");

        // Start connection (will attempt to connect with saved credentials)
        if (wifi_manager_connect("", "") == ESP_OK) {
            ESP_LOGI(TAG, "Attempting to connect with saved credentials...");
        } else {
            ESP_LOGW(TAG, "No saved credentials found, WiFi ready for configuration");
        }
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to initialize WiFi");
        return ESP_FAIL;
    }
}

static esp_err_t init_hardware_controllers(void)
{
    ESP_LOGI(TAG, "Pin configuration:");
    ESP_LOGI(TAG, "  RIGHT_PWMA: %d, RIGHT_IN1: %d, RIGHT_IN2: %d", RIGHT_PWMA_PIN, RIGHT_IN1_PIN,
             RIGHT_IN2_PIN);
    ESP_LOGI(TAG, "  LEFT_PWMB: %d, LEFT_IN1: %d, LEFT_IN2: %d", LEFT_PWMB_PIN, LEFT_IN1_PIN,
             LEFT_IN2_PIN);
    ESP_LOGI(TAG, "  STBY: %d", STBY_PIN);

    // Initialize hardware
    init_gpio();
    init_pwm();

    // Initialize I2C and PCA9685
    if (init_i2c() == ESP_OK) {
        init_pca9685();
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "I2C initialization failed");
        return ESP_FAIL;
    }
}

static esp_err_t init_display_and_feedback(void)
{
    // Initialize OLED display
    init_oled();

    // Display is not critical for operation, so always return OK
    return ESP_OK;
}

static esp_err_t init_inter_board_communication(void)
{
    ESP_LOGI(TAG, "Initializing I2C slave for ESP32-CAM communication...");
    if (i2c_slave_init() == ESP_OK) {
        i2c_slave_start_task();
        ESP_LOGI(TAG, "I2C slave initialized and task started");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to initialize I2C slave");
        return ESP_FAIL;
    }
}

static void finalize_system_startup(void)
{
    ESP_LOGI(TAG, "Hardware initialized");

    // Initialize state mutex before creating tasks
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex — aborting");
        return;
    }

    // Small delay to let system stabilize
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_STABILIZATION_DELAY_MS));

    // Boot beep to indicate ready
    ESP_LOGI(TAG, "Playing boot beep...");
    sound_beep();

    ESP_LOGI(TAG, "RoboCar Serial Control initialized!");
    print_help_commands();

    // Create command processing task with adequate stack for UART + string + I2C ops
    xTaskCreate(command_task, "command_task", COMMAND_TASK_STACK_SIZE, NULL, 5, NULL);
}
