/*
 * ESP32 Kids Audio Toy
 *
 * Interactive audio toy with dual-voice generation and modulation effects
 * Perfect for exploring sound, pitch, and rhythm!
 *
 * Features:
 * - 3 potentiometers control pitch, duration, and interval
 * - 555 timer #1: Generates base drone/rhythm (dual-voice mode)
 * - 555 timer #2 (optional): Modulates ESP32 pitch for warble/vibrato effects
 * - Piezo speaker output with adjustable parameters
 * - Visual LED feedback
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

static const char *TAG = "AUDIO_TOY";

// Pin Definitions
#define PIEZO_PIN           GPIO_NUM_25   // PWM output for piezo speaker
#define LED_PIN             GPIO_NUM_2    // Visual feedback LED

// ADC channels for potentiometers and 555 inputs
#define POT_PITCH_CHANNEL   ADC1_CHANNEL_6  // GPIO34 - Controls pitch (frequency)
#define POT_DURATION_CHANNEL ADC1_CHANNEL_7  // GPIO35 - Controls beep duration
#define POT_INTERVAL_CHANNEL ADC1_CHANNEL_4  // GPIO32 - Controls interval between beeps
#define TIMER_555_MOD_CHANNEL ADC1_CHANNEL_5 // GPIO33 - 555 timer modulation input

// PWM (LEDC) Configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_8_BIT  // 8-bit resolution (0-255)
#define LEDC_DUTY           128  // 50% duty cycle for square wave

// Audio Parameters
#define MIN_FREQ_HZ         100   // Lowest frequency (Hz)
#define MAX_FREQ_HZ         2000  // Highest frequency (Hz)
#define MIN_DURATION_MS     50    // Shortest beep (ms)
#define MAX_DURATION_MS     1000  // Longest beep (ms)
#define MIN_INTERVAL_MS     100   // Shortest pause (ms)
#define MAX_INTERVAL_MS     2000  // Longest pause (ms)

// Modulation Parameters
#define MOD_DEPTH_MAX       200   // Maximum frequency deviation (Hz)
#define MOD_SMOOTHING       0.8   // Smoothing factor for modulation (0.0-1.0)

// Global variables for audio parameters
static float current_pitch_hz = 440.0;
static uint32_t current_duration_ms = 200;
static uint32_t current_interval_ms = 300;
static float modulation_value = 0.0;  // Smoothed modulation value

// ADC calibration
static esp_adc_cal_characteristics_t adc_chars;

/**
 * Initialize ADC for reading potentiometers and 555 timer inputs
 */
static void init_adc(void)
{
    // Configure ADC width (12-bit resolution, 0-4095)
    adc1_config_width(ADC_WIDTH_BIT_12);

    // Configure attenuation for all channels (0-3.3V range)
    adc1_config_channel_atten(POT_PITCH_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT_DURATION_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT_INTERVAL_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(TIMER_555_MOD_CHANNEL, ADC_ATTEN_DB_11);

    // Characterize ADC for more accurate readings
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                             1100, &adc_chars);

    ESP_LOGI(TAG, "ADC initialized");
}

/**
 * Initialize PWM (LEDC) for piezo speaker
 */
static void init_pwm(void)
{
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 1000,  // Initial frequency (will be updated)
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIEZO_PIN,
        .duty           = 0,  // Start silent
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    ESP_LOGI(TAG, "PWM initialized on GPIO %d", PIEZO_PIN);
}

/**
 * Initialize LED for visual feedback
 */
static void init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);

    ESP_LOGI(TAG, "LED initialized on GPIO %d", LED_PIN);
}

/**
 * Map ADC value (0-4095) to a range (min-max)
 */
static float map_adc_to_range(uint32_t adc_value, float min_val, float max_val)
{
    // Clamp ADC value to valid range to prevent out-of-bounds results
    if (adc_value > 4095) {
        adc_value = 4095;
    }
    return min_val + (max_val - min_val) * (adc_value / 4095.0);
}

/**
 * Read potentiometers and update audio parameters
 */
static void read_controls(void)
{
    // Read potentiometers
    uint32_t pitch_adc = adc1_get_raw(POT_PITCH_CHANNEL);
    uint32_t duration_adc = adc1_get_raw(POT_DURATION_CHANNEL);
    uint32_t interval_adc = adc1_get_raw(POT_INTERVAL_CHANNEL);

    // Map to parameter ranges
    current_pitch_hz = map_adc_to_range(pitch_adc, MIN_FREQ_HZ, MAX_FREQ_HZ);
    current_duration_ms = (uint32_t)map_adc_to_range(duration_adc,
                                                      MIN_DURATION_MS,
                                                      MAX_DURATION_MS);
    current_interval_ms = (uint32_t)map_adc_to_range(interval_adc,
                                                      MIN_INTERVAL_MS,
                                                      MAX_INTERVAL_MS);
}

/**
 * Read 555 timer output and apply modulation
 * The 555 output is read as an analog value (voltage level changes create pitch modulation)
 */
static void read_modulation(void)
{
    uint32_t mod_adc = adc1_get_raw(TIMER_555_MOD_CHANNEL);

    // Map ADC to modulation depth (-MOD_DEPTH_MAX to +MOD_DEPTH_MAX Hz)
    float raw_mod = map_adc_to_range(mod_adc, -MOD_DEPTH_MAX, MOD_DEPTH_MAX);

    // Apply smoothing to reduce jitter (exponential moving average)
    modulation_value = (MOD_SMOOTHING * modulation_value) +
                       ((1.0 - MOD_SMOOTHING) * raw_mod);
}

/**
 * Play a tone at the current pitch with modulation
 */
static void play_tone(uint32_t duration_ms)
{
    // Apply modulation to base pitch
    float modulated_freq = current_pitch_hz + modulation_value;

    // Clamp to valid range
    if (modulated_freq < MIN_FREQ_HZ) modulated_freq = MIN_FREQ_HZ;
    if (modulated_freq > MAX_FREQ_HZ) modulated_freq = MAX_FREQ_HZ;

    // Set PWM frequency
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, (uint32_t)modulated_freq);

    // Turn on tone (50% duty cycle)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // Turn on LED for visual feedback
    gpio_set_level(LED_PIN, 1);

    ESP_LOGI(TAG, "Tone: %.1f Hz (base: %.1f Hz, mod: %.1f Hz) for %lu ms",
             modulated_freq, current_pitch_hz, modulation_value, duration_ms);

    // Wait for duration
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Turn off tone
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // Turn off LED
    gpio_set_level(LED_PIN, 0);
}

/**
 * Main audio generation task
 */
static void audio_task(void *arg)
{
    ESP_LOGI(TAG, "Audio task started");

    while (1) {
        // Read all control inputs
        read_controls();
        read_modulation();

        // Play tone with current parameters
        play_tone(current_duration_ms);

        // Wait for interval before next beep
        vTaskDelay(pdMS_TO_TICKS(current_interval_ms));
    }
}

/**
 * Main application
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Kids Audio Toy - Starting!");
    ESP_LOGI(TAG, "Features: Dual-voice (555 + ESP32) + Modulation effects");

    // Initialize hardware
    init_adc();
    init_pwm();
    init_led();

    ESP_LOGI(TAG, "Hardware initialized.");

    // Startup sequence - play a little tune!
    // (Play this BEFORE starting the audio task to avoid race condition)
    ESP_LOGI(TAG, "Playing startup sequence...");
    for (int i = 0; i < 3; i++) {
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, 440 + (i * 110));
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Ready! Starting audio generation...");

    // Create audio task (after startup sequence to avoid race condition)
    xTaskCreate(audio_task, "audio_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Turn the knobs and have fun!");
}
