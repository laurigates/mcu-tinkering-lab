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

#include <math.h>
#include <stdio.h>
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_core.h"

static const char *TAG = "AUDIO_TOY";

// Pin Definitions
#define PIEZO_PIN GPIO_NUM_25  // PWM output for piezo speaker
#define LED_PIN GPIO_NUM_2     // Visual feedback LED

// ADC channels for potentiometers and 555 inputs
#define POT_PITCH_CHANNEL ADC1_CHANNEL_6      // GPIO34 - Controls pitch (frequency)
#define POT_DURATION_CHANNEL ADC1_CHANNEL_7   // GPIO35 - Controls beep duration
#define POT_INTERVAL_CHANNEL ADC1_CHANNEL_4   // GPIO32 - Controls interval between beeps
#define TIMER_555_MOD_CHANNEL ADC1_CHANNEL_5  // GPIO33 - 555 timer modulation input

// PWM (LEDC) Configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT  // 8-bit resolution (0-255)
#define LEDC_DUTY 128                   // 50% duty cycle for square wave

// Audio parameter ranges and modulation constants live in audio_core.h
// (AUDIO_MIN_FREQ_HZ, AUDIO_MOD_DEPTH_MAX, ...) so the firmware, the host
// unit tests, and the host simulator all share one source of truth.

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
    esp_adc_cal_value_t cal_type =
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);

    // Log calibration type for debugging
    if (cal_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ADC calibrated using Two Point values from eFuse");
    } else if (cal_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ADC calibrated using eFuse Vref");
    } else {
        ESP_LOGW(TAG, "ADC calibrated using default Vref (less accurate)");
    }

    ESP_LOGI(TAG, "ADC initialized");
}

/**
 * Initialize PWM (LEDC) for piezo speaker
 */
static void init_pwm(void)
{
    // Configure timer
    ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                      .timer_num = LEDC_TIMER,
                                      .duty_resolution = LEDC_DUTY_RES,
                                      .freq_hz = 1000,  // Initial frequency (will be updated)
                                      .clk_cfg = LEDC_AUTO_CLK};
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return;
    }

    // Configure channel
    ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                          .channel = LEDC_CHANNEL,
                                          .timer_sel = LEDC_TIMER,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .gpio_num = PIEZO_PIN,
                                          .duty = 0,  // Start silent
                                          .hpoint = 0};
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return;
    }

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
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED GPIO: %s", esp_err_to_name(ret));
        return;
    }
    gpio_set_level(LED_PIN, 0);

    ESP_LOGI(TAG, "LED initialized on GPIO %d", LED_PIN);
}

/**
 * Read potentiometers and update audio parameters (mapping in audio_core)
 */
static void read_controls(void)
{
    uint32_t pitch_adc = adc1_get_raw(POT_PITCH_CHANNEL);
    uint32_t duration_adc = adc1_get_raw(POT_DURATION_CHANNEL);
    uint32_t interval_adc = adc1_get_raw(POT_INTERVAL_CHANNEL);

    audio_params_t params;
    audio_params_from_adc(pitch_adc, duration_adc, interval_adc, &params);

    current_pitch_hz = params.pitch_hz;
    current_duration_ms = (uint32_t)params.duration_ms;
    current_interval_ms = (uint32_t)params.interval_ms;
}

/**
 * Read 555 timer output and apply modulation (smoothing/clamp in audio_core)
 * The 555 output is read as an analog value (voltage level changes create pitch modulation)
 */
static void read_modulation(void)
{
    uint32_t mod_adc = adc1_get_raw(TIMER_555_MOD_CHANNEL);
    modulation_value = audio_update_modulation(modulation_value, mod_adc);
}

/**
 * Play a tone at the current pitch with modulation
 */
static void play_tone(uint32_t duration_ms)
{
    // Apply modulation to base pitch (clamped to valid range in audio_core)
    float modulated_freq = audio_modulated_freq(current_pitch_hz, modulation_value);

    // Set PWM frequency
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, (uint32_t)modulated_freq);

    // Turn on tone (50% duty cycle)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // Turn on LED for visual feedback
    gpio_set_level(LED_PIN, 1);

    ESP_LOGI(TAG, "Tone: %.1f Hz (base: %.1f Hz, mod: %.1f Hz) for %lu ms", modulated_freq,
             current_pitch_hz, modulation_value, duration_ms);

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
