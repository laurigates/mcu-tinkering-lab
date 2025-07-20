/**
 * @file main.c
 * @brief ESP32-CAM AI vision system for robot control
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "config.h"
#include "credentials_validator.h"  // Must be first to validate credentials at compile time
#include "camera.h"
#include "wifi_manager.h"
#include "ai_backend.h"
#include "i2c_master.h"
#include "ai_response_parser.h"
#include "camera_pins.h"
#include "credentials_loader.h"

static const char *TAG = "esp32-cam-robocar";

// Global state
static bool g_wifi_connected = false;
static bool g_system_ready = false;
static TimerHandle_t capture_timer = NULL;
static TimerHandle_t status_led_timer = NULL;
static const ai_backend_t* g_ai_backend = NULL;

// Forward declarations
static void capture_and_analyze_task(void *pvParameters);
static void status_led_task(void *pvParameters);
static void capture_timer_callback(TimerHandle_t xTimer);
static void status_led_timer_callback(TimerHandle_t xTimer);
static void init_status_led(void);
static void set_status_led(bool on);
static void send_startup_commands(void);

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-CAM AI Vision System Starting...");
    ESP_LOGI(TAG, "Build timestamp: %s %s", __DATE__, __TIME__);

    // Validate credentials early - fail fast if misconfigured
    validate_credentials_at_runtime();

    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize status LED
    init_status_led();

    // Initialize I2C communication first
    ESP_LOGI(TAG, "Initializing I2C master...");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master");
        return;
    }

    // Initialize camera
    ESP_LOGI(TAG, "Initializing camera...");
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize camera");
        i2c_send_sound_command(SOUND_ALERT); // Send alert sound
        return;
    }

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (wifi_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi");
        i2c_send_sound_command(SOUND_ALERT); // Send alert sound
        return;
    }

    // Load and validate credentials
    ESP_LOGI(TAG, "Loading credentials...");
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Failed to load or validate credentials");
        i2c_send_sound_command(SOUND_ALERT); // Send alert sound
        return;
    }

    // Connect to WiFi using loaded credentials
    const char* wifi_ssid = get_wifi_ssid();
    const char* wifi_password = get_wifi_password();
    
    ESP_LOGI(TAG, "Connecting to WiFi: %s", wifi_ssid);
    if (wifi_connect(wifi_ssid, wifi_password) == ESP_OK) {
        g_wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected successfully");
        i2c_send_sound_command(SOUND_MELODY); // Send melody for success
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        i2c_send_sound_command(SOUND_ALERT); // Send alert sound
        // Continue anyway for testing I2C communication
    }

    // Initialize AI Backend
    if (g_wifi_connected) {
        ESP_LOGI(TAG, "WiFi connected successfully, attempting to initialize AI backend...");
        
        // Check which backend is configured
#if defined(CONFIG_AI_BACKEND_CLAUDE)
        ESP_LOGI(TAG, "Configuration: Claude backend selected");
        ESP_LOGI(TAG, "Claude API URL: %s", CLAUDE_API_URL);
        ESP_LOGI(TAG, "Claude model: %s", CLAUDE_MODEL);
#elif defined(CONFIG_AI_BACKEND_OLLAMA)
        ESP_LOGI(TAG, "Configuration: Ollama backend selected");
        ESP_LOGI(TAG, "Ollama fallback URL: %s", OLLAMA_API_URL);
        ESP_LOGI(TAG, "Ollama model: %s", OLLAMA_MODEL);
#if OLLAMA_USE_SERVICE_DISCOVERY
        ESP_LOGI(TAG, "Service discovery enabled, will attempt mDNS lookup for: %s", OLLAMA_SRV_RECORD);
#else
        ESP_LOGI(TAG, "Service discovery disabled, using static URL only");
#endif
#else
        ESP_LOGE(TAG, "ERROR: No AI backend defined in config.h! Check CONFIG_AI_BACKEND_CLAUDE or CONFIG_AI_BACKEND_OLLAMA");
#endif
        
        g_ai_backend = ai_backend_get_current();
        if (g_ai_backend) {
            ESP_LOGI(TAG, "AI backend interface obtained successfully");
            ai_config_t ai_config;
#if defined(CONFIG_AI_BACKEND_CLAUDE)
            ESP_LOGI(TAG, "Configuring Claude backend...");
            const char* claude_api_key = get_claude_api_key();
            if (!claude_api_key || strlen(claude_api_key) == 0) {
                ESP_LOGE(TAG, "Claude API key not available - cannot initialize Claude backend");
                g_ai_backend = NULL;
            } else {
                ai_config.api_key = claude_api_key;
                ai_config.api_url = "https://api.anthropic.com/v1/messages";
                ai_config.model = "claude-3-haiku-20240307";
            }
#elif defined(CONFIG_AI_BACKEND_OLLAMA)
            ESP_LOGI(TAG, "Configuring Ollama backend...");
            ai_config.api_key = NULL; // Ollama doesn't require a key
            ai_config.api_url = OLLAMA_API_URL;
            ai_config.model = OLLAMA_MODEL;
#endif
            
            // Only initialize if we have a valid backend
            if (g_ai_backend) {
                ESP_LOGI(TAG, "Calling backend initialization...");
                esp_err_t init_result = g_ai_backend->init(&ai_config);
            if (init_result != ESP_OK) {
                ESP_LOGE(TAG, "AI backend initialization FAILED with error: %s", esp_err_to_name(init_result));
#if defined(CONFIG_AI_BACKEND_OLLAMA)
                ESP_LOGE(TAG, "Ollama troubleshooting:");
                ESP_LOGE(TAG, "  1. Check if Ollama is running at %s", OLLAMA_API_URL);
                ESP_LOGE(TAG, "  2. Verify network connectivity to Ollama server");
#if OLLAMA_USE_SERVICE_DISCOVERY
                ESP_LOGE(TAG, "  3. Check if mDNS service is published: %s", OLLAMA_SRV_RECORD);
                ESP_LOGE(TAG, "  4. Try: dns-sd -B %s", "_ollama._tcp");
#endif
                ESP_LOGE(TAG, "  5. Test manually: curl %s", OLLAMA_API_URL);
#elif defined(CONFIG_AI_BACKEND_CLAUDE)
                ESP_LOGE(TAG, "Claude troubleshooting:");
                ESP_LOGE(TAG, "  1. Check API key is valid");
                ESP_LOGE(TAG, "  2. Verify internet connectivity");
                ESP_LOGE(TAG, "  3. Test API endpoint: %s", CLAUDE_API_URL);
#endif
                g_ai_backend = NULL; // Clear the backend pointer on failure
                i2c_send_sound_command(SOUND_ALERT);
            } else {
                ESP_LOGI(TAG, "AI backend initialized successfully!");
                ESP_LOGI(TAG, "System ready for AI-powered image analysis");
                }
            }
        } else {
            ESP_LOGE(TAG, "CRITICAL: ai_backend_get_current() returned NULL!");
            ESP_LOGE(TAG, "This indicates a compilation/configuration problem:");
#if defined(CONFIG_AI_BACKEND_CLAUDE)
            ESP_LOGE(TAG, "  - Claude backend should be available but isn't compiled in");
#elif defined(CONFIG_AI_BACKEND_OLLAMA)
            ESP_LOGE(TAG, "  - Ollama backend should be available but isn't compiled in");
#else
            ESP_LOGE(TAG, "  - No backend is defined in config.h");
            ESP_LOGE(TAG, "  - Add #define CONFIG_AI_BACKEND_OLLAMA or CONFIG_AI_BACKEND_CLAUDE");
#endif
        }
    } else {
        ESP_LOGW(TAG, "WiFi connection failed - skipping AI backend initialization");
        ESP_LOGW(TAG, "WiFi troubleshooting:");
        ESP_LOGW(TAG, "  1. Check WiFi credentials in credentials.h");
        ESP_LOGW(TAG, "  2. Verify WiFi network is available");
        ESP_LOGW(TAG, "  3. Check WiFi signal strength");
        ESP_LOGW(TAG, "AI features will be unavailable until WiFi connects");
    }

    // Send startup commands to main controller
    send_startup_commands();

    // System is ready
    g_system_ready = true;
    ESP_LOGI(TAG, "System initialization complete");

    // Create timers
    capture_timer = xTimerCreate("capture_timer", 
                                pdMS_TO_TICKS(CAPTURE_INTERVAL_MS),
                                pdTRUE, // Auto-reload
                                NULL,
                                capture_timer_callback);

    status_led_timer = xTimerCreate("status_led_timer",
                                   pdMS_TO_TICKS(STATUS_LED_ON_TIME_MS),
                                   pdFALSE, // One-shot
                                   NULL,
                                   status_led_timer_callback);

    // Create tasks
    xTaskCreate(capture_and_analyze_task, "capture_task", 8192, NULL, 5, NULL);
    xTaskCreate(status_led_task, "status_led_task", 2048, NULL, 1, NULL);

    // Start capture timer if system is ready (even without AI backend for testing)
    if (capture_timer) {
        xTimerStart(capture_timer, 0);
        ESP_LOGI(TAG, "Capture timer started");
    }

    // Start status LED timer
    if (status_led_timer) {
        xTimerStart(status_led_timer, 0);
    }

    ESP_LOGI(TAG, "Main loop started");
}

static void capture_timer_callback(TimerHandle_t xTimer) {
    // Signal capture task to run
    static TaskHandle_t capture_task_handle = NULL;
    if (capture_task_handle) {
        xTaskNotifyGive(capture_task_handle);
    }
}

static void status_led_timer_callback(TimerHandle_t xTimer) {
    set_status_led(false);
}

static void capture_and_analyze_task(void *pvParameters) {
    ESP_LOGI(TAG, "Capture and analyze task started");
    
    static char last_movement_command[16] = "S";
    static char last_sound_command[64] = "";
    
    while (1) {
        // Wait for timer notification or run every 5 seconds
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
        
        if (!g_system_ready) {
            continue;
        }

        ESP_LOGI(TAG, "Starting image capture and analysis cycle");
        set_status_led(true);
        xTimerReset(status_led_timer, 0);

        // Capture image
        camera_fb_t *fb = camera_capture();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            i2c_send_sound_command(SOUND_ALERT); // Alert sound
            continue;
        }

        ESP_LOGI(TAG, "Image captured: %zu bytes", fb->len);

        // Check if AI backend is available before using it
        if (!g_ai_backend) {
            ESP_LOGW(TAG, "AI backend not available, skipping analysis");
            ESP_LOGW(TAG, "Diagnostic: WiFi connected: %s", g_wifi_connected ? "YES" : "NO");
            if (g_wifi_connected) {
                ESP_LOGW(TAG, "Diagnostic: AI backend was available during init but failed");
                ESP_LOGW(TAG, "Diagnostic: Check previous logs for backend initialization errors");
#if defined(CONFIG_AI_BACKEND_CLAUDE)
                ESP_LOGW(TAG, "Diagnostic: Claude backend configured - check API key and URL");
#elif defined(CONFIG_AI_BACKEND_OLLAMA)
                ESP_LOGW(TAG, "Diagnostic: Ollama backend configured - check service discovery and fallback URL");
                ESP_LOGW(TAG, "Diagnostic: Ollama URL should be: %s", OLLAMA_API_URL);
                ESP_LOGW(TAG, "Diagnostic: Ollama model: %s", OLLAMA_MODEL);
#else
                ESP_LOGW(TAG, "Diagnostic: No AI backend defined in config.h!");
#endif
            } else {
                ESP_LOGW(TAG, "Diagnostic: WiFi not connected - this is likely the root cause");
                ESP_LOGW(TAG, "Diagnostic: Check WiFi credentials and network connectivity");
            }
            
            camera_return_fb(fb);
            fb = NULL;
            
            // In standalone mode, just capture images without AI processing
            ESP_LOGI(TAG, "Camera working in standalone mode - image capture successful");
            continue;
        }

        // Send to AI API
        ai_response_t response = {0};
        esp_err_t api_result = g_ai_backend->analyze_image(fb->buf, fb->len, &response);
        
        // Return frame buffer immediately
        camera_return_fb(fb);
        fb = NULL;

        if (api_result != ESP_OK) {
            ESP_LOGE(TAG, "AI API call failed");
            i2c_send_sound_command(SOUND_ALERT); // Alert sound
            continue;
        }

        if (!response.response_text) {
            ESP_LOGE(TAG, "Empty response from AI API");
            continue;
        }

        ESP_LOGI(TAG, "Received AI response (%zu bytes)", response.response_length);
        ESP_LOGD(TAG, "Response: %.200s", response.response_text);

        // Parse AI response
        ai_command_t command = {0};
        if (!parse_ai_response(response.response_text, &command)) {
            ESP_LOGE(TAG, "Failed to parse AI response");
            g_ai_backend->free_response(&response);
            continue;
        }

        g_ai_backend->free_response(&response);

        // Send movement command (only if different from last)
        if (command.has_movement && strcmp(command.movement_command, last_movement_command) != 0) {
            movement_command_t move_cmd = MOVE_STOP;
            
            // Convert string command to I2C enum
            if (strcmp(command.movement_command, "F") == 0) move_cmd = MOVE_FORWARD;
            else if (strcmp(command.movement_command, "B") == 0) move_cmd = MOVE_BACKWARD;
            else if (strcmp(command.movement_command, "L") == 0) move_cmd = MOVE_LEFT;
            else if (strcmp(command.movement_command, "R") == 0) move_cmd = MOVE_RIGHT;
            else if (strcmp(command.movement_command, "C") == 0) move_cmd = MOVE_ROTATE_CW;
            else if (strcmp(command.movement_command, "W") == 0) move_cmd = MOVE_ROTATE_CCW;
            else move_cmd = MOVE_STOP;
            
            ESP_LOGI(TAG, "Sending I2C movement command: %s", command.movement_command);
            i2c_send_movement_command(move_cmd, 255); // Full speed
            strcpy(last_movement_command, command.movement_command);
        }

        // Send sound command (only if different from last)
        if (command.has_sound && strcmp(command.sound_command, last_sound_command) != 0) {
            sound_command_t sound_cmd = SOUND_BEEP;
            
            // Convert string command to I2C enum
            if (strcmp(command.sound_command, "SB") == 0) sound_cmd = SOUND_BEEP;
            else if (strcmp(command.sound_command, "SM") == 0) sound_cmd = SOUND_MELODY;
            else if (strcmp(command.sound_command, "SA") == 0) sound_cmd = SOUND_ALERT;
            
            ESP_LOGI(TAG, "Sending I2C sound command: %s", command.sound_command);
            i2c_send_sound_command(sound_cmd);
            strcpy(last_sound_command, command.sound_command);
        }

        // Send pan command
        if (command.has_pan) {
            ESP_LOGI(TAG, "Sending I2C pan command: %d", command.pan_angle);
            i2c_send_servo_command(SERVO_PAN, command.pan_angle);
        }

        // Send tilt command
        if (command.has_tilt) {
            ESP_LOGI(TAG, "Sending I2C tilt command: %d", command.tilt_angle);
            i2c_send_servo_command(SERVO_TILT, command.tilt_angle);
        }

        // Send display message
        if (command.has_display) {
            ESP_LOGI(TAG, "Sending I2C display message to line %d: %s", command.display_line, command.display_message);
            i2c_send_display_command(command.display_line, command.display_message);
        }

        ESP_LOGI(TAG, "Analysis cycle complete");
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay before next cycle
    }
}

static void status_led_task(void *pvParameters) {
    ESP_LOGI(TAG, "Status LED task started");
    
    while (1) {
        if (g_system_ready && g_wifi_connected) {
            // Slow blink when operational
            set_status_led(true);
            vTaskDelay(pdMS_TO_TICKS(200));
            set_status_led(false);
            vTaskDelay(pdMS_TO_TICKS(1800));
        } else if (g_system_ready) {
            // Fast blink when WiFi disconnected
            set_status_led(true);
            vTaskDelay(pdMS_TO_TICKS(100));
            set_status_led(false);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            // Solid on during initialization
            set_status_led(true);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void init_status_led(void) {
#if STATUS_LED_ENABLED
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CAM_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    set_status_led(false);
    ESP_LOGI(TAG, "Status LED initialized on GPIO %d", CAM_LED_PIN);
#else
    ESP_LOGI(TAG, "Status LED disabled in configuration");
#endif
}

static void set_status_led(bool on) {
#if STATUS_LED_ENABLED
    gpio_set_level(CAM_LED_PIN, on ? 1 : 0);
#endif
}

static void send_startup_commands(void) {
    ESP_LOGI(TAG, "Sending startup commands to main controller");
    
    // Test communication with ping
    if (i2c_ping_slave() == ESP_OK) {
        ESP_LOGI(TAG, "I2C communication test successful");
    } else {
        ESP_LOGW(TAG, "I2C communication test failed");
    }
    
    // Stop any movement
    i2c_send_movement_command(MOVE_STOP, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Play startup melody
    i2c_send_sound_command(SOUND_MELODY);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Center servos
    i2c_send_servo_command(SERVO_PAN, 90);
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_send_servo_command(SERVO_TILT, 90);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Send startup display messages
    i2c_send_display_command(7, "CAM READY");
    vTaskDelay(pdMS_TO_TICKS(100));
    #if defined(CONFIG_AI_BACKEND_CLAUDE)
    i2c_send_display_command(6, "CLAUDE AI ON");
    #elif defined(CONFIG_AI_BACKEND_OLLAMA)
    i2c_send_display_command(6, "OLLAMA AI ON");
    #endif
    
    ESP_LOGI(TAG, "Startup commands sent");
}