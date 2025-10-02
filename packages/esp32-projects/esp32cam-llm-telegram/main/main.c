#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "config.h"
#include "config_manager.h"
#include "camera_handler.h"
#include "telegram_bot.h"
#include "llm_client.h"
#include "motor_controller.h"
#include "vision_interpreter.h"

static const char* TAG = "MAIN";

// Event group for WiFi connection
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// Global app configuration
static app_config_t app_config;

// Global bot instance
static telegram_bot_t telegram_bot;

// Task handles
static TaskHandle_t camera_task_handle = NULL;
static TaskHandle_t telegram_task_handle = NULL;

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "WiFi disconnected, retrying...");
                esp_wifi_connect();
                xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi
static esp_err_t wifi_init(void) {
    ESP_LOGI(TAG, "Initializing WiFi");

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    strcpy((char*)wifi_config.sta.ssid, app_config.wifi_ssid);
    strcpy((char*)wifi_config.sta.password, app_config.wifi_password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization finished. Waiting for connection...");

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                          WIFI_CONNECTED_BIT,
                                          pdFALSE, pdFALSE,
                                          portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi SSID: %s", app_config.wifi_ssid);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        return ESP_FAIL;
    }
}

// Camera capture callback
static void camera_capture_callback(const uint8_t* data, size_t size, void* user_data) {
    ESP_LOGI(TAG, "Camera captured image: %d bytes", size);

    // Analyze image with vision interpreter
    vision_result_t vision_result = {0};
    esp_err_t err = vision_analyze_and_interpret(data, size, &vision_result);

    if (err == ESP_OK) {
        // Format and send Telegram message
        char* message = vision_format_telegram_message(&vision_result);
        telegram_send_text(&telegram_bot, message);

        // Send the actual image
        char caption[256];
        snprintf(caption, sizeof(caption),
                 "Action: %s | Confidence: %s",
                 motor_get_command_name(vision_result.suggested_action),
                 vision_result.confidence_level);
        telegram_send_photo(&telegram_bot, data, size, caption);

        // Execute motor command
        motor_command_t cmd;
        int speed;
        uint32_t duration;
        vision_get_motor_action(&vision_result, &cmd, &speed, &duration);

        motor_execute_command(cmd, speed, duration);

        // Send motor telemetry
        char* telemetry = motor_get_telemetry_string();
        telegram_send_text(&telegram_bot, telemetry);

        // Simulate movement
        motor_simulate_movement(duration);

        // Free vision result
        vision_free_result(&vision_result);
    } else {
        ESP_LOGE(TAG, "Failed to analyze image");
        telegram_send_text(&telegram_bot, "❌ Failed to analyze image");
    }
}

// Process Telegram command
static void process_telegram_command(const char* command, const char* args) {
    ESP_LOGI(TAG, "Processing command: %s %s", command, args);

    if (strcmp(command, "start") == 0) {
        telegram_send_text(&telegram_bot,
                          "🤖 *ESP32-CAM LLM Robot Controller*\n\n"
                          "Commands:\n"
                          "/capture - Take a photo and analyze\n"
                          "/forward - Move forward\n"
                          "/backward - Move backward\n"
                          "/left - Turn left\n"
                          "/right - Turn right\n"
                          "/stop - Stop movement\n"
                          "/status - Get system status\n"
                          "/auto [on/off] - Toggle auto capture");
    }
    else if (strcmp(command, "capture") == 0) {
        uint8_t* buffer = NULL;
        size_t size = 0;
        if (camera_capture_jpeg(&buffer, &size) == ESP_OK) {
            camera_capture_callback(buffer, size, NULL);
            camera_free_buffer(buffer);
        } else {
            telegram_send_text(&telegram_bot, "❌ Failed to capture image");
        }
    }
    else if (strcmp(command, "forward") == 0) {
        motor_execute_command(MOTOR_CMD_FORWARD, MOTOR_DEFAULT_SPEED, 2000);
        telegram_send_text(&telegram_bot, "✅ Moving forward");
    }
    else if (strcmp(command, "backward") == 0) {
        motor_execute_command(MOTOR_CMD_BACKWARD, MOTOR_DEFAULT_SPEED, 2000);
        telegram_send_text(&telegram_bot, "✅ Moving backward");
    }
    else if (strcmp(command, "left") == 0) {
        motor_execute_command(MOTOR_CMD_LEFT, MOTOR_DEFAULT_SPEED, 1000);
        telegram_send_text(&telegram_bot, "✅ Turning left");
    }
    else if (strcmp(command, "right") == 0) {
        motor_execute_command(MOTOR_CMD_RIGHT, MOTOR_DEFAULT_SPEED, 1000);
        telegram_send_text(&telegram_bot, "✅ Turning right");
    }
    else if (strcmp(command, "stop") == 0) {
        motor_stop();
        telegram_send_text(&telegram_bot, "✅ Stopped");
    }
    else if (strcmp(command, "status") == 0) {
        camera_status_t cam_status = camera_get_status();
        motor_status_t motor_status = motor_get_status();
        vision_interpreter_t vision_status = vision_get_status();

        char status_msg[512];
        snprintf(status_msg, sizeof(status_msg),
                 "*System Status*\n\n"
                 "📷 Camera: %s\n"
                 "Captures: %lu\n"
                 "Errors: %lu\n\n"
                 "🚗 Motor: %s\n"
                 "Distance: %.1f mm\n"
                 "Heading: %.1f°\n\n"
                 "👁 Vision: %d/%d analyses\n"
                 "Backend: %s",
                 cam_status.is_capturing ? "Active" : "Idle",
                 cam_status.capture_count,
                 cam_status.error_count,
                 motor_status.is_running ? "Running" : "Stopped",
                 motor_status.distance_traveled_mm,
                 motor_status.heading_degrees,
                 vision_status.successful_analyses,
                 vision_status.analysis_count,
                 app_config.llm_backend_type ? "Claude" : "Ollama");

        telegram_send_text(&telegram_bot, status_msg);
    }
    else if (strcmp(command, "auto") == 0) {
        if (strcmp(args, "on") == 0) {
            app_config.auto_capture_enabled = true;
            if (!cam_status.is_capturing) {
                camera_start_capture(app_config.capture_interval_ms,
                                   camera_capture_callback, NULL);
            }
            telegram_send_text(&telegram_bot, "✅ Auto capture enabled");
        } else if (strcmp(args, "off") == 0) {
            app_config.auto_capture_enabled = false;
            camera_stop_capture();
            telegram_send_text(&telegram_bot, "✅ Auto capture disabled");
        } else {
            telegram_send_text(&telegram_bot,
                             app_config.auto_capture_enabled ?
                             "Auto capture is ON" : "Auto capture is OFF");
        }
    }
    else {
        telegram_send_text(&telegram_bot, "❓ Unknown command. Try /start for help");
    }
}

// Telegram polling task
static void telegram_polling_task(void* pvParameters) {
    telegram_message_t msg = {0};

    while (1) {
        esp_err_t err = telegram_poll_updates(&telegram_bot, &msg);

        if (err == ESP_OK && msg.type == TELEGRAM_MSG_COMMAND) {
            char command[32] = {0};
            char args[256] = {0};

            if (telegram_parse_command(msg.text, command, args)) {
                process_telegram_command(command, args);
            }

            telegram_free_message(&msg);
        }

        vTaskDelay(pdMS_TO_TICKS(TELEGRAM_POLL_INTERVAL_MS));
    }
}

// Main camera task
static void camera_task(void* pvParameters) {
    while (1) {
        if (app_config.auto_capture_enabled) {
            uint8_t* buffer = NULL;
            size_t size = 0;

            if (camera_capture_jpeg(&buffer, &size) == ESP_OK) {
                camera_capture_callback(buffer, size, NULL);
                camera_free_buffer(buffer);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(app_config.capture_interval_ms));
    }
}

// Main application
void app_main(void) {
    ESP_LOGI(TAG, "ESP32-CAM LLM Telegram Bot Starting...");

    // Initialize configuration manager
    ESP_ERROR_CHECK(config_manager_init());

    // Load configuration
    ESP_ERROR_CHECK(config_load(&app_config));

    // Validate configuration
    if (!config_validate(&app_config)) {
        ESP_LOGE(TAG, "Invalid configuration! Please check settings");
        // In production, might want to enter configuration mode here
    }

    // Print configuration
    config_print(&app_config);

    // Initialize WiFi
    ESP_ERROR_CHECK(wifi_init());

    // Initialize camera
    ESP_ERROR_CHECK(camera_init());

    // Initialize motor controller
    ESP_ERROR_CHECK(motor_controller_init());

    // Initialize vision interpreter
    ESP_ERROR_CHECK(vision_interpreter_init());

    // Initialize LLM client
    llm_config_t llm_config = {
        .backend_type = app_config.llm_backend_type,
        .api_key = app_config.claude_api_key,
        .server_url = app_config.ollama_server_url,
        .model = app_config.llm_model,
        .timeout_ms = 30000
    };
    ESP_ERROR_CHECK(llm_client_init(&llm_config));

    // Initialize Telegram bot
    ESP_ERROR_CHECK(telegram_bot_init(&telegram_bot,
                                      app_config.telegram_bot_token,
                                      app_config.telegram_chat_id));

    // Send startup message
    telegram_send_status(&telegram_bot,
                        "🚀 ESP32-CAM LLM Robot started!\n"
                        "Backend: %s\n"
                        "Auto capture: %s",
                        app_config.llm_backend_type ? "Claude" : "Ollama",
                        app_config.auto_capture_enabled ? "ON" : "OFF");

    // Create tasks
    if (app_config.telegram_commands_enabled) {
        xTaskCreate(telegram_polling_task, "telegram_poll", TASK_STACK_SIZE,
                   NULL, TELEGRAM_TASK_PRIORITY, &telegram_task_handle);
    }

    if (app_config.auto_capture_enabled) {
        xTaskCreate(camera_task, "camera", TASK_STACK_SIZE,
                   NULL, CAMERA_TASK_PRIORITY, &camera_task_handle);
    }

    ESP_LOGI(TAG, "System ready!");

    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10 seconds

        // Could add periodic health checks here
        ESP_LOGD(TAG, "System running...");
    }
}