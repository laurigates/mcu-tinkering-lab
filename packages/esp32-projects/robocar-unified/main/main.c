/**
 * @file main.c
 * @brief Unified robocar firmware for XIAO ESP32-S3 Sense
 *
 * Single-board architecture replacing the dual-board (Heltec + ESP32-CAM)
 * design. Camera, AI, motor control, and peripherals all run on one board
 * with core affinity for isolation:
 *   - Core 0: motor control, peripheral I/O, serial commands
 *   - Core 1: camera capture, AI analysis, WiFi/MQTT/OTA
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "config.h"
#include "pin_config.h"
#include "i2c_bus.h"
#include "motor_controller.h"
#include "led_controller.h"
#include "servo_controller.h"
#include "buzzer.h"
#include "camera.h"
#include "ai_backend.h"
#include "ai_response_parser.h"
#include "wifi_manager.h"
#include "credentials_loader.h"
#include "mqtt_logger.h"
#include "system_state.h"
#include "ota_manager.h"
#include "mdns.h"

static const char *TAG = "robocar";

// ========================================
// Command types for FreeRTOS queues
// ========================================
typedef enum {
    MOTOR_CMD_FORWARD,
    MOTOR_CMD_BACKWARD,
    MOTOR_CMD_LEFT,
    MOTOR_CMD_RIGHT,
    MOTOR_CMD_ROTATE_CW,
    MOTOR_CMD_ROTATE_CCW,
    MOTOR_CMD_STOP,
} motor_cmd_type_t;

typedef struct {
    motor_cmd_type_t type;
    uint8_t speed;
} motor_cmd_t;

typedef enum {
    PERIPH_CMD_LED_COLOR,
    PERIPH_CMD_SERVO_PAN,
    PERIPH_CMD_SERVO_TILT,
    PERIPH_CMD_SOUND_BEEP,
    PERIPH_CMD_SOUND_MELODY,
    PERIPH_CMD_SOUND_ALERT,
} periph_cmd_type_t;

typedef struct {
    periph_cmd_type_t type;
    union {
        struct {
            uint8_t r, g, b;
            led_position_t position;
        } led;
        int16_t angle;
    };
} periph_cmd_t;

// ========================================
// Global state
// ========================================
static QueueHandle_t s_motor_queue;
static QueueHandle_t s_periph_queue;
static SemaphoreHandle_t s_ai_mutex;
static const ai_backend_t *s_ai_backend;
static int64_t s_last_motor_cmd_time;

// ========================================
// Motor control task (Core 0, priority 6)
// ========================================
static void motor_control_task(void *pvParameters)
{
    (void)pvParameters;
    motor_cmd_t cmd;

    ESP_LOGI(TAG, "Motor control task started on core %d", xPortGetCoreID());

    while (1) {
        if (xQueueReceive(s_motor_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_last_motor_cmd_time = esp_timer_get_time();

            switch (cmd.type) {
                case MOTOR_CMD_FORWARD:  motor_move_forward(cmd.speed); break;
                case MOTOR_CMD_BACKWARD: motor_move_backward(cmd.speed); break;
                case MOTOR_CMD_LEFT:     motor_turn_left(cmd.speed); break;
                case MOTOR_CMD_RIGHT:    motor_turn_right(cmd.speed); break;
                case MOTOR_CMD_ROTATE_CW:  motor_rotate_cw(cmd.speed); break;
                case MOTOR_CMD_ROTATE_CCW: motor_rotate_ccw(cmd.speed); break;
                case MOTOR_CMD_STOP:     motor_stop(); break;
            }
        }

        // Command timeout watchdog
        if (s_last_motor_cmd_time > 0) {
            int64_t elapsed = esp_timer_get_time() - s_last_motor_cmd_time;
            if (elapsed > (int64_t)COMMAND_TIMEOUT_MS * 1000) {
                motor_stop();
                s_last_motor_cmd_time = 0;
            }
        }
    }
}

// ========================================
// Peripheral task (Core 0, priority 4)
// ========================================
static void peripheral_task(void *pvParameters)
{
    (void)pvParameters;
    periph_cmd_t cmd;

    ESP_LOGI(TAG, "Peripheral task started on core %d", xPortGetCoreID());

    while (1) {
        if (xQueueReceive(s_periph_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (cmd.type) {
                case PERIPH_CMD_LED_COLOR:
                    led_set_rgb(cmd.led.position, cmd.led.r, cmd.led.g, cmd.led.b);
                    break;
                case PERIPH_CMD_SERVO_PAN:
                    servo_set_pan(cmd.angle);
                    break;
                case PERIPH_CMD_SERVO_TILT:
                    servo_set_tilt(cmd.angle);
                    break;
                case PERIPH_CMD_SOUND_BEEP:
                    buzzer_beep();
                    break;
                case PERIPH_CMD_SOUND_MELODY:
                    buzzer_melody();
                    break;
                case PERIPH_CMD_SOUND_ALERT:
                    buzzer_alert();
                    break;
            }
        }
    }
}

// ========================================
// Dispatch helpers (used by AI and serial)
// ========================================
static void dispatch_motor_cmd(motor_cmd_type_t type, uint8_t speed)
{
    motor_cmd_t cmd = {.type = type, .speed = speed};
    xQueueSend(s_motor_queue, &cmd, pdMS_TO_TICKS(100));
}

static void dispatch_periph_cmd(const periph_cmd_t *cmd)
{
    xQueueSend(s_periph_queue, cmd, pdMS_TO_TICKS(100));
}

static void dispatch_movement(const char *movement)
{
    if (!movement) return;

    uint8_t speed = (uint8_t)(DEFAULT_SPEED * 255 / PCA9685_PWM_MAX);

    if (strcmp(movement, "forward") == 0) dispatch_motor_cmd(MOTOR_CMD_FORWARD, speed);
    else if (strcmp(movement, "backward") == 0) dispatch_motor_cmd(MOTOR_CMD_BACKWARD, speed);
    else if (strcmp(movement, "left") == 0) dispatch_motor_cmd(MOTOR_CMD_LEFT, speed);
    else if (strcmp(movement, "right") == 0) dispatch_motor_cmd(MOTOR_CMD_RIGHT, speed);
    else if (strcmp(movement, "rotate_cw") == 0) dispatch_motor_cmd(MOTOR_CMD_ROTATE_CW, speed);
    else if (strcmp(movement, "rotate_ccw") == 0) dispatch_motor_cmd(MOTOR_CMD_ROTATE_CCW, speed);
    else if (strcmp(movement, "stop") == 0) dispatch_motor_cmd(MOTOR_CMD_STOP, 0);
}

static void dispatch_sound(const char *sound)
{
    if (!sound) return;

    periph_cmd_t cmd;
    if (strcmp(sound, "beep") == 0) cmd.type = PERIPH_CMD_SOUND_BEEP;
    else if (strcmp(sound, "melody") == 0) cmd.type = PERIPH_CMD_SOUND_MELODY;
    else if (strcmp(sound, "alert") == 0) cmd.type = PERIPH_CMD_SOUND_ALERT;
    else return;

    dispatch_periph_cmd(&cmd);
}

// ========================================
// Process AI commands (replaces I2C dispatch)
// ========================================
static void process_ai_commands(const ai_command_t *cmd)
{
    if (cmd->has_movement) {
        const char *movement = movement_to_command(cmd->movement_command);
        dispatch_movement(movement);
        ESP_LOGI(TAG, "AI movement: %s", movement);
    }

    if (cmd->has_sound) {
        dispatch_sound(cmd->sound_command);
    }

    if (cmd->has_pan) {
        periph_cmd_t pcmd = {.type = PERIPH_CMD_SERVO_PAN, .angle = (int16_t)cmd->pan_angle};
        dispatch_periph_cmd(&pcmd);
    }

    if (cmd->has_tilt) {
        periph_cmd_t pcmd = {.type = PERIPH_CMD_SERVO_TILT, .angle = (int16_t)cmd->tilt_angle};
        dispatch_periph_cmd(&pcmd);
    }
}

// ========================================
// AI analysis task (Core 1, priority 3)
// ========================================
static void ai_analysis_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "AI analysis task started on core %d", xPortGetCoreID());

    // Wait for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_STABILIZATION_DELAY_MS));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));

        if (!s_ai_backend) {
            ESP_LOGW(TAG, "AI backend not available");
            continue;
        }

        // Capture image
        camera_fb_t *fb = camera_capture();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
            continue;
        }

        // Analyze with AI
        if (xSemaphoreTake(s_ai_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
            ai_response_t response = {0};
            esp_err_t ret = s_ai_backend->analyze_image(fb->buf, fb->len, &response);

            camera_return_fb(fb);

            if (ret == ESP_OK && response.response_text) {
                ESP_LOGI(TAG, "AI response: %.100s...", response.response_text);

                ai_command_t cmd = {0};
                if (parse_ai_response(response.response_text, &cmd)) {
                    process_ai_commands(&cmd);
                }

                s_ai_backend->free_response(&response);
            } else {
                ESP_LOGW(TAG, "AI analysis failed: %s", esp_err_to_name(ret));
            }

            xSemaphoreGive(s_ai_mutex);
        }
    }
}

// ========================================
// Serial command task (Core 0, priority 5)
// ========================================
static void command_task(void *pvParameters)
{
    (void)pvParameters;
    char buf[32];
    int buf_pos = 0;

    ESP_LOGI(TAG, "Command task started on core %d", xPortGetCoreID());

    while (1) {
        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SHORT_MS));
            continue;
        }

        if (ch == '\n' || ch == '\r') {
            if (buf_pos > 0) {
                buf[buf_pos] = '\0';
                ESP_LOGI(TAG, "Serial cmd: %s", buf);

                // Single-letter movement commands (backward compatible)
                if (buf_pos == 1) {
                    switch (buf[0]) {
                        case 'F': case 'f': dispatch_movement("forward"); break;
                        case 'B': case 'b': dispatch_movement("backward"); break;
                        case 'L': case 'l': dispatch_movement("left"); break;
                        case 'R': case 'r': dispatch_movement("right"); break;
                        case 'C': case 'c': dispatch_movement("rotate_cw"); break;
                        case 'W': case 'w': dispatch_movement("rotate_ccw"); break;
                        case 'S': case 's': dispatch_movement("stop"); break;
                    }
                }

                buf_pos = 0;
            }
        } else if (buf_pos < (int)sizeof(buf) - 1) {
            buf[buf_pos++] = (char)ch;
        }
    }
}

// ========================================
// Initialization phases
// ========================================
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

static esp_err_t init_hardware(void)
{
    ESP_LOGI(TAG, "Phase 1: I2C bus + peripherals");

    ESP_RETURN_ON_ERROR(i2c_bus_init(), TAG, "I2C bus init failed");
    ESP_RETURN_ON_ERROR(motor_controller_init(), TAG, "Motor init failed");
    ESP_RETURN_ON_ERROR(led_controller_init(), TAG, "LED init failed");
    ESP_RETURN_ON_ERROR(servo_controller_init(), TAG, "Servo init failed");
    ESP_RETURN_ON_ERROR(buzzer_init(), TAG, "Buzzer init failed");

    // Startup indication
    led_set_both(&LED_COLOR_GREEN);
    buzzer_beep();

    return ESP_OK;
}

static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Phase 2: Camera");
    return camera_init();
}

static esp_err_t init_network(void)
{
    ESP_LOGI(TAG, "Phase 3: WiFi + network");

    // Load credentials and connect WiFi
    credentials_t creds = {0};
    load_credentials(&creds);
    wifi_init();
    if (creds.credentials_loaded && strlen(creds.wifi_ssid) > 0) {
        wifi_connect(creds.wifi_ssid, creds.wifi_password);
    } else {
        ESP_LOGW(TAG, "No WiFi credentials - waiting for Improv provisioning");
    }

    // mDNS
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("robocar"));
    ESP_ERROR_CHECK(mdns_instance_name_set("Robocar Unified"));
    ESP_LOGI(TAG, "mDNS: robocar.local");

    return ESP_OK;
}

static esp_err_t init_ai_backend(void)
{
    ESP_LOGI(TAG, "Phase 4: AI backend");

    s_ai_backend = ai_backend_get_current();
    if (!s_ai_backend) {
        ESP_LOGW(TAG, "No AI backend configured");
        return ESP_OK;
    }

    ai_config_t ai_config = {0};
#if defined(CONFIG_AI_BACKEND_OLLAMA)
    ai_config.api_url = OLLAMA_API_URL;
    ai_config.model = OLLAMA_MODEL;
#elif defined(CONFIG_AI_BACKEND_GEMINI)
    ai_config.api_url = GEMINI_API_URL;
    ai_config.model = GEMINI_MODEL;
#endif

    esp_err_t ret = s_ai_backend->init(&ai_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AI backend init failed: %s", esp_err_to_name(ret));
        s_ai_backend = NULL;
    }

    return ESP_OK;
}

static void create_tasks(void)
{
    ESP_LOGI(TAG, "Phase 5: Creating FreeRTOS tasks");

    s_motor_queue = xQueueCreate(MOTOR_CMD_QUEUE_DEPTH, sizeof(motor_cmd_t));
    s_periph_queue = xQueueCreate(PERIPHERAL_CMD_QUEUE_DEPTH, sizeof(periph_cmd_t));
    s_ai_mutex = xSemaphoreCreateMutex();

    // Core 0 tasks: motor control + peripherals + serial
    xTaskCreatePinnedToCore(motor_control_task, "motor", MOTOR_TASK_STACK_SIZE,
                            NULL, MOTOR_TASK_PRIORITY, NULL, MOTOR_TASK_CORE);
    xTaskCreatePinnedToCore(peripheral_task, "periph", PERIPHERAL_TASK_STACK_SIZE,
                            NULL, PERIPHERAL_TASK_PRIORITY, NULL, PERIPHERAL_TASK_CORE);
    xTaskCreatePinnedToCore(command_task, "cmd", COMMAND_TASK_STACK_SIZE,
                            NULL, COMMAND_TASK_PRIORITY, NULL, COMMAND_TASK_CORE);

    // Core 1 tasks: camera + AI
    xTaskCreatePinnedToCore(ai_analysis_task, "ai", AI_TASK_STACK_SIZE,
                            NULL, AI_TASK_PRIORITY, NULL, AI_TASK_CORE);
}

// ========================================
// Application entry point
// ========================================
void app_main(void)
{
    ESP_LOGI(TAG, "=== Robocar Unified (XIAO ESP32-S3 Sense) ===");
    ESP_LOGI(TAG, "Firmware version: %s", esp_app_get_description()->version);

    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(init_hardware());
    ESP_ERROR_CHECK(init_camera());
    init_network();
    init_ai_backend();
    create_tasks();

    ESP_LOGI(TAG, "=== System ready ===");

    // app_main returns; FreeRTOS scheduler runs tasks
}
