/**
 * @file command_handler.h
 * @brief Command Pattern implementation for robot command processing
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Command types supported by the robot
 */
typedef enum {
    CMD_TYPE_MOVE = 0,
    CMD_TYPE_SOUND,
    CMD_TYPE_SERVO,
    CMD_TYPE_LED,
    CMD_TYPE_DISPLAY,
    CMD_TYPE_STATUS,
    CMD_TYPE_SYSTEM,
    CMD_TYPE_UNKNOWN
} command_type_t;

/**
 * @brief Movement command subtypes
 */
typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    MOVE_ROTATE_CW,
    MOVE_ROTATE_CCW
} move_command_t;

/**
 * @brief Sound command subtypes
 */
typedef enum {
    SOUND_BEEP = 0,
    SOUND_CHIRP,
    SOUND_ALERT,
    SOUND_SUCCESS,
    SOUND_ERROR,
    SOUND_CUSTOM
} sound_command_t;

/**
 * @brief Servo command subtypes
 */
typedef enum {
    SERVO_PAN = 0,
    SERVO_TILT,
    SERVO_CENTER,
    SERVO_POSITION
} servo_command_t;

/**
 * @brief LED command subtypes
 */
typedef enum {
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_WHITE,
    LED_YELLOW,
    LED_PURPLE,
    LED_CYAN,
    LED_RGB,
    LED_BLINK
} led_command_t;

/**
 * @brief Command parameter structure
 */
typedef struct {
    uint8_t param1;    // Speed, angle, color component, etc.
    uint8_t param2;    // Second parameter for RGB, position, etc.
    uint8_t param3;    // Third parameter for RGB, timing, etc.
    uint16_t param4;   // Extended parameter for timing, etc.
} command_params_t;

/**
 * @brief Complete command structure
 */
typedef struct {
    command_type_t type;
    uint8_t subtype;           // Cast to appropriate enum based on type
    command_params_t params;
    char source[16];           // Command source identifier
    uint32_t timestamp;        // Command timestamp
} robot_command_t;

/**
 * @brief Command handler function pointer
 */
typedef esp_err_t (*command_handler_func_t)(const robot_command_t *cmd);

/**
 * @brief Command lookup table entry
 */
typedef struct {
    const char *name;
    command_type_t type;
    uint8_t subtype;
    command_handler_func_t handler;
    uint8_t min_params;        // Minimum required parameters
    const char *description;
} command_entry_t;

/**
 * @brief Initialize command handler system
 * @return ESP_OK on success
 */
esp_err_t command_handler_init(void);

/**
 * @brief Parse string command into structured command
 * @param cmd_string Input command string
 * @param cmd Output command structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parsing failed
 */
esp_err_t command_parse_string(const char *cmd_string, robot_command_t *cmd);

/**
 * @brief Execute a structured command
 * @param cmd Command to execute
 * @return ESP_OK on success
 */
esp_err_t command_execute(const robot_command_t *cmd);

/**
 * @brief Execute command from string (convenience function)
 * @param cmd_string Command string to parse and execute
 * @param source Command source identifier
 * @return ESP_OK on success
 */
esp_err_t command_execute_string(const char *cmd_string, const char *source);

/**
 * @brief Get command info by name
 * @param name Command name
 * @return Pointer to command entry or NULL if not found
 */
const command_entry_t* command_get_info(const char *name);

/**
 * @brief List all available commands
 * @param buffer Buffer to store command list
 * @param buffer_size Size of buffer
 * @return Number of commands listed
 */
int command_list_all(char *buffer, size_t buffer_size);

/**
 * @brief Validate command parameters
 * @param cmd Command to validate
 * @return true if valid, false otherwise
 */
bool command_validate(const robot_command_t *cmd);

/**
 * @brief Get command type name
 * @param type Command type
 * @return String representation of command type
 */
const char* command_type_to_string(command_type_t type);

/**
 * @brief Create movement command
 * @param move_type Movement type
 * @param speed Movement speed (0-255)
 * @param source Command source
 * @return Initialized command structure
 */
robot_command_t command_create_move(move_command_t move_type, uint8_t speed, const char *source);

/**
 * @brief Create servo command
 * @param servo_type Servo command type
 * @param param1 Primary parameter (angle, etc.)
 * @param param2 Secondary parameter if needed
 * @param source Command source
 * @return Initialized command structure
 */
robot_command_t command_create_servo(servo_command_t servo_type, uint8_t param1, uint8_t param2, const char *source);

/**
 * @brief Create LED command
 * @param led_type LED command type
 * @param param1 Primary parameter (red/intensity)
 * @param param2 Secondary parameter (green)
 * @param param3 Tertiary parameter (blue)
 * @param source Command source
 * @return Initialized command structure
 */
robot_command_t command_create_led(led_command_t led_type, uint8_t param1, uint8_t param2, uint8_t param3, const char *source);

/**
 * @brief Create sound command
 * @param sound_type Sound type
 * @param duration Duration in milliseconds
 * @param frequency Frequency in Hz (for custom sounds)
 * @param source Command source
 * @return Initialized command structure
 */
robot_command_t command_create_sound(sound_command_t sound_type, uint16_t duration, uint16_t frequency, const char *source);

#endif // COMMAND_HANDLER_H