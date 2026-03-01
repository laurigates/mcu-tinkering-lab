/**
 * @file ai_response_parser.h
 * @brief Parser for Claude AI responses
 */

#ifndef AI_RESPONSE_PARSER_H
#define AI_RESPONSE_PARSER_H

#include <stdbool.h>

typedef struct {
    char movement_command[16];
    char sound_command[64];
    char display_message[64];
    int display_line;
    int pan_angle;
    int tilt_angle;
    bool has_movement;
    bool has_sound;
    bool has_display;
    bool has_pan;
    bool has_tilt;
} ai_command_t;

/**
 * @brief Parse Claude API response and extract commands
 * @param response_text Claude API response JSON
 * @param command Output command structure
 * @return true on success, false on error
 */
bool parse_ai_response(const char *response_text, ai_command_t *command);

/**
 * @brief Convert movement recommendation to command string
 * @param recommendation AI movement recommendation
 * @return Command string (static buffer)
 */
const char *movement_to_command(const char *recommendation);

#endif  // AI_RESPONSE_PARSER_H
