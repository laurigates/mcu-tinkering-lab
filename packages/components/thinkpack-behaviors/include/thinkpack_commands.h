/**
 * @file thinkpack_commands.h
 * @brief Command IDs and payload structures for the ThinkPack behavior layer.
 *
 * These types encode the application-level commands carried inside a
 * thinkpack_command_data_t envelope (MSG_COMMAND packets).  The leader
 * (Brainbox) builds command packets using command_dispatcher.h; followers
 * unpack and dispatch them using command_executor.h.
 */

#ifndef THINKPACK_COMMANDS_H
#define THINKPACK_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

#include "thinkpack_protocol.h"

/* ------------------------------------------------------------------ */
/* Command IDs                                                         */
/* ------------------------------------------------------------------ */

/** Command IDs — encoded in thinkpack_command_data_t::command_id. */
typedef enum {
    /* LED / visual (Glowbug) */
    CMD_LED_PATTERN = 0x10, /**< Render a named LED animation */
    CMD_SET_MOOD = 0x30,    /**< Set a global mood (hue, intensity) */

    /* Audio (Boombox, Chatterbox, Finderbox) */
    CMD_PLAY_MELODY = 0x20,   /**< Switch melody pattern for N beats */
    CMD_BUZZ = 0x21,          /**< One-shot single tone */
    CMD_PLAY_SEQUENCE = 0x22, /**< Play a sequence of notes (array in payload) */

    /* Display (Brainbox) */
    CMD_DISPLAY_LINE = 0x40, /**< Write a line of text to the OLED */
} thinkpack_command_id_t;

/* ------------------------------------------------------------------ */
/* LED / visual payloads                                               */
/* ------------------------------------------------------------------ */

/** LED animation patterns for CMD_LED_PATTERN. */
typedef enum {
    LED_PATTERN_BREATHE = 0,
    LED_PATTERN_RAINBOW = 1,
    LED_PATTERN_SPARKLE = 2,
    LED_PATTERN_NIGHTLIGHT = 3,
    LED_PATTERN_SOLID = 4,
    LED_PATTERN_FLASH = 5,
} thinkpack_led_pattern_t;

/** Payload for CMD_LED_PATTERN. */
typedef struct __attribute__((packed)) {
    uint8_t r;       /**< Red channel 0-255 */
    uint8_t g;       /**< Green channel 0-255 */
    uint8_t b;       /**< Blue channel 0-255 */
    uint8_t pattern; /**< thinkpack_led_pattern_t */
} cmd_led_pattern_payload_t;

/** Payload for CMD_SET_MOOD. */
typedef struct __attribute__((packed)) {
    uint8_t hue;       /**< 0-255 */
    uint8_t intensity; /**< 0-100 */
} cmd_set_mood_payload_t;

/* ------------------------------------------------------------------ */
/* Audio payloads                                                      */
/* ------------------------------------------------------------------ */

/** Payload for CMD_PLAY_MELODY. */
typedef struct __attribute__((packed)) {
    uint8_t pattern_id;   /**< 0=MARCH, 1=WALTZ, 2=PENTATONIC, 3=SILENCE */
    uint8_t repeat_count; /**< Number of beats to stay in this pattern */
} cmd_play_melody_payload_t;

/** Payload for CMD_BUZZ. */
typedef struct __attribute__((packed)) {
    uint8_t note;          /**< thinkpack note_t equivalent index */
    uint16_t duration_ms;  /**< Duration in milliseconds */
    int8_t semitone_shift; /**< Signed semitone offset */
} cmd_buzz_payload_t;

/** Maximum notes per CMD_PLAY_SEQUENCE call. */
#define CMD_SEQUENCE_MAX_NOTES 20

/** Payload for CMD_PLAY_SEQUENCE. Max 20 notes per call. */
typedef struct __attribute__((packed)) {
    uint8_t note_count;        /**< Number of notes in notes[] (0-CMD_SEQUENCE_MAX_NOTES) */
    uint16_t note_duration_ms; /**< Uniform duration per step in milliseconds */
    int8_t semitone_shift;     /**< Signed semitone offset applied to all notes */
    uint8_t notes[CMD_SEQUENCE_MAX_NOTES]; /**< Note index array */
} cmd_play_sequence_payload_t;

/* ------------------------------------------------------------------ */
/* Display payloads                                                    */
/* ------------------------------------------------------------------ */

/** Payload for CMD_DISPLAY_LINE. */
typedef struct __attribute__((packed)) {
    uint8_t line;  /**< Display row 0-7 */
    char text[32]; /**< NUL-terminated text string */
} cmd_display_line_payload_t;

#endif /* THINKPACK_COMMANDS_H */
