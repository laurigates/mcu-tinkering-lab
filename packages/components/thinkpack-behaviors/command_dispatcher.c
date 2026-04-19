/**
 * @file command_dispatcher.c
 * @brief Leader-side command packet builders for the ThinkPack behavior layer.
 *
 * Each helper constructs a thinkpack_command_data_t envelope, copies it into
 * the packet's data field, then calls thinkpack_finalize() to stamp the magic
 * bytes and checksum.  No ESP-IDF runtime calls are made — purely packet
 * construction.
 */

#include "command_dispatcher.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Compile-time payload size assertions                                */
/* ------------------------------------------------------------------ */

_Static_assert(sizeof(cmd_led_pattern_payload_t) <= 48,
               "cmd_led_pattern_payload_t exceeds thinkpack_command_data_t payload limit");
_Static_assert(sizeof(cmd_set_mood_payload_t) <= 48,
               "cmd_set_mood_payload_t exceeds thinkpack_command_data_t payload limit");
_Static_assert(sizeof(cmd_play_melody_payload_t) <= 48,
               "cmd_play_melody_payload_t exceeds thinkpack_command_data_t payload limit");
_Static_assert(sizeof(cmd_buzz_payload_t) <= 48,
               "cmd_buzz_payload_t exceeds thinkpack_command_data_t payload limit");
_Static_assert(sizeof(cmd_play_sequence_payload_t) <= 48,
               "cmd_play_sequence_payload_t exceeds thinkpack_command_data_t payload limit");
_Static_assert(sizeof(cmd_display_line_payload_t) <= 48,
               "cmd_display_line_payload_t exceeds thinkpack_command_data_t payload limit");

/* ------------------------------------------------------------------ */
/* Module tag                                                          */
/* ------------------------------------------------------------------ */

static const char *TAG __attribute__((unused)) = "cmd_build";

/* ------------------------------------------------------------------ */
/* Internal helper                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Common packet setup: zero, set header fields, embed envelope, finalize.
 *
 * @param p           Output packet.
 * @param seq         Sequence number.
 * @param src_mac     Sender 6-byte MAC.
 * @param command_id  Application command identifier.
 * @param payload     Pointer to command-specific payload bytes.
 * @param payload_len Byte count of the payload (must be <= 48).
 */
static void build_command_packet(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                 uint8_t command_id, const void *payload, uint8_t payload_len)
{
    thinkpack_command_data_t env;
    memset(&env, 0, sizeof(env));
    env.command_id = command_id;
    env.length = payload_len;
    memcpy(env.payload, payload, payload_len);

    memset(p, 0, sizeof(*p));
    p->msg_type = MSG_COMMAND;
    p->sequence_number = seq;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = (uint8_t)sizeof(thinkpack_command_data_t);
    memcpy(p->data, &env, sizeof(thinkpack_command_data_t));
    thinkpack_finalize(p);
}

/* ------------------------------------------------------------------ */
/* Public builders                                                     */
/* ------------------------------------------------------------------ */

void command_build_led_pattern(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                               const cmd_led_pattern_payload_t *payload)
{
    if (!p || !payload) {
        return;
    }
    build_command_packet(p, seq, src_mac, CMD_LED_PATTERN, payload, (uint8_t)sizeof(*payload));
}

void command_build_set_mood(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                            const cmd_set_mood_payload_t *payload)
{
    if (!p || !payload) {
        return;
    }
    build_command_packet(p, seq, src_mac, CMD_SET_MOOD, payload, (uint8_t)sizeof(*payload));
}

void command_build_play_melody(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                               const cmd_play_melody_payload_t *payload)
{
    if (!p || !payload) {
        return;
    }
    build_command_packet(p, seq, src_mac, CMD_PLAY_MELODY, payload, (uint8_t)sizeof(*payload));
}

void command_build_buzz(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                        const cmd_buzz_payload_t *payload)
{
    if (!p || !payload) {
        return;
    }
    build_command_packet(p, seq, src_mac, CMD_BUZZ, payload, (uint8_t)sizeof(*payload));
}

void command_build_play_sequence(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                 const cmd_play_sequence_payload_t *payload)
{
    if (!p || !payload) {
        return;
    }
    build_command_packet(p, seq, src_mac, CMD_PLAY_SEQUENCE, payload, (uint8_t)sizeof(*payload));
}

void command_build_display_line(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                const cmd_display_line_payload_t *payload)
{
    if (!p || !payload) {
        return;
    }
    build_command_packet(p, seq, src_mac, CMD_DISPLAY_LINE, payload, (uint8_t)sizeof(*payload));
}
