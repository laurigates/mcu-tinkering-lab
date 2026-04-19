/**
 * @file command_dispatcher.h
 * @brief Leader-side helpers: build MSG_COMMAND packets for each command type.
 *
 * Each helper constructs a thinkpack_packet_t carrying the appropriate
 * thinkpack_command_data_t envelope.  The caller is responsible for
 * transmitting the resulting packet via thinkpack_mesh_send().
 */

#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include "thinkpack_commands.h"

/**
 * @brief Build a MSG_COMMAND packet carrying an LED pattern directive.
 *
 * @param p        Output packet.
 * @param seq      Sequence number.
 * @param src_mac  Sender MAC (dispatcher's MAC).
 * @param payload  LED pattern payload.
 */
void command_build_led_pattern(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                               const cmd_led_pattern_payload_t *payload);

/**
 * @brief Build a MSG_COMMAND packet carrying a mood directive.
 *
 * @param p        Output packet.
 * @param seq      Sequence number.
 * @param src_mac  Sender MAC.
 * @param payload  Mood payload.
 */
void command_build_set_mood(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                            const cmd_set_mood_payload_t *payload);

/**
 * @brief Build a MSG_COMMAND packet carrying a melody directive.
 *
 * @param p        Output packet.
 * @param seq      Sequence number.
 * @param src_mac  Sender MAC.
 * @param payload  Melody payload.
 */
void command_build_play_melody(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                               const cmd_play_melody_payload_t *payload);

/**
 * @brief Build a MSG_COMMAND packet carrying a single-tone buzz directive.
 *
 * @param p        Output packet.
 * @param seq      Sequence number.
 * @param src_mac  Sender MAC.
 * @param payload  Buzz payload.
 */
void command_build_buzz(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                        const cmd_buzz_payload_t *payload);

/**
 * @brief Build a MSG_COMMAND packet carrying a note-sequence directive.
 *
 * @param p        Output packet.
 * @param seq      Sequence number.
 * @param src_mac  Sender MAC.
 * @param payload  Sequence payload (note_count must be <= CMD_SEQUENCE_MAX_NOTES).
 */
void command_build_play_sequence(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                 const cmd_play_sequence_payload_t *payload);

/**
 * @brief Build a MSG_COMMAND packet carrying an OLED display-line directive.
 *
 * @param p        Output packet.
 * @param seq      Sequence number.
 * @param src_mac  Sender MAC.
 * @param payload  Display-line payload.
 */
void command_build_display_line(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                const cmd_display_line_payload_t *payload);

#endif /* COMMAND_DISPATCHER_H */
