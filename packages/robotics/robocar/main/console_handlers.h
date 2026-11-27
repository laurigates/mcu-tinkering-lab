/**
 * @file console_handlers.h
 * @brief Console command handlers outside main.c.
 *
 * Each takes the whole console line. execute_console_line() in main.c calls
 * them by the command console_cmd_lookup() returns (console_cmd_core.h), for
 * the serial console and the MQTT command topic alike. A handler reachable in
 * MQTT read-only mode is one whose bare line is on mqtt_command.c's allow-list;
 * read the handler end to end before adding a line there (#626).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* console_voice.c */
void handle_voice_cmd(const char *buf);
void handle_mic_cmd(const char *buf);
void handle_listen_cmd(const char *buf);

/* console_planner.c */
void handle_plan_cmd(const char *buf);
void handle_idle_cmd(const char *buf);
void handle_quest_cmd(const char *buf);
void handle_report_cmd(const char *buf);

/* console_devices.c */
void handle_gpio_cmd(const char *buf);
void handle_cam_cmd(const char *buf);
void handle_snap_cmd(const char *buf);
void handle_trace_cmd(const char *buf);
void handle_cliff_cmd(const char *buf);
void handle_ota_cmd(const char *buf);

#ifdef __cplusplus
}
#endif
