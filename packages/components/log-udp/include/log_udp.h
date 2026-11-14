/**
 * @file log_udp.h
 * @brief WiFi UDP log sink — forwards ESP_LOG output to a UDP listener.
 *
 * Starts the ESP32 as a WiFi access point named by CONFIG_LOG_UDP_AP_SSID
 * (xbox-switch-bridge: "xbox-bridge-log", facedancer-espdancer-fw:
 * "espdancer-log"). Connect your Mac/PC to that network, then listen for UDP
 * broadcast:
 *
 *   socat UDP-RECV:4444 STDOUT
 *   nc -u -l 4444
 *
 * No router needed. ESP32 AP default subnet: 192.168.4.x.
 * All ESP_LOG* output continues to UART as well (both sinks are active).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Start WiFi AP and broadcast logs over UDP.
 *
 * Must be called after nvs_flash_init(), and before the consumer starts its
 * other radio or USB stacks (xbox-switch-bridge: bp32_host_init();
 * facedancer-espdancer-fw: USB init).
 *
 * @param port  UDP port to broadcast on (e.g. 4444).
 * @return ESP_OK on success.
 */
esp_err_t log_udp_init(uint16_t port);
