/**
 * @file gemini_client.h
 * @brief Thin HTTPS client for Gemini Robotics-ER 1.5 object detection.
 */

#ifndef GEMINI_CLIENT_H
#define GEMINI_CLIENT_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Run object detection on a JPEG image.
 *
 * POSTs the image to gemini-robotics-er-1.5-preview with a prompt requesting a
 * JSON array of {label, box_2d:[ymin,xmin,ymax,xmax]} entries with coordinates
 * normalized 0-1000.
 *
 * On success, `out_json` contains just the detection array (suitable to embed
 * directly under an "objects" key in the /metadata response) and
 * `*out_latency_ms` is the round-trip duration.
 *
 * @param jpeg            JPEG payload
 * @param jpeg_len        JPEG length in bytes
 * @param out_json        Caller-allocated output buffer (UTF-8, null-terminated)
 * @param out_json_size   Size of out_json buffer
 * @param out_latency_ms  Round-trip latency in milliseconds (may be NULL)
 * @return ESP_OK on success; otherwise an error code (buffer may hold an error message)
 */
esp_err_t gemini_detect(const uint8_t *jpeg, size_t jpeg_len, char *out_json, size_t out_json_size,
                        uint32_t *out_latency_ms);

#endif  // GEMINI_CLIENT_H
