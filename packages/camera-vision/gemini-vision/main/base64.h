/**
 * @file base64.h
 * @brief Base64 encoding for image data
 */

#ifndef BASE64_H
#define BASE64_H

#include <stddef.h>
#include <stdint.h>

size_t base64_encode_length(size_t input_length);
int base64_encode(const uint8_t *input, size_t input_length, char *output, size_t *output_length);
char *base64_encode_alloc(const uint8_t *input, size_t input_length);

#endif  // BASE64_H
