/**
 * @file base64.h
 * @brief Base64 encoding for image data
 */

#ifndef BASE64_H
#define BASE64_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Calculate the size needed for base64 encoded data
 * @param input_length Length of input data
 * @return Required buffer size for base64 output
 */
size_t base64_encode_length(size_t input_length);

/**
 * @brief Encode data to base64
 * @param input Input data buffer
 * @param input_length Length of input data
 * @param output Output buffer (must be large enough)
 * @param output_length Pointer to store actual output length
 * @return 0 on success, -1 on error
 */
int base64_encode(const uint8_t* input, size_t input_length, char* output, size_t* output_length);

/**
 * @brief Encode data to base64 with memory allocation
 * @param input Input data buffer
 * @param input_length Length of input data
 * @return Allocated base64 string (caller must free), NULL on error
 */
char* base64_encode_alloc(const uint8_t* input, size_t input_length);

#endif // BASE64_H