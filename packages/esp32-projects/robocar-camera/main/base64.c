/**
 * @file base64.c
 * @brief Base64 encoding implementation
 */

#include "base64.h"
#include <stdlib.h>
#include <string.h>

static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t base64_encode_length(size_t input_length)
{
    return ((input_length + 2) / 3) * 4 + 1;  // +1 for null terminator
}

int base64_encode(const uint8_t *input, size_t input_length, char *output, size_t *output_length)
{
    if (!input || !output || !output_length) {
        return -1;
    }

    size_t required_length = base64_encode_length(input_length);
    if (*output_length < required_length) {
        return -1;
    }

    size_t i = 0;
    size_t j = 0;
    uint8_t char_array_3[3];
    uint8_t char_array_4[4];

    while (i < input_length) {
        char_array_3[0] = input[i++];
        char_array_3[1] = (i < input_length) ? input[i++] : 0;
        char_array_3[2] = (i < input_length) ? input[i++] : 0;

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (int k = 0; k < 4; k++) {
            output[j++] = base64_chars[char_array_4[k]];
        }
    }

    // Handle padding
    if (input_length % 3 == 1) {
        output[j - 2] = '=';
        output[j - 1] = '=';
    } else if (input_length % 3 == 2) {
        output[j - 1] = '=';
    }

    output[j] = '\0';
    *output_length = j;
    return 0;
}

char *base64_encode_alloc(const uint8_t *input, size_t input_length)
{
    if (!input) {
        return NULL;
    }

    size_t output_length = base64_encode_length(input_length);
    char *output = malloc(output_length);
    if (!output) {
        return NULL;
    }

    if (base64_encode(input, input_length, output, &output_length) != 0) {
        free(output);
        return NULL;
    }

    return output;
}
