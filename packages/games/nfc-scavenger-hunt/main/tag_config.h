#ifndef TAG_CONFIG_H
#define TAG_CONFIG_H

#include <stdint.h>

#define MAX_UID_LEN 7
#define MAX_TAGS 10

typedef struct {
    uint8_t uid[MAX_UID_LEN];
    uint8_t uid_len;
    const char *name;
    const char *hint;  // NULL for final tag (triggers celebration)
    uint8_t track_id;
} tag_entry_t;

// clang-format off

/*
 * Tag registry — fill in real UIDs after scanning each toy.
 *
 * Discovery workflow:
 *   1. Flash firmware with these placeholder UIDs.
 *   2. Open serial monitor (idf.py monitor).
 *   3. Tap each NFC-tagged toy against the reader.
 *   4. Firmware logs: "Unknown tag: 04:A2:B3:C4:D5:E6:F7"
 *   5. Copy real UIDs here, write matching hints.
 *   6. Reflash.
 */
static const tag_entry_t TAG_REGISTRY[] = {
    {
        .uid = {0x04, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6},
        .uid_len = 7,
        .name = "dinosaur",
        .hint = "Now look for something soft and cuddly, sitting on your bed!",
        .track_id = 1,
    },
    {
        .uid = {0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66},
        .uid_len = 7,
        .name = "teddy bear",
        .hint = "Find something red with a siren, near the front door!",
        .track_id = 2,
    },
    {
        .uid = {0x04, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC},
        .uid_len = 7,
        .name = "fire truck",
        .hint = "Look for something that floats in the bath!",
        .track_id = 3,
    },
    {
        .uid = {0x04, 0xDD, 0xEE, 0xFF, 0x01, 0x02, 0x03},
        .uid_len = 7,
        .name = "rubber duck",
        .hint = "Find the one that can roll and beep, in the toy box!",
        .track_id = 4,
    },
    {
        .uid = {0x04, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
        .uid_len = 7,
        .name = "robot",
        .hint = NULL,  // Final tag — triggers celebration
        .track_id = 5,
    },
};

// clang-format on

#define TAG_COUNT (sizeof(TAG_REGISTRY) / sizeof(TAG_REGISTRY[0]))

#endif  // TAG_CONFIG_H
