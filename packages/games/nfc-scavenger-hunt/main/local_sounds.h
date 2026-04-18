#ifndef LOCAL_SOUNDS_H
#define LOCAL_SOUNDS_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    SOUND_STARTUP,        // Power-on jingle
    SOUND_HAPPY_CHIME,    // New tag found
    SOUND_ALREADY_FOUND,  // Double beep — tag already discovered
    SOUND_UNKNOWN_TAG,    // Gentle buzz — unknown tag
    SOUND_ERROR,          // Error tone — WiFi/API failure
} sound_id_t;

/**
 * Get a pointer to a generated PCM tone.
 *
 * @param id          Which sound to get.
 * @param pcm_data    Output: pointer to 16-bit PCM data (mono, 24kHz).
 * @param pcm_len     Output: length in bytes.
 */
void local_sounds_get(sound_id_t id, const int16_t **pcm_data, size_t *pcm_len);

/**
 * Generate all tones into static buffers.
 * Call once at startup.
 */
void local_sounds_init(void);

#endif  // LOCAL_SOUNDS_H
