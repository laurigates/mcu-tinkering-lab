#ifndef GAME_LOGIC_H
#define GAME_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "tag_config.h"

typedef enum {
    TAG_RESULT_UNKNOWN,        // Not in registry
    TAG_RESULT_ALREADY_FOUND,  // Already found this tag
    TAG_RESULT_NEW_FIND,       // New discovery!
} tag_result_t;

/**
 * Initialize game logic and load saved progress from NVS.
 */
esp_err_t game_logic_init(void);

/**
 * Process a scanned tag UID and determine the result.
 *
 * @param uid      Tag UID bytes.
 * @param uid_len  UID length.
 * @param entry    Output: pointer to the matching tag_entry_t (if found).
 * @return TAG_RESULT_UNKNOWN, TAG_RESULT_ALREADY_FOUND, or TAG_RESULT_NEW_FIND.
 */
tag_result_t game_logic_process_tag(const uint8_t *uid, uint8_t uid_len, const tag_entry_t **entry);

/**
 * Get the number of tags found so far.
 */
int game_logic_found_count(void);

/**
 * Get the total number of tags in the hunt.
 */
int game_logic_total_count(void);

/**
 * Check if all tags have been found.
 */
bool game_logic_all_found(void);

/**
 * Reset all progress (NVS cleared). Called on button press.
 */
esp_err_t game_logic_reset(void);

#endif  // GAME_LOGIC_H
