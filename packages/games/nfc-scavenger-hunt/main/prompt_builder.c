#include "prompt_builder.h"

#include <stdio.h>

// clang-format off
#define PROMPT_TEMPLATE \
    "AUDIO PROFILE\n" \
    "Character: Friendly treasure hunt narrator for a young child.\n" \
    "Archetype: Warm, playful guide. Like a kind teacher.\n\n" \
    "DIRECTORS NOTES\n" \
    "Style: Enthusiastic but gentle. Not overwhelming.\n" \
    "Pacing: Moderate and clear. Enunciate for a 3-year-old.\n" \
    "Dynamics: Build excitement on congratulations, then slow " \
    "down for the clue.\n\n" \
    "TRANSCRIPT\n" \
    "%s"
// clang-format on

void prompt_builder_clue(char *buf, size_t buf_len, const char *toy_name, int found, int total,
                         const char *hint)
{
    char transcript[512];

    if (hint != NULL) {
        snprintf(transcript, sizeof(transcript), "Yay! You found the %s! That's %d out of %d! %s",
                 toy_name, found, total, hint);
    } else {
        snprintf(transcript, sizeof(transcript),
                 "You found them ALL! What an amazing treasure hunter! "
                 "You found every single toy! Give yourself a big clap!");
    }

    snprintf(buf, buf_len, PROMPT_TEMPLATE, transcript);
}

void prompt_builder_celebration(char *buf, size_t buf_len)
{
    const char *transcript = "You found them ALL! What an amazing treasure hunter! "
                             "You found every single toy! Give yourself a big clap! "
                             "Hooray hooray hooray!";

    snprintf(buf, buf_len, PROMPT_TEMPLATE, transcript);
}
