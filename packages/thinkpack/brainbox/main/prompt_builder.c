/**
 * @file prompt_builder.c
 * @brief Prompt construction for ThinkPack Brainbox LLM queries.
 *
 * Uses snprintf-append style (same pattern as nfc-scavenger-hunt) to
 * assemble a prompt in a single output buffer without heap allocation.
 */

#include "prompt_builder.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/**
 * @brief Append a formatted string to buf[*pos .. out_size-1].
 *
 * Updates *pos by the number of characters written (or that would have
 * been written if truncated).  Always NUL-terminates within out_size.
 *
 * @return Total bytes written so far (may exceed out_size on overflow).
 */
static int append(char *buf, size_t out_size, int *pos, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

static int append(char *buf, size_t out_size, int *pos, const char *fmt, ...)
{
    if (*pos < 0 || out_size == 0) {
        return *pos;
    }
    size_t remaining = (size_t)*pos < out_size ? out_size - (size_t)*pos : 0;
    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(buf + *pos, remaining, fmt, ap);
    va_end(ap);
    if (written > 0) {
        *pos += written;
    }
    return *pos;
}

/**
 * @brief Return a human-readable box-type name for the given type byte.
 */
static const char *box_type_name(uint8_t box_type)
{
    switch (box_type) {
        case 1:
            return "chatterbox";
        case 2:
            return "glowbug";
        case 3:
            return "brainbox";
        case 4:
            return "boombox";
        case 5:
            return "finderbox";
        default:
            return "unknown";
    }
}

/**
 * @brief Append a comma-separated list of capability names for a bitmask.
 */
static void append_capabilities(char *buf, size_t out_size, int *pos, uint16_t caps)
{
    /* CAP_* bit positions from thinkpack_protocol.h */
    static const struct {
        uint16_t bit;
        const char *name;
    } kCaps[] = {
        {1u << 0, "wifi"},     {1u << 1, "llm"},      {1u << 2, "audio_out"}, {1u << 3, "audio_in"},
        {1u << 4, "display"},  {1u << 5, "led_ring"}, {1u << 6, "imu"},       {1u << 7, "light"},
        {1u << 8, "nfc"},      {1u << 9, "pots"},     {1u << 10, "touch"},    {1u << 11, "rhythm"},
        {1u << 12, "storage"},
    };
    bool first = true;
    for (size_t i = 0; i < sizeof(kCaps) / sizeof(kCaps[0]); i++) {
        if (caps & kCaps[i].bit) {
            append(buf, out_size, pos, "%s%s", first ? "" : ", ", kCaps[i].name);
            first = false;
        }
    }
    if (first) {
        append(buf, out_size, pos, "none");
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int prompt_builder_collective(const char *trigger, const group_manifest_t *manifest, char *out,
                              size_t out_size)
{
    if (!trigger || !manifest || !out || out_size == 0) {
        return -1;
    }

    int pos = 0;

    /* --- System context ------------------------------------------- */
    append(out, out_size, &pos,
           "You are the Brainbox — the coordinator of a ThinkPack modular toy group designed for "
           "toddlers aged 2-4 years.\n"
           "\n"
           "ThinkPack boxes connect wirelessly and play together. Each box type has different "
           "physical outputs:\n"
           "  glowbug  — addressable LED ring, produces colourful light animations\n"
           "  boombox  — speaker, plays melodies and rhythm patterns\n"
           "  chatterbox — speaker, plays speech and sound effects\n"
           "  finderbox — NFC reader, detects tagged objects\n"
           "  brainbox — WiFi + display, the group coordinator (you)\n"
           "\n");

    /* --- Trigger --------------------------------------------------- */
    append(out, out_size, &pos, "EVENT: %s\n\n", trigger);

    /* --- Group manifest -------------------------------------------- */
    append(out, out_size, &pos, "CURRENT GROUP (%zu box%s):\n", manifest->peer_count,
           manifest->peer_count == 1 ? "" : "es");

    for (size_t i = 0; i < manifest->peer_count && i < 8; i++) {
        const char *type_name = box_type_name(manifest->peers[i].box_type);
        append(out, out_size, &pos, "  [%zu] %s", i + 1,
               manifest->peers[i].name[0] ? manifest->peers[i].name : type_name);
        append(out, out_size, &pos, " (type=%s, capabilities=", type_name);
        append_capabilities(out, out_size, &pos, manifest->peers[i].capabilities);
        append(out, out_size, &pos, ")\n");
    }
    if (manifest->peer_count == 0) {
        append(out, out_size, &pos, "  (no peers — Brainbox is alone)\n");
    }

    /* --- Task ------------------------------------------------------ */
    append(out, out_size, &pos,
           "\n"
           "TASK: Choose a brief, joyful collective-play moment appropriate for toddlers. "
           "Coordinate the boxes listed above into a short, delightful interaction.\n"
           "\n"
           "SAFETY CONSTRAINTS (mandatory):\n"
           "  - LED brightness must be <= 60%% (r, g, b values <= 153)\n"
           "  - No strobe or rapid flicker effects — use breathe, rainbow, sparkle, or solid\n"
           "  - Melody repeat_count must be <= 32 beats\n"
           "  - Note sequences must be <= 20 notes\n"
           "\n"
           "OUTPUT FORMAT: Respond with ONLY a valid JSON object — no explanation, no markdown "
           "fences. Schema:\n"
           "{\n"
           "  \"commands\": [\n"
           "    {\"target\": \"<box_type>\", \"type\": \"led_pattern\",\n"
           "     \"r\": <0-153>, \"g\": <0-153>, \"b\": <0-153>,\n"
           "     \"pattern\": \"breathe|rainbow|sparkle|nightlight|solid\"},\n"
           "    {\"target\": \"<box_type>\", \"type\": \"play_melody\",\n"
           "     \"pattern_id\": <0-3>, \"repeat_count\": <1-32>},\n"
           "    {\"target\": \"<box_type>\", \"type\": \"buzz\",\n"
           "     \"note\": <0-127>, \"duration_ms\": <50-2000>}\n"
           "  ]\n"
           "}\n"
           "\n"
           "Only include commands for box types present in the current group. "
           "Respond ONLY with the JSON object.\n");

    /* pos may exceed out_size when truncated; return total chars that would
     * have been written (matches snprintf convention). */
    return pos;
}
