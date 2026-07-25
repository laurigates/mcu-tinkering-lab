/**
 * @file dialogue_style.h
 * @brief Per-utterance variation: how the robot avoids saying it the same way.
 *
 * The problem
 * -----------
 * Both spoken paths (the planner's `speak` call and the self-report narration)
 * send an essentially identical prompt every time, so the model settles on one
 * favourite phrasing and repeats it forever — every fi-1950 line opening with
 * "Asianlaita on oikeastaan niin, että ...".  Temperature does not fix this:
 * the repetition comes from the *prompt* naming one construction, not from the
 * sampler being too cold.
 *
 * The approach
 * ------------
 * Variety is bought on-device, for the price of a few hundred prompt bytes and
 * no extra API calls:
 *
 *   1. **Rotating directives.**  Each persona carries pools of one-line
 *      directives — an *opener* pool ("start with a hesitation filler", "start
 *      straight in with no opening word at all") and a *shape* pool ("keep it
 *      under ten words", "make it a polite remark").  One of each is drawn per
 *      utterance and appended to the prompt, so N openers x M shapes gives NxM
 *      distinct briefs from a table that costs a few hundred bytes of rodata.
 *
 *   2. **No immediate repeats.**  Each pool remembers its previous draw and
 *      never draws it twice running (see dialogue_slot_t).
 *
 *   3. **An avoid-list of what was actually said.**  The opening words of the
 *      last few spoken lines are fed back into the next prompt as "do not begin
 *      with these" — the only mechanism here that reacts to what the model
 *      really produced rather than to what it was asked for.
 *
 * The same pools also back the canned fallback lines, so an API outage does not
 * collapse the robot back onto one fixed sentence.
 *
 * Concurrency
 * -----------
 * Deliberately free of FreeRTOS and ESP-IDF so it can be unit-tested on the
 * host; the RNG is seeded via dialogue_style_seed() rather than calling
 * esp_random() internally.  The module state is therefore *unlocked*, and the
 * planner and self-report tasks both touch it.  That is safe but not atomic: a
 * race can only produce a slightly odd directive in one prompt, never memory
 * damage, because every recent-opening slot keeps its final byte NUL so a
 * reader interleaved with a writer still sees a bounded, terminated string.
 * Do not extend this module with state where a torn read would matter.
 */

#ifndef DIALOGUE_STYLE_H
#define DIALOGUE_STYLE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Longest composed directive, including NUL.  Sized for an opener plus a
 *  shape plus the avoid-list clause, in Finnish (the longer of the two). */
#define DIALOGUE_STYLE_MAX 512

/** How many recently spoken openings are fed back as "do not start with". */
#define DIALOGUE_RECENT_SLOTS 3

/** Longest remembered opening, including NUL. */
#define DIALOGUE_OPENING_MAX 32

/** Words of a spoken line treated as its "opening". */
#define DIALOGUE_OPENING_WORDS 3

/**
 * @brief A pool of interchangeable strings, one of which is drawn per use.
 *
 * All entries are string literals with static storage, so a pointer returned
 * from dialogue_style_pick() stays valid for the life of the program.
 * At most 255 entries — the no-immediate-repeat state stores an index byte.
 */
typedef struct {
    const char *const *items;
    size_t count;
} dialogue_pool_t;

/** Entry count of a literal pool array, for the `count` field:
 *  `.openers = {s_fi_openers, DIALOGUE_POOL_COUNT(s_fi_openers)}`.
 *
 *  A count macro rather than a whole-initialiser one: a macro expanding to a
 *  braced list is formatted differently by different clang-format 18.x point
 *  releases, so it fails the format gate on CI while passing locally. */
#define DIALOGUE_POOL_COUNT(arr) (sizeof(arr) / sizeof((arr)[0]))

/**
 * @brief Which pool a draw belongs to, for no-immediate-repeat bookkeeping.
 *
 * Pools are `const` table data, so the "what did I draw last time" byte cannot
 * live in dialogue_pool_t.  It lives here instead, keyed by role — two personas
 * sharing a role share the slot, which is what you want: switching persona
 * mid-run should not hand back the phrasing just used.
 */
typedef enum {
    DIALOGUE_SLOT_OPENER = 0, /**< How the line starts. */
    DIALOGUE_SLOT_SHAPE,      /**< What shape/length the line takes. */
    DIALOGUE_SLOT_FALLBACK,   /**< Canned line used when the API call fails. */
    DIALOGUE_SLOT_COUNT
} dialogue_slot_t;

/**
 * @brief Seed the internal PRNG.  Call once at startup with entropy.
 *
 * Without this the sequence is identical after every boot, which shows up
 * exactly where it is most audible: the self-introduction spoken at power-on
 * would use the same opener every time.  A zero seed is replaced with a
 * non-zero constant (xorshift is absorbing at zero).
 */
void dialogue_style_seed(uint32_t seed);

/**
 * @brief Draw one entry from @p pool, never the same as the previous draw for
 *        the same @p slot.
 *
 * @return A pointer into the pool, or NULL when the pool is empty/NULL.
 */
const char *dialogue_style_pick(const dialogue_pool_t *pool, dialogue_slot_t slot);

/**
 * @brief Compose the variation directive to append to a generation prompt.
 *
 * Draws one opener and one shape, then appends the avoid-list clause built
 * from previously spoken openings (omitted entirely while nothing has been
 * spoken yet, or when @p avoid_lead is NULL).
 *
 * @param openers    Opener directive pool (may be NULL/empty).
 * @param shapes     Sentence-shape directive pool (may be NULL/empty).
 * @param avoid_lead Localised lead-in, e.g. "Do not begin the sentence with
 *                   any of these:".  The openings and a full stop are appended.
 * @param out        Destination; always NUL-terminated when out_len > 0.
 * @param out_len    Size of @p out.
 * @return Length written, excluding the NUL.  0 means "nothing to say", and
 *         the caller should skip the directive rather than emit an empty line.
 */
size_t dialogue_style_directive(const dialogue_pool_t *openers, const dialogue_pool_t *shapes,
                                const char *avoid_lead, char *out, size_t out_len);

/**
 * @brief Record a line the robot actually spoke, so the next prompt can ask
 *        for a different opening.
 *
 * Stores only the first DIALOGUE_OPENING_WORDS words, with trailing
 * punctuation trimmed.  A repeat of the newest stored opening is ignored
 * rather than stored twice — otherwise one stubborn phrase would fill the ring
 * and crowd out the older openings that still need avoiding.
 *
 * Call it for lines the robot generates, not for lines a human typed at the
 * console (`voice say`), which are auditions rather than dialogue.
 */
void dialogue_style_note_spoken(const char *line);

/**
 * @brief The remembered openings as a quoted, comma-separated list.
 *
 * Exposed for tests and for the console; dialogue_style_directive() appends
 * this itself.  Writes "" and returns 0 when nothing has been recorded.
 */
size_t dialogue_style_recent_openings(char *out, size_t out_len);

/** @brief Forget all recorded openings (persona switch, or tests). */
void dialogue_style_reset_recent(void);

#ifdef __cplusplus
}
#endif

#endif /* DIALOGUE_STYLE_H */
