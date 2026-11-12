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
 *   4. **A recall ring of whole lines, and a repetition check against it.**
 *      The avoid-list above constrains only the first DIALOGUE_OPENING_WORDS
 *      words, so a model can honour it and still say the same thing in a
 *      different order forever — which is exactly what it did.  The recall ring
 *      keeps the last few *complete* utterances: they go into the prompt as
 *      "you already said these", and dialogue_style_is_repetitive() re-checks
 *      the answer against them on-device, because an instruction the model can
 *      quietly ignore is not a guarantee.
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

#include <stdbool.h>
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

/** How many whole spoken lines are remembered and shown back to the model.
 *
 *  Storage is DIALOGUE_RECALL_SLOTS * DIALOGUE_RECALL_MAX bytes of .bss, and
 *  the rendered list is prompt payload on every planner request — so this trades
 *  DRAM and tokens for how far back the robot's memory of itself reaches. Five
 *  lines at a ~60 s speaking floor covers roughly the last five minutes. */
#define DIALOGUE_RECALL_SLOTS 5

/** Longest remembered line, including NUL.
 *
 *  Shorter than SPEECH_TEXT_MAX (320) on purpose: the tail of a sentence is the
 *  least useful part for "have I said this already", and five full-length slots
 *  would put 1.6 kB into every prompt. A line longer than this is stored cut at
 *  a word boundary — never mid multi-byte sequence, since the stored text goes
 *  straight into a JSON request body. */
#define DIALOGUE_RECALL_MAX 160

/** Default similarity (percent) at or above which a candidate line counts as a
 *  repeat of something already said. Sørensen–Dice over the two word sets, so
 *  100 is "same words", 0 is "nothing in common".
 *
 *  60 is a starting point, not a derived constant — how much repetition is
 *  tolerable is a judgement only somebody listening to the robot can make, which
 *  is why dialogue_style_set_repeat_pct() exists and the console exposes it. */
#define DIALOGUE_REPEAT_PCT_DEFAULT 60

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
 *        for a different opening — and so the line itself can be recalled.
 *
 * Feeds two rings: the openings ring (first DIALOGUE_OPENING_WORDS words, with
 * trailing punctuation trimmed) and the recall ring (the whole line, cut at a
 * word boundary to DIALOGUE_RECALL_MAX).  A repeat of the newest stored opening
 * is ignored rather than stored twice — otherwise one stubborn phrase would
 * fill the ring and crowd out the older openings that still need avoiding.
 *
 * Call it for lines the robot generates, not for lines a human typed at the
 * console (`voice say`), which are auditions rather than dialogue.
 */
void dialogue_style_note_spoken(const char *line);

/**
 * @brief The remembered whole lines as a quoted, comma-separated list.
 *
 * Newest first, so a truncating buffer keeps the most recent — the ones the
 * listener would notice being repeated. Writes "" and returns 0 when nothing
 * has been recorded, which the caller must treat as "omit the clause entirely"
 * rather than emitting a dangling lead-in.
 */
size_t dialogue_style_recent_lines(char *out, size_t out_len);

/**
 * @brief How similar @p a and @p b are, 0..100 (Sørensen–Dice over word sets).
 *
 * Word comparison ignores punctuation and ASCII case. Non-ASCII bytes are
 * compared as-is: case-folding Finnish ä/ö would need a table, and the only
 * place it would matter is a sentence-initial capital, which changes one word
 * out of a dozen. Exposed mainly so the threshold can be judged from real
 * transcripts in a host test.
 */
unsigned dialogue_style_similarity(const char *a, const char *b);

/**
 * @brief True when @p line repeats something in the recall ring.
 *
 * The on-device half of the anti-repetition contract: the prompt *asks* the
 * model not to repeat itself, this *checks*. Delivery tags are stripped before
 * comparing, so re-saying a line with a different `[sighs]` still counts as a
 * repeat.
 *
 * Apply it to model-generated lines only. A status report whose facts have not
 * changed is supposed to read the same, and a console audition is meant to be
 * repeatable on demand.
 */
bool dialogue_style_is_repetitive(const char *line);

/**
 * @brief Set the similarity percentage that counts as a repeat (1..100).
 *
 * Values outside the range are ignored. 100 effectively means "only reject a
 * verbatim repeat". Tunable at runtime because the right value is a matter of
 * taste — see DIALOGUE_REPEAT_PCT_DEFAULT.
 */
void dialogue_style_set_repeat_pct(uint8_t pct);

/** @brief Current repeat threshold, as set by dialogue_style_set_repeat_pct(). */
uint8_t dialogue_style_repeat_pct(void);

/**
 * @brief The remembered openings as a quoted, comma-separated list.
 *
 * Exposed for tests and for the console; dialogue_style_directive() appends
 * this itself.  Writes "" and returns 0 when nothing has been recorded.
 */
size_t dialogue_style_recent_openings(char *out, size_t out_len);

/** @brief Forget all recorded openings and lines (persona switch, or tests). */
void dialogue_style_reset_recent(void);

#ifdef __cplusplus
}
#endif

#endif /* DIALOGUE_STYLE_H */
