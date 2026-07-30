# Gemini API: Model-ID / thinkingConfig / Token-Budget Drift, and Prompted TTS Style

The Google Gemini API (`generativelanguage.googleapis.com`) is called from
several projects here — `robocar/unified`, `robocar/camera`,
`camera-vision/gemini-vision`, `thinkpack/brainbox`, `games/nfc-scavenger-hunt`.
Its request surface **moves under you**: ids get suffixes, config fields get
renamed, and moving aliases (`*-latest`) repoint to newer model generations with
different rules. Firmware written against an older shape then fails with a fast,
authoritative HTTP error — categorically different from a network hang, and the
fix is always *ask the API*, never *reason from memory of its shape*. Every item
below cost a debugging loop in the robocar-unified bring-up (2026-07).

## 1. Model ids drift — verify against ListModels, never guess

A bare `gemini-robotics-er-1.6` **404s** on `v1beta`
(`"... is not found for API version v1beta, or is not supported for
generateContent"`); the real id is `gemini-robotics-er-1.6-preview`.
`gemini-flash-latest` is a *moving alias* now resolving to a Gemini 3-era model.
Don't invent a suffix — list what the key can actually reach:

```sh
curl -s -H "x-goog-api-key: $KEY" \
  "https://generativelanguage.googleapis.com/v1beta/models?pageSize=1000" \
  | jq -r '.models[] | select(.supportedGenerationMethods[]? == "generateContent") | .name'
```

A 404 with a JSON error body naming `generateContent` is a **wrong id**, not a
down endpoint. Confirm the id against this list before touching anything else.

## 2. `thinkingConfig`: it's `thinkingLevel`, not `thinkingBudget`

Gemini 3-era models **reject** the numeric `thinkingConfig.thinkingBudget` with a
bare `HTTP 400 "Request contains an invalid argument"` that names no field. Use
the string form instead:

```jsonc
"generationConfig": { "thinkingConfig": { "thinkingLevel": "low" } }
```

The 400 is opaque by default because it names nothing — **log the response body
at ERROR level**, not DEBUG, or you get a status code and no clue which field the
API rejected. (robocar-unified's narrate path hid this at `ESP_LOGD` for exactly
that reason.)

## 3. `maxOutputTokens` is a *combined* budget — thinking is spent first, and varies

`maxOutputTokens` covers **thinking tokens + reply tokens**, and Gemini 3-era
models always think. Too small a cap spends the whole budget thinking and returns
`finishReason: MAX_TOKENS` with the reply truncated mid-word — or empty. Thinking
is **variable**: measured 487–745 thought tokens for a *24-token* reply, and a
non-English / persona-styled prompt costs more than plain English (~395). Size
generously (e.g. 2048 for a one-sentence reply); raising the cap is nearly free
because thinking tokens are billed whether or not the cap truncates the reply —
the cap only decides whether the sentence survives. Size from **measurement**
(`usageMetadata.thoughtsTokenCount`), not a guess.

## 4. Free-tier quota is per-model RPM — pace fixed-rate loops under it

Free tier is a per-model requests-per-minute cap (e.g.
`gemini-robotics-er-1.6-preview` = **5 req/min**, quota
`GenerateRequestsPerMinutePerProjectPerModel-FreeTier`). A fixed-rate polling
loop (robocar's 1 Hz planner = 60 RPM) self-inflicts continuous `HTTP 429`, and
the `retryDelay` in the body **grows while you keep asking** — so retrying at
full rate lengthens the outage. Pace the loop under the cap and back off on 429.

## 5. TTS delivery style is *prompted*, not a parameter

For speech generation (`gemini-3.1-flash-tts-preview` etc.) there is no
style/accent/era field. Delivery is steered by prefixing a natural-language
directive to the text: `"<style directive>: <text to speak>"`. The directive is
**interpreted, not read aloud** (verified: a ~40-word directive, 12 s if spoken,
added 0.36 s of audio). Language is `speechConfig.languageCode` (BCP-47 — `fi-FI`
works). The prebuilt voices (`Kore`, `Puck`, `Charon`, …) are **language-agnostic**;
the locale-specific `fi-FI-Chirp3-HD-*` names are **Cloud TTS**, a different
product, and **404** on the Gemini `generateContent` endpoint — do not put one in
a request without testing it against `:generateContent` first.

## Meta

An API `404`/`400`/`429` returns a fast, authoritative JSON error body. Read it
(`jq .error.message`), and for anything id- or capability-shaped, re-derive the
truth from ListModels — the shape in your head is stale. This is
`diagnose-at-the-failure-point` applied to a remote API: the failing call's own
response is the source of truth, above any remembered request shape.

## Related

- `stateless-model-gating.md` — the *design* companion to this file's *API*
  gotchas: every call here is stateless, so an instruction naming a fact the
  model cannot access (what it saw last time, what it already said) is
  unobeyable no matter how it is worded. Withhold the tool instead. Also covers
  the fixed prompt-buffer truncation that silently eats the closing constraint.
