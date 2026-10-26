# PRD-014: Teuvo Sponsored Upgrades — Commissioned Feature Planning Through Conversation

**Status**: draft
**Created**: 2026-10-23
**Confidence**: 6/10, self-assessed. Legal points are interpretation, not
legal advice, and no part of the flow has run. Unverified: that a Payment
Link's consent checkbox reaches the Checkout Session (Flow), Anthropic's
reading of intermediation (Constraints), the `__REQUEST__` format on the live
model (FR-024), the cost of a planning run (FR-028) and every lead time
(FR-031).

---

## Overview

A person talking to Teuvo suggests a new feature, either hardware or software.
Teuvo quotes a fixed planning fee. When it is paid, a Claude session on this
repo writes an upgrade plan and opens a draft PR. The plan covers parts, a
schematic where the parts allow one, a cost estimate including JLCPCB and
Lauri's build time, and lead times. Teuvo hands the person an order code and a
link. Later he tells them, in his own manner, when parts are expected and when
the build might be done.

Lauri gates the summary text that is published, every payment confirmation at
first, every planning run and every purchase. To the person talking to Teuvo,
the flow looks the way it would if it were fully automated.

## Problem

None of the pieces exist today:

- **Nothing captures the request.** The voice turn returns free text; its only
  structured output is the `__IGNORE__` sentinel (`voice_turn.c:494-505`). The
  user's words are never transcribed. History replays each user turn as the
  literal "(User spoke to you)" (`voice_history.c:190`), and ESP_LOG lines do
  not reach MQTT. A spoken feature request leaves no text anywhere.
- **Nothing tells Teuvo facts from a server.** The only downlink is
  `robocar/commands`: 63-byte printable-ASCII console lines, so no ä/ö
  (`mqtt_command.h:33`). Nothing puts "order K7Q2: parts shipped" into Teuvo's
  context. The device has no wall clock (no SNTP), so it cannot compute "next
  week" itself.
- **Nothing turns a request into a PR.** `claude.yml` ignores bot senders and
  needs an `@claude` mention (`claude.yml:13-22`). It runs on `ubuntu-slim`,
  where single-CPU jobs are killed at 15 minutes.
- **There is no payment rail and no link channel.** `robocar-unified` has no
  OLED code; `robocar/bringup/main/oled.c` drives the same 128x32 panel, as
  text only (#780 tracks its geometry). With the ISO quiet zone the panel fits
  only a version-1 QR code (21 modules), about 17 bytes, far short of a PR or
  payment URL.

## Constraints that shape the design

These are legal interpretations for planning, not legal advice. Sources were
read on 2026-10-23.

| Constraint | Consequence | Source |
|---|---|---|
| Private MobilePay may not send or receive money for commercial or business purposes | Lauri's personal MobilePay cannot take these payments; every Vipps MobilePay merchant product needs a Y-tunnus and a business account | MobilePay terms v1.10 §4.4 (20.10.2026); developer.vippsmobilepay.com applying-for-services |
| A collection from the public with nothing given in return is a *rahankeräys*, which a single private person may not run | The planning fee is sold as a **commissioned design service**: the payer receives the plan. Never called *lahjoitus* (donation). A build fee needs its own consideration (see `BUILD_QUOTED` below) | Rahankeräyslaki 863/2019 §2; Vero guide on pienkeräys (2022) |
| A standing, priced, public offer very likely counts as business activity (*elinkeinotoiminta*) | Lauri is a trader under KSL chapter 6. He owes pre-contract information, a 14-day withdrawal right, and a withdrawal function reachable for the whole period (6:14 a, in force 19.6.2026). Planning may start inside the withdrawal period only on the consumer's express request and acknowledgement (6:15–6:16). Before planning starts, he confirms each contract on a durable medium with the pre-contract information and that request (6:13) | Vero VH/8236/00.01.00/2024; KSL 38/1978 (Finlex) |
| No VAT is due while turnover is at most €20,000 in the current and previous year | No VAT on invoices until then. The threshold counts all of Lauri's business turnover, not only this desk's, and resold parts count toward it | Vero VH/1581/00.01.00/2024 |
| The repo is public | No names, contact details, invoices or payment references in issues, PRs, dispatch inputs or Actions logs | REST API: `visibility: public` |
| No direct exhortation to children to buy | Teuvo never pitches upgrades. He answers an explicit request from an adult, and the feature is off unless switched on | UCPD 2005/29/EC Annex I point 28 |
| Gemini API users must be 18+, and an API client may not be "likely to be accessed by" under-18s | Applies to Teuvo whatever this PRD does; an open question below | Gemini API Additional Terms (2026-03-23) |
| Developers building products or services "should use API key authentication", and customers offering Claude Code in their services "may not pay for, resell, or intermediate Claude usage on their end users' behalf" | Customer-paid runs bill to a Console organisation, preferably through OIDC federation, otherwise through an API key held only as an environment secret (FR-028). *Interpretation:* whether a run Lauri performs for a paying requester counts as intermediating is open. Confirm with Anthropic sales before Phase 1 | code.claude.com/docs/en/legal-and-compliance |
| Free and open-source software that its manufacturer monetises can count as supplied "in the course of a commercial activity" | The upgrade desk is **off by default** in every published image and enabled per robot over serial. *Interpretation:* off by default narrows the commercial-activity reading but does not remove it. Whether enabling the desk for a paying requester makes the published firmware a commercial supply is open (PRD-015 Open Questions), and a legal opinion (PRD-015 Route C item 9) comes before enabling it on a unit Lauri supplied or changing the default. Payment happens on the payer's phone, never on the robot. This also keeps Teuvo outside RED 3(3)(f), which covers equipment that "enables the holder or user to transfer money" (interpretation) | CRA 2024/2847 recital 15; RED Delegated Reg. 2022/30 Art 1(3) |

## Goals

- Turn a spoken request into a quoted, paid, planned upgrade, with a PR as the
  deliverable, and no step that only works in a demo
- Keep Lauri the gate for the published text, for starting a planning run and
  for any parts purchase. Automating one gate must not automate another
- Every fee, payment state or date Teuvo states comes from a device template
  rendered from server facts. The model never sees a fee, a state or a date;
  the sentence filter on other replies is a best-effort second layer
- Keep personal data and invoices out of the public repo
- Show the whole experience end to end before any real money moves (Phase 0)

## Non-Goals

- Selling Teuvo units. That is [PRD-015](PRD-015-teuvo-for-other-owners.md),
  which is a separate decision
- Payments made by the robot or on the robot
- Automatic parts purchasing. The `jlcpcb-order` and `aliexpress-parts` skills
  stop at the cart, and Lauri pays
- Fab files from a cloud run. Layout, Gerbers and the JLCONE quote stay on
  Lauri's workstation (`LAYOUT` below)
- Merging plan PRs. A plan PR stays a draft until the upgrade is built or
  dropped

## Flow

```mermaid
sequenceDiagram
    participant P as Person
    participant T as Teuvo (robot)
    participant C as Concierge (home server)
    participant L as Lauri (phone)
    participant S as Stripe
    participant G as GitHub Actions
    participant B as Broker
    P->>T: "Teuvo, could you have a gripper?"
    T->>B: teuvo/<id>/upgrade/request {req_id, summary, category} (QoS 1)
    B-->>T: PUBACK
    T->>P: template: "I passed the wish to Lauri"
    B->>C: request (persistent session)
    C->>L: summary text to edit or approve
    L->>C: approve text
    C->>T: facts {code K7Q2, req_id, category, state QUOTED, fee 40}
    T->>P: at the next voice turn, the template line with the fee and the code
    P->>S: pays on the order page (Payment Link)
    S-->>C: paid (Lauri in Phase 1, signed webhook in Phase 2)
    C->>P: contract confirmation e-mail (KSL 6:13)
    C->>G: workflow_dispatch teuvo-plan.yml {code, approved summary}
    G->>L: environment approval request
    L->>G: approve (planning starts)
    G->>G: Claude writes the plan; a fixed job opens the draft PR
    C->>T: facts {state PLAN_READY, ETA phrase}
    T->>P: template line: the plan is ready
```

Order states, held only on the concierge:

```
QUOTED → AWAITING_PAYMENT → PAYMENT_CLAIMED → PAID → APPROVED → PLANNING
  → PLAN_READY → BUILD_QUOTED → BUILD_PAID → LAYOUT → PARTS_ORDERED
  → PARTS_ARRIVED → BUILDING → DONE
exits: EXPIRED, WITHDRAWN, REFUNDED, FAILED
```

- **Who may set `PAID` and `BUILD_PAID`.** Only two writers: Lauri's
  authenticated action and, from Phase 2, a verified Stripe webhook (FR-027).
  Nothing the person says to Teuvo can set them. Every transition is a
  compare-and-set on the current state in one SQLite transaction, so a webhook
  that arrives after Lauri's *mark paid* finds `PAID` and changes nothing.
  `BUILD_PAID` is unreachable while `BUILD_QUOTED` is off; its evidence and
  consent rules are set with the build-fee question.
- **`AWAITING_PAYMENT`, `APPROVED` and `PLANNING`.** The concierge sets
  `AWAITING_PAYMENT` when the order page first serves the order's Payment
  Link. It sets `APPROVED` when the run's `teuvo-planning` deployment is
  approved, and `PLANNING` when the Claude job starts. A run whose deployment
  Lauri rejects, or that times out waiting, returns the order to `PAID`, and it
  is not re-dispatched until Lauri asks.
- **`PAYMENT_CLAIMED`.** The concierge sets it when the payer returns through
  the Payment Link's `after_completion` redirect. It lets Teuvo say "thank you,
  Lauri will confirm" without claiming the money has arrived. Nothing said to
  Teuvo sets it.
- **Before dispatch.** An order records `consent_at` and
  `confirmation_sent_at`, and dispatch requires both. `consent_at` comes from
  the Stripe session's `consent.terms_of_service = accepted`, whose checkbox
  carries the KSL 6:15 request and acknowledgement. Phase 0 checks that the
  field arrives: a sandbox payment through such a link, then a session
  retrieve showing `consent.terms_of_service = "accepted"`. If it does not,
  dispatch stays blocked until consent is captured another way, such as a
  hosted Checkout Session or a checkbox on the order page. After it, the
  concierge e-mails the payer the contract confirmation: the pre-contract
  information, the request and acknowledgement, the withdrawal notice and the
  per-order URL.
- **The exits.** `EXPIRED`, `WITHDRAWN` and `REFUNDED` are terminal. On
  `WITHDRAWN` or `REFUNDED`, the concierge cancels any waiting or running
  planning run for that order. No run outcome moves an order out of an exit.
  Which move leaves `FAILED` (Lauri's *mark refunded* to `REFUNDED`, or a
  re-run back to `PAID`) depends on the Prices open question.
- **`BUILD_QUOTED` needs its own consideration.** If the build goes into
  Lauri's Teuvo, the quote must say what the payer gets for that fee (for
  example their name on the silkscreen, or the built module); otherwise the fee
  is a collection. If it goes into the payer's own robot, Lauri likely becomes
  that unit's manufacturer (Blue Guide §2.1; CRA Art 69(2)), which is PRD-015
  route C. `BUILD_QUOTED` stays off until this is settled.
- **`LAYOUT` is Lauri's local step** on KiCad 10.0.6: `carrier-board-checked`,
  `carrier-check`, `carrier-review`, `carrier-fab`, then the JLCONE quote and
  cart (`jlcpcb-order` skill).

## Features

FR numbers continue in PRD-015 (FR-032–FR-036) and ADR-028 (FR-037–FR-043).
An FR here is a feature area; the testable units are the acceptance criteria
and the issues under Phases.

### FR-024: Upgrade-request capture in the voice turn

When the desk is enabled and MQTT is connected, the voice-turn system prompt
allows a second sentinel next to `__IGNORE__`. MQTT is connected only on a
`home` network (ADR-028 §6). The reply opens with `__REQUEST__`, then one
English line `<category> | <summary ≤160 bytes>`, then the spoken Finnish
reply. A pure-C splitter in `voice_history.c` style separates the parts. The
device publishes `{req_id, device_id, category, summary, lang}` to
`teuvo/<device_id>/upgrade/request` at QoS 1.

- **Explicit requests only.** The model emits the sentinel only for an explicit
  request addressed to Teuvo. It never volunteers or suggests a paid upgrade.
  The instruction also asks it to leave out names; Lauri's text approval
  (FR-026) is the control, this is only a first filter. The line is untrusted
  input: the splitter on the device and the concierge each reject a category
  outside the FR-026 enum or a summary over 160 bytes, with host tests for
  both.
- **No delivery promise from the model.** The prompt offers `__REQUEST__`
  only while MQTT is connected and `teuvo/concierge/status` is `online`;
  otherwise Teuvo says the desk is closed right now. The concierge's MQTT
  session is persistent (clean session off, client id `concierge`, QoS 1
  subscription), so the broker queues requests while it restarts. A broker
  PUBACK means only that the broker accepted the request. The device appends a
  fixed sentence, "Välitin toiveen Laurille" ("I passed the wish to Lauri"),
  after that PUBACK. Without one within 10 s it appends "En saanut viestiä
  perille" ("I couldn't get the message through").
  `CONFIG_MQTT_OUTBOX_EXPIRED_TIMEOUT_MS=8000` (with
  `CONFIG_MQTT_USE_CUSTOM_CONFIG=y`) stops esp-mqtt from delivering a request
  after the timeout has said it failed.
- **Probe first.** Probe the reply format live, with an audio part, on
  `gemini-flash-latest` before building on it. It is a moving alias
  (`.claude/rules/gemini-api.md` §1), so re-probe after it moves. A reply that
  opens with `__REQUEST__` but fails the splitter publishes nothing, and the
  device speaks only "En saanut viestiä perille", so a format drift fails
  closed. Function calling on the voice turn, or Gemini Live input
  transcription (#619), can replace the sentinel later.
- **Topics.** The topics and the broker ACL come from PRD-015 FR-033. Within
  `teuvo/<id>/upgrade/`, the robot may publish only `request` and read only
  `facts`. Only the concierge may write `facts`. `robocar/commands` is never
  used for this.

### FR-025: Order facts downlink and template phrasing

The concierge publishes a UTF-8 JSON document to
`teuvo/<device_id>/upgrade/facts`, at QoS 1 and not retained. It holds at most
4 open orders, paid states first, then newest. For each order it carries the
code, the category (the FR-026 enum), the `req_id` it answers, the state (an
enum), the fee or quote in whole euros, and a pre-rendered Finnish ETA
phrase. The concierge refuses to publish more than 900 bytes.

The firmware subscribes through `mqtt_logger_subscribe()`. Its callback gains
`total_data_len` and `current_data_offset`, so a fragment check like
`mqtt_command_extract_line()`'s can see fragments. `mqtt_logger_init()` sets
`buffer.size` explicitly (the ESP-IDF default is 1024 bytes), large enough
that a 900-byte document with its topic and header arrives in one event. A
fragmented delivery is dropped and counts as no facts; a host test feeds a
900-byte document whole and in fragments and asserts both outcomes. The
firmware validates the document against a fixed schema.

- **Fail closed.** The concierge keeps `seq` in SQLite, so it never repeats
  across restarts. It republishes the facts every 10 minutes with freshly
  rendered ETA phrases, and at once when the robot's status topic reports it
  online (it reads `teuvo/+/status`, PRD-015 FR-033). It also holds a retained
  `teuvo/concierge/status` of `online`, with a retained Last Will of
  `offline`, and publishes `offline` itself on a clean stop. The robot counts a
  document as new when its `seq` differs from the last one it accepted. It
  treats the facts as absent:
  - until the first delivery after boot or after any MQTT disconnect;
  - while the concierge status is not `online`;
  - when no new `seq` has arrived within 20 minutes on its own uptime clock.

  OTA, facts and status take 3 of the 4 subscribe slots. With no facts, Teuvo
  says he will ask Lauri and never guesses.
- **The model sees order codes only.** The facts sentence in the voice-turn
  prompt lists each open code with its category, never a fee, a state or an
  ETA phrase. If two open orders share a category, the model asks for the code
  instead of choosing. When an order comes up, the model emits
  `__ORDER__ <code>`. In such a reply the device drops all model text and
  speaks only the template sentence for that order's state, rendered from the
  facts. When the facts first name a `req_id` this robot published, the device
  speaks that order's template at the next voice turn; that is how the
  requester learns the code and the fee. Conversation history
  (`voice_history_record()`) stores an `__ORDER__` turn as the bare
  `__ORDER__ <code>` line, never the spoken template, so no fee, state or ETA
  reaches a later prompt. Examples:
  - `QUOTED`, `AWAITING_PAYMENT`: the fee, and a pointer to the sticker.
    "Tilauksen K7Q2 suunnittelu maksaa 40 euroa." ("Planning order K7Q2 costs
    40 euros.")
  - `PAYMENT_CLAIMED`: "Kiitoksia! Lauri kuittaa vielä maksun." ("Thank you!
    Lauri will still confirm the payment.")
  - Later states: the ETA phrase.
- **A second layer on every other reply.** While the desk is on, every other
  reply loses any sentence that contains:
  - a digit;
  - an English number word as a whole word (one to ten, hundred, thousand),
    or a Finnish one in any case, including the gradated stems (yksi/yhde-/yhtä,
    kaksi/kahde-/kahta, kolm-, neljä-, viisi/viide-, kuusi/kuude-, seitsem-,
    kahdeks-, yhdeks-, kymmen-, sata/sada-, tuhat/tuhanne-);
  - a money word (hint-/hinn-, maks-, rah-, euro, €, sentti, palkkio,
    kallis/kallii-, halpa/halva-, ilmai-, veloit-, lasku-, kustann-);
  - a time word (huome-, viiko-/viikk-, kuukau-, weekday and month stems);
  - an order-state word (tilau-; tilaa, tilasi, tilannut, tilattu, tilataan;
    kuitta-/kuitat-/kuitan-; vahvist-).

  Finnish stems match on word starts and English number words as whole
  words, case-insensitive with Ä and Ö folded. The filter is pure C under the
  shared-core pattern (`.claude/rules/testing.md`), and the paraphrase corpus
  is committed beside its host test. It holds gradated and inflected forms of
  every stem, and common words the filter must not catch (kuitenkin, tilanne,
  ensin, and tentti and tenava for "ten"). If nothing is left, Teuvo says a
  fixed "Kysyn Laurilta" ("I'll ask Lauri"). This also strips numbers from
  ordinary conversation while the desk is on, which is the price of the desk.
- **Teuvo never reads PR text.** Only enums, numbers and the pre-rendered ETA
  phrase reach the device, so a PR cannot steer him.
- **Before any of this, four truncations are fixed:**
  1. the 511-byte voice-turn brief (`voice_turn.c:292-301`);
  2. the ~190 bytes of TTS text left under the 383-byte style
     (`gemini_tts.c:238-240`);
  3. `s_reply[SPEECH_TEXT_MAX = 320]` (`voice_turn.c:128`), which must hold a
     sentinel line (about 200 bytes with a 160-byte summary) plus the
     spoken reply. It grows by that line, or the parse gets its own buffer;
     the splitter strips the line before `speech_queue_post()`, so the speech
     queue stays at `SPEECH_TEXT_MAX`;
  4. `tele_summary[128]` inside `prompt_text[512]` (`voice_history.c:273-280`),
     which must hold the telemetry summary plus a facts sentence naming 4
     codes at the longest category name.

  The fix includes a host test that budgets every persona string and these four
  buffers, against the migrated TTS request shape (Phases, step 2).

### FR-026: Concierge service

New Python 3.12 + uv project `packages/robocar/concierge/`, using SQLite,
ruff and ty. It runs on Lauri's home server. Its jobs:

- **Requests.** It deduplicates on `req_id` and assigns a 4-character
  Crockford base32 code. It prices from Lauri's fee table (fixed tiers by
  category) and expires quotes after 14 days.
- **Order state.** It owns the order state machine, the two-writer rule and
  the consent and confirmation rules above.
- **Lauri's phone gate.** In every phase, before anything is published or
  dispatched, the notification shows Lauri the exact summary text with *edit*,
  *approve text* and *reject*. Only text he approved is dispatched. Phase 2
  automates `PAID`, never this step. The same channel carries *mark paid*,
  *reject* and *mark refunded*, which records a refund he has already made in
  the Stripe Dashboard. The channel itself is an open question.
- **Planning runs.** It dispatches `teuvo-plan.yml` on `ref: main` with the
  approved summary, one run at a time. The next order is dispatched only when
  the previous run has concluded. It then polls the run and
  `GET /pulls?head=…:teuvo/plan-<code>`.
  - The one-run rule is a single SQLite lease row (order code, dispatch time,
    run id), taken in the transaction that records the dispatch and released
    when the run concludes. `teuvo-plan.yml` puts the code in its `run-name`,
    so after a restart the concierge finds a leased order's run in the
    workflow's run list. A repeat dispatch for a leased order is a no-op, and
    a lease whose run never appears is released after a timeout.
  - A run cancelled while its order is `APPROVED` or `PLANNING` returns the
    order to `PAID`.
  - A run that started the Claude job and concluded without a PR counts as
    `FAILED`, because the SDK exits 0 on a permission denial
    (reusable-auto-fix.yml). A rejected or timed-out deployment does not
    (see `APPROVED` above).
- **Facts and status.** It publishes the facts and status for FR-025.
- **The order page.** It serves the page for FR-030 behind a short-link host,
  such as a Cloudflare Tunnel, since the home server is presumably behind NAT.
- **Mail.** It sends the contract confirmations and lost-link mail.
- **Limits.** It caps requests per device per day and planning sessions per
  month, mirroring ADR-022's spend ceiling.
- **Backup.** The SQLite file holds the order state and `seq`, so it is backed
  up encrypted, and a restore is tested before Phase 1.

Secrets:

- **Stripe.** The concierge holds a restricted key (`rk_…`) with write on
  Products, Prices and Payment Links and read on Checkout Sessions and Invoices
  only. Refunds stay in Lauri's Dashboard. For an order paid through a
  Payment Link, before setting `PAID` (on Lauri's *mark paid* as on a webhook)
  it re-fetches the Checkout Session with this key and refuses unless
  `payment_status` is `paid`.
- **The public surface.** The order page, the withdrawal and lost-link forms
  and the webhook receiver run in a separate process. It holds the webhook
  signing secret (`whsec_…`) and read-only access to a projection the
  concierge writes. The projection covers every open order and every order
  still inside its 14-day withdrawal period: code, state, quote, Payment Link
  URL, token hash, paid date, hosted invoice URL and PR URL. It holds no
  Stripe API, GitHub or mail credential. It enqueues verified webhook events
  and form submissions. Rate limits key on `CF-Connecting-IP`, since every
  request arrives through the tunnel. The process listens only on localhost,
  for the tunnel connector, so no direct connection can set that header. The
  projection holds capability URLs and token hashes and is handled like a
  secret: owner-only file mode, never logged.
- **GitHub.** Dispatch uses a dedicated GitHub App installed only on this
  repo, with Actions: write, Pull requests: read and Metadata: read. The
  concierge holds the App's private key and mints a one-hour installation token
  per call. A fine-grained PAT is not an option: this repo belongs to a personal
  account, and GitHub gives a collaborator write on such a repo only through a
  classic PAT.
- **Guards on other workflows.** Actions: write reaches every dispatchable
  workflow, and lets the holder re-run past runs. So every workflow that runs
  Claude or spends gets a job-level `if:` that exits when
  `github.triggering_actor` is the App's bot, which also covers re-runs. That
  covers:
  - `auto-fix.yml`, whose guard also tests
    `github.event.workflow_run.triggering_actor.login`, because a failed
    `test.yml` run that the App dispatched or re-ran also starts it;
  - `auto-resolve-conflicts.yml`, `claude.yml`, `claude-code-review.yml` and
    `security-secrets.yml`;
  - `build-firmware.yml`, whose dispatch redeploys the web flasher and the OTA
    manifest from any tag. It also interpolates `inputs.release_tag` into
    `run:` (`build-firmware.yml:217`, `:462`, `:485`), which would turn Actions:
    write into its `contents: write` token. That input moves to an `env:`
    variable in the same change and is checked against the release-please
    tag pattern before use. It lands with the guards, so the interpolation is
    closed before the App exists.

  A re-run uses the original run's workflow file and is allowed for 30 days,
  so these guards land at least 30 days before the App is created.
- **Mail.** An SMTP or mail-API credential, used only by the concierge.
- **The robot** holds only its broker credentials.

### FR-027: Payment rail

| Phase | Rail | Confirmation |
|---|---|---|
| 0 (demo) | Stripe sandbox: no registration, no money (`stripe sandbox create`). Sandboxes send no customer e-mail, so Phase 0 checks the e-mail path by hand | Lauri taps *mark paid*, or the sandbox webhook |
| 1 (live, gated) | Stripe account as `individual` (FI). Per-order Payment Link with `payment_method_types = ["card", "mobilepay"]` (both confirm immediately), `restrictions.completed_sessions.limit = 1`, metadata `order_ref`, `invoice_creation` on, `consent_collection.terms_of_service = required`, and `custom_text.terms_of_service_acceptance` carrying the KSL 6:15 request and acknowledgement. Prerequisite: a terms-of-service URL, the order page's KSL text, in the Dashboard's Public details | Lauri marks paid once the payment shows *Succeeded* in Stripe, not *Processing* |
| 2 | Same link | Webhook: verifies `Stripe-Signature`, then the concierge re-fetches the session. `PAID` is set only on `checkout.session.completed` with `payment_status = "paid"`, or on `checkout.session.async_payment_succeeded`. `livemode` must equal the concierge's configured mode (false only against the Phase 0 sandbox), and `metadata.order_ref`, `amount_total` and `currency` must match the order. Events are deduplicated by event id. `async_payment_failed` returns the order to `AWAITING_PAYMENT` with a new link |

Fees (stripe.com/fi/pricing, 2026-10-23):

- Standard EEA cards: 1.5% + €0.25.
- MobilePay via Stripe: 1.5% + €0.36 in the standard-EEA-card case
  (estimate). Stripe charges the fee for the card behind the wallet and bills a
  separate MobilePay fee daily, so the effective rate may be higher for premium
  or non-EEA cards.
- Post-payment invoices: 0.4%, capped at US$2.
- Disputes: €20 each. Responding costs another €20, refunded if the dispute is
  won, so a contested and lost dispute costs €40.

On a €50 fee that is €1.00 by card, plus €0.20 for the invoice. MobilePay
becomes "Vipps" in Finland on 12.1.2027, so the wallet name is one string
shared by the page copy and Teuvo's templates, changed before that date.

The Vipps MobilePay ePayment API (2.49% + €0.20, HMAC-signed webhooks keyed
to an order reference) is an option only if a Y-tunnus is registered for
other reasons.

### FR-028: Gated planning workflow

`.github/workflows/teuvo-plan.yml` takes `workflow_dispatch` inputs `code`,
`category` and `summary`. The summary is at most 1000 characters and is the
text Lauri approved under FR-026.

**The gate.** The Claude job runs in environment `teuvo-planning`.
- Lauri is the required reviewer. Required reviewers work on public repos on
  the Free plan. *Prevent self-review* is on: the App dispatches, so Lauri can
  approve. The approval is the spend gate.
- The environment's deployment branches are restricted to `main`. A job from
  any other branch, for example a workflow an @claude session pushed, cannot
  enter the environment, so no approval prompt can hand it the Claude
  credential.

How the Claude credential is held:

- **OIDC federation is preferred** (`anthropic_federation_rule_id`,
  `id-token: write`), because an exfiltrated federated token expires within the
  run. The federation rule's subject is exactly
  `repo:laurigates/mcu-tinkering-lab:environment:teuvo-planning`. A repo-wide
  subject prefix would let any job with `id-token: write`, for example
  `claude.yml`'s, reach this billing.
- **With an API key,** the key exists only as a `teuvo-planning` environment
  secret.
- **Either way,** the Console workspace that bills these runs has a monthly
  spend limit.

**Job 1, the Claude run.** It does not hold a credential that writes to the
repo.
- `anthropics/claude-code-action` in automation mode, pinned to a SHA, with
  `allowed_bots` naming the App's bot.
- `github_token: ${{ secrets.GITHUB_TOKEN }}` is passed explicitly. Under
  `contents: read` it is read-only. Without it, the action exchanges the job's
  OIDC token for a Claude GitHub App token, which carries contents,
  pull-request and issue write.
- `runs-on: ubuntu-latest`, `timeout-minutes: 30`, `--max-turns 40`,
  `--max-budget-usd 10`.
- Checkout with `persist-credentials: false`, and permissions `contents: read`.
- The request reaches the prompt through a file, as fenced, labelled untrusted
  data. It is never `${{ }}` inside `run:` or `prompt:`.
- Allowed tools: `Read(./**)` and `Edit(packages/robocar/upgrades/**)`.
  - `Read` rules also govern Grep and Glob, and `Edit` rules govern Write.
  - No tool is listed bare, because a bare name allows every path. A bare
    `Read` could read `/proc/self/environ`, which holds the Claude credential;
    a deny rule `Read(//proc/**)` closes that too.
  - `--disallowedTools Bash WebFetch WebSearch` removes the shell and the web
    tools, so no tool sends request text to a URL. Without the Bash entry,
    Claude Code still
    runs its built-in read-only commands (`cat`, `grep`, `find`, read-only
    `git`) with no rule. An allow rule is no better: `Bash(git *)` admits
    `git -c core.fsmonitor=<cmd>` and `git push`, and a `just` rule admits a
    recipe Claude has just edited.
- After Claude, a fixed step runs
  `git add -A && git diff --cached --name-only "$BASE_SHA"`, which catches
  commits, staged files and untracked files. It fails the job on any path
  outside `packages/robocar/upgrades/<code>-*/`. It uploads only that directory
  as an artifact.

**Job 2, fixed steps.** It never runs Claude.
1. Download the artifact into a fresh checkout.
2. Run gitleaks over the plan directory, and fail on any hit.
3. Run the pricing step (FR-029) and, in Phase 2, the KiCad steps.
4. Commit and push `teuvo/plan-<code>`.
5. Open the draft PR with the release-please App token. PRs made with
   `GITHUB_TOKEN` start no CI (`.claude/rules/ci-cd-workflows.md`).

Nothing Claude wrote under `.git/` or `tools/` ever runs with that token.

`claude-code-review.yml` skips plan PRs, with a job-level
`if: ${{ !startsWith(github.head_ref, 'teuvo/plan-') }}`. Otherwise each plan
PR is reviewed on the subscription OAuth token.

Estimated cost per run on an API key: $3–10 at Opus 5.5 rates, unmeasured.
Phase 0 records the cost of every run, and the fee tiers (Open Questions) are
set from those figures rather than from this estimate. Actions minutes are
free on a public repo.

### FR-029: Upgrade-plan template

Each plan is a directory `packages/robocar/upgrades/<code>-<slug>/`, outside
the release-please package `packages/robocar/unified`. A plan merged there
would add an entry to the pending robocar-unified release PR, and so to the
next firmware release.

- **`README.md`** has:
  1. the request, as approved;
  2. what changes;
  3. the pins, mux channels and PCA9685 channels used, set against the spare
     list in `pin_config.h`: D3, D7, TCA9548A ch5–7, PCA9685 ch14–15, and the 16
     role-free MCP23017 pins on ch2 (GPA7/GPB7 output-only; the expander is
     optional);
  4. the power budget, checked against `.claude/rules/power-stage-checklist.md`.
     Claude drafts it; Lauri signs it off, as a gate-table row, before
     `LAYOUT`;
  5. a gate table: KiCad-free CI, local KiCad gates, layout review, physical
     checks still owed;
  6. lead time;
  7. Lauri's checklist;
  8. open questions.
- **`parts.csv`.** Claude writes only LCSC numbers or listing URLs. Job 2's
  pricing step, a small KiCad-free script, fills price, stock and
  Basic/Extended/Preferred status from the JLCPCB parts API and stamps the
  date. That API answers from a cloud container (verified 2026-10-23). A row it
  cannot price, AliExpress lines included, is marked `unpriced`. No total is
  quoted until Lauri prices those rows.
  - The script takes LCSC numbers only and prices at the board quantity
    `cost.csv` states. An error, a missing field or a listing URL gives
    `unpriced`.
  - The endpoint is the undocumented cart API the `jlcpcb-order` skill calls,
    so a host test runs the script against a recorded response.
- **`cost.csv`** has one row per cost, each with source, date and an
  `estimate`/`quote` label:
  - PCBA setup €7.33 and stencil €1.38;
  - about €2.75 per Extended line;
  - DHL €24–32;
  - VAT 25.5%;
  - EU duty on parcels under €150 (since 1.7.2026);
  - JLC3DP;
  - AliExpress;
  - build hours × rate.

  The figures come from the calibrations in the `jlcpcb-order` and
  `aliexpress-parts` skills, which are dated carts from 2026-10-22/23, not
  quotes.
- **`renders/`** holds committed SVG/PNG files, because `output/` is
  gitignored.
  - **Phase 1** is docs-only: a parts table and a block diagram, with no
    carrier.
  - **Phase 2.** Claude also writes a carrier variant's inputs inside the plan
    directory: `hardware.toml`, and `hardware/carrier/board.toml` at
    `stage = "schematic"`. Job 2 then installs KiCad (`just kicad::setup`,
    about 4.5 minutes, verified in a cloud container on 2026-10-23), generates
    the schematic and renders it, as fixed steps. `tools/kicad` finds projects
    only by a git-tracked `board.toml` (`paths.py`, `tracked_projects()`), and
    the plan files are untracked until step 4, so each step names the plan
    directory: `PYTHONPATH=tools python3 -m kicad.schematic <plan dir>`, then
    `kicad-cli sch export svg --output <plan dir>/renders/` on the generated
    schematic, which writes `renders/<[project] name>.svg`. A
    schematic-stage carrier has no board, so there is no board render. A Phase
    2 plan whose `renders/` lacks that file counts as `FAILED`.
  - **CI on a plan PR.** Plan PRs are not merged (Non-Goals), so `main`'s
    tracked carriers do not grow. An open plan PR's branch runs
    `carrier-check.yml` and `hardware-check.yml` on its variant, and the board
    gates skip a `stage = "schematic"` carrier.
  - **Limits on a variant.** It may use only parts already in
    `docs/reference/parts/`, symbols already vendored in `tools/kicad/lib`, and
    existing pin roles. A plan that needs a new part, symbol or pin role would
    need edits outside its directory, such as to `unified/main/pin_config.h`,
    which the run may not make. Such a plan stays docs-only.

A `README.md` at the upgrades root holds the template.

### FR-030: Link hand-off

- **The sticker.** A printed QR sticker on Teuvo's chassis, about 25 mm, links
  to the order page. Being printed, it is not held to the OLED's version-1
  limit. Teuvo speaks only the order code, one character at a time, and
  "katso tarraa selässäni" ("see the sticker on my back").
- **Before payment.** The page lists no orders. It takes a typed order code,
  rate-limited and case-insensitive, reading I and L as 1 and O as 0
  (Crockford), and shows only the quote, the pre-contract information and the
  Payment Link. None of these carries personal data.
- **After payment.** The invoice and the withdrawal function sit behind a
  per-order URL with a random 128-bit token, which is never logged. The URL
  reaches the payer three ways:
  - the Payment Link's `after_completion` redirect opens it;
  - Stripe's post-payment invoice repeats it in
    `invoice_creation.invoice_data.footer`;
  - the contract confirmation e-mail carries it.
- **Lost link.** A form asking for the order code and the checkout e-mail
  sends the URL to that address. The withdrawal function stays reachable for
  the whole 14 days, and its confirmation is sent without delay on a durable
  medium (KSL 6:14 a).
- **No secret on the robot.** Teuvo never holds or speaks a PIN or token, and
  no secret enters the facts topic or a Gemini prompt.
- **The PR link** sits on the per-order page. The PR itself is public; the
  invoice is not.
- **The OLED.** An OLED QR code is optional and later. It needs bringup's
  SSD1306 driver ported to `robocar-unified` with a pixel API, and a phone scan
  test of a version-1 code at about 0.175 mm per pixel, drawn lit-on-dark.

### FR-031: Lead-time calibration

No order-placed to delivered date exists anywhere in the repo.
`packages/robocar/upgrades/lead-times.csv` records each real order: supplier,
service, ordered, shipped and delivered dates, so ETAs become measurements.
Until enough rows exist, the concierge uses quoted bands:

- **JLCPCB fabrication:** 24 h for 5 two-layer boards; Economic PCBA 1–3 days
  (JLCPCB production-time pages).
- **DHL Express:** 2–4 days (the `jlcpcb-order` skill's checkout, 2026-10-23).
- **Total:** about 4–8 days for an assembled board (estimate). PRD-013
  estimates 6–8 days for a bare board by DHL.
- **AliExpress:** weeks. The only figure in the repo is a 24–46-day delivery
  band quoted for solder paste on 2026-10-23, not a measured delivery.

When Lauri moves an order to `BUILDING`, he sets a `done_week` on it. The
concierge renders the done phrase only from that field, and the parts phrase
only from the bands or measurements, taking a band's later end, for example
"osat tulevat ensi viikon lopulla" ("the parts come at the end of next week").
Until `done_week` is set, Teuvo says Lauri will tell when the build is done.

## Phases

| Phase | Scope | Money | Gates |
|---|---|---|---|
| 0 | FR-024–FR-028 end to end on Lauri's own Teuvo, Stripe sandbox, docs-only plans | none | Lauri: approve text, mark paid, approve run |
| 1 | Live Stripe, KSL texts and confirmation e-mail, invoices, FR-029 docs-only, FR-030 sticker | real, planning fee only | Lauri: approve text, mark paid, approve run, every purchase |
| 2 | Webhook-confirmed payment, schematic-stage variants, FR-031 calibration; build quotes and `BUILD_PAID` only once the build-fee question is settled | real | Lauri: approve text, approve run, `LAYOUT`, every purchase |

Phase 0 also records, per order, the cost of the run and Lauri's minutes at
each gate.

Sequence across PRD-014, PRD-015 and ADR-028. The phases above are milestones
on top of the steps below, and the Phase 0 demo needs steps 1–6. ADR-028
FR-040–FR-042, PRD-015 FR-032 (#852) and PRD-015 phases 2–4 are not
prerequisites: FR-025's budget test covers the compiled-in persona strings,
not the overlay.

1. **The workflow guards** in FR-026, with the `build-firmware.yml` input fix
   (#855, step 1). They depend on nothing else and must be at least 30 days
   old before the dispatch App is created.
2. **The TTS model migration** (ADR-028 FR-043, #844).
   `gemini-3.1-flash-tts-preview` has a listed earliest shutdown of 2026-11-17
   (ai.google.dev deprecations, 2026-10-22).
3. **The runtime Gemini credential** (ADR-028 FR-039, #845) and **per-device
   identity** (PRD-015 FR-033, #851): the `teuvo/<device_id>/` topics, the ACL
   and the client id.
4. **Services armed on link-up** (ADR-028 FR-038, #846), then the network list
   with its `home` flag and the connectivity state of §4 (FR-037, #848).
5. **The four truncation fixes** under FR-025 (#847), after step 2, because
   their budget test builds against the migrated TTS request shape.
6. **MQTT broker credentials** on the robot, with `allow_anonymous false` and
   the ACL on the broker, after step 3. Before that switch:
   - give `build-firmware.yml`'s notify step the `ci-notify` broker credential
     (PRD-015 FR-033). The step is `continue-on-error`, so without it the step
     would fail silently;
   - check whether `vars.MQTT_ENABLED` lets GitHub runners publish anonymously
     to the broker today (`build-firmware.yml:584-632`).
7. **Phase 0.** #853 (device: FR-024, FR-025) after steps 2–6. #854
   (concierge: FR-026, FR-027) can be built against the sandbox earlier and
   needs step 6 for the demo. #855's workflow (FR-028, FR-029) once step 1's
   guards are 30 days old.
8. **Phase 1**, after the Phase 0 demo, Anthropic's confirmation or a chosen
   fallback (Open Questions, Claude billing), the fee tiers and a tested
   restore (FR-026).
9. **Phase 2.**

## Acceptance Criteria

Each criterion names how it is checked (host test, tests, CI, review, or the
Phase 0 demo by hand) and the Phase 0 issue that carries it.

- [ ] *(host tests, #853)* Host tests pin the `__REQUEST__`/`__ORDER__`
  splitter, the sentence filter against the committed paraphrase corpus, every
  order template, the facts schema validator, the facts size limit against the
  MQTT buffer, and the persona and buffer budgets
- [ ] *(demo, all three)* Phase 0 demo: a spoken request leads to Lauri
  approving the text, a template quote with the right fee, a sandbox payment,
  Lauri's run approval, a draft PR under `packages/robocar/upgrades/`, and
  Teuvo saying the plan is ready. No step is faked on the device side; the
  confirmation e-mail is checked by hand, since sandboxes send none (FR-027)
- [ ] *(CI, #855)* A run whose Claude job writes, commits or stages a path
  outside its plan directory fails at the path guard, and no PR is opened
- [ ] *(demo, #853)* With the concierge stopped, Teuvo says he will ask Lauri
  and states no order fact, including after a reconnect
- [ ] *(tests and demo, #854)* Saying "I paid" to Teuvo changes no order
  state. A delayed payment method never sets `PAID`
- [ ] *(tests, #854)* A withdrawn or refunded order is never planned, even if
  its run was already waiting for approval
- [ ] *(demo, #854)* A payer who has lost the order link can still withdraw
  online within 14 days and receives a durable confirmation
- [ ] *(review, all three)* No name, e-mail, phone number or invoice appears
  in any issue, PR, dispatch input or Actions log produced by the flow
- [ ] *(CI, #853)* The desk is off in every CI, web-flasher and OTA image, and
  turns on only through a serial command

## Open Questions

- **Business form.** Toiminimi or company with a Y-tunnus, a light-entrepreneur
  invoicing service, or Stripe as an individual? This decides MobilePay
  merchant access, bookkeeping and how invoices look. Whether an individual
  Stripe account in Finland can enable MobilePay is unconfirmed: if it cannot,
  staying individual makes the Phase 1 link card-only, while a Y-tunnus, or a
  platform that already holds merchant status, keeps MobilePay.
- **Prices.** Fee tiers for planning, the hourly build rate, quote expiry and
  the refund policy if a parts order fails. Also:
  - what a `FAILED` run gives the payer: a free re-run, or a refund recorded
    as `REFUNDED`; either one goes into the terms;
  - whether the fee carries a reserve for disputes, which cost €40 each when
    contested and lost;
  - the monthly caps on requests and planning sessions (FR-026);
  - which model plans: Opus throughout, or Sonnet with escalation to Opus,
    which lowers the per-run cost and may lower plan quality.
- **What the payer receives.** For the planning fee: the plan, plus perhaps
  their name on the PCB silkscreen or a spoken thank-you. For a build fee:
  whose robot the build goes into, and what the payer gets for it.
  `BUILD_QUOTED` stays off until this is answered, so until then the desk
  earns only planning fees. Also whether build quotes stay a conditional part
  of Phase 2, or move to a later phase gated on a written decision, which
  leaves Phase 2 as webhook payment, schematic variants and calibration.
- **Claude billing.** Which Console organisation bills the runs? Does
  Anthropic confirm that a paid planning run for a requester is not
  "intermediating" Claude usage? If it does not, the fallbacks are:
  - Lauri writes plans himself: FR-028 shrinks to Job 2, and his time per
    order grows;
  - an open-weights model on the concierge host: no intermediation question
    or per-run fee, but lower plan quality and longer runs;
  - a provider whose terms allow resale: the credential and spend gate in
    FR-028 change with it.
- **Hosting.** Is there an always-on home server, and is the Mosquitto broker
  on it reachable from outside? Which short-link domain and mail sender?
  The alternative is a managed concierge (Cloudflare Workers with D1, or a
  small VPS) and a hosted broker: NAT, uptime and backups leave the home
  server, but there is a monthly cost and personal data and secrets move to a
  third party. Either way: where the encrypted backup (FR-026) lives, what
  hosting costs, and what uptime a payer can expect.
- **Phone gate.** Which channel carries the text approval and *mark paid*:
  ntfy, Telegram, or Home Assistant actionable notifications? The run approval
  stays in GitHub Mobile. Also the fallback while the channel is down, such as
  a CLI on the concierge host for the same actions, and whether an unanswered
  text approval expires the request or waits.
- **When Lauri is away.** Every gate is Lauri. Options:
  - a deputy for some gates (text approval, *mark paid*), which widens who
    sees request text and payer data;
  - approving several requests in one batch, which cuts the interruptions but
    not the wait;
  - closing the desk (concierge status `offline`, so Teuvo says it is closed)
    and letting quotes expire. A paid order cannot expire; it waits or is
    refunded.
- **Bank transfer.** Is a bank-transfer rail ever needed? If so, it collects
  the payer's e-mail and the 6:15 consent before showing the IBAN, and e-mails
  the per-order URL only after Lauri marks the order paid. Also open: what
  evidence lets Lauri's *mark paid* set `PAID` without a Checkout Session.
- **Publicity.** Is it acceptable that approved requests and plans are public
  in this repo, or should plans live in a private repo? That needs a paid
  GitHub plan for environment approvals.
- **Who talks to Teuvo.** Will people under 18 talk to Teuvo? The Gemini terms
  restrict that independently of this PRD, and UCPD bars pitching to children.
  Options: an adult-only household as a precondition for enabling the desk,
  with the owner's attestation at the serial enable step, which adds a step
  to enabling; or a non-Gemini backend, which needs a request adapter as well
  as FR-034's base-URL change (PRD-015 Open Questions) and changes the model
  and its terms for every turn, not only the desk.
- **Languages.** Finnish only, or English as well? Each language needs its own
  filter stems and templates. Options: v1 Finnish-only, stated on the order
  page and the setup page (PRD-015 FR-035); or a `lang` field in the facts
  document with a template bundle and filter stems per language. The request
  already carries `lang` (FR-024).
- **Personal data.** The concierge holds payer e-mails, request summaries and
  device ids, on a robot that records household audio and images. The answers
  set what the concierge deletes and when, and what the order page says. Each
  point has its options:
  - controller and lawful basis: Lauri as sole controller on the contract, or
    a role shared with an invoicing platform if the business form brings one;
  - retention: one period for orders, e-mails and backups, or invoices for the
    bookkeeping-law period and request text and e-mails for a shorter one;
  - privacy notice: its own page linked from the order page before payment, or
    part of the pre-contract information;
  - data-subject requests: answered by hand from e-mail, or through a form
    beside the lost-link form;
  - DPIA: a short DPIA, or a recorded screening that finds none is needed.

## Related Documents

- [ADR-028: Teuvo connectivity](../decisions/ADR-028-teuvo-connectivity-resilience.md): prerequisites
- [PRD-015: A Teuvo for other owners](PRD-015-teuvo-for-other-owners.md): per-device identity and the ACL
- [ADR-024: Conversational entity](../decisions/ADR-024-conversational-entity-and-multimodal-voice.md)
- [ADR-022: Planner dormancy and spend ceiling](../decisions/ADR-022-planner-dormancy-and-spend-ceiling.md): the cap pattern the concierge mirrors
- [ADR-027: KiCad carrier generation](../decisions/ADR-027-kicad-carrier-generation.md): what a plan can generate
- [PRD-013: Rapid hardware prototyping](PRD-013-rapid-hardware-prototyping.md): bare-board lead-time estimate
- Issue #619: Gemini Live, a later replacement for the request sentinel
