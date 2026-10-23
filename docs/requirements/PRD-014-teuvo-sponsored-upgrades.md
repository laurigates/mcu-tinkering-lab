# PRD-014: Teuvo Sponsored Upgrades — Commissioned Feature Planning Through Conversation

**Status**: draft
**Created**: 2026-10-23
**Confidence**: 6/10 (legal points are interpretation, not legal advice; no part of the flow has run)

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
| A standing, priced, public offer very likely counts as business activity (*elinkeinotoiminta*) | Lauri is a trader under KSL chapter 6. He owes pre-contract information, a 14-day withdrawal right, and a withdrawal function reachable for the whole period (6:14 a, in force 19.6.2026). Planning may start inside the withdrawal period only on the consumer's express request and acknowledgement (6:15–6:16) | Vero VH/8236/00.01.00/2024; KSL 38/1978 (Finlex) |
| No VAT is due while turnover is at most €20,000 in the current and previous year | No VAT on invoices until then; resold parts count toward turnover | Vero VH/1581/00.01.00/2024 |
| The repo is public | No names, contact details, invoices or payment references in issues, PRs, dispatch inputs or Actions logs | REST API: `visibility: public` |
| No direct exhortation to children to buy | Teuvo never pitches upgrades. He answers an explicit request from an adult, and the feature is off unless switched on | UCPD 2005/29/EC Annex I point 28 |
| Gemini API users must be 18+, and an API client may not be "likely to be accessed by" under-18s | Applies to Teuvo whatever this PRD does; an open question below | Gemini API Additional Terms (2026-03-23) |
| Developers building products or services "should use API key authentication", and customers offering Claude Code in their services "may not pay for, resell, or intermediate Claude usage on their end users' behalf" | Customer-paid runs bill to a Console API key through OIDC federation. *Interpretation:* whether a run Lauri performs for a paying requester counts as intermediating is open. Confirm with Anthropic sales before Phase 1 | code.claude.com/docs/en/legal-and-compliance |
| Free and open-source software that its manufacturer monetises can count as supplied "in the course of a commercial activity" | The upgrade desk is **off by default** in every published image and enabled per robot over serial. Payment happens on the payer's phone, never on the robot. This also keeps Teuvo outside RED 3(3)(f), which covers equipment that "enables the holder or user to transfer money" (interpretation) | CRA 2024/2847 recital 15; RED Delegated Reg. 2022/30 Art 1(3) |

## Goals

- Turn a spoken request into a quoted, paid, planned upgrade, with a PR as the
  deliverable, and no step that only works in a demo
- Keep Lauri the gate for the published text, for starting a planning run and
  for any parts purchase. Automating one gate must not automate another
- Never let the model state a price, a payment state or a date. Every figure
  Teuvo says is rendered on the device from server facts by a fixed template
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
    P->>T: "Teuvo, could you have a gripper?"
    T->>C: teuvo/<id>/upgrade/request {req_id, summary, category}
    C->>L: summary text to edit or approve
    L->>C: approve text
    C->>T: facts {code K7Q2, state QUOTED, fee 40}
    T->>P: template line with the fee and the code
    P->>S: pays on the order page (Payment Link)
    S-->>C: paid (Lauri in Phase 1, signed webhook in Phase 2)
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
  Nothing the person says to Teuvo can set them. `PAYMENT_CLAIMED` exists so
  that Teuvo can say "thank you, Lauri will confirm" without claiming the money
  has arrived.
- **Consent before dispatch.** An order records `consent_at`, and dispatch
  requires it. It comes from the Stripe session's
  `consent.terms_of_service = accepted`, which carries the KSL 6:15 request and
  acknowledgement. On the bank rail it comes from a checkbox the payer must
  tick before the IBAN and reference are shown.
- **Withdrawal or refund.** On `WITHDRAWN` or `REFUNDED`, the concierge cancels
  any waiting or running planning run for that order.
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

### FR-024: Upgrade-request capture in the voice turn

When the desk is enabled, the voice-turn system prompt allows a second
sentinel next to `__IGNORE__`. The reply opens with `__REQUEST__`, then one
English line `<category> | <summary ≤160 chars>`, then the spoken Finnish
reply. A pure-C splitter in `voice_history.c` style separates the parts. The
device publishes `{req_id, device_id, category, summary, lang}` to
`teuvo/<device_id>/upgrade/request` at QoS 1.

- The model emits the sentinel only for an explicit request addressed to
  Teuvo. It never volunteers or suggests a paid upgrade. The instruction also
  asks it to leave out names; Lauri's text approval (FR-026) is the control,
  this is only a first filter.
- Teuvo's reply says that the summary goes to Lauri and that an accepted plan
  is published.
- Probe the reply format live, with an audio part, on `gemini-flash-latest`
  before building on it. It is a moving alias (`.claude/rules/gemini-api.md`
  §1), so re-probe after it moves. Function calling on the voice turn, or
  Gemini Live input transcription (#619), can replace the sentinel later.
- The topics and the broker ACL come from PRD-015 FR-033. Within
  `teuvo/<id>/upgrade/`, the robot may publish only `request` and read only
  `facts`. Only the concierge may write `facts`. `robocar/commands` is never
  used for this.

### FR-025: Order facts downlink and template phrasing

The concierge publishes a retained UTF-8 JSON document to
`teuvo/<device_id>/upgrade/facts`. It holds at most 4 open orders, paid states
first, then newest. For each order it carries the code, the state (an enum),
the fee or quote in whole euros, and a pre-rendered Finnish ETA phrase. The
concierge refuses to publish more than 900 bytes.

The firmware subscribes through `mqtt_logger_subscribe()`. Its callback gains
`total_data_len` and `current_data_offset`, so a fragment check like
`mqtt_command_extract_line()`'s can see fragments. `mqtt_logger_init()` sets
`buffer.size` explicitly; the ESP-IDF default is 1024 bytes. The firmware
validates the document against a fixed schema.

- **Fail closed.** The concierge holds a retained `teuvo/concierge/status` of
  `online`, with a retained Last Will of `offline`, and publishes `offline`
  itself on a clean stop. It republishes the facts every 10 minutes, with an
  incrementing `seq` and freshly rendered ETA phrases. The robot treats the
  facts as absent:
  - while the status is not `online`;
  - after any MQTT disconnect, until a new `seq` arrives;
  - when no new `seq` has arrived within 20 minutes on its own uptime clock.

  OTA, facts and status take 3 of the 4 subscribe slots. With no facts, Teuvo
  says he will ask Lauri and never guesses.
- **The model never states a price, payment state or date.** When an order
  comes up, the model emits `__ORDER__ <code>`. While the desk is on, the reply
  splitter drops every model sentence that mentions money or payment (the stems
  maks-, rah-, euro, €, pay/paid). In a reply with `__ORDER__` it also drops
  every sentence containing a digit. The device then appends one sentence
  rendered from the facts by a fixed template for that state. Examples:
  "Tilauksen K7Q2 suunnittelu maksaa 40 euroa." ("Planning order K7Q2 costs 40
  euros.") or, while the state is below `PAID`, "Kiitoksia! Lauri kuittaa vielä
  maksun." ("Thank you! Lauri will still confirm the payment.").
- **Teuvo never reads PR text.** Only enums, numbers and the pre-rendered ETA
  phrase reach the device, so a PR cannot steer him.
- **Before any of this, four truncations are fixed:**
  1. the 511-byte voice-turn brief (`voice_turn.c:292-301`);
  2. the ~190 bytes of TTS text left under the 383-byte style
     (`gemini_tts.c:238-240`);
  3. `s_reply[SPEECH_TEXT_MAX = 320]` (`voice_turn.c:128`), which must hold a
     sentinel line plus the spoken reply;
  4. `tele_summary[128]` inside `prompt_text[512]` (`voice_history.c:273-280`),
     which must hold the facts sentence.

  The fix includes a host test that budgets every persona string and these four
  buffers, against the migrated TTS request shape (Prerequisites).

### FR-026: Concierge service

New Python 3.12 + uv project `packages/robocar/concierge/`, using SQLite,
ruff and ty. It runs on Lauri's home server. Its jobs:

- **Requests.** It deduplicates on `req_id` and assigns a 4-character
  Crockford base32 code. It prices from Lauri's fee table (fixed tiers by
  category) and expires quotes after 14 days.
- **Order state.** It owns the order state machine, the two-writer rule and
  the consent rule above.
- **Lauri's phone gate.** In every phase, before anything is published or
  dispatched, the notification shows Lauri the exact summary text with *edit*,
  *approve text* and *reject*. Only text he approved is dispatched. Phase 2
  automates `PAID`, never this step. The same channel carries *mark paid*,
  *reject* and *refund*. The channel itself is an open question.
- **Planning runs.** It dispatches `teuvo-plan.yml` with the approved summary,
  one run at a time. The next order is dispatched only when the previous run
  has concluded. It then polls the run and `GET /pulls?head=…:teuvo/plan-<code>`.
  A cancelled run returns the order to `PAID`. A run that concluded without a
  PR counts as `FAILED`, because the SDK exits 0 on a permission denial
  (reusable-auto-fix.yml).
- **Facts and status.** It publishes the retained facts and status for FR-025.
- **The order page.** It serves the page for FR-030 behind a short-link host,
  such as a Cloudflare Tunnel, since the home server is presumably behind NAT.
- **Limits.** It caps requests per device per day and planning sessions per
  month, mirroring ADR-022's spend ceiling.

Secrets:

- **Stripe.** The concierge holds a restricted key (`rk_…`) with write on
  Products, Prices and Payment Links and read on Checkout Sessions only.
  Refunds stay in Lauri's Dashboard.
- **The public surface.** The order page and the webhook receiver run in a
  separate process that holds no key. It only enqueues signature-verified
  events.
- **GitHub.** Dispatch uses a fine-grained PAT of a dedicated machine-user
  account, expiring after 90 days. Actions: write reaches every dispatchable
  workflow. So every other `workflow_dispatch` workflow that runs Claude or
  spends (`auto-fix.yml`) exits when `github.actor` is that account.
- **The robot** holds only its broker credentials.

### FR-027: Payment rail

| Phase | Rail | Confirmation |
|---|---|---|
| 0 (demo) | Stripe sandbox: no registration, no money (`stripe sandbox create`) | Lauri taps *mark paid*, or the sandbox webhook |
| 1 (live, gated) | Stripe account as `individual` (FI). Per-order Payment Link with `payment_method_types = ["card", "mobilepay"]` (both confirm immediately), `restrictions.completed_sessions.limit = 1`, metadata `order_ref`, `invoice_creation` on, and `consent_collection` + `custom_text` carrying the KSL 6:15 request, the acknowledgement and the withdrawal notice | Lauri marks paid once the payment shows *Succeeded* in Stripe, not *Processing* |
| 2 | Same link | Webhook: verifies `Stripe-Signature`; sets `PAID` only on `checkout.session.completed` with `payment_status = "paid"`, or on `checkout.session.async_payment_succeeded`, with `livemode = true` and `metadata.order_ref`, `amount_total` and `currency` matching the order; deduplicates by event id. `async_payment_failed` returns the order to `AWAITING_PAYMENT` with a new link |
| fallback | Bank transfer with a Finnish reference number (*viitenumero*) and *virtuaaliviivakoodi* on the order page | Lauri, by hand; dispatch still requires `consent_at` |

Fees (stripe.com/fi/pricing, 2026-10-23):

- Standard EEA cards: 1.5% + €0.25.
- MobilePay via Stripe: 1.5% + €0.36 in the standard-EEA-card case
  (estimate). Stripe charges the fee for the card behind the wallet and bills a
  separate MobilePay fee daily, so the effective rate may be higher for premium
  or non-EEA cards.
- Disputes: €20 each. Responding costs another €20, refunded if the dispute is
  won, so a contested and lost dispute costs €40.

On a €50 fee that is €1.00 by card. MobilePay becomes "Vipps" in Finland on
12.1.2027, and the copy on the page and in Teuvo's templates changes with it.

The Vipps MobilePay ePayment API (2.49% + €0.20, HMAC-signed webhooks keyed
to an order reference) is an option only if a Y-tunnus is registered for
other reasons.

### FR-028: Gated planning workflow

`.github/workflows/teuvo-plan.yml` takes `workflow_dispatch` inputs `code`,
`category` and `summary`. The summary is at most 1000 characters and is the
text Lauri approved under FR-026.

**The gate.** The Claude job runs in environment `teuvo-planning`, with Lauri
as required reviewer. Required reviewers work on public repos on the Free
plan. *Prevent self-review* is on, since the machine user dispatches. The
approval is the spend gate. How the Claude credential is held:

- With OIDC federation (`anthropic_federation_rule_id`, `id-token: write`),
  the federation rule's subject is exactly
  `repo:laurigates/mcu-tinkering-lab:environment:teuvo-planning`. GitHub issues
  that subject only to a job running in that environment, after approval. A
  repo-wide subject prefix would let any job with `id-token: write` (for
  example `claude.yml`'s) reach this billing.
- With an API key, the key exists only as a `teuvo-planning` environment
  secret.
- Either way, the Console workspace that bills these runs has a monthly spend
  limit.

**Job 1, the Claude run.** It does not hold the credential that writes to the
repo.
- `anthropics/claude-code-action` in automation mode, pinned to a SHA.
- `runs-on: ubuntu-latest`, `timeout-minutes: 30`, `--max-turns 40`,
  `--max-budget-usd 10`.
- Checkout with `persist-credentials: false`, and permissions `contents: read`.
- The request reaches the prompt through a file, as fenced, labelled untrusted
  data. It is never `${{ }}` inside `run:` or `prompt:`.
- Allowed tools: `Read`, `Glob`, `Grep` and `Edit(packages/robocar/upgrades/**)`,
  which also governs `Write`. There is no bare `Write` or `Edit`, because a
  bare tool name allows every path. There is no `Bash` rule:
  - `Bash(git *)` admits `git -c core.fsmonitor=<cmd>` and `git push`;
  - a `just` rule admits a recipe Claude has just edited.
- After Claude, a fixed step runs
  `git add -A && git diff --cached --name-only "$BASE_SHA"`, which catches
  commits, staged files and untracked files. It fails the job on any path
  outside `packages/robocar/upgrades/<code>-*/`. It uploads only that directory
  as an artifact.

**Job 2, fixed steps.** It never runs Claude.
1. Download the artifact into a fresh checkout.
2. Run the pricing step (FR-029) and, in Phase 2, the KiCad steps.
3. Commit and push `teuvo/plan-<code>`.
4. Open the draft PR with the release-please App token. PRs made with
   `GITHUB_TOKEN` start no CI (`.claude/rules/ci-cd-workflows.md`).

Nothing Claude wrote under `.git/` or `tools/` ever runs with that token.

`claude-code-review.yml` skips plan PRs, with a job-level
`if: ${{ !startsWith(github.head_ref, 'teuvo/plan-') }}`. Otherwise each plan
PR is reviewed on the subscription OAuth token.

Estimated cost per run on an API key: $3–10 at Opus 5.5 rates, unmeasured.
Actions minutes are free on a public repo.

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
  4. the power budget, checked against `.claude/rules/power-stage-checklist.md`;
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
    `stage = "schematic"`. `tools/kicad` discovers any git-tracked `board.toml`
    (`paths.py:1-11`). Job 2 then installs KiCad (`just kicad::setup`, about 4.5
    minutes, verified in a cloud container on 2026-10-23), generates the
    schematic and renders it with `kicad-cli sch export svg`, as fixed steps. A
    schematic-stage carrier has no board, so there is no board render.
  - **Limits on a variant.** It may use only parts already in
    `docs/reference/parts/`, symbols already vendored in `tools/kicad/lib`, and
    existing pin roles. A plan that needs a new part, symbol or pin role would
    need edits outside its directory, such as to `unified/main/pin_config.h`,
    which the run may not make. Such a plan stays docs-only.

A `README.md` at the upgrades root holds the template.

### FR-030: Link hand-off

- **The sticker.** A printed QR sticker on Teuvo's chassis, about 25 mm, links
  to the order page. Teuvo speaks only the order code and "katso tarraa
  selässäni" ("see the sticker on my back").
- **Before payment.** The page lists no orders. It takes a typed order code,
  rate-limited per IP, and shows only the quote, the pre-contract information
  and the Payment Link. None of these carries personal data.
- **After payment.** The invoice and the withdrawal function sit behind a
  per-order URL with a random 128-bit token. Stripe e-mails the invoice
  (`invoice_creation`). The Payment Link's `after_completion` redirect opens the
  URL, and the receipt e-mail repeats it. On the bank rail the page shows the
  URL once, after the consent checkbox.
- **Lost link.** A payer who has lost it can still withdraw. A form asking
  for the order code and the checkout e-mail sends the link to that address.
  The withdrawal function stays reachable for the whole 14 days, and the
  confirmation is sent without delay on a durable medium (KSL 6:14 a).
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
only from the bands or measurements, for example "osat tulevat ensi viikon
lopulla" ("the parts come at the end of next week"). Until `done_week` is set,
Teuvo says Lauri will tell when the build is done.

## Phases

| Phase | Scope | Money | Gates |
|---|---|---|---|
| 0 | FR-024–FR-028 end to end on Lauri's own Teuvo, Stripe sandbox, docs-only plans | none | Lauri: approve text, mark paid, approve run |
| 1 | Live Stripe, KSL texts on the order page, invoices, FR-029 docs-only, FR-030 sticker | real, planning fee only | Lauri: approve text, mark paid, approve run, every purchase |
| 2 | Webhook-confirmed payment, schematic-stage variants, FR-031 calibration; build quotes and `BUILD_PAID` only once the build-fee question is settled | real | Lauri: approve text, approve run, `LAYOUT`, every purchase |

Prerequisites:

- **Per-device identity** (PRD-015 FR-033): the `teuvo/<device_id>/` topics,
  the ACL and the client id.
- **From ADR-028:** the runtime Gemini credential (FR-039), services armed on
  link-up (FR-038), and the connectivity state of §4 (FR-037).
- **The TTS model migration** (FR-043). `gemini-3.1-flash-tts-preview` has a
  listed earliest shutdown of 2026-11-17 (ai.google.dev deprecations,
  2026-10-22).
- **The four truncation fixes** under FR-025.
- **MQTT broker credentials** on the robot, with `allow_anonymous false` and
  the ACL on the broker. Before that switch:
  - give `build-firmware.yml`'s notify step broker credentials. The step is
    `continue-on-error`, so without them it would fail silently;
  - check whether `vars.MQTT_ENABLED` lets GitHub runners publish anonymously
    to the broker today (`build-firmware.yml:584-632`).

## Acceptance Criteria

- [ ] Host tests pin the `__REQUEST__`/`__ORDER__` splitter, the money and
  digit sentence filter, every order template, the facts schema validator,
  the facts size limit against the MQTT buffer, and the persona and buffer
  budgets
- [ ] Phase 0 demo: a spoken request leads to Lauri approving the text, a
  template quote with the right fee, a sandbox payment, Lauri's run approval, a
  draft PR under `packages/robocar/upgrades/`, and Teuvo saying the plan is
  ready. No step is faked on the device side
- [ ] A run whose Claude job writes, commits or stages a path outside its plan
  directory fails at the path guard, and no PR is opened
- [ ] With the concierge stopped, Teuvo says he will ask Lauri and states no
  order fact, including after a reconnect
- [ ] Saying "I paid" to Teuvo never moves an order past `PAYMENT_CLAIMED`.
  A delayed payment method never sets `PAID`
- [ ] A payer who has lost the order link can still withdraw online within 14
  days and receives a durable confirmation
- [ ] No name, e-mail, phone number or invoice appears in any issue, PR,
  dispatch input or Actions log produced by the flow
- [ ] The desk is off in every CI, web-flasher and OTA image, and turns on only
  through a serial command

## Open Questions

- **Business form.** Toiminimi or company with a Y-tunnus, a light-entrepreneur
  invoicing service, or Stripe as an individual? This decides MobilePay
  merchant access, bookkeeping and how invoices look.
- **Prices.** Fee tiers for planning, the hourly build rate, quote expiry and
  the refund policy if a parts order fails.
- **What the payer receives.** For the planning fee: the plan, plus perhaps
  their name on the PCB silkscreen or a spoken thank-you. For a build fee:
  whose robot the build goes into, and what the payer gets for it.
  `BUILD_QUOTED` stays off until this is answered.
- **Claude billing.** Which Console organisation and API key bills the runs?
  Does Anthropic confirm that a paid planning run for a requester is not
  "intermediating" Claude usage?
- **Hosting.** Is there an always-on home server, and is the Mosquitto broker
  on it reachable from outside? Which short-link domain?
- **Phone gate.** Which channel carries the text approval and *mark paid*:
  ntfy, Telegram, or Home Assistant actionable notifications? The run approval
  stays in GitHub Mobile.
- **Publicity.** Is it acceptable that approved requests and plans are public
  in this repo, or should plans live in a private repo? That needs a paid
  GitHub plan for environment approvals.
- **Who talks to Teuvo.** Will people under 18 talk to Teuvo? The Gemini terms
  restrict that independently of this PRD, and UCPD bars pitching to children.
- **Languages.** Finnish only, or English as well? Each language needs its own
  money stems and templates.

## Related Documents

- [ADR-028: Teuvo connectivity](../decisions/ADR-028-teuvo-connectivity-resilience.md): prerequisites
- [PRD-015: A Teuvo for other owners](PRD-015-teuvo-for-other-owners.md): per-device identity and the ACL
- [ADR-024: Conversational entity](../decisions/ADR-024-conversational-entity-and-multimodal-voice.md)
- [ADR-022: Planner dormancy and spend ceiling](../decisions/ADR-022-planner-dormancy-and-spend-ceiling.md): the cap pattern the concierge mirrors
- [ADR-027: KiCad carrier generation](../decisions/ADR-027-kicad-carrier-generation.md): what a plan can generate
- [PRD-013: Rapid hardware prototyping](PRD-013-rapid-hardware-prototyping.md): bare-board lead-time estimate
- Issue #619: Gemini Live, a later replacement for the request sentinel
