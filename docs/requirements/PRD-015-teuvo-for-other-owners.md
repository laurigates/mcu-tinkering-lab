# PRD-015: A Teuvo of Your Own — Named, Customised Robots for Other People

**Status**: draft
**Created**: 2026-10-23
**Confidence**: 5/10 (the regulatory reading is interpretation, not legal advice; unit costs are estimates)

---

## Overview

People who like Teuvo may want one of their own. They would name it, choose
its initial personality and voice, and connect it to their own network. This
PRD records how that can be done and which routes the law, Google's API terms
and the current firmware rule out. It also lists the firmware work every route
needs. That work also lets Lauri run a second robot on his own network.

## Problem

Teuvo is built as one robot, in one home, on one key:

- **The persona is a compile-time table.** It has two entries, and "Teuvo" is
  baked into the fi-1950 brief and its 8 fallback lines (`voice_persona.c:76-88`,
  `:181-188`). NVS stores only the selected slug and a voice override.
- **Identity is fixed.** Every board has the same MQTT client id
  `robocar_unified`, the same topics and the mDNS name `robocar-unified`
  (`config.h:84-87`, `main.c:1949`). Two robots on one broker would knock each
  other off (MQTT 3.1.1 §3.1.4).
- **The Gemini key is compile-time only.** It is dropped after Improv, and
  OTA images carry none (ADR-028).
- **Nothing is protected at rest.** There is no NVS or flash encryption and no
  secure boot. MQTT is plaintext, with an anonymous read-only mode.
- **Spend is bounded per boot only.** The fuses reset on reboot and TTS has
  none, so nothing bounds a month's spend.
- **One network.** The robot remembers one network and cannot say it is lost
  (ADR-028).

## What decides the route

Sources were read 2026-10-23. Points marked *interpretation* are not legal
advice.

1. **Gemini API Additional Terms**, effective 2026-03-23, page updated
   2026-04-28. They hold whichever route or authentication scheme is chosen.
   - Users must be 18 or older, and an API client may not be "directed towards
     or … likely to be accessed by individuals under the age of 18". Vertex
     has the same clause (Cloud Service Specific Terms §20(d)).
   - The API is "for professional or business purposes, not for consumer use".
   - "You may use only Paid Services when making API Clients available to
     users in the European Economic Area".
   - The Robotics-ER privacy notice asks that identifiable people near the
     robot are notified and have consented.
2. **Supplying a unit in the course of a commercial activity makes Lauri its
   manufacturer** (Blue Guide 2022 §2.2–2.3). That holds for a sale, a
   commissioned build or a branded kit; a kit does not escape it (RED Annex I).
   The duties are:
   - a RED declaration covering 3.1(a) safety, 3.1(b) EMC, 3.2 radio and
     3.3(d)/(e) cybersecurity, applicable since 1.8.2025. EN 18031 gives no
     presumption of conformity if the user may skip setting a password
     (Decision 2025/138);
   - the CRA: Art 14 reporting since 11.9.2026, everything from 11.12.2027,
     with at least 5 years of support;
   - GPSR Art 19/20, RoHS;
   - WEEE and battery producer registration, supervised by Lupa- ja
     valvontavirasto since 1.1.2026;
   - the new Product Liability Directive for units placed after 9.12.2026;
   - AI Act Art 50(1), in force since 2.8.2026.
   Seeed's EU DoC and a CTI verification for the XIAO ESP32S3 / Sense exist
   (May 2023, EN 300 328 V2.2.2), so the radio reports may be reusable. They
   predate EN 18031 and cover no cybersecurity.
3. **Occasional supplies by hobbyists are in principle not commercial** (Blue
   Guide §2.2). Products made for one's own use are not placed on the market at
   all. *Interpretation:* CRA recital 15 counts monetising through a software
   platform as commercial activity. Selling paid upgrades through Teuvo's
   firmware (PRD-014) could therefore turn gifts into supplies. PRD-014 keeps
   its desk off by default for that reason.
4. **A toy is out.** A named, talking robot leans toward "toy" (Toy Safety
   Regulation 2025/2509 definition; Directive 2009/48/EC until 1.8.2030). A
   toy also brings EN 18031-2 parental control, CRA Annex III Class I, and
   the UCPD ban on exhorting children to buy. The Gemini age clause rules it
   out before cost does.

## Routes

| Route | Verdict | Why |
|---|---|---|
| **A. Open hardware.** Publish the build pack; each builder orders parts, assembles their own and uses their own paid Gemini project | **Default** | Nothing is placed on the market. Unmonetised FOSS is outside the CRA (recital 18) and the PLD (Art 2(2)). Lauri never handles other households' audio or images |
| **B. Occasional gifts to adult friends**, served through Lauri's paid proxy | **Possible, grey zone** | Hobbyist supply, but Lauri becomes GDPR controller for household audio and images: privacy notice, short DPIA, Google's DPA. It weakens once PRD-014 sells upgrades |
| **C. Sell finished units, commissioned builds or kits** | **Deferred** | Full manufacturer duties (above). Estimated €5–20k for a first declaration with lab work (unsourced estimate; get itemised quotes). Proportionate only at volume |
| **D. A Teuvo for children** | **Ruled out** | The Gemini age clause; toy, CRA Class I and UCPD duties |

## Goals

- One firmware image serves every Teuvo. Name, personality, voice, network
  and credentials are per-robot data in NVS, so they survive OTA
- Lauri can run two robots on one broker without them colliding
- Nothing on a robot can spend without a monthly bound once it leaves Lauri's
  desk
- Route A is complete enough that a capable adult can build and set up their
  own Teuvo from the repo
- Route C has a written checklist, so the decision is made on known cost

## Non-Goals

- Selling units now (route C) or any child-directed product (route D)
- Free-text personalities. Owners choose from presets, which keeps prompt size
  and injection in bounds
- Cloned voices
- A built-in SIM. ADR-028 §8 defers it; a phone hotspot or pocket router is the
  untethered path

## Features

### FR-032: Persona overlay in NVS

- **Data.** The compile-time table stays as the source of presets. NVS adds:
  - a display name;
  - the name's genitive form;
  - a preset personality slug;
  - a prebuilt voice (one of the 30);
  - an optional opener line of at most 80 bytes.
- **Name substitution.** Briefs and fallback lines take `{name}` and
  `{name_gen}` placeholders. Finnish inflects the name (Teuvo, Teuvon,
  Teuvolle), so lines are written to need only the nominative and the
  genitive, and the owner gives both. The VAD trigger is already name-agnostic
  (`speech_trigger.h:13-33`). The `'robotti'` literal and the `"Robocar"`
  fallbacks (`voice_turn.c:290`, `voice_history.c:129`,
  `gemini_backend.c:712`) move to the persona.
- **Prompt budget.** A host test builds every prompt for every preset with a
  maximum-length name and asserts that nothing is truncated. Today the voice
  turn cuts the fi-1950 brief at 511 bytes (`voice_turn.c:292-301`), and TTS
  keeps about 190 bytes of text under its style (`gemini_tts.c:238-240`).
- **Provisioning.** `persona name <nom> <gen>`, `persona preset <slug>` and
  `persona clear`, over serial only. An owner's choices therefore cannot be
  changed by an anonymous MQTT publisher.
- **Live probe.** `tools/live-probe` reads the persona from the C table and
  asserts the name is "Teuvo". It learns the overlay.
- **Voice design.** The Gemini changelog lists *Voice design* (2026-09-22). The
  `gemini-tts-voice` skill says custom voices are unavailable. Re-probe before
  offering it, and offer it only as a later option.

### FR-033: Per-device identity

The firmware derives a device id from the base MAC, for example the last 3
bytes in hex. It can be overridden over serial. It sets:

- MQTT client id `teuvo-<id>`;
- topic root `teuvo/<id>/` for status, commands, logs and PRD-014's upgrade
  topics;
- mDNS `teuvo-<id>`;
- `robot=<id>` in the facts line.

The broker ACL then allows each robot its own subtree. The old `robocar/*`
topics stay as aliases for one release.

### FR-034: Teuvo backend (Gemini proxy)

A small service holds one **paid** Gemini key, on a billing account separate
from Lauri's personal one. A Tier-1 cap of $250/month pauses every project on
its billing account.

**On the robot.** It calls the proxy instead of
`generativelanguage.googleapis.com`. All four endpoints already go through
`gemini_http_post()`, so this is a base-URL and header change (`gemini_http.c`).
The robot authenticates with a per-device token held in the FR-039 NVS slot,
with HMAC-based NVS encryption. A later version answers a challenge with a
read-protected eFuse HMAC key, so the secret cannot be copied off the chip.

**The proxy:**
- enforces a per-device monthly cost cap, revocation and a global kill switch;
- remaps model ids. That absorbs shutdowns such as the TTS model's 2026-11-17
  date without an OTA. It also absorbs price changes: ER 2 and 3.8 Flash list
  prices double on 1.1.2027;
- may set `mediaResolution` LOW on images, about 280 instead of 1,120 tokens.
  Unverified for ER 2;
- logs usage per device for billing and keeps no request content;
- returns 402 when a device's cap is reached. Teuvo then plays his
  "my credit is out" clip (ADR-028 §5).

**Cost per robot, estimated** at list prices on 2026-10-22, before any
measurement:

| | 2026 | 2027 |
|---|---|---|
| Idle, lived-in household | ~$0.89/day | ~$1.71/day |
| One hour of conversation | ~$1.48/h | ~$2.58/h |
| Reference month (30 idle days + 10 h of conversation) | ~$41 | ~$77 |

Cloudflare Workers Paid ($5/month per account) covers several hundred robots
by request count.

Measure first. Log ER 2 `usageMetadata` and a week of `plan`, `voice` and
`trace` counters on Lauri's robot before quoting anyone. TTS and voice turns
do not log tokens today.

Ephemeral tokens apply only to the Live API, so they cannot cover the planner
or TTS. They become relevant if #619 moves conversation to Live.

### FR-035: Owner onboarding and notices

These are one printed card plus a setup page. They state:

- **Adults only.** The unit is for adult-only households, and why: the Gemini
  terms.
- **Privacy.** Room audio and camera frames go to Google through the proxy, on
  the paid tier, with no retention at the proxy. It names who the controller
  is.
- **AI disclosure.** Teuvo is an AI system (AI Act Art 50(1)). The persona
  says so if asked.
- **People nearby.** The people around the robot should know about it
  (Robotics-ER notice). VAD chime-ins can be turned off with `voice vad off`.
- **Connecting.** How to join WiFi (ADR-028): Improv from the web flasher, a
  QR code from the phone's share-WiFi screen, or the phone hotspot set to
  2.4 GHz.
- **Mute and reset.** How to mute Teuvo and how to factory-reset. A reset
  erases the WiFi list, the token and the persona.

### FR-036: Open-hardware build pack (route A)

The repo already generates most of it:

- the build guide PDF;
- the carrier fab zip, BOM and CPL (`just robocar-unified::carrier-fab`);
- the web flasher.

FR-036 adds:

- a single "build your own Teuvo" page that links them;
- a parts list with order links;
- a setup guide for the builder's own paid Gemini project. The builder enters
  the key with `gemini auth` and sets a project spend cap. The guide states
  the 18+ and "not for consumer use" terms the builder accepts.

Nothing in route A is sold or shipped by Lauri.

## Route C checklist

This is not scheduled. Each item would be done before a first unit is sold:

1. A per-device token instead of any key on the device (FR-034). NVS
   encryption, flash encryption and secure boot, which are irreversible eFuse
   steps that change the web-flasher workflow.
2. MQTT over TLS with mandatory authentication, or no MQTT on customer units.
3. A password the owner must set before any network service is reachable.
   EN 18031 gives no presumption of conformity with a skip option.
4. The RED declaration and technical file. Get Seeed's EN 300 328 test reports
   and the tested antenna, or budget full radio tests. Add EN 18031-1/-2
   self-assessment and a documented interface list (GEC-4/5), including the USB
   console and ROM download mode.
5. Ship without Li-ion cells, or register as a battery producer. From
   18.2.2027, portable batteries must be user-removable (Battery Regulation Art
   11).
6. WEEE producer registration with LVV or a producer organisation; GPSR Art 19
   product information; AI Act Art 50 marking of synthetic audio.
7. CRA readiness: vulnerability handling, an SBOM, 5 years of support and
   reporting through ENISA's single reporting platform.
8. Consumer law: the 14-day withdrawal right. Whether a named, firmware-only
   personalisation counts as "clearly personalised" goods (KSL 6:16) is
   uncertain, so plan for returns.
9. An itemised quote from a test lab (e.g. DEKRA, Eurofins, Nemko) and a short
   legal opinion on the hobbyist/commercial line.

## Phases

| Phase | Scope |
|---|---|
| 1 | FR-033 identity, FR-032 overlay with the prompt-budget test, ADR-028 FR-039 key slot: Lauri runs two Teuvos |
| 2 | FR-036 build pack (route A), FR-035 notices |
| 3 | FR-034 proxy, only if route B is chosen, after a week of measured usage |
| 4 | Route C, only after the checklist is costed and accepted |

## Acceptance Criteria

- [ ] Two robots run on one broker for 24 h with distinct ids, topics and mDNS
  names and no disconnect loop
- [ ] A host test proves no persona prompt is truncated for any preset with a
  32-byte name
- [ ] A renamed robot answers to its name. Its fallback lines contain the new
  name and never "Teuvo"
- [ ] The persona, the WiFi list and the credential survive an OTA to a newer
  release
- [ ] If route B: a robot whose token is revoked stops calling Gemini within
  one request, and says so with an offline clip

## Open Questions

- Will any Teuvo, Lauri's own included, be used where children are present or
  likely to talk to it? The Gemini and Vertex terms decide this before any
  design does.
- Route A only, or also B (gifts through Lauri's proxy, which makes him a GDPR
  controller)? Would route C ever be worth a lab quote?
- Is Lauri's own robot on a billed Gemini project today?
- May units given away have eFuses burned for NVS encryption and device
  identity? This is irreversible and limits reflashing through the web flasher.
- Which presets ship? How many personalities and voices, in which languages?
- Should units ever ship with 18650 cells?

## Related Documents

- [ADR-028: Teuvo connectivity](../decisions/ADR-028-teuvo-connectivity-resilience.md): networks, runtime credentials, offline voice
- [PRD-014: Teuvo sponsored upgrades](PRD-014-teuvo-sponsored-upgrades.md): why its desk is off by default
- [ADR-024: Conversational entity](../decisions/ADR-024-conversational-entity-and-multimodal-voice.md)
- [ADR-022: Planner dormancy and spend ceiling](../decisions/ADR-022-planner-dormancy-and-spend-ceiling.md)
- [PRD-010: ThinkPack](PRD-010-thinkpack-modular-toys.md): toddler-facing toys whose Brainbox can call Gemini, Claude or Ollama. The Gemini age clause applies to its Gemini backend too; the other providers' terms were not checked
- Issue #619: Gemini Live; issue #780: OLED driver
