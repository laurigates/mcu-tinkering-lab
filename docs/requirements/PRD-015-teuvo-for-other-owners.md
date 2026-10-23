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

Sources were read 2026-10-23. Everything below is interpretation for planning,
not legal advice. Points marked *interpretation* go beyond what the cited text
says.

1. **Gemini API Additional Terms**, effective 2026-03-23, page updated
   2026-04-28. They hold whichever route or authentication scheme is chosen.
   - Users must be 18 or older, and an API client may not be "directed towards
     or … likely to be accessed by individuals under the age of 18". Vertex
     has the same clause (Cloud Service Specific Terms §20(d)).
   - The API is "for professional or business purposes, not for consumer use".
   - "You may use only Paid Services when making API Clients available to
     users in the European Economic Area".
   - The Robotics-ER privacy notice is a term the developer agrees to. No
     identifiable person may interact with the robot, or be in the area around
     it, unless they have been notified and have consented. Collection of
     personal data must be kept to a minimum.
2. *Interpretation:* **supplying a unit in the course of a commercial activity
   makes Lauri its manufacturer** (Blue Guide 2022 §2.2–2.3, §3.1; RED Art
   2(1)(12)). That holds for a sale, a commissioned build or a branded kit. RED
   Annex I exempts only radio-amateur kits and professional evaluation kits. The
   duties are:
   - a RED declaration covering 3.1(a) safety, 3.1(b) EMC, 3.2 radio and
     3.3(d)/(e) cybersecurity. The cybersecurity part (Delegated Reg 2022/30)
     applies from 1.8.2025 to 10.12.2027. Reg 2026/339 repeals it, and the CRA's
     essential requirements replace it for units placed from 11.12.2027. EN
     18031 gives no presumption of conformity if the user may skip setting a
     password (Decision 2025/138);
   - the CRA: Art 14 reporting since 11.9.2026, everything from 11.12.2027,
     with at least 5 years of support;
   - GPSR Art 19/20, RoHS;
   - WEEE and battery producer registration, supervised by Lupa- ja
     valvontavirasto since 1.1.2026;
   - the new Product Liability Directive for units placed after 9.12.2026;
   - AI Act Art 50(1), applicable since 2.8.2026.

   Seeed's EU DoC and a CTI verification for the XIAO ESP32S3 / Sense exist
   (May 2023, EN 300 328 V2.2.2), so the radio reports may be reusable. They
   predate EN 18031 and cover no cybersecurity.
3. **Occasional supplies by hobbyists are in principle not commercial** (Blue
   Guide §2.2). Products made for one's own use are not placed on the market at
   all. *Interpretation:* CRA recital 15 counts monetising through a software
   platform as commercial activity. Selling paid upgrades through Teuvo's
   firmware (PRD-014) could therefore turn gifts into supplies. PRD-014 keeps
   its desk off by default for that reason.
4. **A toy is out.** *Interpretation:* a named, talking robot leans toward
   "toy" (Toy Safety Regulation 2025/2509 definition; Directive 2009/48/EC
   until 1.8.2030). A toy also brings EN 18031-2 parental control, CRA Annex
   III Class I, and the UCPD ban on exhorting children to buy. The Gemini age
   clause rules it out before cost does.

## Routes

| Route | Verdict | Why |
|---|---|---|
| **A. Open hardware.** Publish the build pack; each builder orders parts, assembles their own and uses their own paid Gemini project | **Default** | Nothing is placed on the market. Unmonetised FOSS is outside the CRA (recital 18) and the PLD (Art 2(2)). Lauri never handles other households' audio or images |
| **B. Occasional gifts to adult friends** | **Possible, grey zone** | Hobbyist supply. It weakens once PRD-014 sells upgrades. For a handful of gifts the first step needs no proxy. Either use one Google project per robot on a separate fleet billing account, with a project spend cap (experimental) and a key restricted to the Gemini API, stored with `gemini auth` (FR-039); revoking means disabling that key. Or the recipient brings their own paid project, which also keeps Lauri from being GDPR controller. With Lauri's key or proxy, he likely becomes controller for household audio and images (interpretation): privacy notice, short DPIA, Google's DPA |
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
  - an optional opener line of at most 80 bytes;
  - a persisted VAD on/off setting.

  Offline lines (ADR-028 §5) use the default voice until a matching set has
  been rendered.
- **Name substitution.** Briefs and fallback lines take `{name}` and
  `{name_gen}` placeholders. Finnish inflects the name (Teuvo, Teuvon,
  Teuvolle), so lines are written to need only the nominative and the
  genitive, and the owner gives both. The VAD trigger is already name-agnostic
  (`speech_trigger.h:13-33`). The `'robotti'` literal and the `"Robocar"`
  fallbacks (`voice_turn.c:290`, `voice_history.c:129`,
  `gemini_backend.c:712`) move to the persona.
- **Prompt budget.** A host test builds every prompt for every preset with a
  32-byte name and asserts that nothing is truncated. Today the voice turn cuts
  the fi-1950 brief at 511 bytes (`voice_turn.c:292-301`), and TTS keeps about
  190 bytes of text under its style (`gemini_tts.c:238-240`). The test builds
  against the migrated TTS request (FR-043).
- **Provisioning**, over serial only, so an anonymous MQTT publisher cannot
  change an owner's choices:
  - `persona name <nom> <gen>`, `persona preset <slug>`, `persona clear`;
  - `voice vad off persist`;
  - `reset factory`, which erases the network list, the `gemini`, `mqtt_auth`
    and `voice` namespaces and the overlay, then reboots.
- **Live probe.** `tools/live-probe` reads the persona from the C table and
  asserts the name is "Teuvo". It learns the overlay.
- **Voice design.** The Gemini changelog lists *Voice design* (2026-09-22). The
  `gemini-tts-voice` skill says custom voices are unavailable. Re-probe before
  offering it, and offer it only as a later option.

### FR-033: Per-device identity

The firmware derives a device id from the base MAC, for example the last 3
bytes in hex. It can be overridden over serial. When a broker username is
set, it is the device id: the topic root is taken from it, and an override that
differs from it is refused. Otherwise every publish would be silently denied
while MQTT reports connected. `mqtt auth` prints the topic root the username
implies. The id sets:

- MQTT client id `teuvo-<id>`;
- topic root `teuvo/<id>/` for status, commands, logs and PRD-014's upgrade
  topics;
- mDNS `teuvo-<id>`;
- `robot=<id>` in the facts line.

The old `robocar/*` topics stay as aliases for one release. The mDNS change
departs from the repo's hostname convention, the project directory name. So
`.claude/rules/mdns-hostname.md` and the comment at `main.c:1940-1942` change
in the same commit.

The broker ACL gives one broker user per robot, named by its device id, and
uses `%u`, never `%c`. With `%c`, any credential could claim another robot's
subtree. The broker sets `use_username_as_clientid true`, so no credential can
take another robot's client id either. The `teuvo-<id>` client id the firmware
sends is then informational. The Mosquitto ACL:

- **Every robot**, as pattern lines with `%u` = device id:
  - `pattern write teuvo/%u/status`, `teuvo/%u/logs/#` (the firmware publishes
    to `<log_topic>/<level>`), `teuvo/%u/ota/status` and
    `teuvo/%u/upgrade/request`;
  - `pattern read teuvo/%u/commands`, `teuvo/%u/upgrade/facts`,
    `teuvo/concierge/status` and `robocar-unified/ota/notify`.

  Pattern lines apply to every user, so the `concierge` user's write on
  `teuvo/concierge/status` comes from the first one.
- **`user concierge`:** `topic write teuvo/+/upgrade/facts`,
  `topic read teuvo/+/upgrade/request` and `topic read teuvo/+/status`. The
  status grant lets PRD-014 FR-025 republish a robot's facts when it comes
  online.
- **`user ci-notify`**, the credential for `build-firmware.yml`'s notify step:
  `topic write robocar-unified/ota/notify`.
- **Lauri's users**, one per client program (for example `lauri-cli` for
  `mosquitto_pub`, `lauri-watch` for `mosquitto_sub`), each with
  `topic write teuvo/+/commands` and `topic read teuvo/#`. Under
  `use_username_as_clientid` a username holds one connection at a time, so a
  second program on the same user would disconnect the first.

The ACL grants the `robocar/*` aliases nothing, so it goes live in the release
that drops them.

### FR-034: Teuvo backend (Gemini proxy)

It is built only beyond about five gifted robots, or when a model change needs
central remapping. Until then route B uses one capped project per robot (see
Routes). That key sits unencrypted in NVS until NVS encryption is decided, and
a model change needs an OTA.

The service holds one **paid** Gemini key, on a billing account separate from
Lauri's personal one. A Tier-1 cap of $250/month pauses every project on its
billing account.

**On the robot.** It calls the proxy instead of
`generativelanguage.googleapis.com`. All four endpoints already go through
`gemini_http_post()`, so this is a base-URL and header change (`gemini_http.c`).
The robot authenticates with a per-device token held in the FR-039 NVS slot,
with HMAC-based NVS encryption. A later version answers a challenge with a
read-protected eFuse HMAC key, so the secret cannot be copied off the chip.

**The proxy:**
- enforces a per-device monthly cost cap, revocation and a global kill switch;
- remaps model ids for later shutdowns whose replacement accepts the same
  request, always to a model that speaks Finnish. It does not cover the
  2026-11-17 TTS shutdown: that migration is firmware work (FR-043), because
  the 3.8 TTS models read input as a verbatim transcript and their streaming
  is unverified. It also absorbs price changes: ER 2 and 3.8 Flash list prices
  double on 1.1.2027;
- may set `mediaResolution` LOW on images, about 280 instead of 1,120 tokens.
  Unverified for ER 2;
- logs usage per device for billing and keeps no request content;
- returns 402 when a device's cap is reached, and Teuvo plays his credit clip
  (ADR-028 §3–§5).

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
- **Privacy.** Room audio and camera frames go to Google on the paid tier,
  directly or through the proxy, with no retention at the proxy. The card
  names who the controller is.
- **AI disclosure.** Teuvo is an AI system (AI Act Art 50(1)). The persona
  says so if asked.
- **People nearby.** Google's Robotics-ER terms require that everyone near
  the robot, guests included, has been told about it and has consented. The
  card says this duty is the owner's and how to meet it: tell people, ask them,
  and switch the robot off when someone has not consented.
  `voice vad off persist` stops only spontaneous voice turns; the planner still
  sends camera frames.
- **Connecting** (ADR-028).
  - For a gifted unit (route B), Lauri adds the owner's home network and phone
    hotspot with `wifi add` before handover, **without** the `home` flag, and
    runs `mqtt auth clear`. The compiled-in broker address is on Lauri's LAN and
    means nothing on the owner's network, and `home` marks only a network where
    the configured broker lives (ADR-028 §6). A gifted unit runs no MQTT unless
    a broker is set up for that household, or MQTT runs over TLS (Route C item
    2).
  - A builder (route A) uses Improv from a desktop Chromium browser.
  - Android's share-WiFi QR code works with QR join (ADR-028 §6). The iOS 18
    code is listed only after a bench test confirms its payload.
  - A phone hotspot must be 2.4 GHz, and it is added like any other network.
- **Mute and reset.** How to mute Teuvo (`voice vad off persist`, or power
  off) and how to factory-reset (`reset factory`).

### FR-036: Open-hardware build pack (route A)

The repo has the tooling for most of it:

- the build guide PDF;
- the carrier fab zip, BOM and CPL, which `just robocar-unified::carrier-fab`
  generates locally with KiCad 10.0.6 (`output/` is gitignored);
- the web flasher.

FR-036 adds:

- Lauri runs `carrier-fab` for each carrier revision and attaches the Gerber
  zip, BOM and CPL to a release;
- a single "build your own Teuvo" page that links them;
- a parts list with order links;
- a setup guide for the builder's own paid Gemini project. The builder enters
  the key with `gemini auth` and sets a project spend cap. The guide states the
  18+, "not for consumer use" and Robotics-ER consent terms the builder
  accepts.

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
   and the tested antenna, or budget full radio tests. Add an EN 18031-1/-2
   self-assessment if the unit is placed before 11.12.2027, or CRA Annex I
   conformity after that date. Add a documented interface list (GEC-4/5),
   including the USB console and ROM download mode.
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
| 1 | FR-033 identity (#851), FR-032 overlay (#852) with the prompt-budget test (#847), ADR-028 FR-039 key slot (#845): Lauri runs two Teuvos |
| 2 | FR-036 build pack (route A), FR-035 notices |
| 3 | If route B: per-robot capped projects first; FR-034 proxy only past about five robots, after a week of measured usage |
| 4 | Route C, only after the checklist is costed and accepted |

## Acceptance Criteria

- [ ] Two robots run on one broker for 24 h with distinct ids, topics and mDNS
  names and no disconnect loop. Neither can publish to the other's topics
- [ ] A host test proves no persona prompt is truncated for any preset with a
  32-byte name
- [ ] A renamed robot answers to its name. Its fallback lines contain the new
  name and never "Teuvo"
- [ ] The persona, the WiFi list, the VAD setting and the credential survive an
  OTA to a newer release, and `reset factory` erases them
- [ ] The owner card and setup page state the adults-only rule and the
  Robotics-ER notice-and-consent duty for everyone near the robot
- [ ] If route B with a proxy: a robot whose token is revoked stops calling
  Gemini within one request, and says so with an offline clip

## Open Questions

- Will any Teuvo, Lauri's own included, be used where children are present or
  likely to talk to it? The Gemini and Vertex terms decide this before any
  design does.
- Route A only, or also B? With per-robot projects, or the recipient's own
  project? Would route C ever be worth a lab quote?
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
- Issue #619: Gemini Live; issue #780: OLED geometry (driver in `robocar/bringup`)
