# ADR-028: Teuvo Connectivity — Several Networks, Services Armed on Link-Up, QR Join and an Offline Voice

**Status**: proposed
**Date**: 2026-10-23
**Source**: conversation 2026-10-23 (four repo-reading and five web research passes, each fact-checked)
**Confidence**: 7/10, self-assessed (read from source, none of it run on hardware; unmeasured: QR read range, decode time and heap on the OV3660, and `gemini-3.8-flash-tts` behaviour, FR-043)
**Related**: [PRD-014](../requirements/PRD-014-teuvo-sponsored-upgrades.md), [PRD-015](../requirements/PRD-015-teuvo-for-other-owners.md)

---

## Context

`robocar-unified` (Teuvo) was built for one robot on one network. Read from
source on 2026-10-23, not run on hardware:

**One network.** NVS namespace `wifi_config` holds a single `ssid`/`password`
pair, and Improv overwrites it (`credentials_loader.c:17-19`, `:248-272`).
`wifi_connect()` uses `WIFI_FAST_SCAN`, which stops at the first matching AP,
so its `WIFI_CONNECT_AP_BY_SIGNAL` sort has no effect (`wifi_manager.c:182-190`;
ESP-IDF v5.4 Wi-Fi guide). The threshold `WIFI_AUTH_WPA_WPA2_PSK` rejects open,
WEP, WPA1 and OWE networks (`wifi_manager.c:185`), while the loader still says
an empty password means an open network (`credentials_loader.c:177-180`).
`wifi_connect()` blocks `app_main` until it connects or its fast retries fail.
The executor and console start only after that (`wifi_manager.c:199-213`).

**Services armed once, at boot.**
- Improv starts only if WiFi is down at the end of `init_network()`
  (`main.c:1927-1939`). A robot that loses its link later cannot be
  re-provisioned without a reboot.
- MQTT and OTA start only if WiFi is up at that moment, from a single call
  (`main.c:2054-2059`, `:2195`). A robot that boots offline never gets them,
  not even after Improv succeeds.
- `ota_manager_init()` also arms the rollback-confirm timer
  (`ota_manager.c:116-128`). With `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, an
  OTA image whose first boot is offline is never confirmed and rolls back at
  the next reset.

**The Gemini key is compile-time only, and gets dropped.** Once Improv has
written `wifi_config`, `load_credentials()` reads the key only from `getenv()`,
which is empty on the device. The `GEMINI_API_KEY` from `credentials.h` is
skipped (`credentials_loader.c:120-126`). `credentials_reload()` runs straight
after Improv saves (`main.c:1502`), so the planner, TTS and voice turns go
silent at once. The CI, web-flasher and OTA images are built with a stub that
defines no key (`CMakeLists.txt:3-22`). Inferred from code, not run on
hardware: a board with its key compiled in loses it at its first OTA to a newer
release.

**Offline, Teuvo is mute and deaf.** Every voice path is cloud Gemini: TTS,
voice turns, wake-name classification and the self-report narrate.
`self_report` speaks only when `wifi_up && key_present && audio_ok`
(`self_report.c:112-115`). The persona's `fault_wifi` phrase therefore can
never be spoken. While offline, planner and voice requests still run against
the per-boot fuses: 500 planner and 200 voice requests (`plan_budget.h:81`,
`voice_budget.h:111`). A long outage can leave the planner fused off once the
link is back.

**Other limits.**
- The radio is 2.4 GHz only. iPhone 12 and later hotspots need *Maximize
  Compatibility* turned on.
- The build has no BLE. Internal RAM was once driven to 259 bytes, before
  mbedTLS moved to PSRAM.
- The shared `improv-wifi` component treats Improv Serial command 0x02 as
  *Identify* rather than *Request current state*. It sends 0xFE instead of 0x03
  for "unable to connect", and it has no scan RPC (`improv_wifi.c:32-34`,
  `improv_wifi.h:46`; improv-wifi.com/serial).
- Nothing decodes a QR code or reads text on the device.
- The 960 KB `spiffs` partition is allocated but unused (`partitions.csv:18`).
  The app image is 1.34 MB of a 3.5 MB OTA slot (v0.2.13).
- The 128x32 OLED is wired (`config.h:158`), but `robocar-unified` has no OLED
  code. `robocar/bringup/main/oled.c` drives the same panel; #780 tracks its
  geometry.
- MQTT is plaintext to a bare private IP (`config.h:82`), and the broker
  credentials travel in the CONNECT packet.

The owner's notes ask for Teuvo to remember several networks, say when he is
disconnected and ask for help, join from a WiFi QR code, and, as play, hunt
for a connection. Every Teuvo built for someone else (PRD-015) depends on the
same fixes.

## Decision

### 1. Keep a list of networks in application code

ESP-IDF holds one STA config, so the list lives in the app. Espressif ships no
multi-AP component, and the three third-party ones bring their own
provisioning stacks.

- A new pure-C `wifi_netlist_core.{c,h}`, host-tested under the shared-core
  pattern (`.claude/rules/testing.md`). It holds up to 8 entries in one
  versioned NVS blob: SSID (≤32), PSK (≤64), flags (hidden, open-allowed,
  SAE-only, `home`), BSSID and channel hint, last RSSI, last-success boot
  counter, failure count, and source (improv, console, qr). First boot migrates
  the legacy `wifi_config` pair or, when NVS holds none, the compile-time WiFi
  credentials from `credentials.h` (developer builds), as one entry with the
  `home` flag set, because MQTT already runs on that network. A migration
  marker in the blob keeps this a one-time step. Marker and entries land in
  one `nvs_set_blob` and `nvs_commit`, and the legacy keys stay in place, so an
  interrupted migration reruns at the next boot. A blob that fails to read, or
  carries an unknown version, as an OTA rollback to an image with an older
  blob version produces, is not used; whether the migration then rewrites it
  is open (Open Questions).
- Selection runs an all-channel scan, intersects it with the list, and ranks by
  RSSI plus recency minus a failure penalty. It then connects with a BSSID and
  channel hint. Reasons 15/202/204 mark a credential as suspect; 201/210/211
  mean the network is not here.
- `esp_wifi_set_storage(WIFI_STORAGE_RAM)` stops the driver writing its own
  copy to flash on every switch. The app list is the one store.
- The WPA2 minimum stays. An open network is joined only if its entry carries
  the owner's explicit open-allowed flag.
- The serial console gets `wifi list|add|forget|scan|home`. It is serial-only,
  like `mqtt auth`, so a password never travels over MQTT. Apart from the
  migration, the `home` flag is set only here (§6). Entries saved through
  Improv or QR join never get it.
- A phone hotspot is an ordinary entry. The docs say it must be 2.4 GHz.
- The 32-character SSID and 64-hex-digit PSK edge cases are fixed:
  `credentials_t` and the NVS read buffers become 33/65
  (`credentials_loader.h:17-18`, `credentials_loader.c:38-45`). `wifi_connect`
  copies the full 32/64 bytes into `sta.ssid`/`sta.password` without the `- 1`
  (`wifi_manager.c:192-193`); those fields need no NUL at full length.

### 2. Arm network services on link-up, not at boot

- Connecting becomes a state machine driven by WiFi events. `app_main` and the
  Improv callback never block on it, so the console and executor start at once.
- Errors from `set_mode`, `set_config` and `start` are logged, not
  `ESP_ERROR_CHECK`ed, so a failed Improv attempt cannot reboot the board.
- The reconnect timer (`wifi_manager.c:52-64`) stops before any scan, because a
  scan started during a connect fails with `ESP_ERR_WIFI_STATE`.
- On the first `IP_EVENT_STA_GOT_IP` of a boot, on any network, OTA starts,
  including the rollback-confirm timer (`OTA_STABILITY_TIMEOUT_MS`, 60 s,
  `config.h:147`). Time spent joining does not count against it; a reset
  within 60 s of that first `GOT_IP` still rolls back.
- MQTT follows the link instead (§6). The client starts on every `GOT_IP` whose
  entry has the `home` flag, and stops with `esp_mqtt_client_stop()` on every
  `WIFI_EVENT_STA_DISCONNECTED`, before another network is joined.
- `mqtt_logger_init()` creates the client at boot without starting it, so OTA
  and the upgrade desk register their subscriptions whichever network comes
  first; start and stop only toggle the connection. `esp_mqtt_client_stop()`
  dispatches no `MQTT_EVENT_DISCONNECTED`, so the logger clears its connected
  flag itself on stop and runs the same disconnect path.
  `CONFIG_MQTT_SKIP_PUBLISH_IF_DISCONNECTED=y`, so nothing queues in esp-mqtt's
  outbox while the client is stopped.
- Improv re-arms when the link has been down for more than a set time, and
  disarms on reconnect.
- The planner, voice turns and TTS skip their request while there is no link,
  so offline time does not spend the per-boot fuses.
- `improv-wifi` gets 0x02 *current state*, error code 0x03 and real device info.
  Scan (0x04) follows later. CI rebuilds every project that lists
  `components/improv-wifi` in `extra_paths`: seven, per
  `.github/project-matrix.json`, since #670 was fixed. CI compiles the protocol
  but never runs it, so bench-test Improv on `robocar-unified` and on at least
  one other consumer.

### 3. Hold the Gemini credential in NVS

- A serial-only `gemini auth <key>` / `gemini auth clear` command stores the
  key in its own NVS namespace. It is modelled on `mqtt auth`, with the same
  redaction and presence-only reporting.
- The key loads from NVS, then `credentials.h`, whatever the WiFi source.
- The same slot later holds a proxy device token (PRD-015 FR-034).
- `gemini_http.c` handles HTTP 402 (prepay balance exhausted) and 403 (key
  blocked or unrestricted) separately from other failures, so Teuvo can say
  which one it is.

### 4. Make connectivity a stated fact

`wifi_manager` exposes one state: `CONNECTED`, `SEARCHING`, `NO_KNOWN_NETWORK`,
`AUTH_FAILED`, `NO_KEY` or `NO_CREDIT`, plus the connected SSID and RSSI.
`self_report` reports the SSID it is actually connected to, not the cached
credential. `activity_trace` gets a distinct no-link LED state, which today
reads the same as an API error.

### 5. Give Teuvo a small offline voice

- **The lines.** 8–12 persona lines are pre-rendered through Gemini TTS by a
  local `just` recipe. Examples: "I seem to have lost the network", "show me
  the WiFi QR code", "connected, thank you", "my key is not working", "my
  credit has run out". They are stored as 24 kHz IMA-ADPCM, at about 12 kB/s,
  so 40–60 s is about 0.5–0.7 MB. The render uses the migrated TTS request
  (FR-043). There is no desk-closed line: with no link no voice turn runs, so
  nothing can ask for an upgrade, and with a link the voice turn's own reply
  says the desk is closed (PRD-014 FR-024).
- **Where they live.** The default set is **embedded in the app image**
  (`EMBED_FILES`). OTA updates only the app partition, and there is room for
  them (1.34 of 3.5 MB used).
- **Voice and style.** Lines are name-free, but each clip fixes the voice and
  style it was rendered in. The embedded set uses the default persona and
  voice. A robot whose owner picks another voice or preset (PRD-015 FR-032)
  plays the default set until it has its own, so offline it speaks in the
  default voice. That set is rendered on the device at the first link-up after
  the voice or preset changes, into the unused `spiffs` partition (about 80 s
  of IMA-ADPCM). A header naming the persona and voice is checked at playback
  before the set is preferred.
- **When they play.** Playback goes through `audio_player`, like live TTS. A
  help request plays in `NO_KNOWN_NETWORK` or `AUTH_FAILED`. It needs the VAD
  to hear someone nearby, is rate-limited, and comes with a buzzer motif and an
  amber LED. With the VAD off, only the buzzer and LED signal it (Open
  Questions). The key and credit lines play once on entering `NO_KEY` or
  `NO_CREDIT` (§3, §4).

### 6. Join from a WiFi QR code shown to the camera

- **Decoding.** It uses `espressif/quirc` 1.2.0 (ISC). Its input is a
  full-scale luma plane taken from the JPEG frames the camera already produces,
  decoded into quirc's image buffer in PSRAM. At VGA that buffer is about
  300 KB (640x480 bytes). With the default `QUIRC_MAX_REGIONS`, which the
  component does not override, quirc's pixel map aliases it, and its flood-fill
  stack adds about 5 KB (`quirc_resize()`; `h*2/3` entries of 16 bytes).
  The camera is never reconfigured to GRAYSCALE, which would need a deinit and
  init and would disturb the planner.
- **QR mode.** It raises the frame size to VGA, inside the 64 KB JPEG buffer,
  which the bench must confirm. It is entered from the console, or by itself in
  `NO_KNOWN_NETWORK`.
- **The parser.** A pure-C `wifi_qr_core` parser accepts both dialects:
  - ZXing/Android: backslash escapes, `T:WPA`/`T:SAE`/`T:WEP`/`T:nopass`,
    fields in any order;
  - the WPA3 spec v3.5 §7.1: `T:WPA` for every password network (SAE or PSK;
    no other value is defined), `T` absent only for an open or Enhanced Open
    network, percent-encoding, `R:` (Transition Disable), `I:`, `K:`, unknown
    fields ignored.

  A host test pins how a code with no `T` is classified (open under both
  dialects), including one that still carries `P:`.
- **Policy.**
  - A QR-joined entry is saved with source `qr` and without the `home` flag.
  - MQTT, which sends the broker credential in plaintext, runs only while the
    robot is associated with an entry whose `home` flag is set (§2). esp-mqtt
    reconnects every 5 s on its own (`mqtt_logger.c:150`), so stopping the
    client on every disconnect is what keeps it from sending CONNECT on another
    network. On any other network only TLS services run: Gemini and the HTTPS
    OTA manifest. Otherwise an AP that gives itself the broker's address would
    receive the credential, and could then send the full command set. A host
    test pins the start/stop decision for the sequences home→hotspot,
    hotspot→home and qr→home. Lift this only once MQTT runs over TLS with a
    pinned broker certificate.
  - While a known network is reachable, a QR is accepted only after a console
    confirmation. A voice confirmation never authorises a join.
  - WEP is refused. An open network is refused unless the owner allowlisted it.
  - Each scanned credential gets at most 2 attempts.
  - An entry is saved only after `GOT_IP`, as Improv does today.

### 7. The WiFi hunt is opt-in play, built last

The hunt runs only when the owner has enabled it and the link has been down for
more than a set time. It has three moves:

- a head sweep in QR mode, at table and human height;
- a hill-climb on a **known** network's RSSI, heavily averaged;
- asking people for help with the offline clips.

Indoor RSSI noise is about 4 dB, and a ±1 dB local mean needs about 5 m of
travel at 2.4 GHz (Blum & Hafner 2015; Lee 1985). The robot also has no
encoders, IMU or odometry. So the hunt is a game and RSSI-guided wandering back
toward a remembered AP, not a router finder.

Limits:
- no open networks unless allowlisted;
- no guessing or brute force; no WPS PIN; no MAC cloning; no captive-portal
  automation;
- at most 2 attempts per credential;
- time-boxed, with a cap on total drive time at a fixed low speed, since
  distance cannot be measured;
- it stops on the obstacle reflex, and runs on a supervised single-level floor
  only, since there is no cliff sensor.

Reading a handwritten password needs Gemini, because no OCR runs on the
ESP32-S3 (PP-OCRv6 is ESP32-P4 only). So it works only once online, for example
through a phone hotspot. It must be started by the owner, use a paid-tier key,
and have the network owner's consent. *Interpretation:* a WiFi password is the
network owner's secret. Outside the EEA, the unpaid Gemini terms say not to
submit sensitive information. In the EEA the paid data terms also cover unpaid
quota, but any unit made available to someone else must be on Paid Services
anyway.

### 8. Deferred

- **BLE provisioning** (`network_provisioning` Security2 with a per-robot PoP
  sticker). It is the only verified iPhone provisioning path that needs neither
  a camera QR nor a pre-known hotspot. iOS 18's Passwords app can show a
  network QR code, but its payload is unverified, so the §6 bench test includes
  an iPhone-generated code. Revisit BLE for PRD-015 after measuring NimBLE's
  internal-RAM cost.
- **SoftAP captive portal.** It adds an HTTP server and DNS hijack. SoftAP
  visibility on the S3 is an open upstream issue (esp-idf#13508, where RF
  causes are discussed). The repo's `wifitest` turns AMPDU off as an
  unverified workaround, which would cost STA throughput. Bench it before
  relying on it.
- **A built-in SIM modem.**
  - The hardware is not ready. Only D3 and D7 are free, so there is no RTS/CTS,
    and the carrier and the 2x7 ribbon do not route those pads. The 5 V LM2596
    rail is at its limit with two motors stalled (estimate; the stall current
    is simulated, not measured).
  - The bandwidth may not be there. Real-time 24 kHz PCM TTS needs about
    0.6 Mbit/s once SSE overhead is counted (estimate; about 0.51 Mbit/s of
    bare base64). That rules out LTE-M. On 0.5 Mbit/s Finnish device plans,
    lines longer than about 6 s would underrun unless the 1 s pre-roll grows
    (estimate; measure the bytes per utterance first).
  - Until then, a phone hotspot or a pocket LTE router is the untethered path,
    as a list entry.
  - If a modem ever comes: an A7670E-class Cat-1 module, LTE-only
    (`AT+CNMP=38`), at 921600 baud or more, with its own 3.8 V 2 A buck from
    VBAT, through a carrier revision.
  - Re-evaluate if TTS moves to `gemini-3.8-flash-tts`'s 8 kHz µ-law output on
    the Interactions API, which needs roughly a sixth of the downlink (Gemini
    speech-generation docs, 2026-10-12). Flash-Lite TTS has no Finnish.

## Alternatives Considered

| Alternative | Why not |
|---|---|
| Third-party multi-AP component (`thorrak/esp_wifi_config`, `embedblocks/wifi-manager`) | Single-maintainer, pre-1.0, brings SoftAP/REST/BLE stacks that overlap Improv; one needs IDF ≥5.5 |
| Switch the camera to GRAYSCALE for QR | Needs `esp_camera_reconfigure()` (deinit + init), which disrupts the planner's JPEG stream |
| Default offline clips only in the `spiffs` partition | Not covered by OTA; the default set would freeze at the first flash. `spiffs` holds only on-device re-renders (§5) |
| On-device OCR for written passwords | No OCR model runs on the ESP32-S3 |
| Joining any open network when lost | Unsafe and possibly against the network owner's terms; kept behind an explicit allowlist |
| SoftAP or BLE now | Unverified S3 SoftAP visibility issue; unmeasured RAM on an already tight heap |
| SIM modem now | Pins, ribbon, power budget and TTS bandwidth all need a hardware revision; hotspot covers the use case |

## Consequences

**Positive**
- Teuvo survives a move between home, a phone hotspot and a second site
  without a reflash.
- A robot that boots offline, or loses its link mid-uptime, recovers OTA, MQTT
  (on `home` networks) and provisioning on its own.
- An OTA image whose first boot is offline is no longer rolled back, provided
  it gets a link before its next reset. An image that is reset before it ever
  connects still rolls back, which keeps the safety net against images that
  break networking.
- Boot no longer waits on WiFi, so the console and the reflex are up at once.
- Teuvo can say he is lost and what would help, in character, with no network.
- The key survives Improv and OTA. Every PRD-015 unit needs this.
- Selection, the QR parser and the store are pure C and pinned by host tests.

**Negative**
- `wifi_manager.c` stops matching the canonical pattern in the `wifi-sta-setup`
  skill. The skill is updated in the same change, including its esp-idf#13508
  note.
- An all-channel scan adds about 1.5 s to each connect attempt (estimate).
- Committed clip assets go stale when the default voice or style, or the TTS
  request (FR-043), changes, and they are re-rendered by hand.
- On a network without the `home` flag, MQTT logging, status and remote
  commands are off.
- A shown QR code is still a way to move the robot onto another network. TLS
  with the certificate bundle protects Gemini and OTA traffic. The `home` rule
  keeps the plaintext MQTT credential off such networks.
- Secrets in NVS stay plaintext until NVS encryption is enabled. That is
  irreversible eFuse work, decided under PRD-015.

## Implementation

In order. Each step lands with host tests where the logic is pure C. FR
numbers continue from PRD-014 and PRD-015, and each step below is one FR. The
order across all three Teuvo documents, including which PRD-014 and PRD-015
work waits on which step here, is the sequenced plan in PRD-014 § Phases.

0. FR-043: the TTS migration off `gemini-3.1-flash-tts-preview`, whose earliest
   shutdown is 2026-11-17. Probe `gemini-3.8-flash-tts`, not Flash-Lite, which
   has no Finnish. Check streaming on `:streamGenerateContent?alt=sse`, whether
   the `<style>: <text>` prefix is spoken now that 3.8 reads input as a verbatim
   transcript, the delivery tags and the output format. Then migrate
   `gemini_tts.c`. Issue #844.
1. FR-039: runtime Gemini credential (§3), which fixes the key-drop bug.
   Issue #845.
2. FR-038: services armed on link-up, a non-blocking connect, and requests
   skipped offline (§2). Until FR-037 lands, the single stored network counts
   as `home`, so MQTT starts only on it and stops on every disconnect.
   Issue #846.
3. FR-037: the network list, all-channel scan, console commands and the `home`
   flag (§1, §4). Issue #848.
4. FR-040: offline clips and the help request (§5). Issue #849.
5. FR-041: QR join (§6). Bench first: quirc decode time, read distance and
   free PSRAM and internal heap during a decode, at QVGA and VGA on the OV3660,
   with Android and iPhone codes. If VGA does not fit, QR mode stays at QVGA.
   Issue #850.
6. FR-042: the hunt (§7), after the steps above have run on the bench.

## Open Questions

- **What happens to a blob the image cannot read?** A failed read or an
  unknown version (§1) leaves the legacy pair as the only network the image
  can use. Options: rerun the migration and write the blob, which restores a
  working list but replaces it with the legacy entry and drops every network
  added since with `wifi add`, Improv or QR join; or run from the legacy entry
  in RAM and leave the blob untouched, which keeps the newer list for when a
  newer image returns but leaves the older image unable to save a network
  without overwriting it.
- **What prompts for help when nobody is heard?** The §5 help request needs
  the VAD to hear someone, so a lone owner with the VAD off, or a quiet room,
  gets only the buzzer and amber LED. QR mode still starts by itself in
  `NO_KNOWN_NETWORK`, but the 128x32 OLED has no code to show what to do.
  Options: keep it VAD-gated, which never plays to an empty room but may never
  reach the owner; play a short clip on a timer, not VAD-gated, which reaches a
  lone owner but repeats to an empty room; or rely on the owner card (PRD-015
  FR-035) plus an OLED prompt, which adds OLED code to `robocar-unified`.

## Related

- [ADR-019](ADR-019-robocar-voice-gemini-tts.md): named pre-baked clips as
  "the better choice if offline operation later becomes a requirement"
- [ADR-022](ADR-022-planner-dormancy-and-spend-ceiling.md): the per-boot fuses
  that offline attempts currently spend
- [ADR-024](ADR-024-conversational-entity-and-multimodal-voice.md): the
  conversational entity this keeps talking
- [PRD-015](../requirements/PRD-015-teuvo-for-other-owners.md): every unit
  built for someone else depends on §1–§5
- `.claude/skills/wifi-sta-setup/SKILL.md`: canonical STA pattern, to update
