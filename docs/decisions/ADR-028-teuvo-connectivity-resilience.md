# ADR-028: Teuvo Connectivity — Several Networks, Services Armed on Link-Up, QR Join and an Offline Voice

**Status**: proposed
**Date**: 2026-10-23
**Source**: conversation 2026-10-23 (four repo-reading and five web research passes, each fact-checked)
**Confidence**: 7/10 (QR read range and decode time on the OV3660 are unmeasured)
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
- The 128x32 OLED is wired but has no driver (`config.h:158`, issue #780).

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
  SAE-only), BSSID and channel hint, last RSSI, last-success boot counter,
  failure count, and source (improv, console, qr). First boot migrates the
  legacy `wifi_config` pair.
- Selection runs an all-channel scan, intersects it with the list, and ranks by
  RSSI plus recency minus a failure penalty. It then connects with a BSSID and
  channel hint. Reasons 15/202/204 mark a credential as suspect; 201/210/211
  mean the network is not here.
- `esp_wifi_set_storage(WIFI_STORAGE_RAM)` stops the driver writing its own
  copy to flash on every switch. The app list is the one store.
- The WPA2 minimum stays. An open network is joined only if its entry carries
  the owner's explicit open-allowed flag.
- The serial console gets `wifi list|add|forget|scan`. It is serial-only, like
  `mqtt auth`, so a password never travels over MQTT.
- A phone hotspot is an ordinary entry. The docs say it must be 2.4 GHz.
- The 32-character SSID and 64-hex-digit PSK edge cases are fixed: the buffers
  become 33/65 (`credentials_loader.h:17-18`, `wifi_manager.c:192-193`).

### 2. Arm network services on link-up, not at boot

- MQTT and OTA, including the rollback-confirm timer, start on the first
  `IP_EVENT_STA_GOT_IP`, whenever that happens.
- Improv re-arms when the link has been down for more than a set time, and
  disarms on reconnect.
- The planner, voice turns and TTS skip their request while there is no link,
  so offline time does not spend the per-boot fuses.
- `improv-wifi` gets 0x02 *current state*, error code 0x03 and real device info.
  Scan (0x04) follows later. All four consumers build, because CI does not
  build consumers on a component-only change (#670).

### 3. Hold the Gemini credential in NVS

A serial-only `gemini auth <key>` / `gemini auth clear` command stores the key
in its own NVS namespace. It is modelled on `mqtt auth`, with the same
redaction and presence-only reporting. The key loads from NVS, then
`credentials.h`, whatever the WiFi source. The same slot later holds a proxy
device token (PRD-015 FR-034). `gemini_http.c` handles HTTP 402 (prepay balance
exhausted) and 403 (key blocked or unrestricted) separately from other
failures, so Teuvo can say which one it is.

### 4. Make connectivity a stated fact

`wifi_manager` exposes one state: `CONNECTED`, `SEARCHING`, `NO_KNOWN_NETWORK`,
`AUTH_FAILED`, `NO_KEY` or `NO_CREDIT`, plus the connected SSID and RSSI.
`self_report` reports the SSID it is actually connected to, not the cached
credential. `activity_trace` gets a distinct no-link LED state, which today
reads the same as an API error.

### 5. Give Teuvo a small offline voice

- 8–12 persona lines are pre-rendered through the same Gemini TTS voice and
  style by a local `just` recipe. Examples: "I seem to have lost the network",
  "show me the WiFi QR code", "connected, thank you", "my key is not working".
  They are stored as 24 kHz IMA-ADPCM, at about 12 kB/s, so 40–60 s is about
  0.5–0.7 MB.
- The clips are **embedded in the app image** (`EMBED_FILES`), not written to
  the `spiffs` partition. OTA updates only the app partition, and there is room
  for them (1.34 of 3.5 MB used). Clips in `spiffs` would never update.
- Lines are name-free, so a renamed Teuvo (PRD-015 FR-032) can reuse them.
  Finnish comes first; English is an open question.
- Playback goes through `audio_player`, like live TTS. A help request plays
  when the state is `NO_KNOWN_NETWORK` or `AUTH_FAILED`. It needs the VAD to
  hear someone nearby, is rate-limited, and comes with a buzzer motif and an
  amber LED.

### 6. Join from a WiFi QR code shown to the camera

- Decoding uses `espressif/quirc` 1.2.0 (ISC). Its input is a full-scale luma
  plane taken from the JPEG frames the camera already produces. The camera is
  never reconfigured to GRAYSCALE, which would need a deinit and init and would
  disturb the planner.
- *QR mode* raises the frame size to VGA, inside the 64 KB JPEG buffer, which
  the bench must confirm. It is entered from the console, or by itself in
  `NO_KNOWN_NETWORK`.
- A pure-C `wifi_qr_core` parser accepts both dialects:
  - ZXing/Android: backslash escapes, `T:SAE` from Android;
  - the WPA3 spec v3.5 §7.1: percent-encoding, `R:`, `I:`, `K:`, `T` absent.
- Policy:
  - QR codes are accepted only while no known network is reachable, or after a
    console or voice confirmation, because anyone holding a QR could move the
    robot to their AP;
  - WEP is refused;
  - an open network is refused unless the owner allowlisted it;
  - each scanned credential gets at most 2 attempts;
  - an entry is saved only after `GOT_IP`, as Improv does today.

### 7. The WiFi hunt is opt-in play, built last

The hunt runs only when the owner has enabled it and the link has been down for
more than a set time. It has three moves:

- a head sweep in QR mode, at table and human height;
- a hill-climb on a **known** network's RSSI, heavily averaged;
- asking people for help with the offline clips.

Indoor RSSI noise is about 4 dB, and a ±1 dB local mean needs about 5 m of
travel at 2.4 GHz (Blum & Hafner 2015; Lee 1985). So the hunt is a game and a
way back toward a remembered AP, not a router finder.

Limits:
- no open networks unless allowlisted;
- no guessing or brute force; no WPS PIN; no MAC cloning; no captive-portal
  automation;
- at most 2 attempts per credential;
- time- and distance-boxed, and on a supervised single-level floor only, since
  there is no cliff sensor.

Reading a handwritten password needs Gemini, because no OCR runs on the
ESP32-S3 (PP-OCRv6 is ESP32-P4 only). So it works only once online, for example
through a phone hotspot. It must be started by the owner, use a paid-tier key
(the unpaid tier's terms forbid sending sensitive data), and have the network
owner's consent.

### 8. Deferred

- **BLE provisioning** (`network_provisioning` Security2 with a per-robot PoP
  sticker). It is the only phone path that also works on iOS. Revisit it for
  PRD-015 after measuring NimBLE's internal-RAM cost.
- **SoftAP captive portal.** It needs AMPDU off on the S3, which costs STA
  throughput (esp-idf#13508).
- **A built-in SIM modem.** Only D3 and D7 are free, so there is no RTS/CTS.
  The carrier and the 2x7 ribbon do not route those pads. The 5 V LM2596 rail
  is already at its limit with two motors stalled. Real-time 24 kHz PCM TTS
  needs about 0.6 Mbit/s (estimate), which rules out LTE-M and 0.5 Mbit/s
  Finnish device plans.
  - Until then, a phone hotspot or a pocket LTE router is the untethered path,
    as a list entry.
  - If a modem ever comes: an A7670E-class Cat-1 module, LTE-only
    (`AT+CNMP=38`), at 921600 baud or more, with its own 3.8 V 2 A buck from
    VBAT, through a carrier revision.
  - Re-evaluate if TTS moves to the 3.8 TTS models' 8 kHz µ-law output on the
    Interactions API, which needs roughly a sixth of the downlink (Gemini
    speech-generation docs, 2026-10-12).

## Alternatives Considered

| Alternative | Why not |
|---|---|
| Third-party multi-AP component (`thorrak/esp_wifi_config`, `embedblocks/wifi-manager`) | Single-maintainer, pre-1.0, brings SoftAP/REST/BLE stacks that overlap Improv; one needs IDF ≥5.5 |
| Switch the camera to GRAYSCALE for QR | Needs `esp_camera_reconfigure()` (deinit + init), which disrupts the planner's JPEG stream |
| Offline clips in the `spiffs` partition | Not covered by OTA; clips would freeze at the first flash |
| On-device OCR for written passwords | No OCR model runs on the ESP32-S3 |
| Joining any open network when lost | Unsafe and possibly against the network owner's terms; kept behind an explicit allowlist |
| SoftAP or BLE now | AMPDU conflict; unmeasured RAM on an already tight heap |
| SIM modem now | Pins, ribbon, power budget and TTS bandwidth all need a hardware revision; hotspot covers the use case |

## Consequences

**Positive**
- Teuvo survives a move between home, a phone hotspot and a second site
  without a reflash.
- A robot that boots offline, or loses its link mid-uptime, recovers MQTT, OTA
  and provisioning on its own. An OTA image is no longer rolled back because
  its first boot was offline.
- Teuvo can say he is lost and what would help, in character, with no network.
- The key survives Improv and OTA. Every PRD-015 unit needs this.
- Selection, the QR parser and the store are pure C and pinned by host tests.

**Negative**
- `wifi_manager.c` stops matching the canonical pattern in the `wifi-sta-setup`
  skill. The skill is updated in the same change.
- An all-channel scan adds about 1.5 s to each connect attempt (estimate).
- Committed clip assets go stale when the voice or style changes, and they are
  re-rendered by hand.
- A shown QR code is a way to move the robot onto another network. TLS with
  the certificate bundle still protects Gemini traffic, but MQTT is plaintext
  (`config.h:82`). The QR policy above is the mitigation.
- Secrets in NVS stay plaintext until NVS encryption is enabled. That is
  irreversible eFuse work, decided under PRD-015.

## Implementation

In order. Each step lands with host tests where the logic is pure C.

1. FR-039: runtime Gemini credential (§3), which fixes the key-drop bug. Issue
   #TBD-key.
2. FR-038: services armed on link-up, and requests skipped offline (§2).
   Issue #TBD-arm.
3. FR-037: the network list, all-channel scan and console commands (§1, §4).
   Issue #TBD-netlist.
4. FR-040: offline clips and the help request (§5). Issue #TBD-offline.
5. FR-041: QR join (§6). Bench first: quirc decode time and read distance at
   QVGA and VGA on the OV3660. Issue #TBD-qr.
6. FR-042: the hunt (§7), after the five above have run on the bench.

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
