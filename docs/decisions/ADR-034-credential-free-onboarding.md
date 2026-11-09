# ADR-034: Credential-Free Onboarding — Device-Flow Enrolment, BLE Improv and Broker Discovery

**Status**: proposed
**Date**: 2026-11-09
**Confidence**: 4/10 (the Google and Improv facts below are read from the
current docs; nothing here is built, the proxy hop's latency is unmeasured, and
NimBLE's internal-RAM cost on this firmware is unmeasured)
**Related**: [ADR-028](ADR-028-teuvo-connectivity-resilience.md) (several
networks, MQTT only on `home` networks, BLE deferred),
[PRD-015](../requirements/PRD-015-teuvo-for-other-owners.md) FR-034 (Gemini
proxy) and FR-039 (`gemini auth` NVS slot),
[ADR-033](ADR-033-robocar-error-reporting.md) (GlitchTip over MQTT)

---

## Context

Bringing Teuvo up on a fresh image means handling three secrets by hand, and
one address that cannot be set at all:

| Value | Where it comes from today | Settable without a reflash |
|---|---|---|
| WiFi SSID and password | NVS (Improv Serial over USB), else `credentials.h` | Yes, Improv over USB |
| Gemini API key | NVS (`gemini auth <key>` on the serial console), else `credentials.h`, else the environment | Yes, serial console |
| MQTT user and password | NVS (`mqtt auth`), else `credentials.h` | Yes, serial console |
| MQTT broker address | `MQTT_BROKER_URI`: `credentials.h`, else `mqtt://192.168.0.100:1883` in `config.h` | **No** |

Released images (CI, web flasher, OTA) are built with an empty `credentials.h`
stub. Flashing release 0.2.18 onto Teuvo on 2026-11-09 left him without WiFi
(nothing in NVS: local builds had always supplied it) and pointed at
`192.168.0.100`, while the broker is at `.106`. With WiFi provisioned, that
image still could not reach MQTT, so no GlitchTip events and no OTA nudge. An
OTA update installs the same image, so it would do the same.

The owner wants:

1. No credential typed or pasted by hand.
2. A demo on an Android phone's hotspot, with no broker on that network.
3. Several saved networks (ADR-028 §1 already decides this).
4. WiFi provisioning from the phone, over Bluetooth.
5. The OLED to say when Teuvo is waiting for a network to be configured.

Facts the design rests on, read 2026-11-09:

- **Google's device flow cannot authorise Gemini.** "OAuth 2.0 for TV and
  Limited-Input Device Applications" is "supported only for" `email`,
  `openid`, `profile`, `drive.appdata`, `drive.file`, `youtube` and
  `youtube.readonly`
  ([developers.google.com/identity/protocols/oauth2/limited-input-device](https://developers.google.com/identity/protocols/oauth2/limited-input-device)).
  No `generative-language` or `cloud-platform` scope is on the list.
- **Gemini's OAuth path is a desktop-app client**, described as "appropriate
  for a testing environment"; API keys are the default
  ([aistudio.google.com/docs/oauth](https://aistudio.google.com/docs/oauth)).
- **Gemini ephemeral tokens work "only with Live API"**, and minting one takes
  an API key on a backend
  ([aistudio.google.com/docs/live-api/ephemeral-tokens](https://aistudio.google.com/docs/live-api/ephemeral-tokens)).
  The planner uses Robotics-ER `generateContent`, so they do not apply.
- **Improv carries WiFi and little else.** The serial spec defines RPCs 0x01
  (WiFi settings), 0x02 (current state), 0x03 (device info), 0x04 (scan),
  0x05 (hostname), 0x06 (device name) and 0x07 (network state), with no vendor
  command range; its home page says "It is not the goal to offer a way for
  devices to share data or control"
  ([improv-wifi.com/serial](https://www.improv-wifi.com/serial/)). After
  provisioning the device may return a redirect URL. The shared
  `improv-wifi` component implements 0x01–0x03 only, and treats 0x02 as
  *Identify* (ADR-028 Context).
- **The build has no BLE**, and internal RAM was once driven to 259 bytes
  before mbedTLS moved to PSRAM (ADR-028 Context).
- **The OLED's network row** reads `IP <address>` or `WIFI DOWN`
  (`oled_status_core.h`), 21 columns.
- **MQTT sends its credential in plaintext**, so ADR-028 §6 runs MQTT only on
  networks whose entry carries the `home` flag.

## Decision

### 1. The robot enrols with the PRD-015 proxy by device flow (RFC 8628)

The proxy in PRD-015 FR-034 is its own authorisation server; Google is used
only to sign the owner in on the phone.

1. A robot with no device token asks the proxy's device endpoint for a code,
   sending its device id (PRD-015 FR-033).
2. The proxy returns a `user_code` (8 characters, no ambiguous glyphs), a
   `verification_uri` and a polling interval. The robot shows the code on the
   OLED (§4) and says it with an offline clip (ADR-028 §5).
3. The owner opens the URI on a phone and signs in with Google. This is an
   ordinary browser sign-in for `openid email`, not Google's device flow, so
   the scope restriction above does not apply. The page asks the owner to
   confirm the code and names the robot.
4. The robot polls the proxy's token endpoint and receives a per-device token.
   It stores it in the FR-039 NVS slot, where `gemini auth` keeps a key today.
5. Gemini requests go to the proxy, which adds the project key. FR-034's
   per-device cost cap, revocation and kill switch apply.

The Gemini key then exists only in the proxy's secret store. Revoking a robot
is a row in the proxy, not a key rotation.

**Latency gate.** The proxy adds a network hop to every planner, voice-turn and
TTS request, and request latency is already poor and unmeasured. Before the
proxy carries the planner:

- measure the current per-request breakdown on the robot (DNS, TLS handshake,
  JPEG upload, time to first byte, total), on home WiFi and on a hotspot;
- measure the same through a proxy deployment close to the Gemini endpoint;
- the proxy holds one persistent TLS connection to Google, and the robot keeps
  its connection to the proxy alive between requests. That can remove a
  handshake per request the robot pays today, which may offset the hop; the
  measurement decides.

TTS streams over SSE (`:streamGenerateContent?alt=sse`); the proxy forwards it
unbuffered.

Until the proxy exists (PRD-015 builds it beyond about five robots), `gemini
auth` stays the route, as PRD-015 route B describes.

### 2. WiFi provisioning over Improv BLE, beside Improv Serial

- Add Improv's BLE transport to the shared component, so a phone can provision
  from Chrome on Android (Web Bluetooth) with no cable. Improv Serial stays for
  the USB path and the web flasher.
- Provisioned networks land in ADR-028's network list with source `improv`
  and without the `home` flag.
- Fix the component's RPC 0x02 (current state) and error code 0x03 first
  (ADR-028 §2); BLE clients rely on the state machine the spec defines.
- **Gate:** measure NimBLE's internal-RAM cost with camera, TLS, audio and the
  planner running before committing. ADR-028 §8 deferred BLE for exactly this.
  If it does not fit while running, BLE runs only while provisioning is armed
  (no stored network, or link down past `NET_IMPROV_REARM_MS`) and is torn
  down before the planner's first request.
- The Improv BLE spec has not been read for this ADR; read it before the
  component work, including its authorisation step.

### 3. The broker is discovered, and only on a home network

- On link-up with a `home` entry, look up `_mqtt._tcp` over mDNS (the firmware
  already runs the `espressif/mdns` component) and connect to what answers.
  Fall back to an NVS-stored URI (`mqtt broker <uri>`, serial-only like
  `mqtt auth`), then to `MQTT_BROKER_URI`.
- On a network without the `home` flag, discovery does not run, and MQTT stays
  off as ADR-028 §6 decides. Any device on a network can advertise
  `_mqtt._tcp`, so a discovered broker on a hotspot would receive the
  plaintext credential.
- With no broker found, the robot runs as it does today with MQTT down: the
  planner, voice and reflexes work; remote commands and GlitchTip do not.
- GlitchTip events raised while MQTT is off are kept (the newest few, in
  PSRAM) and sent at the next home link-up. Once the proxy exists it can also
  accept them over HTTPS, so a hotspot demo still reports.

### 4. The OLED states what the network is waiting for

Row 0 (`oled_status_core.c`) shows ADR-028 §4's connectivity state instead of
the single `WIFI DOWN`:

| State | Row 0 |
|---|---|
| No stored network, Improv armed | `WIFI SETUP: IMPROV` |
| Stored networks, none visible | `WIFI SEARCHING` |
| Credential rejected | `WIFI AUTH FAILED` |
| Enrolment pending (§1) | `CODE ABCD-EFGH` |
| Connected | `IP <address>` (unchanged) |

The pure formatter keeps its host test; each row stays within 21 columns.

## Alternatives Considered

| Alternative | Why not |
|---|---|
| Google's device flow straight to Gemini | `generative-language` is not an allowed scope for it |
| Gemini ephemeral tokens on the robot | Live API only, so not Robotics-ER `generateContent`; minting needs a backend with the key anyway |
| Carry the broker address or Gemini key in Improv fields (device name, hostname) | Clients show those fields as the robot's name; the spec has no vendor range |
| A setup web page on the robot, reached by the Improv redirect URL | An HTTP server and form on a tight heap, and the owner still types the key |
| Broker discovery on every network | Sends the plaintext MQTT credential to whoever advertises `_mqtt._tcp` |
| `network_provisioning` (Security2, PoP sticker) instead of Improv BLE | ADR-028 §8's candidate. It verifies the device to the phone, which Improv BLE may not; kept as the fallback if the Improv BLE spec's authorisation proves too weak |

## Consequences

**Positive**
- A robot can be brought up with no secret typed: WiFi from the phone over
  Bluetooth, Gemini by approving a code.
- The Gemini key leaves the robot, and each robot is revocable on its own.
- A hotspot demo needs no broker and leaks no MQTT credential.
- A release image, and therefore OTA, reaches the home broker without a
  compile-time address.

**Negative**
- A cloud service to run, with its own uptime, cost and secret store; Teuvo
  cannot plan or talk while it is down.
- One more hop on every Gemini request, unmeasured.
- NimBLE's RAM, on a heap that has run out before.
- Enrolment ties each robot to an owner's Google sign-in on the proxy, which
  PRD-015's GDPR reading (controller for household audio and images) already
  covers.

## Open Questions

- Whether a QR code for `verification_uri_complete` is scannable on the
  0.91-inch 128x32 OLED, or the code stays text and speech only.
- Where the proxy runs (Cloudflare Workers, Cloud Run) and which region sits
  closest to the Gemini endpoint.
- Whether events buffered while MQTT is off should survive a reset (RTC memory
  is already used by ADR-033's breadcrumb ring).
- Whether Improv BLE's authorisation step is enough, or `network_provisioning`
  with a PoP is needed (Alternatives).
