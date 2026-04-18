---
name: wifi-sta-setup
description: Apply or audit the canonical ESP-IDF WiFi STA configuration for a monorepo project
argument-hint: "audit|apply <project-path>"
user-invocable: true
allowed-tools: Read, Write, Edit, Grep, Glob
---

## Task

Bring an ESP-IDF project's WiFi STA setup into line with the monorepo canonical
pattern, or audit it without modifying anything.

Invocation:

- `audit <project-path>` — read-only drift report against canonical.
- `apply <project-path>` — migrate source + sdkconfig to canonical.

`<project-path>` is relative to repo root, typically
`packages/<domain>/<project-name>`.

The canonical source of truth is
`packages/robocar/unified/main/wifi_manager.c` (identical to
`packages/robocar/camera/main/wifi_manager.c`). That project
runs reliably on XIAO ESP32-S3 Sense in the same environment as every other
WiFi project in this monorepo, so its configuration is the reference.

## Canonical STA init code

Drop into `main/wifi_manager.{c,h}` for new projects, or inline into `main.c`
for small projects where a separate module isn't justified. The function
boundary doesn't matter — these **settings** do:

```c
// ---- event handler (reason-code logging is the key bit) ----
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                          void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected. Reason: %d (%s)", disconnected->reason,
                 disconnected->reason == WIFI_REASON_NO_AP_FOUND         ? "AP not found"
                 : disconnected->reason == WIFI_REASON_AUTH_FAIL         ? "Auth failed"
                 : disconnected->reason == WIFI_REASON_ASSOC_FAIL        ? "Assoc failed"
                 : disconnected->reason == WIFI_REASON_HANDSHAKE_TIMEOUT ? "Handshake timeout"
                                                                         : "Other");
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry %d/%d to connect to the AP", s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to AP after %d retries", WIFI_MAXIMUM_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ---- init ----
esp_netif_init();
esp_event_loop_create_default();
esp_netif_create_default_wifi_sta();

wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
esp_wifi_init(&cfg);

// Regulatory: allow channels 1–13 (default US policy clips to 1–11)
wifi_country_t country = {
    .cc = "FI", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_AUTO};
esp_wifi_set_country(&country);

esp_wifi_set_ps(WIFI_PS_NONE);

esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &any_id);
esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &got_ip);

// ---- connect ----
wifi_config_t wifi_config = {
    .sta = {
        .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,        // mixed-mode tolerant
        .pmf_cfg = {.capable = true, .required = false},     // 4-way handshake tolerance
        .scan_method = WIFI_FAST_SCAN,
        .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
    },
};
strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
esp_wifi_set_mode(WIFI_MODE_STA);
esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
esp_wifi_start();
```

Retry cap: `#define WIFI_MAXIMUM_RETRY 5`. Higher values just lengthen the
failure path — they don't recover from a wrong SSID or password.

## Canonical `sdkconfig.defaults` WiFi block

```
CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=10
CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=32
CONFIG_ESP_WIFI_TX_BUFFER_TYPE=1
CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM=32
CONFIG_ESP_WIFI_AMPDU_TX_ENABLED=y
CONFIG_ESP_WIFI_AMPDU_RX_ENABLED=y
CONFIG_ESP_WIFI_AMSDU_TX_ENABLED=y
CONFIG_ESP_WIFI_NVS_ENABLED=y
```

**Anti-pattern to remove:** `CONFIG_ESP_PHY_MAX_WIFI_TX_POWER=<N>` (below
default). Reducing TX power only loses link margin. It's sometimes justified
as a brownout mitigation, but on a board with `CONFIG_ESP_BROWNOUT_DET=n` the
brownout justification is moot — if brownout is disabled, you aren't going to
get a brownout-triggered reset regardless of TX power. Only keep this setting
if `CONFIG_ESP_BROWNOUT_DET=y` AND you've measured USB supply dips correlated
with TX bursts.

## Required `CMakeLists.txt` REQUIRES

```cmake
REQUIRES esp_wifi nvs_flash esp_netif esp_event
```

## Credentials contract

`credentials.h` must define:

```c
#define WIFI_SSID "..."
#define WIFI_PASSWORD "..."
```

See sibling skill `credential-setup` for standardized credential handling
(`.example` template, gitignore, pre-commit protection).

## Disconnect reason → diagnosis

When the canonical event handler is in place and connection still fails, the
log line `WiFi disconnected. Reason: <N> (<name>)` tells you what to fix:

| Reason | Likely cause |
|---|---|
| `WIFI_REASON_NO_AP_FOUND` | SSID wrong, out of range, 5 GHz-only AP (ESP32/S3 is 2.4 GHz), country code blocks channel (fixed by canonical `FI`/`1-13`) |
| `WIFI_REASON_AUTH_FAIL` | Password wrong, or `threshold.authmode` stricter than AP advertises (canonical `WPA_WPA2_PSK` accepts both) |
| `WIFI_REASON_ASSOC_FAIL` | AP rejected (MAC filter, client cap) |
| `WIFI_REASON_HANDSHAKE_TIMEOUT` / `WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT` | PMF mismatch, TX power too low, brownout during handshake |
| `WIFI_REASON_BEACON_TIMEOUT` | Link lost after connect (roaming, interference) |

For hidden SSIDs, use `.scan_method = WIFI_ALL_CHANNEL_SCAN` and
`.bssid_set = 0` (known override; document in the project).

## Audit process (`audit <project-path>`)

1. Read `main/main.c` and `main/wifi_manager.c` if present.
2. Read `sdkconfig.defaults`.
3. Read `main/credentials.h.example` (for contract check).
4. Compare against canonical; produce a drift table with `file:line`
   references.
5. Print the canonical snippets needed to converge.
6. Do **not** modify any files.

Drift check (compare against canonical values):

| Check | Canonical |
|---|---|
| `threshold.authmode` | `WIFI_AUTH_WPA_WPA2_PSK` |
| `pmf_cfg` | `{.capable = true, .required = false}` |
| `scan_method` | `WIFI_FAST_SCAN` (unless hidden SSID) |
| `sort_method` | `WIFI_CONNECT_AP_BY_SIGNAL` |
| `esp_wifi_set_country()` | Called with `FI`/`1`/`13`/`POLICY_AUTO` |
| `esp_wifi_set_ps(WIFI_PS_NONE)` | Called |
| Disconnect reason logged | Yes, with name mapping |
| Retry cap | `WIFI_MAXIMUM_RETRY = 5` (higher is drift) |
| `CONFIG_ESP_PHY_MAX_WIFI_TX_POWER` | Not set (unless justified by brownout+measurement) |
| `CONFIG_ESP_WIFI_AMPDU_*` / `AMSDU_TX_ENABLED` | `y` (ESP-IDF default; materially improves STA throughput) |
| SoftAP-on-S3 exception | If the project runs `WIFI_MODE_AP` on ESP32-S3, keep AMPDU off — see espressif/esp-idf#13508 (clients can't see the AP with AMPDU on) |
| `CMakeLists.txt REQUIRES` | Includes `esp_wifi nvs_flash esp_netif esp_event` |

## Apply process (`apply <project-path>`)

1. Detect structure: separate `wifi_manager.{c,h}` or inline in `main.c`?
   - If WiFi code is already self-contained in `main.c` (one handler + one
     init function), keep it inline and edit in place.
   - If WiFi code is spread across `main.c` OR the project is large enough
     that extracting clarifies it, create `main/wifi_manager.{c,h}` with the
     canonical shape (see `robocar-unified/main/wifi_manager.{c,h}` for exact
     file contents). Update `main/CMakeLists.txt` SRCS accordingly and
     replace inline WiFi code in `main.c` with `wifi_init()` +
     `wifi_connect(WIFI_SSID, WIFI_PASSWORD)` calls.
2. Apply targeted edits to reach canonical:
   - `threshold.authmode` → `WIFI_AUTH_WPA_WPA2_PSK`
   - Insert `esp_wifi_set_country()` call right after `esp_wifi_init()` (and
     before the config path — country affects scan).
   - `scan_method` → `WIFI_FAST_SCAN` (unless hidden SSID — verify with user).
   - Add reason-code logging in the `STA_DISCONNECTED` branch.
   - Ensure `esp_wifi_set_ps(WIFI_PS_NONE)` is present.
3. Update `sdkconfig.defaults`:
   - Add any missing entries from the canonical WiFi block.
   - Remove `CONFIG_ESP_PHY_MAX_WIFI_TX_POWER=N` unless brownout is enabled
     and the user confirms a measurement.
4. Ensure `main/CMakeLists.txt` REQUIRES includes the four components.
5. Delete the generated `sdkconfig` (ESP-IDF preserves existing values in
   `sdkconfig` and silently ignores new defaults — see
   `.claude/rules/esp-idf-sdkconfig.md`).
6. Report each file modified with before/after snippets, and cross-reference:
   - `.claude/skills/sdkconfig-audit` — further sdkconfig drift review
   - `.claude/skills/credential-setup` — `credentials.h` contract
   - `.claude/rules/mdns-hostname.md` — every STA project should also
     advertise an mDNS hostname

## Output format

Bullet list of changes per file (use `file:line` form), with the delta
between before/after for each. End with a "Next steps" block:

```
- Build: just <project>::clean && just <project>::build
- Flash + monitor: just <project>::flash-monitor
- On failure: the log will now show `WiFi disconnected. Reason: <N> (<name>)` —
  cross-reference the diagnosis table in this skill.
```
