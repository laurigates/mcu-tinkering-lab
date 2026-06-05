# NotebookLM suggested sources — 2026-06-05

Curated from the LakuVault NotebookLM suggestion queue
(`Inbox/NBLM/Suggestions/mcu-tinkering-lab/`, 22 files / 539 unchecked URLs).
The vault accumulates URLs in each anchor's scope and queues them here rather
than pushing silently, because these notebooks are `notebook_owner:
mcu-tinkering-lab` (see [`docs/nblm-integration.md` §Ownership](https://github.com/laurigates/LakuVault)
in the vault). This file is the curator's draft for a human to review before
adding the kept URLs to the matching notebooks via `notebooklm source add
--notebook <id> <url>`.

Notebook IDs and titles below match
[`docs/reference/notebook-mapping.md`](reference/notebook-mapping.md). Apply
respecting the ~30-source cap per notebook (drop a stale source if adding
pushes a notebook over).

## Triage rules applied

The vault's per-anchor scope (anchor note + backlinks + shared dominant tag)
leaks the same URLs into many files. Dropped across **all** notebooks:

- **The repo itself** — `github.com/laurigates/mcu-tinkering-lab` (not a source).
- **Generic hardware-news churn** attached indiscriminately to every anchor
  via a shared tag: the three `cnx-software.com/2025/11/...` posts (Heltec
  V4, Arduino Nesso N1, EBYTE ECM50), `hackaday.com/.../tiny386-on-an-...esp32-s3`,
  the Hackster *XIAO Debug Mate* news post, and the `esp32-marauder` post.
  These are news, not durable reference material, and the same six appear in
  ~20 files. Add manually to a single relevant notebook only if any is worth
  keeping.
- **Intranet / LAN-local URLs** — `192.168.*`, `192.168.4.1`,
  `popos.intra.lakuz.com`, `ha.intra.lakuz.com`, `http://shelly/ota?...` —
  dead outside the LAN.
- **Screenshot image assets** — the `savjee.be/uploads/.../*.png` series and
  other `*.png`/`*.jpeg` — not citable source documents.
- **Truncated / malformed duplicates** — Reddit/YouTube URLs cut at a path
  segment (e.g. `.../comments/10pqdy8/looking`, `watch?v=c`) that duplicate a
  complete URL already listed.

Cross-board contamination is also collapsed: the XIAO C3/C5/C6/P4/RP2350
blocks are duplicated across every board anchor; each URL is kept only under
the board it actually documents.

---

## ESP32-S3 hardware families — `3bb67ba2-58b4-4485-a306-6ab49ec38160`

Anchor: `ESP32-S3 Hardware Families.md` (45 raw → 5 kept). XIAO C3/C5/C6/P4
links removed (belong to their own notebooks).

- https://www.waveshare.com/wiki/ESP32-S3-DEV-KIT-N8R8
- https://www.cnx-software.com/2025/02/28/esp32-s3-infrared-thermal-imaging-camera-module-offers-80x62-resolution-45-and-90-wide-angle-versions/
- https://www.hackster.io/news/running-linux-on-an-esp32-s3-0d96c5a24081

(AliExpress shortlinks `a.aliexpress.com/_*` dropped — opaque redirects with
no durable content.)

## ESP32 classic boards (CAM, Heltec, TTGO) — `d2d63711-4843-48e7-9965-018362dad0e5`

Anchor: `ESP32 Classic Boards.md` (24 raw → 0 kept). All topical URLs in this
file are actually XIAO C3 or ESP32-P4 links (wrong board class); they route to
the C3 / P4 notebooks below. Nothing classic-board-specific remains after
removing leakage. **No additions.**

## Sensors & modules — `c2d44a8c-cba1-436f-9c5c-0fe98f672909`

Anchor: `Sensors and Modules.md` (7 raw → 0 kept). Only universal-noise URLs
present. **No additions.**

## Motor drivers & power electronics — `197ef318-9983-4168-9a3b-d92288d832b6`

Anchor: `Motor Drivers and Power Electronics.md` (7 raw → 0 kept). Only
universal-noise URLs present. **No additions.**

## I2S audio & synthesis on ESP32 — `8cbdd8bb-e88e-4a26-a94f-d0126e68a09e`

Anchor: `I2S Audio on ESP32.md` (9 raw → 2 kept).

- https://esphome.io/components/i2s_audio/
- https://esphome.io/components/wireguard/

## USB gadget & HID on ESP32-S3 — `e91ebe8d-4b93-4321-9d7e-b97aa34956aa`

Anchor: `USB HID on ESP32-S3.md` (7 raw → 0 kept). Only universal-noise URLs
present. **No additions.**

## ChameleonUltra RFID/NFC — `4831062f-84ab-4f67-89c0-61345941e3ea`

Anchor: `ChameleonUltra RFID NFC.md` (7 raw → 0 kept). Only universal-noise
URLs present. **No additions.**

## ESP-IDF v5.4+ core reference — `f5af16f1-d307-4cc6-b7ab-39b7dbff0381`

Anchor: `ESP-IDF.md` (16 raw → 9 kept). Strong topical set — host-side unit
testing, QEMU, FreeRTOS simulator (relevant to the repo's shared-core
host-test pattern).

- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/host-apps.html
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#introduction
- https://github.com/espressif/qemu
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/qemu.html
- https://www.throwtheswitch.org/cmock
- https://www.freertos.org/FreeRTOS-simulator-for-Linux.html
- https://www.freertos.org/
- https://gist.github.com/MortezaRamezani/3f971889359a9d661c0e12ba297f7b1e

## ESP32 peripherals: I2C, UART, I2S, GPIO — `5601a768-acdb-4aeb-a2f0-455b3e493b1b`

Anchor: `ESP32 Peripherals.md` (8 raw → 1 kept).

- https://www.freertos.org/

## ESPHome platform & components — `2cb7153a-c51f-43c5-9a95-25261be28aec`

Anchor: `ESPHome.md` (57 raw → 16 kept). Dropped: all savjee screenshot PNGs,
LAN URLs, the OTA-firmware binary links, the universal news set. Kept the
Shelly-flashing walkthrough prose, ESPHome component docs, and the Mongoose-OS
→ Tasmota migration references.

- https://www.esphome-devices.com/devices/Shelly-RGBW2
- https://community.home-assistant.io/t/is-shelly-rgbw2-supported/133761/3
- https://github.com/yaourdt/mgos-to-tasmota
- https://koen.vervloesem.eu/blog/flashing-a-shelly-rgbw2-with-crocodile-clips-and-cut-resistor-leads/
- https://tasmota.github.io/docs/devices/Shelly-RGBW2/
- https://savjee.be/2020/09/shelly-2.5-flash-esphome-over-the-air/
- https://bitekmindenhol.blog.hu/2019/03/30/shelly_rgbw2
- https://www.die-welt.net/2020/05/building-a-shelly-25-usb-to-ttl-adapter-cable/
- https://esphome-configs.io/devices/shelly-25/
- https://mongoose-os.com/
- https://esphome.io/components/sensor/ade7953.html
- https://esphome.io/components/switch/restart.html
- https://savjee.be/2020/11/shelly25-esphome-potential-fire-hazard-fix/
- https://esphome.io/components/wireguard/
- https://esphome.io/components/i2s_audio/
- https://www.analysir.com/blog/

## Bluepad32 & BLE HID input — `f696b00a-1c42-44ae-92de-1bb1fef30c1d`

Anchor: `Bluepad32 and BLE HID.md` (8 raw → 1 kept).

- https://github.com/ruuvi/docs/blob/master/communication/bluetooth-connection/nordic-uart-service-nus/README.md

## WiFi provisioning, mDNS & MQTT — `ada2b18e-6172-4fea-8176-36db2031e404`

Anchor: `WiFi.md` (9 raw → 1 kept). The Ruuvi NUS doc belongs to Bluepad32/BLE
(listed above); the Arch wireless page is host-side, kept here as the only
WiFi-topical item.

- https://wiki.archlinux.org/title/Network_configuration/Wireless

## OTA updates & browser-based flashing — `c6868def-e128-4367-b70d-0efeb289d052`

Anchor: `OTA Updates and Browser Flashing.md` (8 raw → 1 kept).

- https://components.espressif.com/components/espressif/esp_delta_ota/versions/1.1.2

## AI vision backends for embedded — `a755b5cd-22ec-4b5a-844e-ba7c1a258957`

Anchor: `AI Vision for Embedded.md` (7 raw → 0 kept). Only universal-noise URLs
present (the news posts are about boards, not vision backends). **No additions.**

## Claude Code workflow & plugins — `ae5b0f8b-91fa-4e9e-9871-ee85d8ce24f9`

Anchor: `Claude Code Configuration.md` (62 raw → 6 kept). This anchor is
heavily polluted by the vault's broad AI/ML tag — most URLs are ComfyUI,
Stable Diffusion, Flux, video-gen, and TTS links that belong to the **vault's**
AI/ML MOC, not this repo's Claude Code notebook. Kept only the genuinely
Claude-Code / agent-tooling items:

- https://www.anthropic.com/claude-code
- https://github.com/musistudio/claude-code-router
- https://github.com/ruvnet/claude-code-flow
- https://github.com/hesreallyhim/awesome-claude-code
- https://worksonmymachine.substack.com/p/mcp-an-accidentally-universal-plugin
- https://research.trychroma.com/context-rot

(The ComfyUI/SD/Flux/Wan/TTS/LangGraph bulk is off-topic for this repo and is
left for the vault's own AI/ML notebook.)

## Python simulation & robotics math — `28c00388-b713-4e0e-b7f2-591c31b60d90`

Anchor: `Python Simulation and Robotics Math.md` (7 raw → 0 kept). Only
universal-noise URLs present. **No additions.**

## The Modern Soldering Equipment and Techniques Guide — `4e584364-0c55-408b-a7db-a69c177af71c`

Anchor: `Soldering Guide and Shopping List.md` (115 raw → 28 kept — at cap).
This is the richest, most on-topic queue: hot-air station reviews, flux/solder
selection, fume extraction, desoldering, ultrasonic cleaning, Kapton tape, and
ESP32-module soldering. Dropped all `*.png`/`*.jpeg` assets, the truncated
duplicate URLs (lines cut at a path segment), and the PostmarketOS block (a
phone-OS tangent unrelated to soldering). Selected the 28 highest-value
durable references:

- https://eleshop.eu/atten-st-862d-hot-air-soldering-station.html
- https://store.rossmanngroup.com/atten-862.html
- https://eleshop.eu/review/atten-st-862d-hot-air-soldering-station
- https://www.toolify.ai/ai-news/honest-review-atten-st862d-after-a-year-of-use-345477
- https://electronics.stackexchange.com/questions/70735/should-i-use-lead-free-solder
- https://ipsystemsusa.com/all-about-solder-smoke-fume-extractors/
- https://fumedog.com/blogs/news/how-to-choose-the-right-fume-extractor-system
- https://www.eevblog.com/forum/reviews/fume-extractor-advice/
- https://hakkousa.com/products/fume-extraction.html
- https://www.chemtronics.com/essential-guide-to-flux-for-soldering-electronics
- https://www.kester.com/products/product/rf741-rework-flux
- https://www.eevblog.com/forum/projects/best-solder-flux/
- https://www.mouser.com/c/tools-supplies/soldering/soldering-flux/?product=Flux%20Syringes
- https://hyepeak.com/blog/lead-vs-lead-free-solder/
- https://hakkousa.com/lead-free/
- https://www.globalwellpcba.com/leaded-vs-lead-free-solder/
- https://www.instructables.com/Electronics-Workbench-Equipment-List/
- https://www.chemtronics.com/circuitworks-no-clean-tacky-flux
- https://www.gotopac.com/products/solder-rework/soldering-accessories/pcb-holders.html
- https://learn.sparkfun.com/tutorials/how-to-use-a-hot-air-rework-station/all
- https://www.adafruit.com/product/3197
- https://thepihut.com/products/stickvise-pcb-vise
- https://www.raypcb.com/desoldering-wick-vs-pump/
- https://forum.kicad.info/t/re-work-tools-desoldering-tools/40116
- https://www.digikey.com/en/maker/projects/how-to-solder-a-qfn-component-using-a-hot-air-rework-station/77b9331e7183452f97041c75d946c7c7
- https://zeph.com/lowmelt.htm
- https://chipcodelab.com/how-to-use-low-melt-alloy-solder-to-remove-ic-chip/
- https://kk.org/cooltools/kapton-tape/

---

## New notebooks not yet in the mapping

The vault has four board anchors with no matching notebook in
`docs/reference/notebook-mapping.md` (ESP32-C3, C5, C6, P4 each have a vault
anchor and a distinct `notebook_id`, but the mapping doc lists only the S3 and
classic-board notebooks). These IDs come from the suggestion-file frontmatter
and are **candidates to add to the mapping** if the repo wants to track them.
Their curated sources are below.

### ESP32-C3 (XIAO) — `4ca90889-755f-4918-afa9-25999fb07d05`

Anchor: `ESP32-C3.md` (39 raw → 8 kept; C5/C6/P4/RP2350 leakage removed).

- https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html
- https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/
- https://files.seeedstudio.com/wiki/XIAO_WiFi/Resources/Seeed_Studio_XIAO_ESP32C3_Power_Consumption_Tests.pdf
- https://wiki.seeedstudio.com/xiao_esp32c3_with_micropython/
- https://wiki.seeedstudio.com/xiao_esp32c3_with_circuitpython/
- https://forum.seeedstudio.com/t/bricked-xiao-esp32-c3-cant-flash-new-code-or-bootloader/281377
- https://www.hackster.io/fusion_automate/how-to-flash-factory-firmware-in-seed-studio-xiao-esp32c3-0a6ecc
- https://hackaday.com/2025/04/07/simple-antenna-makes-for-better-esp32-c3-wifi/

### ESP32-C5 (XIAO) — `878b1f67-d2ac-4e77-a009-ee6f96d1bce5`

Anchor: `ESP32-C5.md` (30 raw → 7 kept; C3/C6/RP2350 leakage removed).

- https://files.seeedstudio.com/wiki/XIAO_ESP32C5/res/esp32-c5_datasheet_en.pdf
- https://www.makerguides.com/getting-started-with-xiao-esp32-c5-and-arduino-ide/
- https://wiki.seeedstudio.com/xaio_esp32c5_wifi_throughput_tester/
- https://wiki.seeedstudio.com/xiao_esp32c5_with_micropyhton/
- https://www.derekseaman.com/2026/03/esphome-introducing-the-seeed-studio-xiao-esp32-c5.html
- https://dronebotworkshop.com/xiao-esp32-c5/
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32c5/api-guides/current-consumption-measurement-modules.html

### ESP32-C6 (XIAO) — `4165a6db-5021-44c0-b1f5-42ceb2b2bab7`

Anchor: `ESP32-C6.md` (39 raw → 3 kept; C3/C5/P4/RP2350 leakage removed).

- https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/
- https://www.hackster.io/JasonFang/easy-diy-zigbee-smart-air-monitor-with-xiao-esp32c6-8e14d0
- https://www.reddit.com/r/esp32/comments/1r7e1mo/esp32c6_super_mini_deep_sleep_power_consumption/

### ESP32-P4 — `25de0275-7d54-44a2-a574-5d4efb8ef62d`

Anchor: `ESP32-P4.md` (16 raw → 9 kept).

- https://documentation.espressif.com/esp32-p4_datasheet_en.pdf
- https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32p4/esp-idf-en-v5.3-esp32p4.pdf
- https://developer.espressif.com/blog/2024/12/pie-introduction/
- https://www.hackster.io/news/espressif-s-new-high-performance-risc-v-esp32-p4-soc-packs-in-tons-of-io-and-security-features-12272d4b067e
- https://viewedisplay.com/esp32-p4-vs-esp32-s3/
- https://www.elecrow.com/blog/esp32-p4-hmi-display-high-performance-guide.html
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/lcd/dsi_lcd.html
- https://manuals.plus/m/b01285359c066391f432882425108fb919e1c2b61273c4f774e033925e2b0429
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-guides/current-consumption-measurement-modules.html

### RP2350 (XIAO) — `c8675955-0cd1-4ab0-a641-58f1ad139bb9`

Anchor: `RP2350.md` (12 raw → 5 kept). Not an ESP32 part; a candidate for a new
"RP2350 / Raspberry Pi silicon" notebook if the repo wants to track it.

- https://wiki.seeedstudio.com/getting-started-xiao-rp2350/
- https://nuttx.apache.org/docs/latest/platforms/arm/rp23xx/boards/xiao-rp2350/index.html
- https://circuitpython.org/board/seeeduino_xiao_rp2350/
- https://cs.stanford.edu/people/nick/low-power-pico/
- https://www.digikey.fi/en/maker/tutorials/2025/what-is-the-rp2350-high-speed-transmit-interface-hstx

---

## Summary — kept vs. dropped per notebook

| Notebook | Raw | Kept | Dropped |
|---|---|---|---|
| ESP32-S3 hardware families | 45 | 5 | 40 (XIAO leakage, AliExpress shortlinks, news) |
| ESP32 classic boards | 24 | 0 | 24 (all leakage/news) |
| Sensors & modules | 7 | 0 | 7 (noise only) |
| Motor drivers & power | 7 | 0 | 7 (noise only) |
| I2S audio | 9 | 2 | 7 |
| USB gadget & HID | 7 | 0 | 7 (noise only) |
| ChameleonUltra RFID/NFC | 7 | 0 | 7 (noise only) |
| ESP-IDF core | 16 | 9 | 7 |
| ESP32 peripherals | 8 | 1 | 7 |
| ESPHome | 57 | 16 | 41 (PNGs, LAN, news) |
| Bluepad32 & BLE HID | 8 | 1 | 7 |
| WiFi / mDNS / MQTT | 9 | 1 | 8 |
| OTA & web flashing | 8 | 1 | 7 |
| AI vision backends | 7 | 0 | 7 (noise only) |
| Claude Code workflow | 62 | 6 | 56 (vault AI/ML MOC bleed) |
| Python simulation | 7 | 0 | 7 (noise only) |
| Soldering guide | 115 | 28 | 87 (PNGs, truncated dupes, PostmarketOS) |
| ESP32-C3 (no mapping yet) | 39 | 8 | 31 (cross-board leakage) |
| ESP32-C5 (no mapping yet) | 30 | 7 | 23 (cross-board leakage) |
| ESP32-C6 (no mapping yet) | 39 | 3 | 36 (cross-board leakage) |
| ESP32-P4 (no mapping yet) | 16 | 9 | 7 |
| RP2350 (no mapping yet) | 12 | 5 | 7 |
| **Total** | **539** | **102** | **437** |

102 curated additions across 22 anchors (15 notebooks in the mapping + 5
new-notebook candidates; 9 anchors yielded only universal noise and contribute
nothing). No single notebook is pushed over the ~30-source cap by these
additions alone — verify against current source counts (`just
notebooks-status`) before adding, and drop a stale source if any notebook
would exceed ~30.
