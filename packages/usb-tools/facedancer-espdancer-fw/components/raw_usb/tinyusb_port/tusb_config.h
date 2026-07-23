/*
 * tusb_config.h — DCD-only TinyUSB port for espdancer (ADR-0006).
 *
 * Configures TinyUSB for the ESP32-S3's DWC2 OTG-FS device controller ONLY.
 * We bypass TinyUSB's `usbd` device stack and supply our own
 * `dcd_event_handler` in components/raw_usb/raw_usb.c, so every class-driver
 * knob (CFG_TUD_CLASS_*) is left at its 0 default and host mode is disabled.
 * The controller is enabled via CFG_TUSB_RHPORT0_MODE = OPT_MODE_DEVICE; that
 * makes tusb_option.h compute CFG_TUD_ENABLED = truthy (needed, because
 * dwc2_common.c is compiled under #if defined(TUP_USBIP_DWC2) &&
 * (CFG_TUH_ENABLED || CFG_TUD_ENABLED)).
 *
 * TinyUSB 0.19.0 (the version `espressif/esp_tinyusb` ≥1.4.0 pulls under
 * ESP-IDF v5.4). Pin recorded in tinyusb_port/README.md.
 */
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#include "tusb_option.h"   /* brings in OPT_MCU_ESP32S3, OPT_MODE_*, OPT_OS_* */

#ifdef __cplusplus
extern "C" {
#endif

/* ── MCU / OS ──────────────────────────────────────────────────────────── */
#define CFG_TUSB_MCU         OPT_MCU_ESP32S3
#define CFG_TUSB_OS          OPT_OS_NONE           /* bare-metal; raw_usb_pump_task runs the loop */

/* ── Root hub: one OTG port, Device mode, Full Speed ───────────────────── */
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE)   /* FS device; no HIGHSPEED bit */
#define CFG_TUSB_RHPORT1_MODE   OPT_MODE_NONE

/* ── Class drivers — all disabled. We bypass usbd entirely (ADR-0006).
 *     Leaving these undefined lets tusb_option.h default them to 0; we set
 *     them explicitly to be defensive against future defaults. ──────────── */
#define CFG_TUD_CDC            0
#define CFG_TUD_MSC            0
#define CFG_TUD_HID            0
#define CFG_TUD_AUDIO          0
#define CFG_TUD_VIDEO          0
#define CFG_TUD_MIDI           0
#define CFG_TUD_VENDOR         0
#define CFG_TUD_USBTMC         0
#define CFG_TUD_DFU            0
#define CFG_TUD_DFU_RT         0
#define CFG_TUD_BTH            0
#define CFG_TUD_ECM_RNDIS      0
#define CFG_TUD_NCM            0

/* No host classes either (host mode disabled above). */
#define CFG_TUH_HUB            0
#define CFG_TUH_CDC            0
#define CFG_TUH_MSC            0
#define CFG_TUH_HID            0
#define CFG_TUH_VENDOR         0
#define CFG_TUH_MAX3421        0

/* EP0 Full-Speed max packet size */
#define CFG_TUD_ENDPOINT0_SIZE 64

/* DMA disabled — internal-RAM-only path; matches xbox-switch-bridge's
 * findings and avoids the dwc2_esp32.h esp_cache.h dependency (guarded by
 * CFG_TUD_DWC2_DMA_ENABLE which we leave at its 0 default). */
/* CFG_TUD_DWC2_DMA_ENABLE   0   (default, don't redefine) */
/* CFG_TUD_DWC2_SLAVE_ENABLE 1   (default, don't redefine) */

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */