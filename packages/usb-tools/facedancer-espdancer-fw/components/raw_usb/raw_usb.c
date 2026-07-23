/**
 * @file raw_usb.c
 * @brief Minimal USB device stack: our own `dcd_event_handler` + relay pump.
 *
 * See docs/raw-usb-design.md and ADR-0006. tinyusb_port/ (ADR-0006) is now
 * vendored and supplies the strong dcd_* definitions; raw_usb.c provides
 * dcd_event_handler itself, bypassing TinyUSB's usbd. The #else branch below
 * is dead code kept so flipping RAW_USB_DCD_VENDORED back to 0 still compiles
 * (useful for diffing stub vs. real behaviour).
 */

#include "raw_usb.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* --- Vendored DCD surface ---
 * tinyusb_port/ supplies the strong dcd_* definitions (ADR-0006).
 * raw_usb.c provides dcd_event_handler itself, bypassing usbd.
 */
#define RAW_USB_DCD_VENDORED 1  /* tinyusb_port/ is now vendored (ADR-0006) */

#if RAW_USB_DCD_VENDORED
#include "device/dcd.h"           /* DCD protos + dcd_event_t (+ helpers) */
#include "common/tusb_types.h"    /* tusb_rhport_init_t, TUSB_ROLE_DEVICE */
#include "tusb_option.h"         /* OPT_MODE_*, TUSB_SPEED_FULL */
#include "osal/osal.h"           /* osal_spin_* + OSAL_SPINLOCK_DEF (usbd_spin_*) */
#else
/* Minimal type shims so this file compiles even before the port is vendored. */
typedef struct { uint8_t bmRequestType, bRequest; uint16_t wValue, wIndex, wLength; }
        tusb_control_request_t;
typedef struct {
    uint8_t rhport, event_id;
    union {
        tusb_control_request_t setup_received;
        struct { uint8_t ep_addr, result; uint32_t len; } xfer_complete;
        struct { uint8_t speed; } bus_reset;
    };
} dcd_event_t;
enum { DCD_EVENT_BUS_RESET=1, DCD_EVENT_SETUP_RECEIVED=6, DCD_EVENT_XFER_COMPLETE=7,
       DCD_EVENT_SUSPEND=4, DCD_EVENT_RESUME=5 };
extern bool dcd_init(uint8_t rhport, const void *init);
extern void dcd_connect(uint8_t rhport);
extern void dcd_disconnect(uint8_t rhport);
extern void dcd_int_enable(uint8_t rhport);
extern bool dcd_edpt_open(uint8_t rhport, const void *desc_ep);
extern bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buf, uint16_t len);
extern void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr);
extern void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr);
extern void dcd_edpt_close_all(uint8_t rhport);
extern void dcd_set_address(uint8_t rhport, uint8_t addr);
#define TUSB_SPEED_FULL 0
#define TUSB_ROLE_DEVICE 1
typedef struct { int role; int speed; } tusb_rhport_init_t;
#endif

static const char *TAG = "raw_usb";

#define USB_RHPORT 0   /* ESP32-S3 has one OTG port (rhport 0) */
#define EVENT_QUEUE_LEN 32
#define OUT_BUF_MAX     512

/* --- Endpoint table (single-writer-owned by the pump; ADR-0005) --- */
typedef struct {
    raw_usb_ep_state_t state;
    uint8_t  addr;       /* with direction bit */
    uint8_t  type;
    uint16_t max_packet;
    /* OUT: scratch buffer; IN: unused (host-supplied per send). */
    uint8_t *out_buf; uint16_t out_len;
} ep_entry_t;

static ep_entry_t s_ep[RAW_USB_MAX_ENDPOINTS];
static uint8_t    s_out_buffers[RAW_USB_MAX_ENDPOINTS][OUT_BUF_MAX];

/* --- Runtime descriptor table (host-populated at raw_usb_connect) --- */
static raw_usb_descriptors_t s_desc;
static volatile bool   s_connected  = false;
static volatile bool   s_enumerated = false;
static volatile uint8_t s_quirks     = 0;
static QueueHandle_t s_event_q = NULL;

/* --- Helpers --- */
static int ep_index(uint8_t addr) { return addr & RAW_USB_ADDR_EP_MASK; }
static bool ep_dir_in(uint8_t addr) { return (addr & RAW_USB_ADDR_DIR_IN) != 0; }

/* --- usbd.c-provided symbols the vendored DCD references ---
 * dcd_dwc2.c guards its register access with usbd_spin_lock/unlock, normally
 * defined in TinyUSB's usbd.c. Since we bypass usbd (ADR-0006), we supply them
 * here as a faithful port of usbd.c (TinyUSB 0.19.0): the spinlock toggles the
 * DCD interrupt on our single rhport. Under CFG_TUSB_OS==OPT_OS_NONE this is a
 * no-op in ISR context and an int-disable in task context (osal_none.h).
 */
#if RAW_USB_DCD_VENDORED
static void usbd_int_set(bool enabled)
{
    if (enabled) dcd_int_enable(USB_RHPORT);
    else         dcd_int_disable(USB_RHPORT);
}
static OSAL_SPINLOCK_DEF(_usbd_spin, usbd_int_set);
void usbd_spin_lock(bool in_isr)   { osal_spin_lock(&_usbd_spin, in_isr); }
void usbd_spin_unlock(bool in_isr) { osal_spin_unlock(&_usbd_spin, in_isr); }
#endif

/* --- dcd_event_handler: the bridge from DCD ISR to the relay pump ---
 * ISR context. NEVER sends on endpoints. Just enqueue (ADR-0005).
 * Implemented here (strong) since we bypass TinyUSB's usbd.c (ADR-0006).
 */
void dcd_event_handler(dcd_event_t const *ev, bool in_isr)
{
    BaseType_t hpw = pdFALSE;
    if (!s_event_q) return;
    if (in_isr) {
        xQueueSendFromISR(s_event_q, ev, &hpw);
        if (hpw) portYIELD_FROM_ISR();
    } else {
        xQueueSend(s_event_q, ev, portMAX_DELAY);
    }
}

/* --- Init ---
 * dcd_init installs the USB ISR via esp_intr_alloc (dwc2_common.c). Once it
 * returns, BUS_RESET events will start flowing into s_event_q when a host
 * drives SE0. We pass a default-role/speed init struct.
 */
esp_err_t raw_usb_init(void)
{
    memset(s_ep, 0, sizeof(s_ep));
    for (int i = 0; i < RAW_USB_MAX_ENDPOINTS; i++) {
        s_ep[i].out_buf = s_out_buffers[i];
    }
    s_event_q = xQueueCreate(EVENT_QUEUE_LEN, sizeof(dcd_event_t));
    if (!s_event_q) {
        ESP_LOGE(TAG, "event queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

#if RAW_USB_DCD_VENDORED
    tusb_rhport_init_t rh_init = { .role = TUSB_ROLE_DEVICE,
                                   .speed = TUSB_SPEED_FULL };
    if (!dcd_init(USB_RHPORT, &rh_init)) {
        ESP_LOGE(TAG, "dcd_init failed");
        return ESP_FAIL;
    }
    dcd_int_enable(USB_RHPORT);
#endif
    ESP_LOGI(TAG, "raw_usb init (dcd_vendored=%d)", RAW_USB_DCD_VENDORED);
    return ESP_OK;
}

/* --- Lifecycle primitives (wire to DCD once vendored) --- */
esp_err_t raw_usb_connect(const raw_usb_descriptors_t *desc,
                          uint8_t ep0_max, uint8_t speed, uint8_t quirks)
{
    if (!desc) return ESP_ERR_INVALID_ARG;
    if (speed != RAW_USB_SPEED_FULL) {
        ESP_LOGW(TAG, "ESP32-S3 only supports Full Speed; ignoring speed=%u",
                 speed);
    }
    s_desc = *desc;
    s_quirks = quirks;
    memset(s_ep, 0, sizeof(s_ep));
#if RAW_USB_DCD_VENDORED
    tusb_rhport_init_t rh_init = { .role = TUSB_ROLE_DEVICE,
                                   .speed = TUSB_SPEED_FULL };
    if (!dcd_init(USB_RHPORT, &rh_init)) return ESP_FAIL;
    dcd_connect(USB_RHPORT);   /* assert D+ pull-up -> host sees attach */
    dcd_int_enable(USB_RHPORT);
#endif
    s_connected = true;
    ESP_LOGI(TAG, "connect (ep0_max=%u, quirks=0x%02X)", ep0_max, quirks);
    return ESP_OK;
}

esp_err_t raw_usb_disconnect(void)
{
#if RAW_USB_DCD_VENDORED
    if (s_connected) dcd_disconnect(USB_RHPORT);
    dcd_edpt_close_all(USB_RHPORT);
#endif
    s_connected = s_enumerated = false;
    memset(s_ep, 0, sizeof(s_ep));
    return ESP_OK;
}

esp_err_t raw_usb_reset(void)
{
#if RAW_USB_DCD_VENDORED
    dcd_disconnect(USB_RHPORT);
    memset(s_ep, 0, sizeof(s_ep));
    dcd_connect(USB_RHPORT);
#else
    memset(s_ep, 0, sizeof(s_ep));
#endif
    s_enumerated = false;
    return ESP_OK;
}

esp_err_t raw_usb_configure_ep(uint8_t addr, uint8_t type, uint16_t max_pkt)
{
    int idx = ep_index(addr);
    if (idx <= 0 || idx >= RAW_USB_MAX_ENDPOINTS) {
        ESP_LOGE(TAG, "ep index %d out of range (max %d)", idx, RAW_USB_MAX_ENDPOINTS-1);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ep[idx].state != RAW_USB_EP_UNUSED) {
        ESP_LOGW(TAG, "ep %d already configured -- re-opening", idx);
    }
    s_ep[idx].state      = RAW_USB_EP_OPEN;
    s_ep[idx].addr       = addr;
    s_ep[idx].type       = type;
    s_ep[idx].max_packet = max_pkt;
#if RAW_USB_DCD_VENDORED
    /* TODO build a tusb_desc_endpoint_t (bEndpointAddress=addr, bmAttributes=type,
     * wMaxPacketSize=max_pkt, bInterval per type) and call dcd_edpt_open(). */
    ESP_LOGW(TAG, "raw_usb_configure_ep: dcd_edpt_open wiring pending");
#else
    ESP_LOGI(TAG, "configure_ep(addr=0x%02X type=%u max=%u) [stub]", addr, type, max_pkt);
#endif
    return ESP_OK;
}

esp_err_t raw_usb_send(uint8_t ep, const uint8_t *data, uint16_t len)
{
    int idx = ep_index(ep);
    if (idx >= RAW_USB_MAX_ENDPOINTS || s_ep[idx].state == RAW_USB_EP_UNUSED) {
        return ESP_ERR_INVALID_STATE;
    }
#if RAW_USB_DCD_VENDORED
    return dcd_edpt_xfer(USB_RHPORT, ep | RAW_USB_ADDR_DIR_IN,
                         (uint8_t *)data, len) ? ESP_OK : ESP_FAIL;
#else
    ESP_LOGI(TAG, "send(ep=0x%02X len=%u) [stub]", ep, len);
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t raw_usb_stall(uint8_t ep, uint8_t dir)
{
    uint8_t addr = ep | (dir == RAW_USB_DIR_IN ? RAW_USB_ADDR_DIR_IN : 0);
    int idx = ep_index(addr);
    if (idx >= RAW_USB_MAX_ENDPOINTS) return ESP_ERR_INVALID_ARG;
#if RAW_USB_DCD_VENDORED
    dcd_edpt_stall(USB_RHPORT, addr);
#endif
    if (s_ep[idx].state != RAW_USB_EP_UNUSED)
        s_ep[idx].state = RAW_USB_EP_STALLED;
    return ESP_OK;
}

esp_err_t raw_usb_clear_halt(uint8_t ep, uint8_t dir)
{
    uint8_t addr = ep | (dir == RAW_USB_DIR_IN ? RAW_USB_ADDR_DIR_IN : 0);
    int idx = ep_index(addr);
#if RAW_USB_DCD_VENDORED
    if (idx < RAW_USB_MAX_ENDPOINTS && s_ep[idx].state == RAW_USB_EP_STALLED)
        dcd_edpt_clear_stall(USB_RHPORT, addr);
#endif
    if (idx < RAW_USB_MAX_ENDPOINTS && s_ep[idx].state == RAW_USB_EP_STALLED)
        s_ep[idx].state = RAW_USB_EP_OPEN;
    return ESP_OK;
}

esp_err_t raw_usb_ack_status(uint8_t dir, uint8_t ep)
{
    /* ZLP on the endpoint OPPOSITE the data-stage direction. */
    uint8_t addr = ep | (dir == RAW_USB_DIR_OUT ? RAW_USB_ADDR_DIR_IN : 0);
#if RAW_USB_DCD_VENDORED
    return dcd_edpt_xfer(USB_RHPORT, addr, NULL, 0) ? ESP_OK : ESP_FAIL;
#else
    ESP_LOGI(TAG, "ack_status(dir=%u ep=%u) [stub]", dir, ep);
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t raw_usb_set_address(uint8_t addr, uint8_t defer)
{
    if (!(s_quirks & RAW_USB_QUIRK_MANUAL_SET_ADDRESS) && !defer) {
#if RAW_USB_DCD_VENDORED
        dcd_set_address(USB_RHPORT, addr);
#endif
    } else if (s_quirks & RAW_USB_QUIRK_MANUAL_SET_ADDRESS) {
#if RAW_USB_DCD_VENDORED
        if (!defer) dcd_set_address(USB_RHPORT, addr);
#endif
        ESP_LOGI(TAG, "manual set_address(%u, defer=%u)", addr, defer);
    }
    return ESP_OK;
}

/* --- Introspection --- */
bool raw_usb_ready(void) { return s_connected && s_enumerated; }

raw_usb_ep_state_t raw_usb_ep_state(uint8_t ep, uint8_t dir)
{
    int idx = ep_index(ep | (dir == RAW_USB_DIR_IN ? RAW_USB_ADDR_DIR_IN : 0));
    if (idx >= RAW_USB_MAX_ENDPOINTS) return RAW_USB_EP_UNUSED;
    return s_ep[idx].state;
}

/* --- Relay pump: sole writer to the DCD; drains the event queue ---
 * Forwards DCD events to the host via usb_rpc_send_event() (M2 wiring).
 * In M1, before M2 hooks the dispatcher, this just logs events to prove the
 * USB ISR fires (e.g. DCD_EVENT_BUS_RESET on a cable plug).
 */
void raw_usb_pump_task(void *arg)
{
    (void)arg;
    dcd_event_t ev;
    ESP_LOGI(TAG, "pump task up");
    for (;;) {
        if (!xQueueReceive(s_event_q, &ev, portMAX_DELAY)) continue;
        switch (ev.event_id) {
        case DCD_EVENT_BUS_RESET:
            s_enumerated = false;
            memset(s_ep, 0, sizeof(s_ep));
            ESP_LOGI(TAG, "EVT bus_reset (speed=%u)", ev.bus_reset.speed);
            /* TODO: mark enumerated once STABLE — currently set only for log
             * readability; the real enumeration flag should follow a stable
             * address / SET_CONFIG loop. */
            break;
        case DCD_EVENT_SETUP_RECEIVED:
            ESP_LOGI(TAG, "EVT setup ep0 bmRT=0x%02X bReq=0x%02X",
                     ev.setup_received.bmRequestType, ev.setup_received.bRequest);
            /* TODO M2: usb_rpc_send_event(RAW_USB_EVENT_SETUP, setup[8], 1+8) */
            break;
        case DCD_EVENT_XFER_COMPLETE: {
            uint8_t ep = ev.xfer_complete.ep_addr & RAW_USB_ADDR_EP_MASK;
            bool in = ep_dir_in(ev.xfer_complete.ep_addr);
            ESP_LOGD(TAG, "EVT xfer ep=%u %s len=%lu", ep,
                     in ? "IN" : "OUT", (unsigned long)ev.xfer_complete.len);
            /* TODO M2: OUT -> EVENT_OUT_PACKET; IN -> EVENT_SEND_COMPLETE */
            break;
        }
        case DCD_EVENT_SUSPEND:
        case DCD_EVENT_RESUME:
            ESP_LOGI(TAG, "EVT %s", ev.event_id == DCD_EVENT_SUSPEND ? "suspend" : "resume");
            break;
        default:
            break;
        }
    }
}