/**
 * @file bluepad32_host.c
 * @brief Bluepad32 BLE gamepad host wrapper.
 *
 * Initializes Bluepad32 as a custom platform to receive Xbox controller
 * input over BLE and store it in a shared state struct.
 */

#include "bluepad32_host.h"

#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"
#include "uni_bt.h"
#include "uni_gamepad.h"
#include "uni_hid_device.h"
#include "uni_platform.h"

static const char *TAG = "bluepad32_host";

/* Shared state protected by a single-writer assumption (BP32 task) */
static xbox_gamepad_state_t s_gamepad_state;
static bp32_connection_cb_t s_conn_cb = NULL;

/*--- Bluepad32 Platform Callbacks ---*/

static void bridge_init(int argc, const char **argv) {
    (void)argc;
    (void)argv;
    ESP_LOGI(TAG, "Bluepad32 bridge platform initialized");
}

static void bridge_on_init_complete(void) {
    ESP_LOGI(TAG, "Bluepad32 init complete, scanning for controllers...");
    /* Enable BLE scanning for new devices */
    uni_bt_enable_new_connections_unsafe(true);
}

static void bridge_on_device_connected(uni_hid_device_t *d) {
    ESP_LOGI(TAG, "Device connected: %p", d);
}

static void bridge_on_device_disconnected(uni_hid_device_t *d) {
    ESP_LOGW(TAG, "Device disconnected: %p", d);
    memset(&s_gamepad_state, 0, sizeof(s_gamepad_state));
    s_gamepad_state.connected = false;
    if (s_conn_cb) s_conn_cb(false);
}

static uni_error_t bridge_on_device_ready(uni_hid_device_t *d) {
    ESP_LOGI(TAG, "Device ready: VID=0x%04X PID=0x%04X",
             uni_hid_device_get_vendor_id(d),
             uni_hid_device_get_product_id(d));
    s_gamepad_state.connected = true;
    if (s_conn_cb) s_conn_cb(true);
    return UNI_ERROR_SUCCESS;
}

static void bridge_on_controller_data(uni_hid_device_t *d,
                                       uni_controller_t *ctl) {
    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) return;

    uni_gamepad_t *gp = &ctl->gamepad;
    s_gamepad_state.buttons = gp->buttons;
    s_gamepad_state.dpad = gp->dpad;
    s_gamepad_state.misc_buttons = gp->misc_buttons;
    s_gamepad_state.axis_x = gp->axis_x;
    s_gamepad_state.axis_y = gp->axis_y;
    s_gamepad_state.axis_rx = gp->axis_rx;
    s_gamepad_state.axis_ry = gp->axis_ry;
    s_gamepad_state.brake = gp->brake;
    s_gamepad_state.throttle = gp->throttle;
    s_gamepad_state.connected = true;
}

static const char *bridge_get_variant(void) {
    return "Xbox-Switch Bridge";
}

static void bridge_on_oob_event(uni_platform_oob_event_t event, void *data) {
    (void)data;
    ESP_LOGD(TAG, "OOB event: %d", event);
}

/*--- Platform Registration ---*/

static const uni_platform_t s_bridge_platform = {
    .name = "xbox_switch_bridge",
    .init = bridge_init,
    .on_init_complete = bridge_on_init_complete,
    .on_device_connected = bridge_on_device_connected,
    .on_device_disconnected = bridge_on_device_disconnected,
    .on_device_ready = bridge_on_device_ready,
    .on_controller_data = bridge_on_controller_data,
    .get_property = NULL,
    .on_oob_event = bridge_on_oob_event,
};

/*--- Public API ---*/

esp_err_t bp32_host_init(bp32_connection_cb_t conn_cb) {
    s_conn_cb = conn_cb;
    memset(&s_gamepad_state, 0, sizeof(s_gamepad_state));

    ESP_LOGI(TAG, "Registering Bluepad32 custom platform");
    uni_platform_set_custom(&s_bridge_platform);

    /* Bluepad32 main entry point - starts BT stack */
    uni_init(0, NULL);

    return ESP_OK;
}

void bp32_host_process(void) {
    /* Bluepad32 4.x uses its own task, but we call this to keep the
     * API consistent in case polling is needed in future versions. */
}

bool bp32_host_get_state(xbox_gamepad_state_t *state) {
    if (!s_gamepad_state.connected) return false;
    memcpy(state, &s_gamepad_state, sizeof(xbox_gamepad_state_t));
    return true;
}
