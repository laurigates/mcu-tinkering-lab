set(REQUIRES
    "status_led"
    "log_udp"
    "driver"
    "esp_timer"
    "nvs_flash"
)

# BLE components (bluepad32, btstack) require CONFIG_BT_ENABLED.
# In wifi-test builds (BT disabled), these components are excluded
# and main.c uses #if CONFIG_BT_ENABLED guards to skip BLE code.
set(BT_COMPONENTS "bluepad32" "bluepad32_host" "button_mapper")
set(USB_COMPONENTS "esp_tinyusb" "switch_pro_usb")

# Include BLE components only when BT is enabled.
# CONFIG_BT_ENABLED is always defined after kconfig processing (y or n),
# so this guard works reliably on both fresh and incremental builds.
if(CONFIG_BT_ENABLED)
    list(APPEND REQUIRES ${BT_COMPONENTS})
endif()

# CONFIG_TINYUSB_ENABLED is defined in main/Kconfig.projbuild (default y).
# The wifi-test overlay sets it to n to keep USB-Serial-JTAG for monitoring.
if(CONFIG_TINYUSB_ENABLED)
    list(APPEND REQUIRES ${USB_COMPONENTS})
endif()

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)
