set(REQUIRES
    "bluepad32"
    "bluepad32_host"
    "button_mapper"
    "status_led"
    "log_udp"
    "driver"
    "esp_timer"
    "nvs_flash"
)

# Only link TinyUSB / switch_pro_usb when TinyUSB is enabled (not in debug builds)
if(CONFIG_TINYUSB_ENABLED)
    list(APPEND REQUIRES "esp_tinyusb" "switch_pro_usb")
endif()

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ${REQUIRES}
)
