idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        "bluepad32"
        "esp_tinyusb"
        "switch_pro_usb"
        "bluepad32_host"
        "button_mapper"
        "status_led"
        "driver"
        "esp_timer"
        "nvs_flash"
)
