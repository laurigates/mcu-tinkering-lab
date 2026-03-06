idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        "bluepad32"
        "esp_tinyusb"
        "switch_pro_usb"
        "bluepad32_host"
        "button_mapper"
        "driver"
        "esp_timer"
        "nvs_flash"
)
