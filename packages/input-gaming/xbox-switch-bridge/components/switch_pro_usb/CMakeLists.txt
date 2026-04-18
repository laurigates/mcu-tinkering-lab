idf_component_register(
    SRCS "switch_pro_usb.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_tinyusb" "driver" "esp_timer"
)
