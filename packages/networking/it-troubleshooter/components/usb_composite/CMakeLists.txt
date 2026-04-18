idf_component_register(
    SRCS "usb_composite.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_tinyusb" "driver" "esp_timer"
)
