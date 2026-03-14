# All components listed unconditionally — ESP-IDF resolves REQUIRES before
# kconfig is loaded (two-pass build), so CONFIG_ guards don't work here.
# Conditional compilation is handled in main.c via #if CONFIG_BT_ENABLED
# and #if CONFIG_TINYUSB_ENABLED source-level guards.
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        status_led
        log_udp
        driver
        esp_timer
        nvs_flash
        bluepad32
        bluepad32_host
        button_mapper
        esp_tinyusb
        switch_pro_usb
)
