# All components listed unconditionally — ESP-IDF resolves REQUIRES before
# kconfig is loaded (two-pass build), so CONFIG_ guards don't work here.
# Conditional compilation is handled in main.c via source-level guards.
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        status_led
        log_udp
        usb_rpc
        raw_usb
        nvs_flash
        esp_timer
        freertos
)