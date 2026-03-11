idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES "nvs_flash" "usb_composite" "status_led" "wifi_manager" "state_machine" "claude_client"
)
