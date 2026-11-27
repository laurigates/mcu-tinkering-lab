# The WS2812 layer is shared (packages/components/status-led) and compiled
# here, beside this project's mode table; it is not a component of its own.
set(_status_led_core "${CMAKE_CURRENT_LIST_DIR}/../../../../components/status-led")

idf_component_register(
    SRCS "status_led.c" "${_status_led_core}/status_led_core.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "${_status_led_core}/include"
    REQUIRES "led_strip" "esp_timer" "freertos"
)
