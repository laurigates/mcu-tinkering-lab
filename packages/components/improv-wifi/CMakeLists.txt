idf_component_register(SRCS "improv_wifi.c" "improv_wifi_uart.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "log" "freertos")
