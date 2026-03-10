idf_component_register(
    SRCS "log_udp.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_wifi" "esp_event" "esp_netif" "lwip" "freertos" "status_led"
)
