idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES "esp32-camera" "esp_wifi" "nvs_flash" "esp_netif" "esp_event"
                             "esp_http_server" "esp_timer")
