idf_component_register(SRCS "main.c" "i2c_slave.c" "wifi_manager.c" "ota_handler.c"
                    INCLUDE_DIRS "."
                    REQUIRES "pca9685" "i2cdev" "esp_lcd" "driver" "nvs_flash" "esp_wifi" "esp_netif" "esp_event" "lwip" "esp_https_ota" "app_update" "robocar-i2c-protocol" "improv-wifi" "mdns"
                    PRIV_REQUIRES "console" "esp_timer")
