idf_component_register(SRCS "main.c" "i2c_slave.c" "i2c_protocol.c" "wifi_manager.c"
                    INCLUDE_DIRS "."
                    REQUIRES "pca9685" "i2cdev" "esp_lcd" "driver" "esp_wifi" "nvs_flash" "lwip"
                    PRIV_REQUIRES "console")
