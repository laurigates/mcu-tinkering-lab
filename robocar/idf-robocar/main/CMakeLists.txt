idf_component_register(SRCS "main.c" "i2c_slave.c" "i2c_protocol.c"
                    INCLUDE_DIRS "."
                    REQUIRES "pca9685" "i2cdev" "esp_lcd" "driver"
                    PRIV_REQUIRES "console")
