idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_timer
                       PRIV_REQUIRES bluepad32)
