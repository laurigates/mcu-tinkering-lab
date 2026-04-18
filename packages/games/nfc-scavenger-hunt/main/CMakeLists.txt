idf_component_register(
    SRCS "main.c"
         "rc522_driver.c"
         "i2s_audio.c"
         "wifi_manager.c"
         "gemini_tts.c"
         "game_logic.c"
         "prompt_builder.c"
         "local_sounds.c"
         "status_led.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_netif esp_event esp_http_client
             driver esp_psram json mdns
    PRIV_REQUIRES esp_timer mbedtls
)
