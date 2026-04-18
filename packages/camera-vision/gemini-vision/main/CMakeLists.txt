idf_component_register(
    SRCS
        "main.c"
        "base64.c"
        "gemini_client.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "index.html"
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_timer
        esp_http_server
        esp_http_client
        esp-tls
        mbedtls
        esp32-camera
        mdns
        json
)
