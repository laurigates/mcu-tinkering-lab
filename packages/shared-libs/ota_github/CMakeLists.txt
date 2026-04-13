idf_component_register(
    SRCS
        "src/ota_github.c"
        "src/ota_github_pull.c"
        "src/ota_github_direct.c"
        "src/ota_github_mqtt.c"
    INCLUDE_DIRS "include"
    # Public REQUIRES — propagated to consumers that add `ota_github` to
    # their own REQUIRES. Includes esp_event (event base types in
    # ota_github_events.h) and mqtt (for esp_mqtt_client_handle_t in
    # ota_github.h). esp_ghota is declared in idf_component.yml and is
    # auto-propagated by the IDF Component Manager.
    REQUIRES
        "esp_event"
        "mqtt"
    PRIV_REQUIRES
        "log"
        "esp_http_client"
        "esp_https_ota"
        "app_update"
        "esp_partition"
        "esp-tls"
        "freertos"
        "nvs_flash"
        "bootloader_support"
)
