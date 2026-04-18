idf_component_register(
    SRCS "claude_client.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES "esp_http_client" "mbedtls" "json"
)
