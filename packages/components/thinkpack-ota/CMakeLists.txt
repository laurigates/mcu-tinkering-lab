idf_component_register(
    SRCS "chunk_state.c"
         "manifest_parser.c"
         "sha256_verify.c"
         "apply.c"
         "ota_receiver.c"
    INCLUDE_DIRS "include"
    REQUIRES thinkpack-protocol thinkpack-mesh
             app_update esp_partition esp_system log
             mbedtls esp_common
)
