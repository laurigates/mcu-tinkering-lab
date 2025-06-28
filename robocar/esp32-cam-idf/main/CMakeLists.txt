idf_component_register(SRCS "main.c" "camera.c" "wifi_manager.c" "claude_api.c" "base64.c" "ai_response_parser.c" "i2c_master.c" "i2c_protocol.c" "ai_backend.c" "ollama_backend.c" "ollama_discovery.c"
                    INCLUDE_DIRS "."
                    REQUIRES "esp32-camera" "esp_wifi" "esp_http_client" "json" "nvs_flash" "driver" "espressif__mdns")