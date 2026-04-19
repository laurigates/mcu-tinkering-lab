idf_component_register(
    SRCS "main.c"
         "thinkpack_ai.c"
         "claude_text_backend.c"
         "gemini_text_backend.c"
         "ollama_text_backend.c"
         "prompt_builder.c"
         "wifi_manager.c"
         "credentials_loader.c"
         "ollama_discovery.c"
         "response_parser.c"
         "display_manager.c"
         "standalone_mode.c"
         "group_mode.c"
    INCLUDE_DIRS "."
    REQUIRES thinkpack-protocol thinkpack-mesh thinkpack-behaviors
             esp_wifi esp_event esp_netif esp_http_client esp_timer
             nvs_flash mdns driver esp_adc json improv-wifi
             led_strip
    PRIV_REQUIRES freertos
)
