idf_component_register(
    SRCS
        "main.c"
        "tone_engine.c"
        "melody_gen.c"
        "pot_reader.c"
        "button.c"
        "led_single.c"
        "standalone_mode.c"
        "group_mode.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_adc led_strip thinkpack-protocol thinkpack-mesh thinkpack-behaviors
)
