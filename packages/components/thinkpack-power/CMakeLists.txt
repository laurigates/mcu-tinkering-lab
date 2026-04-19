idf_component_register(
    SRCS "power_mgr.c"
         "classify.c"
         "led_pulse.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc esp_timer thinkpack-protocol thinkpack-mesh
    PRIV_REQUIRES log
)
