idf_component_register(
    SRCS
        "main.c"
        "audio_test.c"
        "button.c"
        "camera_capture.c"
        "status_led.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_timer
        esp32-camera
)
