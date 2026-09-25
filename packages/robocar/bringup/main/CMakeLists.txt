# Drivers are pulled from ../../unified/main/ rather than copied.
#
# There is exactly one copy of every pin assignment and every peripheral driver
# on this board, so a wiring fix lands in the robot firmware and in the test
# firmware at the same moment. A copy would drift, and the class of bug it would
# hide — a bringup sweep that passes against last month's pinout — is precisely
# what this project exists to catch.
#
# All ten shared files depend only on pin_config.h plus ESP-IDF and i2cdev; none
# of them reaches for WiFi, HTTP, cJSON or credentials. That is what makes this
# separable at all, and it is worth preserving: if a driver here ever needs a
# network header, it belongs on the robot side of the line, not in this list.
set(UNIFIED_MAIN "${CMAKE_CURRENT_LIST_DIR}/../../unified/main")

idf_component_register(
    SRCS
        "main.c"
        "checks.c"
        "cues.c"
        "oled.c"
        "speech.c"
        # --- shared, unmodified, from robocar-unified ---
        "${UNIFIED_MAIN}/i2c_bus.c"
        "${UNIFIED_MAIN}/pca9685_phase.c"
        "${UNIFIED_MAIN}/gpio_expander.c"
        "${UNIFIED_MAIN}/motor_controller.c"
        "${UNIFIED_MAIN}/led_controller.c"
        "${UNIFIED_MAIN}/servo_controller.c"
        "${UNIFIED_MAIN}/buzzer.c"
        "${UNIFIED_MAIN}/ultrasonic.c"
        "${UNIFIED_MAIN}/audio_player.c"
        "${UNIFIED_MAIN}/voice_fx_core.c"
        "${UNIFIED_MAIN}/mic_pdm.c"
    EMBED_FILES
        # Spoken check names: raw 24 kHz mono s16 PCM, rendered offline by
        # tools/voices.json (see speech.h). The device never talks to the
        # network; regenerating these is an explicit developer action.
        "../data/tts_psram.pcm"
        "../data/tts_flash.pcm"
        "../data/tts_buzzer.pcm"
        "../data/tts_i2c_mux.pcm"
        "../data/tts_i2c_scan.pcm"
        "../data/tts_oled.pcm"
        "../data/tts_leds.pcm"
        "../data/tts_servos.pcm"
        "../data/tts_motors.pcm"
        "../data/tts_mcp23017.pcm"
        "../data/tts_sonar.pcm"
        "../data/tts_amp.pcm"
        "../data/tts_mic.pcm"
    INCLUDE_DIRS
        "."
        "${UNIFIED_MAIN}"
    REQUIRES
        "pca9685"
        "tca9548"
        "mcp23x17"
        "i2cdev"
        "driver"
        "esp_timer"
        "spi_flash"
        "app_update"
)
