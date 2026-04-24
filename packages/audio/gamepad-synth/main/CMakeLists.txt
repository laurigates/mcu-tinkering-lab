idf_component_register(SRCS "main.c" "piezo_voice.c" "drums.c" "tts_player.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES
                           "../data/tts_theremin.pcm"
                           "../data/tts_melody.pcm"
                           "../data/tts_effects.pcm"
                       REQUIRES driver nvs_flash esp_timer
                       PRIV_REQUIRES bluepad32)
