idf_component_register(SRCS "main.c" "piezo_voice.c" "drums.c" "tts_player.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES
                           "../data/tts_continuous.pcm"
                           "../data/tts_discrete.pcm"
                           "../data/tts_one_shot.pcm"
                       REQUIRES driver nvs_flash esp_timer
                       PRIV_REQUIRES bluepad32)
