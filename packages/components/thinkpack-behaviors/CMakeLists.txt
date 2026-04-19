idf_component_register(SRCS "command_dispatcher.c"
                            "command_executor.c"
                            "echo_chamber.c"
                    INCLUDE_DIRS "include"
                    REQUIRES thinkpack-protocol log esp_common
                    PRIV_REQUIRES freertos)
