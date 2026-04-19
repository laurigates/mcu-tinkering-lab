idf_component_register(SRCS "espnow_mesh.c"
                            "leader_election.c"
                            "group_manager.c"
                            "fragment_reassembler.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_now log thinkpack-protocol
                    PRIV_REQUIRES freertos)
