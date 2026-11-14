idf_component_register(
    SRCS "log_udp.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_wifi" "esp_event" "esp_netif" "lwip" "freertos"
)

# The cyan "AP ready" flash goes through the consumer's own status_led
# component. status_led is per-project (xbox-switch-bridge and
# facedancer-espdancer-fw each have one), while robocar and the thinkpack boxes
# compile every directory under packages/components and have none, so a hard
# REQUIRES would stop their builds. ESP-IDF resolves REQUIRES before Kconfig is
# loaded, so link status_led only when the build already has it.
idf_build_get_property(_log_udp_build_components BUILD_COMPONENTS)
if(status_led IN_LIST _log_udp_build_components)
    idf_component_optional_requires(PRIVATE status_led)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE LOG_UDP_HAVE_STATUS_LED=1)
endif()
