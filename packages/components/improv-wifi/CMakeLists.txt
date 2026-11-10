idf_component_register(SRCS "improv_wifi.c" "improv_wifi_uart.c" "improv_wifi_device_info.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "log" "freertos" "esp_app_format")

# RPC 0x03's firmware name must equal the web-flasher manifest's `name`, or
# ESP Web Tools offers "Install" (an erase) instead of "Update". The manifest
# takes `name` from the project's flasher.json (tools/generate-flasher-manifests.sh),
# so read it from there rather than repeat it in C. A project with no
# flasher.json reports its CMake project name (improv_wifi_device_info.c).
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    idf_build_get_property(improv_project_dir PROJECT_DIR)
    set(improv_flasher_json "${improv_project_dir}/flasher.json")
    if(EXISTS "${improv_flasher_json}")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${improv_flasher_json}")
        file(READ "${improv_flasher_json}" improv_flasher_text)
        string(JSON improv_firmware_name ERROR_VARIABLE improv_json_error
               GET "${improv_flasher_text}" name)
        if(improv_json_error)
            message(FATAL_ERROR "improv-wifi: cannot read `name` from ${improv_flasher_json}: "
                                "${improv_json_error}")
        endif()
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
                                   "IMPROV_FIRMWARE_NAME=\"${improv_firmware_name}\"")
    endif()
endif()
