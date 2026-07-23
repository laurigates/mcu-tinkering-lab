idf_component_register(
    SRCS "usb_rpc.c"
    INCLUDE_DIRS "include"
    REQUIRES "lwip" "freertos"
)