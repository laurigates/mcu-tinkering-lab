idf_component_register(
    SRCS "raw_usb.c"
    INCLUDE_DIRS "include"
    REQUIRES "freertos"
    # tinyusb_port supplies the vendored DCD layer (dcd_dwc2.c, dwc2_common.c,
    # tusb_fifo.c + headers; ADR-0006). It contributes the strong dcd_*
    # definitions that raw_usb.c calls (RAW_USB_DCD_VENDORED=1) and the
    # `device/dcd.h` / `common/tusb_types.h` / `tusb_option.h` include paths
    # that raw_usb.c pulls in its vendored branch.
    PRIV_REQUIRES "tinyusb_port"
)