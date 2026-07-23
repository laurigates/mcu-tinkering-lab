# DCD-only TinyUSB port (ADR-0006). Vendored, not a managed component.
#
# Compiles exactly the DWC2 device controller driver + supporting common
# sources from TinyUSB 0.19.0 (pinned; see README.md) — NOT usbd.c, NOT the
# class drivers. raw_usb.c supplies `dcd_event_handler` itself, bypassing usbd.

idf_component_register(
    SRCS
        "src/common/tusb_fifo.c"
        "src/portable/synopsys/dwc2/dcd_dwc2.c"
        "src/portable/synopsys/dwc2/dwc2_common.c"
    INCLUDE_DIRS
        "."        # so #include "tusb_config.h" resolves (ours)
        "src"      # so #include "common/...", "device/...", "osal/..." resolve
    PRIV_INCLUDE_DIRS
        # DCD source files include siblings as #include "dwc2_common.h"
        # (GCC searches the including file's own dir by default, but be
        #  explicit to survive future toolchain changes).
        "src/portable/synopsys/dwc2"
    PRIV_REQUIRES
        driver
        esp_hw_support   # esp_intr_alloc.h, ETS_USB_INTR_SOURCE
        soc              # soc/periph_defs.h, soc/usb_wrap_struct.h
        esp_timer
        freertos
)

# TinyUSB compile knobs. -DCFG_TUSB_CONFIG_FILE pins whose tusb_config.h wins.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -DCFG_TUSB_MCU=OPT_MCU_ESP32S3
    -DCFG_TUSB_OS=OPT_OS_NONE
    -DCFG_TUSB_CONFIG_FILE=\"tusb_config.h\"
    -Wno-type-limits       # mirrors the upstream esp_tinyusb usbd.c compile flag
    -Wno-unused-parameter
)