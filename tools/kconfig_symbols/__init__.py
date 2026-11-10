"""Kconfig symbol lists for checking sdkconfig.defaults without a build.

ESP-IDF ignores an `sdkconfig.defaults` line whose symbol no Kconfig file
defines, and says so only as a warning in the middle of a build log
(`warning: unknown kconfig symbol 'MDNS_ENABLED' assigned to 'y'`). Nine
projects carried `CONFIG_MDNS_ENABLED=y` for months that way (issue #707).

This package holds the symbols ESP-IDF itself defines (`idf-v<ver>.txt`,
extracted from the espressif/idf image) and the symbols each managed component
defines (`components/<ns>__<name>.txt`), and checks every tracked
sdkconfig.defaults against them plus the project's own Kconfig files.

    python3 -m kconfig_symbols.check            # the gate (PYTHONPATH=tools)
    just kconfig-symbols::idf                   # regenerate the ESP-IDF list
    just kconfig-symbols::component <dir>       # record one managed component
"""
