# Changelog

## [0.1.4](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.3...robocar-unified-v0.1.4) (2026-07-13)


### Bug Fixes

* **build-firmware:** resolve release firmware build failures blocking web flasher deploy ([ac3cd9b](https://github.com/laurigates/mcu-tinkering-lab/commit/ac3cd9be0e7fa5f1e71064ca06916702ddbb67d2)), closes [#362](https://github.com/laurigates/mcu-tinkering-lab/issues/362)

## [0.1.3](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.2...robocar-unified-v0.1.3) (2026-07-13)


### Documentation

* codify build-guide generation as a reusable skill + Typst template ([8aa3a82](https://github.com/laurigates/mcu-tinkering-lab/commit/8aa3a8223f2d14ed1f54977035e0449034c0c4a9))
* **robocar-unified:** add printable Typst build guide ([618f4a7](https://github.com/laurigates/mcu-tinkering-lab/commit/618f4a774de8efc920881a60bce9cd0f795ad601))

## [0.1.2](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.1...robocar-unified-v0.1.2) (2026-07-02)


### Features

* **mqtt-logger:** implement publish() and subscribe() referenced by ota_manager ([#312](https://github.com/laurigates/mcu-tinkering-lab/issues/312)) ([4d26d9f](https://github.com/laurigates/mcu-tinkering-lab/commit/4d26d9f040e744f510ce9634ffcab2b29b96e270))


### Bug Fixes

* **auto:** apply clang-format to remaining firmware files ([#172](https://github.com/laurigates/mcu-tinkering-lab/issues/172)) ([251cc9e](https://github.com/laurigates/mcu-tinkering-lab/commit/251cc9ecac21a1db102a43ba5985d0c04631cc79))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))
* **robocar-unified:** code-quality fixes in mqtt_logger, servo, OTA orchestration ([#304](https://github.com/laurigates/mcu-tinkering-lab/issues/304)) ([18a0af9](https://github.com/laurigates/mcu-tinkering-lab/commit/18a0af9bfb8fb1bc19f2341b588ec0855ac19654))


### Documentation

* **robocar-unified:** add schemdraw schematic ([#263](https://github.com/laurigates/mcu-tinkering-lab/issues/263)) ([2a5a5bf](https://github.com/laurigates/mcu-tinkering-lab/commit/2a5a5bf359514e521f48e20fcf0112ad55677b6c))


### Miscellaneous

* release ([#359](https://github.com/laurigates/mcu-tinkering-lab/issues/359)) ([303717a](https://github.com/laurigates/mcu-tinkering-lab/commit/303717a6c2b51b7df62b846a31161e879223db9d))

## [0.1.1](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.0...robocar-unified-v0.1.1) (2026-07-02)


### Features

* **mqtt-logger:** implement publish() and subscribe() referenced by ota_manager ([#312](https://github.com/laurigates/mcu-tinkering-lab/issues/312)) ([4d26d9f](https://github.com/laurigates/mcu-tinkering-lab/commit/4d26d9f040e744f510ce9634ffcab2b29b96e270))


### Bug Fixes

* **auto:** apply clang-format to remaining firmware files ([#172](https://github.com/laurigates/mcu-tinkering-lab/issues/172)) ([251cc9e](https://github.com/laurigates/mcu-tinkering-lab/commit/251cc9ecac21a1db102a43ba5985d0c04631cc79))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))
* **robocar-unified:** code-quality fixes in mqtt_logger, servo, OTA orchestration ([#304](https://github.com/laurigates/mcu-tinkering-lab/issues/304)) ([18a0af9](https://github.com/laurigates/mcu-tinkering-lab/commit/18a0af9bfb8fb1bc19f2341b588ec0855ac19654))


### Documentation

* **robocar-unified:** add schemdraw schematic ([#263](https://github.com/laurigates/mcu-tinkering-lab/issues/263)) ([2a5a5bf](https://github.com/laurigates/mcu-tinkering-lab/commit/2a5a5bf359514e521f48e20fcf0112ad55677b6c))
