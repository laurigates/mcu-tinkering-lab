# Changelog

## [0.1.8](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.7...robocar-main-v0.1.8) (2026-09-11)


### Miscellaneous

* **robocar:** migrate esp-idf-lib from a vendored snapshot to managed components ([#529](https://github.com/laurigates/mcu-tinkering-lab/issues/529)) ([8b3b9dd](https://github.com/laurigates/mcu-tinkering-lab/commit/8b3b9dd0d958bba82c134d76bbb5f8f13a0798aa))

## [0.1.7](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.6...robocar-main-v0.1.7) (2026-09-05)


### Bug Fixes

* **robocar-unified:** two hardware-phase init bugs exposed by the first boot with the PCA9685 fitted ([#498](https://github.com/laurigates/mcu-tinkering-lab/issues/498)) ([6a6f94e](https://github.com/laurigates/mcu-tinkering-lab/commit/6a6f94e3c71eb785ba2c330e2f4482f62a5e2baa))

## [0.1.6](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.5...robocar-main-v0.1.6) (2026-08-26)


### Bug Fixes

* **tools:** gate shared flash recipes on the assumptions they bake in ([#476](https://github.com/laurigates/mcu-tinkering-lab/issues/476)) ([0f9fbbe](https://github.com/laurigates/mcu-tinkering-lab/commit/0f9fbbe306f0f654071473dc9031c79052ac1ea7))

## [0.1.5](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.4...robocar-main-v0.1.5) (2026-08-22)


### Miscellaneous

* drop cargo-culted 'set positional-arguments' from justfiles ([998a9c4](https://github.com/laurigates/mcu-tinkering-lab/commit/998a9c479095bb2018e11b37af512d793dc17fa3)), closes [#410](https://github.com/laurigates/mcu-tinkering-lab/issues/410)

## [0.1.4](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.3...robocar-main-v0.1.4) (2026-07-14)


### Bug Fixes

* **robocar:** apply custom OTA partition table via sdkconfig so release builds fit ([#389](https://github.com/laurigates/mcu-tinkering-lab/issues/389)) ([ea72395](https://github.com/laurigates/mcu-tinkering-lab/commit/ea72395109229d2359141ce9fa74ad90721c1afd))

## [0.1.3](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.2...robocar-main-v0.1.3) (2026-07-13)


### Bug Fixes

* **build-firmware:** resolve release firmware build failures blocking web flasher deploy ([ac3cd9b](https://github.com/laurigates/mcu-tinkering-lab/commit/ac3cd9be0e7fa5f1e71064ca06916702ddbb67d2)), closes [#362](https://github.com/laurigates/mcu-tinkering-lab/issues/362)
* **robocar-main:** correct uint32_t/int32_t log format specifiers ([8f93f45](https://github.com/laurigates/mcu-tinkering-lab/commit/8f93f4589b37c6bde901d1cadca3ccfeefc749fa))

## [0.1.2](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.1...robocar-main-v0.1.2) (2026-07-02)


### Bug Fixes

* **auto:** apply clang-format to remaining firmware files ([#172](https://github.com/laurigates/mcu-tinkering-lab/issues/172)) ([251cc9e](https://github.com/laurigates/mcu-tinkering-lab/commit/251cc9ecac21a1db102a43ba5985d0c04631cc79))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **robocar-main:** resolve I2C_NUM_1 driver-install conflict ([#311](https://github.com/laurigates/mcu-tinkering-lab/issues/311)) ([6b8bc0b](https://github.com/laurigates/mcu-tinkering-lab/commit/6b8bc0bad2a57c3353d1a057cd5d282af2e4fb11))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))


### Miscellaneous

* release ([#359](https://github.com/laurigates/mcu-tinkering-lab/issues/359)) ([303717a](https://github.com/laurigates/mcu-tinkering-lab/commit/303717a6c2b51b7df62b846a31161e879223db9d))

## [0.1.1](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-main-v0.1.0...robocar-main-v0.1.1) (2026-07-02)


### Bug Fixes

* **auto:** apply clang-format to remaining firmware files ([#172](https://github.com/laurigates/mcu-tinkering-lab/issues/172)) ([251cc9e](https://github.com/laurigates/mcu-tinkering-lab/commit/251cc9ecac21a1db102a43ba5985d0c04631cc79))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **robocar-main:** resolve I2C_NUM_1 driver-install conflict ([#311](https://github.com/laurigates/mcu-tinkering-lab/issues/311)) ([6b8bc0b](https://github.com/laurigates/mcu-tinkering-lab/commit/6b8bc0bad2a57c3353d1a057cd5d282af2e4fb11))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))
