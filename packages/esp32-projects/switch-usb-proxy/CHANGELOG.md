# Changelog

## [0.1.1](https://github.com/laurigates/mcu-tinkering-lab/compare/switch-usb-proxy-v0.1.0...switch-usb-proxy-v0.1.1) (2026-04-05)


### Features

* add Switch Pro Controller USB protocol tester ([#127](https://github.com/laurigates/mcu-tinkering-lab/issues/127)) ([4b03d70](https://github.com/laurigates/mcu-tinkering-lab/commit/4b03d704ffd4eeb92293adf21fe8e603a12d370d))
* add Switch USB Proxy project and UART flash tooling ([#131](https://github.com/laurigates/mcu-tinkering-lab/issues/131)) ([d2d583e](https://github.com/laurigates/mcu-tinkering-lab/commit/d2d583e1ae96b61e142b7b3d5e028538a682e632))
* extend web flasher with dynamic project discovery ([#161](https://github.com/laurigates/mcu-tinkering-lab/issues/161)) ([c891678](https://github.com/laurigates/mcu-tinkering-lab/commit/c891678bc6b05affbf204f067e30944bebc9f474)), closes [#152](https://github.com/laurigates/mcu-tinkering-lab/issues/152)
* move Switch Pro Controller protocol handling on-device ([#134](https://github.com/laurigates/mcu-tinkering-lab/issues/134)) ([fca7216](https://github.com/laurigates/mcu-tinkering-lab/commit/fca7216c8e7ffc50b37ba0ef45c5d55f86e6ea11))


### Bug Fixes

* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#160](https://github.com/laurigates/mcu-tinkering-lab/issues/160)) ([c5c6798](https://github.com/laurigates/mcu-tinkering-lab/commit/c5c6798a9f5b44120c8dd10c4eeaa5f1b072ab2a))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* only stamp timer counter on 0x21 subcmd replies, not 0x81 USB replies ([#135](https://github.com/laurigates/mcu-tinkering-lab/issues/135)) ([ecf3b8b](https://github.com/laurigates/mcu-tinkering-lab/commit/ecf3b8b3722450bea1e5e36eaa6467cc0e14d9e6))
* resolve pre-existing lint and format violations ([#147](https://github.com/laurigates/mcu-tinkering-lab/issues/147)) ([6870451](https://github.com/laurigates/mcu-tinkering-lab/commit/68704511a86c76890ccb8d1fb45f7fc16058eb6e))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))
