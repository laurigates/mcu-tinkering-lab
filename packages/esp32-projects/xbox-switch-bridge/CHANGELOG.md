# Changelog

## [0.1.1](https://github.com/laurigates/mcu-tinkering-lab/compare/xbox-switch-bridge-v0.1.0...xbox-switch-bridge-v0.1.1) (2026-04-05)


### Features

* add esp32-wifitest project and Switch Pro Controller protocol docs ([#126](https://github.com/laurigates/mcu-tinkering-lab/issues/126)) ([f3446d9](https://github.com/laurigates/mcu-tinkering-lab/commit/f3446d9201c50d93576eaf4d46c5ae3e682f9df2))
* add host-based unit tests for i2c_protocol and status_led ([#129](https://github.com/laurigates/mcu-tinkering-lab/issues/129)) ([e9a426a](https://github.com/laurigates/mcu-tinkering-lab/commit/e9a426a3b91d341ae700ddfdaed8973e4f6bba98)), closes [#114](https://github.com/laurigates/mcu-tinkering-lab/issues/114)
* add Switch USB Proxy project and UART flash tooling ([#131](https://github.com/laurigates/mcu-tinkering-lab/issues/131)) ([d2d583e](https://github.com/laurigates/mcu-tinkering-lab/commit/d2d583e1ae96b61e142b7b3d5e028538a682e632))
* extend web flasher with dynamic project discovery ([#161](https://github.com/laurigates/mcu-tinkering-lab/issues/161)) ([c891678](https://github.com/laurigates/mcu-tinkering-lab/commit/c891678bc6b05affbf204f067e30944bebc9f474)), closes [#152](https://github.com/laurigates/mcu-tinkering-lab/issues/152)
* xbox-switch-bridge ([b63d35a](https://github.com/laurigates/mcu-tinkering-lab/commit/b63d35a0187dd8aafd586f7433d8dd2b94e4427c))
* **xbox-switch-bridge:** add UDP log broadcasting, debug build, and USB handshake improvements ([#101](https://github.com/laurigates/mcu-tinkering-lab/issues/101)) ([2381246](https://github.com/laurigates/mcu-tinkering-lab/commit/23812467100f49e33845f4489b0ed004d2193954))
* **xbox-switch-bridge:** add WS2812 status LED and document pinout ([#89](https://github.com/laurigates/mcu-tinkering-lab/issues/89)) ([8c3eb40](https://github.com/laurigates/mcu-tinkering-lab/commit/8c3eb4059c00c74ac941a0419746719fd17f360e))
* **xbox-switch-bridge:** build variants, conditional compilation, firmware fixes ([#106](https://github.com/laurigates/mcu-tinkering-lab/issues/106)) ([a8531eb](https://github.com/laurigates/mcu-tinkering-lab/commit/a8531eb893418f177ea195973b74e9ac77fd80cc))


### Bug Fixes

* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#160](https://github.com/laurigates/mcu-tinkering-lab/issues/160)) ([c5c6798](https://github.com/laurigates/mcu-tinkering-lab/commit/c5c6798a9f5b44120c8dd10c4eeaa5f1b072ab2a))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **auto:** fix C/C++ and Python formatting violations ([#92](https://github.com/laurigates/mcu-tinkering-lab/issues/92)) ([1a25d5a](https://github.com/laurigates/mcu-tinkering-lab/commit/1a25d5a14ce8cfeb570b70aa5997f39fbac57861))
* **auto:** fix C/C++ formatting and trailing whitespace issues ([#72](https://github.com/laurigates/mcu-tinkering-lab/issues/72)) ([de01562](https://github.com/laurigates/mcu-tinkering-lab/commit/de0156207323c212ac3847ee6bc492183db7de58))
* resolve pre-existing lint and format violations ([#147](https://github.com/laurigates/mcu-tinkering-lab/issues/147)) ([6870451](https://github.com/laurigates/mcu-tinkering-lab/commit/68704511a86c76890ccb8d1fb45f7fc16058eb6e))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))
* **switch_pro_usb:** correct SPI address map, ACK bytes, and remove rumble stub ([#124](https://github.com/laurigates/mcu-tinkering-lab/issues/124)) ([dd6e79c](https://github.com/laurigates/mcu-tinkering-lab/commit/dd6e79c0c6445953cd2082edc1b69d5e09c83ddb))
* **xbox-switch-bridge:** correct Switch Pro Controller protocol fidelity ([#122](https://github.com/laurigates/mcu-tinkering-lab/issues/122)) ([b0702f7](https://github.com/laurigates/mcu-tinkering-lab/commit/b0702f73661ab7f66c3c44682ef817cc3eeea5f3))
* **xbox-switch-bridge:** fix TinyUSB handshake, SoftAP visibility, add debug-uart variant ([#121](https://github.com/laurigates/mcu-tinkering-lab/issues/121)) ([1058fe7](https://github.com/laurigates/mcu-tinkering-lab/commit/1058fe720631e807b07778ec7fb324e39adfc8d4))
* **xbox-switch-bridge:** status LED cleanup and TinyUSB init resilience ([#97](https://github.com/laurigates/mcu-tinkering-lab/issues/97)) ([c871c18](https://github.com/laurigates/mcu-tinkering-lab/commit/c871c1847a48b4bb38ffc81430056fcc8dffebdf))


### Miscellaneous

* improve justfile port detection and device recipes ([#103](https://github.com/laurigates/mcu-tinkering-lab/issues/103)) ([a6566e4](https://github.com/laurigates/mcu-tinkering-lab/commit/a6566e4d70082ede8f86eb6e87a2a9c7d59b544f))
