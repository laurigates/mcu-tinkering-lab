# Changelog

## [0.1.12](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.11...robocar-unified-v0.1.12) (2026-07-24)


### Bug Fixes

* **robocar-unified:** give the self_report task an 8 KB stack for its TLS call ([#425](https://github.com/laurigates/mcu-tinkering-lab/issues/425)) ([daaa0f3](https://github.com/laurigates/mcu-tinkering-lab/commit/daaa0f319d0c7de80f765a690734d4995ed29fe8))
* **robocar-unified:** make the ultrasonic RMT receive path functional (signal ranges + no-echo recovery) ([#424](https://github.com/laurigates/mcu-tinkering-lab/issues/424)) ([021679f](https://github.com/laurigates/mcu-tinkering-lab/commit/021679f99b4f24accba45c2daa1b10a8f5371816))

## [0.1.11](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.10...robocar-unified-v0.1.11) (2026-07-24)


### Features

* **robocar-unified:** speak a self-introduction + status self-diagnostic ([#421](https://github.com/laurigates/mcu-tinkering-lab/issues/421)) ([cd5273f](https://github.com/laurigates/mcu-tinkering-lab/commit/cd5273f021b9bac74359e88b630a521e756e2324))


### Bug Fixes

* **robocar-unified:** size ultrasonic RMT RX block to the SoC minimum (48) ([#422](https://github.com/laurigates/mcu-tinkering-lab/issues/422)) ([c011ac7](https://github.com/laurigates/mcu-tinkering-lab/commit/c011ac785edaf67059373650ecf5454d7ea7a6fa))

## [0.1.10](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.9...robocar-unified-v0.1.10) (2026-07-23)


### Bug Fixes

* **robocar-unified:** unbrick the boot — flash offset + I2C driver conflict + graceful degradation ([#418](https://github.com/laurigates/mcu-tinkering-lab/issues/418)) ([2203d66](https://github.com/laurigates/mcu-tinkering-lab/commit/2203d66a0eda4eb37edc1b742981dba88366d0d3))

## [0.1.9](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.8...robocar-unified-v0.1.9) (2026-07-20)


### Features

* **robocar-unified:** give the robot a voice via MAX98357A + Gemini TTS ([#412](https://github.com/laurigates/mcu-tinkering-lab/issues/412)) ([5379d83](https://github.com/laurigates/mcu-tinkering-lab/commit/5379d8319a97e81b29541342747668a1b7933990))

## [0.1.8](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.7...robocar-unified-v0.1.8) (2026-07-17)


### Features

* **robocar-unified:** MCP23017 GPIO expander + latent IDF 5.4 build repairs ([#399](https://github.com/laurigates/mcu-tinkering-lab/issues/399)) ([4bbf98e](https://github.com/laurigates/mcu-tinkering-lab/commit/4bbf98ea5dcd92d90953d94fc96784be40d678e9))

## [0.1.7](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.6...robocar-unified-v0.1.7) (2026-07-16)


### Bug Fixes

* **robocar-unified:** repair esp-idf-lib symlink broken by monorepo re-org ([1eee6e5](https://github.com/laurigates/mcu-tinkering-lab/commit/1eee6e52dbfa6c262491e1de1f3921f6094f9e5f))

## [0.1.6](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.5...robocar-unified-v0.1.6) (2026-07-16)


### Bug Fixes

* **flasher:** show per-project versions and add missing project cards ([#393](https://github.com/laurigates/mcu-tinkering-lab/issues/393)) ([df12129](https://github.com/laurigates/mcu-tinkering-lab/commit/df1212974b9bae4a391f94a0d0de67ec1d2de87e))

## [0.1.5](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.4...robocar-unified-v0.1.5) (2026-07-14)


### Features

* **schematics:** add obstacle-aware Manhattan auto-router ([#391](https://github.com/laurigates/mcu-tinkering-lab/issues/391)) ([5ec1723](https://github.com/laurigates/mcu-tinkering-lab/commit/5ec1723e7de7be27f2848c85ad6bc4945395fd6c))

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
