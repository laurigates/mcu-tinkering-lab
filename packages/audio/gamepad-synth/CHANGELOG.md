# Changelog

## [0.1.5](https://github.com/laurigates/mcu-tinkering-lab/compare/gamepad-synth-v0.1.4...gamepad-synth-v0.1.5) (2026-07-16)


### Bug Fixes

* **flasher:** show per-project versions and add missing project cards ([#393](https://github.com/laurigates/mcu-tinkering-lab/issues/393)) ([df12129](https://github.com/laurigates/mcu-tinkering-lab/commit/df1212974b9bae4a391f94a0d0de67ec1d2de87e))

## [0.1.4](https://github.com/laurigates/mcu-tinkering-lab/compare/gamepad-synth-v0.1.3...gamepad-synth-v0.1.4) (2026-07-14)


### Features

* **gamepad-synth:** encode lfo_rate as value-scaled blip ladder ([#387](https://github.com/laurigates/mcu-tinkering-lab/issues/387)) ([877c7e4](https://github.com/laurigates/mcu-tinkering-lab/commit/877c7e4674643e8fd8c2a774e1dd0868d39b2aa0)), closes [#267](https://github.com/laurigates/mcu-tinkering-lab/issues/267)

## [0.1.3](https://github.com/laurigates/mcu-tinkering-lab/compare/gamepad-synth-v0.1.2...gamepad-synth-v0.1.3) (2026-07-13)


### Features

* **gamepad-synth:** persist settings to NVS across boots ([#367](https://github.com/laurigates/mcu-tinkering-lab/issues/367)) ([c3af7dc](https://github.com/laurigates/mcu-tinkering-lab/commit/c3af7dcfa9aa85362bf48fa756e6e34b39430736)), closes [#266](https://github.com/laurigates/mcu-tinkering-lab/issues/266)


### Bug Fixes

* **gamepad-synth:** correct fetch-deps xbox-switch-bridge path and use relative symlink ([#366](https://github.com/laurigates/mcu-tinkering-lab/issues/366)) ([a777dde](https://github.com/laurigates/mcu-tinkering-lab/commit/a777dde8082718e3fdbccde1ce878c12fc95cf68)), closes [#269](https://github.com/laurigates/mcu-tinkering-lab/issues/269)

## [0.1.2](https://github.com/laurigates/mcu-tinkering-lab/compare/gamepad-synth-v0.1.1...gamepad-synth-v0.1.2) (2026-07-02)


### Features

* **gamepad-synth:** add delay effect (Phase D) ([#256](https://github.com/laurigates/mcu-tinkering-lab/issues/256)) ([7929a1b](https://github.com/laurigates/mcu-tinkering-lab/commit/7929a1bfb8e9e40c1c923e45ea67eb194667e843))
* **gamepad-synth:** add Dual Osc, Delay Synth, Drone modes (Phase E) ([#257](https://github.com/laurigates/mcu-tinkering-lab/issues/257)) ([90b9fb1](https://github.com/laurigates/mcu-tinkering-lab/commit/90b9fb1c637b06585b5baf31d170c44ca63645ad))
* **gamepad-synth:** add LFO modulation (Phase C) ([#255](https://github.com/laurigates/mcu-tinkering-lab/issues/255)) ([248e0d2](https://github.com/laurigates/mcu-tinkering-lab/commit/248e0d2a1dc457a2d71bd06f1a8d103dd82cd28e))
* **gamepad-synth:** add multi-waveform DDS oscillator (FR-A02) ([#253](https://github.com/laurigates/mcu-tinkering-lab/issues/253)) ([dbc7925](https://github.com/laurigates/mcu-tinkering-lab/commit/dbc79250ccc49c2059e8fae4af56909d1215ac23))
* **gamepad-synth:** add piezo accent voices for Drone mode ([#264](https://github.com/laurigates/mcu-tinkering-lab/issues/264)) ([fde8792](https://github.com/laurigates/mcu-tinkering-lab/commit/fde8792b5847b5db71a95738c94209f4b3ac537d))
* **gamepad-synth:** add resonant low-pass filter (Phase B) ([#254](https://github.com/laurigates/mcu-tinkering-lab/issues/254)) ([d7eae73](https://github.com/laurigates/mcu-tinkering-lab/commit/d7eae7373d921772f4ce7edb8ec3565eb77c81b3))
* **gamepad-synth:** Finnish TTS + new control bindings ([#334](https://github.com/laurigates/mcu-tinkering-lab/issues/334)) ([ae84093](https://github.com/laurigates/mcu-tinkering-lab/commit/ae840934d1438bf3e6356ee71f07d3e8ecf74747))
* **gamepad-synth:** gemini TTS voicing announcements ([#273](https://github.com/laurigates/mcu-tinkering-lab/issues/273)) ([8cea4fc](https://github.com/laurigates/mcu-tinkering-lab/commit/8cea4fc870581bc30f4f272cfb0f7e63fda208aa))
* **gamepad-synth:** octave-shift face buttons in Theremin ([#274](https://github.com/laurigates/mcu-tinkering-lab/issues/274)) ([eae4722](https://github.com/laurigates/mcu-tinkering-lab/commit/eae4722bab23f994d380b5f96052a8039dce22d1))
* **gamepad-synth:** replace LEDC PWM with I2S DAC output (Phase A) ([#252](https://github.com/laurigates/mcu-tinkering-lab/issues/252)) ([c350f9c](https://github.com/laurigates/mcu-tinkering-lab/commit/c350f9cf07939c49d0781ec739b1247a73b06ba6))
* **gamepad-synth:** settings page, drum engine, pitch-bend triggers, confirmation blips ([#270](https://github.com/laurigates/mcu-tinkering-lab/issues/270)) ([a6121ed](https://github.com/laurigates/mcu-tinkering-lab/commit/a6121ed6acfbc1284ecc8394c531195623c27055))
* **gamepad-synth:** unified controls phase 1 + voicing refactor ([#272](https://github.com/laurigates/mcu-tinkering-lab/issues/272)) ([a61b304](https://github.com/laurigates/mcu-tinkering-lab/commit/a61b304cf717136c214d3545abdef153bfce669d))
* **schematics:** scaffold schemdraw schematic library (gamepad-synth) ([#262](https://github.com/laurigates/mcu-tinkering-lab/issues/262)) ([c2402f2](https://github.com/laurigates/mcu-tinkering-lab/commit/c2402f29514070724325c71e8e117c05346d1e8e))


### Bug Fixes

* **auto:** apply clang-format to remaining firmware files ([#172](https://github.com/laurigates/mcu-tinkering-lab/issues/172)) ([251cc9e](https://github.com/laurigates/mcu-tinkering-lab/commit/251cc9ecac21a1db102a43ba5985d0c04631cc79))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **gamepad-synth:** use TPT SVF to prevent filter instability at high cutoff ([#271](https://github.com/laurigates/mcu-tinkering-lab/issues/271)) ([8091e27](https://github.com/laurigates/mcu-tinkering-lab/commit/8091e272d724a40a90a24b51e20be5f8d9936463))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))


### Documentation

* **gamepad-synth:** refresh WIRING.md for v1.0.0 hardware build ([#260](https://github.com/laurigates/mcu-tinkering-lab/issues/260)) ([7c27041](https://github.com/laurigates/mcu-tinkering-lab/commit/7c27041be0b452d17454187683c59a4582a3ee70))


### Miscellaneous

* release ([#359](https://github.com/laurigates/mcu-tinkering-lab/issues/359)) ([303717a](https://github.com/laurigates/mcu-tinkering-lab/commit/303717a6c2b51b7df62b846a31161e879223db9d))

## [0.1.1](https://github.com/laurigates/mcu-tinkering-lab/compare/gamepad-synth-v0.1.0...gamepad-synth-v0.1.1) (2026-07-02)


### Features

* **gamepad-synth:** add delay effect (Phase D) ([#256](https://github.com/laurigates/mcu-tinkering-lab/issues/256)) ([7929a1b](https://github.com/laurigates/mcu-tinkering-lab/commit/7929a1bfb8e9e40c1c923e45ea67eb194667e843))
* **gamepad-synth:** add Dual Osc, Delay Synth, Drone modes (Phase E) ([#257](https://github.com/laurigates/mcu-tinkering-lab/issues/257)) ([90b9fb1](https://github.com/laurigates/mcu-tinkering-lab/commit/90b9fb1c637b06585b5baf31d170c44ca63645ad))
* **gamepad-synth:** add LFO modulation (Phase C) ([#255](https://github.com/laurigates/mcu-tinkering-lab/issues/255)) ([248e0d2](https://github.com/laurigates/mcu-tinkering-lab/commit/248e0d2a1dc457a2d71bd06f1a8d103dd82cd28e))
* **gamepad-synth:** add multi-waveform DDS oscillator (FR-A02) ([#253](https://github.com/laurigates/mcu-tinkering-lab/issues/253)) ([dbc7925](https://github.com/laurigates/mcu-tinkering-lab/commit/dbc79250ccc49c2059e8fae4af56909d1215ac23))
* **gamepad-synth:** add piezo accent voices for Drone mode ([#264](https://github.com/laurigates/mcu-tinkering-lab/issues/264)) ([fde8792](https://github.com/laurigates/mcu-tinkering-lab/commit/fde8792b5847b5db71a95738c94209f4b3ac537d))
* **gamepad-synth:** add resonant low-pass filter (Phase B) ([#254](https://github.com/laurigates/mcu-tinkering-lab/issues/254)) ([d7eae73](https://github.com/laurigates/mcu-tinkering-lab/commit/d7eae7373d921772f4ce7edb8ec3565eb77c81b3))
* **gamepad-synth:** Finnish TTS + new control bindings ([#334](https://github.com/laurigates/mcu-tinkering-lab/issues/334)) ([ae84093](https://github.com/laurigates/mcu-tinkering-lab/commit/ae840934d1438bf3e6356ee71f07d3e8ecf74747))
* **gamepad-synth:** gemini TTS voicing announcements ([#273](https://github.com/laurigates/mcu-tinkering-lab/issues/273)) ([8cea4fc](https://github.com/laurigates/mcu-tinkering-lab/commit/8cea4fc870581bc30f4f272cfb0f7e63fda208aa))
* **gamepad-synth:** octave-shift face buttons in Theremin ([#274](https://github.com/laurigates/mcu-tinkering-lab/issues/274)) ([eae4722](https://github.com/laurigates/mcu-tinkering-lab/commit/eae4722bab23f994d380b5f96052a8039dce22d1))
* **gamepad-synth:** replace LEDC PWM with I2S DAC output (Phase A) ([#252](https://github.com/laurigates/mcu-tinkering-lab/issues/252)) ([c350f9c](https://github.com/laurigates/mcu-tinkering-lab/commit/c350f9cf07939c49d0781ec739b1247a73b06ba6))
* **gamepad-synth:** settings page, drum engine, pitch-bend triggers, confirmation blips ([#270](https://github.com/laurigates/mcu-tinkering-lab/issues/270)) ([a6121ed](https://github.com/laurigates/mcu-tinkering-lab/commit/a6121ed6acfbc1284ecc8394c531195623c27055))
* **gamepad-synth:** unified controls phase 1 + voicing refactor ([#272](https://github.com/laurigates/mcu-tinkering-lab/issues/272)) ([a61b304](https://github.com/laurigates/mcu-tinkering-lab/commit/a61b304cf717136c214d3545abdef153bfce669d))
* **schematics:** scaffold schemdraw schematic library (gamepad-synth) ([#262](https://github.com/laurigates/mcu-tinkering-lab/issues/262)) ([c2402f2](https://github.com/laurigates/mcu-tinkering-lab/commit/c2402f29514070724325c71e8e117c05346d1e8e))


### Bug Fixes

* **auto:** apply clang-format to remaining firmware files ([#172](https://github.com/laurigates/mcu-tinkering-lab/issues/172)) ([251cc9e](https://github.com/laurigates/mcu-tinkering-lab/commit/251cc9ecac21a1db102a43ba5985d0c04631cc79))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **gamepad-synth:** use TPT SVF to prevent filter instability at high cutoff ([#271](https://github.com/laurigates/mcu-tinkering-lab/issues/271)) ([8091e27](https://github.com/laurigates/mcu-tinkering-lab/commit/8091e272d724a40a90a24b51e20be5f8d9936463))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))


### Documentation

* **gamepad-synth:** refresh WIRING.md for v1.0.0 hardware build ([#260](https://github.com/laurigates/mcu-tinkering-lab/issues/260)) ([7c27041](https://github.com/laurigates/mcu-tinkering-lab/commit/7c27041be0b452d17454187683c59a4582a3ee70))
