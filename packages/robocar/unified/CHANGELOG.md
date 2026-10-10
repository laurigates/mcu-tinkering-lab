# Changelog

## [0.2.10](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.9...robocar-unified-v0.2.10) (2026-10-10)


### Features

* **hardware:** add the board × header × parts join, re-point generate-pin-defs ([#636](https://github.com/laurigates/mcu-tinkering-lab/issues/636)) ([a68dd55](https://github.com/laurigates/mcu-tinkering-lab/commit/a68dd5551a630ce5e3ac1c2932d91755d045591a))
* **hardware:** generate the WIRING.md pin tables from the join ([#647](https://github.com/laurigates/mcu-tinkering-lab/issues/647)) ([f540f93](https://github.com/laurigates/mcu-tinkering-lab/commit/f540f938e65d57b41ca73296eba0011478925179))
* **hardware:** generate WIRING.md's PCA9685 channel map from [[channel_nets]] ([fe517e7](https://github.com/laurigates/mcu-tinkering-lab/commit/fe517e7f86ba419bd1d9d8ff2b8c5315e4d3e1f9)), closes [#694](https://github.com/laurigates/mcu-tinkering-lab/issues/694)
* **hardware:** generate WIRING.md's PCA9685 channel map from channel_nets ([#717](https://github.com/laurigates/mcu-tinkering-lab/issues/717)) ([fe517e7](https://github.com/laurigates/mcu-tinkering-lab/commit/fe517e7f86ba419bd1d9d8ff2b8c5315e4d3e1f9))
* **hardware:** model PCA9685 channel nets in the join for pinout labels ([#695](https://github.com/laurigates/mcu-tinkering-lab/issues/695)) ([9aa9135](https://github.com/laurigates/mcu-tinkering-lab/commit/9aa91356ba634546f21fad443e6203c3173b0661))
* **hardware:** model power rails in the join and generate the WIRING power diagram ([#680](https://github.com/laurigates/mcu-tinkering-lab/issues/680)) ([214bfc7](https://github.com/laurigates/mcu-tinkering-lab/commit/214bfc7ca7d70d18ca3e7e82be4323fe1a8717c1))
* **robocar-unified:** add a workstation probe for the Gemini Live API ([#635](https://github.com/laurigates/mcu-tinkering-lab/issues/635)) ([9e588ca](https://github.com/laurigates/mcu-tinkering-lab/commit/9e588caeb5994d3db971b941b184daa9a2a0dd29))
* **robocar-unified:** boot with the voice effect disabled ([#615](https://github.com/laurigates/mcu-tinkering-lab/issues/615)) ([bd41850](https://github.com/laurigates/mcu-tinkering-lab/commit/bd4185021232f7f77c422e4c65af571447f38da5))
* **robocar-unified:** lock MQTT commands to read-only without broker credentials ([#638](https://github.com/laurigates/mcu-tinkering-lab/issues/638)) ([4f9fdcf](https://github.com/laurigates/mcu-tinkering-lab/commit/4f9fdcffb4f0de3f79f2f6898b88b37eb4dbc653))
* **robocar-unified:** ration hands-free voice turns and cap their spend ([#659](https://github.com/laurigates/mcu-tinkering-lab/issues/659)) ([00b60d0](https://github.com/laurigates/mcu-tinkering-lab/commit/00b60d0e0ba68a221a4c21e2aa7c433b0218403f))
* **robocar-unified:** trigger hands-free listening on speech, not loudness ([#622](https://github.com/laurigates/mcu-tinkering-lab/issues/622)) ([20c7217](https://github.com/laurigates/mcu-tinkering-lab/commit/20c721757bb107d017126ff14b4f2e52ee891028))
* **schematics:** draw robocar-unified's boards with their physical pin layout ([#661](https://github.com/laurigates/mcu-tinkering-lab/issues/661)) ([73cf1df](https://github.com/laurigates/mcu-tinkering-lab/commit/73cf1df1fb9b9db91ac3d4f21dbf503a5b4311ec))
* **schematics:** draw suggested bulk and decoupling capacitors for robocar-unified ([#665](https://github.com/laurigates/mcu-tinkering-lab/issues/665)) ([5acc005](https://github.com/laurigates/mcu-tinkering-lab/commit/5acc005fac6933b33319df8a02e504da881432cc))
* **schematics:** take robocar-unified's MCU wiring and labels from the hardware join ([#663](https://github.com/laurigates/mcu-tinkering-lab/issues/663)) ([68f4728](https://github.com/laurigates/mcu-tinkering-lab/commit/68f47282880353f994633d0748204962381f4c42))


### Bug Fixes

* **hardware:** reject a rail on an MCU signal pad and a Mermaid-keyword part id ([#687](https://github.com/laurigates/mcu-tinkering-lab/issues/687)) ([a49f907](https://github.com/laurigates/mcu-tinkering-lab/commit/a49f90749b85a50c878ee1b4d8e7b45c70a01a74))
* **robocar-unified:** build the voice-turn request body with one copy of the clip ([#660](https://github.com/laurigates/mcu-tinkering-lab/issues/660)) ([ca56cba](https://github.com/laurigates/mcu-tinkering-lab/commit/ca56cba544067a13476cb6bc2f8691490b59ae93))
* **robocar-unified:** carry the build commit SHA in the OTA manifest ([#656](https://github.com/laurigates/mcu-tinkering-lab/issues/656)) ([9027323](https://github.com/laurigates/mcu-tinkering-lab/commit/90273231bf3b9a7647d863a919c75f4e85aac755)), closes [#627](https://github.com/laurigates/mcu-tinkering-lab/issues/627)
* **robocar-unified:** fail the scene gate closed until a frame decodes ([#700](https://github.com/laurigates/mcu-tinkering-lab/issues/700)) ([9a43e29](https://github.com/laurigates/mcu-tinkering-lab/commit/9a43e295d0bb7ec641cbe6beb6419a652ad83aec))
* **robocar-unified:** keep the triggering speech in a VAD voice turn ([#621](https://github.com/laurigates/mcu-tinkering-lab/issues/621)) ([ed22bdc](https://github.com/laurigates/mcu-tinkering-lab/commit/ed22bdcc5848778b883a86e1a5568087c0e4eddd))
* **robocar-unified:** keep the voice-turn start beep out of the ambient gate ([#652](https://github.com/laurigates/mcu-tinkering-lab/issues/652)) ([80132d8](https://github.com/laurigates/mcu-tinkering-lab/commit/80132d870ca949a98827ab8c55d8c6559cffcae0))
* **robocar-unified:** say "since you last spoke" only for a sense that compared ([#691](https://github.com/laurigates/mcu-tinkering-lab/issues/691)) ([77a1917](https://github.com/laurigates/mcu-tinkering-lab/commit/77a19174ae1f92126dd6556c51f108a24e9d823c))
* **robocar-unified:** send Improv replies over the USB console, not UART0 ([#672](https://github.com/laurigates/mcu-tinkering-lab/issues/672)) ([7712a59](https://github.com/laurigates/mcu-tinkering-lab/commit/7712a5995e85a3f4358e238631d4fd59816943c5))
* **robocar-unified:** stop the planner inventing sounds on audio-only openings ([#620](https://github.com/laurigates/mcu-tinkering-lab/issues/620)) ([911fc56](https://github.com/laurigates/mcu-tinkering-lab/commit/911fc5691eee27b1f832dc21073a7c26450eb15e))
* **schematics:** keep routed wires off power and ground tags ([#642](https://github.com/laurigates/mcu-tinkering-lab/issues/642)) ([a9c39bd](https://github.com/laurigates/mcu-tinkering-lab/commit/a9c39bde444db4d7db7a81cfec18cfc5186e2f22))
* **schematics:** keep routed wires out of text labels ([#676](https://github.com/laurigates/mcu-tinkering-lab/issues/676)) ([e7558fa](https://github.com/laurigates/mcu-tinkering-lab/commit/e7558fa8c0b50c59d0e7068312b79c5963b1bc5b)), closes [#641](https://github.com/laurigates/mcu-tinkering-lab/issues/641)


### Documentation

* **robocar-unified:** count D6/D7 as spare in the GPIO budget callouts ([#668](https://github.com/laurigates/mcu-tinkering-lab/issues/668)) ([d7a0328](https://github.com/laurigates/mcu-tinkering-lab/commit/d7a03280630efe118c900b1c283814786ccd4ff5)), closes [#645](https://github.com/laurigates/mcu-tinkering-lab/issues/645)
* **robocar-unified:** generate board pinout images for the build guide ([#667](https://github.com/laurigates/mcu-tinkering-lab/issues/667)) ([2902f3a](https://github.com/laurigates/mcu-tinkering-lab/commit/2902f3acfe9fbf495b20c21a30ddc2dab722cb18)), closes [#629](https://github.com/laurigates/mcu-tinkering-lab/issues/629)
* **robocar-unified:** put TB6612FNG and PCA9685 VCC on 3.3 V in the build guide ([#673](https://github.com/laurigates/mcu-tinkering-lab/issues/673)) ([7e388af](https://github.com/laurigates/mcu-tinkering-lab/commit/7e388afc21aaf1db0489479dd7263fe81f0bbfc6)), closes [#664](https://github.com/laurigates/mcu-tinkering-lab/issues/664)
* **schematics:** draw the onboard PDM microphone on robocar-unified ([#658](https://github.com/laurigates/mcu-tinkering-lab/issues/658)) ([0b3bf40](https://github.com/laurigates/mcu-tinkering-lab/commit/0b3bf408c814086dc322974ebbf9bea0ed8d2758))

## [0.2.9](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.8...robocar-unified-v0.2.9) (2026-09-26)


### Bug Fixes

* **robocar-unified:** expire the ambient latch in the score getters ([#599](https://github.com/laurigates/mcu-tinkering-lab/issues/599)) ([55dac27](https://github.com/laurigates/mcu-tinkering-lab/commit/55dac2738f8e01a15c052c7cf3020a28d60b9d46))


### Documentation

* **claude:** stop hard-coding the robocar-unified suite count ([#602](https://github.com/laurigates/mcu-tinkering-lab/issues/602)) ([b458905](https://github.com/laurigates/mcu-tinkering-lab/commit/b458905ed9f0a7e0df367f54fbbce84da0110e39))

## [0.2.8](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.7...robocar-unified-v0.2.8) (2026-09-24)


### Documentation

* **robocar-unified:** recompile build guide for the re-rendered schematic ([7bf5ec4](https://github.com/laurigates/mcu-tinkering-lab/commit/7bf5ec4b76ca4d0f81e50317b8ad313277370281))

## [0.2.7](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.6...robocar-unified-v0.2.7) (2026-09-22)


### Features

* **robocar-unified:** add host-side voice auditioning tools ([e4959c4](https://github.com/laurigates/mcu-tinkering-lab/commit/e4959c4f6db84aeaf3fda796eeb1e2eb01451622))
* **robocar-unified:** give Teuvo a metal body and the Schedar voice ([306b335](https://github.com/laurigates/mcu-tinkering-lab/commit/306b335b61c654fadd14aa881488bb1d3b15d036))

## [0.2.6](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.5...robocar-unified-v0.2.6) (2026-09-18)


### Bug Fixes

* **robocar:** the servo buzz is the PCA9685 frame rate — 200 Hz → 100 Hz, measured with a repaired bringup sweep ([#580](https://github.com/laurigates/mcu-tinkering-lab/issues/580)) ([6492e04](https://github.com/laurigates/mcu-tinkering-lab/commit/6492e0429c758b7b5763ed5a7a3b161b70941459))

## [0.2.5](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.4...robocar-unified-v0.2.5) (2026-09-15)


### Features

* **robocar-unified:** aim the pan/tilt head from the reactive executor ([#578](https://github.com/laurigates/mcu-tinkering-lab/issues/578)) ([5e38916](https://github.com/laurigates/mcu-tinkering-lab/commit/5e38916adfcc8eb1d0829b6527885b8212a9aaee))
* **robocar-unified:** let credentials.h override the MQTT broker URI ([#572](https://github.com/laurigates/mcu-tinkering-lab/issues/572)) ([ac338f3](https://github.com/laurigates/mcu-tinkering-lab/commit/ac338f3169068c8d646d8a746891c6db183af97f))


### Bug Fixes

* **robocar-unified:** allocate mbedTLS buffers in PSRAM ([#569](https://github.com/laurigates/mcu-tinkering-lab/issues/569)) ([cc31e4f](https://github.com/laurigates/mcu-tinkering-lab/commit/cc31e4f1f11302b8d32e105381aa1c08c988591c))
* **robocar-unified:** keep servos off their end stops ([#574](https://github.com/laurigates/mcu-tinkering-lab/issues/574)) ([d1520c2](https://github.com/laurigates/mcu-tinkering-lab/commit/d1520c26d087d7574ac4a623023671d89db176f5))


### Documentation

* **robocar-unified:** interpolate planner cadence into the build guide ([#577](https://github.com/laurigates/mcu-tinkering-lab/issues/577)) ([2f413d1](https://github.com/laurigates/mcu-tinkering-lab/commit/2f413d13ebea33252b67bb0b0b95fc205f58aa8f)), closes [#485](https://github.com/laurigates/mcu-tinkering-lab/issues/485)

## [0.2.4](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.3...robocar-unified-v0.2.4) (2026-09-14)


### Features

* **robocar-unified:** conversational entity Teuvo with multi-turn multimodal voice and VAD ([#567](https://github.com/laurigates/mcu-tinkering-lab/issues/567)) ([dff0f66](https://github.com/laurigates/mcu-tinkering-lab/commit/dff0f66348b8138a391b86fad7176b9c9e10d655))
* **robocar-unified:** digital gain and peak normalisation for mic audio ([#566](https://github.com/laurigates/mcu-tinkering-lab/issues/566)) ([a63aad3](https://github.com/laurigates/mcu-tinkering-lab/commit/a63aad330fe522515b26018c06cee0d04f691f59)), closes [#561](https://github.com/laurigates/mcu-tinkering-lab/issues/561)

## [0.2.3](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.2...robocar-unified-v0.2.3) (2026-09-13)


### Bug Fixes

* **ci:** decouple OTA asset names from web-flasher names, per-project MQTT notify ([#558](https://github.com/laurigates/mcu-tinkering-lab/issues/558)) ([752df04](https://github.com/laurigates/mcu-tinkering-lab/commit/752df046e1a71e7fc3128443f0229ecf72b2380c))
* **robocar-unified:** implement MQTT remote-command handler ([#554](https://github.com/laurigates/mcu-tinkering-lab/issues/554)) ([1571d62](https://github.com/laurigates/mcu-tinkering-lab/commit/1571d621f6db9259d7547f10374e1a9184b6786b)), closes [#524](https://github.com/laurigates/mcu-tinkering-lab/issues/524)
* **robocar-unified:** poll the web-flasher manifest for OTA instead of esp_ghota ([#559](https://github.com/laurigates/mcu-tinkering-lab/issues/559)) ([915d5f1](https://github.com/laurigates/mcu-tinkering-lab/commit/915d5f1cecd07c9273f2f733e6c872f574bbb210))
* **robocar-unified:** use the new I2C driver for camera SCCB ([#548](https://github.com/laurigates/mcu-tinkering-lab/issues/548)) ([9c1f57a](https://github.com/laurigates/mcu-tinkering-lab/commit/9c1f57a00c0e237dfe67fc2c16cb8029fac27476))


### Miscellaneous

* **robocar-unified:** stop clangd faulting on the containerized compile db ([#553](https://github.com/laurigates/mcu-tinkering-lab/issues/553)) ([947246f](https://github.com/laurigates/mcu-tinkering-lab/commit/947246f9ef75a4c8f45db5650f10630c8e1a86a2)), closes [#536](https://github.com/laurigates/mcu-tinkering-lab/issues/536)

## [0.2.2](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.1...robocar-unified-v0.2.2) (2026-09-11)


### Bug Fixes

* **robocar-unified:** brake on the reflex, pin the bus clock, stagger PWM phase ([20a2838](https://github.com/laurigates/mcu-tinkering-lab/commit/20a28385a4b1a7df89cb9bc57d4c571834c3000e))


### Documentation

* **robocar-unified:** the TB6612FNG and PCA9685 logic rails are 3.3 V, not 5 V ([6029765](https://github.com/laurigates/mcu-tinkering-lab/commit/6029765116e38d37c336652cac105083152ac159))


### Miscellaneous

* **robocar:** migrate esp-idf-lib from a vendored snapshot to managed components ([#529](https://github.com/laurigates/mcu-tinkering-lab/issues/529)) ([8b3b9dd](https://github.com/laurigates/mcu-tinkering-lab/commit/8b3b9dd0d958bba82c134d76bbb5f8f13a0798aa))

## [0.2.1](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.2.0...robocar-unified-v0.2.1) (2026-09-10)


### Features

* **robocar-unified:** halve the speaker volume, and make it a console knob ([#519](https://github.com/laurigates/mcu-tinkering-lab/issues/519)) ([c4a5ea3](https://github.com/laurigates/mcu-tinkering-lab/commit/c4a5ea3c3a3b4b5738a3c01cd8237309e2a22ef7))


### Bug Fixes

* **robocar-unified:** index batched channel writes by name, and gate the doc equivalent ([#517](https://github.com/laurigates/mcu-tinkering-lab/issues/517)) ([f0dbcec](https://github.com/laurigates/mcu-tinkering-lab/commit/f0dbcec6bac9f4694e4939892266edfc8cd70e9d))

## [0.2.0](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.21...robocar-unified-v0.2.0) (2026-09-09)


### ⚠ BREAKING CHANGES

* **robocar-unified:** number the motor channels in the driver's pin order, and add a printable wiring card ([#514](https://github.com/laurigates/mcu-tinkering-lab/issues/514))

### refactor

* **robocar-unified:** number the motor channels in the driver's pin order, and add a printable wiring card ([#514](https://github.com/laurigates/mcu-tinkering-lab/issues/514)) ([74eba69](https://github.com/laurigates/mcu-tinkering-lab/commit/74eba69ac8c21bb2c2c6c236ef5affdcf46f9b10))


### Features

* **robocar-unified:** servo bring-up gesture, live PWM frequency, frequency-aware pulse maths ([#509](https://github.com/laurigates/mcu-tinkering-lab/issues/509)) ([700bfce](https://github.com/laurigates/mcu-tinkering-lab/commit/700bfcecd2e8bd1ecf2f103ada943c979f76f231))


### Bug Fixes

* **robocar-unified:** correct the camera mounting orientation ([#508](https://github.com/laurigates/mcu-tinkering-lab/issues/508)) ([475df58](https://github.com/laurigates/mcu-tinkering-lab/commit/475df58181bccafa686fd7ba2305c3664f0bac9b))
* **robocar-unified:** LED indicator refresh and I2C bus activity counters ([#505](https://github.com/laurigates/mcu-tinkering-lab/issues/505), [#506](https://github.com/laurigates/mcu-tinkering-lab/issues/506)) ([77fe57b](https://github.com/laurigates/mcu-tinkering-lab/commit/77fe57b9d878d5e403564229e56a5b633dc4491d))
* **robocar-unified:** stop re-writing an unchanged motor state 30 times a second ([#504](https://github.com/laurigates/mcu-tinkering-lab/issues/504)) ([674a03d](https://github.com/laurigates/mcu-tinkering-lab/commit/674a03d6353d3d3b4d0409aa813cb3916f51806e))


### Documentation

* **robocar-unified:** correct the power supply to an LM2596 buck, 2S pack ([#510](https://github.com/laurigates/mcu-tinkering-lab/issues/510)) ([8b4cd0f](https://github.com/laurigates/mcu-tinkering-lab/commit/8b4cd0f240cd906cc9fef301ca962eaf5e439ead))

## [0.1.21](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.20...robocar-unified-v0.1.21) (2026-09-05)


### Bug Fixes

* **robocar-unified:** make the hardware phase report failures instead of rebooting ([#503](https://github.com/laurigates/mcu-tinkering-lab/issues/503)) ([96614f3](https://github.com/laurigates/mcu-tinkering-lab/commit/96614f334ccdaf61b4375e157130e5925666bef6)), closes [#500](https://github.com/laurigates/mcu-tinkering-lab/issues/500)
* **robocar-unified:** migrate the planner to gemini-robotics-er-2-preview ([#499](https://github.com/laurigates/mcu-tinkering-lab/issues/499)) ([880841f](https://github.com/laurigates/mcu-tinkering-lab/commit/880841f0133d736f84d11ac8ff4fbab164d50690))
* **robocar-unified:** two hardware-phase init bugs exposed by the first boot with the PCA9685 fitted ([#498](https://github.com/laurigates/mcu-tinkering-lab/issues/498)) ([6a6f94e](https://github.com/laurigates/mcu-tinkering-lab/commit/6a6f94e3c71eb785ba2c330e2f4482f62a5e2baa))

## [0.1.20](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.19...robocar-unified-v0.1.20) (2026-08-29)


### Features

* **robocar-unified:** gate the planner request on evidence and a spend ceiling ([#480](https://github.com/laurigates/mcu-tinkering-lab/issues/480)) ([bff01cd](https://github.com/laurigates/mcu-tinkering-lab/commit/bff01cd17a35479d41378b389ff7a1f8283e93f1))


### Documentation

* **robocar-unified:** correct four stale facts and retire the duplicated diagrams ([#483](https://github.com/laurigates/mcu-tinkering-lab/issues/483)) ([18099f1](https://github.com/laurigates/mcu-tinkering-lab/commit/18099f19f70a852581a926cd0bd427088c197b82))

## [0.1.19](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.18...robocar-unified-v0.1.19) (2026-08-26)


### Bug Fixes

* **robocar-unified:** stop the playback ring tally drifting and muting the mic ([#474](https://github.com/laurigates/mcu-tinkering-lab/issues/474)) ([480fc2b](https://github.com/laurigates/mcu-tinkering-lab/commit/480fc2b3dacac5c22ad04e807c320d8147b63355))

## [0.1.18](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.17...robocar-unified-v0.1.18) (2026-08-22)


### Features

* **robocar-unified:** indicate camera captures and endpoint calls ([fc234ef](https://github.com/laurigates/mcu-tinkering-lab/commit/fc234ef964a554069a374306d832b58a7e8c661f))


### Bug Fixes

* **build-guide:** drop the firmware version from the generated guide ([#470](https://github.com/laurigates/mcu-tinkering-lab/issues/470)) ([723c32d](https://github.com/laurigates/mcu-tinkering-lab/commit/723c32d3e23f2c30241e13f085d716ea5987a001)), closes [#439](https://github.com/laurigates/mcu-tinkering-lab/issues/439)
* **robocar-unified:** fail the ambient gate closed when the mic never speaks ([fc63990](https://github.com/laurigates/mcu-tinkering-lab/commit/fc639909ed9820bd34ce2fcd889a4cb487a50a2b))


### Documentation

* **robocar-unified:** resync build guide to 0.1.17 ([#465](https://github.com/laurigates/mcu-tinkering-lab/issues/465)) ([a339111](https://github.com/laurigates/mcu-tinkering-lab/commit/a339111390b3410e46969787d2d938e67586eef7))


### Miscellaneous

* drop cargo-culted 'set positional-arguments' from justfiles ([998a9c4](https://github.com/laurigates/mcu-tinkering-lab/commit/998a9c479095bb2018e11b37af512d793dc17fa3)), closes [#410](https://github.com/laurigates/mcu-tinkering-lab/issues/410)

## [0.1.17](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.16...robocar-unified-v0.1.17) (2026-07-31)


### Features

* **robocar-unified:** let someone talk to the robot and get an answer ([#454](https://github.com/laurigates/mcu-tinkering-lab/issues/454)) ([39c1571](https://github.com/laurigates/mcu-tinkering-lab/commit/39c1571ee67fc83195eacb7700f10e6708c38bdb))
* **robocar-unified:** let the robot hear the room, and speak when it changes ([#453](https://github.com/laurigates/mcu-tinkering-lab/issues/453)) ([c017c2a](https://github.com/laurigates/mcu-tinkering-lab/commit/c017c2a9c52ed7eef58f97879dac05b91cd2f170))

## [0.1.16](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.15...robocar-unified-v0.1.16) (2026-07-30)


### Features

* **robocar-unified:** give the robot a memory of what it said, and ration how often it speaks ([#449](https://github.com/laurigates/mcu-tinkering-lab/issues/449)) ([731fd6f](https://github.com/laurigates/mcu-tinkering-lab/commit/731fd6f3aca9ecf8050844e3d1e1653f4c175303))
* **robocar-unified:** only offer the speak tool when the view has actually changed ([#452](https://github.com/laurigates/mcu-tinkering-lab/issues/452)) ([b510061](https://github.com/laurigates/mcu-tinkering-lab/commit/b5100616435090985f2ea471d90672dd23d531ad))

## [0.1.15](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.14...robocar-unified-v0.1.15) (2026-07-28)


### Features

* **robocar-unified:** stream TTS audio and add inline delivery tags ([#442](https://github.com/laurigates/mcu-tinkering-lab/issues/442)) ([706271a](https://github.com/laurigates/mcu-tinkering-lab/commit/706271aa34db46c838cb4e3ea66755b0573b493f))


### Bug Fixes

* **robocar-unified:** stop TTS audio tearing and make camera frames measurable ([#446](https://github.com/laurigates/mcu-tinkering-lab/issues/446)) ([d1ad55d](https://github.com/laurigates/mcu-tinkering-lab/commit/d1ad55d1389e50b03dbee954357df8471b38f84d))


### Miscellaneous

* **robocar-unified:** add a just recipe for the host unit tests ([#445](https://github.com/laurigates/mcu-tinkering-lab/issues/445)) ([85f5ed5](https://github.com/laurigates/mcu-tinkering-lab/commit/85f5ed547f5fc9b860678500dc1d2ab04bb89c84))

## [0.1.14](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.13...robocar-unified-v0.1.14) (2026-07-26)


### Documentation

* **robocar-unified:** refresh build guide for 0.1.13 ([#440](https://github.com/laurigates/mcu-tinkering-lab/issues/440)) ([ff47d2a](https://github.com/laurigates/mcu-tinkering-lab/commit/ff47d2a26b54e4eff2472c8de056dcd9a21f3ffb))

## [0.1.13](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.12...robocar-unified-v0.1.13) (2026-07-26)


### Features

* **robocar-unified:** generate build-guide pin data from pin_config.h ([#434](https://github.com/laurigates/mcu-tinkering-lab/issues/434)) ([310edda](https://github.com/laurigates/mcu-tinkering-lab/commit/310edda534bef512a21a21631d7897b2896fcbbc))
* **robocar-unified:** vary spoken dialogue per utterance ([#432](https://github.com/laurigates/mcu-tinkering-lab/issues/432)) ([e25d53a](https://github.com/laurigates/mcu-tinkering-lab/commit/e25d53a3e09a295e047c23d16f1b6d6b8a6c65f8))

## [0.1.12](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-unified-v0.1.11...robocar-unified-v0.1.12) (2026-07-25)


### Features

* **robocar-unified:** working Finnish voice — audio-static, Gemini-API, rate-limit fixes + voice personas ([#428](https://github.com/laurigates/mcu-tinkering-lab/issues/428)) ([35c07dc](https://github.com/laurigates/mcu-tinkering-lab/commit/35c07dc7f8ca0cc6d321772416f01fd0494ccddb))


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
