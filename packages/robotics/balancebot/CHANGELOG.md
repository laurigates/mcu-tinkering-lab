# Changelog

## [0.1.18](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.17...balancebot-v0.1.18) (2026-11-27)


### Miscellaneous

* **balancebot:** re-route the carrier with the fixed DSN code, KiCad 10.0.6 stamp ([#1303](https://github.com/laurigates/mcu-tinkering-lab/issues/1303)) ([73ac910](https://github.com/laurigates/mcu-tinkering-lab/commit/73ac9108d21ddc4fb4064a6087a255940b17a25a))
* **balancebot:** recompile the perfboard step guide PDF for the new label placement ([#762](https://github.com/laurigates/mcu-tinkering-lab/issues/762)) ([51a6812](https://github.com/laurigates/mcu-tinkering-lab/commit/51a6812777b3db29de05d0b0d8c83653d6b767a7))
* **balancebot:** regenerate the perfboard drawing with the new label placement ([#762](https://github.com/laurigates/mcu-tinkering-lab/issues/762)) ([56fd0c4](https://github.com/laurigates/mcu-tinkering-lab/commit/56fd0c44243b4066d2a2d5e02f7d499739d2b584))
* **balancebot:** set the carrier's min_resolved_spokes to the rule's 1 ([#1302](https://github.com/laurigates/mcu-tinkering-lab/issues/1302)) ([897fc20](https://github.com/laurigates/mcu-tinkering-lab/commit/897fc2053370b734f03f1eef45d41ef8b3a7b60f))

## [0.1.17](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.16...balancebot-v0.1.17) (2026-11-25)


### Features

* **balancebot:** analyze and lqr-sweep tuning recipes ([5f3202d](https://github.com/laurigates/mcu-tinkering-lab/commit/5f3202dbfa4bad932fbc3916c5eebfd0db79fc73))
* **balancebot:** drive and turn while balancing ([#1257](https://github.com/laurigates/mcu-tinkering-lab/issues/1257)) ([5b1615e](https://github.com/laurigates/mcu-tinkering-lab/commit/5b1615ef945f4392ed714aead629a2d25621911e))
* **balancebot:** lower the tilt cutoff from 45 to 30 degrees ([a2e1f06](https://github.com/laurigates/mcu-tinkering-lab/commit/a2e1f0637c7aee2e943b044ae300f8dc47ffb940)), closes [#1278](https://github.com/laurigates/mcu-tinkering-lab/issues/1278)
* **balancebot:** record each run untethered and save it to flash ([#1266](https://github.com/laurigates/mcu-tinkering-lab/issues/1266)) ([a4ae3fe](https://github.com/laurigates/mcu-tinkering-lab/commit/a4ae3fe97894728754ba0e7959c316caa591a836)), closes [#1262](https://github.com/laurigates/mcu-tinkering-lab/issues/1262)
* **balancebot:** RGB LED guides the hand to the auto-arm window ([#1248](https://github.com/laurigates/mcu-tinkering-lab/issues/1248)) ([6c292cb](https://github.com/laurigates/mcu-tinkering-lab/commit/6c292cbc9657bc5d324d3305a24ee2e1478b5f4c))
* **balancebot:** show firmware, gains, maxrate and model in stat ([efed044](https://github.com/laurigates/mcu-tinkering-lab/commit/efed044fa3b244de1fd7507c72c4531a9d91af93))
* **balancebot:** widen the auto-arm window to +/-3 deg ([02036d7](https://github.com/laurigates/mcu-tinkering-lab/commit/02036d78f45401f1d0fe73812cbb205679585e57))
* **balancebot:** widen the auto-arm window to ±3° ([#1255](https://github.com/laurigates/mcu-tinkering-lab/issues/1255)) ([02036d7](https://github.com/laurigates/mcu-tinkering-lab/commit/02036d78f45401f1d0fe73812cbb205679585e57))
* **carriers:** ADR-038 power design: pack cutoff firmware, switch and latch, 2S USB charger ([#1282](https://github.com/laurigates/mcu-tinkering-lab/issues/1282)) ([1632776](https://github.com/laurigates/mcu-tinkering-lab/commit/1632776a1cdd2adc9870987cdde6ba7127183354))


### Bug Fixes

* **balancebot:** alpha default for 500 Hz, and core 1 'starting' during boot ([#1244](https://github.com/laurigates/mcu-tinkering-lab/issues/1244)) ([2c2c749](https://github.com/laurigates/mcu-tinkering-lab/commit/2c2c7499337a199a91e897ae4690e63ed6650c4f))
* **balancebot:** bound IMU I2C transfers and free a held bus ([e586197](https://github.com/laurigates/mcu-tinkering-lab/commit/e58619758518874f576e12b21451fb47b531f266))
* **balancebot:** drop the second reboot from the flash recipe ([#1267](https://github.com/laurigates/mcu-tinkering-lab/issues/1267)) ([8311690](https://github.com/laurigates/mcu-tinkering-lab/commit/8311690fb126d4f930f8fe69c657f7e9afad7fb4))
* **balancebot:** give the accel pitch the gyro's forward-positive sign ([#1242](https://github.com/laurigates/mcu-tinkering-lab/issues/1242)) ([6182c13](https://github.com/laurigates/mcu-tinkering-lab/commit/6182c13024fbe676834846e2242af3f6243e6f35))
* **balancebot:** hold off auto-arm after disarm and trim capture; report save ([#1243](https://github.com/laurigates/mcu-tinkering-lab/issues/1243)) ([dc6c21b](https://github.com/laurigates/mcu-tinkering-lab/commit/dc6c21b073fd968f9e17bc33eb8bead01330e7de))
* **balancebot:** let the PID velocity integral reach its lean limit ([f245f42](https://github.com/laurigates/mcu-tinkering-lab/commit/f245f4201800b372fe5f3fc198f64d2bc55d4229)), closes [#1258](https://github.com/laurigates/mcu-tinkering-lab/issues/1258)
* **balancebot:** refuse rate and cal outside IDLE, and say what load found ([8fe247a](https://github.com/laurigates/mcu-tinkering-lab/commit/8fe247aa31f0f4d1324a15008205889e68051990)), closes [#1128](https://github.com/laurigates/mcu-tinkering-lab/issues/1128)


### Documentation

* **balancebot:** publish the balancebot story on GitHub Pages ([#1263](https://github.com/laurigates/mcu-tinkering-lab/issues/1263)) ([31ee2f7](https://github.com/laurigates/mcu-tinkering-lab/commit/31ee2f7f75116691bb7c0e0f407c54f6a06dc574))
* **balancebot:** reach BOOTSEL without the board's buttons ([b405f11](https://github.com/laurigates/mcu-tinkering-lab/commit/b405f114c18de069b1ff759d86f5ec6d66610f36))
* **balancebot:** the tuning loop, choosing r, and bring-up signs ([#1246](https://github.com/laurigates/mcu-tinkering-lab/issues/1246)) ([303d8c1](https://github.com/laurigates/mcu-tinkering-lab/commit/303d8c10a546e9462f8b547bd94e621559ec61ff))
* **skills:** balancebot-tuning skill, and the filter and test-model lessons ([#1247](https://github.com/laurigates/mcu-tinkering-lab/issues/1247)) ([47c727f](https://github.com/laurigates/mcu-tinkering-lab/commit/47c727fb1436f1ac1b96ed872322a22b816cd576))


### Miscellaneous

* **balancebot:** analyze and lqr-sweep tuning recipes ([#1245](https://github.com/laurigates/mcu-tinkering-lab/issues/1245)) ([5f3202d](https://github.com/laurigates/mcu-tinkering-lab/commit/5f3202dbfa4bad932fbc3916c5eebfd0db79fc73))

## [0.1.16](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.15...balancebot-v0.1.16) (2026-11-23)


### Features

* **balancebot:** add a saved pitch trim ([#1232](https://github.com/laurigates/mcu-tinkering-lab/issues/1232)) ([9d3e023](https://github.com/laurigates/mcu-tinkering-lab/commit/9d3e023730999a0d78b783b386f62b469f887304))
* **balancebot:** capture and fit-leff recipes ([#1234](https://github.com/laurigates/mcu-tinkering-lab/issues/1234)) ([113b125](https://github.com/laurigates/mcu-tinkering-lab/commit/113b12509cae36c930cd227a3fad325bc2da09c9))
* **balancebot:** trim capture and a stillness check on the gyro calibration ([#1233](https://github.com/laurigates/mcu-tinkering-lab/issues/1233)) ([d8236dc](https://github.com/laurigates/mcu-tinkering-lab/commit/d8236dc4626578561bf8a5873d8a372fcccbc84f))


### Miscellaneous

* **balancebot:** add cli, arm, disarm and stat recipes ([#1229](https://github.com/laurigates/mcu-tinkering-lab/issues/1229)) ([d51a6cf](https://github.com/laurigates/mcu-tinkering-lab/commit/d51a6cf1954a67e894453043c3ce18b232e6512f))

## [0.1.15](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.14...balancebot-v0.1.15) (2026-11-20)


### Documentation

* **balancebot:** add BRINGUP.md for the buck, straps and Vref ([#1225](https://github.com/laurigates/mcu-tinkering-lab/issues/1225)) ([1684a2a](https://github.com/laurigates/mcu-tinkering-lab/commit/1684a2af386a09bf63b6fbb9f9be38ece3df5529))

## [0.1.14](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.13...balancebot-v0.1.14) (2026-11-12)


### Bug Fixes

* **balancebot:** close the CLI arm/mount/load gaps left after [#1041](https://github.com/laurigates/mcu-tinkering-lab/issues/1041) ([#1118](https://github.com/laurigates/mcu-tinkering-lab/issues/1118)) ([f4af358](https://github.com/laurigates/mcu-tinkering-lab/commit/f4af35856dd9b147e26ffaf61fb21474163800b4)), closes [#1063](https://github.com/laurigates/mcu-tinkering-lab/issues/1063)

## [0.1.13](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.12...balancebot-v0.1.13) (2026-11-10)


### Bug Fixes

* **enclosure:** give balancebot's plugged headers a look-alike kind ([#1085](https://github.com/laurigates/mcu-tinkering-lab/issues/1085)) ([6c73a41](https://github.com/laurigates/mcu-tinkering-lab/commit/6c73a411a8cf338c014130491f542aba2a770aae)), closes [#1060](https://github.com/laurigates/mcu-tinkering-lab/issues/1060)

## [0.1.12](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.11...balancebot-v0.1.12) (2026-11-09)


### Features

* **balancebot:** refuse arm when a still IMU reads upside down ([#1014](https://github.com/laurigates/mcu-tinkering-lab/issues/1014)) ([3921edb](https://github.com/laurigates/mcu-tinkering-lab/commit/3921edb3e451b2aa37212b72ae6645be4850acf4))
* **balancebot:** seat J3 on the generated GY-521 socket footprint ([d236a07](https://github.com/laurigates/mcu-tinkering-lab/commit/d236a07561970ed7796e971c6728aa107a02541d))
* **kicad:** gate module sockets on silkscreen orientation marks ([#1022](https://github.com/laurigates/mcu-tinkering-lab/issues/1022)) ([9c456b3](https://github.com/laurigates/mcu-tinkering-lab/commit/9c456b3d81a507c916a1a458b1107a223b2b1174))
* **kicad:** gate the balancebot IMU mount against its layout ([#1020](https://github.com/laurigates/mcu-tinkering-lab/issues/1020)) ([449689b](https://github.com/laurigates/mcu-tinkering-lab/commit/449689b2f8b9685124846eb228d91e0a623a8116))
* **kicad:** put the commit on the fab silkscreen and in the file names ([f33f01f](https://github.com/laurigates/mcu-tinkering-lab/commit/f33f01f02d63921389684cb4b035c4340dfaa30a))


### Bug Fixes

* **balancebot:** refuse arm and load the CLI cannot vouch for ([#1041](https://github.com/laurigates/mcu-tinkering-lab/issues/1041)) ([0aeb607](https://github.com/laurigates/mcu-tinkering-lab/commit/0aeb6076c61acd82ed1cc00cdc9c9a1bcd36ea3b))
* **kicad:** keep a dirty fab's silk text as wide as a clean one ([5580784](https://github.com/laurigates/mcu-tinkering-lab/commit/5580784ef605e38239ce2aec6c476c0311012cf6))


### Documentation

* **carrier-pcb:** check each cart line's commit against the hand-off CSV ([0a829bd](https://github.com/laurigates/mcu-tinkering-lab/commit/0a829bdb528d1876969fe86e4e2d025305c33a60))

## [0.1.11](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.10...balancebot-v0.1.11) (2026-11-09)


### Features

* **balancebot:** carrier rev3 with expansion, I2C and test headers ([#972](https://github.com/laurigates/mcu-tinkering-lab/issues/972)) ([df86551](https://github.com/laurigates/mcu-tinkering-lab/commit/df865517fc9f9b1f3d4dd64951516fd72f4de6af))
* **balancebot:** IMU mounting setting for a GY-521 fitted component side down ([#971](https://github.com/laurigates/mcu-tinkering-lab/issues/971)) ([bfef65b](https://github.com/laurigates/mcu-tinkering-lab/commit/bfef65bbf4733d341af16a20c62e5510e7016fd5)), closes [#968](https://github.com/laurigates/mcu-tinkering-lab/issues/968)


### Documentation

* correct claims the new sensor reference pages contradicted ([#1007](https://github.com/laurigates/mcu-tinkering-lab/issues/1007)) ([9e20faa](https://github.com/laurigates/mcu-tinkering-lab/commit/9e20faaf1bb135552ad5f8d47f15eead1bdba112))

## [0.1.10](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.9...balancebot-v0.1.10) (2026-11-01)


### Features

* **kicad:** fail CI on a carrier part with no source ([#885](https://github.com/laurigates/mcu-tinkering-lab/issues/885)) ([800f66a](https://github.com/laurigates/mcu-tinkering-lab/commit/800f66a0eb4e8ffdfa38cafed1384a956785f78b))


### Bug Fixes

* **kicad:** enforce JLCPCB's silkscreen rules (1.0 mm text, 0.15 mm off pads) ([#877](https://github.com/laurigates/mcu-tinkering-lab/issues/877)) ([6ae892e](https://github.com/laurigates/mcu-tinkering-lab/commit/6ae892e647795a67e26f4c171969bd885b1b6e4c))


### Documentation

* **carrier:** date the bare order 2026-10-23, not the sourcing check ([7d40356](https://github.com/laurigates/mcu-tinkering-lab/commit/7d4035610ef395e5ca3174643dcda6eb93f650a5))
* **carrier:** every carrier part needs a source before the board is ordered ([6ceb3fc](https://github.com/laurigates/mcu-tinkering-lab/commit/6ceb3fc7cd5c6bbf5d18766ebb7e21755c864a7b))

## [0.1.9](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.8...balancebot-v0.1.9) (2026-10-25)


### Features

* **enclosure:** captioned product renders with breakout look-alikes, and Fibo 2.0 photos ([#842](https://github.com/laurigates/mcu-tinkering-lab/issues/842)) ([ee435fe](https://github.com/laurigates/mcu-tinkering-lab/commit/ee435fe55b687a2d7ab104b652da3872b19d77d1))
* **kicad:** printable per-step perfboard build guide ([#858](https://github.com/laurigates/mcu-tinkering-lab/issues/858)) ([eb8f5a4](https://github.com/laurigates/mcu-tinkering-lab/commit/eb8f5a44146f5cab62e1eae7879f027a7d29c6ae))
* **kicad:** route perfboard wires over free holes with splices and buses ([#857](https://github.com/laurigates/mcu-tinkering-lab/issues/857)) ([9f3693c](https://github.com/laurigates/mcu-tinkering-lab/commit/9f3693cc50f8f7e55960d7f3b6adfa90719eaac0))

## [0.1.8](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.7...balancebot-v0.1.8) (2026-10-23)


### Features

* **enclosure:** case configs for the balancebot and robocar carriers ([31a423f](https://github.com/laurigates/mcu-tinkering-lab/commit/31a423f0495682a1d73b1e7aa171093e06ff98b6))
* **kicad:** perfboard 1:1 print and continuity checklist ([#828](https://github.com/laurigates/mcu-tinkering-lab/issues/828)) ([b8ba591](https://github.com/laurigates/mcu-tinkering-lab/commit/b8ba591d6d50aeeb9ab565ab6263af550c5b018f))

## [0.1.7](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.6...balancebot-v0.1.7) (2026-10-21)


### Features

* **kicad:** add carrier-measure, layer-aware copper measurements ([#821](https://github.com/laurigates/mcu-tinkering-lab/issues/821)) ([2eba373](https://github.com/laurigates/mcu-tinkering-lab/commit/2eba37357a09f2d4e06df132a2847d46c3ab4f15))


### Bug Fixes

* **balancebot:** pre-order carrier fixes, reverse-polarity input, stitching vias ([#822](https://github.com/laurigates/mcu-tinkering-lab/issues/822)) ([684bb7d](https://github.com/laurigates/mcu-tinkering-lab/commit/684bb7de95200f9f0c1eb6250d30ea61f83995d5))

## [0.1.6](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.5...balancebot-v0.1.6) (2026-10-19)


### Features

* **kicad:** pre-fab review tooling and layout-quality gates ([#811](https://github.com/laurigates/mcu-tinkering-lab/issues/811)) ([81f5ff7](https://github.com/laurigates/mcu-tinkering-lab/commit/81f5ff7769ac8900baf8253f9430294f43cc43b2))


### Miscellaneous

* **carrier:** ignore KiCad local history folders ([#806](https://github.com/laurigates/mcu-tinkering-lab/issues/806)) ([547bf40](https://github.com/laurigates/mcu-tinkering-lab/commit/547bf40eb17c7c23f850ca31a1a8121711fd0d21))

## [0.1.5](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.4...balancebot-v0.1.5) (2026-10-15)


### Features

* **balancebot:** carrier PCB (KiCad 10), generated and checked against the firmware pin map ([#749](https://github.com/laurigates/mcu-tinkering-lab/issues/749)) ([fe563c7](https://github.com/laurigates/mcu-tinkering-lab/commit/fe563c721934cf0c3eb1a61bdc9eb73ee1a485be))
* **balancebot:** maze-route the perfboard wires between holes ([#757](https://github.com/laurigates/mcu-tinkering-lab/issues/757)) ([fb24b28](https://github.com/laurigates/mcu-tinkering-lab/commit/fb24b28b5dde154b06aa9ced38d03eb3597c26a7))
* **balancebot:** perfboard prototype layout of the carrier ([#754](https://github.com/laurigates/mcu-tinkering-lab/issues/754)) ([0227f28](https://github.com/laurigates/mcu-tinkering-lab/commit/0227f28325d6b7cf3f71f64a4d920381cf0f6864))
* **balancebot:** readable perfboard wiring, one panel per build step ([#755](https://github.com/laurigates/mcu-tinkering-lab/issues/755)) ([bdfdc0e](https://github.com/laurigates/mcu-tinkering-lab/commit/bdfdc0e052b4d2da5673585dfe18a366dbcacc45))
* **balancebot:** SMD passives on the carrier, with JLCPCB assembly files ([#753](https://github.com/laurigates/mcu-tinkering-lab/issues/753)) ([8d18cd8](https://github.com/laurigates/mcu-tinkering-lab/commit/8d18cd8d5edfb3363eb064db7e034e028e65fb47))
* **hardware:** harness and carrier views of hardware.toml ([#758](https://github.com/laurigates/mcu-tinkering-lab/issues/758)) ([1dcaaeb](https://github.com/laurigates/mcu-tinkering-lab/commit/1dcaaeb28671f3ce5df67161a630562e3fb46b63))
* **kicad:** generate the balancebot carrier schematic from the hardware join ([#759](https://github.com/laurigates/mcu-tinkering-lab/issues/759)) ([3c469fc](https://github.com/laurigates/mcu-tinkering-lab/commit/3c469fcfa19c11294b4f28474b4c9abd3a32d1b2))
* **kicad:** XIAO ESP32-S3 socket in the Seeed library and parts registry ([#774](https://github.com/laurigates/mcu-tinkering-lab/issues/774)) ([5353949](https://github.com/laurigates/mcu-tinkering-lab/commit/53539499e19f3cfa12b4d280857a7be976499755)), closes [#767](https://github.com/laurigates/mcu-tinkering-lab/issues/767)
* **schematics:** draw balancebot from the hardware join ([#765](https://github.com/laurigates/mcu-tinkering-lab/issues/765)) ([f867ac0](https://github.com/laurigates/mcu-tinkering-lab/commit/f867ac00e741d6264b69fa77fd058459fcbb6fd8)), closes [#738](https://github.com/laurigates/mcu-tinkering-lab/issues/738)


### Bug Fixes

* **balancebot:** seat the GY-521 right side up and take the pack on JST-XH ([#770](https://github.com/laurigates/mcu-tinkering-lab/issues/770)) ([6d4aaa6](https://github.com/laurigates/mcu-tinkering-lab/commit/6d4aaa68c4afe858d13c91bc9a3656a40d1a6a02))


### Documentation

* **balancebot:** lock motor power to a 3S 18650 pack with a BMS ([#744](https://github.com/laurigates/mcu-tinkering-lab/issues/744)) ([67f3767](https://github.com/laurigates/mcu-tinkering-lab/commit/67f3767103f0f9dc7c02d6c9585cd73b9f1b9d3c)), closes [#739](https://github.com/laurigates/mcu-tinkering-lab/issues/739)
* **balancebot:** step-by-step JLCPCB ordering for the carrier ([#756](https://github.com/laurigates/mcu-tinkering-lab/issues/756)) ([8d9c49b](https://github.com/laurigates/mcu-tinkering-lab/commit/8d9c49b94020ff18bfcbfb167e3e286c96681f25))
* **rules:** add KiCad scripting traps, correct the FootprintLoad blame ([#751](https://github.com/laurigates/mcu-tinkering-lab/issues/751)) ([3ac3f7d](https://github.com/laurigates/mcu-tinkering-lab/commit/3ac3f7d95121777ecd6c5089f896dbec8737665f))

## [0.1.4](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.3...balancebot-v0.1.4) (2026-10-12)


### Documentation

* **balancebot:** add a printable build guide generated from the hardware join ([13b99d7](https://github.com/laurigates/mcu-tinkering-lab/commit/13b99d73d01dc8ee04190e0fc24fa5277e4f9dbb))

## [0.1.3](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.2...balancebot-v0.1.3) (2026-08-22)


### Miscellaneous

* drop cargo-culted 'set positional-arguments' from justfiles ([998a9c4](https://github.com/laurigates/mcu-tinkering-lab/commit/998a9c479095bb2018e11b37af512d793dc17fa3)), closes [#410](https://github.com/laurigates/mcu-tinkering-lab/issues/410)

## [0.1.2](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.1...balancebot-v0.1.2) (2026-07-03)


### Documentation

* **balancebot:** add wiring schematic ([#364](https://github.com/laurigates/mcu-tinkering-lab/issues/364)) ([7274ac6](https://github.com/laurigates/mcu-tinkering-lab/commit/7274ac68912d59ca8a06c30c10a399ee31151fc4))

## [0.1.1](https://github.com/laurigates/mcu-tinkering-lab/compare/balancebot-v0.1.0...balancebot-v0.1.1) (2026-07-02)


### Features

* **balancebot:** XIAO RP2350 self-balancing robot + Pico SDK infrastructure ([#355](https://github.com/laurigates/mcu-tinkering-lab/issues/355)) ([64b5a7f](https://github.com/laurigates/mcu-tinkering-lab/commit/64b5a7fda1871936f986c56782de9fe0b78b971b))


### Bug Fixes

* **auto:** apply clang-format to remaining firmware files ([#172](https://github.com/laurigates/mcu-tinkering-lab/issues/172)) ([251cc9e](https://github.com/laurigates/mcu-tinkering-lab/commit/251cc9ecac21a1db102a43ba5985d0c04631cc79))
* **auto:** apply clang-format to system_config.h and test_i2c_protocol.c ([#165](https://github.com/laurigates/mcu-tinkering-lab/issues/165)) ([40f8d36](https://github.com/laurigates/mcu-tinkering-lab/commit/40f8d369f054a6606bbeded293cf7c8053f0bddc))
* **auto:** apply clang-format to test_i2c_protocol.c ([#163](https://github.com/laurigates/mcu-tinkering-lab/issues/163)) ([2b6a688](https://github.com/laurigates/mcu-tinkering-lab/commit/2b6a6888e563ac3f5a23cc1daf6cd9f19cb75e1a))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))
