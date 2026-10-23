# Changelog

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
