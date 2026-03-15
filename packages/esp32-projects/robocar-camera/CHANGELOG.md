# Changelog

## [0.1.1](https://github.com/laurigates/mcu-tinkering-lab/compare/robocar-camera-v0.1.0...robocar-camera-v0.1.1) (2026-03-15)


### Features

* Add OTA firmware update system ([#78](https://github.com/laurigates/mcu-tinkering-lab/issues/78)) ([b41bedc](https://github.com/laurigates/mcu-tinkering-lab/commit/b41bedc4e80eaf851007b214874cd81f66c3b13c))
* Extend I2C protocol for OTA commands (Issue [#23](https://github.com/laurigates/mcu-tinkering-lab/issues/23)) ([#35](https://github.com/laurigates/mcu-tinkering-lab/issues/35)) ([5ac6c0e](https://github.com/laurigates/mcu-tinkering-lab/commit/5ac6c0ec4ea70d26ee6e772983f716d937e9e5ea))
* **ota:** complete orchestration loop and maintenance mode safe state ([#109](https://github.com/laurigates/mcu-tinkering-lab/issues/109)) ([9d611a0](https://github.com/laurigates/mcu-tinkering-lab/commit/9d611a048d2e7b1fefc2de0d507e1d0fd42f8257))
* Update partition tables for 4MB flash OTA support ([#22](https://github.com/laurigates/mcu-tinkering-lab/issues/22)) ([#34](https://github.com/laurigates/mcu-tinkering-lab/issues/34)) ([137e563](https://github.com/laurigates/mcu-tinkering-lab/commit/137e563fe9e4f58fe05d24209a313be09303cd47))


### Bug Fixes

* Add mutex protection for OTA state and fix I2C reboot handling ([#91](https://github.com/laurigates/mcu-tinkering-lab/issues/91)) ([392897c](https://github.com/laurigates/mcu-tinkering-lab/commit/392897c1a342fc73610fe322114d1b7e6a6ad7a4))
* **auto:** fix C/C++ and Python formatting violations ([#92](https://github.com/laurigates/mcu-tinkering-lab/issues/92)) ([1a25d5a](https://github.com/laurigates/mcu-tinkering-lab/commit/1a25d5a14ce8cfeb570b70aa5997f39fbac57861))
* **auto:** fix C/C++ formatting in robocar-camera claude_api.c ([#79](https://github.com/laurigates/mcu-tinkering-lab/issues/79)) ([83a3688](https://github.com/laurigates/mcu-tinkering-lab/commit/83a3688d1299536e3154cc303c89ffcde4819ed5))
* **auto:** fix C/C++ formatting in robocar-camera main.c ([#77](https://github.com/laurigates/mcu-tinkering-lab/issues/77)) ([74e8284](https://github.com/laurigates/mcu-tinkering-lab/commit/74e8284fab17612e94780cfacffc46ee54d933be))
* code quality improvements, CI hardening, and audiobook player board migration ([#51](https://github.com/laurigates/mcu-tinkering-lab/issues/51)) ([0a0d68c](https://github.com/laurigates/mcu-tinkering-lab/commit/0a0d68c2c84944a1ac90a7c3f2dbd2f3e705f0bf))
* Enable component tags and remove version.txt files ([a203fd2](https://github.com/laurigates/mcu-tinkering-lab/commit/a203fd287d2fbefb6a3007f9bf48648efa274a92))
* **robocar-simulation:** code quality and lint fixes ([#86](https://github.com/laurigates/mcu-tinkering-lab/issues/86)) ([a7a13bf](https://github.com/laurigates/mcu-tinkering-lab/commit/a7a13bfce2cd40593f1f5b0eeff85e5f1fe330ea))
* **robocar-simulation:** code quality and ruff fixes ([#68](https://github.com/laurigates/mcu-tinkering-lab/issues/68)) ([12a5b6e](https://github.com/laurigates/mcu-tinkering-lab/commit/12a5b6eb8820c50ef8581efda4d291894dbb10fe))
* **robocar-simulation:** code quality and ruff fixes ([#69](https://github.com/laurigates/mcu-tinkering-lab/issues/69)) ([c7013de](https://github.com/laurigates/mcu-tinkering-lab/commit/c7013de7d99f36be35c2d94cff390e29d4329b26))


### Miscellaneous

* add justfiles and robocar-simulation CLAUDE.md ([#61](https://github.com/laurigates/mcu-tinkering-lab/issues/61)) ([6517652](https://github.com/laurigates/mcu-tinkering-lab/commit/6517652a8c9e83431a15632ff715f7366f0e50fd))
* Add release-please configuration for automated firmware releases ([eb061bc](https://github.com/laurigates/mcu-tinkering-lab/commit/eb061bc85c73050d7b5e9700b7d58fd755a7748c)), closes [#24](https://github.com/laurigates/mcu-tinkering-lab/issues/24)
* remove Makefiles replaced by justfiles, update docs ([#100](https://github.com/laurigates/mcu-tinkering-lab/issues/100)) ([3872691](https://github.com/laurigates/mcu-tinkering-lab/commit/3872691fa0739b6845a2fa7f176daea817ee6b36)), closes [#54](https://github.com/laurigates/mcu-tinkering-lab/issues/54)
* robocar-simulation fixes and add justfiles ([#60](https://github.com/laurigates/mcu-tinkering-lab/issues/60)) ([27e7cfb](https://github.com/laurigates/mcu-tinkering-lab/commit/27e7cfb88632247f9d9449b2d6f3bccfd2af86c7))
