# Changelog

## [Unreleased]

## [v0.1.4] - 2021-01-02

* Improve flash write performance by reducing JTAG state transitions.
* Fix ECP5 programming in long scan chains.

## [v0.1.3] - 2021-01-02

* Support CMSIS-DAPv1 probes with non-64-byte HID reports, such as MCU-Link.

## [v0.1.2] - 2021-01-02

* Swap to using extracted spi-flash crate
* Add reload command and --no-reload option to flash write
* Add :v1 option to probe specifier to force CMSIS-DAPv1 usage
* Drain probes on connection

## [v0.1.1] - 2021-01-01

* No functional changes.

## [v0.1.0] - 2021-01-01

* Initial release.

[Unreleased]: https://github.com/adamgreig/ecpdap/compare/v0.1.4...HEAD
[v0.1.4]: https://github.com/adamgreig/ecpdap/compare/v0.1.3...v0.1.4
[v0.1.3]: https://github.com/adamgreig/ecpdap/compare/v0.1.2...v0.1.3
[v0.1.2]: https://github.com/adamgreig/ecpdap/compare/v0.1.1...v0.1.2
[v0.1.1]: https://github.com/adamgreig/ecpdap/compare/v0.1.0...v0.1.1
[v0.1.0]: https://github.com/adamgreig/ecpdap/tree/v0.1.0
