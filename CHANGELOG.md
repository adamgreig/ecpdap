# Changelog

## [Unreleased]

* Change `-f`/`--freq` clock frequency option to take Hz instead of kHz,
  but permit using `k` and `M` suffixes e.g. `-f 10M`.
* Permit using `0x` prefixes for hex offsets and addresses when accessing
  flash memory.
* Move `--no-remove-spimode` flag to only apply to `program` subcommand
* Add new `-v`/`--verbose` flag to raise log level (specifying the RUST_LOG
  environment variable still works).
* Add new `flash jump` subcommand which can read or write a small JUMP
  bitstream at the end of the SPI flash, used for dual boot mode.
* Re-export `jtagdap` and `spi_flash` dependencies.

## [v0.2.0] - 2022-10-02

* Add support for removing SPI_MODE commands when writing bitstreams to SRAM
    * Use `--no-remove-spimode` to disable
* Add support for patching VERIFY_IDCODE commands when writing bitstreams
    * Use `--no-fix-idcode` to disable
* Add support for removing VERIFY_IDCODE commands when writing bitstreams
    * Use `--remove-idcode` to remove VERIFY_IDCODE commands
* Update to clap v4

## [v0.1.8] - 2022-07-31

* Update to latest jtagdap 0.1.1, improving CMSIS-DAP detection
* Update to latest spi-flash 0.3.0, no functional changes
* Improve progress bars

## [v0.1.7] - 2021-07-03

* Print names of detected ECP5 parts when scanning the JTAG chain.

## [v0.1.6] - 2021-04-30

* No functional changes.
* Updated to include new `Cargo.lock` and udev rules file as part of a tagged
  release.

## [v0.1.5] - 2021-01-20

* Detect HID-only probes that are not found by libusb.
* Update spi-flash dependency to fix errors when using some flash ICs.

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

[Unreleased]: https://github.com/adamgreig/ecpdap/compare/v0.2.0...HEAD
[v0.2.0]: https://github.com/adamgreig/ecpdap/compare/v0.1.8...v0.2.0
[v0.1.8]: https://github.com/adamgreig/ecpdap/compare/v0.1.7...v0.1.8
[v0.1.7]: https://github.com/adamgreig/ecpdap/compare/v0.1.6...v0.1.7
[v0.1.6]: https://github.com/adamgreig/ecpdap/compare/v0.1.5...v0.1.6
[v0.1.5]: https://github.com/adamgreig/ecpdap/compare/v0.1.4...v0.1.5
[v0.1.4]: https://github.com/adamgreig/ecpdap/compare/v0.1.3...v0.1.4
[v0.1.3]: https://github.com/adamgreig/ecpdap/compare/v0.1.2...v0.1.3
[v0.1.2]: https://github.com/adamgreig/ecpdap/compare/v0.1.1...v0.1.2
[v0.1.1]: https://github.com/adamgreig/ecpdap/compare/v0.1.0...v0.1.1
[v0.1.0]: https://github.com/adamgreig/ecpdap/tree/v0.1.0
