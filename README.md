# ECPDAP

[![crates.io](https://img.shields.io/crates/v/ecpdap.svg)](https://crates.io/crates/ecpdap)
[![docs.rs](https://docs.rs/ecpdap/badge.svg)](https://docs.rs/ecpdap)
![CI](https://github.com/adamgreig/ecpdap/workflows/CI/badge.svg)

ECPDAP allows you to program ECP5 FPGAs and attached SPI flash using CMSIS-DAP
probes in JTAG mode.

This crate uses [jtagdap] to handle CMSIS-DAP and JTAG, and [spi-flash-rs] to
handle the SPI flash itself. For programming SPI flashes directly, for example
when using iCE40 FPGAs, check out [spidap], which uses the same libraries.

[jtagdap]: https://github.com/adamgreig/jtagdap
[spi-flash-rs]: https://github.com/adamgreig/spi-flash-rs
[spidap]: https://github.com/adamgreig/spidap

## JTAG Scan Chains

ECP5 FPGAs can be programmed on arbitrary length JTAG scan chains; you may need
to specify `--ir-lengths` and possibly specify a higher `--scan-chain-length`
depending on the other devices on your scan chain.

However, accessing the attached SPI flashes require that the ECP5 is the only
device on the scan chain (in other words, the probe TDI and TDO connect
directly to the ECP5 pins).

## Pre-built Binaries

Pre-built binaries are available for Windows and Linux on the [Releases] page.
You must have [libusb] installed or available on your system, and you must
have permissions or drivers set up to access your CMSIS-DAP probe.

[Releases]: https://github.com/adamgreig/ecpdap/releases
[libusb]: https://libusb.info

## Building

* You must have a working Rust compiler installed.
  Visit [rustup.rs](https://rustup.rs) to install Rust.
* [libusb] is recommended to use the higher-speed CMSIS-DAPv2 protocol, where
  supported by your probe.
* You may need to set up drivers or permissions to access the USB device.

To build and install for your user, without checking out the repository:

```
cargo install ecpdap
```

Or, building locally after checking out this repository:

```
cargo build --release
```

You can either run the ecpdap executable directly from `target/release/ecpdap`,
or you can install it for your user using `cargo install --path .`.

## Usage

Run `ecpdap help` for detailed usage. Commonly used commands:

* `ecpdap probes`: List all detected CMSIS-DAP probes
* `ecpdap scan`: Scan the JTAG chain to detect ECP5 devices
* `ecpdap program bitstream.bit`: Program `bitstream.bit` to the ECP5
* `ecpdap flash id`: Read the flash manufacturer and product IDs
* `ecpdap flash scan`: Read the flash SFDP metadata and status registers
* `ecpdap flash write bitstream.bit`: Write `bitstream.bit` to flash memory.

## Licence

ecpdap is licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
