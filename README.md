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

## IDCODEs and SPI modes

ECP5 bitstreams can specify an IDCODE which the FPGA checks against its
internal IDCODE. By default this would prevent loading (for example) an
`LFE5U-45F` bitstream onto an `LFE5UM-45F`, even though it would be
compatible. By default, `ecpdap` will patch the bitstream IDCODE when
programming either SRAM or SPI flash to match the IDCODE detected via JTAG.
Use `--no-fix-idcode` to disable this functionality. Alternatively, use
`--remove-idcode` to entirely remove the IDCODE check from the bitstream.

Bitstreams can also include an SPI mode command to enable faster bitstream
loading from SPI flash. However, if a bitstream with such a command is loaded
directly to SRAM, and the SPI flash does not have a valid bitstream loaded,
the ECP5 aborts loading the bitstream. To work around this issue, `ecpdap`
will by default remove SPI mode commands from bitstreams loaded to SRAM.
Use `--no-remove-spimode` to disable this functionality.

## JTAG Clock Frequency

The default clock frequency is 1MHz, but in many situations higher frequencies
are possible and reduce operation time. It is also possible to require lower
speeds in situations with poor signal integrity.

Use `-f` or `--freq` to change, for example `-f 10M`.

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
have permissions or drivers set up to access your CMSIS-DAP probe. See the
[drivers] page for information on setup.

ECPDAP is also packaged for NixOS under the `ecpdap` attribute.

[Releases]: https://github.com/adamgreig/ecpdap/releases
[libusb]: https://libusb.info
[drivers]: https://github.com/adamgreig/ecpdap/tree/master/drivers

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
* `ecpdap program bitstream.bit -f10M`: Program `bitstream.bit` to the ECP5 at 10MHz
* `ecpdap flash id`: Read the flash manufacturer and product IDs
* `ecpdap flash scan`: Read the flash SFDP metadata and status registers
* `ecpdap flash write bitstream.bit`: Write `bitstream.bit` to flash memory.

## Licence

ecpdap is licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
