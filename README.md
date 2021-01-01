# ECPDAP

ECPDAP allows you to program ECP5 FPGAs and attached SPI flash using CMSIS-DAP
probes in JTAG mode.

## JTAG Scan Chains

ECP5 FPGAs can be programmed on arbitrary JTAG scan chains; you may need to
specify `--ir-lengths` and possibly specify a higher `--scan-chain-length`
depending on the other devices on your scan chain.

However, the attached SPI flashes require that the ECP5 is the only device
on the scan chain (in other words, the probe TDI and TDO connect directly
to the ECP5 pins).

## Pre-built Binaries

Pre-built binaries are available for Windows and Linux on the [Releases] page.
You must have [libusb] installed or available on your system, and you must
have permissions or drivers set up to access your CMSIS-DAP probe.

[Releases]: https://github.com/adamgreig/ecpdap/releases
[libusb]: https://libusb.info

## Building

* You must have a working Rust compiler installed.
  Visit [rustup.rs](https://rustup.rs) to install Rust.
* You must have [libusb] installed.
* You'll need to set up drivers or permissions to access the USB device.

Build and install for your user, without checking out the repository:

```
cargo install ecpdap
```

Building locally after checking out this repository:

```
cargo build --release
```

You can either run the ecpdap executable directly from `target/release/ecpdap`,
or you can install it for your user using `cargo install --path .`.

## Usage

Run `ecpdap help` for detailed usage. Commonly used commands:

* `ecpdap probes`
* `ecpdap scan`
* `ecpdap program bitstream.bit`
* `ecpdap flash id`
* `ecpdap flash scan`
* `ecpdap flash write bitstream.bit`
