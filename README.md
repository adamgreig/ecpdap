# ECPDAP

ECPDAP allows you to program ECP5 FPGAs and attached SPI flash using CMSIS-DAP
probes in JTAG mode.

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

Build and install:

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

* `ecpdap list`
* `ecpdap program bitstream.bit`
* `ecpdap flash id`
* `ecpdap flash write bitstream.bit`
