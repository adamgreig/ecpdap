// Copyright 2020, 2021 Adam Greig
// Licensed under the Apache-2.0 and MIT licenses.

//! ecpdap
//!
//! ECP5 FPGA and SPI flash programming utility using CMSIS-DAP probes.

#[macro_use]
pub mod bitvec;

pub mod probe;
pub mod dap;
pub mod jtag;
pub mod ecp5;
