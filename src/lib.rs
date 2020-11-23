//! ecpdap
//! Copyright 2020 Adam Greig
//! Licensed under the Apache-2.0 and MIT licenses.

use thiserror::Error;

#[macro_use]
pub mod bitvec;

pub mod probe;
pub mod dap;
pub mod jtag;
pub mod ecp5;
pub mod flash;

#[derive(Error, Debug)]
pub enum Error {
    #[error("probe errror")]
    Probe(#[from] probe::Error),

    #[error("DAP error")]
    DAP(#[from] dap::Error),

    #[error(transparent)]
    Other(#[from] anyhow::Error),

    #[error("unknown error")]
    Unknown,
}

pub type Result<T> = std::result::Result<T, Error>;
