use std::time::Duration;
use thiserror::Error;
use crate::dap::{DAP, Error as DAPError};

#[derive(Error, Debug)]
pub enum Error {
    #[error("DAP error")]
    DAP(#[from] DAPError),
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

pub struct JTAG {
    dap: DAP,
}

impl JTAG {
    pub fn new(dap: DAP) -> Result<JTAG> {
        Ok(JTAG { dap })
    }

    pub fn set_clock(&self, freq: u32) -> Result<()> {
        Ok(self.dap.set_clock(freq)?)
    }

    pub fn pulse_nrst(&self, duration: Duration) -> Result<()> {
        Ok(self.dap.pulse_nrst(duration)?)
    }
}
