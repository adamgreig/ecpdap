use std::time::Duration;
use thiserror::Error;
use crate::dap::{DAP, Error as DAPError};

#[derive(Error, Debug)]
pub enum Error {
    #[error("Unexpected JTAG length returned from probe")]
    UnexpectedJTAGLength,
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
        // Enter Test-Logic-Reset and then Run-Test/Idle
        Sequence::new(&dap).mode(5, 1).mode(1, 0).execute()?;

        Ok(JTAG { dap })
    }

    /// Set JTAG bus clock speed.
    pub fn set_clock(&self, freq: u32) -> Result<()> {
        Ok(self.dap.set_clock(freq)?)
    }

    /// Pulse nRST low for `duration`.
    pub fn pulse_nrst(&self, duration: Duration) -> Result<()> {
        Ok(self.dap.pulse_nrst(duration)?)
    }

    pub fn sequence(&self) -> Sequence {
        Sequence::new(&self.dap)
    }

    /// Read all IDCODEs on the JTAG scan chain.
    pub fn idcodes(&self) -> Result<Vec<u32>> {
        // Set all devices up for a read of IDCODE
        let request = self.sequence()
            // Write TMS=1 for 5 clocks to ensure we are in test-logic-reset.
            .mode(5, 1)
            // Enter run-test/idle
            .mode(1, 0)
            // Enter select-dr-scan
            .mode(1, 1)
            // Enter capture-dr, shift-dr
            .mode(2, 0)
            // Read first IDCODE
            .read(32, 0);

        let mut idcodes = Vec::new();
        let mut data = request.execute()?;
        let mut idcode = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);

        // Read subsequent IDCODEs
        let request = self.sequence().read(32, 0);

        // Loop over all the incoming IDCODEs
        while idcode != 0xFFFF_FFFF && idcode != 0x0000_0000 {
            idcodes.push(idcode);
            data = request.clone().execute()?;
            idcode = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        }

        self.sequence()
            // Ensure we are in test-logic-reset.
            .mode(5, 1)
            // Enter run-test/idle.
            .mode(1, 0)
            .execute()?;

        Ok(idcodes)
    }

    /// Scan through and print all IDCODEs on the scan chain
    pub fn print_idcodes(&self) -> Result<()> {
        let idcodes = self.idcodes()?;
        for idcode in idcodes.iter() {
            println!("Read IDCODE: 0x{:08X}", idcode);
        }
        println!("Read {} IDCODEs in total.", idcodes.len());

        Ok(())
    }

    pub fn write_ir(&self, data: &[u8], nbits: usize) -> Result<()> {
        assert!(data.len() * 8 >= nbits);
        self.sequence()
            .mode(2, 1)     // Select-DR-Scan, Select-IR-Scan
            .mode(2, 0)     // Capture-IR, Shift-IR
            .write(nbits - 1, 0, data)
            .write(1, 1, &[data.last().unwrap() >> 7])
            .mode(1, 1)     // Update-IR
            .mode(1, 0)     // Run-Test/Idle
            .execute()?;
        Ok(())
    }

    pub fn read_dr(&self, nbits: usize) -> Result<Vec<u8>> {
        self.sequence()
            .mode(1, 1)     // Select-DR-Scan
            .mode(2, 0)     // Capture-DR, Shift-DR
            .read(nbits, 0)
            .mode(2, 1)     // Exit1-DR, Update-DR
            .mode(1, 0)     // Run-Test/Idle
            .execute()
    }

    pub fn write_dr(&self, data: &[u8], nbits: usize) -> Result<()> {
        assert!(data.len() * 8 >= nbits);
        self.sequence()
            .mode(1, 1)     // Select-DR-Scan
            .mode(2, 0)     // Capture-DR, Shift-DR
            .write(nbits - 1, 0, data)
            .write(1, 1, &[data.last().unwrap() >> 7])
            .mode(1, 1)     // Update-DR
            .mode(1, 0)     // Run-Test/Idle
            .execute()?;
        Ok(())
    }

    pub fn run_test_idle(&self, n: usize) -> Result<()> {
        self.sequence()
            .mode(n, 0)     // Select-DR-Scan
            .execute()?;
        Ok(())
    }
}

#[derive(Clone)]
pub struct Sequence<'a> {
    dap: &'a DAP,
    num_sequences: usize,
    capture_length: usize,
    request: Vec<u8>,
}

impl<'a> Sequence<'a> {
    pub fn new(dap: &'a DAP) -> Self {
        Sequence {
            dap,
            num_sequences: 0,
            capture_length: 0,
            request: Vec::new(),
        }
    }

    pub fn request(mut self, len: usize, tms: u8, tdi: Option<&[u8]>, capture: bool)
        -> Self
    {
        let nbytes = bytes_for_bits(len);
        let dummy = vec![0xff; nbytes];

        let tdi = match tdi {
            Some(tdi) => tdi,
            None => &dummy,
        };

        assert!(len <= 64);
        assert!(tdi.len() == nbytes);
        assert!(2 + self.request.len() + 1 + nbytes <= 64);

        let mut header = if len == 64 { 0 } else { len as u8 };
        if tms != 0 {
            header |= 1 << 6;
        }
        if capture {
            header |= 1 << 7;
            self.capture_length += nbytes;
        }

        self.request.extend_from_slice(&[header]);
        self.request.extend_from_slice(tdi);
        self.num_sequences += 1;
        self
    }

    pub fn write(self, len: usize, tms: u8, tdi: &[u8]) -> Self {
        self.request(len, tms, Some(tdi), false)
    }

    pub fn mode(self, len: usize, tms: u8) -> Self {
        self.request(len, tms, None, false)
    }

    pub fn read(self, len: usize, tms: u8) -> Self {
        self.request(len, tms, None, true)
    }

    fn to_bytes(&self) -> Vec<u8> {
        let mut request = vec![self.num_sequences as u8];
        request.extend_from_slice(&self.request[..]);
        request
    }

    pub fn execute(self) -> Result<Vec<u8>> {
        let request = self.to_bytes();
        let result = self.dap.jtag_sequence(&request[..])?;
        if result.len() == self.capture_length {
            Ok(result)
        } else {
            Err(Error::UnexpectedJTAGLength)
        }
    }
}

/// Returns number of whole bytes required to hold `n` bits.
fn bytes_for_bits(n: usize) -> usize {
    (n + 7) / 8
}
