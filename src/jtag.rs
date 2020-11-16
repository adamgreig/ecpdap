//! This module implements JTAG functionality on top of the DAP_JTAG_Sequence
//! command provided by the DAP module.
//!
//! The JTAG chain may be scanned for TAPs, and then a TAP interface created to
//! address that specific device.

use std::time::Duration;
use crate::dap::{DAP, Error as DAPError};
use crate::bitvec::{bits_to_bytes, bytes_to_bits, drain_u32, Error as BitVecError};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Unexpected JTAG length returned from probe")]
    UnexpectedJTAGLength,
    #[error("Internal error: invalid JTAG sequence")]
    InvalidSequence,
    #[error("JTAG scan chain broken: check connection")]
    ScanChainBroken,
    #[error("Internal error: Tried to run sequences without providing a DAP")]
    NoDAPProvided,
    #[error("Internal error: Unexpected JTAG state")]
    BadState,
    #[error("Invalid IDCODE detected")]
    InvalidIDCODE,
    #[error("Error scanning IR lengths")]
    InvalidIR,
    #[error("Bit vector error")]
    BitVec(#[from] BitVecError),
    #[error("DAP error")]
    DAP(#[from] DAPError),
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Copy, Clone, Debug)]
#[allow(unused)]
enum TAPState {
    Unknown,
    TestLogicReset,
    RunTestIdle,
    SelectDRScan,
    CaptureDR,
    ShiftDR,
    Exit1DR,
    PauseDR,
    Exit2DR,
    UpdateDR,
    SelectIRScan,
    CaptureIR,
    ShiftIR,
    Exit1IR,
    PauseIR,
    Exit2IR,
    UpdateIR,
}

pub struct JTAG {
    dap: DAP,
    state: TAPState,
    max_length: usize,
}

impl JTAG {
    /// Create a new JTAG object which takes ownership of the provided DAP.
    ///
    /// The initial state is TAPState::Unknown; the only way to enter a
    /// known state is by calling `enter_test_logic_reset()` to reset.
    pub fn new(dap: DAP) -> JTAG {
        JTAG { dap, state: TAPState::Unknown, max_length: 128 }
    }

    /// Set the maximum scan chain length to attempt to detect.
    pub fn set_max_length(&mut self, max_length: usize) {
        self.max_length = max_length;
    }

    /// Set JTAG bus clock speed.
    pub fn set_clock(&self, freq: u32) -> Result<()> {
        Ok(self.dap.set_clock(freq)?)
    }

    /// Pulse nRST low for `duration`.
    pub fn pulse_nrst(&self, duration: Duration) -> Result<()> {
        Ok(self.dap.pulse_nrst(duration)?)
    }

    /// Scan JTAG chain for IDCODEs.
    ///
    /// Returns a Vec of Option<IDCODE>, which are Some(IDCODE) for detected IDCODEs
    /// and are None for detected BYPASS.
    pub fn scan(&mut self, ir_lengths: Option<&[usize]>) -> Result<Vec<Option<IDCODE>>> {
        let (ir, dr) = self.reset_scan()?;
        let idcodes = extract_idcodes(&dr)?;
        extract_ir_lens(&ir, idcodes.len(), ir_lengths)?;
        Ok(idcodes)
    }

    /// Force all TAPs into Test-Logic-Reset state from any current state.
    pub fn enter_test_logic_reset(&mut self) -> Result<()> {
        log::trace!("Resetting JTAG state to Test-Logic-Reset");
        self.mode(bv![1, 1, 1, 1, 1])?;
        self.state = TAPState::TestLogicReset;
        Ok(())
    }

    /// Enter Run-Test/Idle state from TestLogicReset, Exit1*, Pause*, or Update*.
    pub fn enter_run_test_idle(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> RunTestIdle", self.state);
        match self.state {
            TAPState::RunTestIdle                   => (),
            TAPState::TestLogicReset                => self.mode(bv![0])?,
            TAPState::Exit1DR  | TAPState::Exit1IR  => self.mode(bv![1, 0])?,
            TAPState::PauseDR  | TAPState::PauseIR  => self.mode(bv![1, 1, 0])?,
            TAPState::UpdateDR | TAPState::UpdateIR => self.mode(bv![0])?,
            _                                       => return Err(Error::BadState),
        };
        self.state = TAPState::RunTestIdle;
        Ok(())
    }

    /// Enter Shift-DR state from Test-Logic-Reset, Run-Test/Idle, Update*, or Pause*.
    pub fn enter_shift_dr(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> ShiftDR", self.state);
        match self.state {
            TAPState::ShiftDR                       => (),
            TAPState::TestLogicReset                => self.mode(bv![0, 1, 0, 0])?,
            TAPState::RunTestIdle                   => self.mode(bv![1, 0, 0])?,
            TAPState::UpdateIR | TAPState::UpdateDR => self.mode(bv![1, 0, 0])?,
            TAPState::PauseIR                       => self.mode(bv![1, 1, 1, 0, 0])?,
            TAPState::PauseDR                       => self.mode(bv![1, 0])?,
            _                                       => return Err(Error::BadState),
        }
        self.state = TAPState::ShiftDR;
        Ok(())
    }

    /// Enter Pause-DR from Exit1-DR.
    pub fn enter_pause_dr(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> PauseDR", self.state);
        match self.state {
            TAPState::PauseDR                       => (),
            TAPState::Exit1DR                       => self.mode(bv![0])?,
            _                                       => return Err(Error::BadState),
        }
        self.state = TAPState::PauseDR;
        Ok(())
    }

    /// Enter Update-DR from Shift-DR or Exit1-DR.
    pub fn enter_update_dr(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> UpdateDR", self.state);
        match self.state {
            TAPState::UpdateDR                      => (),
            TAPState::ShiftDR                       => self.mode(bv![1, 1])?,
            TAPState::Exit1DR                       => self.mode(bv![1])?,
            _                                       => return Err(Error::BadState),
        }
        self.state = TAPState::UpdateDR;
        Ok(())
    }


    /// Enter Shift-IR state from Test-Logic-Reset, Run-Test/Idle, Update*, or Pause*.
    pub fn enter_shift_ir(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> ShiftIR", self.state);
        match self.state {
            TAPState::ShiftIR                       => (),
            TAPState::TestLogicReset                => self.mode(bv![0, 1, 1, 0, 0])?,
            TAPState::RunTestIdle                   => self.mode(bv![1, 1, 0, 0])?,
            TAPState::UpdateIR | TAPState::UpdateDR => self.mode(bv![1, 1, 0, 0])?,
            TAPState::PauseIR                       => self.mode(bv![1, 0])?,
            TAPState::PauseDR                       => self.mode(bv![1, 1, 1, 1, 0, 0])?,
            _                                       => return Err(Error::BadState),
        }
        self.state = TAPState::ShiftIR;
        Ok(())
    }

    /// Enter Pause-IR from Exit1-IR.
    pub fn enter_pause_ir(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> PauseIR", self.state);
        match self.state {
            TAPState::PauseIR                       => (),
            TAPState::Exit1IR                       => self.mode(bv![0])?,
            _                                       => return Err(Error::BadState),
        }
        self.state = TAPState::PauseIR;
        Ok(())
    }

    /// Enter Update-IR from Shift-IR or Exit1-IR.
    pub fn enter_update_ir(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> UpdateIR", self.state);
        match self.state {
            TAPState::UpdateIR                      => (),
            TAPState::ShiftIR                       => self.mode(bv![1, 1])?,
            TAPState::Exit1IR                       => self.mode(bv![1])?,
            _                                       => return Err(Error::BadState),
        }
        self.state = TAPState::UpdateIR;
        Ok(())
    }

    /// Move to Shift-IR, write `tdi` to the IR register, then enter Update-IR.
    pub fn write_ir(&mut self, tdi: &[bool]) -> Result<()> {
        log::debug!("Writing to IR: {:02X?}", bits_to_bytes(tdi));
        self.enter_shift_ir()?;
        self.write(tdi)?;
        Ok(())
    }

    /// Move to Shift-IR, read `n` bits of IR while writing 0xFF, then enter Update-IR.
    /// Returns the captured bits from TDO.
    pub fn read_ir(&mut self, n: usize) -> Result<Vec<bool>> {
        log::debug!("Reading from IR...");
        self.enter_shift_ir()?;
        let tdo = self.read(n)?;
        log::trace!("Read from IR: {:02X?}", bits_to_bytes(&tdo));
        Ok(tdo)
    }

    /// Move to Shift-IR, write `tdi` to the IR register while capturing TDO, then enter Update-IR.
    /// Returns the captured bits from TDO.
    pub fn exchange_ir(&mut self, tdi: &[bool]) -> Result<Vec<bool>> {
        log::debug!("Exchanging with IR: {:02X?}", bits_to_bytes(tdi));
        self.enter_shift_ir()?;
        let tdo = self.exchange(tdi)?;
        log::trace!("Exchanged from IR: {:02X?}", bits_to_bytes(&tdo));
        Ok(tdo)
    }

    /// Move to Shift-DR, write `tdi` to the DR register, then enter Update-DR.
    pub fn write_dr(&mut self, tdi: &[bool]) -> Result<()> {
        log::debug!("Writing to DR: {:?}", bits_to_bytes(tdi));
        self.enter_shift_dr()?;
        self.write(tdi)?;
        Ok(())
    }

    /// Move to Shift-DR, read `n` bits of DR while writing 0xFF, then enter Update-DR.
    /// Returns the captured bits from TDO.
    pub fn read_dr(&mut self, n: usize) -> Result<Vec<bool>> {
        log::debug!("Reading from DR...");
        self.enter_shift_dr()?;
        let tdo = self.read(n)?;
        log::trace!("Read from DR: {:?}", bits_to_bytes(&tdo));
        Ok(tdo)
    }

    /// Move to Shift-DR, write `tdi` to the DR register while capturing TDO, then enter Update-DR.
    /// Returns the captured bits from TDO.
    pub fn exchange_dr(&mut self, tdi: &[bool]) -> Result<Vec<bool>> {
        log::debug!("Exchanging with DR: {:?}", bits_to_bytes(tdi));
        self.enter_shift_dr()?;
        let tdo = self.exchange(tdi)?;
        log::trace!("Exchanged from DR: {:?}", bits_to_bytes(&tdo));
        Ok(tdo)
    }

    /// Create a new Sequences object for running custom JTAG sequences.
    ///
    /// This method is not `pub` to prevent changing the JTAG state without
    /// updating the `state` member.
    fn sequences(&self) -> Sequences {
        Sequences::with_dap(&self.dap)
    }

    /// Shortcut for shifting TMS mode bits.
    ///
    /// This method does not change `self.state`, which must be updated manually
    /// after calling this method.
    fn mode(&self, tms: &[bool]) -> Result<()> {
        self.sequences().mode(tms)?.run()?;
        Ok(())
    }

    /// Shortcut for reading `n` bits from the current register and exiting into Update*.
    ///
    /// Must already be in a Shift* state, and `self.state` is updated upon completion.
    fn read(&mut self, n: usize) -> Result<Vec<bool>> {
        match self.state {
            TAPState::ShiftIR | TAPState::ShiftDR => (),
            _ => return Err(Error::BadState),
        }
        log::trace!("JTAG state: {:?} -> Update, reading {} bits", self.state, n);
        let result = self.sequences().read(n, true)?.mode(bv![1])?.run()?;
        match self.state {
            TAPState::ShiftIR => self.state = TAPState::UpdateIR,
            TAPState::ShiftDR => self.state = TAPState::UpdateDR,
            _ => unreachable!(),
        }
        Ok(result)
    }

    /// Shortcut for writing `tdi` bits to the current register and exiting into Update*.
    ///
    /// Must already be in a Shift* state, and `self.state` is updated upon completion.
    fn write(&mut self, tdi: &[bool]) -> Result<()> {
        match self.state {
            TAPState::ShiftIR | TAPState::ShiftDR => (),
            _ => return Err(Error::BadState),
        }
        log::trace!("JTAG state: {:?} -> Update, writing {} bits", self.state, tdi.len());
        self.sequences().write(tdi, true)?.mode(bv![1])?.run()?;
        match self.state {
            TAPState::ShiftIR => self.state = TAPState::UpdateIR,
            TAPState::ShiftDR => self.state = TAPState::UpdateDR,
            _ => unreachable!(),
        }
        Ok(())
    }

    /// Shortcut for exchanging `tdi` bits with the current register and exiting into Update*.
    ///
    /// Must already be in a Shift* state, and `self.state` is updated upon completion.
    fn exchange(&mut self, tdi: &[bool]) -> Result<Vec<bool>> {
        match self.state {
            TAPState::ShiftIR | TAPState::ShiftDR => (),
            _ => return Err(Error::BadState),
        }
        log::trace!("JTAG state: {:?} -> Update, exchanging {} bits", self.state, tdi.len());
        let tdo = self.sequences().exchange(tdi, true)?.mode(bv![1])?.run()?;
        match self.state {
            TAPState::ShiftIR => self.state = TAPState::UpdateIR,
            TAPState::ShiftDR => self.state = TAPState::UpdateDR,
            _ => unreachable!(),
        }
        Ok(tdo)
    }

    /// Capture the power-up scan chain values, including all IDCODEs.
    ///
    /// Returns the IR and DR results as (IR, DR).
    fn reset_scan(&mut self) -> Result<(Vec<bool>, Vec<bool>)> {
        log::debug!("Running reset scan of JTAG chain");
        self.enter_test_logic_reset()?;
        let dr = self.scan_dr()?;
        let ir = self.scan_ir()?;
        Ok((ir, dr))
    }

    /// Detect the IR chain length and return its current contents.
    ///
    /// Replaces the current contents with all 1s (BYPASS) and enters
    /// the Run-Test/Idle state.
    fn scan_ir(&mut self) -> Result<Vec<bool>> {
        log::debug!("Scanning IR up to {} bits", self.max_length);
        self.enter_shift_ir()?;
        let data = self.scan_inner("IR")?;
        self.enter_run_test_idle()?;
        Ok(data)
    }

    /// Detect the DR chain length and return its contents.
    ///
    /// Replaces the current contents with all 1s and enters
    /// the Run-Test/Idle state.
    fn scan_dr(&mut self) -> Result<Vec<bool>> {
        log::debug!("Scanning DR up to {} bits", self.max_length);
        self.enter_shift_dr()?;
        let data = self.scan_inner("DR")?;
        self.enter_run_test_idle()?;
        Ok(data)
    }

    /// Detect current chain length and return its contents.
    /// Must already be in either ShiftIR or ShiftDR state.
    fn scan_inner(&mut self, name: &'static str) -> Result<Vec<bool>> {
        match self.state {
            TAPState::ShiftIR | TAPState::ShiftDR => (),
            _ => return Err(Error::BadState),
        };
        let len = self.max_length;

        // Completely fill xR with 0s, capture result.
        let d0 = self.sequences().exchange(&vec![false; len+1], false)?.run()?;

        // Completely fill xR with 1s, capture result, exit afterwards.
        let d1 = self.sequences().exchange(&vec![true; len+1], true)?.run()?;
        match self.state {
            TAPState::ShiftIR => self.state = TAPState::Exit1IR,
            TAPState::ShiftDR => self.state = TAPState::Exit1DR,
            _ => unreachable!(),
        }

        // Find first 1 in d1, which indicates length of register.
        let n = match d1.iter().position(|bit| *bit) {
            Some(n) => {
                log::info!("JTAG {} scan chain detected as {} bits long", name, n);
                n
            },
            None => {
                log::error!("JTAG {} scan chain either broken or too long: \
                             did not detect 1", name);
                return Err(Error::ScanChainBroken);
            }
        };

        // Check at least one register is detected in the scan chain.
        if n == 0 {
            log::error!("JTAG {} scan chain is empty", name);
            return Err(Error::ScanChainBroken);
        }

        // Check d0[n..] are all 0.
        if d0[n..].iter().any(|bit| *bit) {
            log::warn!("JTAG {} scan chain either broken or too long: did not detect 0", name);
            return Err(Error::ScanChainBroken);
        }

        // Extract d0[..n] as the initial scan chain contents.
        let data = &d0[..n];
        log::trace!("JTAG {} scan chain contents: {:02X?}", name, bits_to_bytes(data));
        Ok(data.to_vec())
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct IDCODE(u32);

impl IDCODE {
    pub fn valid(&self) -> bool {
        // Check LSbit is 1 and Manufacturer field is not the reserved value.
        (self.0 & 1 == 1) && (self.0 & 0b1111_1111_1110 != 0b0000_1111_1110)
    }

    /// Extract the manufacturer ID, which is an 11-bit field in bits 1-11.
    pub fn manufacturer(&self) -> u16 {
        ((self.0 >> 1) & 0x7FF) as u16
    }

    /// Return the manufacturer name, if available.
    pub fn manufacturer_name(&self) -> Option<&'static str> {
        let cc = ((self.0 >> 8) & 0x0F) as u8;
        if cc >= 11 {
            return None;
        }
        let id = ((self.0 >> 1) & 0x7F) as u8;
        jep106::JEP106Code::new(cc, id).get()
    }

    /// Extract the part number, which is a 16-bit field in bits 12-27.
    pub fn part_number(&self) -> u16 {
        ((self.0 >> 12) & 0xFFFF) as u16
    }

    /// Extract the IDCODE version, which is a 4-bit field in bits 28-31.
    pub fn version(&self) -> u8 {
        ((self.0 >> 28) & 0xF) as u8
    }
}

impl std::fmt::Display for IDCODE {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(mfn) = self.manufacturer_name() {
            write!(f, "0x{:08X} ({})", self.0, mfn)
        } else {
            write!(f, "0x{:08X}", self.0)
        }
    }
}

#[test]
fn test_idcode() {
    // Made-up IDCODE with invalid manufacturer name.
    let idcode = IDCODE(0b1010_0000111100001111_11101010101_1);
    assert!(idcode.valid());
    assert_eq!(idcode.manufacturer(), 0b11101010101);
    assert_eq!(idcode.part_number(), 0b0000111100001111);
    assert_eq!(idcode.version(), 0b1010);
    assert!(format!("{}", idcode) == "0xA0F0FEAB");

    // Example IDCODE from an ECP5.
    let idcode = IDCODE(0x41111043);
    assert!(idcode.manufacturer_name() == Some("Lattice Semi."));
    assert!(format!("{}", idcode) == "0x41111043 (Lattice Semi.)");
}

/// Stores information about a JTAG scan chain,
/// including detected IDCODEs and IR lengths.
pub struct JTAGChain {
    idcodes: Vec<Option<IDCODE>>,
    irlen: Vec<usize>,
}

/// Extract all IDCODEs from a test-logic-reset DR chain `dr`.
///
/// Valid IDCODEs have a '1' in the least significant (first) bit,
/// and are 32 bits long. DRs in BYPASS always have a single 0 bit.
///
/// We can therefore unambiguously scan through the DR capture to find
/// all IDCODEs and TAPs in BYPASS.
///
/// Returns Vec<Option<u32>>, with None for TAPs in BYPASS.
fn extract_idcodes(dr: &[bool]) -> Result<Vec<Option<IDCODE>>> {
    let mut dr = &dr[..];
    let mut idcodes = Vec::new();
    while !dr.is_empty() {
        if dr[0] {
            if dr.len() < 32 {
                log::error!("Truncated IDCODE: {:02X?}", dr);
                return Err(Error::InvalidIDCODE);
            }
            let (idcode, rem) = drain_u32(dr)?;
            let idcode = IDCODE(idcode);
            if !idcode.valid() {
                log::error!("Invalid IDCODE: {:08X}", idcode.0);
                return Err(Error::InvalidIDCODE);
            }
            log::debug!("Found IDCODE: {}", idcode);
            idcodes.push(Some(idcode));
            dr = rem;
        } else {
            idcodes.push(None);
            log::debug!("Found bypass TAP");
            dr = &dr[1..];
        }
    }
    Ok(idcodes)
}

#[test]
fn test_extract_idcode() {
    assert_eq!(extract_idcodes(bv![
            // 0x3BA00477
            1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0,
            // 0x16410041
            1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0,
            // BYPASS
            0,
            // 41111043
            1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0,
            // BYPASS
            0
        ]).unwrap(),
        vec![
            Some(IDCODE(0x3BA00477)),
            Some(IDCODE(0x16410041)),
            None,
            Some(IDCODE(0x41111043)),
            None
        ]
    );
}

/// Best-effort extraction of IR lengths from a test-logic-reset IR chain `ir`,
/// which is known to contain `n_taps` TAPs (as discovered by scanning DR for IDCODEs).
///
/// If expected IR lengths are provided, specify them in `expected`, and they are
/// verified against the IR scan and then returned.
///
/// Valid IR registers in the capture must start with `10` (a 1 in the last-significant,
/// and therefore first, bit). However, IRs may contain `10` in other positions, so we
/// can only find a superset of all possible start positions. If this happens to match
/// the number of taps, or there is only one tap, we can find all IR lengths. Otherwise,
/// they must be provided, and are then checked.
///
/// This implementation is a port of the algorithm from:
/// https://github.com/GlasgowEmbedded/glasgow/blob/30dc11b2/
/// /software/glasgow/applet/interface/jtag_probe/__init__.py#L712
///
/// Returns Vec<usize>, with an entry for each TAP.
fn extract_ir_lens(ir: &[bool], n_taps: usize, expected: Option<&[usize]>)
    -> Result<Vec<usize>>
{
    // Find all `10` patterns which indicate potential IR start positions.
    let starts = ir.windows(2)
                   .enumerate()
                   .filter(|(_, w)| w[0] && !w[1])
                   .map(|(i, _)| i)
                   .collect::<Vec<usize>>();
    log::trace!("Possible IR start positions: {:?}", starts);

    if n_taps == 0 {
        log::error!("Cannot scan IR without at least one TAP");
        Err(Error::InvalidIR)
    } else if n_taps > starts.len() {
        // We must have at least as many `10` patterns as TAPs.
        log::error!("Fewer IRs detected than TAPs");
        Err(Error::InvalidIR)
    } else if starts[0] != 0 {
        // The chain must begin with a possible start location.
        log::error!("IR chain does not begin with a valid start pattern");
        Err(Error::InvalidIR)
    } else if let Some(expected) = expected {
        // If expected lengths are available, verify and return them.
        if expected.len() != n_taps {
            log::error!("Number of provided IR lengths ({}) does not match \
                         number of detected TAPs ({})", expected.len(), n_taps);
            Err(Error::InvalidIR)
        } else if expected.iter().sum::<usize>() != ir.len() {
            log::error!("Sum of provided IR lengths ({}) does not match \
                         length of IR scan ({} bits)",
                         expected.iter().sum::<usize>(), ir.len());
            Err(Error::InvalidIR)
        } else {
            let exp_starts = expected.iter()
                                     .scan(0, |a, &x| { let b = *a; *a += x; Some(b) })
                                     .collect::<Vec<usize>>();
            log::trace!("Provided IR start positions: {:?}", exp_starts);
            let unsupported = exp_starts.iter().filter(|s| !starts.contains(s)).count();
            if unsupported > 0 {
                log::error!("Provided IR lengths imply an IR start position \
                             which is not supported by the IR scan");
                Err(Error::InvalidIR)
            } else {
                log::debug!("Verified provided IR lengths against IR scan");
                Ok(exp_starts)
            }
        }
    } else if n_taps == 1 {
        // If there's only one TAP, this is easy.
        log::info!("Only one TAP detected, IR length {}", ir.len());
        Ok(vec![ir.len()])
    } else if n_taps == starts.len() {
        // Likewise if the number of possible starts matches the number of TAPs.
        let irlens = starts.windows(2).map(|w| w[1] - w[0]).collect();
        log::info!("IR lengths are unambiguous: {:?}", irlens);
        Ok(irlens)
    } else {
        log::error!("IR lengths are ambiguous and must be specified with --ir-lengths.");
        Err(Error::InvalidIR)
    }
}

/// Represents a series of JTAG sequences to be sent to a DAP.
/// The maximum number of sequences is limited by the DAP packet size, or at most 255.
#[derive(Clone)]
pub struct Sequences<'a> {
    dap: Option<&'a DAP>,
    num_sequences: usize,
    capture_length: usize,
    request: Vec<u8>,
}

impl<'a> Sequences<'a> {
    /// Create a new Sequences object without providing a DAP.
    /// Only used by unit tests.
    #[cfg(test)]
    fn new() -> Self {
        Sequences { dap: None, num_sequences: 0, capture_length: 0, request: Vec::new() }
    }

    /// Create a new Sequences object which can be sent to the provided DAP.
    pub fn with_dap(dap: &'a DAP) -> Self {
        Sequences {
            dap: Some(dap), num_sequences: 0, capture_length: 0, request: Vec::new()
        }
    }

    /// Add a new sequence to this set of sequences.
    ///
    /// len: number of clock cycles, from 1 to 64.
    /// tms: value to set TMS during this sequence.
    /// tdi: optional data to clock out TDI, bits in transmission order,
    ///      set to all-1 if None.
    /// capture: if true, capture TDO state during this sequence.
    pub fn add_sequence(&mut self, len: usize, tms: bool, tdi: Option<&[bool]>, capture: bool)
        -> Result<()>
    {
        // Check length is within CMSIS-DAP bounds.
        if len == 0 || len > 64 {
            log::error!("Sequence length must be between 1 and 64 bits, but is {}", len);
            return Err(Error::InvalidSequence);
        }

        // Check enough TDI data is specified for the length.
        if let Some(tdi) = tdi {
            if tdi.len() != len {
                log::error!("{} bits of TDI data supplied, expected {}", tdi.len(), len);
                return Err(Error::InvalidSequence);
            }
        }

        // Compute number of bytes of TDI data required for `len` clock cycles.
        let nbytes = (len + 7) / 8;

        // Check we won't exceed the DAP packet size limit.
        // 2 byte header (command ID and sequence count)
        // + current request length
        // + 1 byte of info for this sequence
        // + length of this sequence's TDI data
        if let Some(dap) = self.dap {
            if 2 + self.request.len() + 1 + nbytes > dap.packet_size() {
                log::error!("Request will cause sequence to exceed DAP packet size");
                return Err(Error::InvalidSequence);
            }
        }

        // If supplied, convert input [bool] into [u8], otherwise use dummy 0xFF bytes.
        let tdi = match tdi {
            Some(tdi) => bits_to_bytes(tdi),
            None => vec![0xff; nbytes],
        };

        // Form info byte, encoding len to represent 64 as 0.
        let mut info = if len == 64 { 0 } else { len as u8 };
        if tms {
            info |= 1 << 6;
        }
        if capture {
            info |= 1 << 7;
            self.capture_length += len;
        }

        // Write info and TDI data to request.
        self.request.extend_from_slice(&[info]);
        self.request.extend_from_slice(&tdi[..]);
        self.num_sequences += 1;

        Ok(())
    }

    /// Add multiple sequences as required to reach the desired total bit length.
    ///
    /// len: number of clock cycles, limited only by maximum packet size.
    /// tms: value to set TMS during these sequence.
    /// tdi: optional data to clock out TDI, bits in transmission order,
    ///      set to all-1 if None.
    /// capture: if true, capture TDO state during this sequence.
    pub fn add_sequences(&mut self, len: usize, tms: bool, tdi: Option<&[bool]>, capture: bool)
        -> Result<()>
    {
        // Check correct length of data is provided.
        if let Some(tdi) = tdi {
            if tdi.len() != len {
                log::error!("{} bits of TDI data supplied, expected {}", tdi.len(), len);
                return Err(Error::InvalidSequence);
            }
        }

        let mut idx = 0;
        while idx < len {
            let n = usize::min(len - idx, 64);
            match tdi {
                Some(tdi) => self.add_sequence(n, tms, Some(&tdi[idx..idx+n]), capture)?,
                None      => self.add_sequence(n, tms, None, capture)?,
            }
            idx += 64;
        }
        Ok(())
    }

    /// Shift out `tms` bits.
    ///
    /// Internally each run of identical bits is collapsed into a single request.
    pub fn mode(mut self, tms: &[bool]) -> Result<Self> {
        if tms.is_empty() {
            return Ok(self);
        }

        let mut val = tms[0];
        let mut len = 1;

        for bit in &tms[1..] {
            if *bit == val {
                len += 1;
            } else {
                self.add_sequences(len, val, None, false)?;
                val = *bit;
                len = 1;
            }
        }

        self.add_sequences(len, val, None, false)?;

        Ok(self)
    }

    /// Write bits to TDI without capturing TDO state.
    ///
    /// If `exit` is true, the first bits have TMS=0 and the final bit has TMS=1,
    /// otherwise all bits have TMS=0.
    pub fn write(mut self, tdi: &[bool], exit: bool) -> Result<Self> {
        let len = tdi.len();
        if exit {
            self.add_sequences(len - 1, false, Some(&tdi[..len-1]), false)?;
            self.add_sequences(1, true, Some(&[tdi[len-1]]), false)?;
        } else {
            self.add_sequences(len, false, Some(tdi), false)?;
        }
        Ok(self)
    }

    /// Read `n` bits from TDO, writing all 1 bits to TDI.
    ///
    /// If `exit` is true, the first bits have TMS=0 and the final bit has TMS=1,
    /// otherwise all bits have TMS=0.
    pub fn read(mut self, n: usize, exit: bool) -> Result<Self> {
        if exit {
            self.add_sequences(n - 1, false, None, true)?;
            self.add_sequences(1, true, None, true)?;
        } else {
            self.add_sequences(n, false, None, true)?;
        }
        Ok(self)
    }

    /// Write bits to TDI and capture output bits from TDO.
    ///
    /// If `exit` is true, the first bits have TMS=0 and the final bit has TMS=1,
    /// otherwise all bits have TMS=0.
    pub fn exchange(mut self, tdi: &[bool], exit: bool) -> Result<Self> {
        let len = tdi.len();
        if exit {
            self.add_sequences(len - 1, false, Some(&tdi[..len-1]), true)?;
            self.add_sequences(1, true, Some(&[tdi[len-1]]), true)?;
        } else {
            self.add_sequences(len, false, Some(tdi), true)?;
        }
        Ok(self)
    }

    /// Generate the bytes representing the DAP request packet for these sequences.
    fn to_bytes(&self) -> Vec<u8> {
        let mut request = vec![self.num_sequences as u8];
        request.extend_from_slice(&self.request[..]);
        request
    }

    /// Run these sequences, returning all captured TDO bits.
    pub fn run(self) -> Result<Vec<bool>> {
        if let Some(dap) = self.dap {
            let request = self.to_bytes();
            let result = dap.jtag_sequence(&request[..])?;
            let expected = (self.capture_length + 7)/8;
            if result.len() != expected {
                log::error!("Expected {} bytes from probe, but got {}", expected, result.len());
                Err(Error::UnexpectedJTAGLength)
            } else {
                Ok(bytes_to_bits(&result, self.capture_length)?)
            }
        } else {
            Err(Error::NoDAPProvided)
        }
    }
}

#[test]
fn test_sequences() {
    // 64 bits only requires 1 sequence.
    let mut seqs = Sequences::new();
    seqs.add_sequences(64, false, None, false).unwrap();
    assert_eq!(
        seqs.to_bytes(),
        &[1, 0b0_0_00000000, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
    );

    // 65 bits should require 2 sequences.
    let mut seqs = Sequences::new();
    seqs.add_sequences(65, false, None, false).unwrap();
    assert_eq!(
        seqs.to_bytes(),
        &[2, 0b0_0_00000000, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
             0b0_0_00000001, 0xFF]
    );

    // 65 bits with TDI provided.
    let mut seqs = Sequences::new();
    seqs.add_sequences(65, false, Some(&[false; 65]), false).unwrap();
    assert_eq!(
        seqs.to_bytes(),
        &[2, 0b0_0_00000000, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0b0_0_00000001, 0x00]
    );
}

#[test]
fn test_sequences_mode() {
    // No TMS bits generates no sequences.
    assert_eq!(
        Sequences::new().mode(bv![]).unwrap().to_bytes(),
        &[0]
    );

    // One TMS bit generates one sequence with one clock.
    assert_eq!(
        Sequences::new().mode(bv![0]).unwrap().to_bytes(),
        &[1, 0b0_0_000001, 0xFF]
    );

    // Four identical TMS bits generates one sequence with four clocks.
    assert_eq!(
        Sequences::new().mode(bv![1, 1, 1, 1]).unwrap().to_bytes(),
        &[1, 0b0_1_000100, 0xFF]
    );

    // Three separate requests with 1, 3, and 1 clocks.
    assert_eq!(
        Sequences::new().mode(bv![0, 1, 1, 1, 0]).unwrap().to_bytes(),
        &[3, 0b0_0_000001, 0xFF, 0b0_1_000011, 0xFF, 0b0_0_000001, 0xFF]
    );
}

#[test]
fn test_sequences_write() {
    // No exit bit generates one sequence with 4 clocks.
    assert_eq!(
        Sequences::new().write(bv![1, 0, 1, 0], false).unwrap().to_bytes(),
        &[1, 0b0_0_000100, 0x05]
    );

    // Exit bit set generates two sequences with TMS set on second.
    assert_eq!(
        Sequences::new().write(bv![1, 0, 1, 0], true).unwrap().to_bytes(),
        &[2, 0b0_0_000011, 0x05, 0b0_1_000001, 0x00]
    );

    // Long sequence is split into multiple bytes.
    assert_eq!(
        Sequences::new().write(bv![1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1], false).unwrap().to_bytes(),
        &[1, 0b0_0_001100, 0xFF, 0x08]
    );
}

#[test]
fn test_sequences_read() {
    // No exit bit generates one sequence with 4 clocks, capture bit set.
    assert_eq!(
        Sequences::new().read(4, false).unwrap().to_bytes(),
        &[1, 0b1_0_000100, 0xFF]
    );

    // Exit bit set generates two sequences with TMS set on second.
    assert_eq!(
        Sequences::new().read(4, true).unwrap().to_bytes(),
        &[2, 0b1_0_000011, 0xFF, 0b1_1_000001, 0xFF]
    );

    // Long sequence results in multiple dummy bytes being sent.
    assert_eq!(
        Sequences::new().read(12, false).unwrap().to_bytes(),
        &[1, 0b1_0_001100, 0xFF, 0xFF]
    );
}

#[test]
fn test_sequences_exchange() {
    // No exit bit generates one sequence with 4 clocks.
    assert_eq!(
        Sequences::new().exchange(bv![1, 0, 1, 0], false).unwrap().to_bytes(),
        &[1, 0b1_0_000100, 0x05]
    );

    // Exit bit set generates two sequences with TMS set on second.
    assert_eq!(
        Sequences::new().exchange(bv![1, 0, 1, 0], true).unwrap().to_bytes(),
        &[2, 0b1_0_000011, 0x05, 0b1_1_000001, 0x00]
    );

    // Long sequence is split into multiple bytes.
    assert_eq!(
        Sequences::new().exchange(bv![1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1], false).unwrap().to_bytes(),
        &[1, 0b1_0_001100, 0xFF, 0x08]
    );
}

#[test]
fn test_sequences_multiple() {
    // Should see TMS=1 for 5 clocks, TMS=0 for 1 clock, TMS=1 for 1 clock, TMS=0 for 2 clocks,
    // then capture for 31 bits with TMS=0, then capture for 1 bit with TMS=1.
    assert_eq!(
        Sequences::new().mode(bv![1, 1, 1, 1, 1, 0, 1, 0, 0]).unwrap()
                        .read(32, true).unwrap()
                        .to_bytes(),
        &[6, 0b0_1_000101, 0xFF, 0b0_0_000001, 0xFF, 0b0_1_000001, 0xFF, 0b0_0_000010, 0xFF,
             0b1_0_011111, 0xFF, 0xFF, 0xFF, 0xFF, 0b1_1_000001, 0xFF]
    );
}
