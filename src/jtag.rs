//! This module implements JTAG functionality on top of the DAP_JTAG_Sequence
//! command provided by the DAP module.
//!
//! The JTAG chain may be scanned for TAPs, and then a TAP interface created to
//! address that specific device.

use std::time::Duration;
use crate::dap::{DAP, Error as DAPError};
use crate::bitvec::{bits_to_bytes, bytes_to_bits, drain_u32, Error as BitVecError};
use crate::ecp5::ECP5IDCODE;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Unexpected JTAG length returned from probe.")]
    UnexpectedJTAGLength,
    #[error("Internal error: invalid JTAG sequence.")]
    InvalidSequence,
    #[error("JTAG scan chain broken: check connection or try a larger --scan-chain-length.")]
    ScanChainBroken,
    #[error("Internal error: Tried to run sequences without providing a DAP.")]
    NoDAPProvided,
    #[error("Internal error: Unexpected JTAG state.")]
    BadState,
    #[error("Invalid IDCODE detected.")]
    InvalidIDCODE,
    #[error("Error scanning IR lengths.")]
    InvalidIR,
    #[error("Invalid index: does not exist in JTAG scan chain.")]
    InvalidTAPIndex,
    #[error("Provided IR command was the wrong length for the TAP.")]
    WrongIRLength,
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

    /// Scan JTAG chain, detecting TAPs and their IDCODEs and IR lengths.
    ///
    /// If IR lengths for each TAP are known, provide them in `ir_lengths`.
    ///
    /// Returns a new JTAGChain, which contains detected IDCODEs and
    /// can be used to create a JTAGTAP at a specific index.
    pub fn scan(&mut self, ir_lengths: Option<&[usize]>) -> Result<JTAGChain> {
        let (ir, dr) = self.reset_scan()?;
        let idcodes = extract_idcodes(&dr)?;
        let irlens = extract_ir_lengths(&ir, idcodes.len(), ir_lengths)?;
        Ok(JTAGChain::new(&idcodes, &irlens))
    }

    /// Create a new JTAG TAP, allowing read/write access to a single TAP
    /// on the chain.
    pub fn to_tap(self, chain: JTAGChain, index: usize) -> Result<JTAGTAP> {
        if index < chain.n_taps() {
            Ok(JTAGTAP::new(self, chain, index))
        } else {
            log::error!("Requested TAP {}, but there are only {} TAPs.", index, chain.n_taps());
            Err(Error::InvalidTAPIndex)
        }
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

    /// Enter Run-Test/Idle and remain there for `n` clock cycles.
    pub fn run_test_idle(&mut self, n: usize) -> Result<()> {
        self.enter_run_test_idle()?;
        self.mode(&vec![false; n])
    }

    /// Enter Shift-DR state from Test-Logic-Reset, Run-Test/Idle, Update*, or Pause*.
    pub fn enter_shift_dr(&mut self) -> Result<()> {
        log::trace!("JTAG state: {:?} -> ShiftDR", self.state);
        match self.state {
            TAPState::ShiftDR                       => (),
            TAPState::TestLogicReset                => self.mode(bv![0, 1, 0, 0])?,
            TAPState::RunTestIdle                   => self.mode(bv![1, 0, 0])?,
            TAPState::Exit1DR                       => self.mode(bv![0, 1, 0])?,
            TAPState::PauseIR                       => self.mode(bv![1, 1, 1, 0, 0])?,
            TAPState::PauseDR                       => self.mode(bv![1, 0])?,
            TAPState::UpdateIR | TAPState::UpdateDR => self.mode(bv![1, 0, 0])?,
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
            TAPState::Exit1IR                       => self.mode(bv![0, 1, 0])?,
            TAPState::PauseIR                       => self.mode(bv![1, 0])?,
            TAPState::PauseDR                       => self.mode(bv![1, 1, 1, 1, 0, 0])?,
            TAPState::UpdateIR | TAPState::UpdateDR => self.mode(bv![1, 1, 0, 0])?,
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

    /// Move to Shift-IR, write `tdi` to IR, then enter Update-IR.
    pub fn write_ir(&mut self, tdi: &[bool]) -> Result<()> {
        log::debug!("Writing to IR: {:02X?}", bits_to_bytes(tdi));
        self.enter_shift_ir()?;
        self.write(tdi, true)?;
        Ok(())
    }

    /// Move to Shift-IR, read `n` bits of IR while writing 0xFF, then enter Update-IR.
    /// Returns the captured bits from TDO.
    pub fn read_ir(&mut self, n: usize) -> Result<Vec<bool>> {
        log::debug!("Reading from IR...");
        self.enter_shift_ir()?;
        let tdo = self.read(n, true)?;
        log::trace!("Read from IR: {:02X?}", bits_to_bytes(&tdo));
        Ok(tdo)
    }

    /// Move to Shift-IR, write `tdi` to IR while capturing TDO, then enter Update-IR.
    /// Returns the captured bits from TDO.
    pub fn exchange_ir(&mut self, tdi: &[bool]) -> Result<Vec<bool>> {
        log::debug!("Exchanging with IR: {:02X?}", bits_to_bytes(tdi));
        self.enter_shift_ir()?;
        let tdo = self.exchange(tdi, true)?;
        log::trace!("Exchanged from IR: {:02X?}", bits_to_bytes(&tdo));
        Ok(tdo)
    }

    /// Move to Shift-DR, write `tdi` to DR, then either enter Update-DR
    /// if `update` is true, otherwise enter Exit1-DR.
    pub fn write_dr(&mut self, tdi: &[bool], update: bool) -> Result<()> {
        log::debug!("Writing to DR: {:?}", bits_to_bytes(tdi));
        self.enter_shift_dr()?;
        self.write(tdi, update)?;
        Ok(())
    }

    /// Move to Shift-DR, read `n` bits of DR while writing 0xFF, then either enter Update-DR
    /// if `update` is true, otherwise enter Exit1-DR.
    /// Returns the captured bits from TDO.
    pub fn read_dr(&mut self, n: usize, update: bool) -> Result<Vec<bool>> {
        log::debug!("Reading from DR...");
        self.enter_shift_dr()?;
        let tdo = self.read(n, update)?;
        log::trace!("Read from DR: {:?}", bits_to_bytes(&tdo));
        Ok(tdo)
    }

    /// Move to Shift-DR, write `tdi` to DR while capturing TDO, then either enter Update-DR
    /// if `update` is true, otherwise enter Exit1-DR.
    /// Returns the captured bits from TDO.
    pub fn exchange_dr(&mut self, tdi: &[bool], update: bool) -> Result<Vec<bool>> {
        log::debug!("Exchanging with DR: {:?}", bits_to_bytes(tdi));
        self.enter_shift_dr()?;
        let tdo = self.exchange(tdi, update)?;
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

    /// Shortcut for reading `n` bits from the current register.
    /// If `update` is true, exit into the relevant Update state,
    /// otherwise exit to Exit1.
    ///
    /// Must already be in a Shift* state, and `self.state` is updated upon completion.
    fn read(&mut self, n: usize, update: bool) -> Result<Vec<bool>> {
        let new_state = match (self.state, update) {
            (TAPState::ShiftIR, true) => TAPState::UpdateIR,
            (TAPState::ShiftIR, false) => TAPState::Exit1IR,
            (TAPState::ShiftDR, true) => TAPState::UpdateDR,
            (TAPState::ShiftDR, false) => TAPState::Exit1DR,
            _ => return Err(Error::BadState),
        };
        log::trace!("JTAG state: {:?} -> {:?}, reading {} bits", self.state, new_state, n);

        let mut seq = self.sequences().read(n, true)?;
        if update {
            seq = seq.mode(bv![1])?;
        }
        let result = seq.run()?;

        self.state = new_state;
        Ok(result)
    }

    /// Shortcut for writing `tdi` bits to the current register.
    /// If `update` is true, exit into the relevant Update state,
    /// otherwise exit to Exit1.
    ///
    /// Must already be in a Shift* state, and `self.state` is updated upon completion.
    fn write(&mut self, tdi: &[bool], update: bool) -> Result<()> {
        let new_state = match (self.state, update) {
            (TAPState::ShiftIR, true) => TAPState::UpdateIR,
            (TAPState::ShiftIR, false) => TAPState::Exit1IR,
            (TAPState::ShiftDR, true) => TAPState::UpdateDR,
            (TAPState::ShiftDR, false) => TAPState::Exit1DR,
            _ => return Err(Error::BadState),
        };
        log::trace!("JTAG state: {:?} -> {:?}, writing {} bits", self.state, new_state, tdi.len());
        let mut seq = self.sequences().write(tdi, true)?;
        if update {
            seq = seq.mode(bv![1])?;
        }
        seq.run()?;
        self.state = new_state;
        Ok(())
    }

    /// Shortcut for exchanging `tdi` bits with the current register.
    /// If `update` is true, exit into the relevant Update state,
    /// otherwise exit to Exit1.
    ///
    /// Must already be in a Shift* state, and `self.state` is updated upon completion.
    fn exchange(&mut self, tdi: &[bool], update: bool) -> Result<Vec<bool>> {
        let new_state = match (self.state, update) {
            (TAPState::ShiftIR, true) => TAPState::UpdateIR,
            (TAPState::ShiftIR, false) => TAPState::Exit1IR,
            (TAPState::ShiftDR, true) => TAPState::UpdateDR,
            (TAPState::ShiftDR, false) => TAPState::Exit1DR,
            _ => return Err(Error::BadState),
        };
        log::trace!("JTAG state: {:?} -> {:?}, writing {} bits", self.state, new_state, tdi.len());
        let mut seq = self.sequences().exchange(tdi, true)?;
        if update {
            seq = seq.mode(bv![1])?;
        }
        let tdo = seq.run()?;
        self.state = new_state;
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
pub struct IDCODE(pub u32);

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

    /// Convert to an ECP5IDCODE if this IDCODE belongs to an ECP5.
    pub fn try_to_ecp5(&self) -> Option<ECP5IDCODE> {
        ECP5IDCODE::try_from_idcode(self)
    }
}

impl std::fmt::Display for IDCODE {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(ecp5) = self.try_to_ecp5() {
            write!(f, "0x{:08X} ECP5 {}", self.0, ecp5.name())
        } else if let Some(mfn) = self.manufacturer_name() {
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

    // Example IDCODE from an ARM Cortex-M.
    let idcode = IDCODE(0x3BA00477);
    assert_eq!(idcode.manufacturer_name(), Some("ARM Ltd"));
    assert_eq!(idcode.try_to_ecp5(), None);
    assert_eq!(format!("{}", idcode), "0x3BA00477 (ARM Ltd)");

    // Example IDCODE from an ECP5.
    let idcode = IDCODE(0x41111043);
    assert_eq!(idcode.manufacturer_name(), Some("Lattice Semi."));
    assert_eq!(idcode.try_to_ecp5(), Some(ECP5IDCODE::LFE5U_25));
    assert_eq!(format!("{}", idcode), "0x41111043 ECP5 LFE5U-25");
}

/// Stores information about a JTAG scan chain,
/// including detected IDCODEs and IR lengths.
pub struct JTAGChain {
    idcodes: Vec<Option<IDCODE>>,
    irlens: Vec<usize>,
}

impl JTAGChain {
    pub fn new(idcodes: &[Option<IDCODE>], irlens: &[usize]) -> Self {
        JTAGChain { idcodes: idcodes.to_vec(), irlens: irlens.to_vec() }
    }

    /// Return the number of TAPs in the chain.
    pub fn n_taps(&self) -> usize {
        self.idcodes.len()
    }

    /// Return the detected IDCODEs.
    /// The returned slice has one entry per TAP;
    /// TAPs which were in BYPASS are represented by None
    /// while those that output an IDCODE are represented
    /// by Some(IDCODE).
    pub fn idcodes(&self) -> &[Option<IDCODE>] {
        &self.idcodes
    }

    /// Return the IR lengths for each TAP.
    pub fn irlens(&self) -> &[usize] {
        &self.irlens
    }

    /// Format each TAP into a String suitable for display.
    pub fn to_lines(&self) -> Vec<String> {
        self.idcodes().iter().zip(self.irlens()).enumerate().map(|(idx, (idcode, irlen))| {
            match idcode {
                Some(idcode) => format!("{}: {} [IR length: {}]", idx, idcode, irlen),
                None         => format!("{}: [Bypass, IR length: {}]", idx, irlen),
            }
        }).collect()
    }

    /// Check whether the provided TAP index is an ECP5.
    pub fn check_idx(&self, index: usize) -> bool {
        match self.idcodes().iter().nth(index) {
            Some(tap) => match tap {
                Some(idcode) => idcode.try_to_ecp5().is_some(),
                None => false,
            },
            None => false,
        }
    }

    /// If possible, return the unique TAP index for an ECP5
    /// device in this chain.
    pub fn auto_idx(&self) -> Option<usize> {
        let ecp5_idxs: Vec<usize> = self.idcodes()
                                        .iter()
                                        .enumerate()
                                        .filter_map(|(idx, id)| id.map(|id| (idx, id)))
                                        .filter_map(|(idx, id)| id.try_to_ecp5().map(|_| idx))
                                        .collect();
        let len = ecp5_idxs.len();
        if len == 0 {
            log::info!("No ECP5 found in JTAG chain");
            None
        } else if len > 1 {
            log::info!("Multiple ECP5 devices found in JTAG chain, specify one using --tap");
            None
        } else {
            let index = ecp5_idxs.first().unwrap();
            log::debug!("Automatically selecting ECP5 at TAP {}", index);
            Some(*index)
        }
    }
}

/// Represents a single TAP in a JTAG scan chain,
/// with methods to read and write this TAP while
/// other TAPs are placed into BYPASS.
pub struct JTAGTAP {
    jtag: JTAG,
    chain: JTAGChain,
    index: usize,
    ir_len: usize,
    ir_prefix: usize,
    ir_suffix: usize,
    dr_prefix: usize,
    dr_suffix: usize,
}

impl JTAGTAP {
    /// Create a new JTAGTAP from the provided chain and index.
    /// The index must be within the number of taps in the chain.
    pub(crate) fn new(jtag: JTAG, chain: JTAGChain, index: usize) -> Self {
        let irl = chain.irlens();
        assert!(index < irl.len(), "index not contained in chain");
        let ir_len = irl[index];
        let ir_prefix = irl[..index].iter().sum();
        let ir_suffix = irl[index + 1..].iter().sum();
        let dr_prefix = index;
        let dr_suffix = irl.len() - index - 1;
        JTAGTAP { jtag, chain, index, ir_len, ir_prefix, ir_suffix, dr_prefix, dr_suffix }
    }

    /// Returns the index of this TAP.
    pub fn index(&self) -> usize {
        self.index
    }

    /// Consume the JTAGTAP and return its JTAG and JTAGChain.
    pub fn release(self) -> (JTAG, JTAGChain) {
        (self.jtag, self.chain)
    }

    /// Reset JTAG TAPs, entering Test-Logic-Reset.
    pub fn test_logic_reset(&mut self) -> Result<()> {
        self.jtag.enter_test_logic_reset()
    }

    /// Move to Run-Test/Idle and remain there for `n` clock cycles.
    pub fn run_test_idle(&mut self, n: usize) -> Result<()> {
        self.jtag.run_test_idle(n)
    }

    /// Move to Shift-IR, write `ir` to IR, then enter Update-IR.
    pub fn write_ir(&mut self, ir: &[bool]) -> Result<()> {
        if ir.len() != self.ir_len {
            return Err(Error::WrongIRLength);
        }

        // Add BYPASS commands before and after our IR as required.
        let mut tdi = vec![true; self.ir_prefix];
        tdi.extend_from_slice(ir);
        tdi.extend_from_slice(&vec![true; self.ir_suffix]);

        self.jtag.write_ir(&tdi)
    }

    /// Move to Shift-IR, read the IR while writing 0xFF, then enter Update-IR.
    /// Returns the captured bits from TDO, leaving BYPASS in all IRs.
    pub fn read_ir(&mut self) -> Result<Vec<bool>> {
        let tdo = self.jtag.read_ir(self.ir_prefix + self.ir_len + self.ir_suffix)?;

        // Cut out prefix and suffix IR data.
        let tdo = tdo[self.ir_prefix..self.ir_prefix+self.ir_len].to_vec();
        Ok(tdo)
    }

    /// Move to Shift-IR, write `ir` to IR while capturing TDO, then enter Update-IR.
    /// Returns the captured bits from TDO.
    pub fn exchange_ir(&mut self, ir: &[bool]) -> Result<Vec<bool>> {
        if ir.len() != self.ir_len {
            return Err(Error::WrongIRLength);
        }

        // Add BYPASS commands before and after our IR as required.
        let mut tdi = vec![true; self.ir_prefix];
        tdi.extend_from_slice(ir);
        tdi.extend_from_slice(&vec![true; self.ir_suffix]);

        let tdo = self.jtag.exchange_ir(&tdi)?;

        // Cut out prefix and suffix IR data.
        let tdo = tdo[self.ir_prefix..self.ir_prefix+self.ir_len].to_vec();
        Ok(tdo)
    }

    /// Move to Shift-DR, write `dr` to DR, then enter either Update-DR or Exit1-DR.
    pub fn write_dr(&mut self, dr: &[bool], update: bool) -> Result<()> {
        // Add dummy bits before and after our IR as required.
        let mut tdi = vec![true; self.dr_prefix];
        tdi.extend_from_slice(dr);
        tdi.extend_from_slice(&vec![true; self.dr_suffix]);

        self.jtag.write_dr(&tdi, update)?;
        Ok(())
    }

    /// Move to Shift-DR, read `n` bits of DR while writing 0xFF, then enter Update-DR.
    /// Returns the captured bits from TDO.
    pub fn read_dr(&mut self, n: usize, capture: bool) -> Result<Vec<bool>> {
        let tdo = self.jtag.read_dr(self.dr_prefix + n + self.dr_suffix, capture)?;
        let tdo = tdo[self.dr_prefix..self.dr_prefix+n].to_vec();
        Ok(tdo)
    }

    /// Move to Shift-DR, write `dr` to DR while capturing TDO, then enter Update-DR.
    /// Returns the captured bits from TDO.
    pub fn exchange_dr(&mut self, dr: &[bool], capture: bool) -> Result<Vec<bool>> {
        // Add dummy bits before and after our IR as required.
        let mut tdi = vec![true; self.dr_prefix];
        tdi.extend_from_slice(dr);
        tdi.extend_from_slice(&vec![true; self.dr_suffix]);

        let tdo = self.jtag.exchange_dr(&tdi, capture)?;
        let tdo = tdo[self.dr_prefix..self.dr_prefix+dr.len()].to_vec();
        Ok(tdo)
    }
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
/// Valid IRs in the capture must start with `10` (a 1 in the last-significant,
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
fn extract_ir_lengths(ir: &[bool], n_taps: usize, expected: Option<&[usize]>)
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
                Ok(starts_to_lens(&exp_starts, ir.len()))
            }
        }
    } else if n_taps == 1 {
        // If there's only one TAP, this is easy.
        log::info!("Only one TAP detected, IR length {}", ir.len());
        Ok(vec![ir.len()])
    } else if n_taps == starts.len() {
        // If the number of possible starts matches the number of TAPs,
        // we can unambiguously find all lengths.
        let irlens = starts_to_lens(&starts, ir.len());
        log::info!("IR lengths are unambiguous: {:?}", irlens);
        Ok(irlens)
    } else {
        log::error!("IR lengths are ambiguous and must be specified with --ir-lengths.");
        Err(Error::InvalidIR)
    }
}

#[test]
fn test_extract_ir_lengths() {
    // Passing case for error baseline with just one TAP
    assert!(extract_ir_lengths(bv![1, 0, 0, 0, 0, 0], 1, None).is_ok());
    // Error when n_taps is 0 instead
    assert!(extract_ir_lengths(bv![0, 0, 0, 0, 0, 0], 0, None).is_err());
    // Error when not enough start patterns for n_taps
    assert!(extract_ir_lengths(bv![0, 0, 0, 0, 0, 0], 1, None).is_err());
    assert!(extract_ir_lengths(bv![1, 0, 0, 0, 0, 0], 2, None).is_err());
    // Error when chain does not begin with a start pattern
    assert!(extract_ir_lengths(bv![0, 1, 0, 0, 0, 0], 1, None).is_err());
    // Passing case with provided lengths
    assert!(extract_ir_lengths(bv![1, 0, 0, 0, 1, 0], 2, Some(&[4, 2])).is_ok());
    // Error if number of lengths doesn't match number of TAPs
    assert!(extract_ir_lengths(bv![1, 0, 0, 0, 1, 0], 3, Some(&[4, 2])).is_err());
    // Error if sum of lengths doesn't match scan length
    assert!(extract_ir_lengths(bv![1, 0, 0, 0, 1, 0, 0], 2, Some(&[4, 2])).is_err());
    // Error if provided lengths don't match the IR data
    assert!(extract_ir_lengths(bv![1, 0, 0, 0, 1, 0], 2, Some(&[3, 3])).is_err());
    // Error if start patterns are ambiguous
    assert!(extract_ir_lengths(bv![1, 0, 1, 0, 1, 0], 2, None).is_err());

    // Extract length for one TAP
    assert_eq!(extract_ir_lengths(bv![1, 0, 0, 0, 0, 0], 1, None).unwrap(), vec![6]);
    // Extract lengths for two unambiguous TAPs
    assert_eq!(extract_ir_lengths(bv![1, 0, 0, 0, 1, 0], 2, None).unwrap(), vec![4, 2]);
    // Extract lengths for three unambiguous TAPs
    assert_eq!(extract_ir_lengths(bv![1, 0, 1, 0, 1, 0], 3, None).unwrap(), vec![2, 2, 2]);
    // Validate provided lengths
    assert_eq!(extract_ir_lengths(bv![1, 0, 1, 0, 1, 0], 3, Some(&[2, 2, 2])).unwrap(),
               vec![2, 2, 2]);
}

/// Convert a list of start positions to a list of lengths.
fn starts_to_lens(starts: &[usize], total: usize) -> Vec<usize> {
    let mut lens: Vec<usize> = starts.windows(2)
                                     .map(|w| w[1] - w[0])
                                     .collect();
    lens.push(total - lens.iter().sum::<usize>());
    lens
}

#[test]
fn test_starts_to_lens() {
    assert_eq!(starts_to_lens(&[0],       6), vec![6]);
    assert_eq!(starts_to_lens(&[0, 3],    6), vec![3, 3]);
    assert_eq!(starts_to_lens(&[0, 2, 4], 6), vec![2, 2, 2]);
}

/// Represents a series of JTAG sequences to be sent to a DAP.
/// The maximum number of sequences is limited by the DAP packet size, or at most 255.
#[derive(Clone)]
pub struct Sequences<'a> {
    dap: Option<&'a DAP>,
    num_sequences: usize,
    capture_lengths: Vec<usize>,
    request: Vec<u8>,
}

impl<'a> Sequences<'a> {
    /// Create a new Sequences object without providing a DAP.
    /// Only used by unit tests.
    #[cfg(test)]
    fn new() -> Self {
        Sequences { dap: None, num_sequences: 0, capture_lengths: Vec::new(), request: Vec::new() }
    }

    /// Create a new Sequences object which can be sent to the provided DAP.
    pub fn with_dap(dap: &'a DAP) -> Self {
        Sequences {
            dap: Some(dap), num_sequences: 0, capture_lengths: Vec::new(), request: Vec::new()
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
            self.capture_lengths.push(len);
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
            let expected_n_bytes = self.capture_lengths.iter().map(|l| (l+7)/8).sum::<usize>();
            if result.len() != expected_n_bytes {
                log::error!("Expected {} bytes from probe, but got {}",
                            expected_n_bytes, result.len());
                Err(Error::UnexpectedJTAGLength)
            } else {
                // The probe responds with byte-padded bits per request,
                // e.g., with three one-cycle request, three bytes are
                // returned, with one bit (the LSbit) set per byte.
                // We combine those back into a single Vec of bits.
                let mut bits = Vec::new();
                let mut bytes = &result[..];
                for l in self.capture_lengths.iter() {
                    bits.append(&mut bytes_to_bits(bytes, *l)?);
                    bytes = &bytes[(l+7)/8..];
                }
                Ok(bits)
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
