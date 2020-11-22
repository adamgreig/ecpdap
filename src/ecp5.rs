//! This module implements ECP5-specific functionality on top of a JTAG TAP.

use std::convert::TryFrom;
use std::fmt;
use num_enum::{FromPrimitive, TryFromPrimitive};
use crate::jtag::{IDCODE, JTAGTAP, Error as JTAGError};
use crate::bitvec::{byte_to_bits, bytes_to_bits, bits_to_bytes, drain_u32, Error as BitvecError};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("JTAG error")]
    JTAG(#[from] JTAGError),
    #[error("Bitvec error")]
    Bitvec(#[from] BitvecError),
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

/// IDCODEs for all ECP5 device types.
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[allow(non_camel_case_types)]
#[repr(u32)]
pub enum ECP5IDCODE {
    LFE5U_12 = 0x21111043,
    LFE5U_25 = 0x41111043,
    LFE5U_45 = 0x41112043,
    LFE5U_85 = 0x41113043,
    LFE5UM_25 = 0x01111043,
    LFE5UM_45 = 0x01112043,
    LFE5UM_85 = 0x01113043,
    LFE5UM5G_25 = 0x81111043,
    LFE5UM5G_45 = 0x81112043,
    LFE5UM5G_85 = 0x81113043,
}

impl ECP5IDCODE {
    pub fn try_from_idcode(idcode: &IDCODE) -> Option<Self> {
        Self::try_from(idcode.0).ok()
    }

    pub fn name(&self) -> &'static str {
        match self {
            ECP5IDCODE::LFE5U_12 => "LFE5U-12",
            ECP5IDCODE::LFE5U_25 => "LFE5U-25",
            ECP5IDCODE::LFE5U_45 => "LFE5U-45",
            ECP5IDCODE::LFE5U_85 => "LFE5U-85",
            ECP5IDCODE::LFE5UM_25 => "LFE5UM-25",
            ECP5IDCODE::LFE5UM_45 => "LFE5UM-45",
            ECP5IDCODE::LFE5UM_85 => "LFE5UM-85",
            ECP5IDCODE::LFE5UM5G_25 => "LFE5UM5G-25",
            ECP5IDCODE::LFE5UM5G_45 => "LFE5UM5G-45",
            ECP5IDCODE::LFE5UM5G_85 => "LFE5UM5G-85",
        }
    }
}

/// All known ECP5 JTAG instructions.
#[derive(Copy, Clone, Debug)]
#[allow(unused, non_camel_case_types)]
#[repr(u8)]
enum Command {
    ISC_NOOP = 0xFF,
    READ_ID  = 0xE0,
    USERCODE = 0xC0,
    LSC_READ_STATUS = 0x3C,
    LSB_CHECK_BUSY = 0xF0,
    LSC_REFRESH = 0x79,
    ISC_ENABLE = 0xC6,
    ISC_ENABLE_X = 0x74,
    ISC_DISABLE = 0x26,
    ISC_PROGRAM_USERCODE = 0xC2,
    ISC_ERASE = 0x0E,
    ISC_PROGRAM_DONE = 0x5E,
    ISC_PROGRAM_SECURITY = 0xCE,
    LSC_INIT_ADDRESS = 0x46,
    LSC_WRITE_ADDRESS = 0xB4,
    LSC_BITSTREAM_BURST = 0x7A,
    LSC_PROG_INCR_RTI = 0x82,
    LSC_PROG_INCR_ENC = 0xB6,
    LSC_PROG_INCR_CMP = 0xB8,
    LSC_PROG_INCR_CNE = 0xBA,
    LSC_VERIFY_INCR_RTI = 0x6A,
    LSC_PROG_CTRL0 = 0x22,
    LSC_READ_CTRL0 = 0x20,
    LSC_RESET_CRC = 0x3B,
    LSC_READ_CRC = 0x60,
    LSC_PROG_SED_CRC = 0xA2,
    LSC_READ_SED_CRC = 0xA4,
    LSC_PROG_PASSWORD = 0xF1,
    LSC_READ_PASSWORD = 0xF2,
    LSC_SHIFT_PASSWORD = 0xBC,
    LSC_PROG_CIPHER_KEY = 0xF3,
    LSC_READ_CIPHER_KEY = 0xF4,
    LSC_PROG_FEATURE = 0xE4,
    LSC_READ_FEATURE = 0xE7,
    LSC_PROG_FEABITS = 0xF8,
    LSC_READ_FEABITS = 0xFB,
    LSC_PROG_OTP = 0xF9,
    LSC_READ_OTP = 0xFA,
    LSC_BACKGROUND_SPI = 0x3A,
}

impl Command {
    pub fn bits(&self) -> Vec<bool> {
        byte_to_bits(*self as u8)
    }
}

#[derive(Copy, Clone, Debug, FromPrimitive)]
#[allow(unused, non_camel_case_types)]
#[repr(u8)]
pub enum BSEError {
    #[num_enum(default)]
    NoError = 0,
    IDError = 1,
    CMDError = 2,
    CRCError = 3,
    PRMBError = 4,
    ABRTError = 5,
    OVFLError = 6,
    SDMError = 7,
}

#[derive(Copy, Clone, Debug)]
#[allow(unused, non_camel_case_types)]
#[repr(u8)]
pub enum ConfigTarget {
    SRAM = 0,
    eFuse = 1,
    Unknown = 0xFF,
}

#[derive(Copy, Clone)]
pub struct Status(u32);

impl Status {
    pub fn new(word: u32) -> Self {
        Self(word)
    }

    pub fn transparent(&self) -> bool {
        (self.0 & 1) == 1
    }

    pub fn config_target(&self) -> ConfigTarget {
        match (self.0 >> 1) & 0b111 {
            0 => ConfigTarget::SRAM,
            1 => ConfigTarget::eFuse,
            _ => ConfigTarget::Unknown,
        }
    }

    pub fn jtag_active(&self) -> bool {
        ((self.0 >> 4) & 1) == 1
    }

    pub fn pwd_protection(&self) -> bool {
        ((self.0 >> 5) & 1) == 1
    }

    pub fn decrypt_enable(&self) -> bool {
        ((self.0 >> 7) & 1) == 1
    }

    pub fn done(&self) -> bool {
        ((self.0 >> 8) & 1) == 1
    }

    pub fn isc_enable(&self) -> bool {
        ((self.0 >> 9) & 1) == 1
    }

    pub fn write_enable(&self) -> bool {
        ((self.0 >> 10) & 1) == 1
    }

    pub fn read_enable(&self) -> bool {
        ((self.0 >> 11) & 1) == 1
    }

    pub fn busy(&self) -> bool {
        ((self.0 >> 12) & 1) == 1
    }

    pub fn fail(&self) -> bool {
        ((self.0 >> 13) & 1) == 1
    }

    pub fn feature_otp(&self) -> bool {
        ((self.0 >> 14) & 1) == 1
    }

    pub fn decrypt_only(&self) -> bool {
        ((self.0 >> 15) & 1) == 1
    }

    pub fn pwd_enable(&self) -> bool {
        ((self.0 >> 16) & 1) == 1
    }

    pub fn encrypt_preamble(&self) -> bool {
        ((self.0 >> 20) & 1) == 1
    }

    pub fn standard_preamble(&self) -> bool {
        ((self.0 >> 21) & 1) == 1
    }

    pub fn spi_m_fail_1(&self) -> bool {
        ((self.0 >> 22) & 1) == 1
    }

    pub fn bse_error(&self) -> BSEError {
        BSEError::from(((self.0 >> 23) & 0b111) as u8)
    }

    pub fn execution_error(&self) -> bool {
        ((self.0 >> 26) & 1) == 1
    }

    pub fn id_error(&self) -> bool {
        ((self.0 >> 27) & 1) == 1
    }

    pub fn invalid_command(&self) -> bool {
        ((self.0 >> 28) & 1) == 1
    }

    pub fn sed_error(&self) -> bool {
        ((self.0 >> 29) & 1) == 1
    }

    pub fn bypass_mode(&self) -> bool {
        ((self.0 >> 30) & 1) == 1
    }

    pub fn flow_through_mode(&self) -> bool {
        ((self.0 >> 31) & 1) == 1
    }
}

impl fmt::Debug for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!(
            "ECP5 Status: {:08X}
              Transparent: {}
              Config target: {:?}
              JTAG active: {}
              PWD protection: {}
              Decrypt enable: {}
              DONE: {}
              ISC enable: {}
              Write enable: {}
              Read enable: {}
              Busy: {}
              Fail: {}
              Feature OTP: {}
              Decrypt only: {}
              PWD enable: {}
              Encrypt preamble: {}
              Standard preamble: {}
              SPIm fail 1: {}
              BSE error: {:?}
              Execution error: {}
              ID error: {}
              Invalid command: {}
              SED error: {}
              Bypass mode: {}
              Flow-through mode: {}",
            self.0, self.transparent(), self.config_target(),
            self.jtag_active(), self.pwd_protection(), self.decrypt_enable(),
            self.done(), self.isc_enable(), self.write_enable(),
            self.read_enable(), self.busy(), self.fail(), self.feature_otp(),
            self.decrypt_only(), self.pwd_enable(), self.encrypt_preamble(),
            self.standard_preamble(), self.spi_m_fail_1(), self.bse_error(),
            self.execution_error(), self.id_error(), self.invalid_command(),
            self.sed_error(), self.bypass_mode(), self.flow_through_mode()))
    }
}

/// ECP5 FPGA manager
pub struct ECP5 {
    tap: JTAGTAP,
}

impl ECP5 {
    pub fn new(tap: JTAGTAP) -> Self {
        ECP5 { tap }
    }

    pub fn status(&mut self) -> Result<Status> {
        self.command(Command::LSC_READ_STATUS)?;
        let data = self.tap.read_dr(32, true)?;
        let (status, _) = drain_u32(&data)?;
        Ok(Status::new(status))
    }

    /// Program the ECP5 configuration SRAM.
    ///
    /// The ECP5 will be reset and start configuration after programming completion.
    pub fn program(&mut self, data: &[u8]) -> Result<()> {
        self.status()?;

        // Enable configuration
        self.command(Command::ISC_ENABLE)?;
        self.tap.run_test_idle(50)?;

        let status = self.status()?;
        println!("Cfg starting, {:?}", status);

        self.command(Command::LSC_BITSTREAM_BURST)?;

        // Load in entire bitstream
        for group in data.chunks(55) {
            for chunk in group.chunks(6) {
                let data: Vec<u8> = chunk.iter().map(|x| x.reverse_bits()).collect();
                let bits = bytes_to_bits(&data, data.len() * 8)?;
                self.tap.write_dr(&bits, false)?;
            }
        }

        // Enter Update-DR and return to Run-Test/Idle
        self.tap.run_test_idle(0)?;

        // Disable configuration
        self.command(Command::ISC_DISABLE)?;
        self.tap.run_test_idle(50)?;

        let status = self.status()?;
        println!("Cfg done, {:?}", status);

        Ok(())
    }

    fn command(&mut self, command: Command) -> Result<()> {
        Ok(self.tap.write_ir(&command.bits())?)
    }
}
