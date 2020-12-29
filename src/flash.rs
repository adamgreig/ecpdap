use std::convert::TryInto;
use crate::jtag::Error as JTAGError;
use crate::ecp5::Error as ECP5Error;
use crate::bitvec::Error as BitvecError;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Error during flash readback verification.")]
    ReadbackError,
    #[error("Could not determine flash capacity automatically.")]
    CannotDetectCapacity,
    #[error("Invalid manufacturer ID detected.")]
    InvalidManufacturer,
    #[error("Invalid SFDP header.")]
    InvalidSFDPHeader,
    #[error("Not enough SFDP header data read.")]
    NotEnoughSFDPHeader,
    #[error("Invalid parameter in SFDP parameter table.")]
    InvalidSFDPParams,
    #[error("ECP5 error")]
    ECP5(#[from] ECP5Error),
    #[error("JTAG error")]
    JTAG(#[from] JTAGError),
    #[error("Bitvec error")]
    Bitvec(#[from] BitvecError),
    #[error(transparent)]
    Other(#[from] anyhow::Error)
}

pub type Result<T> = std::result::Result<T, Error>;

/// Standard SPI flash command opcodes.
///
/// These are taken from the Winbond W25Q16JV datasheet, but most are
/// widely applicable. If SFDP is supported, it is used to discover
/// the relevant erase opcodes and sizes.
///
/// Only single I/O commands are listed.
#[derive(Copy, Clone, Debug)]
#[allow(unused)]
#[repr(u8)]
enum Command {
    // Core instruction set.
    // These commands are almost universally available.
    WriteEnable = 0x06,
    WriteDisable = 0x04,
    ReadData = 0x03,
    PageProgram = 0x02,
    ReadStatusRegister1 = 0x05,
    WriteStatusRegister1 = 0x01,

    // Standard instruction set.
    // These commands are typically available.
    ReadJEDECID = 0x9F,
    FastRead = 0x0B,
    Powerdown = 0xB9,
    ReleasePowerdown = 0xAB,
    ReadDeviceID = 0x90,
    ChipErase = 0xC7,

    // Extended instruction set.
    // These commands may be available.
    ReadUniqueID = 0x4B,
    ReadSFDPRegister = 0x5A,
    ReadStatusRegister2 = 0x35,
    ReadStatusRegister3 = 0x15,
    WriteStatusRegister2 = 0x31,
    WriteStatusRegister3 = 0x11,
    EnableReset = 0x66,
    Reset = 0x99,
    ProgramSuspend = 0x75,
    ProgramResume = 0x7A,

    // Erase instructions.
    // The size affected by each erase operation can vary.
    // A typical value is 4kB sector, 32kB block erase 1, 64kB block erase 2.
    SectorErase = 0x20,
    BlockErase1 = 0x52,
    BlockErase2 = 0xD8,

    // Security/lock related instructions.
    EraseSecurityRegisters = 0x44,
    ProgramSecurityRegisters = 0x42,
    ReadSecurityRegisters = 0x48,
    IndividualBlockLock = 0x36,
    IndividualBlockUnlock = 0x39,
    ReadBlockLock = 0x3D,
    GlobalBlockLock = 0x7E,
    GlobalBlockUnlock = 0x98,
}

/// Store the ID read off an SPI flash memory.
///
/// The manufacturer ID and (long, 16-bit) device ID are read using the 0x9F command,
/// and the number of 0x7F continuation code bytes present before the manufacturer ID
/// is stored as `manufacturer_bank`.
///
/// The 64-bit unique ID is read using the 0x4B command.
#[derive(Copy, Clone, Debug)]
pub struct FlashID {
    pub manufacturer_bank: u8,
    pub manufacturer_id: u8,
    pub device_id_long: u16,
    pub device_id_short: u8,
    pub unique_id: u64,
}

impl FlashID {
    /// Look up a manufacturer name from the JEDEC ID.
    pub fn manufacturer_name(&self) -> Option<&'static str> {
        let (bank, id) = (self.manufacturer_bank, self.manufacturer_id & 0x7F);
        match jep106::JEP106Code::new(bank, id).get() {
            // Winbond acquired NEXCOM and so the ID 0xEF is commonly used for Winbond memory.
            Some(mfn) if mfn == "NEXCOM" => Some("Winbond/NEXCOM"),
            Some(mfn) => Some(mfn),
            None => None,
        }
    }
}

impl std::fmt::Display for FlashID {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let mfn = match self.manufacturer_name() {
            Some(mfn) => format!(" ({})", mfn),
            None => "".to_string(),
        };
        write!(f, "Manufacturer 0x{:02X}{}, Device 0x{:02X}/0x{:04X}, Unique ID {:016X}",
               self.manufacturer_id, mfn, self.device_id_short,
               self.device_id_long, self.unique_id)
    }
}

/// Trait for objects which provide access to SPI flash.
///
/// Providers only need to implement `exchange()`, which asserts CS, writes all the bytes
/// in `data`, then returns all the received bytes. If it provides a performance optimisation,
/// providers may also implement `write()`, which does not require the received data.
pub trait FlashAccess {
    /// Assert CS, write all bytes in `data` to the SPI bus, then de-assert CS.
    fn write(&mut self, data: &[u8]) -> Result<()> {
        // Default implementation uses `exchange()` and ignores the result data.
        self.exchange(data)?;
        Ok(())
    }

    /// Assert CS, write all bytes in `data` while capturing received data, then de-assert CS.
    ///
    /// Returns the received data.
    fn exchange(&mut self, data: &[u8]) -> Result<Vec<u8>>;
}

/// SPI Flash.
///
/// This struct provides methods for interacting with common SPI flashes.
pub struct Flash<'a, A: FlashAccess> {
    access: &'a mut A,
}

impl<'a, A: FlashAccess> Flash<'a, A> {
    /// Create a new Flash instance using the given FlashAccess provider.
    pub fn new(access: &'a mut A) -> Self {
        Flash { access }
    }

    /// Read the device's manufacturer ID and device IDs.
    ///
    /// This method additionally brings the flash out of powerdown and resets it.
    pub fn read_id(&mut self) -> Result<FlashID> {
        log::debug!("Reading SPI Flash ID");
        self.power_up()?;
        self.reset()?;
        let (manufacturer_bank, manufacturer_id, device_id_long) = self.read_jedec_id()?;
        let (_, _, device_id_short) = self.read_device_id()?;
        let unique_id = self.read_unique_id()?;
        Ok(FlashID {
            manufacturer_bank, manufacturer_id, device_id_short, device_id_long, unique_id
        })
    }

    /// Read SFDP JEDEC Basic Flash Parameter table from flash.
    ///
    /// This fails if SFDP is not supported by the flash memory.
    ///
    /// Depending on the version of SFDP supported, some fields may
    /// not be available.
    pub fn read_sfdp_basic_params(&mut self) -> Result<JEDECBasicParams> {
        Err(Error::InvalidSFDPParams)
    }

    /// Attempt to discover memory capacity in bytes.
    pub fn detect_capacity(&mut self) -> Result<usize> {
        log::debug!("Attempting to read SFDP information to determine capacity");
        let params = self.read_sfdp_jedec_params()?;
        let capacity = params.capacity_bytes() as usize;
        log::debug!("Detected capacitty as {}kB", capacity/1024);
        Ok(capacity)
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`
    pub fn read(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        self.fast_read(address, length)
    }

    /// Program the attached flash with `data` starting at `address`.
    ///
    /// If `verify` is true, also read-back the programmed data and
    /// return ReadbackError if it did not match what was written.
    pub fn program(&mut self, address: u32, data: &[u8], verify: bool) -> Result<()> {
        self.erase_for_data(address, data.len())?;
        self.program_data(address, data)?;
        if verify {
            let programmed = self.read(address, data.len())?;
            if programmed == data {
                Ok(())
            } else {
                Err(Error::ReadbackError)
            }
        } else {
            Ok(())
        }
    }

    /// Erase entire flash chip
    pub fn erase(&mut self) -> Result<()> {
        self.write_enable()?;
        self.chip_erase()?;
        self.wait_while_busy()?;
        Ok(())
    }

    /// Reset the attached flash
    pub fn reset(&mut self) -> Result<()> {
        self.command(Command::EnableReset)?;
        self.command(Command::Reset)
    }

    /// Unprotect the flash memory.
    pub fn unprotect(&mut self) -> Result<()> {
        let status1 = self.read_status1()?;
        let status2 = self.read_status2()?;
        let new_status1 = status1 & 0b11100011;
        self.write_enable()?;
        self.write_status1(new_status1, status2)?;
        Ok(())
    }

    /// Power down the flash.
    pub fn power_down(&mut self) -> Result<()> {
        log::debug!("Sending Powerdown command");
        self.command(Command::Powerdown)
    }

    /// Power up the flash.
    pub fn power_up(&mut self) -> Result<()> {
        log::debug!("Sending Release Powerdown command");
        self.command(Command::ReleasePowerdown)
    }

    /// Read SFDP JEDEC flash parameters
    pub fn read_sfdp_jedec_params(&mut self) -> Result<SFDPJEDECParams> {
        let header = self.read_sfdp_header()?;
        let params = header.params[0];
        if params.parameter_id != 0xFF00 || params.major != 0x01 {
            log::error!("Unexpected first SFDP parameter header, expected JEDEC 0x00 1.0");
            Err(Error::InvalidSFDPHeader)
        } else {
            let data = self.read_sfdp(params.ptp, params.plen * 4)?;
            Ok(SFDPJEDECParams::from_bytes(&data))
        }
    }

    fn erase_for_data(&mut self, address: u32, length: usize) -> Result<()> {
        // Adjust length and address to be 64K aligned
        const BLOCK_SIZE: usize = 64 * 1024;
        let length = length + (address as usize % BLOCK_SIZE) as usize;
        let address = address & 0xFF0000;
        let mut n_blocks = length / BLOCK_SIZE;
        if length % BLOCK_SIZE != 0 { n_blocks += 1 };
        for block in 0..n_blocks {
            self.write_enable()?;
            self.block_erase_2(address + (block * BLOCK_SIZE) as u32)?;
            self.wait_while_busy()?;
        }
        Ok(())
    }

    fn program_data(&mut self, address: u32, data: &[u8]) -> Result<()> {
        // Pad to obtain page alignment
        const PAGE_SIZE: usize = 256;
        let pad_length = address as usize % PAGE_SIZE;
        let tx = if pad_length != 0 {
            let mut tx = vec![0xFF; pad_length];
            tx.extend(data);
            tx
        } else {
            data.to_vec()
        };
        let address = address & 0xFFFF00;

        // Write pages
        for (idx, page_data) in tx.chunks(PAGE_SIZE).enumerate() {
            self.write_enable()?;
            self.page_program(address + (idx*PAGE_SIZE) as u32, page_data)?;
            self.wait_while_busy()?;
        }
        Ok(())
    }

    fn write_enable(&mut self) -> Result<()> {
        self.command(Command::WriteEnable)
    }

    fn page_program(&mut self, address: u32, data: &[u8]) -> Result<()> {
        assert!(!data.is_empty(), "Cannot program 0 bytes of data");
        assert!(data.len() <= 256, "Cannot program more than 256 bytes per page");
        let mut tx = address.to_be_bytes()[1..].to_vec();
        tx.extend(data);
        self.exchange(Command::PageProgram, &tx, 0)?;
        Ok(())
    }

    fn fast_read(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        let length = length + 1;
        let address = &address.to_be_bytes()[1..];
        self.exchange(Command::FastRead, address, length).map(|data| data[1..].to_vec())
    }

    fn chip_erase(&mut self) -> Result<()> {
        self.command(Command::ChipErase)
    }

    fn block_erase_2(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::BlockErase2, &address.to_be_bytes()[1..], 0)?;
        Ok(())
    }

    #[allow(dead_code)]
    fn block_erase_1(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::BlockErase1, &address.to_be_bytes()[1..], 0)?;
        Ok(())
    }

    #[allow(dead_code)]
    fn sector_erase(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::SectorErase, &address.to_be_bytes()[1..], 0)?;
        Ok(())
    }

    /// Reads the JEDEC manufacturer and long (16-bit) device IDs.
    ///
    /// The manufacturer ID may be prefixed with up to 13 of the
    /// continuation code 0x7F; the number of continuation codes
    /// is returned as the bank number.
    ///
    /// Returns (bank, manufacturer ID, device ID).
    fn read_jedec_id(&mut self) -> Result<(u8, u8, u16)> {
        // Attempt to read assuming a single-byte manufacturer ID.
        let data = self.exchange(Command::ReadJEDECID, &[], 3)?;
        if data[0] != 0x7F {
            Ok((0, data[0], u16::from_be_bytes([data[1], data[2]])))
        } else {
            // If the first byte is continuation, read 16 bytes, to allow
            // up to 13 continuation bytes, and then parse it to find the IDs.
            let data = self.exchange(Command::ReadJEDECID, &[], 16)?;
            for n in 1..=13 {
                if data[n] != 0x7F {
                    return Ok((n as u8, data[n], u16::from_be_bytes([data[n+1], data[n+2]])));
                }
            }
            log::error!("Found more than 11 continuation bytes in manufacturer ID");
            Err(Error::InvalidManufacturer)
        }
    }

    /// Reads the JEDEC manufacturer and short (8-bit) device IDs.
    ///
    /// The manufacturer ID may be prefixed with up to 13 of the
    /// continuation code 0x7F; the number of continuation codes
    /// is returned as the bank number.
    ///
    /// Returns (bank, manufacturer ID, device ID).
    fn read_device_id(&mut self) -> Result<(u8, u8, u8)> {
        // Attempt to read assuming a single-byte manufacturer ID.
        let data = self.exchange(Command::ReadDeviceID, &[0, 0, 0], 2)?;
        if data[0] != 0x7F {
            Ok((0, data[0], data[1]))
        } else {
            // If the first byte is continuation, read 15 bytes, to allow
            // up to 13 continuation bytes, and then parse it to find the IDs.
            let data = self.exchange(Command::ReadJEDECID, &[0, 0, 0], 15)?;
            for n in 1..=13 {
                if data[n] != 0x7F {
                    return Ok((n as u8, data[n], data[n+1]))
                }
            }
            log::error!("Found more than 11 continuation bytes in manufacturer ID");
            Err(Error::InvalidManufacturer)
        }
    }

    /// Read the device's unique ID, if present.
    fn read_unique_id(&mut self) -> Result<u64> {
        self.exchange(Command::ReadUniqueID, &[0, 0, 0, 0], 8)
            .map(|data| u64::from_be_bytes(data.try_into().unwrap()))
    }

    fn read_status1(&mut self) -> Result<u8> {
        self.exchange(Command::ReadStatusRegister1, &[], 1).map(|data| data[0])
    }

    fn write_status1(&mut self, status1: u8, status2: u8) -> Result<()> {
        self.write(Command::WriteStatusRegister1, &[status1, status2])
    }

    fn read_status2(&mut self) -> Result<u8> {
        self.exchange(Command::ReadStatusRegister2, &[], 1).map(|data| data[0])
    }

    fn is_busy(&mut self) -> Result<bool> {
        self.read_status1().map(|status| status & 1 == 1)
    }

    fn wait_while_busy(&mut self) -> Result<()> {
        while self.is_busy()? {}
        Ok(())
    }

    fn read_sfdp(&mut self, addr: u32, len: usize) -> Result<Vec<u8>> {
        let bytes = addr.to_be_bytes();
        self.exchange(Command::ReadSFDPRegister, &bytes[1..], 1+len)
            .map(|data| data[1..].to_vec())
    }

    fn read_sfdp_header(&mut self) -> Result<SFDPHeader> {
        // Read just SFDP header to get NPH first.
        let data = self.read_sfdp(0, 8)?;
        let nph = data[5] as usize;
        // Re-read overall SFDP header including parameters.
        let data = self.read_sfdp(0, 8 + nph*8)?;
        SFDPHeader::from_bytes(&data)
    }

    /// Writes `command` and `data` to the flash memory, then returns `nbytes` of response.
    fn exchange(&mut self, command: Command, data: &[u8], nbytes: usize) -> Result<Vec<u8>> {
        let mut tx = vec![command as u8];
        tx.extend(data);
        tx.extend(vec![0u8; nbytes]);
        let rx = self.access.exchange(&tx)?;
        Ok(rx[1+data.len()..].to_vec())
    }

    /// Writes `command` and `data` to the flash memory, without reading the response.
    fn write(&mut self, command: Command, data: &[u8]) -> Result<()> {
        let mut tx = vec![command as u8];
        tx.extend(data);
        self.access.write(&tx)?;
        Ok(())
    }

    /// Convenience method for issuing a single command and not caring about the returned data
    fn command(&mut self, command: Command) -> Result<()> {
        self.write(command, &[])?;
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct SFDPHeader {
    nph: usize,
    major: u8,
    minor: u8,
    params: Vec<SFDPParameterHeader>,
}

impl SFDPHeader {
    fn from_bytes(data: &[u8]) -> Result<Self> {
        log::debug!("Parsing SFDP header from data: {:X?}", data);
        if &data[0..4] != b"SFDP" {
            log::error!("Did not read expected SFDP signature");
            Err(Error::InvalidSFDPHeader)
        } else if data[7] != 0xFF {
            log::error!("Unsupported SFDP access protocol {:02X}", data[7]);
            Err(Error::InvalidSFDPHeader)
        } else {
            let minor = data[4];
            let major = data[5];
            let nph = data[6] as usize + 1;
            log::debug!("Read SFDP header, NPH={} MAJOR={} MINOR={}", nph, major, minor);
            if data.len() < (nph + 1) * 8 {
                log::error!("Did not read enough SFDP bytes: got {}, needed {}",
                            data.len(), (nph + 1) * 8);
                Err(Error::NotEnoughSFDPHeader)
            } else {
                let params = data[8..].chunks(8).map(SFDPParameterHeader::from_bytes).collect();
                Ok(SFDPHeader { nph, major, minor, params })
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct SFDPParameterHeader {
    plen: usize,
    major: u8,
    minor: u8,
    parameter_id: u16,
    ptp: u32,
}

impl SFDPParameterHeader {
    fn from_bytes(data: &[u8]) -> Self {
        log::debug!("Reading SFDP parameter header from: {:X?}", data);
        let parameter_id = u16::from_be_bytes([data[7], data[0]]);
        let minor = data[1];
        let major = data[2];
        let plen = data[3] as usize;
        let ptp = u32::from_be_bytes([0, data[6], data[5], data[4]]);
        log::debug!("Read JEDEC parameter header, plen={} major={} minor={} \
                     ID=0x{:04X} PTP=0x{:06X}",
                    plen, major, minor, parameter_id, ptp);
        SFDPParameterHeader { plen, major, minor, parameter_id, ptp }
    }
}

/// JEDEC Basic Flash Parameter Table
///
/// This table contains standard SFDP information which may be
/// read from a flash memory. Only fields relevant to single I/O
/// operation are parsed.
///
/// Fields are taken from JESD216D-01, supporting parameter versions
/// up to 1.7.
#[derive(Copy, Clone, Debug)]
pub struct JEDECBasicParams {
    /// Parameter header major version field.
    pub version_major: u8,
    /// Parameter header minor version field.
    pub version_minor: u8,

    /// Number of address bytes to use in read/write commands.
    pub address_bytes: SFDPAddressBytes,
    /// Flash memory density in bits.
    pub density: u64,

    /// If true, 4kB erase is supported.
    /// Newer memories indicate all erase sizes with `erase_*` fields.
    pub legacy_4kb_erase_supported: bool,
    /// Instruction for 4kB erase, or 0xFF if unsupported.
    /// Newer memories also include this instruction in `erase_*` fields.
    pub legacy_erase_4kb_inst: u8,
    /// Write enable instruction for volatile status register, either 0x50 or 0x06.
    /// Newer memories use `write_en_inst` instead.
    pub legacy_write_en_inst: u8,
    /// If true, Block Protect bits in status register are only volatile,
    /// otherwise they may be only non-volatile or may be programmed either
    /// as volatile with instruction 0x50 or non-volatile with instruction 0x06.
    /// Newer memories use `write_en_inst` instead.
    pub legacy_block_protect_volatile: bool,
    /// If true, writes can be performed with byte granularity.
    /// Newer memories use `page_size`.
    pub legacy_byte_write_granularity: bool,

    /// Erase instructions.
    ///
    /// Up to four erase instructions may be available,
    /// each specifying the opcode for the instruction
    /// and the number of bytes erased.
    pub erase_insts: [Option<SFDPEraseInst>; 4],

    /// Typical time to erase the entire chip, if known.
    pub chip_erase_time_typ: Option<std::time::Duration>,
    /// Maximum time to erase the entire chip, if known.
    pub chip_erase_time_max: Option<std::time::Duration>,
    /// Typical time to program the first byte in a sequence, if known.
    pub first_byte_prog_time_typ: Option<std::time::Duration>,
    /// Maximum time to program the first byte in a sequence, if known.
    pub first_byte_prog_time_max: Option<std::time::Duration>,
    /// Typical time to program each successive byte in a sequence, if known.
    pub succ_byte_prog_time_typ: Option<std::time::Duration>,
    /// Maximum time to program each successive byte in a sequence, if known.
    pub succ_byte_prog_time_max: Option<std::time::Duration>,
    /// Typical time to program a full page, if known.
    pub page_prog_time_typ: Option<std::time::Duration>,
    /// Maximum time to program a full page, if known.
    pub page_prog_time_max: Option<std::time::Duration>,

    /// Page size, in bytes.
    pub page_size: Option<u32>,

    // Omitted: Suspend/Resume support and instructions.

    /// Deep Powerdown supported.
    pub deep_powerdown_supported: Option<bool>,
    /// Instruction for entering deep powerdown, if supported.
    pub enter_deep_powerdown_inst: Option<u8>,
    /// Instruction for exiting deep powerdown, if supported.
    pub exit_deep_powerdown_inst: Option<u8>,
    /// Maximum time required to exit deep powerdown and be ready for next instruction.
    pub exit_deep_powerdown_time: Option<std::time::Duration>,

    /// If true, polling busy status via the flag status register is supported.
    /// Instruction 0x70 reads the flag register, where bit 7 is 0 if busy and 1 if ready.
    pub busy_poll_flag: Option<bool>,
    /// If true, polling busy status via the status register is supported.
    /// Instruction 0x05 reads the status register, where bit 0 is 0 if ready and 1 if busy.
    pub busy_poll_status: Option<bool>,

    // Omitted: instructions for entering/exiting 4-byte address mode.

    /// If true, the device may be reset using instruction 0xF0.
    pub reset_inst_f0: Option<bool>,
    /// If true, the device may be reset using instruction 0x66 followed by 0x99.
    pub reset_inst_66_99: Option<bool>,

    /// Status register 1 volatility and write-enable instruction.
    pub status_1_vol: Option<SFDPStatus1Volatility>,
}

/// SFDP Address Bytes field.
#[derive(Copy, Clone, Debug)]
pub enum SFDPAddressBytes {
    /// Three-byte only addressing.
    Three,
    /// Three- or four-byte addressing; default is three-byte
    /// but may be configured for four-byte.
    ThreeOrFour,
    /// Four-byte only addressing.
    Four,
    /// Reserved as of JESD216D-01, JEDEC Basic Flash Parameters version 1.7.
    Reserved,
}

impl SFDPAddressBytes {
    fn from_bits(bits: u8) -> Self {
        match bits {
            0b00 => SFDPAddressBytes::Three,
            0b01 => SFDPAddressBytes::ThreeOrFour,
            0b10 => SFDPAddressBytes::Four,
            _    => SFDPAddressBytes::Reserved,
        }
    }
}

/// SFDP Erase Instruction.
#[derive(Copy, Clone, Debug)]
pub struct SFDPEraseInst {
    /// Opcode for erase instruction.
    pub opcode: u8,
    /// Size in bytes of erase instruction.
    pub size: u32,
    /// Typical erase time, if known.
    pub time_typ: Option<std::time::Duration>,
    /// Maximum erase time, if known.
    pub time_max: Option<std::time::Duration>,
}

#[derive(Copy, Clone, Debug)]
pub enum SFDPStatus1Volatility {
    /// Status register 1 is non-volatile, powers up to its last state, write-enable with 0x06.
    NonVolatile06,
    /// Status register 1 is volatile, powers up to all '1', write-enable with 0x06.
    Volatile06,
    /// Status register 1 is volatile, powers up to all '1', write-enable with 0x50.
    Volatile50,
    /// Status register 1 powers up to its last non-volatile state, use 0x06
    /// to write to non-volatile register, or use 0x06 to active and write
    /// volatile register.
    NonVolatile06Volatile50,
    /// Status register 1 contains a mix of volatile and non-vilatile bits.
    /// Use instruction 0x06 to write.
    Mixed06,
    /// Reserved volatility mode.
    Reserved,
}

impl SFDPStatus1Volatility {
    pub fn from_bits(bits: u8) -> Self {
        if bits & 0b000_0001 != 0 {
            SFDPStatus1Volatility::NonVolatile06
        } else if bits & 0b000_0010 != 0 {
            SFDPStatus1Volatility::Volatile06
        } else if bits & 0b000_0100 != 0 {
            SFDPStatus1Volatility::Volatile50
        } else if bits & 0b000_1000 != 0 {
            SFDPStatus1Volatility::NonVolatile06Volatile50
        } else if bits & 0b001_0000 != 0 {
            SFDPStatus1Volatility::Mixed06
        } else {
            SFDPStatus1Volatility::Reserved
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SFDPJEDECParams {
    addr_bytes: SFDPAddressBytes,
    write_en_opcode: u8,
    write_en_required: bool,
    /// Flash memory capacity in bits.
    density: u64,
    erase_s1_opcode: u8,
    erase_s1_size: u8,
    erase_s2_opcode: u8,
    erase_s2_size: u8,
    erase_s3_opcode: u8,
    erase_s3_size: u8,
    erase_s4_opcode: u8,
    erase_s4_size: u8,
    erase_4kb_opcode: u8,
    erase_4kb_available: bool,
}


impl SFDPJEDECParams {
    fn from_bytes(data: &[u8]) -> Self {
        log::debug!("Reading SFDP JEDEC Basic Flash Parameters from: {:X?}", data);
        let addr_bytes = SFDPAddressBytes::from_bits((data[2] >> 1) & 0b11);
        let write_en_opcode = match (data[0] >> 4) & 1 {
            0 => 0x50,
            1 => 0x06,
            _ => unreachable!(),
        };
        let write_en_required = ((data[0] >> 3) & 1) == 1;
        let raw_density = u32::from_be_bytes([data[7], data[6], data[5], data[4]]);
        let density = if raw_density >> 31 == 0 {
            raw_density as u64
        } else {
            1u64 << (raw_density & 0x7FFF_FFFF)
        };
        let erase_s1_size = data[28];
        let erase_s1_opcode = data[29];
        let erase_s2_size = data[30];
        let erase_s2_opcode = data[31];
        let erase_s3_size = data[32];
        let erase_s3_opcode = data[33];
        let erase_s4_size = data[34];
        let erase_s4_opcode = data[35];
        let erase_4kb_opcode = data[1];
        let erase_4kb_available = (data[0] & 0b11) == 0b01;
        SFDPJEDECParams {
            addr_bytes, write_en_opcode, write_en_required, density,
            erase_s1_opcode, erase_s1_size, erase_s2_opcode, erase_s2_size,
            erase_s3_opcode, erase_s3_size, erase_s4_opcode, erase_s4_size,
            erase_4kb_opcode, erase_4kb_available,
        }
    }

    pub fn capacity_bytes(&self) -> u64 {
        (self.density + 1) / 8
    }
}
