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

#[derive(Copy, Clone, Debug)]
#[allow(unused)]
#[repr(u8)]
enum Command {
    WriteEnable = 0x06,
    WriteDisable = 0x04,
    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    WriteStatusRegister = 0x01,
    PageProgram = 0x02,
    SectorErase = 0x20,
    BlockErase32KB = 0x52,
    BlockErase64KB = 0xD8,
    ChipErase = 0xC7,
    ProgramSuspend = 0x75,
    ProgramResume = 0x7A,
    PowerDown = 0xB9,
    ReadData = 0x03,
    FastRead = 0x0B,
    ReleasePowerdown = 0xAB,
    ReadDeviceID = 0x90,
    ReadJEDECID = 0x9F,
    ReadUniqueID = 0x4B,
    ReadSFDPRegister = 0x5A,
    EnableReset = 0x66,
    Reset = 0x99,
}

#[derive(Copy, Clone, Debug)]
pub struct FlashID {
    manufacturer_bank: u8,
    manufacturer_id: u8,
    device_id: u16,
    unique_id: u64,
}

impl FlashID {
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
        write!(f, "Manufacturer 0x{:02X}{}, Device 0x{:04X}, Unique ID {:016X}",
               self.manufacturer_id, mfn, self.device_id, self.unique_id)
    }
}

/// Trait for objects which provide access to SPI flash.
pub trait FlashAccess {
    /// Assert CS, write all bytes in `data` to the SPI bus, then de-assert CS.
    fn write(&mut self, data: &[u8]) -> Result<()>;

    /// Assert CS, write all bytes in `data` while capturing received data, then de-assert CS.
    ///
    /// Returns the received data.
    fn exchange(&mut self, data: &[u8]) -> Result<Vec<u8>>;
}

pub struct Flash<'a, A: FlashAccess> {
    access: &'a mut A,
}

impl<'a, A: FlashAccess> Flash<'a, A> {
    /// Create a new Flash instance wrapping a FlashAccess provider.
    pub fn new(access: &'a mut A) -> Self {
        Flash { access }
    }

    /// Read the attached flash device, manufacturer, and unique IDs
    pub fn read_id(&mut self) -> Result<FlashID> {
        self.power_up()?;
        self.reset()?;
        let (manufacturer_bank, manufacturer_id, device_id) = self.read_jedec_id()?;
        // XXX: Without sleep, subsequent commands return all 0s.
        std::thread::sleep(std::time::Duration::from_millis(200));
        let unique_id = self.read_unique_id()?;
        // XXX: Without sleep, subsequent commands return all 0s.
        std::thread::sleep(std::time::Duration::from_millis(200));
        Ok(FlashID { manufacturer_bank, manufacturer_id, device_id, unique_id })
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

    /// Unprotect the flash memory
    pub fn unprotect(&mut self) -> Result<()> {
        let status1 = self.read_status1()?;
        let status2 = self.read_status2()?;
        let new_status1 = status1 & 0b11100011;
        self.write_enable()?;
        self.write_status(new_status1, status2)?;
        Ok(())
    }

    /// Power down the attached flash
    pub fn power_down(&mut self) -> Result<()> {
        self.command(Command::PowerDown)
    }

    /// Power up the attached flash
    pub fn power_up(&mut self) -> Result<()> {
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
            self.block_erase_64k(address + (block * BLOCK_SIZE) as u32)?;
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

    fn block_erase_64k(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::BlockErase64KB, &address.to_be_bytes()[1..], 0)?;
        Ok(())
    }

    #[allow(dead_code)]
    fn block_erase_32k(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::BlockErase32KB, &address.to_be_bytes()[1..], 0)?;
        Ok(())
    }

    #[allow(dead_code)]
    fn sector_erase(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::SectorErase, &address.to_be_bytes()[1..], 0)?;
        Ok(())
    }

    /// Reads the JEDEC manufacturer and device IDs.
    ///
    /// The manufacturer ID may be prefixed with any number of the
    /// continuation code 0x7F; the number of continuation codes
    /// is returned as the bank number.
    ///
    /// Returns (bank, manufacturer ID, device ID).
    fn read_jedec_id(&mut self) -> Result<(u8, u8, u16)> {
        for n in 0..=11 {
            let data = self.exchange(Command::ReadJEDECID, &[], n+3)?;
            if data[n] != 0x7F {
                let device_id = u16::from_be_bytes([data[n+1], data[n+2]]);
                return Ok((n as u8, data[n], device_id));
            }
        }
        log::error!("Found more than 11 continuation bytes in manufacturer ID");
        Err(Error::InvalidManufacturer)
    }

    fn read_unique_id(&mut self) -> Result<u64> {
        self.exchange(Command::ReadUniqueID, &[], 4+8)
            .map(|data| u64::from_be_bytes((&data[4..]).try_into().unwrap()))
    }

    fn read_status1(&mut self) -> Result<u8> {
        self.exchange(Command::ReadStatusRegister1, &[], 1).map(|data| data[0])
    }

    fn write_status(&mut self, status1: u8, status2: u8) -> Result<()> {
        self.write(Command::WriteStatusRegister, &[status1, status2])
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

#[derive(Copy, Clone, Debug)]
pub enum SFDPAddressBytes {
    Three,
    ThreeOrFour,
    Four,
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
