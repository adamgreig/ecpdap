use std::convert::TryInto;
use std::time::Duration;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Mismatch during flash readback verification.")]
    ReadbackError,
    #[error("Invalid manufacturer ID detected.")]
    InvalidManufacturer,
    #[error("Invalid SFDP header.")]
    InvalidSFDPHeader,
    #[error("Invalid parameter in SFDP parameter table.")]
    InvalidSFDPParams,
    #[error("Address out of range for memory.")]
    InvalidAddress,
    #[error("No supported reset instruction is available.")]
    NoResetInstruction,

    #[error(transparent)]
    Access(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

/// Trait for objects which provide access to SPI flash.
///
/// Providers only need to implement `exchange()`, which asserts CS, writes all the bytes
/// in `data`, then returns all the received bytes. If it provides a performance optimisation,
/// providers may also implement `write()`, which does not require the received data.
pub trait FlashAccess {
    /// Assert CS, write all bytes in `data` to the SPI bus, then de-assert CS.
    fn write(&mut self, data: &[u8]) -> anyhow::Result<()> {
        // Default implementation uses `exchange()` and ignores the result data.
        self.exchange(data)?;
        Ok(())
    }

    /// Assert CS, write all bytes in `data` while capturing received data, then de-assert CS.
    ///
    /// Returns the received data.
    fn exchange(&mut self, data: &[u8]) -> anyhow::Result<Vec<u8>>;
}

/// SPI Flash.
///
/// This struct provides methods for interacting with common SPI flashes.
pub struct Flash<'a, A: FlashAccess> {
    access: &'a mut A,

    /// Once read, ID details are cached.
    id: Option<FlashID>,

    /// Once read, SFDP parameters are cached.
    params: Option<FlashParams>,

    /// Number of address bytes to use when reading and writing.
    /// This is set to 3 by default for compatibility, but may
    /// be set to 2 for legacy memories or 4 for high-density memories.
    address_bytes: u8,

    /// Total data memory capacity in bytes, up to 4GB.
    capacity: Option<usize>,

    /// Page size in bytes, used for programming operations.
    page_size: Option<usize>,

    /// Sector size in bytes, used for the smallest erase operations.
    sector_size: Option<usize>,

    /// EraseSector instruction opcode.
    /// This is set to 0x20 by default but may be overridden.
    erase_sector_opcode: u8,
}

impl<'a, A: FlashAccess> Flash<'a, A> {
    /// Create a new Flash instance using the given FlashAccess provider.
    pub fn new(access: &'a mut A) -> Self {
        Flash {
            access,
            id: None,
            params: None,
            address_bytes: 3,
            capacity: None,
            page_size: None,
            sector_size: None,
            erase_sector_opcode: 0x20,
        }
    }

    /// Get the number of address bytes which will be used in read and write commands.
    pub fn address_bytes(&self) -> u8 {
        self.address_bytes
    }

    /// Set the number of address bytes to use with read and write commands.
    /// By default this is set to 3, and can also be autodiscovered using SFDP.
    ///
    /// Panics if `n` is less than 1 or greater than 4.
    pub fn set_address_bytes(&mut self, n: u8) {
        assert!(n >= 1, "set_address_bytes: n must be at least 1");
        assert!(n <= 4, "set_address_bytes: n must not exceed 4");
        self.address_bytes = n;
    }

    /// Get the total memory capacity in bytes, if known.
    pub fn capacity(&self) -> Option<usize> {
        self.capacity
    }

    /// Set the total memory capacity in bytes.
    ///
    /// If set, or discovered using SFDP, reads and writes are prevented
    /// from going beyond the memory capacity.
    pub fn set_capacity(&mut self, n: usize) {
        self.capacity = Some(n);
    }

    /// Get the page program size in bytes.
    pub fn page_size(&self) -> Option<usize> {
        self.page_size
    }

    /// Set the page program size in bytes.
    ///
    /// This must be known before page program operations can be performed.
    pub fn set_page_size(&mut self, n: usize) {
        self.page_size = Some(n);
    }

    /// Get the sector erase size in bytes, if known.
    pub fn sector_size(&self) -> Option<usize> {
        self.sector_size
    }

    /// Set the sector erase size in bytes.
    ///
    /// This must be known before sector erase operations can be performed.
    pub fn set_sector_size(&mut self, n: usize) {
        self.sector_size = Some(n);
    }

    /// Get the opcode used for the Erase Sector instruction.
    pub fn erase_sector_opcode(&self) -> u8 {
        self.erase_sector_opcode
    }

    /// Set the opcode used for the Erase Sector instruction.
    ///
    /// This is 0x20 by default.
    pub fn set_erase_sector_opcode(&mut self, opcode: u8) {
        self.erase_sector_opcode = opcode;
    }

    /// Get the flash ID, if it has already been read.
    ///
    /// Call `read_id()` to read the ID from the flash.
    pub fn get_id(&self) -> Option<FlashID> {
        self.id
    }

    /// Get the flash parameters, if they have already been read.
    ///
    /// Call `read_params()` to read the params from the flash.
    pub fn get_params(&self) -> Option<FlashParams> {
        self.params
    }

    /// Read the device's manufacturer ID and device IDs.
    ///
    /// This method additionally brings the flash out of powerdown and resets it.
    ///
    /// `self.id` is updated with the new ID; use `get_id()` to
    /// retrieve it without re-reading the ID from the flash.
    pub fn read_id(&mut self) -> Result<FlashID> {
        log::debug!("Reading SPI Flash ID");

        let legacy_id = self.release_power_down()?;
        self.reset()?;

        let (bank_long, mfn_id_long, device_id_long) = self.read_jedec_id()?;
        let (bank_short, mfn_id_short, mut device_id_short) = self.read_device_id()?;
        let unique_id = self.read_unique_id()?;

        // The device may implement any or none of the three identification
        // instructions; attempt to obtain a valid manufacturer ID and device ID.
        // If there is no device present or a communication error, we'll probably
        // receive all-0s or all-1s for all the data.
        let manufacturer_bank;
        let manufacturer_id;
        if mfn_id_long != 0x00 && mfn_id_long != 0xFF {
            manufacturer_bank = bank_long;
            manufacturer_id = mfn_id_long;
        } else if mfn_id_short != 0x00 && mfn_id_short != 0xFF {
            manufacturer_bank = bank_short;
            manufacturer_id = mfn_id_short;
        } else {
            log::warn!("No valid manufacturer ID found");
            if legacy_id == 0x00 || legacy_id == 0xFF {
                log::error!("No device or manufacturer ID found");
                return Err(Error::InvalidManufacturer);
            } else {
                device_id_short = legacy_id;
                manufacturer_bank = 0;
                manufacturer_id = 0;
            }
        }

        let id = FlashID {
            manufacturer_bank, manufacturer_id, device_id_short, device_id_long, unique_id
        };

        log::debug!("Read ID: {:?}", id);
        self.id = Some(id);
        Ok(id)
    }

    /// Read SFDP JEDEC Basic Flash Parameter table from flash.
    ///
    /// This fails if SFDP is not supported by the flash memory.
    ///
    /// Depending on the version of SFDP supported, some fields may
    /// not be available.
    ///
    /// Once read, the parameters are available using `get_params()`,
    /// and the parameter values are automatically used for the
    /// configuration of address bytes, capacity, page size, sector size,
    /// and erase sector opcode. Additionally, larger erase commands
    /// described by the SFDP parameters will be used when appropriate.
    pub fn read_params(&mut self) -> Result<FlashParams> {
        log::debug!("Reading SFDP data");

        // Read just SFDP header to get NPH first.
        let data = self.read_sfdp(0, 8)?;
        let nph = data[5] as usize;

        // Re-read overall SFDP header including parameter headers.
        let data = self.read_sfdp(0, 8 + nph*8)?;
        let header = SFDPHeader::from_bytes(&data)?;

        // Check the first parameter header is the JEDEC basic flash parameters,
        // as required by JESD216.
        let params = header.params[0];
        if params.parameter_id != 0xFF00 || params.major != 0x01 {
            log::error!("Unexpected first SFDP parameter header: expected 0xFF00 version 1.");
            return Err(Error::InvalidSFDPHeader);
        }

        // Read SFDP table data and parse into a FlashParams struct.
        let data = self.read_sfdp(params.ptp, params.plen * 4)?;
        let params = FlashParams::from_bytes(params.major, params.minor, &data)?;
        self.params = Some(params);

        // Use params to update settings where posssible.
        self.address_bytes = match params.address_bytes {
            SFDPAddressBytes::Three => 3,
            SFDPAddressBytes::ThreeOrFour => 3,
            SFDPAddressBytes::Four => 4,
            _ => 3,
        };
        self.capacity = Some(params.capacity_bytes());
        if let Some(page_size) = params.page_size {
            self.page_size = Some(page_size as usize);
        }
        if let Some((size, opcode)) = params.sector_erase() {
            self.sector_size = Some(size);
            self.erase_sector_opcode = opcode;
        } else if params.legacy_4kb_erase_supported && params.legacy_erase_4kb_inst != 0xFF {
            self.sector_size = Some(4096);
            self.erase_sector_opcode = params.legacy_erase_4kb_inst;
        }
        log::debug!("Updated settings from parameters:");
        log::debug!("Address bytes: {}, capacity: {:?} bytes",
                    self.address_bytes, self.capacity);
        log::debug!("Page size: {:?}, sector size: {:?}, sector op: {}",
                    self.page_size, self.sector_size, self.erase_sector_opcode);

        Ok(params)
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`.
    ///
    /// This method uses the FastRead instruction; if it is not supported
    /// try using `legacy_read()` instead.
    pub fn read(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        self.check_address_length(address, length)?;
        let length = length + 1;
        let address = &address.to_be_bytes()[1..];
        self.exchange(Command::FastRead, address, length).map(|data| data[1..].to_vec())
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`.
    ///
    /// This method uses the legacy ReadData instruction, which often has a low
    /// maximum clock speed compared to other operations.
    pub fn legacy_read(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        self.check_address_length(address, length)?;
        let address = &address.to_be_bytes()[1..];
        self.exchange(Command::ReadData, address, length).map(|data| data[1..].to_vec())
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

    /// Erase entire flash chip.
    ///
    /// This method uses the ChipErase instruction, so no progress information
    /// is available, but the typical and maximum times taken may be available
    /// from the SFDP parameters.
    ///
    /// Returns once erase operation is complete.
    pub fn chip_erase(&mut self) -> Result<()> {
        self.write_enable()?;
        self.chip_erase()?;
        self.wait_while_busy()?;
        Ok(())
    }

    /// Reset the attached flash.
    ///
    /// The instruction sequence EnableReset 0x66 followed by Reset 0x99
    /// is sent by default, but if the SFDP parameters indicate that only
    /// the 0xF0 instruction is supported for reset, that is sent instead.
    pub fn reset(&mut self) -> Result<()> {
        let mut do_f0 = false;
        let mut do_66_99 = true;

        if let Some(params) = self.params {
            if let Some(op_66_99) = params.reset_inst_66_99 {
                do_66_99 = op_66_99;
            }
            if let Some(op_f0) = params.reset_inst_f0 {
                do_f0 = op_f0;
            }
        }

        if do_66_99 {
            self.command(Command::EnableReset)?;
            self.command(Command::Reset)
        } else if do_f0 {
            self.command(0xF0)
        } else {
            log::error!("No reset instruction available.");
            Err(Error::NoResetInstruction)
        }
    }

    /// Check if any block protect bits are set in status register 1.
    pub fn is_protected(&mut self) -> Result<bool> {
        log::debug!("Checking if BP bits are set");
        let status1 = self.read_status1()?;
        let (bp0, bp1, bp2) = status1.get_block_protect();
        log::debug!("BP0: {}, BP1: {}, BP2: {}", bp0, bp1, bp2);
        Ok(bp0 || bp1 || bp2)
    }

    /// Clear any protection bits that are set.
    ///
    /// This checks and clears the block protect bits in status register 1,
    /// using the non-volatile commands if supported. If available, the SFDP
    /// parameters are used to determine the correct non-volatile instruction.
    pub fn unprotect(&mut self) -> Result<()> {
        log::debug!("Checking if BP bits are set before clearing them");
        let mut status1 = self.read_status1()?;
        let (bp0, bp1, bp2) = status1.get_block_protect();
        if bp0 || bp1 || bp2 {
            log::debug!("Block protect bits are currently set, clearing.");
            status1.set_block_protect(false, false, false);
            self.write_status1(status1)?;
            self.wait_while_busy()?;
        }
        Ok(())
    }

    /// Clear the write-protect-selection bit in status register 3, if set.
    ///
    /// This status bit configures the fine-granularity write protection
    /// which is a vendor-specific extension that is disabled by default.
    ///
    /// Unfortunately it is not possible to determine automatically if a
    /// flash chip supports the WPS feature or even has a status register 3,
    /// so this command is not called automatically.
    pub fn unprotect_wps(&mut self) -> Result<()> {
        let mut status3 = self.read_status3()?;
        if status3.get_wps() {
            log::debug!("WPS bit set, clearing.");
            status3.set_wps(false);
            self.write_status3(status3)?;
            self.wait_while_busy()?;
        }
        Ok(())
    }

    /// Power down the flash.
    pub fn power_down(&mut self) -> Result<()> {
        log::debug!("Sending Powerdown command");
        self.command(Command::Powerdown)
    }

    /// Power up the flash.
    ///
    /// Returns the legacy device ID.
    pub fn release_power_down(&mut self) -> Result<u8> {
        log::debug!("Sending Release Powerdown command");
        let data = self.exchange(Command::ReleasePowerdown, &[0, 0, 0], 1)?;
        Ok(data[0])
    }

    /// Checks if `address` and `length` together are permissible:
    /// * `address` must not exceed the current number of address bytes
    /// * Both `address` and `address+length` must be within the flash memory bounds,
    ///   if the capacity is known.
    /// Returns either Err(Error::InvalidAddress) or Ok(()).
    fn check_address_length(&self, address: u32, length: usize) -> Result<()> {
        let start = address as usize;
        let end = (address as usize) + length - 1;
        let max_addr = 1 << (self.address_bytes * 8);

        if (end & (max_addr - 1)) < start {
            log::error!("Operation would wrap");
            Err(Error::InvalidAddress)
        } else if end > max_addr {
            log::error!("Operation would exceed largest address");
            Err(Error::InvalidAddress)
        } else {
            match self.capacity {
                Some(capacity) if (end >= capacity) => {
                    log::error!("Operation would exceed flash capacity");
                    Err(Error::InvalidAddress)
                },
                _ => Ok(()),
            }
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

    /// Send the WriteEnable command, setting the WEL in the status register.
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

    #[allow(dead_code)]
    fn block_erase_1(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::BlockErase1, &address.to_be_bytes()[1..], 0)?;
        Ok(())
    }

    fn block_erase_2(&mut self, address: u32) -> Result<()> {
        self.exchange(Command::BlockErase2, &address.to_be_bytes()[1..], 0)?;
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

    /// Read the device's 64-bit unique ID, if present.
    fn read_unique_id(&mut self) -> Result<u64> {
        self.exchange(Command::ReadUniqueID, &[0, 0, 0, 0], 8)
            .map(|data| u64::from_be_bytes(data.try_into().unwrap()))
    }

    /// Read status register 1.
    fn read_status1(&mut self) -> Result<StatusRegister1> {
        self.exchange(Command::ReadStatusRegister1, &[], 1).map(|data| StatusRegister1(data[0]))
    }

    /// Read status register 3.
    ///
    /// This status register is less widely supported and SFDP does
    /// not indicate whether or not it is present.
    fn read_status3(&mut self) -> Result<StatusRegister3> {
        self.exchange(Command::ReadStatusRegister3, &[], 1).map(|data| StatusRegister3(data[0]))
    }

    /// Write status register 1.
    ///
    /// This method does *not* require you call `write_enable()` first.
    ///
    /// If the SFDP parameters indicate a specific command should be used
    /// to enable writing to status register 1, that is used, otherwise the
    /// default WriteEnable of 0x06 is used.
    fn write_status1(&mut self, status1: StatusRegister1) -> Result<()> {
        let we_opcode = if let Some(params) = self.params {
            match params.status_1_vol {
                Some(SFDPStatus1Volatility::NonVolatile06) => 0x06,
                Some(SFDPStatus1Volatility::Volatile06) => 0x06,
                Some(SFDPStatus1Volatility::Volatile50) => 0x50,
                Some(SFDPStatus1Volatility::NonVolatile06Volatile50) => 0x06,
                Some(SFDPStatus1Volatility::Mixed06) => 0x06,
                _ => if params.legacy_block_protect_volatile {
                    params.legacy_volatile_write_en_inst
                } else {
                    Command::WriteEnable.into()
                }
            }
        } else {
            Command::WriteEnable.into()
        };
        self.command(we_opcode)?;
        self.write(Command::WriteStatusRegister1, &[status1.0])
    }

    /// Write status register 3.
    fn write_status3(&mut self, status3: StatusRegister3) -> Result<()> {
        self.write_enable()?;
        self.write(Command::WriteStatusRegister3, &[status3.0])
    }

    /// Check if the device is currently busy performing an operation.
    ///
    /// If the flash parameters indicate support for the Flag Status Register
    /// instruction (0x70), it is used, otherwise legacy polling of status
    /// register 1 is used.
    fn is_busy(&mut self) -> Result<bool> {
        // If we have read parameters and flag status polling is supported, use that.
        // Bit 7 of FSR is 0=busy and 1=ready.
        if let Some(params) = self.params {
            if let Some(busy_poll_flag) = params.busy_poll_flag {
                if busy_poll_flag {
                    let fsr = self.exchange(Command::ReadFlagStatusRegister, &[], 1)?[0];
                    return Ok(fsr & 0b1000_0000 == 0);
                }
            }
        }

        // Otherwise and by default, poll status register 1 instead.
        self.read_status1().map(|status| status.get_busy())
    }

    /// Wait until the device stops being busy.
    ///
    /// This polls using `is_busy()`, which uses the flag status
    /// register if available or otherwise uses status register 1.
    fn wait_while_busy(&mut self) -> Result<()> {
        while self.is_busy()? {}
        Ok(())
    }

    /// Read SFDP register data.
    ///
    /// `addr` is always sent as a 24-bit address, regardless of the address_bytes setting.
    fn read_sfdp(&mut self, addr: u32, len: usize) -> Result<Vec<u8>> {
        let bytes = addr.to_be_bytes();
        self.exchange(Command::ReadSFDPRegister, &bytes[1..], 1+len)
            .map(|data| data[1..].to_vec())
    }

    /// Writes `command` and `data` to the flash memory, then returns `nbytes` of response.
    fn exchange<C: Into<u8>>(&mut self, command: C, data: &[u8], nbytes: usize)
        -> Result<Vec<u8>>
    {
        let mut tx = vec![command.into()];
        tx.extend(data);
        tx.extend(vec![0u8; nbytes]);
        let rx = self.access.exchange(&tx)?;
        Ok(rx[1+data.len()..].to_vec())
    }

    /// Writes `command` and `data` to the flash memory, without reading the response.
    fn write<C: Into<u8>>(&mut self, command: C, data: &[u8]) -> Result<()> {
        let mut tx = vec![command.into()];
        tx.extend(data);
        self.access.write(&tx)?;
        Ok(())
    }

    /// Convenience method for issuing a single command and not caring about the returned data
    fn command<C: Into<u8>>(&mut self, command: C) -> Result<()> {
        self.write(command, &[])?;
        Ok(())
    }
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
                Err(Error::InvalidSFDPHeader)
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

/// SFDP JEDEC Basic Flash Parameter Table
///
/// This table contains standard SFDP information which may be
/// read from a flash memory. Only fields relevant to single I/O
/// operation are parsed.
///
/// Fields are taken from JESD216D-01, supporting parameter versions up to 1.7.
#[derive(Copy, Clone, Debug)]
pub struct FlashParams {
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
    /// Newer memories use `status_1_vol` instead.
    pub legacy_volatile_write_en_inst: u8,
    /// If true, Block Protect bits in status register are only volatile,
    /// otherwise they may be only non-volatile or may be programmed either
    /// as volatile with instruction 0x50 or non-volatile with instruction 0x06.
    /// Newer memories use `status_1_vol` instead.
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
    pub chip_erase_time_typ: Option<Duration>,
    /// Maximum time to erase the entire chip, if known.
    pub chip_erase_time_max: Option<Duration>,
    /// Typical time to program the first byte in a sequence, if known.
    pub first_byte_prog_time_typ: Option<Duration>,
    /// Maximum time to program the first byte in a sequence, if known.
    pub first_byte_prog_time_max: Option<Duration>,
    /// Typical time to program each successive byte in a sequence, if known.
    pub succ_byte_prog_time_typ: Option<Duration>,
    /// Maximum time to program each successive byte in a sequence, if known.
    pub succ_byte_prog_time_max: Option<Duration>,
    /// Typical time to program a full page, if known.
    pub page_prog_time_typ: Option<Duration>,
    /// Maximum time to program a full page, if known.
    pub page_prog_time_max: Option<Duration>,

    /// Page size, in bytes.
    pub page_size: Option<u32>,

    // Omitted: Suspend/Resume support and instructions.

    // Omitted: Deep powerdown support and instructions.

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
    fn from_bits(bits: u32) -> Self {
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
    /// to write to non-volatile register, or use 0x50 to active and write
    /// volatile register.
    NonVolatile06Volatile50,
    /// Status register 1 contains a mix of volatile and non-volatile bits.
    /// Use instruction 0x06 to write.
    Mixed06,
    /// Reserved volatility mode.
    Reserved,
}

impl SFDPStatus1Volatility {
    pub fn from_bits(bits: u32) -> Self {
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

macro_rules! bits {
    ($d:expr, $n:expr, $o:expr) => {($d & (((1 << $n) - 1) << $o)) >> $o}
}

impl FlashParams {
    fn from_bytes(major: u8, minor: u8, data: &[u8]) -> Result<Self> {
        log::debug!("Reading SFDP JEDEC Basic Flash Parameters from: {:X?}", data);

        // Check we have enough data.
        if data.len() % 4 != 0 {
            log::error!("SFPD data is not a multiple of 4 bytes.");
            return Err(Error::InvalidSFDPParams);
        } else if data.len() < 9 * 4 {
            log::error!("SFPD data is not long enough for version >= 1.0.");
            return Err(Error::InvalidSFDPParams);
        } else if major != 1 {
            log::error!("Only SFPD major version 1 is supported.");
            return Err(Error::InvalidSFDPParams);
        } else if minor > 5 && data.len() < 16 * 4 {
            log::error!("SFPD data is not long enough for version >= 1.5.");
            return Err(Error::InvalidSFDPParams);
        }

        // Convert the bytes into "DWORD"s (u32) for easier reference to the specification.
        let mut dwords = Vec::new();
        for bytes in data.chunks(4) {
            dwords.push(u32::from_be_bytes([bytes[3], bytes[2], bytes[1], bytes[0]]));
        }

        // 1st DWORD
        let address_bytes = SFDPAddressBytes::from_bits(bits!(dwords[0], 2, 17));
        let legacy_erase_4kb_inst = bits!(dwords[0], 8, 8) as u8;
        let legacy_volatile_write_en_inst = match bits!(dwords[0], 1, 4) {
            0 => 0x50,
            1 => 0x06,
            _ => unreachable!(),
        };
        let legacy_block_protect_volatile = bits!(dwords[0], 1, 3) == 1;
        let legacy_byte_write_granularity = bits!(dwords[0], 1, 2) == 1;
        let legacy_4kb_erase_supported = bits!(dwords[0], 2, 0) == 0b01;

        // 2nd DWORD
        let density = if dwords[1] >> 31 == 0 {
            dwords[1] as u64
        } else {
            1u64 << (dwords[1] & 0x7FFF_FFFF)
        };

        // DWORDS 3,4, 5, 6, and 7 relate to multiple I/O and are skipped.

        // 8th and 9th DWORD
        let mut erase_insts = [None; 4];
        let erase_size_1 = bits!(dwords[7], 8, 0);
        let erase_size_2 = bits!(dwords[7], 8, 16);
        let erase_size_3 = bits!(dwords[8], 8, 0);
        let erase_size_4 = bits!(dwords[9], 8, 16);
        if erase_size_1 != 0 {
            let opcode = bits!(dwords[7], 8, 8) as u8;
            if opcode != 0 {
                erase_insts[0] = Some(SFDPEraseInst {
                    opcode, size: 1 << erase_size_1, time_typ: None, time_max: None,
                });
            }
        }
        if erase_size_2 != 0 {
            let opcode = bits!(dwords[7], 8, 24) as u8;
            if opcode != 0 {
                erase_insts[1] = Some(SFDPEraseInst {
                    opcode, size: 1 << erase_size_2, time_typ: None, time_max: None,
                });
            }
        }
        if erase_size_3 != 0 {
            let opcode = bits!(dwords[8], 8, 8) as u8;
            if opcode != 0 {
                erase_insts[2] = Some(SFDPEraseInst {
                    opcode, size: 1 << erase_size_3, time_typ: None, time_max: None,
                });
            }
        }
        if erase_size_4 != 0 {
            let opcode = bits!(dwords[8], 8, 24) as u8;
            if opcode != 0 {
                erase_insts[3] = Some(SFDPEraseInst {
                    opcode, size: 1 << erase_size_4, time_typ: None, time_max: None,
                });
            }
        }

        // Subsequent DWORDs are only defined in JESD216A and above, which
        // use a minor version number of 5 and above. Create the FlashParams
        // object with what we've got so far, and update it with the extra
        // DWORDs if available.
        let mut params = FlashParams {
            version_major: major, version_minor: minor,
            address_bytes, density, legacy_4kb_erase_supported, legacy_erase_4kb_inst,
            legacy_volatile_write_en_inst, legacy_block_protect_volatile,
            legacy_byte_write_granularity, erase_insts,
            chip_erase_time_typ: None, chip_erase_time_max: None,
            first_byte_prog_time_typ: None, first_byte_prog_time_max: None,
            succ_byte_prog_time_typ: None, succ_byte_prog_time_max: None,
            page_prog_time_typ: None, page_prog_time_max: None,
            page_size: None, busy_poll_flag: None, busy_poll_status: None,
            reset_inst_f0: None, reset_inst_66_99: None, status_1_vol: None,
        };

        if minor < 5 {
            return Ok(params);
        }

        // 10th DWORD
        let erase_scale = bits!(dwords[9], 4, 0);
        if let Some(inst) = params.erase_insts[0].as_mut() {
            let typ = bits!(dwords[9], 7, 4);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }
        if let Some(inst) = params.erase_insts[1].as_mut() {
            let typ = bits!(dwords[9], 7, 11);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }
        if let Some(inst) = params.erase_insts[2].as_mut() {
            let typ = bits!(dwords[9], 7, 18);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }
        if let Some(inst) = params.erase_insts[3].as_mut() {
            let typ = bits!(dwords[9], 7, 25);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }

        // 11th DWORD
        let typ = bits!(dwords[10], 7, 24);
        let (typ, max) = Self::chip_erase_duration(typ, erase_scale);
        params.chip_erase_time_typ = Some(typ);
        params.chip_erase_time_max = Some(max);
        let program_scale = bits!(dwords[10], 4, 0);
        let typ = bits!(dwords[10], 5, 19);
        let (typ, max) = Self::byte_program_duration(typ, program_scale);
        params.succ_byte_prog_time_typ = Some(typ);
        params.succ_byte_prog_time_max = Some(max);
        let typ = bits!(dwords[10], 5, 14);
        let (typ, max) = Self::byte_program_duration(typ, program_scale);
        params.first_byte_prog_time_typ = Some(typ);
        params.first_byte_prog_time_max = Some(max);
        let typ = bits!(dwords[10], 6, 8);
        let (typ, max) = Self::page_program_duration(typ, program_scale);
        params.page_prog_time_typ = Some(typ);
        params.page_prog_time_max = Some(max);
        params.page_size = Some(1 << bits!(dwords[10], 4, 4));

        // 12th and 13th DWORDs skipped: suspend/resume.

        // 14th DWORD
        let status_reg_poll = bits!(dwords[13], 6, 2);
        params.busy_poll_flag = Some((status_reg_poll & 0b00_0010) != 0);
        params.busy_poll_status = Some((status_reg_poll & 0b00_0001) != 0);

        // 15th DWORD skipped: multiple I/O.

        // 16th DWORD
        let reset = bits!(dwords[15], 6, 8);
        params.reset_inst_f0 = Some((reset & 0b00_1000) != 0);
        params.reset_inst_66_99 = Some((reset & 0b01_0000) != 0);
        let vol = bits!(dwords[15], 7, 0);
        params.status_1_vol = Some(SFDPStatus1Volatility::from_bits(vol));

        Ok(params)
    }

    /// Get the flash capacity in bytes.
    pub fn capacity_bytes(&self) -> usize {
        ((self.density + 1) / 8) as usize
    }

    /// Get the smallest erase granularity and its opcode.
    pub fn sector_erase(&self) -> Option<(usize, u8)> {
        let mut size = u32::MAX;
        let mut opcode = 0u8;
        for inst in self.erase_insts.iter() {
            if let Some(inst) = inst {
                if inst.size < size {
                    size = inst.size;
                    opcode = inst.opcode;
                }
            }
        }

        if size != u32::MAX {
            Some((size as usize, opcode))
        } else {
            None
        }
    }

    /// Convert SFPD sector erase time to typical and maximum Duration.
    ///
    /// Uses scale factors of 1ms, 16ms, 128ms, and 1s.
    fn sector_erase_durations(typ: u32, max_scale: u32) -> (Duration, Duration) {
        let scale = match bits!(typ, 2, 5) {
            0b00 => Duration::from_millis(1),
            0b01 => Duration::from_millis(16),
            0b10 => Duration::from_millis(128),
            0b11 => Duration::from_secs(1),
            _    => unreachable!(),
        };
        let count = bits!(typ, 5, 0);
        let typ = (count + 1) * scale;
        let max = 2 * (max_scale + 1) * typ;
        (typ, max)
    }

    /// Convert SFPD chip erase time to typical and maximum Durations.
    ///
    /// Uses scale factors of 16ms, 256ms, 4s, and 64s.
    fn chip_erase_duration(typ: u32, max_scale: u32) -> (Duration, Duration) {
        let scale = match bits!(typ, 2, 5) {
            0b00 => Duration::from_millis(16),
            0b01 => Duration::from_millis(256),
            0b10 => Duration::from_secs(4),
            0b11 => Duration::from_secs(64),
            _    => unreachable!(),
        };
        let count = bits!(typ, 5, 0);
        let typ = (count + 1) * scale;
        let max = 2 * (max_scale + 1) * typ;
        (typ, max)
    }

    /// Convert SFPD byte program times to typical and maximum Durations.
    ///
    /// Uses scale factors of 1µs and 8µs.
    fn byte_program_duration(typ: u32, max_scale: u32) -> (Duration, Duration) {
        let scale = match bits!(typ, 1, 4) {
            0b0 => Duration::from_micros(1),
            0b1 => Duration::from_micros(8),
            _    => unreachable!(),
        };
        let count = bits!(typ, 4, 0);
        let typ = (count + 1) * scale;
        let max = 2 * (max_scale + 1) * typ;
        (typ, max)
    }

    /// Convert SFPD page program times to typical and maximum Durations.
    ///
    /// Uses scale factors of 8µs and 64µs.
    fn page_program_duration(typ: u32, max_scale: u32) -> (Duration, Duration) {
        let scale = match bits!(typ, 1, 5) {
            0b0 => Duration::from_micros(8),
            0b1 => Duration::from_micros(64),
            _    => unreachable!(),
        };
        let count = bits!(typ, 5, 0);
        let typ = (count + 1) * scale;
        let max = 2 * (max_scale + 1) * typ;
        (typ, max)
    }
}

/// Standard SPI flash command opcodes.
///
/// These are taken from the Winbond W25Q16JV datasheet, but most are
/// widely applicable. If SFDP is supported, it is used to discover
/// the relevant erase opcodes and sizes.
///
/// Only single I/O commands are listed.
#[derive(Copy, Clone, Debug, num_enum::IntoPrimitive)]
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
    ReadFlagStatusRegister = 0x70,
    WriteStatusRegister2 = 0x31,
    WriteStatusRegister3 = 0x11,
    WriteEnableVolatile = 0x50,
    EnableReset = 0x66,
    Reset = 0x99,
    ProgramSuspend = 0x75,
    ProgramResume = 0x7A,

    // Erase instructions.
    // The size affected by each erase operation can vary.
    // Typical sizes are 4kB for sector erase, 32kB for block erase 1,
    // and 64kB for block erase 2.
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

/// Status Register 1
#[derive(Copy, Clone, Debug)]
struct StatusRegister1(pub u8);

impl StatusRegister1 {
    /// Get BUSY bit.
    fn get_busy(&self) -> bool {
        self.0 & 0b0000_0001 != 0
    }

    /// Get (BP0, BP1, BP2) bits.
    fn get_block_protect(&self) -> (bool, bool, bool) {
        let bp = (self.0 & 0b0001_1100) >> 2;
        (bp & 0b001 != 0, bp & 0b010 != 0, bp & 0b100 != 0)
    }

    /// Set (BP0, BP1, BP2) bits.
    fn set_block_protect(&mut self, bp0: bool, bp1: bool, bp2: bool) {
        self.0 &= 0b1110_0011;
        self.0 |= ((bp0 as u8) << 2) | ((bp1 as u8) << 3) | ((bp2 as u8) << 4);
    }
}

/// Status Register 3
#[derive(Copy, Clone, Debug)]
struct StatusRegister3(pub u8);

impl StatusRegister3 {
    /// Get WPS (write protect selection) bit.
    fn get_wps(&self) -> bool {
        self.0 & 0b0000_0100 != 0
    }

    /// Set WPS (write protect selecetion) bit.
    fn set_wps(&mut self, wps: bool) {
        self.0 &= 0b1111_1011;
        self.0 |= (wps as u8) << 2;
    }
}
