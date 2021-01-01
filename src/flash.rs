use std::convert::TryInto;
use std::time::{Duration, Instant};
use indicatif::{ProgressBar, ProgressStyle};

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
    #[error("No erase instruction has been specified.")]
    NoEraseInstruction,
    #[error("No page size has been specified.")]
    NoPageSize,

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
    erase_size: Option<usize>,

    /// EraseSector instruction opcode.
    /// This is set to 0x20 by default but may be overridden.
    erase_opcode: u8,
}

const DATA_PROGRESS_TPL: &str =
    " {msg} [{bar:40}] {bytes}/{total_bytes} ({bytes_per_sec}; {eta_precise})";
const DATA_PROGRESS_CHARS: &str = "=> ";

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
            erase_size: None,
            erase_opcode: 0x20,
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
    pub fn erase_size(&self) -> Option<usize> {
        self.erase_size
    }

    /// Set the sector erase size in bytes.
    ///
    /// This must be known before sector erase operations can be performed.
    pub fn set_erase_size(&mut self, n: usize) {
        self.erase_size = Some(n);
    }

    /// Get the opcode used for the Erase Sector instruction.
    pub fn erase_opcode(&self) -> u8 {
        self.erase_opcode
    }

    /// Set the opcode used for the Erase Sector instruction.
    ///
    /// This is 0x20 by default.
    pub fn set_erase_opcode(&mut self, opcode: u8) {
        self.erase_opcode = opcode;
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
    /// Access errors are returned as usual, but if SFDP is not supported
    /// (no SFDP signature is detected in the first SFDP DWORD) then
    /// `Ok(None)` is returned instead.
    ///
    /// Depending on the version of SFDP supported, some fields may
    /// not be available.
    ///
    /// Once read, the parameters are available using `get_params()`,
    /// and the parameter values are automatically used for the
    /// configuration of address bytes, capacity, page size, sector size,
    /// and erase sector opcode. Additionally, larger erase commands
    /// described by the SFDP parameters will be used when appropriate.
    pub fn read_params(&mut self) -> Result<Option<FlashParams>> {
        log::debug!("Reading SFDP data");

        // Read just SFDP header to get NPH first.
        let data = self.read_sfdp(0, 8)?;
        let nph = data[5] as usize;

        // Re-read overall SFDP header including parameter headers.
        // Handle errors parsing the header by returning Ok(None),
        // since not all flash devices support SFDP.
        // After this parse is successful, however, subsequent errors
        // are returned as errors.
        let data = self.read_sfdp(0, 8 + nph*8)?;
        let header = match SFDPHeader::from_bytes(&data) {
            Ok(header) => header,
            Err(_) => return Ok(None),
        };

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
            self.erase_size = Some(size);
            self.erase_opcode = opcode;
        } else if params.legacy_4kb_erase_supported && params.legacy_4kb_erase_inst != 0xFF {
            self.erase_size = Some(4096);
            self.erase_opcode = params.legacy_4kb_erase_inst;
        }
        log::debug!("Updated settings from parameters:");
        log::debug!("Address bytes: {}, capacity: {:?} bytes",
                    self.address_bytes, self.capacity);
        log::debug!("Page size: {:?}, erase size: {:?}, erase op: {}",
                    self.page_size, self.erase_size, self.erase_opcode);

        Ok(Some(params))
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`.
    ///
    /// This method uses the FastRead instruction; if it is not supported
    /// try using `legacy_read()` instead.
    pub fn read(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        self.check_address_length(address, length)?;
        let mut param = self.make_address(address);
        // Dummy byte after address.
        param.push(0);
        self.exchange(Command::FastRead, &param, length)
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`.
    ///
    /// This method uses the legacy ReadData instruction, which often has a low
    /// maximum clock speed compared to other operations, but is more widely supported
    /// and may be faster for very short reads as it does not require a dummy byte.
    pub fn legacy_read(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        self.check_address_length(address, length)?;
        let param = self.make_address(address);
        self.exchange(Command::ReadData, &param, length)
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`.
    ///
    /// This method is similar to the `read()` method, except it calls the provided
    /// callback function at regular intervals with the number of bytes read so far.
    ///
    /// While `read()` performs a single long SPI exchange, this method performs
    /// up to 128 separate SPI exchanges to allow progress to be reported.
    pub fn read_cb<F: Fn(usize)>(&mut self, address: u32, length: usize, cb: F)
        -> Result<Vec<u8>>
    {
        self.check_address_length(address, length)?;
        let chunk_size = usize::max(1024, length / 128);
        let start = address as usize;
        let end = start + length;
        let mut data = Vec::new();
        cb(0);
        for addr in (start..end).step_by(chunk_size) {
            let size = usize::min(chunk_size, end - addr);
            let mut param = self.make_address(addr as u32);
            param.push(0);
            data.append(&mut self.exchange(Command::FastRead, &param, size)?);
            cb(data.len());
        }
        cb(data.len());
        Ok(data)
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`.
    ///
    /// This method is similar to the `read()` method, except it renders a progress
    /// bar to the terminal during the read.
    ///
    /// While `read()` performs a single long SPI exchange, this method performs
    /// up to 128 separate SPI exchanges to allow progress to be reported.
    pub fn read_progress(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        let pb = ProgressBar::new(length as u64).with_style(ProgressStyle::default_bar()
            .template(DATA_PROGRESS_TPL).progress_chars(DATA_PROGRESS_CHARS));
        pb.set_message("Reading");
        let result = self.read_cb(address, length, |n| pb.set_position(n as u64));
        pb.finish();
        result
    }

    /// Erase entire flash chip.
    ///
    /// This method uses the ChipErase instruction, so no progress information
    /// is available, but the typical and maximum times taken may be available
    /// from the SFDP parameters.
    ///
    /// Returns only after erase operation is complete.
    pub fn erase(&mut self) -> Result<()> {
        self.write_enable()?;
        self.command(Command::ChipErase)?;
        self.wait_while_busy()?;
        Ok(())
    }

    /// Erase entire flash chip.
    ///
    /// This method is identical to `erase()`, except it draws a progress bar
    /// to the terminal during the erase operation.
    ///
    /// If SFDP data indicates a typical chip erase time, that value is used,
    /// otherwise the progress bar is drawn as a spinner.
    pub fn erase_progress(&mut self) -> Result<()> {
        let time = self.params.map(|p| p.timing.map(|t| t.chip_erase_time_typ));
        let pb = if let Some(Some(time)) = time {
            ProgressBar::new(time.as_millis() as u64).with_style(ProgressStyle::default_bar()
            .template(" {msg} [{bar:40}] {elapsed} < {eta}")
            .progress_chars("=> "))
        } else {
            ProgressBar::new_spinner()
        };
        pb.set_message("Erasing");
        let t0 = Instant::now();
        self.write_enable()?;
        self.command(Command::ChipErase)?;
        while self.is_busy()? {
            let t = t0.elapsed().as_millis() as u64;
            pb.set_position(t);
        }
        pb.finish();
        Ok(())
    }

    /// Program the attached flash with `data` starting at `address`.
    ///
    /// Sectors and blocks are erased as required for the new data,
    /// and existing data outside the new data to write is written
    /// back if it has to be erased.
    ///
    /// If `verify` is true, the programmed data is read back, and
    /// a ReadbackError will be returned if it did not match what was written.
    ///
    /// When available, SFDP parameters are used to generate an efficient
    /// sequence of erase instructions. If unavailable, the single erase
    /// instruction in `erase_opcode` is used, and its size of effect
    /// must be given in `erase_size`. If these are not set, a
    /// `NoEraseInstruction` is returned.
    ///
    /// The programming page size is set by `page_size`, which is
    /// automatically set when SFDP parameters are read.
    pub fn program(&mut self, address: u32, data: &[u8], verify: bool) -> Result<()> {
        self.check_address_length(address, data.len())?;

        // Work out a good erasure plan.
        let erase_plan = self.make_erase_plan(address, data.len())?;

        // Read data which will be inadvertently erased so we can restore it.
        let full_data = self.make_restore_data(address, data, &erase_plan)?;

        // Execute erasure plan.
        self.run_erase_plan(&erase_plan, |_| {})?;

        // Write new data.
        let start_addr = erase_plan.0[0].2;
        self.program_data(start_addr, &full_data)?;

        // Optionally do a readback to verify all written data.
        if verify {
            let programmed = self.read(start_addr, full_data.len())?;
            if programmed == full_data {
                Ok(())
            } else {
                log::error!("Readback verification failed. Check flash protection bits.");
                Err(Error::ReadbackError)
            }
        } else {
            Ok(())
        }
    }

    /// Program the attached flash with `data` starting at `address`.
    ///
    /// This is identical to `program()`, except it also draws progress bars to the terminal.
    pub fn program_progress(&mut self, address: u32, data: &[u8], verify: bool) -> Result<()> {
        self.check_address_length(address, data.len())?;

        // Work out a good erasure plan.
        let erase_plan = self.make_erase_plan(address, data.len())?;

        // Read data which will be inadvertently erased so we can restore it.
        let full_data = self.make_restore_data(address, data, &erase_plan)?;

        // Execute erasure plan.
        self.run_erase_plan_progress(&erase_plan)?;

        // Write new data.
        let start_addr = erase_plan.0[0].2;
        self.program_data_progress(start_addr, &full_data)?;

        // Optionally do a readback to verify all written data.
        if verify {
            let programmed = self.read_progress(start_addr, full_data.len())?;
            if programmed == full_data {
                Ok(())
            } else {
                log::error!("Readback verification failed. Check flash protection bits.");
                Err(Error::ReadbackError)
            }
        } else {
            Ok(())
        }
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

    /// Set block protection bits.
    ///
    /// This sets the block protect bits in status register 1,
    /// using the non-volatile commands if supported. If available,
    /// the SFDP parameters are used to determine the correct
    /// non-volatile instruction.
    pub fn protect(&mut self, bp0: bool, bp1: bool, bp2: bool) -> Result<()> {
        log::debug!("Setting block protection bits to BP0={}, BP1={}, BP2={}", bp0, bp1, bp2);
        let mut status1 = self.read_status1()?;
        status1.set_block_protect(bp0, bp1, bp2);
        self.write_status1(status1)?;
        self.wait_while_busy()?;
        Ok(())
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

    /// Program `data` to `address`, automatically split into multiple page program operations.
    ///
    /// Note that this does *not* erase the flash beforehand; use `program()` for a higher-level
    /// erase-program-verify interface.
    pub fn program_data(&mut self, address: u32, data: &[u8]) -> Result<()> {
        self.program_data_cb(address, data, |_| {})
    }

    /// Program `data` to `address`, automatically split into multiple page program operations,
    /// and draws a progress bar to the terminal.
    ///
    /// Note that this does *not* erase the flash beforehand;
    /// use `program()` for a higher-level erase-program-verify interface.
    pub fn program_data_progress(&mut self, address: u32, data: &[u8]) -> Result<()> {
        let pb = ProgressBar::new(data.len() as u64).with_style(ProgressStyle::default_bar()
            .template(DATA_PROGRESS_TPL).progress_chars(DATA_PROGRESS_CHARS));
        pb.set_message("Writing");
        self.program_data_cb(address, &data, |n| pb.set_position(n as u64))?;
        pb.finish();
        Ok(())
    }

    /// Program `data` to `address`, automatically split into multiple page program operations.
    ///
    /// Note that this does *not* erase the flash beforehand; use `program()` for a higher-level
    /// erase-program-verify interface.
    ///
    /// Calls `cb` with the number of bytes programmed so far after each
    /// page programming operation.
    pub fn program_data_cb<F: Fn(usize)>(&mut self, address: u32, mut data: &[u8], cb: F)
        -> Result<()>
    {
        let page_size = match self.page_size {
            Some(page_size) => page_size,
            None => {
                log::error!("No page size available. Set one with `set_page_size()`.");
                return Err(Error::NoPageSize);
            }
        };

        log::trace!("Programming data to 0x{:08X}, page size {} bytes", address, page_size);

        let mut total_bytes = 0;
        cb(total_bytes);

        // If the address is not page-aligned, we need to do a
        // smaller-than-page-size initial program.
        let first_write = page_size - ((address as usize) % page_size);
        if first_write != page_size {
            log::trace!("Programming partial first page of {} bytes", first_write);
            self.write_enable()?;
            self.page_program(address, &data[..first_write])?;
            self.wait_while_busy()?;
            total_bytes += first_write;
            data = &data[first_write..];
            cb(total_bytes);
        }

        for page_data in data.chunks(page_size) {
            self.write_enable()?;
            self.page_program(address + total_bytes as u32, page_data)?;
            self.wait_while_busy()?;
            total_bytes += page_data.len();
            cb(total_bytes);
        }

        Ok(())
    }

    /// Send the WriteEnable command, setting the WEL in the status register.
    pub fn write_enable(&mut self) -> Result<()> {
        self.command(Command::WriteEnable)
    }

    /// Program up to one page of data.
    ///
    /// This method sets the write-enable latch and then waits for programming to complete,
    /// including sleeping half the typical page program time if known before polling.
    ///
    /// Note that this does *not* erase the flash beforehand;
    /// use `program()` for a higher-level erase-program-verify interface.
    pub fn page_program(&mut self, address: u32, data: &[u8]) -> Result<()> {
        let mut tx = self.make_address(address);
        tx.extend(data);
        self.write_enable()?;
        self.exchange(Command::PageProgram, &tx, 0)?;
        if let Some(params) = self.params {
            if let Some(timing) = params.timing {
                std::thread::sleep(timing.page_prog_time_typ / 2);
            }
        }
        self.wait_while_busy()?;
        Ok(())
    }

    /// Reads the JEDEC manufacturer and long (16-bit) device IDs.
    ///
    /// The manufacturer ID may be prefixed with up to 13 of the
    /// continuation code 0x7F; the number of continuation codes
    /// is returned as the bank number.
    ///
    /// Returns (bank, manufacturer ID, device ID).
    pub fn read_jedec_id(&mut self) -> Result<(u8, u8, u16)> {
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
    pub fn read_device_id(&mut self) -> Result<(u8, u8, u8)> {
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
    pub fn read_unique_id(&mut self) -> Result<u64> {
        self.exchange(Command::ReadUniqueID, &[0, 0, 0, 0], 8)
            .map(|data| u64::from_be_bytes(data.try_into().unwrap()))
    }

    /// Read status register 1.
    pub fn read_status1(&mut self) -> Result<StatusRegister1> {
        self.exchange(Command::ReadStatusRegister1, &[], 1).map(|data| StatusRegister1(data[0]))
    }

    /// Read status register 2.
    ///
    /// This status register is less widely supported and SFDP does
    /// not indicate whether or not it is present.
    pub fn read_status2(&mut self) -> Result<StatusRegister2> {
        self.exchange(Command::ReadStatusRegister2, &[], 1).map(|data| StatusRegister2(data[0]))
    }

    /// Read status register 3.
    ///
    /// This status register is less widely supported and SFDP does
    /// not indicate whether or not it is present.
    pub fn read_status3(&mut self) -> Result<StatusRegister3> {
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
        let s1 = self.read_status1()?;
        log::debug!("Set WEL, s1 now: {:02X}", s1.0);
        self.write(Command::WriteStatusRegister1, &[status1.0])
    }

    /// Write status register 2.
    pub fn write_status2(&mut self, status2: StatusRegister2) -> Result<()> {
        self.write_enable()?;
        self.write(Command::WriteStatusRegister2, &[status2.0])
    }

    /// Write status register 3.
    pub fn write_status3(&mut self, status3: StatusRegister3) -> Result<()> {
        self.write_enable()?;
        self.write(Command::WriteStatusRegister3, &[status3.0])
    }

    /// Check if the device is currently busy performing an operation.
    ///
    /// If the flash parameters indicate support for the Flag Status Register
    /// instruction (0x70), it is used, otherwise legacy polling of status
    /// register 1 is used.
    pub fn is_busy(&mut self) -> Result<bool> {
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
    pub fn wait_while_busy(&mut self) -> Result<()> {
        while self.is_busy()? {}
        Ok(())
    }

    /// Read SFDP register data.
    ///
    /// `addr` is always sent as a 24-bit address, regardless of the address_bytes setting.
    pub fn read_sfdp(&mut self, addr: u32, len: usize) -> Result<Vec<u8>> {
        let bytes = addr.to_be_bytes();
        self.exchange(Command::ReadSFDPRegister, &bytes[1..], 1+len)
            .map(|data| data[1..].to_vec())
    }

    /// Writes `command` and `data` to the flash memory, then returns `nbytes` of response.
    pub fn exchange<C: Into<u8>>(&mut self, command: C, data: &[u8], nbytes: usize)
        -> Result<Vec<u8>>
    {
        let mut tx = vec![command.into()];
        tx.extend(data);
        log::trace!("SPI exchange: write {:02X?}, read {} bytes", &tx, nbytes);
        tx.extend(vec![0u8; nbytes]);
        let rx = self.access.exchange(&tx)?;
        log::trace!("SPI exchange: read {:02X?}", &rx[1+data.len()..]);
        Ok(rx[1+data.len()..].to_vec())
    }

    /// Writes `command` and `data` to the flash memory, without reading the response.
    pub fn write<C: Into<u8>>(&mut self, command: C, data: &[u8]) -> Result<()> {
        let mut tx = vec![command.into()];
        tx.extend(data);
        log::trace!("SPI write: {:02X?}", &tx);
        self.access.write(&tx)?;
        Ok(())
    }

    /// Convenience method for issuing a single command and not caring about the returned data
    pub fn command<C: Into<u8>>(&mut self, command: C) -> Result<()> {
        self.write(command, &[])?;
        Ok(())
    }

    /// Checks if `address` and `length` together are permissible:
    /// * `address` must not exceed the current number of address bytes
    /// * Both `address` and `address+length` must be within the flash memory bounds,
    ///   if the capacity is known.
    /// Returns either Err(Error::InvalidAddress) or Ok(()).
    fn check_address_length(&self, address: u32, length: usize) -> Result<()> {
        log::debug!("Checking address={:08X} length={}", address, length);
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

    /// Generate a 1-, 2-, 3-, or 4-byte address, depending on current `address_bytes` setting.
    ///
    /// Panics if address_bytes is not 1-, 2, 3, or 4.
    fn make_address(&self, addr: u32) -> Vec<u8> {
        let bytes = addr.to_be_bytes();
        bytes[(4 - self.address_bytes as usize)..].to_vec()
    }

    /// Work out what combination of erase operations to run to efficiently
    /// erase the specified memory.
    fn make_erase_plan(&self, address: u32, length: usize) -> Result<ErasePlan> {
        log::debug!("Creating erase plan for address={} length={}", address, length);
        // Erase instructions: (size in bytes, opcode).
        let mut insts = Vec::new();

        // Find available erase instructions.
        if let Some(params) = self.params {
            if params.erase_insts.iter().any(|&inst| inst.is_some()) {
                log::trace!("Using SFDP erase instructions.");
                for inst in params.erase_insts.iter() {
                    if let Some(inst) = inst {
                        insts.push((inst.size as usize, inst.opcode, inst.time_typ));
                    }
                }
            } else if params.legacy_4kb_erase_supported {
                log::trace!("No erase instructions in SFDP, using legacy 4kB erase.");
                insts.push((4096, params.legacy_4kb_erase_inst, None));
            } else {
                log::trace!("SFDP indicates no erase instructions available.");
            }
        }
        if insts.is_empty() {
            if let Some(erase_size) = self.erase_size {
                log::trace!("No SFDP erase instructions found, using `erase_size` parameter.");
                insts.push((erase_size, self.erase_opcode, None));
            } else {
                log::warn!("No erase instructions could be found.");
                log::warn!("Try setting one manually using `Flash::set_erase_size()`.");
                return Err(Error::NoEraseInstruction);
            }
        }
        insts.sort();

        // Create plan given the list of available erase instructions.
        Ok(ErasePlan::new(&insts, address as usize, length))
    }

    /// Read all the bytes before `address` in memory which will be erased by `plan`.
    ///
    /// If all those bytes are 0xFF, returns an empty Vec instead, as they won't be changed
    /// by the erase operation.
    fn read_erase_preamble(&mut self, address: u32, plan: &ErasePlan) -> Result<Vec<u8>> {
        let base = plan.0[0].2;
        let len = address - base;
        if len > 0 {
            log::debug!("Reading erase preamble: base={} len={}", base, len);
            let data = self.read(base, len as usize)?;
            // If all the preamble is already 0xFF, there's no point reprogramming it.
            if data.iter().all(|x| *x == 0xFF) {
                Ok(Vec::new())
            } else {
                Ok(data)
            }
        } else {
            Ok(Vec::new())
        }
    }

    /// Read all the bytes after `address + length` in memory which will be erased by `plan`.
    ///
    /// If all those bytes are 0xFF, returns an empty Vec instead, as they won't be changed
    /// by the erase operation.
    fn read_erase_postamble(&mut self, address: u32, length: usize, plan: &ErasePlan)
        -> Result<Vec<u8>>
    {
        let (_, size, base, _) = plan.0.last().unwrap();
        let start = address + (length as u32);
        let len = (*base as usize + *size) - start as usize;
        if len > 0 {
            log::debug!("Reading erase postamble: addr={} len={}", start, len);
            let data = self.read(start, len)?;
            // If all the postamble is already 0xFF, there's no point reprogramming it.
            if data.iter().all(|x| *x == 0xFF) {
                Ok(Vec::new())
            } else {
                Ok(data)
            }
        } else {
            Ok(Vec::new())
        }
    }

    /// Extend `data` by adding any preamble and postamble required to preserve
    /// existing data after erasing and reprogramming.
    fn make_restore_data(&mut self, address: u32, data: &[u8], erase_plan: &ErasePlan)
        -> Result<Vec<u8>>
    {
        let preamble = self.read_erase_preamble(address, &erase_plan)?;
        let postamble = self.read_erase_postamble(address, data.len(), &erase_plan)?;
        let mut full_data = preamble;
        full_data.extend(data);
        full_data.extend(&postamble);
        Ok(full_data)
    }

    /// Execute the sequence of erase operations from `plan`.
    ///
    /// `cb` is called with the number of bytes erased so far.
    fn run_erase_plan<F: Fn(usize)>(&mut self, plan: &ErasePlan, cb: F) -> Result<()> {
        let mut total_erased = 0;
        cb(total_erased);
        for (opcode, size, base, duration) in plan.0.iter() {
            log::trace!("Executing erase plan: Erase 0x{:02X} ({} bytes) from 0x{:08X}",
                        opcode, size, base);
            let addr = self.make_address(*base);
            self.write_enable()?;
            self.write(*opcode, &addr)?;
            if let Some(duration) = duration {
                std::thread::sleep(*duration / 2);
            }
            self.wait_while_busy()?;
            total_erased += size;
            cb(total_erased);
        }
        cb(total_erased);
        Ok(())
    }

    /// Execute the sequence of erase operations from `plan`, and draw a progress bar
    /// to the terminal.
    fn run_erase_plan_progress(&mut self, plan: &ErasePlan) -> Result<()> {
        let erase_size = plan.total_size() as u64;
        let pb = ProgressBar::new(erase_size).with_style(ProgressStyle::default_bar()
            .template(DATA_PROGRESS_TPL).progress_chars(DATA_PROGRESS_CHARS));
        pb.set_message("Erasing");
        self.run_erase_plan(&plan, |n| pb.set_position(n as u64))?;
        pb.finish();
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
        let unique_id = match self.unique_id {
            0x0000_0000_0000_0000 | 0xFFFF_FFFF_FFFF_FFFF => "".to_string(),
            id => format!(", Unique ID: {:016X}", id),
        };
        write!(f, "Manufacturer 0x{:02X}{}, Device 0x{:02X}/0x{:04X}{}",
               self.manufacturer_id, mfn, self.device_id_short,
               self.device_id_long, unique_id)
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
    pub legacy_4kb_erase_inst: u8,
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

    /// Chip erase and programming times, if available.
    pub timing: Option<SFDPTiming>,

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

impl std::fmt::Display for SFDPEraseInst {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Opcode 0x{:02X}: {} bytes", self.opcode, self.size)?;
        if let Some(typ) = self.time_typ {
            write!(f, ", typ {:?}", typ)?;
        }
        if let Some(max) = self.time_max {
            write!(f, ", max {:?}", max)?;
        }
        Ok(())
    }
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

/// Struct of timing information from JESD216A-compliant tables.
///
/// Note that erase instruction timing is stored inside the respective erase instructions.
#[derive(Copy, Clone, Debug)]
pub struct SFDPTiming {
    /// Typical time to erase the entire chip, if known.
    pub chip_erase_time_typ: Duration,
    /// Maximum time to erase the entire chip, if known.
    pub chip_erase_time_max: Duration,
    /// Typical time to program the first byte in a sequence, if known.
    pub first_byte_prog_time_typ: Duration,
    /// Maximum time to program the first byte in a sequence, if known.
    pub first_byte_prog_time_max: Duration,
    /// Typical time to program each successive byte in a sequence, if known.
    pub succ_byte_prog_time_typ: Duration,
    /// Maximum time to program each successive byte in a sequence, if known.
    pub succ_byte_prog_time_max: Duration,
    /// Typical time to program a full page, if known.
    pub page_prog_time_typ: Duration,
    /// Maximum time to program a full page, if known.
    pub page_prog_time_max: Duration,
}

/// Bitfield extraction helper macro.
///
/// `bits!(word, length, offset)` extracts `length` number of bits at offset `offset`.
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

        // Parse the first 9 DWORDs, which must always be available if SFDP is supported.
        let mut params = Self::read_jesd216(major, minor, &dwords);

        // 1.5: JESD216A adds DWORDs 10-16.
        if minor >= 5 {
            params.read_jesd216a(&dwords);
        }

        // 1.6: JESD216B adds quad-SPI information to DWORD15 bits 8, 14, and 18.
        // 1.7: JESD216C adds DWORDs 17-20 which describe octal SPI.
        // 1.8: JESD216D doesn't change the basic flash parameters table.

        Ok(params)
    }

    /// Get the flash capacity in bytes.
    pub fn capacity_bytes(&self) -> usize {
        (self.density / 8) as usize
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

    /// Read the legacy information from JESD216 (DWORDs 1-9) and create a new FlashParams object.
    fn read_jesd216(major: u8, minor: u8, dwords: &[u32]) -> FlashParams {
        // 1st DWORD
        let address_bytes = SFDPAddressBytes::from_bits(bits!(dwords[0], 2, 17));
        let legacy_4kb_erase_inst = bits!(dwords[0], 8, 8) as u8;
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
            (dwords[1] as u64) + 1
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

        // Return a FlashParams with the legacy information set and further information
        // cleared, which can be filled in if additional DWORDs are available.
        FlashParams {
            version_major: major, version_minor: minor,
            address_bytes, density, legacy_4kb_erase_supported, legacy_4kb_erase_inst,
            legacy_volatile_write_en_inst, legacy_block_protect_volatile,
            legacy_byte_write_granularity, erase_insts,
            timing: None, page_size: None, busy_poll_flag: None, busy_poll_status: None,
            reset_inst_f0: None, reset_inst_66_99: None, status_1_vol: None,
        }
    }

    /// Parse JESD216A DWORDs 10 to 16.
    fn read_jesd216a(&mut self, dwords: &[u32]) {
        // 10th DWORD: erase instruction timings.
        let erase_scale = bits!(dwords[9], 4, 0);
        if let Some(inst) = self.erase_insts[0].as_mut() {
            let typ = bits!(dwords[9], 7, 4);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }
        if let Some(inst) = self.erase_insts[1].as_mut() {
            let typ = bits!(dwords[9], 7, 11);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }
        if let Some(inst) = self.erase_insts[2].as_mut() {
            let typ = bits!(dwords[9], 7, 18);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }
        if let Some(inst) = self.erase_insts[3].as_mut() {
            let typ = bits!(dwords[9], 7, 25);
            let (typ, max) = Self::sector_erase_durations(typ, erase_scale);
            inst.time_typ = Some(typ);
            inst.time_max = Some(max);
        }

        // 11th DWORD: chip erase and programming timings, page size.
        let typ = bits!(dwords[10], 7, 24);
        let (chip_erase_time_typ, chip_erase_time_max) =
            Self::chip_erase_duration(typ, erase_scale);
        let program_scale = bits!(dwords[10], 4, 0);
        let typ = bits!(dwords[10], 5, 19);
        let (succ_byte_prog_time_typ, succ_byte_prog_time_max) =
            Self::byte_program_duration(typ, program_scale);
        let typ = bits!(dwords[10], 5, 14);
        let (first_byte_prog_time_typ, first_byte_prog_time_max) =
            Self::byte_program_duration(typ, program_scale);
        let typ = bits!(dwords[10], 6, 8);
        let (page_prog_time_typ, page_prog_time_max) =
            Self::page_program_duration(typ, program_scale);
        self.timing = Some(SFDPTiming {
            chip_erase_time_typ, chip_erase_time_max,
            succ_byte_prog_time_typ, succ_byte_prog_time_max,
            first_byte_prog_time_typ, first_byte_prog_time_max,
            page_prog_time_typ, page_prog_time_max,
        });
        self.page_size = Some(1 << bits!(dwords[10], 4, 4));

        // 12th and 13th DWORDs skipped: suspend/resume.

        // 14th DWORD
        let status_reg_poll = bits!(dwords[13], 6, 2);
        self.busy_poll_flag = Some((status_reg_poll & 0b00_0010) != 0);
        self.busy_poll_status = Some((status_reg_poll & 0b00_0001) != 0);

        // 15th DWORD skipped: multiple I/O.

        // 16th DWORD
        let reset = bits!(dwords[15], 6, 8);
        self.reset_inst_f0 = Some((reset & 0b00_1000) != 0);
        self.reset_inst_66_99 = Some((reset & 0b01_0000) != 0);
        let vol = bits!(dwords[15], 7, 0);
        self.status_1_vol = Some(SFDPStatus1Volatility::from_bits(vol));
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

impl std::fmt::Display for FlashParams {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        writeln!(f, "SFDP JEDEC Basic Flash Parameter Table v{}.{}",
               self.version_major, self.version_minor)?;
        writeln!(f, "  Density: {} bits ({} KiB)", self.density, self.capacity_bytes() / 1024)?;
        writeln!(f, "  Address bytes: {:?}", self.address_bytes)?;
        writeln!(f, "  Legacy information:")?;
        writeln!(f, "    4kB erase supported: {}", self.legacy_4kb_erase_supported)?;
        writeln!(f, "    4kB erase opcode: 0x{:02X}", self.legacy_4kb_erase_inst)?;
        writeln!(f, "    Block Protect always volatile: {}", self.legacy_block_protect_volatile)?;
        writeln!(f, "    Volatile write enable opcode: 0x{:02X}", self.legacy_volatile_write_en_inst)?;
        writeln!(f, "    Writes have byte granularity: {}", self.legacy_byte_write_granularity)?;
        writeln!(f, "  Erase instructions:")?;
        for i in 0..4 {
            if let Some(inst) = self.erase_insts[i] {
                writeln!(f, "    {}: {}", i+1, inst)?;
            } else {
                writeln!(f, "    {}: Not present", i+1)?;
            }
        }
        if let Some(timing) = self.timing {
            writeln!(f, "  Timing:")?;
            writeln!(f, "    Chip erase: typ {:?}, max {:?}",
                     timing.chip_erase_time_typ, timing.chip_erase_time_max)?;
            writeln!(f, "    First byte program: typ {:?}, max {:?}",
                     timing.first_byte_prog_time_typ, timing.first_byte_prog_time_max)?;
            writeln!(f, "    Subsequent byte program: typ {:?}, max {:?}",
                     timing.succ_byte_prog_time_typ, timing.succ_byte_prog_time_max)?;
            writeln!(f, "    Page program: typ {:?}, max {:?}",
                     timing.page_prog_time_typ, timing.page_prog_time_max)?;
        }
        if let Some(page_size) = self.page_size {
            writeln!(f, "  Page size: {} bytes", page_size)?;
        }
        if let Some(busy_poll_flag) = self.busy_poll_flag {
            writeln!(f, "  Poll busy from FSR: {}", busy_poll_flag)?;
        }
        if let Some(busy_poll_status) = self.busy_poll_status {
            writeln!(f, "  Poll busy from SR1: {}", busy_poll_status)?;
        }
        if let Some(reset_inst_f0) = self.reset_inst_f0 {
            writeln!(f, "  Reset using opcode 0xF0: {}", reset_inst_f0)?;
        }
        if let Some(reset_inst_66_99) = self.reset_inst_66_99 {
            writeln!(f, "  Reset using opcodes 0x66, 0x99: {}", reset_inst_66_99)?;
        }
        if let Some(status_1_vol) = self.status_1_vol {
            writeln!(f, "  Status register 1 volatility: {:?}", status_1_vol)?;
        }
        Ok(())
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
pub struct StatusRegister1(pub u8);

impl StatusRegister1 {
    /// Get BUSY bit.
    pub fn get_busy(&self) -> bool {
        self.0 & 0b0000_0001 != 0
    }

    /// Get (BP0, BP1, BP2) bits.
    pub fn get_block_protect(&self) -> (bool, bool, bool) {
        let bp = (self.0 & 0b0001_1100) >> 2;
        (bp & 0b001 != 0, bp & 0b010 != 0, bp & 0b100 != 0)
    }

    /// Set (BP0, BP1, BP2) bits.
    fn set_block_protect(&mut self, bp0: bool, bp1: bool, bp2: bool) {
        self.0 &= 0b1110_0011;
        self.0 |= ((bp0 as u8) << 2) | ((bp1 as u8) << 3) | ((bp2 as u8) << 4);
    }

    /// Get SEC (sector protect) bit.
    pub fn get_sec(&self) -> bool {
        self.0 & 0b0100_0000 != 0
    }

    /// Get TB (top/bottom protection) bit.
    pub fn get_tb(&self) -> bool {
        self.0 & 0b0010_0000 != 0
    }

    /// Get SRP (status register protect) bit.
    pub fn get_srp(&self) -> bool {
        self.0 & 0b1000_0000 != 0
    }
}

/// Status Register 2
#[derive(Copy, Clone, Debug)]
pub struct StatusRegister2(pub u8);

impl StatusRegister2 {
    /// Get CMP (protection complement) bit.
    pub fn get_cmp(&self) -> bool {
        self.0 & 0b0100_0000 != 0
    }
}

/// Status Register 3
#[derive(Copy, Clone, Debug)]
pub struct StatusRegister3(pub u8);

impl StatusRegister3 {
    /// Get WPS (write protect selection) bit.
    fn get_wps(&self) -> bool {
        self.0 & 0b0000_0100 != 0
    }

    /// Set WPS (write protect selection) bit.
    fn set_wps(&mut self, wps: bool) {
        self.0 &= 0b1111_1011;
        self.0 |= (wps as u8) << 2;
    }
}

/// Erase plan of (opcode, size, base address, typical duration) to erase a range of memory.
#[derive(Clone, Debug)]
struct ErasePlan(Vec<(u8, usize, u32, Option<Duration>)>);

impl ErasePlan {
    fn new(insts: &[(usize, u8, Option<Duration>)], start: usize, length: usize) -> Self {
        log::trace!("Creating erase plan, start={} length={}", start, length);
        let mut plan = Vec::new();

        // Sort instructions by smallest area of effect first.
        let mut insts = insts.to_vec();
        insts.sort();

        // We compute the number of useful bytes erased for each operation,
        // then from those with the same maximum number of useful bytes erased,
        // we select the smallest operation, and repeat until all bytes are erased.
        let end = start + length;
        let mut pos = start;
        while pos < end {
            log::trace!("Evaluating candidates, pos={} end={}", pos, end);
            // Current candidate, (bytes, size, opcode, base).
            let mut candidate = (0, usize::MAX, 0, 0, None);
            for (erase_size, opcode, duration) in insts.iter() {
                let erase_base = pos - (pos % erase_size);
                let erase_end = erase_base + erase_size - 1;
                let mut bytes = erase_size - (pos - erase_base);
                if erase_end > end {
                    bytes -= erase_end - end;
                }
                log::trace!("  Candidate 0x{:02X} ({} bytes): base={} end={} bytes={}",
                            opcode, erase_size, erase_base, erase_end, bytes);
                if bytes > candidate.0 || (bytes == candidate.0 && *erase_size < candidate.1) {
                    candidate = (bytes, *erase_size, *opcode, erase_base, *duration);
                }
            }

            log::trace!("Candidate selected: {:?}", candidate);
            pos += candidate.0;
            plan.push((candidate.2, candidate.1, candidate.3 as u32, candidate.4));
        }

        log::debug!("Erase plan: {:?}", plan);

        ErasePlan(plan)
    }

    fn total_size(&self) -> usize {
        self.0.iter().map(|x| x.1).sum()
    }
}

#[test]
fn test_erase_plan() {
    let insts = &[(4, 1, None), (32, 2, None), (64, 3, None)];
    // Use a single 64kB erase to erase an aligned 64kB block.
    assert_eq!(ErasePlan::new(insts, 0, 64).0,
               vec![(3, 64, 0, None)]);
    // Use three 64kB erases to erase an aligned 192kB block.
    assert_eq!(ErasePlan::new(insts, 0, 192).0,
               vec![(3, 64, 0, None), (3, 64, 64, None), (3, 64, 128, None)]);
    // Use 64kB followed by 32kB to erase an aligned 70kB block.
    assert_eq!(ErasePlan::new(insts, 0, 70).0,
               vec![(3, 64, 0, None), (2, 32, 64, None)]);
    // Use 64kB followed by 4kB to erase an aligned 66kB block.
    assert_eq!(ErasePlan::new(insts, 0, 66).0,
               vec![(3, 64, 0, None), (1, 4, 64, None)]);
    // Use 4kB followed by 64kB to erase a misaligned 64kB block.
    assert_eq!(ErasePlan::new(insts, 62, 64).0,
               vec![(1, 4, 60, None), (3, 64, 64, None)]);
    // Use a 4kB, 64kB, 4kB to erase a misaligned 68kB block.
    assert_eq!(ErasePlan::new(insts, 62, 68).0,
               vec![(1, 4, 60, None), (3, 64, 64, None), (1, 4, 128, None)]);
}
