use std::convert::TryInto;
use crate::ecp5::{ECP5, ECP5Flash, Error as ECP5Error};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Error during flash readback verification.")]
    ReadbackError,
    #[error("ECP5 error")]
    ECP5(#[from] ECP5Error),
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
    manufacturer_id: u8,
    device_id: u8,
    unique_id: u64,
}

impl std::fmt::Display for FlashID {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Manufacturer {:02X}, Device {:02X}, Unique ID {:016X}",
               self.manufacturer_id, self.device_id, self.unique_id)
    }
}

pub struct Flash {
    ecp5: ECP5Flash,
}

impl Flash {
    /// Create a new Flash instance wrapping an ECP5 instance.
    pub fn new(ecp5: ECP5) -> Result<Self> {
        let ecp5 = ecp5.to_flash()?;
        Ok(Flash { ecp5 })
    }

    /// Consume this Flash instance, returning the underlying ECP5 instance.
    pub fn release(self) -> ECP5 {
        self.ecp5.release()
    }

    /// Read the attached flash device, manufacturer, and unique IDs
    pub fn read_id(&mut self) -> Result<FlashID> {
        self.power_up()?;
        self.reset()?;
        let (manufacturer_id, device_id) = self.read_device_id()?;
        let unique_id = self.read_unique_id()?;
        Ok(FlashID { manufacturer_id, device_id, unique_id })
    }

    /// Read `length` bytes of data from the attached flash, starting at `address`
    pub fn read(&mut self, address: u32, length: usize) -> Result<Vec<u8>> {
        self.fast_read(address, length)
    }

    /// Program the attached flash with `data` starting at `address`.
    ///
    /// If `verify` is true, also read-back the programmed data and
    /// return FFPError::ReadbackError if it did not match what was written.
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
        assert!(data.len() >= 1, "Cannot program 0 bytes of data");
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

    fn read_device_id(&mut self) -> Result<(u8, u8)> {
        self.exchange(Command::ReadDeviceID, &[], 3+2)
            .map(|data| (data[3], data[4]))
    }

    fn read_unique_id(&mut self) -> Result<u64> {
        self.exchange(Command::ReadUniqueID, &[], 4+8)
            .and_then(|data| Ok(u64::from_be_bytes((&data[4..]).try_into().unwrap())))
    }

    fn read_status1(&mut self) -> Result<u8> {
        self.exchange(Command::ReadStatusRegister1, &[], 1).map(|data| data[0])
    }

    fn write_status(&mut self, status1: u8, status2: u8) -> Result<()> {
        self.write(Command::WriteStatusRegister, &[status1, status2])
    }

    #[allow(dead_code)]
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

    /// Writes `command` and `data` to the flash memory, then returns `nbytes` of response.
    fn exchange(&mut self, command: Command, data: &[u8], nbytes: usize) -> Result<Vec<u8>> {
        let mut tx = vec![command as u8];
        tx.extend(data);
        tx.extend(vec![0u8; nbytes]);
        let rx = self.ecp5.exchange(&tx)?;
        Ok(rx[1+data.len()..].to_vec())
    }

    /// Writes `command` and `data` to the flash memory, without reading the response.
    fn write(&mut self, command: Command, data: &[u8]) -> Result<()> {
        let mut tx = vec![command as u8];
        tx.extend(data);
        self.ecp5.write(&tx)?;
        Ok(())
    }

    /// Convenience method for issuing a single command and not caring about the returned data
    fn command(&mut self, command: Command) -> Result<()> {
        self.write(command, &[])?;
        Ok(())
    }
}
