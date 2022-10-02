// Copyright 2022 Adam Greig
// Licensed under the Apache-2.0 and MIT licenses.

use std::{fs::File, path::Path, io::Read, collections::HashSet};
use num_enum::TryFromPrimitive;
use crate::{Result, Error, ECP5IDCODE};

mod parser;

/// ECP5 bitstream in-memory.
///
/// Can be loaded from a path or file, and modified as required to
/// change IDCODE checks and SPI mode and frequency.
pub struct Bitstream {
    /// Full bitstream data.
    data: Vec<u8>,

    /// Parsed metadata.
    meta: Option<BitstreamMeta>,
}

/// Relevant metadata parsed from a bitstream.
#[derive(Clone, Debug, PartialEq, Eq)]
struct BitstreamMeta {
    /// IDCODE and offset of VERIFY_ID command in bitstream.
    verify_id: Option<(ECP5IDCODE, usize)>,
    /// Mode and offset of SPI_MODE command in bitstream.
    spi_mode: Option<(u8, usize)>,
    /// 8-byte compression lookup dictionary from bitstream.
    comp_dict: Option<[u8; 8]>,
    /// Locations and included bytes of all CRCs in bitstream.
    stored_crcs: Vec<StoredCrc>,
}

/// Commands found in the bitstream.
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[allow(non_camel_case_types)]
#[repr(u8)]
enum BitstreamCommandId {
    DUMMY               = 0xFF,
    VERIFY_ID           = 0xE2,
    SPI_MODE            = 0x79,
    JUMP                = 0x7E,
    LSC_RESET_CRC       = 0x3B,
    LSC_WRITE_COMP_DIC  = 0x02,
    LSC_PROG_CNTRL0     = 0x22,
    LSC_INIT_ADDRESS    = 0x46,
    LSC_WRITE_ADDRESS   = 0xB4,
    ISC_PROGRAM_SECURITY= 0xCE,
    ISC_PROGRAM_USERCODE= 0xC2,
    ISC_PROGRAM_DONE    = 0x5E,
    LSC_PROG_INCR_RTI   = 0x82,
    LSC_PROG_INCR_CMP   = 0xB8,
    LSC_PROG_SED_CRC    = 0xA2,
    LSC_EBR_ADDRESS     = 0xF6,
    LSC_EBR_WRITE       = 0xB2,
}

/// CRC stored in bitstream.
#[derive(Clone, Debug, PartialEq, Eq)]
struct StoredCrc {
    /// Offset of first byte covered by this CRC.
    start: usize,

    /// Offset of first byte of CRC value stored in bitstream.
    /// All bytes between start..pos are included in CRC except those excluded below.
    pos: usize,

    /// Indices of all excluded bytes, which are any bytes that form a DUMMY command.
    excluded: HashSet<usize>,
}

impl Bitstream {
    /// Open a bitstream from the provided path.
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Self> {
        let mut file = File::open(path)?;
        Self::from_file(&mut file)
    }

    /// Open a bitstream from the provided open `File`.
    pub fn from_file(file: &mut File) -> Result<Self> {
        let mut data = if let Ok(metadata) = file.metadata() {
            Vec::with_capacity(metadata.len() as usize)
        } else {
            Vec::new()
        };
        file.read_to_end(&mut data)?;
        Ok(Self::new(data))
    }

    /// Load a bitstream from the provided raw bitstream data.
    pub fn from_data(data: &[u8]) -> Self {
        Self::new(data.to_owned())
    }

    /// Load a bitstream directly from a `Vec<u8>`.
    pub fn new(data: Vec<u8>) -> Self {
        let meta = match BitstreamMeta::parse(&data[..]) {
            Ok(meta) => Some(meta),
            Err(e)   => {
                log::warn!("Failed to parse bitstream: {e}.");
                log::warn!("Proceeding without IDCODE/SPIMODE checks.");
                None
            },
        };
        Self { data, meta }
    }

    /// Get the underlying bitstream data.
    pub fn data(&self) -> &[u8] {
        &self.data[..]
    }

    /// Check the provided IDCODE matches the IDCODE check in the bitstream.
    ///
    /// If the bitstream does not contain an IDCODE check, no action is taken.
    ///
    /// If the bitstream contains a compatible but different IDCODE, and `fix`
    /// is true, a warning is logged and the bitstream is patched to `idcode`.
    ///
    /// If the bitstream contains an incompatible IDCODE, an error is logged
    /// and this method returns an error.
    ///
    /// See [`ECP5IDCODE::compatible`].
    pub fn check_and_fix_idcode(&mut self, idcode: ECP5IDCODE, fix: bool) -> Result<()> {
        let meta = match &mut self.meta {
            Some(meta) => meta,
            None => {
                log::debug!("Skipping check_and_fix_idcode as no metadata parsed.");
                return Ok(());
            }
        };

        let verify_id = match meta.verify_id {
            Some(verify_id) => verify_id,
            None => {
                log::debug!("Skipping check_and_fix_idcode as no VERIFY_ID command found.");
                return Ok(());
            }
        };

        log::info!(
            "Checking bitstream IDCODE 0x{:08X} against JTAG IDCODE 0x{:08X}",
            verify_id.0 as u32, idcode as u32
        );

        if verify_id.0 == idcode {
            log::debug!("IDCODEs match exactly, no action required.");
            Ok(())
        } else if verify_id.0.compatible(idcode) {
            log::warn!(
                "Bitstream IDCODE 0x{:08X} ({}) is compatible with JTAG IDCODE 0x{:08X} ({}).",
                verify_id.0 as u32, verify_id.0.name(), idcode as u32, idcode.name(),
            );
            if fix {
                log::warn!("Patching programmed bitstream to match JTAG IDCODE. \
                            Use --no-fix-idcode to disable patching.");
                for (idx, byte) in (idcode as u32).to_be_bytes().iter().enumerate() {
                    self.data[verify_id.1 + idx] = *byte;
                }
                self.fix_crc(verify_id.1);
                Ok(())
            } else {
                log::warn!("Not patching because --no-fix-idcode was set.");
                Err(Error::IncompatibleIdcode {
                    bitstream: verify_id.0 as u32,
                    jtag: idcode as u32
                })
            }
        } else {
            log::error!(
                "Bitstream IDCODE 0x{:08X} ({}) is not compatible with JTAG IDCODE 0x{:08X} ({}).",
                verify_id.0 as u32, verify_id.0.name(), idcode as u32, idcode.name(),
            );
            Err(Error::IncompatibleIdcode {
                bitstream: verify_id.0 as u32,
                jtag: idcode as u32
            })
        }
    }

    /// Replace the VERIFY_ID command, if any, with NOOPs.
    ///
    /// If the bitstream does not contain an IDCODE check, no action is taken.
    pub fn remove_idcode(&mut self) -> Result<()> {
        let meta = match &mut self.meta {
            Some(meta) => meta,
            None => {
                log::error!("Cannot remove VERIFY_IDCODE as no metadata was parsed.");
                return Err(Error::RemoveIdcodeNoMetadata);
            }
        };

        let verify_id = match meta.verify_id {
            Some(verify_id) => verify_id,
            None => {
                log::debug!("Skipping remove_verify_id as no VERIFY_ID command found.");
                return Ok(());
            }
        };

        let start = verify_id.1 - 4;
        let end = verify_id.1 + 4;
        log::info!("Replacing VERIFY_ID command at bytes {start}..{end} with NOOP");
        for x in &mut self.data[start..end] {
            *x = 0xFF;
        }
        self.fix_crc_exclude(start..end);
        Ok(())
    }

    /// Update whichever CRC is affected by changes to the provided `offset`.
    fn fix_crc(&mut self, offset: usize) {
        log::debug!("Fixing CRC affected by change to offset {offset}");
    }

    /// Update whichever CRC is affected by excluding the provided offsets.
    fn fix_crc_exclude(&mut self, exclude: std::ops::Range<usize>) {
        log::debug!("Fixing CRC affected by excluding {exclude:?}");
    }
}

impl StoredCrc {
    fn new() -> Self {
        Self { start: 0, pos: 0, excluded: HashSet::new() }
    }

    /// Reset to the provided start position, clearing the list of excluded indices.
    fn start(&mut self, start: usize) {
        self.start = start;
        self.pos = start;
        self.excluded.clear();
    }

    /// Mark `idx` as excluded.
    fn exclude(&mut self, idx: usize) {
        self.excluded.insert(idx);
    }

    /// Save `pos` as the index of the first byte of the stored CRC, which
    /// is also one past the last byte included in the CRC computation.
    ///
    /// Use `compute()` to check the actual CRC over those bytes.
    fn finish(&mut self, pos: usize) {
        self.pos = pos;
    }

    /// Compute the CRC of the indices covered by this StoredCrc,
    /// using the data provided in `data`.
    ///
    /// Does not consider `self.crc`.
    fn compute(&self, data: &[u8]) -> u16 {
        let mut crc = 0;
        for (idx, byte) in data[self.start..self.pos].iter().enumerate() {
            if self.excluded.contains(&(idx + self.start)) {
                continue;
            }
            crc ^= (*byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x8005;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
}
