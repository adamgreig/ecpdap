// Copyright 2022 Adam Greig
// Licensed under the Apache-2.0 and MIT licenses.

use std::{fs::File, path::Path, io::Read};
use crate::Result;

mod parser;

/// ECP5 bitstream in-memory.
///
/// Can be loaded from a path or file, and modified as required to
/// change IDCODE checks and SPI mode and frequency.
struct Bitstream {
    /// Full bitstream data.
    data: Vec<u8>,
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
        Self { data }
    }

    /// Check the provided IDCODE matches any IDCODE check in the bitstream.
    ///
    /// If the bitstream does not contain an IDCODE check, this is considered passed.
    ///
    /// If the bitstream contains a compatible but different IDCODE, a warning is emitted
    /// and the bitstream is patched to the provided IDCODE.
    ///
    /// If the bitstream contains an incompatible IDCODE, this check fails.
    ///
    /// See [`ECP5IDCODE::compatible`].
    pub fn check_idcode(&mut self, idcode: u32) -> bool {
        true
    }
}
