// Copyright 2022 Adam Greig
// Licensed under the Apache-2.0 and MIT licenses.

use std::time::Instant;
use std::convert::{TryFrom, TryInto};
use crate::ECP5IDCODE;
use super::{BitstreamCommandId, BitstreamMeta, StoredCrc};

#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub(super) enum ParseError {
    #[error("Exhausted: Ran out of bitstream data earlier than expected")]
    Exhausted,
    #[error("Did not find the preamble marker")]
    NoPreamble,
    #[error("Invalid CRC inside bitstream: computed {computed:04x}, read {read:04x}")]
    InvalidCrc { computed: u16, read: u16 },
    #[error("Unexpected data: expected {expected:?}, read {read:?}")]
    BadCheck { expected: Vec<u8>, read: Vec<u8> },
    #[error("Did not find a VERIFY_ID command, so cannot decode the bitstream")]
    NoVerifyId,
    #[error("Found an unknown IDCODE 0x{0:08X}")]
    UnknownIdcode(u32),
    #[error("Did not find a compression dictionary but the configuration data is compressed")]
    NoCompDict,
    #[error("Found unknown command {0:02X}")]
    UnknownCommand(u8),
}

/// Read from a slice, keeping track of current position.
///
/// Provides convenient methods like peek, skip, and fetching constant-sized arrays.
struct Reader<'a> {
    data: &'a [u8],
    offset: usize,
}

impl<'a> Reader<'a> {
    fn new(data: &'a [u8]) -> Reader<'a> {
        Self { data, offset: 0 }
    }

    /// Return original input.
    fn input(&self) -> &[u8] {
        self.data
    }

    /// Fetch current offset, which is the position of the next byte to be read.
    fn offset(&self) -> usize {
        self.offset
    }

    /// Fetch next byte to be read, incrementing internal offset.
    fn next(&mut self) -> Result<u8, ParseError> {
        let x = self.data.get(self.offset).copied().ok_or(ParseError::Exhausted);
        self.offset += 1;
        x
    }

    /// Look at next n bytes without consuming them.
    fn peek(&self, n: usize) -> Result<&[u8], ParseError> {
        self.data.get(self.offset..(self.offset + n)).ok_or(ParseError::Exhausted)
    }

    /// Skip `n` bytes.
    fn skip(&mut self, n: usize) -> Result<(), ParseError> {
        self.offset += n;
        if self.offset <= self.data.len() {
            Ok(())
        } else {
            Err(ParseError::Exhausted)
        }
    }

    /// Skip until specific byte will be returned next.
    ///
    /// Make sure to advance the offset somehow before calling again or you'll get stuck.
    fn skip_to(&mut self, x: u8) -> Result<(), ParseError> {
        while self.offset < self.data.len() && self.data[self.offset] != x {
            self.offset += 1;
        }

        if self.eof() {
            Err(ParseError::Exhausted)
        } else {
            Ok(())
        }
    }

    /// Verify the next bytes match the provided slice and skip them.
    fn check_skip(&mut self, x: &[u8]) -> Result<(), ParseError> {
        let data = self.peek(x.len())?;
        if data != x {
            Err(ParseError::BadCheck { expected: x.to_vec(), read: data.to_vec() })
        } else {
            self.skip(x.len())?;
            Ok(())
        }
    }

    /// Take constant `n` bytes, returning an array reference.
    fn take<const N: usize>(&mut self) -> Result<&[u8; N], ParseError> {
        self.offset += N;
        self.data.get(self.offset - N..self.offset)
                 .map(|d| d.try_into().unwrap())
                 .ok_or(ParseError::Exhausted)
    }

    /// Read compressed bytes using ECP5 bitstream compression.
    ///
    /// Returns exactly `n` decompressed bytes or an error.
    #[allow(unused)]
    fn read_compressed(&mut self, n: usize, dict: &[u8; 8]) -> Result<Vec<u8>, ParseError> {
        let mut out = Vec::with_capacity(n);
        let mut bits = BitReader::new();

        for _ in 0..n {
            if bits.bit(self)? == 0 {
                // Code 0 represents a whole 0 byte of input.
                out.push(0);
            } else if bits.bit(self)? == 0 {
                if bits.bit(self)? == 0 {
                    let pos = bits.bits(self, 3)? as u8;
                    // Code 100xxx is a single set-bit position.
                    out.push(1 << pos);
                } else {
                    // Code 101xxx is a stored byte.
                    let idx = bits.bits(self, 3)? as usize;
                    out.push(dict[idx]);
                }
            } else {
                // Code 11xxxxxxxx is a literal byte.
                let byte = bits.bits(self, 8)?;
                out.push(byte as u8);
            }
        }

        Ok(out)
    }

    /// Skip compressed bytes using ECP5 bitstream compression.
    ///
    /// Skips exactly `n` decompressed bytes.
    fn skip_compressed(&mut self, n: usize) -> Result<(), ParseError> {
        let mut bits = BitReader::new();
        for _ in 0..n {
            // 0 bits map to a whole byte, so only read extra bits on 1.
            if bits.bit(self)? == 1 {
                if bits.bit(self)? == 0 {
                    // 10yxxx always requires 4 additional bits.
                    bits.bits(self, 4)?;
                } else {
                    // 11xxxxxxxx always requires 8 additional bits.
                    bits.bits(self, 8)?;
                }
            }
        }
        Ok(())
    }

    /// Check if internal offset is now at the end of the input data.
    fn eof(&self) -> bool {
        self.offset >= self.data.len()
    }
}

struct BitReader {
    bits: u32,
    bits_left: usize,
}

impl BitReader {
    fn new() -> Self {
        BitReader { bits: 0, bits_left: 0 }
    }

    fn bit(&mut self, rd: &mut Reader) -> Result<u8, ParseError> {
        if self.bits_left == 0 {
            self.bits = rd.next()? as u32;
            self.bits_left += 8;
        }
        self.bits_left -= 1;
        Ok((self.bits >> (self.bits_left)) as u8 & 1)
    }

    fn bits(&mut self, rd: &mut Reader, n: usize) -> Result<u32, ParseError> {
        assert!(n <= 32);
        while self.bits_left < n {
            self.bits = (self.bits << 8) | (rd.next()? as u32);
            self.bits_left += 8;
        }
        self.bits_left -= n;
        Ok((self.bits >> (self.bits_left)) & ((1 << n) - 1))
    }
}

impl BitstreamMeta {
    /// Parse the provided bitstream, extracting the relevant metadata.
    pub fn parse(input: &[u8]) -> Result<Self, ParseError> {
        log::info!("Parsing bitstream for metadata");
        let t0 = Instant::now();
        let mut rd = Reader::new(input);

        // Detect comment block at start and skip it.
        if rd.peek(2)? == [0xFF, 0x00] {
            while rd.take()? != &[0x00, 0xFF] {
                rd.skip_to(0x00)?;
            }
        }

        // Skip any further dummy bytes until start of preamble.
        match rd.skip_to(0xBD) {
            Ok(_) => (),
            Err(ParseError::Exhausted) => Err(ParseError::NoPreamble)?,
            Err(e) => Err(e)?,
        }

        // Check preamble.
        if rd.take()? != &[0xBD, 0xB3] {
            return Err(ParseError::NoPreamble);
        }

        let mut meta = BitstreamMeta {
            verify_id: None,
            spi_mode: None,
            stored_crcs: Vec::new(),
            comp_dict: None,
            jump: None,
        };
        let mut current_crc = StoredCrc::new();

        // Process commands.
        loop {
            match BitstreamCommandId::try_from(rd.next()?) {
                Ok(BitstreamCommandId::DUMMY) => {
                    // Don't include DUMMY bytes in CRC processing.
                    current_crc.exclude(rd.offset() - 1);
                },
                Ok(BitstreamCommandId::VERIFY_ID) => {
                    rd.check_skip(&[0, 0, 0])?;
                    let offset = rd.offset();
                    let idcode = u32::from_be_bytes(*rd.take()?);
                    let idcode = ECP5IDCODE::try_from_u32(idcode)
                                             .ok_or(ParseError::UnknownIdcode(idcode))?;
                    log::trace!("VERIFY_ID command for IDCODE 0x{:08X} ({})",
                                idcode as u32, idcode.name());
                    meta.verify_id = Some((idcode, offset));
                },
                Ok(BitstreamCommandId::SPI_MODE) => {
                    let offset = rd.offset();
                    let mode = rd.next()?;
                    log::trace!("SPI_MODE command for mode {mode:02X}");
                    rd.check_skip(&[0, 0])?;
                    meta.spi_mode = Some((mode, offset));
                },
                Ok(BitstreamCommandId::LSC_RESET_CRC) => {
                    log::trace!("LSC_RESET_CRC command");
                    rd.check_skip(&[0, 0, 0])?;
                    current_crc.start(rd.offset());
                },
                Ok(BitstreamCommandId::LSC_WRITE_COMP_DIC) => {
                    log::trace!("LSC_WRITE_COMP_DIC command");
                    let crc_check = rd.next()?;
                    rd.check_skip(&[0, 0])?;
                    meta.comp_dict = Some(u64::from_be_bytes(*rd.take()?).to_le_bytes());
                    meta.maybe_check_crc(crc_check, &mut rd, &mut current_crc)?;
                },
                Ok(BitstreamCommandId::ISC_PROGRAM_USERCODE) => {
                    log::trace!("ISC_PROGRAM_USERCODE command");
                    let crc_check = rd.next()?;
                    rd.check_skip(&[0, 0])?;
                    rd.skip(4)?;
                    meta.maybe_check_crc(crc_check, &mut rd, &mut current_crc)?;
                },
                Ok(BitstreamCommandId::JUMP) => {
                    rd.check_skip(&[0, 0, 0])?;
                    let data = u32::from_be_bytes(*rd.take()?);
                    let spimode = (data >> 24) as u8;
                    let target = data & 0x00FF_FFFF;
                    meta.jump = Some((spimode, target));
                    log::trace!("JUMP command: SPI mode 0x{spimode:02X}, target 0x{target:06X}");
                },
                Ok(BitstreamCommandId::LSC_PROG_CNTRL0)
                    | Ok(BitstreamCommandId::LSC_WRITE_ADDRESS)
                    | Ok(BitstreamCommandId::ISC_PROGRAM_SECURITY)
                    | Ok(BitstreamCommandId::LSC_EBR_ADDRESS)
                => {
                    rd.check_skip(&[0, 0, 0])?;
                    rd.skip(4)?;
                },
                Ok(BitstreamCommandId::LSC_INIT_ADDRESS)
                    | Ok(BitstreamCommandId::ISC_PROGRAM_DONE)
                => {
                    rd.check_skip(&[0, 0, 0])?;
                },
                Ok(BitstreamCommandId::LSC_PROG_SED_CRC) => {
                    rd.check_skip(&[0, 0, 0])?;
                    rd.skip(8)?;
                },
                Ok(BitstreamCommandId::LSC_EBR_WRITE) => {
                    log::trace!("LSC_EBR_WRITE command");
                    let crc_check = rd.next()?;
                    let num_frames = u16::from_be_bytes(*rd.take()?);
                    for _ in 0..num_frames {
                        rd.skip(9)?;
                    }
                    meta.maybe_check_crc(crc_check, &mut rd, &mut current_crc)?;
                },
                Ok(BitstreamCommandId::LSC_PROG_INCR_RTI) => {
                    log::trace!("LSC_PROG_INCR_RTI command");
                    meta.read_frame(&mut rd, false, &mut current_crc)?;
                },
                Ok(BitstreamCommandId::LSC_PROG_INCR_CMP) => {
                    log::trace!("LSC_PROG_INCR_CMP command");
                    meta.read_frame(&mut rd, true, &mut current_crc)?;
                },
                Err(e) => {
                    return Err(ParseError::UnknownCommand(e.number));
                },
            }

            // Stop looking for commands if we reached the end of the bitstream.
            if rd.eof() {
                let elapsed = t0.elapsed().as_millis();
                log::debug!("Parsed to end of bitstream in {elapsed}ms");
                break;
            }
        }

        Ok(meta)
    }

    /// Verify a trailing CRC16 after a command/frame.
    ///
    /// Validates the CRC and updates `stored_crcs` as appropriate.
    fn check_crc(&mut self, rd: &mut Reader, current_crc: &mut StoredCrc)
        -> Result<(), ParseError>
    {
        let pos = rd.offset();
        let crc = u16::from_be_bytes(*rd.take()?);
        current_crc.finish(pos);
        let computed = current_crc.compute(rd.input());
        if computed != crc {
            return Err(ParseError::InvalidCrc { computed, read: crc });
        }
        self.stored_crcs.push(current_crc.clone());
        current_crc.start(rd.offset());
        Ok(())
    }

    /// Conditionally verify a trailing CRC16 if set.
    fn maybe_check_crc(&mut self, check: u8, rd: &mut Reader, current_crc: &mut StoredCrc)
        -> Result<(), ParseError>
    {
        if check & 0x80 != 0 {
            self.check_crc(rd, current_crc)
        } else {
            Ok(())
        }
    }

    /// Skip configuration frame data, which may be compressed.
    ///
    /// Requires `verify_id` to determine frame size, and if compressed, a `comp_dict`.
    fn read_frame(&mut self, rd: &mut Reader, compressed: bool, current_crc: &mut StoredCrc)
        -> Result<(), ParseError>
    {
        // Check we have an ID and (if required) a compression dictionary.
        let id = self.verify_id.ok_or(ParseError::NoVerifyId)?.0;
        let (mut pad_before, bits_per_frame, mut pad_after) = id.config_bits_per_frame();
        if compressed && self.comp_dict.is_none() {
            return Err(ParseError::NoCompDict);
        }

        // Work out size per frame.
        let params = rd.next()?;
        let check_crc = params & 0x80 != 0;
        let crc_after_each = check_crc && (params & 0x40 == 0);
        let use_dummy_bits = params & 0x20 == 0;
        let use_dummy_bytes = params & 0x10 != 0;
        let mut dummy_bytes = (params & 0x0F) as usize;
        if !use_dummy_bits {
            log::warn!("Frame doesn't use dummy bits, which is unusual");
            pad_before = 0;
            pad_after = 0;
        }
        if !use_dummy_bytes {
            log::warn!("Frame doesn't use dummy bytes, which is unusual");
            dummy_bytes = 0;
        }
        let frame_count = u16::from_be_bytes(*rd.take()?);
        let mut bytes_per_frame = (pad_before + bits_per_frame + pad_after) / 8;
        if compressed {
            bytes_per_frame += 7 - ((bytes_per_frame - 1) % 8);
        }

        // Skip through frame data.
        for i in 0..frame_count {
            if compressed {
                rd.skip_compressed(bytes_per_frame)?;
            } else {
                rd.skip(bytes_per_frame)?;
            }
            if crc_after_each || (check_crc && i == frame_count - 1) {
                self.check_crc(rd, current_crc)?;
            }
            rd.skip(dummy_bytes)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_preamble() {
        assert_eq!(
            BitstreamMeta::parse(&[
                0xFF, 0x00, b'c', b'o', b'm', b'm', b'e', b'n', b't', 0x00, 0xFF,
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xE2, 0x00, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78,
                0xFF, 0xFF, 0xFF, 0xFF, 0x79, 0xAB, 0x00, 0x00,
            ]),
            Err(ParseError::NoPreamble),
        );
    }

    #[test]
    fn test_parse_simple() {
        let mut crc = StoredCrc::new();
        crc.start(20);
        for i in &[20, 21, 22, 23, 32, 33, 34, 35] {
            crc.exclude(*i);
        }
        crc.finish(48);
        assert_eq!(
            BitstreamMeta::parse(&[
                0xFF, 0x00, b'c', b'o', b'm', b'm', 0x00, 0xFF,
                0xFF, 0xFF, 0xBD, 0xB3, 0xFF, 0xFF, 0xFF, 0xFF,
                0x3B, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
                0xE2, 0x00, 0x00, 0x00, 0x41, 0x11, 0x20, 0x43,
                0xFF, 0xFF, 0xFF, 0xFF, 0x79, 0xAB, 0x00, 0x00,
                0xC2, 0x80, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78, 0xF6, 0xA4,
            ]),
            Ok(BitstreamMeta {
                verify_id: Some((ECP5IDCODE::LFE5U_45, 28)),
                spi_mode: Some((0xAB, 37)),
                stored_crcs: vec![ crc ],
                comp_dict: None,
                jump: None,
            }),
        );
    }

    #[test]
    fn test_parse_jump() {
        assert_eq!(
            BitstreamMeta::parse(&[
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xBD, 0xB3, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x7E, 0x00, 0x00, 0x00, 0x03, 0x1C, 0x00, 0x00,
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            ]),
            Ok(BitstreamMeta {
                verify_id: None,
                spi_mode: None,
                stored_crcs: vec![],
                comp_dict: None,
                jump: Some((0x03, 0x1c0000)),
            }),
        );
    }
}
