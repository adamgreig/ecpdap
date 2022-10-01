// Copyright 2022 Adam Greig
// Licensed under the Apache-2.0 and MIT licenses.

use nom::{
    IResult,
    sequence::{preceded, delimited, tuple},
    combinator::map,
    branch::alt,
    multi::many1,
    number::complete::{be_u8, be_u16, be_u32, be_u64},
    bytes::complete::{tag, take, take_until},
};

/// Commands found in the bitstream.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[allow(non_camel_case_types)]
#[repr(u32)]
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

impl BitstreamCommandId {
    /// Returns a parser for this command ID.
    fn tag<'a>(self) -> impl Fn(&'a [u8]) -> IResult<&'a [u8], &'a [u8]> {
        tag([self as u8])
    }

    /// Returns a parser for this command ID followed by three 0s.
    fn tag_zeros<'a>(self) -> impl Fn(&'a [u8]) -> IResult<&'a [u8], &'a [u8]> {
        tag([self as u8, 0, 0, 0])
    }
}

#[derive(Clone, Debug, PartialEq)]
struct ConfigData {
    crc: bool,
    crc_at_end: bool,
    use_dummy_bits: bool,
    use_dummy_bytes: bool,
    num_dummy_bytes: u8,
    num_frames: u16,
    frames: Vec<Vec<u8>>,
}

#[derive(Clone, Debug, PartialEq)]
enum BitstreamCommand {
    Dummy,
    VerifyId(u32),
    SpiMode(u8),
    Jump,
    ResetCrc,
    WriteCompDic([u8; 8]),
    ProgCntrl0(u32),
    InitAddress,
    WriteAddress(u32),
    ProgSecurity(u32),
    ProgUsercode(u32),
    ProgDone,
    ProgIncrRti(ConfigData),
    ProgIncrCmp(ConfigData),
    ProgSedCrc,
    EbrAddress(u32),
    EbrWrite,
}

struct Bitstream {
    comment: Vec<u8>,
    commands: Vec<BitstreamCommand>,
}

fn comment(input: &[u8]) -> IResult<&[u8], &[u8]> {
    delimited(
        tag([0xFF, 0x00]),
        take_until(&[0x00][..]),
        tag([0x00, 0xFF])
    )(input)
}

fn preamble(input: &[u8]) -> IResult<&[u8], ()> {
    map(tag([0xFF, 0xFF, 0xBD, 0xB3]), |_| ())(input)
}

fn dummy(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(tag([0xFF]), |_| BitstreamCommand::Dummy)(input)
}

fn verify_id(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::VERIFY_ID.tag_zeros(), be_u32),
        |idcode: u32| BitstreamCommand::VerifyId(idcode),
    )(input)
}

fn spi_mode(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        delimited(
            BitstreamCommandId::SPI_MODE.tag(),
            be_u8,
            tag([0, 0])
        ),
        |mode: u8| BitstreamCommand::SpiMode(mode),
    )(input)
}

fn jump(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::JUMP.tag_zeros(), be_u32),
        |_| BitstreamCommand::Jump,
    )(input)
}

fn lsc_reset_crc(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        BitstreamCommandId::LSC_RESET_CRC.tag_zeros(),
        |_| BitstreamCommand::ResetCrc
    )(input)
}

fn lsc_write_comp_dic(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::LSC_WRITE_COMP_DIC.tag_zeros(), be_u64),
        |dict| BitstreamCommand::WriteCompDic(dict.to_le_bytes()),
    )(input)
}

fn lsc_prog_cntrl0(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::LSC_PROG_CNTRL0.tag_zeros(), be_u32),
        |c0| BitstreamCommand::ProgCntrl0(c0),
    )(input)
}

fn lsc_init_address(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        BitstreamCommandId::LSC_INIT_ADDRESS.tag_zeros(),
        |_| BitstreamCommand::InitAddress,
    )(input)
}

fn lsc_write_address(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::LSC_WRITE_ADDRESS.tag_zeros(), be_u32),
        |addr| BitstreamCommand::WriteAddress(addr),
    )(input)
}

fn isc_program_security(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::ISC_PROGRAM_SECURITY.tag_zeros(), be_u32),
        |security| BitstreamCommand::ProgSecurity(security),
    )(input)
}

fn isc_program_usercode(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::ISC_PROGRAM_USERCODE.tag_zeros(), be_u32),
        |usercode| BitstreamCommand::ProgUsercode(usercode),
    )(input)
}

fn isc_program_done(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        BitstreamCommandId::ISC_PROGRAM_DONE.tag_zeros(),
        |_| BitstreamCommand::ProgDone,
    )(input)
}

fn lsc_prog_incr_rti(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    todo!()
}

fn lsc_prog_incr_cmp(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    todo!()
}

fn lsc_prog_sed_crc(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    // Format uncertain. Seems to be 12 bytes total, i.e. command, 3 bytes params, 8 bytes data.
    todo!()
}

fn lsc_ebr_address(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    map(
        preceded(BitstreamCommandId::LSC_EBR_ADDRESS.tag_zeros(), be_u32),
        |addr| BitstreamCommand::EbrAddress(addr),
    )(input)
}

fn lsc_ebr_write(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    // command, byte with CRC, two bytes with number of frames
    // each frame is then 9 bytes
    // followed by CRC at end
    todo!()
}

fn bitstream_command(input: &[u8]) -> IResult<&[u8], BitstreamCommand> {
    alt((
        dummy,
        verify_id,
        spi_mode,
        jump,
        lsc_reset_crc,
        lsc_write_comp_dic,
        lsc_prog_cntrl0,
        lsc_init_address,
        lsc_write_address,
        isc_program_security,
        isc_program_usercode,
        isc_program_done,
        lsc_prog_incr_rti,
        lsc_prog_incr_cmp,
        lsc_prog_sed_crc,
        lsc_ebr_address,
        lsc_ebr_write,
    ))(input)
}

fn bitstream(input: &[u8]) -> IResult<&[u8], Bitstream> {
    map(
        tuple((comment, preamble, many1(bitstream_command))),
        |(comment, _, commands)| Bitstream {
            comment: comment.iter().copied().collect(),
            commands
        }
    )(input)
}

// TODO:
// 1. Add tests for remaining command parsers
// 2. Work out how to handle CRCs:
//      * Write CRC calculator
//      * Add CRC bytes to commands that may have them
//      * Ideally make this conditional on the CRC bit being set
//      * Need to compute running CRCs of all data until we get to a CRC/reset
//      * Need to error on bad CRCs
//  3. Implement the ebr_write variable-length business
//  4. Implement the uncompressed data frame handling
//  5. Implement the compressed data frame handling.
//      * Need to somehow access the compression dictionary from earlier
//      * Use nom's bit handling methods
//  6. More tests.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_comment() {
        assert_eq!(comment(b"\xFF\x00This is a comment.\x00\xFFleft"),
                   Ok((&b"left"[..], &b"This is a comment."[..])));
    }

    #[test]
    fn test_preamble() {
        assert_eq!(preamble(b"\xFF\xFF\xBD\xB3left"), Ok((&b"left"[..], ())));
    }

    #[test]
    fn test_dummy() {
        assert_eq!(dummy(b"\xFFleft"), Ok((&b"left"[..], BitstreamCommand::Dummy)));
    }

    #[test]
    fn test_verify_id() {
        assert_eq!(verify_id(b"\xE2\x00\x00\x00\x12\x34\x56\x78left"),
                   Ok((&b"left"[..], BitstreamCommand::VerifyId(0x12345678))));
    }

    #[test]
    fn test_spi_mode() {
        assert_eq!(spi_mode(b"\x79\xAB\x00\x00left"),
                   Ok((&b"left"[..], BitstreamCommand::SpiMode(0xAB))));
    }

    #[test]
    fn test_jump() {
        assert_eq!(jump(b"\x7E\x00\x00\x001234left"),
                   Ok((&b"left"[..], BitstreamCommand::Jump)));
    }

    #[test]
    fn test_lsc_reset_crc() {
        assert_eq!(lsc_reset_crc(b"\x3B\x00\x00\x00left"),
                   Ok((&b"left"[..], BitstreamCommand::ResetCrc)));
    }

    #[test]
    fn test_lsc_write_comp_dic() {
        assert_eq!(lsc_write_comp_dic(b"\x02\x00\x00\x0076543210left"),
                   Ok((&b"left"[..], BitstreamCommand::WriteCompDic(*b"01234567"))));
    }

    #[test]
    fn test_lsc_prog_cntrl0() {
        assert_eq!(lsc_prog_cntrl0(b"\x22\x00\x00\x00\x12\x34\x56\x78left"),
                   Ok((&b"left"[..], BitstreamCommand::ProgCntrl0(0x12345678))));
    }

    #[test]
    fn test_lsc_init_address() {
        assert_eq!(lsc_init_address(b"\x46\x00\x00\x00left"),
                   Ok((&b"left"[..], BitstreamCommand::InitAddress)));
    }

    #[test]
    fn test_lsc_write_address() {
        assert_eq!(lsc_write_address(b"\xB4\x00\x00\x00\x12\x34\x56\x78left"),
                   Ok((&b"left"[..], BitstreamCommand::WriteAddress(0x12345678))));
    }
}