//! This module contains convenience functions for manipulating Vec<bool> and &[bool].

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Not enough bits to extract required data.")]
    NotEnoughBits,
    #[error("Unknown word size.")]
    InvalidWordSize,
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

/// Convert a slice of bits to a Vec of u8s, least-significant-bit first.
///
/// If `bits` is not a multiple of 8, the final byte has the higher-order
/// bits set to 0.
pub fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity((bits.len() + 7) / 8);
    for chunk in bits.chunks(8) {
        let mut byte = 0u8;
        for (idx, bit) in chunk.iter().enumerate() {
            byte |= (*bit as u8) << idx;
        }
        bytes.push(byte);
    }
    bytes
}

/// Convert a slice of u8 to a Vec of bool, least-significant-bit first.
/// Reads exactly `n` bits; returns an error if n>bytes.len()*8.
pub fn bytes_to_bits(bytes: &[u8], mut n: usize) -> Result<Vec<bool>> {
    if n == 0 {
        return Ok(Vec::new());
    }

    let mut bits = Vec::with_capacity(n);
    for byte in bytes {
        for i in 0..8 {
            bits.push((byte >> i) & 1 == 1);
            n -= 1;

            if n == 0 {
                return Ok(bits);
            }
        }
    }
    Err(Error::NotEnoughBits)
}

/// Convert a single byte to a Vec of bool.
pub fn byte_to_bits(byte: u8) -> Vec<bool> {
    match bytes_to_bits(&[byte], 8) {
        Ok(bits) => bits,
        _ => unreachable!(),
    }
}

/// Extracts the next n bits as a u64, least significant bit first,
/// with any higher order bits set to 0. Returns the extracted word
/// and any remaining bits.
///
/// Returns an error if bits.len() < n or if n > 64.
pub fn drain_word(bits: &[bool], n: usize) -> Result<(u64, &[bool])> {
    if bits.len() < n {
        Err(Error::NotEnoughBits)
    } else if n > 64 {
        Err(Error::InvalidWordSize)
    } else {
        let mut word = 0u64;
        for (idx, bit) in bits[..n].iter().enumerate() {
            word |= (*bit as u64) << idx;
        }
        Ok((word, &bits[n..]))
    }
}

/// Extract the next bit as a u8.
pub fn drain_bit(bits: &[bool]) -> Result<(u8, &[bool])> {
    let (word, bits) = drain_word(bits, 1)?;
    Ok((word as u8, bits))
}

/// Extract the next 8 bits as a u8.
pub fn drain_u8(bits: &[bool]) -> Result<(u8, &[bool])> {
    let (word, bits) = drain_word(bits, 8)?;
    Ok((word as u8, bits))
}

/// Extract the next 16 bits as a u16.
pub fn drain_u16(bits: &[bool]) -> Result<(u16, &[bool])> {
    let (word, bits) = drain_word(bits, 16)?;
    Ok((word as u16, bits))
}

/// Extract the next 32 bits as a u32.
pub fn drain_u32(bits: &[bool]) -> Result<(u32, &[bool])> {
    let (word, bits) = drain_word(bits, 32)?;
    Ok((word as u32, bits))
}

/// Append n bits from a u64 to a Vec<bool>, least significant bit first.
pub fn append_word(bits: &mut Vec<bool>, word: u64, n: usize) {
    for idx in 0..n {
        let bit = ((word >> idx) & 1) == 1;
        bits.push(bit);
    }
}

/// Append a single bit from a u8 (0 or 1) to a Vec<bool>.
pub fn append_bit(bits: &mut Vec<bool>, bit: u8) {
    append_word(bits, bit as u64, 1)
}

/// Append a u8 to a Vec<bool>, least significant bit first.
pub fn append_u8(bits: &mut Vec<bool>, word: u8) {
    append_word(bits, word as u64, 8)
}

/// Append a u16 to a Vec<bool>, least significant bit first.
pub fn append_u16(bits: &mut Vec<bool>, word: u16) {
    append_word(bits, word as u64, 16)
}

/// Append a u32 to a Vec<bool>, least significant bit first.
pub fn append_u32(bits: &mut Vec<bool>, word: u32) {
    append_word(bits, word as u64, 32)
}

/// Convenience macro for creating a &[bool] from &[u8] of 0/1.
macro_rules! bv {
    ($($x:expr),*) => {
        &[ $(($x != 0),)* ] as &[bool]
    }
}

#[test]
fn test_bits_to_bytes() {
    assert_eq!(bits_to_bytes(&[]),                                      vec![]);
    assert_eq!(bits_to_bytes(bv![1, 1, 1, 0]),                          vec![0x07]);
    assert_eq!(bits_to_bytes(bv![1, 1, 1, 0, 0, 1, 0, 0]),              vec![0x27]);
    assert_eq!(bits_to_bytes(bv![1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1]),  vec![0x27, 0x08]);
}

#[test]
fn test_bytes_to_bits() {
    assert_eq!(bytes_to_bits(&[0xFF], 1).unwrap(), bv![1]);
    assert_eq!(bytes_to_bits(&[0xFF], 8).unwrap(), bv![1, 1, 1, 1, 1, 1, 1, 1]);
    assert_eq!(bytes_to_bits(&[0xFF, 0x01], 10).unwrap(), bv![1, 1, 1, 1, 1, 1, 1, 1, 1, 0]);
    assert!(bytes_to_bits(&[0xFF], 9).is_err());
}

#[test]
fn test_drain_word() {
    assert_eq!(drain_word(bv![1, 0   ], 2).unwrap(), (0b01, bv![]));
    assert_eq!(drain_word(bv![1, 0, 1], 2).unwrap(), (0b01, bv![1]));
    assert!(drain_word(bv![1, 0, 1], 4).is_err());
}

#[test]
fn test_drain_bit() {
    assert_eq!(drain_bit(bv![1   ]).unwrap(), (1,  bv![]));
    assert_eq!(drain_bit(bv![0, 1]).unwrap(), (0, bv![1]));
}

#[test]
fn test_drain_u8() {
    assert_eq!(drain_u8(bv![1, 0, 0, 0, 1, 0, 1, 0      ]).unwrap(), (0x51, bv![]));
    assert_eq!(drain_u8(bv![1, 0, 0, 0, 1, 0, 1, 0, 1, 1]).unwrap(), (0x51, bv![1, 1]));
}

#[test]
fn test_drain_u16() {
    assert_eq!(drain_u16(bv![1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0]).unwrap(),
               (0x4351, bv![]));
    assert_eq!(drain_u16(bv![1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0,
                             0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1]).unwrap(),
               (0x4351,  bv![0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1]));
}

#[test]
fn test_drain_u32() {
    assert_eq!(drain_u32(bv![1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0,
                             0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1]).unwrap(),
               (0xBCAE4351,  bv![]));
}

#[test]
fn test_append_word() {
    let mut bits = bv![1, 1, 1, 1].to_vec();
    append_word(&mut bits, 0b1101, 4);
    assert_eq!(&bits[..], bv![1, 1, 1, 1, 1, 0, 1, 1]);
}

#[test]
fn test_append_bit() {
    let mut bits = bv![1, 1, 1, 1].to_vec();
    append_bit(&mut bits, 0);
    assert_eq!(&bits[..], bv![1, 1, 1, 1, 0]);
}

#[test]
fn test_append_u8() {
    let mut bits = bv![1, 1, 1, 1].to_vec();
    append_u8(&mut bits, 0x81);
    assert_eq!(&bits[..], bv![1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1]);
}

#[test]
fn test_append_u16() {
    let mut bits = bv![1, 1, 1, 1].to_vec();
    append_u16(&mut bits, 0x55AA);
    assert_eq!(&bits[..], bv![1, 1, 1, 1,
                              0, 1, 0, 1, 0, 1, 0, 1,
                              1, 0, 1, 0, 1, 0, 1, 0]);
}

#[test]
fn test_append_u32() {
    let mut bits = bv![1, 1, 1, 1].to_vec();
    append_u32(&mut bits, 0x76543210);
    assert_eq!(&bits[..], bv![1, 1, 1, 1,
                              0, 0, 0, 0, 1, 0, 0, 0,
                              0, 1, 0, 0, 1, 1, 0, 0,
                              0, 0, 1, 0, 1, 0, 1, 0,
                              0, 1, 1, 0, 1, 1, 1, 0]);
}
