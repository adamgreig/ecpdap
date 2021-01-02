//! The DAP module implements the CMSIS-DAP commands relevant to JTAG programming,
//! formatting them into packets which can be exchanged with the Probe module to
//! control the CMSIS-DAP probe.

use std::{time::Duration, thread};
use thiserror::Error;
use num_enum::IntoPrimitive;
use crate::probe::{Probe, Error as ProbeError};

#[derive(Error, Debug)]
pub enum Error {
    #[error("Probe error")]
    Probe(#[from] ProbeError),
    #[error("Probe does not support JTAG mode.")]
    NoJTAG,
    #[error("Invalid response from probe.")]
    InvalidResponse,
    #[error("Probe reported error during connect.")]
    Connection,
    #[error("Probe reported error setting clock frequency.")]
    Clock,
    #[error("Probe reported error while running JTAG sequence.")]
    JTAG,
    #[error("Internal error: JTAG request too long.")]
    JTAGTooLong,
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

pub struct DAP {
    probe: Probe,
    packet_size: usize,
}

impl DAP {
    pub fn new(probe: Probe) -> Result<DAP> {
        // Drain any pending data in the probe's USB buffer, to
        // prevent our first reads being corrupted by unexpected
        // data.
        probe.drain()?;

        let mut dap = DAP { probe, packet_size: 0 };

        // Query the packet size over CMSIS-DAP, then use it
        // to update the underlying probe packet size for
        // CMSIS-DAPv1 devices.
        dap.packet_size = dap.get_packet_size()?;
        dap.probe.set_packet_size(dap.packet_size);

        if !dap.has_jtag()? {
            return Err(Error::NoJTAG);
        }

        dap.connect()?;
        dap.set_led(true)?;

        Ok(dap)
    }

    pub fn packet_size(&self) -> usize {
        self.packet_size
    }

    fn has_jtag(&self) -> Result<bool> {
        log::debug!("Checking probe for JTAG support");
        let request = Request {
            command: Command::DAP_Info, data: vec![DAPInfoID::Capabilities.into()]
        };
        let response = self.execute(request)?;
        match response.get(1) {
            Some(1) | Some(2) => Ok((response[2] & 0b10) == 0b10),
            _ => Err(Error::InvalidResponse),
        }
    }

    fn get_packet_size(&self) -> Result<usize> {
        log::debug!("Requesting maximum packet size");
        let request = Request {
            command: Command::DAP_Info, data: vec![DAPInfoID::MaxPacketSize.into()]
        };
        let response = self.execute(request)?;
        match response.get(1) {
            Some(2) => {
                let size = u16::from_le_bytes([response[2], response[3]]);
                log::trace!("Got packet size {} bytes", size);
                Ok(size as usize)
            },
            _ => Err(Error::InvalidResponse),
        }
    }

    fn connect(&self) -> Result<()> {
        log::debug!("Connecting to target");
        let request = Request {
            command: Command::DAP_Connect, data: vec![ConnectPort::JTAG.into()]
        };
        let response = self.execute(request)?;
        match response.get(1) {
            Some(result) if *result == ConnectPortResponse::JTAG.into() => Ok(()),
            _ => Err(Error::Connection),
        }
    }

    fn disconnect(&self) -> Result<()> {
        log::debug!("Disconnecting from target");
        let request = Request { command: Command::DAP_Disconnect, data: vec![] };
        let response = self.execute(request)?;
        match response.get(1) {
            Some(status) if *status == ResponseStatus::DAP_OK.into() => Ok(()),
            _ => Err(Error::Connection),
        }
    }

    pub fn set_led(&self, state: bool) -> Result<()> {
        log::trace!("Setting probe LED to {}", state);
        let request = Request {
            command: Command::DAP_HostStatus,
            data: vec![HostStatusType::Connect.into(), state as u8]
        };
        self.execute(request)?;
        Ok(())
    }

    fn set_nrst(&self, state: bool) -> Result<()> {
        log::trace!("Setting NRST to {}", state);
        let state = (state as u8) << 7;
        let select = 1 << 7;
        let request = Request { command: Command::DAP_SWJ_Pins,
                                data: vec![state, select, 0, 0, 0, 0] };
        self.execute(request)?;
        Ok(())
    }

    pub fn pulse_nrst(&self, duration: Duration) -> Result<()> {
        log::debug!("Pulsing nRST");
        self.set_nrst(false)?;
        thread::sleep(duration);
        self.set_nrst(true)?;
        Ok(())
    }

    pub fn set_clock(&self, freq: u32) -> Result<()> {
        log::debug!("Setting clock to {}Hz", freq);
        let request = Request {
            command: Command::DAP_SWJ_Clock, data: freq.to_le_bytes().to_vec()
        };
        let response = self.execute(request)?;
        match response.get(1) {
            Some(status) if *status == ResponseStatus::DAP_OK.into() => Ok(()),
            _ => Err(Error::Clock),
        }
    }

    pub fn jtag_sequence(&self, data: &[u8]) -> Result<Vec<u8>> {
        log::trace!("Running JTAG sequence");
        if data.len() > self.packet_size - 1 {
            log::error!("Attempted JTAG sequence of length {} which exceeds packet size {}",
                        data.len(), self.packet_size);
            return Err(Error::JTAGTooLong);
        }
        let request = Request { command: Command::DAP_JTAG_Sequence, data: data.to_vec() };
        let response = self.execute(request)?;
        match response.get(1) {
            Some(status) if *status == ResponseStatus::DAP_OK.into() => Ok(response[2..].to_vec()),
            _ => Err(Error::JTAG),
        }
    }

    fn execute(&self, request: Request) -> Result<Vec<u8>> {
        let request_command = request.command.into();
        self.probe.write(&request.to_bytes())?;
        let response = self.probe.read()?;
        match response.get(0) {
            Some(command) if *command == request_command => Ok(response),
            _ => Err(Error::InvalidResponse),
        }
    }
}

impl std::ops::Drop for DAP {
    fn drop(&mut self) {
        log::debug!("DAP dropped, disconnecting");
        self.disconnect().ok();
        self.set_led(false).ok();
    }
}

#[derive(Copy, Clone, IntoPrimitive)]
#[allow(non_camel_case_types)]
#[repr(u8)]
enum Command {
    DAP_Info            = 0x00,
    DAP_HostStatus      = 0x01,
    DAP_Connect         = 0x02,
    DAP_Disconnect      = 0x03,
    DAP_SWJ_Pins        = 0x10,
    DAP_SWJ_Clock       = 0x11,
    DAP_JTAG_Sequence   = 0x14,
}

#[derive(Copy, Clone, IntoPrimitive)]
#[allow(non_camel_case_types)]
#[repr(u8)]
enum ResponseStatus {
    DAP_OK              = 0x00,
}

#[derive(Copy, Clone, IntoPrimitive)]
#[allow(non_camel_case_types)]
#[repr(u8)]
enum DAPInfoID {
    Capabilities        = 0xF0,
    MaxPacketSize       = 0xFF,
}

#[derive(Copy, Clone, IntoPrimitive)]
#[repr(u8)]
enum HostStatusType {
    Connect             = 0,
}

#[derive(Copy, Clone, IntoPrimitive)]
#[repr(u8)]
enum ConnectPort {
    JTAG                = 2,
}

#[derive(Copy, Clone, IntoPrimitive)]
#[repr(u8)]
enum ConnectPortResponse {
    JTAG                = 2,
}

struct Request {
    command: Command,
    data: Vec<u8>,
}

impl Request {
    fn to_bytes(&self) -> Vec<u8> {
        // Insert command ID as first byte.
        let mut bytes = vec![self.command.into()];
        bytes.extend_from_slice(&self.data[..]);
        bytes
    }
}
