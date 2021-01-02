//! This is the lowest-level module. It is responsible for scanning the USB bus
//! to find a CMSIS-DAP probe, and reading and writing packets to it. Both CMSIS-DAP
//! v1 and v2 probes are supported; v1 probes use `hidapi` to communicate with HID
//! reports, while v2 probes use `rusb` to directly read/write the v2 bulk endpoint.

use std::time::Duration;
use thiserror::Error;
use rusb::{Device, Context, UsbContext};
use hidapi::HidApi;

#[derive(Error, Debug)]
pub enum Error {
    #[error("invalid specifier, use VID:PID or VID:PID:Serial.")]
    InvalidSpecifier,
    #[error("specified probe not found.")]
    NotFound,
    #[error("no CMSIS-DAP probes found.")]
    NoProbesFound,
    #[error("multiple CMSIS-DAP probes found, select a specific probe.")]
    MultipleProbesFound,
    #[error("USB error")]
    USB(#[from] rusb::Error),
    #[error("USB HID error")]
    HID(#[from] hidapi::HidError),
    #[error(transparent)]
    Other(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, Error>;

/// Handle to an open probe, either CMSIS-DAP v1 or v2.
pub enum Probe {
    /// CMSIS-DAP v1 over HID.
    V1 {
        device: hidapi::HidDevice,
        report_size: usize,
    },

    /// CMSIS-DAP v2 over WinUSB/Bulk.
    V2 {
        handle: rusb::DeviceHandle<rusb::Context>,
        out_ep: u8,
        in_ep: u8,
        max_packet_size: usize,
    },
}

impl Probe {
    /// Attempt to open an unspecified connected probe.
    ///
    /// Fails if zero or more than one probe is detected.
    pub fn new() -> Result<Probe> {
        log::debug!("Attempting to open any connected probe");
        let probes = ProbeInfo::list();
        if probes.is_empty() {
            Err(Error::NoProbesFound)
        } else if probes.len() > 1 {
            Err(Error::MultipleProbesFound)
        } else {
            probes[0].open()
        }
    }

    /// Update the packet size.
    ///
    /// The maximum packet size is initially set when the probe is opened,
    /// using the USB descriptor size for CMSIS-DAPv2 devices, but using a
    /// default of 64 bytes for CMSIS-DAPv1 devices, as the HID report size
    /// cannot be queried.
    ///
    /// Once the probe is opened, the maximum packet size can be requested
    /// over CMSIS-DAP and then this method used to update it.
    ///
    /// This method only has an effect for CMSIS-DAPv1 devices, as v2 devices
    /// should always have the correct packet size from the USB descriptor.
    pub fn set_packet_size(&mut self, packet_size: usize) {
        match self {
            Probe::V1 { report_size, .. } => *report_size = packet_size,
            _ => (),
        }
    }

    /// Attempt to open a v2 probe from a specified Device.
    fn from_device(device: Device<Context>) -> Result<Probe> {
        log::trace!("Attempting to open in CMSIS-DAPv2 mode: {:?}", device);
        let timeout = Duration::from_millis(100);
        let mut handle = device.open()?;
        let language = handle.read_languages(timeout)?.get(0).cloned().unwrap();
        let cdesc = device.config_descriptor(0)?;
        for interface in cdesc.interfaces() {
            for idesc in interface.descriptors() {
                // Skip interfaces without CMSIS-DAP in their interface string.
                match handle.read_interface_string(language, &idesc, timeout) {
                    Ok(istr) if !istr.contains("CMSIS-DAP") => continue,
                    Err(_) => continue,
                    Ok(_) => (),
                }

                // Check interface has 2 or 3 endpoints.
                let eps: Vec<_> = idesc.endpoint_descriptors().collect();
                if eps.len() < 2 || eps.len() > 3 {
                    continue;
                }

                // Check first interface is bulk out.
                if eps[0].transfer_type() != rusb::TransferType::Bulk
                    || eps[0].direction() != rusb::Direction::Out
                {
                    continue;
                }

                // Check second interface is bulk in.
                if eps[1].transfer_type() != rusb::TransferType::Bulk
                    || eps[1].direction() != rusb::Direction::In
                {
                    continue;
                }

                // Attempt to claim interface.
                match handle.claim_interface(interface.number()) {
                    Ok(()) => {
                        log::debug!("Successfully opened v2 probe: {:?}", device);
                        let out_ep = eps[0].address();
                        let in_ep = eps[1].address();
                        let max_packet_size = eps[1].max_packet_size() as usize;
                        return Ok(Probe::V2 { handle, out_ep, in_ep, max_packet_size });
                    }
                    Err(_) => continue,
                }
            }
        }
        Err(Error::NotFound)
    }

    /// Attempt to open a v1 probe from a specified vid/pid and optional sn.
    fn from_hid(info: &ProbeInfo) -> Result<Probe> {
        log::trace!("Attempting to open in CMSIS-DAPv1 mode: {}", info);
        let hid_device = match info.sn.clone() {
            Some(sn) => HidApi::new().and_then(|api| api.open_serial(info.vid, info.pid, &sn)),
            None     => HidApi::new().and_then(|api| api.open(info.vid, info.pid)),
        };
        match hid_device {
            Ok(device) => match device.get_product_string() {
                Ok(Some(s)) if s.contains("CMSIS-DAP") => {
                    log::debug!("Successfully opened v1 probe: {:?}", info);
                    // Start with a default of 64 byte packet size, which is
                    // the most common report size for CMSIS-DAPv1 HID devices.
                    Ok(Probe::V1 { device, report_size: 64 })
                },
                _ => Err(Error::NotFound),
            },
            _ => Err(Error::NotFound),
        }
    }

    /// Read any pending data available without waiting.
    pub fn drain(&self) -> Result<()> {
        log::trace!("Draining pending data from probe");
        let mut buf = vec![0u8; 1024];
        match self {
            Self::V1 { device, report_size } => {
                loop {
                    match device.read_timeout(&mut buf[..*report_size], 1) {
                        Ok(n) if n > 0 => continue,
                        Ok(_) => break,
                        Err(e) => return Err(e)?,
                    }
                }
            },
            Self::V2 { handle, in_ep, .. } => {
                let timeout = Duration::from_millis(1);
                loop {
                    match handle.read_bulk(*in_ep, &mut buf[..], timeout) {
                        Ok(n) if n > 0 => continue,
                        Ok(_) => break,
                        Err(rusb::Error::Timeout) => break,
                        Err(e) => return Err(e)?,
                    }
                }
            },
        };
        Ok(())
    }

    /// Read up to one CMSIS-DAP packet from the probe, waiting up to 100ms.
    pub fn read(&self) -> Result<Vec<u8>> {
        // Read up to the maximum packet size. For HID devices, we'll always read a full
        // report which is exactly this size.
        let bufsize = match self {
            Self::V1 { report_size, .. } => *report_size,
            Self::V2 { max_packet_size, .. } => *max_packet_size,
        };
        let mut buf = vec![0u8; bufsize];
        let n = match self {
            Self::V1 { device, .. } => device.read_timeout(&mut buf[..], 100)?,
            Self::V2 { handle, in_ep, .. } => {
                let timeout = Duration::from_millis(100);
                handle.read_bulk(*in_ep, &mut buf[..], timeout)?
            },
        };
        buf.truncate(n);
        log::trace!("RX: {:02X?}", buf);
        Ok(buf)
    }

    /// Write `buf` to CMSIS-DAP probe, waiting up to 10ms.
    pub fn write(&self, buf: &[u8]) -> Result<usize> {
        log::trace!("TX: {:02X?}", buf);
        match self {
            Self::V1 { device, report_size, .. } => {
                let mut buf = buf.to_vec();
                // Extend buffer to report size for HID access, which requires
                // exactly report-sized packets.
                buf.resize(*report_size, 0);
                // Insert HID report ID at start.
                buf.insert(0, 0);
                Ok(device.write(&buf[..])?)
            },
            Self::V2 { handle, out_ep, .. } => {
                let timeout = Duration::from_millis(10);
                Ok(handle.write_bulk(*out_ep, buf, timeout)?)
            },
        }
    }
}

/// Metadata about a CMSIS-DAP probe.
///
/// Used to enumerate available probes and to specify
/// a specific probe to attempt to connect to.
#[derive(Clone, Debug)]
pub struct ProbeInfo {
    pub name: Option<String>,
    pub vid: u16,
    pub pid: u16,
    pub sn: Option<String>,
    pub v1_only: bool,
}

impl ProbeInfo {
    /// Find all connected CMSIS-DAP probes.
    pub fn list() -> Vec<Self> {
        log::trace!("Searching for CMSIS-DAP probes");
        match Context::new().and_then(|ctx| ctx.devices()) {
            Ok(devices) => devices.iter().filter_map(|d| Self::from_device(&d)).collect(),
            Err(_) => vec![],
        }
    }

    /// Create a ProbeInfo from a specifier string.
    pub fn from_specifier(spec: &str) -> Result<Self> {
        let parts: Vec<&str> = spec.split(':').collect();
        if parts.len() < 2 || parts.len() > 4 {
            return Err(Error::InvalidSpecifier);
        }
        let vid = u16::from_str_radix(parts[0], 16).or(Err(Error::InvalidSpecifier))?;
        let pid = u16::from_str_radix(parts[1], 16).or(Err(Error::InvalidSpecifier))?;
        let sn = if parts.len() >= 3 { Some(parts[2].to_owned()) } else { None };
        if parts.len() == 4 && parts[3] != "v1" {
            return Err(Error::InvalidSpecifier);
        }
        let v1_only = parts.len() == 4 && parts[3] == "v1";
        Ok(ProbeInfo { name: None, vid, pid, sn, v1_only })
    }

    /// Attempt to open a Probe corresponding to this ProbeInfo.
    pub fn open(&self) -> Result<Probe> {
        log::trace!("Opening probe: {}", self);

        // Only attempt to open as a v2 device if v1_only is not true.
        if !self.v1_only {
            if let Ok(devices) = Context::new().and_then(|ctx| ctx.devices()) {
                for device in devices.iter() {
                    match ProbeInfo::from_device(&device) {
                        Some(info) => if info.matches(self) {
                            if let Ok(probe) = Probe::from_device(device) {
                                return Ok(probe);
                            }
                        },
                        None => continue,
                    }
                }
            }
        }

        // Always attempt to open as a v1 device if opening as a v2 device fails.
        Probe::from_hid(self)
    }

    /// Create a ProbeInfo from an rusb Device if it is a CMSIS-DAP probe.
    ///
    /// Returns None if the device could not be read or was not a CMSIS-DAP device.
    fn from_device(device: &Device<Context>) -> Option<ProbeInfo> {
        let timeout = Duration::from_millis(100);
        let desc = device.device_descriptor().ok()?;
        let handle = device.open().ok()?;
        let language = handle.read_languages(timeout).ok()?.get(0).cloned()?;
        let prod_str = handle.read_product_string(language, &desc, timeout).ok()?;
        let sn_str = handle.read_serial_number_string(language, &desc, timeout).ok();

        if prod_str.contains("CMSIS-DAP") {
            Some(Self {
                name: Some(prod_str),
                vid: desc.vendor_id(),
                pid: desc.product_id(),
                sn: sn_str,
                v1_only: false,
            })
        } else {
            None
        }
    }

    /// Check if this `ProbeInfo` is valid for `target`.
    ///
    /// Always checks `vid` and `pid`, checks `sn` if not None.
    fn matches(&self, target: &Self) -> bool {
        if self.vid == target.vid && self.pid == target.pid {
            match target.sn {
                None => true,
                Some(_) => self.sn == target.sn,
            }
        } else {
            false
        }
    }
}

impl std::fmt::Display for ProbeInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = self.name.clone().unwrap_or_else(|| "Unknown".to_owned());
        let sn = self.sn.clone().unwrap_or_else(|| "".to_owned());
        write!(f, "{:04x}:{:04x}:{} {}", self.vid, self.pid, sn, name)
    }
}
