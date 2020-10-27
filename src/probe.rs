use std::time::Duration;
use thiserror::Error;
use rusb::{Device, Context, UsbContext};
use hidapi::HidApi;

#[derive(Error, Debug)]
pub enum Error {
    #[error("invalid specifier, use VID:PID or VID:PID:Serial")]
    InvalidSpecifier,
    #[error("specified probe not found")]
    NotFound,
    #[error("no CMSIS-DAP probes found")]
    NoProbesFound,
    #[error("multiple CMSIS-DAP probes found, select a specific probe")]
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
    V1(hidapi::HidDevice),

    /// CMSIS-DAP v2 over WinUSB/Bulk.
    V2 {
        handle: rusb::DeviceHandle<rusb::Context>,
        out_ep: u8,
        in_ep: u8,
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
                        return Ok(Probe::V2 { handle, out_ep, in_ep });
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
                    Ok(Probe::V1(device))
                },
                _ => Err(Error::NotFound),
            },
            _ => Err(Error::NotFound),
        }
    }

    pub fn read(&self) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; 1024];
        let n = match self {
            Self::V1(device) => device.read_timeout(&mut buf[..], 100)?,
            Self::V2 { handle, out_ep: _, in_ep } => {
                let timeout = Duration::from_millis(100);
                handle.read_bulk(*in_ep, &mut buf[..], timeout)?
            },
        };
        buf.truncate(n);
        log::trace!("RX: {:02X?}", buf);
        Ok(buf)
    }

    pub fn write(&self, buf: &[u8]) -> Result<usize> {
        log::trace!("TX: {:02X?}", buf);
        match self {
            Self::V1(device) => {
                let mut buf = buf.to_vec();
                // Extend buffer to 64 bytes for HID access, which requires
                // exactly report-sized packets.  We can't in general find
                // out what the report size is, but 64 is very common.
                buf.resize(64, 0);
                // Insert HID report ID at start.
                buf.insert(0, 0);
                Ok(device.write(&buf[..])?)
            },
            Self::V2 { handle, out_ep, in_ep: _ } => {
                let timeout = Duration::from_millis(100);
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
        if parts.len() < 2 || parts.len() > 3 {
            return Err(Error::InvalidSpecifier);
        }
        let vid = u16::from_str_radix(parts[0], 16).or(Err(Error::InvalidSpecifier))?;
        let pid = u16::from_str_radix(parts[1], 16).or(Err(Error::InvalidSpecifier))?;
        let sn = if parts.len() == 3 { Some(parts[2].to_owned()) } else { None };
        Ok(ProbeInfo { name: None, vid, pid, sn })
    }

    /// Attempt to open a Probe corresponding to this ProbeInfo.
    pub fn open(&self) -> Result<Probe> {
        log::trace!("Opening probe: {}", self);
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
