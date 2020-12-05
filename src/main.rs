use std::fs::File;
use std::io::prelude::*;
use std::time::{Instant, Duration};
use clap::{Arg, App, AppSettings, SubCommand};
use clap::{value_t, values_t, crate_description, crate_version};
use anyhow::bail;

use ecpdap::probe::{Probe, ProbeInfo};
use ecpdap::dap::DAP;
use ecpdap::jtag::{JTAG, JTAGChain};
use ecpdap::ecp5::ECP5;
use ecpdap::flash::Flash;

#[allow(clippy::cognitive_complexity)]
fn main() -> anyhow::Result<()> {
    let matches = App::new("ecpdap")
        .version(crate_version!())
        .about(crate_description!())
        .setting(AppSettings::SubcommandRequiredElseHelp)
        .global_setting(AppSettings::ColoredHelp)
        .global_setting(AppSettings::DeriveDisplayOrder)
        .global_setting(AppSettings::GlobalVersion)
        .global_setting(AppSettings::InferSubcommands)
        .global_setting(AppSettings::VersionlessSubcommands)
        .arg(Arg::with_name("quiet")
             .help("Suppress informative output")
             .long("quiet")
             .short("q")
             .global(true))
        .arg(Arg::with_name("probe")
             .help("VID:PID[:SN] of CMSIS-DAP device to use")
             .long("probe")
             .short("p")
             .takes_value(true)
             .global(true))
        .arg(Arg::with_name("freq")
             .help("JTAG clock frequency to use, in kHz")
             .long("freq")
             .short("f")
             .takes_value(true)
             .default_value("1000")
             .global(true))
        .arg(Arg::with_name("tap")
             .help("ECP5's TAP position in scan chain (0-indexed, see `scan` output)")
             .long("tap")
             .short("t")
             .takes_value(true)
             .global(true))
        .arg(Arg::with_name("ir-lengths")
             .help("Lengths of each IR, starting from TAP 0, comma-separated")
             .long("ir-lengths")
             .short("i")
             .multiple(true)
             .require_delimiter(true)
             .takes_value(true)
             .global(true))
        .arg(Arg::with_name("scan-chain-length")
             .help("Maximum JTAG scan chain length to check")
             .long("scan-chain-length")
             .short("l")
             .takes_value(true)
             .default_value("192")
             .global(true))
        .subcommand(SubCommand::with_name("probes")
            .about("List available CMSIS-DAP probes"))
        .subcommand(SubCommand::with_name("scan")
            .about("Scan JTAG chain and detect ECP5 IDCODEs"))
        .subcommand(SubCommand::with_name("reset")
            .about("Pulse the JTAG nRST line for 100ms"))
        .subcommand(SubCommand::with_name("program")
            .about("Program ECP5 SRAM with bitstream")
            .arg(Arg::with_name("file")
                 .help("File to program to ECP5")
                 .required(true)))
        .subcommand(SubCommand::with_name("flash")
            .about("Access SPI flash attached to ECP5 when no other devices on JTAG chain")
            .setting(AppSettings::SubcommandRequiredElseHelp)
            .subcommand(SubCommand::with_name("id")
                .about("Read SPI flash ID"))
            .subcommand(SubCommand::with_name("erase")
                .about("Erase SPI flash"))
            .subcommand(SubCommand::with_name("write")
                .about("Write binary file to SPI flash")
                .arg(Arg::with_name("file")
                     .help("File to write to SPI flash")
                     .required(true))
                .arg(Arg::with_name("offset")
                     .help("Start address (in bytes) to write to")
                     .long("offset")
                     .default_value("0"))
                .arg(Arg::with_name("no-verify")
                     .help("Disable readback verification")
                     .short("n")
                     .long("no-verify")))
            .subcommand(SubCommand::with_name("read")
                .about("Read SPI flash contents to file")
                .arg(Arg::with_name("file")
                     .help("File to write SPI flash contents to")
                     .required(true))
                .arg(Arg::with_name("length")
                     .help("Length (in bytes) of read")
                     .long("length")
                     .takes_value(true)
                     .required(true))
                .arg(Arg::with_name("offset")
                     .help("Start address (in bytes) of read")
                     .long("offset")
                     .takes_value(true)
                     .default_value("0")))
            .subcommand(SubCommand::with_name("unprotect")
                .about("Unprotect SPI flash")))
        .get_matches();

    pretty_env_logger::init();
    let t0 = Instant::now();
    let quiet = matches.is_present("quiet");

    // Listing probes does not require first connecting to a probe,
    // so we just list them and quit early.
    if matches.subcommand_name().unwrap() == "probes" {
        print_probe_list();
        return Ok(());
    }

    // All functions after this point require an open probe, so
    // we now attempt to connect to the specified probe.
    let probe = if matches.is_present("probe") {
        ProbeInfo::from_specifier(matches.value_of("probe").unwrap())?.open()?
    } else {
        Probe::new()?
    };

    // Create a JTAG interface using the probe.
    let dap = DAP::new(probe)?;
    let mut jtag = JTAG::new(dap);

    // At this point we can handle the reset command.
    if matches.subcommand_name().unwrap() == "reset" {
        if !quiet { println!("Pulsing nRST line.") };
        return Ok(jtag.pulse_nrst(Duration::from_millis(100))?);
    }

    // If the user specified a JTAG clock frequency, apply it now.
    match value_t!(matches, "freq", u32) {
        Ok(freq) => jtag.set_clock(freq * 1000)?,
        Err(e) => {
            drop(jtag);
            e.exit();
        }
    }

    // If the user specified a JTAG scan chain length, apply it now.
    match value_t!(matches, "scan-chain-length", usize) {
        Ok(max_length) => jtag.set_max_length(max_length),
        Err(e) => {
            drop(jtag);
            e.exit();
        }
    }

    // If the user specified IR lengths, parse and save them.
    let ir_lens = if matches.is_present("ir-lengths") {
        match values_t!(matches, "ir-lengths", usize) {
            Ok(lengths) => Some(lengths),
            Err(e) => {
                drop(jtag);
                e.exit();
            }
        }
    } else {
        None
    };

    // Scan the JTAG chain to detect all available TAPs.
    let chain = jtag.scan(ir_lens.as_deref())?;

    // At this point we can handle the 'scan' command.
    if matches.subcommand_name().unwrap() == "scan" {
        print_jtag_chain(&chain);
        return Ok(());
    }

    // If the user specified a TAP, we'll use it, but otherwise
    // attempt to find a single ECP5 in the scan chain.
    let tap_idx = if matches.is_present("tap") {
        match value_t!(matches, "tap", usize) {
            Ok(tap) => if chain.check_idx(tap) {
                log::debug!("Provided tap index is an ECP5");
                tap
            } else {
                print_jtag_chain(&chain);
                bail!("The provided tap index {} does not have an ECP5 IDCODE.", tap);
            },
            Err(e) => {
                drop(jtag);
                e.exit();
            }
        }
    } else {
        match chain.auto_idx() {
            Some(index) => index,
            None => {
                print_jtag_chain(&chain);
                bail!("Could not find an ECP5 IDCODE in the JTAG chain.");
            }
        }
    };

    // Create a TAP instance, consuming the JTAG instance.
    let tap = jtag.to_tap(chain, tap_idx)?;

    // Create an ECP5 instance from the TAP.
    let mut ecp5 = ECP5::new(tap);

    // We can finally handle 'program' and 'flash' commands.
    match matches.subcommand_name() {
        Some("program") => {
            let matches = matches.subcommand_matches("program").unwrap();
            let path = matches.value_of("file").unwrap();
            let mut file = File::open(path)?;
            let mut data = Vec::new();
            file.read_to_end(&mut data)?;
            ecp5.program(&data)?;
            if !quiet { println!("Configuration programmed OK.") };
        },
        Some("flash") => {
            let mut flash = Flash::new(ecp5)?;
            let matches = matches.subcommand_matches("flash").unwrap();
            match matches.subcommand_name() {
                Some("id") => {
                    let id = flash.read_id()?;
                    println!("{}", id);
                },
                Some("erase") => {
                    if !quiet { println!("Erasing flash...") };
                    flash.erase()?;
                    if !quiet { println!("Flash erased.") };
                },
                Some("write") => {
                    let matches = matches.subcommand_matches("write").unwrap();
                    let path = matches.value_of("file").unwrap();
                    let offset = value_t!(matches, "offset", u32).unwrap();
                    let verify = !matches.is_present("no-verify");
                    let mut file = File::open(path)?;
                    let mut data = Vec::new();
                    file.read_to_end(&mut data)?;
                    if !quiet { println!("Programming flash...") };
                    flash.program(offset, &data, verify)?;
                    if !quiet { println!("Flash programmed.") };
                },
                Some("read") => {
                    let matches = matches.subcommand_matches("read").unwrap();
                    let path = matches.value_of("file").unwrap();
                    let offset = value_t!(matches, "offset", u32).unwrap();
                    let length = value_t!(matches, "length", usize).unwrap();
                    let mut file = File::create(path)?;
                    if !quiet { println!("Reading flash...") };
                    let data = flash.read(offset, length)?;
                    file.write_all(&data)?;
                    if !quiet { println!("Flash read.") };
                },
                Some("unprotect") => {
                    if !quiet { println!("Disabling flash write protection...") };
                    flash.unprotect()?;
                    if !quiet { println!("Flash protected disabled.") };
                }
                _ => panic!("Unhandled flash subcommand."),
            }
        },
        _ => panic!("Unhandled command."),
    }

    let t1 = t0.elapsed();
    if !quiet {
        println!("Finished in {}.{:02}s", t1.as_secs(), t1.subsec_millis()/10);
    }

    Ok(())
}

fn print_probe_list() {
    let probes = ProbeInfo::list();
    if probes.is_empty() {
        println!("No CMSIS-DAP probes found.");
    } else {
        println!("Found {} CMSIS-DAP probe{}:", probes.len(),
                 if probes.len() == 1 { "" } else { "s" });
        for probe in probes {
            println!("  {}", probe);
        }
    }
}

fn print_jtag_chain(chain: &JTAGChain) {
    println!("Detected JTAG chain, closest to TDO first:");
    for line in chain.to_lines() {
        println!(" - {}", line);
    }
}
