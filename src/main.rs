use std::fs::File;
use std::io::prelude::*;
use std::time::{Instant, Duration};
use clap::{Arg, App, AppSettings, SubCommand};
use clap::{value_t, crate_description, crate_version};

use ecpdap::probe::{Probe, ProbeInfo};
use ecpdap::dap::DAP;
use ecpdap::jtag::JTAG;
use ecpdap::ecp5::ECP5;

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
        .subcommand(SubCommand::with_name("probes")
            .about("List available CMSIS-DAP probes"))
        .subcommand(SubCommand::with_name("scan")
            .about("Scan JTAG chain for ECP5 IDCODEs"))
        .subcommand(SubCommand::with_name("reset")
            .about("Pulse the JTAG nRST line for 100ms"))
        .subcommand(SubCommand::with_name("program")
            .about("Program ECP5 SRAM with bitstream")
            .arg(Arg::with_name("file")
                 .help("File to program to ECP5")
                 .required(true)))
        .subcommand(SubCommand::with_name("flash")
            .about("Access SPI flash attached to ECP5")
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
                     .help("Disable automatic readback verification")
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
                     .required(true))
                .arg(Arg::with_name("offset")
                     .help("Start address (in bytes) of read")
                     .long("offset")
                     .default_value("0"))))
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

    let dap = DAP::new(probe)?;
    let jtag = JTAG::new(dap)?;

    match value_t!(matches, "freq", u32) {
        Ok(freq) => jtag.set_clock(freq * 1000)?,
        Err(e) => {
            drop(jtag);
            e.exit();
        }
    }

    match matches.subcommand_name() {
        Some("scan") => {
            let (code, _) = ECP5::scan(&jtag)?;
            println!("Found {}", code.name());
        },
        Some("reset") => jtag.pulse_nrst(Duration::from_millis(100))?,
        Some("program") => {
            let ecp5 = ECP5::new(&jtag)?;
            let matches = matches.subcommand_matches("program").unwrap();
            let path = matches.value_of("file").unwrap();
            let mut file = File::open(path)?;
            let mut data = Vec::new();
            file.read_to_end(&mut data)?;
            ecp5.program(&data)?;
        },
        Some("flash") => {
            let matches = matches.subcommand_matches("flash").unwrap();
            match matches.subcommand_name() {
                Some("id") => {},
                Some("erase") => {},
                Some("write") => {},
                Some("read") => {},
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
    if probes.len() == 0 {
        println!("No CMSIS-DAP probes found.");
    } else {
        println!("Found {} CMSIS-DAP probe{}:", probes.len(),
                 if probes.len() == 1 { "" } else { "s" });
        for probe in probes {
            println!("  {}", probe);
        }
    }
}
