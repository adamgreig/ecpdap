// Copyright 2019-2022 Adam Greig
// Licensed under the Apache-2.0 and MIT licenses.

use std::{io::Write, fs::File, time::{Instant, Duration}};
use clap::{Command, Arg, ArgAction, crate_description, crate_version, value_parser};
use clap_num::{maybe_hex, si_number};
use anyhow::bail;
use spi_flash::Flash;

use jtagdap::probe::{Probe, ProbeInfo};
use jtagdap::dap::DAP;
use jtagdap::jtag::{JTAG, JTAGChain};
use ecpdap::{ECP5, ECP5IDCODE, Bitstream, check_tap_idx, auto_tap_idx};

#[allow(clippy::cognitive_complexity)]
fn main() -> anyhow::Result<()> {
    let matches = Command::new("ecpdap")
        .version(crate_version!())
        .about(crate_description!())
        .subcommand_required(true)
        .arg_required_else_help(true)
        .propagate_version(true)
        .infer_subcommands(true)
        .arg(Arg::new("quiet")
             .help("Suppress informative output and raise log level to errors only")
             .long("quiet")
             .short('q')
             .action(ArgAction::SetTrue)
             .global(true))
        .arg(Arg::new("verbose")
             .help("Increase log level, specify once for info, twice for debug, three times for trace")
             .long("verbose")
             .short('v')
             .action(ArgAction::Count)
             .conflicts_with("quiet")
             .global(true))
        .arg(Arg::new("probe")
             .help("VID:PID[:SN] of CMSIS-DAP device to use")
             .long("probe")
             .short('p')
             .action(ArgAction::Set)
             .global(true))
        .arg(Arg::new("freq")
             .help("JTAG clock frequency in Hz (k and M suffixes allowed)")
             .long("freq")
             .short('f')
             .action(ArgAction::Set)
             .default_value("1M")
             .value_parser(si_number::<u32>)
             .global(true))
        .arg(Arg::new("tap")
             .help("ECP5's TAP position in scan chain (0-indexed, see `scan` output)")
             .long("tap")
             .short('t')
             .action(ArgAction::Set)
             .value_parser(value_parser!(usize))
             .global(true))
        .arg(Arg::new("ir-lengths")
             .help("Lengths of each IR, starting from TAP 0, comma-separated")
             .long("ir-lengths")
             .short('i')
             .action(ArgAction::Set)
             .value_delimiter(',')
             .value_parser(value_parser!(usize))
             .global(true))
        .arg(Arg::new("scan-chain-length")
             .help("Maximum JTAG scan chain length to check")
             .long("scan-chain-length")
             .short('l')
             .action(ArgAction::Set)
             .default_value("192")
             .value_parser(value_parser!(usize))
             .global(true))
        .arg(Arg::new("fix-idcode")
            .help("Disable fixing compatible IDCODEs when writing bitstreams")
            .long("no-fix-idcode")
            .action(ArgAction::SetFalse)
            .global(true))
        .arg(Arg::new("remove-idcode")
            .help("Replace VERIFY_IDCODE bitstream commands with NOOP when writing bitstreams")
            .long("remove-idcode")
            .action(ArgAction::SetTrue)
            .global(true))
        .subcommand(Command::new("probes")
            .about("List available CMSIS-DAP probes"))
        .subcommand(Command::new("scan")
            .about("Scan JTAG chain and detect ECP5 IDCODEs"))
        .subcommand(Command::new("reset")
            .about("Pulse the JTAG nRST line for 100ms"))
        .subcommand(Command::new("reload")
            .about("Request the ECP5 reload its configuration"))
        .subcommand(Command::new("program")
            .about("Program ECP5 SRAM with bitstream")
            .arg(Arg::new("file")
                 .help("File to program to ECP5")
                 .required(true))
            .arg(Arg::new("remove-spimode")
                .help("Disable removing SPI_MODE commands when writing bitstreams to SRAM")
                .long("no-remove-spimode")
                .action(ArgAction::SetFalse)
                .global(true)))
        .subcommand(Command::new("flash")
            .about("Access SPI flash attached to ECP5 when no other devices on JTAG chain")
            .subcommand_required(true)
            .arg_required_else_help(true)
            .subcommand(Command::new("id")
                .about("Read SPI flash ID"))
            .subcommand(Command::new("scan")
                .about("Read SPI flash parameters"))
            .subcommand(Command::new("erase")
                .about("Erase entire SPI flash"))
            .subcommand(Command::new("write")
                .about("Write binary file to SPI flash")
                .arg(Arg::new("file")
                     .help("File to write to SPI flash")
                     .required(true))
                .arg(Arg::new("offset")
                     .help("Start address (in bytes) to write to (decimal, or hex with 0x prefix)")
                     .long("offset")
                     .value_parser(maybe_hex::<u32>)
                     .default_value("0"))
                .arg(Arg::new("verify")
                     .help("Disable readback verification")
                     .short('n')
                     .long("no-verify")
                     .action(ArgAction::SetFalse))
                .arg(Arg::new("reload")
                     .help("Don't request the ECP5 reload configuration after programming")
                     .long("no-reload")
                     .action(ArgAction::SetFalse)))
            .subcommand(Command::new("read")
                .about("Read SPI flash contents to file")
                .arg(Arg::new("file")
                     .help("File to write SPI flash contents to")
                     .required(true))
                .arg(Arg::new("offset")
                     .help("Start address (in bytes) of read (decimal, or hex with 0x prefix)")
                     .long("offset")
                     .action(ArgAction::Set)
                     .value_parser(maybe_hex::<u32>)
                     .default_value("0"))
                .arg(Arg::new("length")
                     .help("Length (in bytes) of read, defaults to detected capacity \
                           (decimal, or hex with 0x prefix)")
                     .long("length")
                     .action(ArgAction::Set)
                     .value_parser(maybe_hex::<usize>)))
            .subcommand(Command::new("jump")
                .about("Read and write ECP5 JUMP command in last page of flash")
                .subcommand_required(true)
                .arg_required_else_help(true)
                .arg(Arg::new("offset")
                     .help("Offset for jump address, default is last 256 bytes of flash \
                           (decimal, or hex with 0x prefix)")
                     .long("offset")
                     .global(true)
                     .action(ArgAction::Set)
                     .value_parser(maybe_hex::<u32>))
                .subcommand(Command::new("read")
                    .about("Read ECP5 JUMP command from flash"))
                .subcommand(Command::new("write")
                    .about("Write ECP5 JUMP command to flash")
                    .arg(Arg::new("spimode")
                         .help("SPI flash read opcode")
                         .long("spimode")
                         .default_value("read")
                         .value_parser(["read", "fast-read", "dual", "quad"])
                         .action(ArgAction::Set))
                    .arg(Arg::new("target")
                         .help("Jump target address (decimal, or hex with 0x prefix)")
                         .required(true)
                         .value_parser(maybe_hex::<u32>))
                    .arg(Arg::new("reload")
                         .help("Don't request the ECP5 reload configuration after programming")
                         .long("no-reload")
                         .action(ArgAction::SetFalse))))
            .subcommand(Command::new("protect")
                .about("Set all block protection bits in status register"))
            .subcommand(Command::new("unprotect")
                .about("Clear all block protection bits in status register"))
            )
        .get_matches();

    let t0 = Instant::now();
    let quiet = matches.get_flag("quiet");
    let verbose = matches.get_count("verbose");
    let env = if quiet {
        env_logger::Env::default().default_filter_or("error")
    } else if verbose == 0 {
        env_logger::Env::default().default_filter_or("warn")
    } else if verbose == 1 {
        env_logger::Env::default().default_filter_or("info")
    } else if verbose == 2 {
        env_logger::Env::default().default_filter_or("debug")
    } else {
        env_logger::Env::default().default_filter_or("trace")
    };
    env_logger::Builder::from_env(env).format_timestamp(None).init();

    // Listing probes does not require first connecting to a probe,
    // so we just list them and quit early.
    if matches.subcommand_name().unwrap() == "probes" {
        print_probe_list();
        return Ok(());
    }

    // All functions after this point require an open probe, so
    // we now attempt to connect to the specified probe.
    let probe = if let Some(probe) = matches.get_one::<String>("probe") {
        ProbeInfo::from_specifier(probe)?.open()?
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
    if let Some(&freq) = matches.get_one::<u32>("freq") {
        jtag.set_clock(freq)?;
    }

    // If the user specified a JTAG scan chain length, apply it now.
    if let Some(&max_length) = matches.get_one("scan-chain-length") {
        jtag.set_max_length(max_length);
    }

    // If the user specified IR lengths, parse and save them.
    let ir_lens = matches
        .get_many("ir-lengths")
        .map(|lens| lens.copied().collect::<Vec<usize>>());

    // Scan the JTAG chain to detect all available TAPs.
    let chain = jtag.scan(ir_lens.as_deref())?;

    // At this point we can handle the 'scan' command.
    if matches.subcommand_name().unwrap() == "scan" {
        print_jtag_chain(&chain);
        return Ok(());
    }

    // If the user specified a TAP, we'll use it, but otherwise
    // attempt to find a single ECP5 in the scan chain.
    let (tap_idx, idcode) = if let Some(&tap_idx) = matches.get_one("tap") {
        if let Some(idcode) = check_tap_idx(&chain, tap_idx) {
            log::debug!("Provided tap index is an ECP5");
            (tap_idx, idcode)
        } else {
            print_jtag_chain(&chain);
            bail!("The provided tap index {tap_idx} does not have an ECP5 IDCODE.");
        }
    } else if let Some((index, idcode)) = auto_tap_idx(&chain) {
        (index, idcode)
    } else {
        print_jtag_chain(&chain);
        bail!("Could not find an ECP5 IDCODE in the JTAG chain.");
    };

    // Create a TAP instance, consuming the JTAG instance.
    let tap = jtag.into_tap(chain, tap_idx)?;

    // Create an ECP5 instance from the TAP.
    let mut ecp5 = ECP5::new(tap, idcode);
    let idcode = ecp5.idcode();

    // We can finally handle 'program' and 'flash' commands.
    match matches.subcommand_name() {
        Some("reload") => {
            if !quiet { println!("Reloading ECP5 configuration...") };
            ecp5.refresh()?;
        },
        Some("program") => {
            let matches = matches.subcommand_matches("program").unwrap();
            let path = matches.get_one::<String>("file").unwrap();
            let mut bitstream = Bitstream::from_path(path)?;
            if matches.get_flag("remove-idcode") {
                bitstream.remove_idcode()?;
            } else {
                let fix_idcode = matches.get_flag("fix-idcode");
                bitstream.check_and_fix_idcode(idcode, fix_idcode)?;
            }
            if matches.get_flag("remove-spimode") {
                bitstream.remove_spimode()?;
            }
            if quiet {
                ecp5.program(bitstream.data())?;
            } else {
                ecp5.program_progress(bitstream.data())?;
            }
        },
        Some("flash") => {
            let mut ecp5_flash = ecp5.into_flash()?;
            let mut flash = Flash::new(&mut ecp5_flash);
            // Always bring flash out of power-down before trying to access.
            flash.release_power_down()?;
            // Always read parameter table if available, to load
            // settings for address bytes, capacity, opcodes, etc.
            flash.read_params()?;
            let matches = matches.subcommand_matches("flash").unwrap();
            match matches.subcommand_name() {
                Some("id") => {
                    let id = flash.read_id()?;
                    println!("{}", id);
                },
                Some("scan") => {
                    if !quiet { println!("Reading flash ID...") };
                    let id = flash.read_id()?;
                    println!("{}", id);
                    if !quiet { println!("\nReading flash parameters...") };
                    match flash.get_params() {
                        Some(params) => println!("{}", params),
                        None => println!("No SFDP header found. Check flash supports SFDP."),
                    }
                    if !quiet { println!("Reading status registers...") };
                    let status1 = flash.read_status1()?;
                    let status2 = flash.read_status2()?;
                    let status3 = flash.read_status3()?;
                    println!("Status 1: 0x{:02X}, status 2: 0x{:02X}, status 3: 0x{:02X}",
                             status1.0, status2.0, status3.0);
                    let (bp0, bp1, bp2) = status1.get_block_protect();
                    let sec = status1.get_sec();
                    let tb = status1.get_tb();
                    println!("BP0: {}, BP1: {}, BP2: {}, SEC: {}, TB: {}", bp0, bp1, bp2, sec, tb);
                },
                Some("erase") => {
                    if quiet {
                        flash.erase()?;
                    } else {
                        flash.erase_progress()?;
                    }
                },
                Some("write") => {
                    if !quiet && flash.is_protected()? {
                        println!("Flash appears to be write-protected; writing may fail.");
                    }
                    let matches = matches.subcommand_matches("write").unwrap();
                    let path = matches.get_one::<String>("file").unwrap();
                    let offset = *matches.get_one("offset").unwrap();
                    let verify = matches.get_flag("verify");
                    let reload = matches.get_flag("reload");
                    let mut bitstream = Bitstream::from_path(path)?;
                    if matches.get_flag("remove-idcode") {
                        bitstream.remove_idcode()?;
                    } else {
                        let fix_idcode = matches.get_flag("fix-idcode");
                        bitstream.check_and_fix_idcode(idcode, fix_idcode)?;
                    }
                    if quiet {
                        flash.program(offset, bitstream.data(), verify)?;
                    } else {
                        flash.program_progress(offset, bitstream.data(), verify)?;
                    }
                    if reload {
                        let mut ecp5 = ecp5_flash.release();
                        ecp5.refresh()?;
                    }
                },
                Some("read") => {
                    let matches = matches.subcommand_matches("read").unwrap();
                    let path = matches.get_one::<String>("file").unwrap();
                    let offset = *matches.get_one("offset").unwrap();
                    let length = if let Some(length) = matches.get_one("length") {
                        *length
                    } else if let Some(capacity) = flash.capacity() {
                            log::info!("No length specified, using detected capacity");
                            capacity
                    } else {
                        bail!("Could not detect flash capacity; specify --length instead.");
                    };
                    let mut file = File::create(path)?;
                    let data = if quiet {
                        flash.read(offset, length)?
                    } else {
                        flash.read_progress(offset, length)?
                    };
                    file.write_all(&data)?;
                },
                Some("jump") => {
                    let matches = matches.subcommand_matches("jump").unwrap();
                    let offset = if let Some(offset) = matches.get_one("offset") {
                        *offset
                    } else if let Some(capacity) = flash.capacity() {
                        let offset = (capacity - 256) as u32;
                        log::info!("No offset specified, using 0x{offset:06X}");
                        offset
                    } else {
                        // If capacity is unknown, spi-flash will permit writing to this
                        // large address, and the spi flash itself will ignore any unused
                        // high bits, leaving us with the right address, so long as it uses
                        // 3-byte addressing.
                        let offset = 0x00FFFF00;
                        log::info!("No offset specified and no capacity detected, using 0x{offset:06X}");
                        offset
                    };
                    match matches.subcommand_name() {
                        Some("read") => {
                            let data = flash.read(offset, 256)?;
                            let bitstream = Bitstream::new(data);
                            if let Some((spimode, target)) = bitstream.jump() {
                                let spimode_name = match spimode {
                                    0x03 => "read",
                                    0x0B => "fast-read",
                                    0xBB => "dual",
                                    0xEB => "quad",
                                    _    => "unknown",
                                };
                                println!("Found JUMP to 0x{target:06X} with SPI opcode \
                                         0x{spimode:02X} ({spimode_name})");
                            } else {
                                println!("No JUMP command found");
                            }
                        },
                        Some("write") => {
                            let matches = matches.subcommand_matches("write").unwrap();
                            let target = matches.get_one::<u32>("target").unwrap();
                            let reload = matches.get_flag("reload");
                            let spimode = match matches.get_one::<String>("spimode").unwrap().as_str() {
                                "read" => 0x03,
                                "fast-read" => 0x0B,
                                "dual" => 0xBB,
                                "quad" => 0xEB,
                                _ => panic!("Unhandled flash jump spimode"),
                            };
                            let bitstream = Bitstream::from_jump(spimode, *target);
                            if quiet {
                                flash.program(offset, bitstream.data(), true)?;
                            } else {
                                flash.program_progress(offset, bitstream.data(), true)?;
                            }
                            if reload {
                                let mut ecp5 = ecp5_flash.release();
                                ecp5.refresh()?;
                            }
                        },
                        _ => panic!("Unhandled flash jump subcommand."),
                    }
                },
                Some("protect") => {
                    if !quiet { println!("Setting block protection bits...") };
                    flash.protect(true, true, true)?;
                    if !quiet { println!("All block protection bits set.") };
                },
                Some("unprotect") => {
                    if !quiet { println!("Disabling flash write protection...") };
                    flash.unprotect()?;
                    if !quiet { println!("Flash protected disabled.") };
                },
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
    let idcodes = chain.idcodes();
    let lines = chain.to_lines();
    for (idcode, line) in idcodes.iter().zip(lines.iter()) {
        if let Some(Some(ecp5)) = idcode.map(ECP5IDCODE::try_from_idcode) {
            println!(" - {} [{}]", line, ecp5.name());
        } else {
            println!(" - {}", line);
        }
    }
}
