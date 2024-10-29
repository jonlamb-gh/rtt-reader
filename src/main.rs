use clap::Parser;
use human_bytes::human_bytes;
use probe_rs::{
    probe::{list::Lister, DebugProbeSelector, WireProtocol},
    rtt::{ChannelMode, Rtt, ScanRegion},
    Core, CoreStatus, HaltReason, Permissions, RegisterValue, VectorCatchCondition,
};
use simple_moving_average::{NoSumSMA, SMA};
use std::{
    fs,
    io::{self, Write},
    path::PathBuf,
    sync::atomic::{AtomicBool, Ordering::SeqCst},
    sync::Arc,
    time::{Duration, Instant},
};
use tracing::{debug, error, info, trace, warn};

/// Write RTT data from target to a file
#[derive(Parser, Debug, Clone)]
#[clap(version)]
#[command(name = "rtt-reader")]
struct Opts {
    /// Specify a target attach timeout.
    /// When provided, the plugin will continually attempt to attach and search
    /// for a valid RTT control block anywhere in the target RAM.
    ///
    /// Accepts durations like "10ms" or "1minute 2seconds 22ms".
    #[clap(long, name = "attach-timeout")]
    pub attach_timeout: Option<humantime::Duration>,

    /// Use the provided RTT control block address instead of scanning the target memory for it.
    #[clap(long, name = "control-block-address", value_parser=clap_num::maybe_hex::<u32>)]
    pub control_block_address: Option<u32>,

    /// The RTT up (target to host) channel number to poll on (defaults to 1).
    #[clap(long, name = "up-channel", default_value = "1")]
    pub up_channel: usize,

    /// Select a specific probe instead of opening the first available one.
    ///
    /// Use '--probe VID:PID' or '--probe VID:PID:Serial' if you have more than one probe with the same VID:PID.
    #[structopt(long = "probe", name = "probe")]
    pub probe_selector: Option<DebugProbeSelector>,

    /// The target chip to attach to (e.g. STM32F407VE).
    #[clap(long, name = "chip")]
    pub chip: String,

    /// Protocol used to connect to chip.
    /// Possible options: [swd, jtag].
    ///
    /// The default value is swd.
    #[structopt(long, name = "protocol", default_value = "Swd")]
    pub protocol: WireProtocol,

    /// The protocol speed in kHz.
    ///
    /// The default value is 4000.
    #[clap(long, name = "speed", default_value = "4000")]
    pub speed: u32,

    /// The selected core to target.
    ///
    /// The default value is 0.
    #[clap(long, name = "core", default_value = "0")]
    pub core: usize,

    /// Reset the target on startup.
    #[clap(long, name = "reset")]
    pub reset: bool,

    /// Attach to the chip under hard-reset.
    #[clap(long, name = "attach-under-reset")]
    pub attach_under_reset: bool,

    /// Chip description YAML file path.
    /// Provides custom target descriptions based on CMSIS Pack files.
    #[clap(long, name = "chip-description-path")]
    pub chip_description_path: Option<PathBuf>,

    /// Show verbose information (location info, etc)
    #[arg(short, long)]
    pub verbose: bool,

    /// Set a breakpoint on the address of the given symbol when
    /// enabling RTT BlockIfFull channel mode.
    ///
    /// Can be an absolute address or symbol name.
    #[arg(long, requires = "elf-file")]
    pub breakpoint: Option<String>,

    /// Assume thumb mode when resolving symbols from the ELF file
    /// for breakpoints.
    #[arg(long, requires = "breakpoint")]
    pub thumb: bool,

    /// Enable RTT BlockIfFull channel mode
    #[arg(long)]
    pub blocking: bool,

    /// The ELF file containing the RTT symbols
    #[clap(long, name = "elf-file")]
    pub elf_file: Option<PathBuf>,

    /// The output file to write to
    #[clap(long, short = 'o', default_value = "rtt.bin")]
    pub output: PathBuf,
}

fn main() {
    let opts = Opts::parse();

    tracing_subscriber::fmt::init();

    let intr = Interruptor::new();
    let intr_clone = intr.clone();
    ctrlc::set_handler(move || {
        if intr_clone.is_set() {
            let exit_code = if cfg!(target_family = "unix") {
                // 128 (fatal error signal "n") + 2 (control-c is fatal error signal 2)
                130
            } else {
                // Windows code 3221225786
                // -1073741510 == C000013A
                -1073741510
            };
            std::process::exit(exit_code);
        }

        debug!("Shutdown signal received");
        intr_clone.set();
    })
    .expect("ctrlc handler");

    let mut buffer = vec![0_u8; 1024];
    debug!(output = %opts.output.display(), "Creating output file");
    let mut out_file = fs::File::create(&opts.output).expect("create output file");

    if let Some(chip_desc) = &opts.chip_description_path {
        debug!(path = %chip_desc.display(), "Adding custom chip description");
        let f = fs::File::open(chip_desc).expect("chip-desc file open");
        probe_rs::config::add_target_from_yaml(f).unwrap();
    }

    let lister = Lister::new();
    let mut probe = if let Some(probe_selector) = &opts.probe_selector {
        debug!(probe_selector = %probe_selector, "Opening selected probe");
        lister.open(probe_selector.clone()).expect("listen open")
    } else {
        let probes = lister.list_all();
        debug!(probes = probes.len(), "Opening first available probe");
        if probes.is_empty() {
            panic!("No probes available");
        }
        probes[0].open().expect("probe open")
    };

    debug!(protocol = %opts.protocol, speed = opts.speed, "Configuring probe");
    probe.select_protocol(opts.protocol).unwrap();
    probe.set_speed(opts.speed).unwrap();

    debug!(chip = opts.chip, core = opts.core, "Attaching to chip");

    let mut session = if opts.attach_under_reset {
        probe
            .attach_under_reset(opts.chip, Permissions::default())
            .expect("probe attach session")
    } else {
        probe
            .attach(opts.chip, Permissions::default())
            .expect("probe attach session")
    };

    let rtt_scan_region = if let Some(user_provided_addr) = opts.control_block_address {
        debug!(
            rtt_addr = user_provided_addr,
            "Using explicit RTT control block address"
        );
        ScanRegion::Exact(user_provided_addr.into())
    } else if let Some(elf_file) = opts.elf_file.as_ref() {
        debug!(elf_file = %elf_file.display(), "Reading ELF file");
        let mut file = fs::File::open(elf_file).expect("open elf file");
        if let Some(rtt_addr) = get_rtt_symbol(&mut file) {
            debug!(rtt_addr = rtt_addr, "Found RTT symbol");
            ScanRegion::Exact(rtt_addr as _)
        } else {
            session.target().rtt_scan_regions.clone()
        }
    } else {
        session.target().rtt_scan_regions.clone()
    };

    let mut core = session.core(opts.core).unwrap();

    let core_status = core.status().unwrap();
    debug!(status = ?core_status, "core status");

    if opts.reset {
        debug!("Reset and halt core");
        core.reset_and_halt(Duration::from_millis(100)).unwrap();
    }

    // Disable any previous vector catching (i.e. user just ran probe-rs run or a debugger)
    core.disable_vector_catch(VectorCatchCondition::All)
        .unwrap();
    core.clear_all_hw_breakpoints().unwrap();

    if let Some(bp_sym_or_addr) = opts.breakpoint.as_ref() {
        let num_bp = core.available_breakpoint_units().unwrap();
        assert!(num_bp > 0, "No breakpoints available");

        let bp_addr = if let Some(bp_addr) = bp_sym_or_addr
            .parse::<u64>()
            .ok()
            .or(u64::from_str_radix(bp_sym_or_addr.trim_start_matches("0x"), 16).ok())
        {
            bp_addr
        } else {
            let mut file = opts
                .elf_file
                .as_ref()
                .map(fs::File::open)
                .expect("open elf file")
                .expect("open elf file"); // TODO fixme
            let bp_addr = get_symbol(&mut file, bp_sym_or_addr).unwrap();
            if opts.thumb {
                bp_addr & !1
            } else {
                bp_addr
            }
        };

        debug!(
            available_breakpoints = num_bp,
            symbol_or_addr = bp_sym_or_addr,
            addr = format_args!("0x{:X}", bp_addr),
            "Setting breakpoint"
        );
        core.set_hw_breakpoint(bp_addr).unwrap();
    }

    let mut rtt = match opts.attach_timeout {
        Some(to) if !to.is_zero() => attach_retry_loop(&mut core, &rtt_scan_region, to).unwrap(),
        _ => {
            debug!("Attaching to RTT");
            Rtt::attach_region(&mut core, &rtt_scan_region).unwrap()
        }
    };

    let up_channel = rtt.up_channels.remove(opts.up_channel);
    let up_channel_mode = up_channel.mode(&mut core).unwrap();
    let up_channel_name = up_channel.name().unwrap_or("NA");
    debug!(channel = up_channel.number(), name = up_channel_name, mode = ?up_channel_mode, buffer_size = up_channel.buffer_size(), "Opened up channel");

    if opts.reset || opts.attach_under_reset {
        let sp_reg = core.stack_pointer();
        let sp: RegisterValue = core.read_core_reg(sp_reg.id()).unwrap();
        let pc_reg = core.program_counter();
        let pc: RegisterValue = core.read_core_reg(pc_reg.id()).unwrap();
        debug!(pc = %pc, sp = %sp, "Run core");
        core.run().unwrap();
    }

    if opts.breakpoint.is_some() {
        debug!("Waiting for breakpoint");
        'bp_loop: loop {
            if intr.is_set() {
                break;
            }

            match core.status().unwrap() {
                CoreStatus::Running => (),
                CoreStatus::Halted(halt_reason) => match halt_reason {
                    HaltReason::Breakpoint(_) => break 'bp_loop,
                    _ => {
                        warn!(reason = ?halt_reason, "Unexpected halt reason");
                        break 'bp_loop;
                    }
                },
                state => panic!("Core is in an unexpected state {state:?}"),
            }

            std::thread::sleep(Duration::from_millis(100));
        }

        if opts.blocking {
            let mode = ChannelMode::BlockIfFull;
            debug!(mode = ?mode, "Set channel mode");
            up_channel.set_mode(&mut core, mode).unwrap();
        }

        debug!("Run core post breakpoint");
        core.run().unwrap();
    }

    let mut metrics = Metrics::new(buffer.len());
    loop {
        if intr.is_set() {
            break;
        }
        let rtt_bytes_read = up_channel
            .read(&mut core, &mut buffer)
            .expect("RTT channel read");

        if rtt_bytes_read != 0 {
            trace!(bytes = rtt_bytes_read, "Writing RTT data");
            out_file.write_all(&buffer[..rtt_bytes_read]).unwrap();
        }

        // NOTE: this is what probe-rs does
        //
        // Poll RTT with a frequency of 10 Hz if we do not receive any new data.
        // Once we receive new data, we bump the frequency to 1kHz.
        //
        // If the polling frequency is too high, the USB connection to the probe
        // can become unstable. Hence we only pull as little as necessary.
        if rtt_bytes_read != 0 {
            std::thread::sleep(Duration::from_millis(1));
        } else {
            std::thread::sleep(Duration::from_millis(100));
        }

        metrics.update(rtt_bytes_read);
    }

    debug!("Shutting down");

    if opts.blocking {
        let mode = ChannelMode::NoBlockTrim;
        debug!(mode = ?mode, "Set channel mode");
        up_channel.set_mode(&mut core, mode).unwrap();
    }

    out_file.flush().unwrap();
}

fn get_rtt_symbol<T: io::Read + io::Seek>(file: &mut T) -> Option<u64> {
    get_symbol(file, "_SEGGER_RTT")
}

fn get_symbol<T: io::Read + io::Seek>(file: &mut T, symbol: &str) -> Option<u64> {
    let mut buffer = Vec::new();
    if file.read_to_end(&mut buffer).is_ok() {
        if let Ok(binary) = goblin::elf::Elf::parse(buffer.as_slice()) {
            for sym in &binary.syms {
                if let Some(name) = binary.strtab.get_at(sym.st_name) {
                    if name == symbol {
                        return Some(sym.st_value);
                    }
                }
            }
        }
    }
    None
}

fn attach_retry_loop(
    core: &mut Core,
    scan_region: &ScanRegion,
    timeout: humantime::Duration,
) -> Option<Rtt> {
    debug!(timeout = %timeout, "Attaching to RTT");
    let timeout: Duration = timeout.into();
    let start = Instant::now();
    while Instant::now().duration_since(start) <= timeout {
        match Rtt::attach_region(core, scan_region) {
            Ok(rtt) => return Some(rtt),
            Err(e) => {
                if matches!(e, probe_rs::rtt::Error::ControlBlockNotFound) {
                    std::thread::sleep(Duration::from_millis(50));
                    continue;
                }

                error!("Failed to attach to RTT");
                return None;
            }
        }
    }

    // Timeout reached
    warn!("Timed out attaching to RTT");
    Some(Rtt::attach(core).expect("RTT attach"))
}

#[derive(Clone, Debug)]
#[repr(transparent)]
pub struct Interruptor(Arc<AtomicBool>);

impl Interruptor {
    pub fn new() -> Self {
        Interruptor(Arc::new(AtomicBool::new(false)))
    }

    pub fn set(&self) {
        self.0.store(true, SeqCst);
    }

    pub fn is_set(&self) -> bool {
        self.0.load(SeqCst)
    }
}

impl Default for Interruptor {
    fn default() -> Self {
        Self::new()
    }
}

struct Metrics {
    rtt_buffer_size: u64,
    window_start: Instant,
    read_cnt: u64,
    bytes_read: u64,
    read_zero_cnt: u64,
    read_max_cnt: u64,
    sma: NoSumSMA<f64, f64, 8>,
}

impl Metrics {
    const WINDOW_DURATION: Duration = Duration::from_secs(2);

    fn new(host_rtt_buffer_size: usize) -> Self {
        Self {
            rtt_buffer_size: host_rtt_buffer_size as u64,
            window_start: Instant::now(),
            read_cnt: 0,
            bytes_read: 0,
            read_zero_cnt: 0,
            read_max_cnt: 0,
            sma: NoSumSMA::new(),
        }
    }

    fn reset(&mut self) {
        self.read_cnt = 0;
        self.bytes_read = 0;
        self.read_zero_cnt = 0;
        self.read_max_cnt = 0;

        self.window_start = Instant::now();
    }

    fn update(&mut self, bytes_read: usize) {
        let dur = Instant::now().duration_since(self.window_start);

        self.read_cnt += 1;
        self.bytes_read += bytes_read as u64;
        if bytes_read == 0 {
            self.read_zero_cnt += 1;
        } else {
            if bytes_read as u64 == self.rtt_buffer_size {
                self.read_max_cnt += 1;
            }
            self.sma.add_sample(bytes_read as f64);
        }

        if dur >= Self::WINDOW_DURATION {
            let bytes = self.bytes_read as f64;
            let secs = dur.as_secs_f64();

            info!(
                transfer_rate = format!("{}/s", human_bytes(bytes / secs)),
                cnt = self.read_cnt,
                zero_cnt = self.read_zero_cnt,
                max_cnt = self.read_max_cnt,
                avg = self.sma.get_average(),
            );

            self.reset();
        }
    }
}
