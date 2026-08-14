//! Runs one bounded LeKiwi bridge session and writes correlated evidence.

use rne_hardware_gateway::wire::{
    DeviceWireFrame, HardwareWireCodec, HardwareWireTraceOutcome, HostWireFrame,
};
use rne_hardware_gateway::HardwareMode;
use rne_hardware_lekiwi::session::{
    LeKiwiMonotonicClock, LeKiwiReferenceSampleOutcome, LeKiwiReferenceSessionConfig,
    LeKiwiReferenceSessionEvidence, LeKiwiReferenceSessionRunner, LeKiwiTransportError,
    LeKiwiWireTransport,
};
use std::ffi::OsString;
use std::fs::OpenOptions;
use std::io::{BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const USAGE: &str = "usage: rne-lekiwi-session --output PATH [--mock | --physical-session] \
    [--session-id ID] [--mode shadow|hil|live] [--samples N] \
    [--action-vx-m-s V] [--action-vy-m-s V] [--action-wz-rad-s V] \
    [--sample-period-ms N] [--response-timeout-ms N] [--python PATH] \
    [--bridge PATH] [--robot-id ID] [--port PATH] [--allow-actuation]";

fn main() {
    if std::env::args()
        .skip(1)
        .any(|argument| argument == "--help")
    {
        println!("{USAGE}");
        return;
    }
    match run() {
        Ok((path, evidence)) => {
            println!(
                "wrote {} ({}, {:?})",
                path.display(),
                evidence.device_id,
                evidence.session.wire_trace.outcome
            );
            if !matches!(
                evidence.session.wire_trace.outcome,
                HardwareWireTraceOutcome::Completed
            ) {
                std::process::exit(3);
            }
        }
        Err(error) => {
            eprintln!("rne-lekiwi-session: {error}");
            eprintln!("{USAGE}");
            std::process::exit(1);
        }
    }
}

fn run() -> Result<(PathBuf, LeKiwiReferenceSessionEvidence), String> {
    let args = CliArgs::parse(std::env::args_os().skip(1))?;
    let python = args.python.clone().map(Ok).unwrap_or_else(find_python)?;
    let mut bridge_args = Vec::<OsString>::new();
    if args.mock_device {
        bridge_args.push("--mock".into());
    } else {
        bridge_args.extend([
            "--robot-id".into(),
            args.robot_id.clone().into(),
            "--port".into(),
            args.port.clone().into(),
        ]);
    }
    let transport = ProcessTransport::spawn(
        &python,
        &args.bridge,
        &bridge_args,
        Duration::from_millis(args.response_timeout_ms),
    )?;
    let clock = InstantClock::new();
    let config =
        LeKiwiReferenceSessionConfig::new(args.session_id.clone(), args.mode, args.samples);
    let mut runner = LeKiwiReferenceSessionRunner::new(transport, clock, config)
        .map_err(|error| error.to_string())?;
    runner.open().map_err(|error| error.to_string())?;

    let period = Duration::from_millis(args.sample_period_ms);
    let mut next_sample = Instant::now();
    let mut terminal = None;
    for sample_index in 0..args.samples {
        if sample_index > 0 {
            sleep_until(next_sample);
        }
        match runner
            .sample(args.action.to_vec())
            .map_err(|error| error.to_string())?
        {
            LeKiwiReferenceSampleOutcome::Sample(_) => {}
            LeKiwiReferenceSampleOutcome::Terminal(evidence) => {
                terminal = Some(*evidence);
                break;
            }
        }
        next_sample = next_sample
            .checked_add(period)
            .ok_or_else(|| "sample deadline overflow".to_string())?;
    }
    let evidence = match terminal {
        Some(evidence) => evidence,
        None => runner.close().map_err(|error| error.to_string())?,
    };
    evidence.validate().map_err(|error| error.to_string())?;
    write_evidence(&args.output, &evidence)?;
    Ok((args.output, evidence))
}

#[derive(Debug)]
struct CliArgs {
    output: PathBuf,
    mock_device: bool,
    session_id: String,
    mode: HardwareMode,
    samples: usize,
    action: [f64; 3],
    sample_period_ms: u64,
    response_timeout_ms: u64,
    python: Option<PathBuf>,
    bridge: PathBuf,
    robot_id: String,
    port: String,
}

impl CliArgs {
    fn parse(arguments: impl Iterator<Item = OsString>) -> Result<Self, String> {
        let arguments = arguments.collect::<Vec<_>>();
        let mut index = 0;
        let mut output = None;
        let mut mock_device = false;
        let mut physical_session = false;
        let mut allow_actuation = false;
        let mut session_id = None;
        let mut mode = HardwareMode::Shadow;
        let mut samples = 3_usize;
        let mut action = [0.0_f64; 3];
        let mut sample_period_ms = 34_u64;
        let mut response_timeout_ms = 2_000_u64;
        let mut python = None;
        let mut bridge =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("python/rne_hardware_lekiwi_device.py");
        let mut robot_id = None;
        let mut port = "/dev/ttyACM0".to_string();

        while index < arguments.len() {
            let flag = arguments[index]
                .to_str()
                .ok_or_else(|| "arguments must be valid UTF-8".to_string())?;
            index += 1;
            match flag {
                "--output" => output = Some(PathBuf::from(take(&arguments, &mut index, flag)?)),
                "--mock" => mock_device = true,
                "--physical-session" => physical_session = true,
                "--allow-actuation" => allow_actuation = true,
                "--session-id" => session_id = Some(take_string(&arguments, &mut index, flag)?),
                "--mode" => {
                    mode = match take_string(&arguments, &mut index, flag)?.as_str() {
                        "shadow" => HardwareMode::Shadow,
                        "hil" => HardwareMode::Hil,
                        "live" => HardwareMode::Live,
                        actual => return Err(format!("unsupported --mode {actual:?}")),
                    }
                }
                "--samples" => samples = take_parse(&arguments, &mut index, flag)?,
                "--action-vx-m-s" => action[0] = take_parse(&arguments, &mut index, flag)?,
                "--action-vy-m-s" => action[1] = take_parse(&arguments, &mut index, flag)?,
                "--action-wz-rad-s" => action[2] = take_parse(&arguments, &mut index, flag)?,
                "--sample-period-ms" => {
                    sample_period_ms = take_parse(&arguments, &mut index, flag)?
                }
                "--response-timeout-ms" => {
                    response_timeout_ms = take_parse(&arguments, &mut index, flag)?
                }
                "--python" => python = Some(PathBuf::from(take(&arguments, &mut index, flag)?)),
                "--bridge" => bridge = PathBuf::from(take(&arguments, &mut index, flag)?),
                "--robot-id" => robot_id = Some(take_string(&arguments, &mut index, flag)?),
                "--port" => port = take_string(&arguments, &mut index, flag)?,
                actual => return Err(format!("unknown argument {actual:?}")),
            }
        }

        if mock_device == physical_session {
            return Err("select exactly one of --mock or --physical-session".to_string());
        }
        if !mock_device
            && matches!(mode, HardwareMode::Hil | HardwareMode::Live)
            && !allow_actuation
        {
            return Err("physical HIL/live requires --allow-actuation".to_string());
        }
        if samples == 0 {
            return Err("--samples must be greater than zero".to_string());
        }
        if sample_period_ms == 0 || response_timeout_ms == 0 {
            return Err("sample period and response timeout must be greater than zero".to_string());
        }
        if action.iter().any(|value| !value.is_finite()) {
            return Err("action values must be finite".to_string());
        }
        let output = output.ok_or_else(|| "--output is required".to_string())?;
        if output.exists() {
            return Err(format!(
                "refusing to overwrite existing evidence {}",
                output.display()
            ));
        }
        if !bridge.is_file() {
            return Err(format!("bridge script not found: {}", bridge.display()));
        }
        let session_id = match session_id {
            Some(session_id) => session_id,
            None if mock_device => "rne.lekiwi.mock.session.v1".to_string(),
            None => return Err("physical sessions require --session-id".to_string()),
        };
        let robot_id = match robot_id {
            Some(robot_id) if !robot_id.trim().is_empty() => robot_id,
            Some(_) => return Err("--robot-id must not be empty".to_string()),
            None if mock_device => "mock".to_string(),
            None => return Err("physical sessions require --robot-id".to_string()),
        };
        Ok(Self {
            output,
            mock_device,
            session_id,
            mode,
            samples,
            action,
            sample_period_ms,
            response_timeout_ms,
            python,
            bridge,
            robot_id,
            port,
        })
    }
}

fn take<'a>(
    arguments: &'a [OsString],
    index: &mut usize,
    flag: &str,
) -> Result<&'a OsString, String> {
    let value = arguments
        .get(*index)
        .ok_or_else(|| format!("{flag} requires a value"))?;
    *index += 1;
    Ok(value)
}

fn take_string(arguments: &[OsString], index: &mut usize, flag: &str) -> Result<String, String> {
    take(arguments, index, flag)?
        .to_str()
        .map(ToOwned::to_owned)
        .ok_or_else(|| format!("{flag} must be valid UTF-8"))
}

fn take_parse<T>(arguments: &[OsString], index: &mut usize, flag: &str) -> Result<T, String>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    let value = take_string(arguments, index, flag)?;
    value
        .parse()
        .map_err(|error| format!("invalid {flag} value {value:?}: {error}"))
}

#[derive(Debug)]
struct InstantClock {
    origin: Instant,
}

impl InstantClock {
    fn new() -> Self {
        Self {
            origin: Instant::now(),
        }
    }
}

impl LeKiwiMonotonicClock for InstantClock {
    fn now_ms(&mut self) -> u64 {
        u64::try_from(self.origin.elapsed().as_millis()).unwrap_or(u64::MAX)
    }
}

#[derive(Debug)]
struct ProcessTransport {
    child: Child,
    stdin: Option<ChildStdin>,
    responses: Receiver<Result<DeviceWireFrame, String>>,
    reader: Option<JoinHandle<()>>,
    codec: HardwareWireCodec,
    response_timeout: Duration,
}

impl ProcessTransport {
    fn spawn(
        python: &Path,
        bridge: &Path,
        arguments: &[OsString],
        response_timeout: Duration,
    ) -> Result<Self, String> {
        let mut child = Command::new(python)
            .arg(bridge)
            .args(arguments)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|error| format!("failed to spawn {}: {error}", python.display()))?;
        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| "LeKiwi bridge stdin was not piped".to_string())?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| "LeKiwi bridge stdout was not piped".to_string())?;
        let (sender, responses) = mpsc::sync_channel(1);
        let reader = std::thread::Builder::new()
            .name("rne-lekiwi-wire-reader".to_string())
            .spawn(move || {
                let codec = HardwareWireCodec::default();
                let mut stdout = BufReader::new(stdout);
                loop {
                    let result = match codec.read_line(&mut stdout) {
                        Ok(Some(line)) => codec
                            .decode_device(&line)
                            .map_err(|error| error.to_string()),
                        Ok(None) => Err("LeKiwi bridge closed stdout".to_string()),
                        Err(error) => Err(error.to_string()),
                    };
                    let terminal = result.is_err();
                    if sender.send(result).is_err() || terminal {
                        break;
                    }
                }
            });
        let reader = match reader {
            Ok(reader) => reader,
            Err(error) => {
                drop(stdin);
                let _ = child.kill();
                let _ = child.wait();
                return Err(format!("failed to start bridge reader: {error}"));
            }
        };
        Ok(Self {
            child,
            stdin: Some(stdin),
            responses,
            reader: Some(reader),
            codec: HardwareWireCodec::default(),
            response_timeout,
        })
    }
}

impl LeKiwiWireTransport for ProcessTransport {
    fn exchange(
        &mut self,
        request: &HostWireFrame,
    ) -> Result<DeviceWireFrame, LeKiwiTransportError> {
        let encoded = self
            .codec
            .encode_host(request)
            .map_err(|error| LeKiwiTransportError::new(error.to_string()))?;
        let stdin = self
            .stdin
            .as_mut()
            .ok_or_else(|| LeKiwiTransportError::new("LeKiwi bridge stdin is closed"))?;
        stdin
            .write_all(&encoded)
            .and_then(|()| stdin.flush())
            .map_err(|error| LeKiwiTransportError::new(format!("bridge write failed: {error}")))?;
        match self.responses.recv_timeout(self.response_timeout) {
            Ok(Ok(response)) => Ok(response),
            Ok(Err(error)) => Err(LeKiwiTransportError::new(error)),
            Err(RecvTimeoutError::Timeout) => Err(LeKiwiTransportError::new(format!(
                "bridge response timed out after {} ms",
                self.response_timeout.as_millis()
            ))),
            Err(RecvTimeoutError::Disconnected) => {
                Err(LeKiwiTransportError::new("LeKiwi bridge reader terminated"))
            }
        }
    }
}

impl Drop for ProcessTransport {
    fn drop(&mut self) {
        self.stdin.take();
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            match self.child.try_wait() {
                Ok(Some(_)) => break,
                Ok(None) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(10));
                }
                _ => {
                    let _ = self.child.kill();
                    let _ = self.child.wait();
                    break;
                }
            }
        }
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

fn find_python() -> Result<PathBuf, String> {
    let candidates = std::env::var_os("PYTHON")
        .map(PathBuf::from)
        .into_iter()
        .chain([PathBuf::from("python3"), PathBuf::from("python")]);
    for candidate in candidates {
        let status = Command::new(&candidate)
            .arg("--version")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        if status.is_ok_and(|status| status.success()) {
            return Ok(candidate);
        }
    }
    Err("Python 3 was not found; pass --python PATH".to_string())
}

fn sleep_until(deadline: Instant) {
    if let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
        std::thread::sleep(remaining);
    }
}

fn write_evidence(path: &Path, evidence: &LeKiwiReferenceSessionEvidence) -> Result<(), String> {
    let parent = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    if !parent.is_dir() {
        return Err(format!(
            "output directory does not exist: {}",
            parent.display()
        ));
    }
    let name = path
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| "output must have a UTF-8 file name".to_string())?;
    let temporary = parent.join(format!(".{name}.{}.tmp", std::process::id()));
    let result = (|| {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temporary)
            .map_err(|error| format!("failed to create {}: {error}", temporary.display()))?;
        serde_json::to_writer_pretty(&mut file, evidence)
            .map_err(|error| format!("failed to serialize evidence: {error}"))?;
        file.write_all(b"\n")
            .and_then(|()| file.sync_all())
            .map_err(|error| format!("failed to flush evidence: {error}"))?;
        drop(file);
        std::fs::rename(&temporary, path).map_err(|error| {
            format!(
                "failed to publish {} as {}: {error}",
                temporary.display(),
                path.display()
            )
        })
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&temporary);
    }
    result
}
