//! Standalone CLI for external hardware adapter process conformance.

use rne_hardware_gateway::conformance::{
    run_hardware_adapter_conformance, HardwareAdapterConformanceConfig,
};
use std::ffi::OsString;
use std::path::PathBuf;

const USAGE: &str = "rne-hardware-conformance \
  --adapter PROGRAM [--adapter-arg ARG]... [--subject FILE] \
  --task TASK.json --allow-hil [--timeout-ms N] [--output REPORT.json]";

fn main() {
    if let Err(error) = run() {
        eprintln!("hardware adapter conformance failed: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let options = parse_args(std::env::args().skip(1))?;
    let mut config = HardwareAdapterConformanceConfig::new(&options.adapter);
    config.arguments = options.adapter_args;
    config.subject = options.subject.unwrap_or_else(|| options.adapter.clone());
    config.response_timeout_ms = options.timeout_ms;
    config.allow_hil = options.allow_hil;
    let report = run_hardware_adapter_conformance(&options.task, &config)?;
    if let Some(parent) = options
        .output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&options.output, report.to_json_pretty()?)?;
    println!(
        "hardware adapter conformance: status={} report={}",
        report.status,
        options.output.display()
    );
    if !report.passed() {
        return Err(format!("adapter did not pass; inspect {}", options.output.display()).into());
    }
    Ok(())
}

#[derive(Debug)]
struct Options {
    adapter: PathBuf,
    adapter_args: Vec<OsString>,
    subject: Option<PathBuf>,
    task: PathBuf,
    allow_hil: bool,
    timeout_ms: u64,
    output: PathBuf,
}

fn parse_args(
    mut args: impl Iterator<Item = String>,
) -> Result<Options, Box<dyn std::error::Error>> {
    let mut adapter = None;
    let mut adapter_args = Vec::new();
    let mut subject = None;
    let mut task = None;
    let mut allow_hil = false;
    let mut timeout_ms = 5_000;
    let mut output = PathBuf::from("artifacts/hardware-adapter-conformance/report.json");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--adapter" => adapter = Some(PathBuf::from(required(&mut args, "--adapter")?)),
            "--adapter-arg" => {
                adapter_args.push(OsString::from(required(&mut args, "--adapter-arg")?))
            }
            "--subject" => subject = Some(PathBuf::from(required(&mut args, "--subject")?)),
            "--task" => task = Some(PathBuf::from(required(&mut args, "--task")?)),
            "--allow-hil" => allow_hil = true,
            "--timeout-ms" => {
                let value = required(&mut args, "--timeout-ms")?;
                timeout_ms = value
                    .parse::<u64>()
                    .map_err(|error| format!("invalid --timeout-ms {value:?}: {error}"))?;
            }
            "--output" => output = PathBuf::from(required(&mut args, "--output")?),
            "--help" | "-h" => {
                println!("{USAGE}");
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument {other:?}; usage: {USAGE}").into()),
        }
    }
    Ok(Options {
        adapter: adapter.ok_or_else(|| format!("--adapter is required; usage: {USAGE}"))?,
        adapter_args,
        subject,
        task: task.ok_or_else(|| format!("--task is required; usage: {USAGE}"))?,
        allow_hil,
        timeout_ms,
        output,
    })
}

fn required(
    args: &mut impl Iterator<Item = String>,
    option: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    args.next()
        .filter(|value| !value.is_empty())
        .ok_or_else(|| format!("{option} requires a value").into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parser_requires_explicit_hil_but_preserves_adapter_arguments() {
        let options = parse_args(
            [
                "--adapter",
                "adapter",
                "--adapter-arg",
                "--mock",
                "--subject",
                "adapter.py",
                "--task",
                "task.json",
                "--allow-hil",
            ]
            .into_iter()
            .map(str::to_string),
        )
        .unwrap();
        assert_eq!(options.adapter_args, vec![OsString::from("--mock")]);
        assert!(options.allow_hil);
        assert_eq!(options.subject, Some(PathBuf::from("adapter.py")));
    }
}
