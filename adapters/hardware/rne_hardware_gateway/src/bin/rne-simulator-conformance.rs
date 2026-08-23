//! Standalone CLI for external fixed-step simulator adapter conformance.

use rne_hardware_gateway::simulator::conformance::{
    run_simulator_adapter_conformance, SimulatorAdapterConformanceConfig,
};
use std::ffi::OsString;
use std::path::PathBuf;

const USAGE: &str = "rne-simulator-conformance \
  --adapter PROGRAM [--adapter-arg ARG]... [--subject FILE] \
  --runtime-manifest RUNTIME.json --task TASK.json \
  [--timeout-ms N] [--output REPORT.json]";

fn main() {
    if let Err(error) = run() {
        eprintln!("simulator adapter conformance failed: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let options = parse_args(std::env::args().skip(1))?;
    let mut config =
        SimulatorAdapterConformanceConfig::new(&options.adapter, &options.runtime_manifest);
    config.arguments = options.adapter_args;
    config.subject = options.subject.unwrap_or_else(|| options.adapter.clone());
    config.response_timeout_ms = options.timeout_ms;
    let report = run_simulator_adapter_conformance(&options.task, &config)?;
    if let Some(parent) = options
        .output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&options.output, report.to_json_pretty()?)?;
    println!(
        "simulator adapter conformance: status={} report={}",
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
    runtime_manifest: PathBuf,
    task: PathBuf,
    timeout_ms: u64,
    output: PathBuf,
}

fn parse_args(
    mut args: impl Iterator<Item = String>,
) -> Result<Options, Box<dyn std::error::Error>> {
    let mut adapter = None;
    let mut adapter_args = Vec::new();
    let mut subject = None;
    let mut runtime_manifest = None;
    let mut task = None;
    let mut timeout_ms = 5_000;
    let mut output = PathBuf::from("artifacts/simulator-adapter-conformance/report.json");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--adapter" => adapter = Some(PathBuf::from(required(&mut args, "--adapter")?)),
            "--adapter-arg" => {
                adapter_args.push(OsString::from(required(&mut args, "--adapter-arg")?))
            }
            "--subject" => subject = Some(PathBuf::from(required(&mut args, "--subject")?)),
            "--runtime-manifest" => {
                runtime_manifest = Some(PathBuf::from(required(&mut args, "--runtime-manifest")?))
            }
            "--task" => task = Some(PathBuf::from(required(&mut args, "--task")?)),
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
        runtime_manifest: runtime_manifest
            .ok_or_else(|| format!("--runtime-manifest is required; usage: {USAGE}"))?,
        task: task.ok_or_else(|| format!("--task is required; usage: {USAGE}"))?,
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
    fn parser_preserves_adapter_arguments_and_runtime_manifest() {
        let options = parse_args(
            [
                "--adapter",
                "adapter",
                "--adapter-arg",
                "--world",
                "--subject",
                "adapter.py",
                "--runtime-manifest",
                "runtime.json",
                "--task",
                "task.json",
            ]
            .into_iter()
            .map(str::to_string),
        )
        .unwrap();
        assert_eq!(options.adapter_args, vec![OsString::from("--world")]);
        assert_eq!(options.runtime_manifest, PathBuf::from("runtime.json"));
    }
}
