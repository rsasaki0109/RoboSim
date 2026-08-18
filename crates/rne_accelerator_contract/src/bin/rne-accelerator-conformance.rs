//! Standalone CLI for external accelerator process conformance.

use rne_accelerator_contract::{
    run_accelerator_process_conformance, scaffold_accelerator_adapter_for_schema,
    AcceleratorProcessConformanceConfig, ACCELERATOR_SCAFFOLD_SCHEMA_VERSION,
};
use std::ffi::OsString;
use std::path::PathBuf;

const USAGE: &str = "rne-accelerator-conformance \
  --adapter PROGRAM [--adapter-arg ARG]... [--subject FILE] \
  --manifest accelerator.toml --runtime runtime.toml --task task.json \
  [--timeout-ms N] [--output REPORT.json]\n\
rne-accelerator-conformance scaffold NAME [--dir PARENT] [--schema 1]";

fn main() {
    if let Err(error) = run() {
        eprintln!("accelerator process conformance failed: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let arguments = std::env::args().skip(1).collect::<Vec<_>>();
    if arguments.first().map(String::as_str) == Some("scaffold") {
        return run_scaffold(arguments.into_iter().skip(1));
    }
    let options = parse_args(arguments.into_iter())?;
    let mut config = AcceleratorProcessConformanceConfig::new(&options.adapter);
    config.arguments = options.adapter_args;
    config.subject = options.subject.unwrap_or_else(|| options.adapter.clone());
    config.response_timeout_ms = options.timeout_ms;
    let report = run_accelerator_process_conformance(
        &options.manifest,
        &options.runtime,
        &options.task,
        &config,
    )?;
    if let Some(parent) = options
        .output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&options.output, report.to_json_pretty()?)?;
    println!(
        "accelerator process conformance: status={} checks={}/{} report={}",
        report.status,
        report
            .checks
            .iter()
            .filter(|check| check.status == "passed")
            .count(),
        report.checks.len(),
        options.output.display()
    );
    if !report.passed() {
        return Err(format!("adapter did not pass; inspect {}", options.output.display()).into());
    }
    Ok(())
}

fn run_scaffold(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn std::error::Error>> {
    let name = required(&mut args, "scaffold")?;
    if matches!(name.as_str(), "--help" | "-h") {
        println!("{USAGE}");
        return Ok(());
    }
    let mut parent = PathBuf::from("accelerators");
    let mut parent_was_set = false;
    let mut schema_version = ACCELERATOR_SCAFFOLD_SCHEMA_VERSION;
    let mut schema_was_set = false;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--dir" if !parent_was_set => {
                parent = PathBuf::from(required(&mut args, "--dir")?);
                parent_was_set = true;
            }
            "--dir" => return Err("--dir may only be specified once".into()),
            "--schema" if !schema_was_set => {
                let value = required(&mut args, "--schema")?;
                schema_version = value
                    .parse::<u32>()
                    .map_err(|error| format!("invalid --schema {value:?}: {error}"))?;
                schema_was_set = true;
            }
            "--schema" => return Err("--schema may only be specified once".into()),
            "--help" | "-h" => {
                println!("{USAGE}");
                return Ok(());
            }
            other => {
                return Err(format!("unknown scaffold argument {other:?}; usage: {USAGE}").into())
            }
        }
    }
    let directory = scaffold_accelerator_adapter_for_schema(&name, &parent, schema_version)?;
    println!(
        "created accelerator adapter scaffold {}",
        directory.display()
    );
    Ok(())
}

#[derive(Debug)]
struct Options {
    adapter: PathBuf,
    adapter_args: Vec<OsString>,
    subject: Option<PathBuf>,
    manifest: PathBuf,
    runtime: PathBuf,
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
    let mut manifest = None;
    let mut runtime = None;
    let mut task = None;
    let mut timeout_ms = 5_000;
    let mut output = PathBuf::from("artifacts/accelerator-conformance/report.json");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--adapter" => adapter = Some(PathBuf::from(required(&mut args, "--adapter")?)),
            "--adapter-arg" => {
                adapter_args.push(OsString::from(required(&mut args, "--adapter-arg")?))
            }
            "--subject" => subject = Some(PathBuf::from(required(&mut args, "--subject")?)),
            "--manifest" => manifest = Some(PathBuf::from(required(&mut args, "--manifest")?)),
            "--runtime" => runtime = Some(PathBuf::from(required(&mut args, "--runtime")?)),
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
        manifest: manifest.ok_or_else(|| format!("--manifest is required; usage: {USAGE}"))?,
        runtime: runtime.ok_or_else(|| format!("--runtime is required; usage: {USAGE}"))?,
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
    fn parser_requires_all_contracts_and_preserves_adapter_arguments() {
        let options = parse_args(
            [
                "--adapter",
                "python",
                "--adapter-arg",
                "adapter.py",
                "--subject",
                "adapter.py",
                "--manifest",
                "accelerator.toml",
                "--runtime",
                "runtime.toml",
                "--task",
                "task.json",
            ]
            .into_iter()
            .map(str::to_string),
        )
        .unwrap();
        assert_eq!(options.adapter_args, vec![OsString::from("adapter.py")]);
        assert_eq!(options.subject, Some(PathBuf::from("adapter.py")));
        assert_eq!(options.timeout_ms, 5_000);
    }

    #[test]
    fn scaffold_parser_rejects_repeated_output_directory() {
        let error = run_scaffold(
            ["example", "--dir", "first", "--dir", "second"]
                .into_iter()
                .map(str::to_string),
        )
        .unwrap_err();
        assert_eq!(error.to_string(), "--dir may only be specified once");

        let error = run_scaffold(
            ["example", "--schema", "1", "--schema", "1"]
                .into_iter()
                .map(str::to_string),
        )
        .unwrap_err();
        assert_eq!(error.to_string(), "--schema may only be specified once");
    }
}
