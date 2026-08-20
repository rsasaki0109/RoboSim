//! Installed compatibility-corpus verifier.

use anyhow::{bail, Context};
use rne_compatibility_suite::{run_compatibility, write_report};
use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("rne-compatibility error: {error:#}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> anyhow::Result<()> {
    let mut root = PathBuf::from(".");
    let mut registry = PathBuf::from("release/compatibility-fixtures.toml");
    let mut output = PathBuf::from("artifacts/compatibility/report.json");
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--root" => {
                root = PathBuf::from(args.next().context("--root requires a path")?);
            }
            "--registry" => {
                registry = PathBuf::from(args.next().context("--registry requires a path")?);
            }
            "--output" => {
                output = PathBuf::from(args.next().context("--output requires a path")?);
            }
            other => bail!("unknown argument: {other}"),
        }
    }

    let registry = if registry.is_absolute() {
        registry
    } else {
        root.join(registry)
    };
    let output = if output.is_absolute() {
        output
    } else {
        root.join(output)
    };
    let report = run_compatibility(&root, &registry)?;
    write_report(&report, &output)?;
    println!(
        "compatibility fixtures: status={} checks={} report={}",
        if report.passed { "passed" } else { "failed" },
        report.checks.len(),
        output.display()
    );
    if !report.passed {
        bail!("one or more compatibility fixtures failed");
    }
    Ok(())
}
