use anyhow::{bail, ensure, Context, Result};
use rne_mobility_benchmark::run_mobility_benchmark;
use std::path::PathBuf;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut output = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output" => {
                output = Some(PathBuf::from(
                    args.next().context("--output requires a path")?,
                ));
            }
            other => bail!("unknown argument: {other}"),
        }
    }
    let report = run_mobility_benchmark()?;
    ensure!(report.passed, "mobility benchmark verdict failed");
    let json = serde_json::to_string_pretty(&report)? + "\n";
    if let Some(path) = output {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("create {}", parent.display()))?;
        }
        std::fs::write(&path, json).with_context(|| format!("write {}", path.display()))?;
        println!("mobility benchmark passed: {}", path.display());
    } else {
        print!("{json}");
    }
    Ok(())
}
