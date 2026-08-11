use anyhow::{bail, Context};
use rne_physics_conformance::run_conformance;
use std::path::PathBuf;

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let output = match (args.next().as_deref(), args.next()) {
        (None, None) => PathBuf::from("artifacts/physics-conformance/report.json"),
        (Some("--output"), Some(path)) => PathBuf::from(path),
        _ => bail!("usage: rne-physics-conformance [--output PATH]"),
    };
    if args.next().is_some() {
        bail!("usage: rne-physics-conformance [--output PATH]");
    }

    let report = run_conformance();
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("create report directory {}", parent.display()))?;
    }
    let mut json = serde_json::to_string_pretty(&report)?;
    json.push('\n');
    std::fs::write(&output, json)
        .with_context(|| format!("write conformance report {}", output.display()))?;
    println!("wrote {}", output.display());
    if !report.all_passed() {
        bail!("physics conformance failed; inspect {}", output.display());
    }
    Ok(())
}
