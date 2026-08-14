use anyhow::{bail, Context};
use rne_physics_conformance::run_divergence_diagnostic;
use std::path::{Path, PathBuf};

fn main() -> anyhow::Result<()> {
    let Some((report_path, replay_path)) = parse_args()? else {
        return Ok(());
    };
    anyhow::ensure!(
        !report_path.exists() && !replay_path.exists(),
        "refusing to overwrite diagnostic output"
    );
    let (report, replay) = run_divergence_diagnostic()?;
    anyhow::ensure!(
        !report.all_passed(),
        "diagnostic report must contain the injected failure"
    );

    write_new(
        &report_path,
        &format!("{}\n", serde_json::to_string_pretty(&report)?),
    )?;
    write_new(&replay_path, &format!("{}\n", replay.to_json_pretty()?))?;
    println!("wrote {}", report_path.display());
    println!("wrote {}", replay_path.display());
    Ok(())
}

fn parse_args() -> anyhow::Result<Option<(PathBuf, PathBuf)>> {
    let mut args = std::env::args().skip(1);
    let mut report = PathBuf::from("artifacts/physics-divergence/conformance-report.json");
    let mut replay = PathBuf::from("artifacts/physics-divergence/divergence.rne-replay");
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--report" => report = PathBuf::from(args.next().context("--report requires a path")?),
            "--replay" => replay = PathBuf::from(args.next().context("--replay requires a path")?),
            "--help" | "-h" => {
                println!(
                    "rne-physics-divergence [--report PATH] [--replay PATH]\n\
                     writes a deliberately failing Rapier-vs-MuJoCo diagnostic pair"
                );
                return Ok(None);
            }
            other => bail!("unknown argument `{other}`"),
        }
    }
    anyhow::ensure!(report != replay, "report and replay paths must differ");
    Ok(Some((report, replay)))
}

fn write_new(path: &Path, contents: &str) -> anyhow::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("create output directory {}", parent.display()))?;
    }
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create_new(true);
    let mut file = options
        .open(path)
        .with_context(|| format!("refusing to overwrite output {}", path.display()))?;
    std::io::Write::write_all(&mut file, contents.as_bytes())
        .with_context(|| format!("write {}", path.display()))?;
    Ok(())
}
