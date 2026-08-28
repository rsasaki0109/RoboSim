//! Emits content-bound sparse-depth evidence for a registered 3DGS camera.

use rne_render::validate_gaussian_splat_manifest;
use rne_render_3dgs::validate_registered_splat_depth;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn Error>> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    let manifest = PathBuf::from(required_argument(&arguments, "--manifest")?);
    let camera_id = required_argument(&arguments, "--camera")?;
    let output = PathBuf::from(required_argument(&arguments, "--output")?);
    let environment = validate_gaussian_splat_manifest(&manifest)?;
    let fixture = environment
        .validation_fixture_path
        .as_ref()
        .ok_or("manifest has no validation fixture")?;
    let report = validate_registered_splat_depth(&environment, fixture, camera_id)?;
    let encoded = serde_json::to_string_pretty(&report)? + "\n";
    fs::write(&output, encoded)?;
    println!(
        "wrote {}: matched={}/{} mae={} max={} metric_qualified={}",
        output.display(),
        report.matched_landmarks,
        report.landmarks.len(),
        report
            .mean_absolute_error_source_units
            .map_or_else(|| "none".into(), |value| format!("{value:.9}")),
        report
            .max_absolute_error_source_units
            .map_or_else(|| "none".into(), |value| format!("{value:.9}")),
        report.metric_qualified,
    );
    Ok(())
}

fn required_argument<'a>(arguments: &'a [String], name: &str) -> Result<&'a str, Box<dyn Error>> {
    let index = arguments
        .iter()
        .position(|argument| argument == name)
        .ok_or_else(|| format!("missing required {name}"))?;
    let value = arguments
        .get(index + 1)
        .ok_or_else(|| format!("missing value for {name}"))?;
    Ok(value)
}
