//! Emits depth and false-occlusion evidence over two registered real cameras.

use rne_render::validate_gaussian_splat_manifest;
use rne_render_3dgs::validate_registered_splat_multiview_depth;
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn Error>> {
    let arguments = env::args().skip(1).collect::<Vec<_>>();
    let manifest = PathBuf::from(required_argument(&arguments, "--manifest")?);
    let tracks = PathBuf::from(required_argument(&arguments, "--tracks")?);
    let output = PathBuf::from(required_argument(&arguments, "--output")?);
    let environment = validate_gaussian_splat_manifest(&manifest)?;
    let report = validate_registered_splat_multiview_depth(&environment, &tracks)?;
    fs::write(&output, serde_json::to_string_pretty(&report)? + "\n")?;
    println!(
        "wrote {}: matched={}/{} depth_delta_mae={} false_occlusion={}/{} passed={}",
        output.display(),
        report.matched_track_count,
        report.tracks.len(),
        report
            .depth_delta_mae_source_units
            .map_or_else(|| "none".into(), |value| format!("{value:.9}")),
        report.false_occlusion_view_count,
        report.matched_view_count,
        report.passed,
    );
    Ok(())
}

fn required_argument<'a>(arguments: &'a [String], name: &str) -> Result<&'a str, Box<dyn Error>> {
    let index = arguments
        .iter()
        .position(|argument| argument == name)
        .ok_or_else(|| format!("missing required {name}"))?;
    arguments
        .get(index + 1)
        .map(String::as_str)
        .ok_or_else(|| format!("missing value for {name}").into())
}
