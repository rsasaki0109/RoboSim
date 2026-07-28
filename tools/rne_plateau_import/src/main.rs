//! Command-line entry point for deterministic PLATEAU CityGML conversion.

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use rne_plateau::{import_citygml_file, CoordinateMode, ImportOptions, SourceOrigin};
use std::path::PathBuf;

#[derive(Debug, Parser)]
#[command(
    name = "rne-plateau-import",
    about = "Convert PLATEAU CityGML building LOD1/LOD2 and road LOD1 into RNE assets"
)]
struct Cli {
    /// Input PLATEAU CityGML file.
    input: PathBuf,
    /// Directory receiving scene, OBJ, and metadata assets.
    #[arg(short, long)]
    output: PathBuf,
    /// Stable base name for generated tile assets.
    #[arg(long, default_value = "plateau_tile")]
    tile_name: String,
    /// Coordinate interpretation; auto uses the CityGML CRS.
    #[arg(long, value_enum, default_value_t = CliCoordinateMode::Auto)]
    coordinate_mode: CliCoordinateMode,
    /// Optional source origin as first,second,height (lat,lon,m or east,north,m).
    #[arg(long, value_parser = parse_origin)]
    origin: Option<SourceOrigin>,
    /// Deterministic scene seed.
    #[arg(long, default_value_t = 0)]
    seed: u64,
}

#[derive(Clone, Copy, Debug, Default, ValueEnum)]
enum CliCoordinateMode {
    #[default]
    Auto,
    Geographic,
    Projected,
}

impl From<CliCoordinateMode> for CoordinateMode {
    fn from(value: CliCoordinateMode) -> Self {
        match value {
            CliCoordinateMode::Auto => Self::Auto,
            CliCoordinateMode::Geographic => Self::GeographicDegrees,
            CliCoordinateMode::Projected => Self::ProjectedMeters,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let result = import_citygml_file(
        &cli.input,
        &cli.output,
        &ImportOptions {
            tile_name: cli.tile_name,
            coordinate_mode: cli.coordinate_mode.into(),
            origin: cli.origin,
            world_seed: cli.seed,
            ..ImportOptions::default()
        },
    )
    .with_context(|| format!("import {}", cli.input.display()))?;
    println!(
        "imported buildings={} lod2={} textured_surfaces={} roads={} lanes={} triangles={} mode={:?} scene={} metadata={}",
        result.building_count,
        result.lod2_building_count,
        result.textured_surface_count,
        result.road_count,
        result.lane_count,
        result.triangle_count,
        result.coordinate_mode,
        result.scene_path.display(),
        result.metadata_path.display()
    );
    Ok(())
}

fn parse_origin(value: &str) -> Result<SourceOrigin, String> {
    let values = value
        .split(',')
        .map(str::trim)
        .map(|item| {
            item.parse::<f64>()
                .map_err(|_| format!("`{item}` is not a finite number"))
                .and_then(|number| {
                    number
                        .is_finite()
                        .then_some(number)
                        .ok_or_else(|| format!("`{item}` is not a finite number"))
                })
        })
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 3 {
        return Err("origin must contain exactly first,second,height".into());
    }
    Ok(SourceOrigin {
        first_deg_or_m: values[0],
        second_deg_or_m: values[1],
        height_m: values[2],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_explicit_origin() {
        assert_eq!(
            parse_origin("35.0, 139.0, 4.5").expect("origin"),
            SourceOrigin {
                first_deg_or_m: 35.0,
                second_deg_or_m: 139.0,
                height_m: 4.5
            }
        );
        assert!(parse_origin("35,139").is_err());
    }
}
