//! Versioned `.rne.map` serialization for occupancy maps.
//!
//! Occupancy maps are stored as JSON with a format tag and a version so a map
//! written by one build can be validated by another. The fixed-point log-odds
//! array round-trips exactly, so a serialized map reloads bit-for-bit.

use crate::grid::{GridError, OccupancyGrid};
use crate::pose2d::Pose2d;
use serde::{Deserialize, Serialize};
use std::path::Path;
use thiserror::Error;

/// Format tag written into every map file.
pub const RNE_MAP_FORMAT: &str = "rne.map";
/// Current map format version.
pub const RNE_MAP_VERSION: u32 = 1;

#[derive(Clone, Debug, Serialize, Deserialize)]
struct MapFile {
    format: String,
    version: u32,
    width: usize,
    height: usize,
    resolution_m: f64,
    origin: Pose2d,
    log_odds: Vec<i16>,
}

/// Errors raised by map serialization or loading.
#[derive(Debug, Error)]
pub enum MapIoError {
    /// JSON (de)serialization failed.
    #[error("map serialization failed: {0}")]
    Serialize(#[from] serde_json::Error),
    /// The file carried an unknown format tag.
    #[error("unsupported map format {0:?}")]
    UnsupportedFormat(String),
    /// The file carried a newer or unknown version.
    #[error("unsupported map version {0}")]
    UnsupportedVersion(u32),
    /// The map geometry was invalid.
    #[error("invalid map geometry: {0}")]
    Grid(#[from] GridError),
    /// Reading or writing the map file failed.
    #[error("map file io failed: {0}")]
    Io(#[from] std::io::Error),
}

/// Serializes a grid to compact `.rne.map` JSON.
pub fn to_map_json(grid: &OccupancyGrid) -> Result<String, MapIoError> {
    let file = MapFile {
        format: RNE_MAP_FORMAT.to_string(),
        version: RNE_MAP_VERSION,
        width: grid.width(),
        height: grid.height(),
        resolution_m: grid.resolution_m(),
        origin: grid.origin(),
        log_odds: grid.log_odds().to_vec(),
    };
    Ok(serde_json::to_string(&file)?)
}

/// Parses `.rne.map` JSON, validating the format tag and version.
pub fn from_map_json(text: &str) -> Result<OccupancyGrid, MapIoError> {
    let file: MapFile = serde_json::from_str(text)?;
    if file.format != RNE_MAP_FORMAT {
        return Err(MapIoError::UnsupportedFormat(file.format));
    }
    if file.version != RNE_MAP_VERSION {
        return Err(MapIoError::UnsupportedVersion(file.version));
    }
    Ok(OccupancyGrid::from_log_odds(
        file.width,
        file.height,
        file.resolution_m,
        file.origin,
        file.log_odds,
    )?)
}

/// Writes a grid to `path` in `.rne.map` form.
pub fn save_map(path: &Path, grid: &OccupancyGrid) -> Result<(), MapIoError> {
    std::fs::write(path, to_map_json(grid)?)?;
    Ok(())
}

/// Loads a grid from a `.rne.map` file.
pub fn load_map(path: &Path) -> Result<OccupancyGrid, MapIoError> {
    let text = std::fs::read_to_string(path)?;
    from_map_json(&text)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::GridCoord;

    #[test]
    fn empty_map_golden_json_is_stable() {
        let grid = OccupancyGrid::new(2, 2, 0.5, Pose2d::IDENTITY).unwrap();
        let json = to_map_json(&grid).unwrap();
        assert_eq!(
            json,
            "{\"format\":\"rne.map\",\"version\":1,\"width\":2,\"height\":2,\
             \"resolution_m\":0.5,\"origin\":{\"x_m\":0.0,\"y_m\":0.0,\"yaw_rad\":0.0},\
             \"log_odds\":[0,0,0,0]}"
        );
    }

    #[test]
    fn round_trips_updated_cells_exactly() {
        let mut grid = OccupancyGrid::new(4, 3, 0.25, Pose2d::new(-1.0, -0.5, 0.1)).unwrap();
        grid.apply_occupied(GridCoord { x: 1, y: 1 }, 0.85);
        grid.apply_free(GridCoord { x: 2, y: 2 }, -0.4);
        let json = to_map_json(&grid).unwrap();
        let restored = from_map_json(&json).unwrap();
        assert_eq!(restored.log_odds(), grid.log_odds());
        assert_eq!(restored.origin(), grid.origin());
        assert_eq!(restored.resolution_m(), grid.resolution_m());
    }

    #[test]
    fn rejects_unknown_format_and_version() {
        let grid = OccupancyGrid::new(2, 2, 0.5, Pose2d::IDENTITY).unwrap();
        let good = to_map_json(&grid).unwrap();
        assert!(matches!(
            from_map_json(&good.replace("\"rne.map\"", "\"other\"")),
            Err(MapIoError::UnsupportedFormat(_))
        ));
        assert!(matches!(
            from_map_json(&good.replace("\"version\":1", "\"version\":9")),
            Err(MapIoError::UnsupportedVersion(9))
        ));
    }
}
