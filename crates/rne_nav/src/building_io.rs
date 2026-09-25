//! Versioned `.rne.building` serialization for multi-floor maps.
//!
//! A [`BuildingMap`](crate::building::BuildingMap) built in code cannot be
//! shared, reviewed, or swapped for a different building without a recompile.
//! This module gives it a file format, so the floors of a site and the lifts and
//! stairs that join them are data a caller edits rather than Rust they write.
//!
//! Each floor references its occupancy map by path rather than embedding it, so
//! a building description stays readable and the maps it points at remain
//! ordinary `.rne.map` files that the SLAM and navigation tools already
//! produce. Paths resolve relative to the building file's own directory, so a
//! site directory can be moved or copied whole.
//!
//! The costmap inflation applied to each floor is part of the description, not
//! a caller default: two robots with different footprints need different
//! inflation over the same map, and a building that silently used one of them
//! would plan routes the other cannot drive.

use crate::building::{
    BuildingError, BuildingMap, Floor, FloorId, FloorTransition, TransitionKind,
};
use crate::costmap::{Costmap, CostmapConfig};
use crate::map_io::{load_map, MapIoError};
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use thiserror::Error;

/// Format tag written into every building file.
pub const RNE_BUILDING_FORMAT: &str = "rne.building";
/// Current building format version.
pub const RNE_BUILDING_VERSION: u32 = 1;

/// One floor of a serialized building.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FloorEntry {
    /// Identifier, unique within the building.
    pub id: usize,
    /// Human-readable name, such as `"1F"`.
    pub name: String,
    /// Height of the walking surface in world meters.
    pub elevation_m: f64,
    /// Occupancy map path, relative to the building file's directory.
    pub map: PathBuf,
    /// Costmap inflation for this floor, in meters.
    pub inflation_radius_m: f64,
}

/// One crossing of a serialized building.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TransitionEntry {
    /// Human-readable name, such as `"north lift"`.
    pub name: String,
    /// How the crossing is made.
    pub kind: TransitionKind,
    /// Floor the robot leaves.
    pub from: usize,
    /// Floor the robot arrives on.
    pub to: usize,
    /// Boarding point in the `from` floor's frame, in meters.
    pub from_point_m: [f64; 3],
    /// Alighting point in the `to` floor's frame, in meters.
    pub to_point_m: [f64; 3],
    /// Expected traversal cost in seconds.
    pub cost_s: f64,
    /// Whether the crossing may also be made in reverse.
    #[serde(default)]
    pub bidirectional: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct BuildingFile {
    format: String,
    version: u32,
    name: String,
    floors: Vec<FloorEntry>,
    #[serde(default)]
    transitions: Vec<TransitionEntry>,
}

/// Errors raised by building serialization or loading.
#[derive(Debug, Error)]
pub enum BuildingIoError {
    /// JSON (de)serialization failed.
    #[error("building serialization failed: {0}")]
    Serialize(#[from] serde_json::Error),
    /// The file carried an unknown format tag.
    #[error("unsupported building format {0:?}")]
    UnsupportedFormat(String),
    /// The file carried a newer or unknown version.
    #[error("unsupported building version {0}")]
    UnsupportedVersion(u32),
    /// A floor's inflation radius was not finite and non-negative.
    #[error("floor {0:?} has an invalid inflation radius")]
    InvalidInflation(String),
    /// A floor's occupancy map could not be read.
    #[error("floor {floor:?} map failed to load: {source}")]
    Map {
        /// Name of the floor whose map failed.
        floor: String,
        /// Underlying map error.
        #[source]
        source: MapIoError,
    },
    /// A floor's map could not be turned into a costmap.
    #[error("floor {0:?} map could not be inflated into a costmap")]
    Costmap(String),
    /// The described building was not a valid map.
    #[error(transparent)]
    Building(#[from] BuildingError),
    /// Reading or writing the building file failed.
    #[error("building file io failed: {0}")]
    Io(#[from] std::io::Error),
}

/// A building description, separate from the maps it points at.
///
/// This is the serializable half of a site: which floors exist, where their
/// maps live, and how a robot crosses between them. [`Self::load`] resolves it
/// into a [`BuildingMap`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BuildingDescription {
    /// Site name.
    pub name: String,
    /// Floors, in any order; identifiers must be unique.
    pub floors: Vec<FloorEntry>,
    /// Crossings between floors.
    pub transitions: Vec<TransitionEntry>,
}

impl BuildingDescription {
    /// Serializes the description to `.rne.building` JSON.
    pub fn to_json(&self) -> Result<String, BuildingIoError> {
        let file = BuildingFile {
            format: RNE_BUILDING_FORMAT.to_string(),
            version: RNE_BUILDING_VERSION,
            name: self.name.clone(),
            floors: self.floors.clone(),
            transitions: self.transitions.clone(),
        };
        Ok(serde_json::to_string_pretty(&file)?)
    }

    /// Parses a description from `.rne.building` JSON.
    ///
    /// Rejects an unknown format tag or version rather than guessing, so a file
    /// written by a newer build fails loudly instead of loading partially.
    pub fn from_json(text: &str) -> Result<Self, BuildingIoError> {
        let file: BuildingFile = serde_json::from_str(text)?;
        if file.format != RNE_BUILDING_FORMAT {
            return Err(BuildingIoError::UnsupportedFormat(file.format));
        }
        if file.version != RNE_BUILDING_VERSION {
            return Err(BuildingIoError::UnsupportedVersion(file.version));
        }
        Ok(Self {
            name: file.name,
            floors: file.floors,
            transitions: file.transitions,
        })
    }

    /// Writes the description to a file.
    pub fn save(&self, path: &Path) -> Result<(), BuildingIoError> {
        std::fs::write(path, self.to_json()?)?;
        Ok(())
    }

    /// Reads a description without loading the maps it references.
    pub fn read(path: &Path) -> Result<Self, BuildingIoError> {
        Self::from_json(&std::fs::read_to_string(path)?)
    }

    /// Resolves the description into a usable [`BuildingMap`].
    ///
    /// `base_dir` is the directory floor map paths resolve against; passing the
    /// building file's own parent directory keeps a site relocatable. Each
    /// floor's occupancy map is loaded and inflated with that floor's declared
    /// radius, and the assembled building is validated, so a description whose
    /// transitions fall outside their floors fails here rather than during
    /// planning.
    pub fn load(&self, base_dir: &Path) -> Result<BuildingMap, BuildingIoError> {
        let mut floors = Vec::with_capacity(self.floors.len());
        for entry in &self.floors {
            if !entry.inflation_radius_m.is_finite() || entry.inflation_radius_m < 0.0 {
                return Err(BuildingIoError::InvalidInflation(entry.name.clone()));
            }
            let grid =
                load_map(&base_dir.join(&entry.map)).map_err(|source| BuildingIoError::Map {
                    floor: entry.name.clone(),
                    source,
                })?;
            let costmap = Costmap::from_occupancy(
                &grid,
                &CostmapConfig {
                    inflation_radius_m: entry.inflation_radius_m,
                    ..CostmapConfig::default()
                },
            )
            .map_err(|_| BuildingIoError::Costmap(entry.name.clone()))?;
            floors.push(Floor {
                id: FloorId(entry.id),
                name: entry.name.clone(),
                elevation_m: entry.elevation_m,
                costmap,
            });
        }
        let transitions = self
            .transitions
            .iter()
            .map(|entry| FloorTransition {
                name: entry.name.clone(),
                kind: entry.kind,
                from: FloorId(entry.from),
                to: FloorId(entry.to),
                from_point_m: Vec3::new(
                    entry.from_point_m[0],
                    entry.from_point_m[1],
                    entry.from_point_m[2],
                ),
                to_point_m: Vec3::new(
                    entry.to_point_m[0],
                    entry.to_point_m[1],
                    entry.to_point_m[2],
                ),
                cost_s: entry.cost_s,
                bidirectional: entry.bidirectional,
            })
            .collect();
        Ok(BuildingMap::new(floors, transitions)?)
    }

    /// Reads a building file and resolves it against its own directory.
    pub fn load_from(path: &Path) -> Result<BuildingMap, BuildingIoError> {
        let description = Self::read(path)?;
        let base = path.parent().unwrap_or(Path::new("."));
        description.load(base)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::building::{plan_building_route, FloorPosition, RouteCosts, RouteLeg};
    use crate::grid::{GridCoord, OccupancyGrid};
    use crate::map_io::save_map;
    use crate::planner::GlobalPlannerConfig;
    use crate::pose2d::Pose2d;

    /// A fully observed open floor, written to `dir/name`.
    fn write_open_map(dir: &Path, name: &str) -> PathBuf {
        let mut grid = OccupancyGrid::new(40, 40, 0.25, Pose2d::new(0.0, 0.0, 0.0)).expect("grid");
        for y in 0..40 {
            for x in 0..40 {
                for _ in 0..8 {
                    grid.mark_free(GridCoord {
                        x: x as isize,
                        y: y as isize,
                    });
                }
            }
        }
        let path = dir.join(name);
        save_map(&path, &grid).expect("save map");
        PathBuf::from(name)
    }

    fn description(first: PathBuf, second: PathBuf) -> BuildingDescription {
        BuildingDescription {
            name: "test site".to_string(),
            floors: vec![
                FloorEntry {
                    id: 0,
                    name: "1F".to_string(),
                    elevation_m: 0.0,
                    map: first,
                    inflation_radius_m: 0.2,
                },
                FloorEntry {
                    id: 1,
                    name: "2F".to_string(),
                    elevation_m: 3.5,
                    map: second,
                    inflation_radius_m: 0.2,
                },
            ],
            transitions: vec![TransitionEntry {
                name: "lift".to_string(),
                kind: TransitionKind::Elevator,
                from: 0,
                to: 1,
                from_point_m: [2.0, 2.0, 0.0],
                to_point_m: [2.0, 2.0, 0.0],
                cost_s: 20.0,
                bidirectional: true,
            }],
        }
    }

    #[test]
    fn a_description_round_trips_through_json_and_rejects_foreign_files() {
        let original = description(PathBuf::from("1f.rne.map"), PathBuf::from("2f.rne.map"));
        let json = original.to_json().expect("serialize");
        assert_eq!(
            BuildingDescription::from_json(&json).expect("deserialize"),
            original
        );

        // The tag and version are checked, so a foreign or newer file fails
        // loudly instead of loading partially.
        let foreign = json.replace(RNE_BUILDING_FORMAT, "rne.map");
        assert!(matches!(
            BuildingDescription::from_json(&foreign),
            Err(BuildingIoError::UnsupportedFormat(tag)) if tag == "rne.map"
        ));
        let newer = json.replace("\"version\": 1", "\"version\": 99");
        assert!(matches!(
            BuildingDescription::from_json(&newer),
            Err(BuildingIoError::UnsupportedVersion(99))
        ));
    }

    #[test]
    fn a_saved_building_reloads_into_a_routable_map() {
        let dir = tempfile::tempdir().expect("temp dir");
        let first = write_open_map(dir.path(), "1f.rne.map");
        let second = write_open_map(dir.path(), "2f.rne.map");
        let path = dir.path().join("site.rne.building");
        description(first, second).save(&path).expect("save");

        let map = BuildingDescription::load_from(&path).expect("load");
        assert_eq!(map.floors().len(), 2);
        assert_eq!(map.transitions().len(), 1);
        assert_eq!(map.floor(FloorId(1)).expect("2F").name, "2F");
        assert_eq!(map.floor(FloorId(1)).expect("2F").elevation_m, 3.5);

        // The loaded building plans across floors, so the maps it pointed at
        // really were inflated into usable costmaps.
        let route = plan_building_route(
            &map,
            FloorPosition::new(FloorId(0), Vec3::new(8.0, 8.0, 0.0)),
            FloorPosition::new(FloorId(1), Vec3::new(8.0, 8.0, 0.0)),
            RouteCosts::default(),
            &GlobalPlannerConfig::default(),
        )
        .expect("route");
        assert!(route
            .legs
            .iter()
            .any(|leg| matches!(leg, RouteLeg::Cross { .. })));
    }

    #[test]
    fn loading_reports_which_floor_failed_rather_than_failing_anonymously() {
        let dir = tempfile::tempdir().expect("temp dir");
        let first = write_open_map(dir.path(), "1f.rne.map");
        let mut broken = description(first, PathBuf::from("missing.rne.map"));
        let err = broken.load(dir.path()).expect_err("missing map");
        assert!(
            matches!(&err, BuildingIoError::Map { floor, .. } if floor == "2F"),
            "expected the failing floor to be named: {err}"
        );

        broken.floors[1].map = PathBuf::from("1f.rne.map");
        broken.floors[1].inflation_radius_m = f64::NAN;
        assert!(matches!(
            broken.load(dir.path()),
            Err(BuildingIoError::InvalidInflation(name)) if name == "2F"
        ));

        // A transition that lands outside its floor is rejected on load, not
        // discovered later during planning.
        broken.floors[1].inflation_radius_m = 0.2;
        broken.transitions[0].to_point_m = [500.0, 500.0, 0.0];
        assert!(matches!(
            broken.load(dir.path()),
            Err(BuildingIoError::Building(_))
        ));
    }
}
