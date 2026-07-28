//! Deterministic offline import of PLATEAU CityGML building LOD1/LOD2 and road LOD1 data.
//!
//! The importer deliberately lives outside the simulation core. It converts a
//! bounded CityGML tile into ordinary RNE scene, OBJ, and JSON assets so runtime
//! simulation remains independent of XML, geospatial, and PLATEAU-specific types.

#![deny(missing_docs)]

use rne_assets::scene::{GroundAsset, ObstacleBodyType, SceneWorldAsset};
use rne_assets::{
    parse_scene_asset, SceneAsset, SceneCollisionAsset, SceneObjectAsset, SceneVisualAsset,
};
use roxmltree::{Document, Node};
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};
use thiserror::Error;

const EARTH_RADIUS_M: f64 = 6_378_137.0;
const EPSILON: f64 = 1.0e-10;

/// Coordinate interpretation used for CityGML `gml:posList` triples.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CoordinateMode {
    /// Infer geographic or projected coordinates from the document CRS.
    #[default]
    Auto,
    /// Interpret triples as latitude degrees, longitude degrees, and height meters.
    GeographicDegrees,
    /// Interpret triples as first horizontal axis, second horizontal axis, and height meters.
    ProjectedMeters,
}

/// Source-space origin used to create local RNE coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SourceOrigin {
    /// First source ordinate: latitude degrees or the first projected axis in meters.
    pub first_deg_or_m: f64,
    /// Second source ordinate: longitude degrees or the second projected axis in meters.
    pub second_deg_or_m: f64,
    /// Source height in meters mapped to local `y = 0`.
    pub height_m: f64,
}

/// Options controlling deterministic CityGML conversion.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportOptions {
    /// Base name used for generated scene and metadata files.
    pub tile_name: String,
    /// Source coordinate interpretation.
    pub coordinate_mode: CoordinateMode,
    /// Optional explicit source origin. The tile center and minimum height are used otherwise.
    pub origin: Option<SourceOrigin>,
    /// Deterministic seed written to the generated RNE scene.
    pub world_seed: u64,
    /// Linear RGBA tint applied to imported building meshes.
    pub building_color_rgba: [f32; 4],
    /// Linear RGBA tint applied to imported road meshes.
    pub road_color_rgba: [f32; 4],
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self {
            tile_name: "plateau_tile".into(),
            coordinate_mode: CoordinateMode::Auto,
            origin: None,
            world_seed: 0,
            building_color_rgba: [0.625, 0.6875, 0.75, 1.0],
            road_color_rgba: [0.0625, 0.078125, 0.09375, 1.0],
        }
    }
}

/// Summary and generated paths returned by an import.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportResult {
    /// Generated `.rne.scene.toml` path.
    pub scene_path: PathBuf,
    /// Generated stable building metadata JSON path.
    pub metadata_path: PathBuf,
    /// Number of imported CityGML buildings.
    pub building_count: usize,
    /// Number of buildings imported from semantic LOD2 boundary surfaces.
    pub lod2_building_count: usize,
    /// Number of building surfaces linked to Appearance textures.
    pub textured_surface_count: usize,
    /// Number of imported CityGML roads.
    pub road_count: usize,
    /// Number of deterministically derived traffic lanes.
    pub lane_count: usize,
    /// Total number of generated mesh triangles.
    pub triangle_count: usize,
    /// Resolved coordinate mode after auto detection.
    pub coordinate_mode: CoordinateMode,
    /// Source-space origin used by the conversion.
    pub origin: SourceOrigin,
    /// Deterministically derived lanes available to runtime examples.
    pub lanes: Vec<ImportedLane>,
}

/// A deterministic straight lane derived from one PLATEAU LOD1 road surface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImportedLane {
    /// Stable lane identifier derived from the road `gml:id` and surface index.
    pub lane_id: String,
    /// Stable source `tran:Road` identifier.
    pub road_source_id: String,
    /// Directed two-point centerline in local RNE meters.
    pub centerline_m: [[f64; 3]; 2],
    /// Derived lane width in meters.
    pub width_m: f64,
    /// Travel direction relative to the road surface's canonical principal axis.
    pub travel_direction: LaneTravelDirection,
}

/// Direction assigned to a derived lane.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneTravelDirection {
    /// Travel from the negative to positive principal-axis endpoint.
    PrincipalAxisPositive,
    /// Travel from the positive to negative principal-axis endpoint.
    PrincipalAxisNegative,
}

/// PLATEAU import failure.
#[derive(Debug, Error)]
pub enum ImportError {
    /// The CityGML XML document is malformed.
    #[error("invalid CityGML XML: {0}")]
    Xml(String),
    /// The document contains no supported building LOD1/LOD2 or road LOD1 geometry.
    #[error(
        "CityGML contains no Building lod2MultiSurface/lod1Solid or Road lod1MultiSurface geometry"
    )]
    NoSupportedLod1Geometry,
    /// A building has no stable `gml:id`.
    #[error("Building is missing gml:id")]
    MissingBuildingId,
    /// Two buildings share the same stable identifier.
    #[error("duplicate Building gml:id `{0}`")]
    DuplicateBuildingId(String),
    /// A road has no stable `gml:id`.
    #[error("Road is missing gml:id")]
    MissingRoadId,
    /// Two roads share the same stable identifier.
    #[error("duplicate Road gml:id `{0}`")]
    DuplicateRoadId(String),
    /// A polygon uses geometry outside the Phase 1 subset.
    #[error("unsupported geometry in CityGML feature `{feature_id}`: {message}")]
    UnsupportedGeometry {
        /// Stable CityGML feature identifier.
        feature_id: String,
        /// Description of the unsupported geometry.
        message: String,
    },
    /// A coordinate list is invalid or non-finite.
    #[error("invalid coordinates in CityGML feature `{feature_id}`: {message}")]
    InvalidCoordinates {
        /// Stable CityGML feature identifier.
        feature_id: String,
        /// Description of the invalid coordinate data.
        message: String,
    },
    /// An Appearance texture reference is unsafe or cannot be resolved.
    #[error("invalid Appearance texture `{uri}`: {message}")]
    InvalidTexture {
        /// Source `app:imageURI` value.
        uri: String,
        /// Validation or resolution failure.
        message: String,
    },
    /// A generated asset could not be read or written.
    #[error("I/O error at {path}: {message}")]
    Io {
        /// Asset path involved in the failure.
        path: String,
        /// Underlying I/O error message.
        message: String,
    },
    /// A generated scene or metadata document could not be serialized.
    #[error("could not serialize generated {kind}: {message}")]
    Serialize {
        /// Generated document kind.
        kind: &'static str,
        /// Serialization error message.
        message: String,
    },
    /// The generated scene did not satisfy the RNE asset schema.
    #[error("generated RNE scene is invalid: {0}")]
    InvalidGeneratedScene(String),
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SourcePoint {
    first_deg_or_m: f64,
    second_deg_or_m: f64,
    height_m: f64,
}

#[derive(Clone, Debug, PartialEq)]
struct ParsedBuilding {
    id: String,
    name: Option<String>,
    function: Option<String>,
    measured_height_m: Option<f64>,
    lod: u8,
    polygons: Vec<ParsedBuildingPolygon>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
enum BuildingSurface {
    Roof,
    Wall,
    Ground,
    OuterCeiling,
    OuterFloor,
    Closure,
    Unknown,
}

#[derive(Clone, Debug, PartialEq)]
struct ParsedBuildingPolygon {
    polygon_id: Option<String>,
    points: Vec<SourcePoint>,
    surface: BuildingSurface,
    texture: Option<TextureBinding>,
}

#[derive(Clone, Debug, PartialEq)]
struct TextureBinding {
    image_uri: String,
    texcoords: Vec<[f64; 2]>,
}

#[derive(Clone, Debug, PartialEq)]
struct ParsedRoad {
    id: String,
    name: Option<String>,
    function: Option<String>,
    polygons: Vec<Vec<SourcePoint>>,
}

#[derive(Clone, Debug, Serialize)]
struct TileMetadata {
    schema_version: u32,
    source: String,
    source_crs: Option<String>,
    coordinate_mode: CoordinateMode,
    source_origin: SourceOrigin,
    axis_mapping: &'static str,
    scene_path: String,
    buildings: Vec<BuildingMetadata>,
    roads: Vec<RoadMetadata>,
}

#[derive(Clone, Debug, Serialize)]
struct BuildingMetadata {
    source_id: String,
    entity_name: String,
    name: Option<String>,
    function: Option<String>,
    measured_height_m: Option<f64>,
    lod: u8,
    surface_counts: BTreeMap<BuildingSurface, usize>,
    textured_surface_count: usize,
    texture_paths: Vec<String>,
    mesh_path: String,
    material_path: Option<String>,
    translation_m: [f64; 3],
    bounds_min_m: [f64; 3],
    bounds_max_m: [f64; 3],
    triangle_count: usize,
}

#[derive(Clone, Debug, Serialize)]
struct RoadMetadata {
    source_id: String,
    entity_name: String,
    name: Option<String>,
    function: Option<String>,
    mesh_path: String,
    bounds_min_m: [f64; 3],
    bounds_max_m: [f64; 3],
    triangle_count: usize,
    lane_derivation: &'static str,
    lanes: Vec<ImportedLane>,
}

#[derive(Clone, Debug)]
struct GeneratedBuilding {
    metadata: BuildingMetadata,
    obj: String,
    mtl: Option<String>,
    size_m: [f64; 3],
}

#[derive(Clone, Debug)]
struct GeneratedRoad {
    metadata: RoadMetadata,
    obj: String,
}

/// Imports a CityGML file and writes deterministic RNE assets into `output_dir`.
pub fn import_citygml_file(
    input_path: &Path,
    output_dir: &Path,
    options: &ImportOptions,
) -> Result<ImportResult, ImportError> {
    let xml = fs::read_to_string(input_path).map_err(|error| io_error(input_path, error))?;
    let source_name = input_path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("<citygml>");
    import_citygml_impl(&xml, source_name, input_path.parent(), output_dir, options)
}

/// Imports CityGML text and writes deterministic RNE assets into `output_dir`.
pub fn import_citygml_str(
    xml: &str,
    source_name: &str,
    output_dir: &Path,
    options: &ImportOptions,
) -> Result<ImportResult, ImportError> {
    import_citygml_impl(xml, source_name, None, output_dir, options)
}

fn import_citygml_impl(
    xml: &str,
    source_name: &str,
    source_dir: Option<&Path>,
    output_dir: &Path,
    options: &ImportOptions,
) -> Result<ImportResult, ImportError> {
    validate_options(options)?;
    let document = Document::parse(xml).map_err(|error| ImportError::Xml(error.to_string()))?;
    let source_crs = document
        .descendants()
        .find_map(|node| node.attribute("srsName"))
        .map(str::to_owned);
    let appearances = parse_appearance_textures(&document)?;
    let mut buildings = parse_buildings(&document, &appearances)?;
    buildings.sort_by(|left, right| left.id.cmp(&right.id));
    let mut roads = parse_roads(&document)?;
    roads.sort_by(|left, right| left.id.cmp(&right.id));
    if buildings.is_empty() && roads.is_empty() {
        return Err(ImportError::NoSupportedLod1Geometry);
    }

    let mode = resolve_coordinate_mode(options.coordinate_mode, source_crs.as_deref());
    let origin = options
        .origin
        .unwrap_or_else(|| default_origin(&buildings, &roads));
    let tile_name = sanitize_component(&options.tile_name);
    let texture_paths = resolve_texture_paths(&buildings, source_dir)?;
    let mut generated = Vec::with_capacity(buildings.len());
    for (index, building) in buildings.iter().enumerate() {
        generated.push(generate_building(
            building,
            index,
            mode,
            origin,
            &texture_paths,
        )?);
    }
    let mut generated_roads = Vec::with_capacity(roads.len());
    for (index, road) in roads.iter().enumerate() {
        generated_roads.push(generate_road(road, index, mode, origin)?);
    }

    fs::create_dir_all(output_dir).map_err(|error| io_error(output_dir, error))?;
    let meshes_dir = output_dir.join("meshes");
    fs::create_dir_all(&meshes_dir).map_err(|error| io_error(&meshes_dir, error))?;
    copy_appearance_textures(source_dir, output_dir, &texture_paths)?;
    for building in &generated {
        let path = output_dir.join(&building.metadata.mesh_path);
        fs::write(&path, &building.obj).map_err(|error| io_error(&path, error))?;
        if let (Some(material_path), Some(mtl)) = (&building.metadata.material_path, &building.mtl)
        {
            let material_path = output_dir.join(material_path);
            fs::write(&material_path, mtl).map_err(|error| io_error(&material_path, error))?;
        }
    }
    for road in &generated_roads {
        let path = output_dir.join(&road.metadata.mesh_path);
        fs::write(&path, &road.obj).map_err(|error| io_error(&path, error))?;
    }

    let scene = generated_scene(&generated, &generated_roads, options);
    let scene_text = toml::to_string_pretty(&scene).map_err(|error| ImportError::Serialize {
        kind: "scene TOML",
        message: error.to_string(),
    })?;
    parse_scene_asset(&scene_text, Path::new("<generated-plateau-scene>"))
        .map_err(|error| ImportError::InvalidGeneratedScene(error.to_string()))?;
    let scene_path = output_dir.join(format!("{tile_name}.rne.scene.toml"));
    fs::write(&scene_path, format!("{scene_text}\n"))
        .map_err(|error| io_error(&scene_path, error))?;

    let metadata = TileMetadata {
        schema_version: 3,
        source: source_name.to_owned(),
        source_crs,
        coordinate_mode: mode,
        source_origin: origin,
        axis_mapping: match mode {
            CoordinateMode::GeographicDegrees => "longitude -> +X, height -> +Y, latitude -> -Z",
            CoordinateMode::ProjectedMeters | CoordinateMode::Auto => {
                "projected axis 1 -> +X, height -> +Y, projected axis 2 -> -Z"
            }
        },
        scene_path: scene_path
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or_default()
            .to_owned(),
        buildings: generated
            .iter()
            .map(|building| building.metadata.clone())
            .collect(),
        roads: generated_roads
            .iter()
            .map(|road| road.metadata.clone())
            .collect(),
    };
    let mut metadata_text =
        serde_json::to_string_pretty(&metadata).map_err(|error| ImportError::Serialize {
            kind: "metadata JSON",
            message: error.to_string(),
        })?;
    metadata_text.push('\n');
    let metadata_path = output_dir.join(format!("{tile_name}.plateau.json"));
    fs::write(&metadata_path, metadata_text).map_err(|error| io_error(&metadata_path, error))?;

    Ok(ImportResult {
        scene_path,
        metadata_path,
        building_count: generated.len(),
        lod2_building_count: generated
            .iter()
            .filter(|building| building.metadata.lod == 2)
            .count(),
        textured_surface_count: generated
            .iter()
            .map(|building| building.metadata.textured_surface_count)
            .sum(),
        road_count: generated_roads.len(),
        lane_count: generated_roads
            .iter()
            .map(|road| road.metadata.lanes.len())
            .sum(),
        triangle_count: generated
            .iter()
            .map(|building| building.metadata.triangle_count)
            .chain(
                generated_roads
                    .iter()
                    .map(|road| road.metadata.triangle_count),
            )
            .sum(),
        coordinate_mode: mode,
        origin,
        lanes: generated_roads
            .iter()
            .flat_map(|road| road.metadata.lanes.iter().cloned())
            .collect(),
    })
}

fn validate_options(options: &ImportOptions) -> Result<(), ImportError> {
    if options.tile_name.trim().is_empty() {
        return Err(ImportError::InvalidGeneratedScene(
            "tile_name must not be empty".into(),
        ));
    }
    if !options
        .building_color_rgba
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(ImportError::InvalidGeneratedScene(
            "building_color_rgba must be finite".into(),
        ));
    }
    if !options
        .road_color_rgba
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(ImportError::InvalidGeneratedScene(
            "road_color_rgba must be finite".into(),
        ));
    }
    if let Some(origin) = options.origin {
        if ![
            origin.first_deg_or_m,
            origin.second_deg_or_m,
            origin.height_m,
        ]
        .iter()
        .all(|value| value.is_finite())
        {
            return Err(ImportError::InvalidGeneratedScene(
                "explicit origin must be finite".into(),
            ));
        }
    }
    Ok(())
}

fn parse_appearance_textures(
    document: &Document<'_>,
) -> Result<HashMap<String, TextureBinding>, ImportError> {
    let mut textures = HashMap::new();
    for parameterized in document
        .descendants()
        .filter(|node| node.is_element() && node.tag_name().name() == "ParameterizedTexture")
    {
        let image_uri = descendant_text(parameterized, "imageURI").ok_or_else(|| {
            ImportError::InvalidTexture {
                uri: "<missing>".into(),
                message: "ParameterizedTexture has no imageURI".into(),
            }
        })?;
        for target in parameterized
            .descendants()
            .filter(|node| node.is_element() && node.tag_name().name() == "target")
        {
            let Some(polygon_id) = target
                .attributes()
                .find(|attribute| attribute.name() == "uri")
                .map(|attribute| attribute.value().trim().trim_start_matches('#'))
                .filter(|id| !id.is_empty())
            else {
                continue;
            };
            let coordinates = target
                .descendants()
                .find(|node| node.is_element() && node.tag_name().name() == "textureCoordinates")
                .ok_or_else(|| ImportError::InvalidTexture {
                    uri: image_uri.clone(),
                    message: format!("target #{polygon_id} has no textureCoordinates"),
                })?;
            let values = parse_numbers(coordinates.text().unwrap_or_default(), polygon_id)?;
            if values.len() < 6 || !values.len().is_multiple_of(2) {
                return Err(ImportError::InvalidTexture {
                    uri: image_uri.clone(),
                    message: format!("target #{polygon_id} must contain at least three UV pairs"),
                });
            }
            let mut texcoords: Vec<[f64; 2]> = values
                .chunks_exact(2)
                .map(|value| [value[0], value[1]])
                .collect();
            if texcoords.first() == texcoords.last() {
                texcoords.pop();
            }
            let binding = TextureBinding {
                image_uri: image_uri.clone(),
                texcoords,
            };
            if textures.insert(polygon_id.to_owned(), binding).is_some() {
                return Err(ImportError::InvalidTexture {
                    uri: image_uri.clone(),
                    message: format!("polygon #{polygon_id} has multiple texture targets"),
                });
            }
        }
    }
    Ok(textures)
}

fn parse_buildings(
    document: &Document<'_>,
    appearances: &HashMap<String, TextureBinding>,
) -> Result<Vec<ParsedBuilding>, ImportError> {
    let mut seen = HashSet::new();
    let mut buildings = Vec::new();
    for node in document
        .descendants()
        .filter(|node| node.is_element() && node.tag_name().name() == "Building")
    {
        let id = node
            .attributes()
            .find(|attribute| attribute.name() == "id")
            .map(|attribute| attribute.value().trim().to_owned())
            .filter(|id| !id.is_empty())
            .ok_or(ImportError::MissingBuildingId)?;
        if !seen.insert(id.clone()) {
            return Err(ImportError::DuplicateBuildingId(id));
        }
        let lod2_roots: Vec<_> = node
            .descendants()
            .filter(|child| child.is_element() && child.tag_name().name() == "lod2MultiSurface")
            .collect();
        let (lod, geometry_roots) = if lod2_roots.is_empty() {
            let Some(lod1) = node
                .descendants()
                .find(|child| child.is_element() && child.tag_name().name() == "lod1Solid")
            else {
                continue;
            };
            (1, vec![lod1])
        } else {
            (2, lod2_roots)
        };
        let mut polygons = Vec::new();
        for geometry in geometry_roots {
            for polygon in geometry
                .descendants()
                .filter(|child| child.is_element() && child.tag_name().name() == "Polygon")
            {
                let polygon_id = polygon
                    .attributes()
                    .find(|attribute| attribute.name() == "id")
                    .map(|attribute| attribute.value().trim().to_owned())
                    .filter(|value| !value.is_empty());
                let points = parse_polygon(polygon, &id)?;
                let texture = polygon_id
                    .as_ref()
                    .and_then(|polygon_id| appearances.get(polygon_id))
                    .cloned();
                if let Some(texture) = &texture {
                    if texture.texcoords.len() != points.len() {
                        return Err(ImportError::InvalidTexture {
                            uri: texture.image_uri.clone(),
                            message: format!(
                                "polygon #{} has {} vertices but {} UV pairs",
                                polygon_id.as_deref().unwrap_or_default(),
                                points.len(),
                                texture.texcoords.len()
                            ),
                        });
                    }
                }
                polygons.push(ParsedBuildingPolygon {
                    polygon_id,
                    points,
                    surface: building_surface(polygon),
                    texture,
                });
            }
        }
        if polygons.is_empty() {
            continue;
        }
        buildings.push(ParsedBuilding {
            id,
            name: descendant_text(node, "name"),
            function: descendant_text(node, "function"),
            measured_height_m: descendant_text(node, "measuredHeight")
                .and_then(|value| value.parse().ok()),
            lod,
            polygons,
        });
    }
    Ok(buildings)
}

fn building_surface(polygon: Node<'_, '_>) -> BuildingSurface {
    for ancestor in polygon.ancestors() {
        let surface = match ancestor.tag_name().name() {
            "RoofSurface" => BuildingSurface::Roof,
            "WallSurface" => BuildingSurface::Wall,
            "GroundSurface" => BuildingSurface::Ground,
            "OuterCeilingSurface" => BuildingSurface::OuterCeiling,
            "OuterFloorSurface" => BuildingSurface::OuterFloor,
            "ClosureSurface" => BuildingSurface::Closure,
            "Building" => break,
            _ => continue,
        };
        return surface;
    }
    BuildingSurface::Unknown
}

fn parse_roads(document: &Document<'_>) -> Result<Vec<ParsedRoad>, ImportError> {
    let mut seen = HashSet::new();
    let mut roads = Vec::new();
    for node in document
        .descendants()
        .filter(|node| node.is_element() && node.tag_name().name() == "Road")
    {
        let id = node
            .attributes()
            .find(|attribute| attribute.name() == "id")
            .map(|attribute| attribute.value().trim().to_owned())
            .filter(|id| !id.is_empty())
            .ok_or(ImportError::MissingRoadId)?;
        if !seen.insert(id.clone()) {
            return Err(ImportError::DuplicateRoadId(id));
        }
        let Some(lod1) = node
            .descendants()
            .find(|child| child.is_element() && child.tag_name().name() == "lod1MultiSurface")
        else {
            continue;
        };
        let mut polygons = Vec::new();
        for polygon in lod1
            .descendants()
            .filter(|child| child.is_element() && child.tag_name().name() == "Polygon")
        {
            polygons.push(parse_polygon(polygon, &id)?);
        }
        if polygons.is_empty() {
            continue;
        }
        roads.push(ParsedRoad {
            id,
            name: descendant_text(node, "name"),
            function: descendant_text(node, "function"),
            polygons,
        });
    }
    Ok(roads)
}

fn parse_polygon(polygon: Node<'_, '_>, feature_id: &str) -> Result<Vec<SourcePoint>, ImportError> {
    if polygon
        .descendants()
        .any(|node| node.is_element() && node.tag_name().name() == "interior")
    {
        return Err(ImportError::UnsupportedGeometry {
            feature_id: feature_id.into(),
            message: "polygon interior rings are not supported in Phase 1".into(),
        });
    }
    let exterior = polygon
        .descendants()
        .find(|node| node.is_element() && node.tag_name().name() == "exterior")
        .ok_or_else(|| ImportError::UnsupportedGeometry {
            feature_id: feature_id.into(),
            message: "polygon has no exterior ring".into(),
        })?;
    let ring = exterior
        .descendants()
        .find(|node| node.is_element() && node.tag_name().name() == "LinearRing")
        .ok_or_else(|| ImportError::UnsupportedGeometry {
            feature_id: feature_id.into(),
            message: "polygon exterior has no LinearRing".into(),
        })?;
    let mut points = if let Some(pos_list) = ring
        .descendants()
        .find(|node| node.is_element() && node.tag_name().name() == "posList")
    {
        parse_pos_list(pos_list, feature_id)?
    } else {
        parse_pos_elements(ring, feature_id)?
    };
    remove_duplicate_ring_end(&mut points);
    remove_consecutive_duplicates(&mut points);
    if points.len() < 3 {
        return Err(ImportError::InvalidCoordinates {
            feature_id: feature_id.into(),
            message: "polygon ring must contain at least three distinct points".into(),
        });
    }
    Ok(points)
}

fn parse_pos_list(
    pos_list: Node<'_, '_>,
    feature_id: &str,
) -> Result<Vec<SourcePoint>, ImportError> {
    let values = parse_numbers(pos_list.text().unwrap_or_default(), feature_id)?;
    let dimension = pos_list
        .attribute("srsDimension")
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(3);
    if dimension != 3 || !values.len().is_multiple_of(3) {
        return Err(ImportError::InvalidCoordinates {
            feature_id: feature_id.into(),
            message: format!(
                "expected 3D posList triples, got dimension {dimension} and {} values",
                values.len()
            ),
        });
    }
    Ok(values
        .chunks_exact(3)
        .map(|value| SourcePoint {
            first_deg_or_m: value[0],
            second_deg_or_m: value[1],
            height_m: value[2],
        })
        .collect())
}

fn parse_pos_elements(
    ring: Node<'_, '_>,
    feature_id: &str,
) -> Result<Vec<SourcePoint>, ImportError> {
    let mut points = Vec::new();
    for pos in ring
        .descendants()
        .filter(|node| node.is_element() && node.tag_name().name() == "pos")
    {
        let values = parse_numbers(pos.text().unwrap_or_default(), feature_id)?;
        if values.len() != 3 {
            return Err(ImportError::InvalidCoordinates {
                feature_id: feature_id.into(),
                message: format!("expected 3 values in gml:pos, got {}", values.len()),
            });
        }
        points.push(SourcePoint {
            first_deg_or_m: values[0],
            second_deg_or_m: values[1],
            height_m: values[2],
        });
    }
    if points.is_empty() {
        return Err(ImportError::UnsupportedGeometry {
            feature_id: feature_id.into(),
            message: "LinearRing has neither posList nor pos elements".into(),
        });
    }
    Ok(points)
}

fn parse_numbers(text: &str, feature_id: &str) -> Result<Vec<f64>, ImportError> {
    text.split_whitespace()
        .map(|token| {
            token
                .parse::<f64>()
                .ok()
                .filter(|value| value.is_finite())
                .ok_or_else(|| ImportError::InvalidCoordinates {
                    feature_id: feature_id.into(),
                    message: format!("`{token}` is not a finite number"),
                })
        })
        .collect()
}

fn descendant_text(node: Node<'_, '_>, local_name: &str) -> Option<String> {
    node.descendants()
        .find(|child| child.is_element() && child.tag_name().name() == local_name)
        .and_then(|child| child.text())
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .map(str::to_owned)
}

fn remove_duplicate_ring_end(points: &mut Vec<SourcePoint>) {
    if points.len() >= 2 && points.first() == points.last() {
        points.pop();
    }
}

fn remove_consecutive_duplicates(points: &mut Vec<SourcePoint>) {
    points.dedup();
}

fn resolve_coordinate_mode(requested: CoordinateMode, crs: Option<&str>) -> CoordinateMode {
    if requested != CoordinateMode::Auto {
        return requested;
    }
    let normalized = crs.unwrap_or_default().to_ascii_lowercase();
    if normalized.contains("6697")
        || normalized.contains("6668")
        || normalized.contains("crs84")
        || normalized.contains("4326")
    {
        CoordinateMode::GeographicDegrees
    } else {
        CoordinateMode::ProjectedMeters
    }
}

fn default_origin(buildings: &[ParsedBuilding], roads: &[ParsedRoad]) -> SourceOrigin {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for point in buildings
        .iter()
        .flat_map(|building| building.polygons.iter())
        .flat_map(|polygon| polygon.points.iter())
        .chain(roads.iter().flat_map(|road| road.polygons.iter()).flatten())
    {
        min[0] = min[0].min(point.first_deg_or_m);
        min[1] = min[1].min(point.second_deg_or_m);
        min[2] = min[2].min(point.height_m);
        max[0] = max[0].max(point.first_deg_or_m);
        max[1] = max[1].max(point.second_deg_or_m);
    }
    SourceOrigin {
        first_deg_or_m: (min[0] + max[0]) * 0.5,
        second_deg_or_m: (min[1] + max[1]) * 0.5,
        height_m: min[2],
    }
}

fn resolve_texture_paths(
    buildings: &[ParsedBuilding],
    source_dir: Option<&Path>,
) -> Result<BTreeMap<String, String>, ImportError> {
    let image_uris: BTreeSet<_> = buildings
        .iter()
        .flat_map(|building| building.polygons.iter())
        .filter_map(|polygon| polygon.texture.as_ref())
        .map(|texture| texture.image_uri.clone())
        .collect();
    let mut paths = BTreeMap::new();
    for (index, uri) in image_uris.into_iter().enumerate() {
        let relative = Path::new(&uri);
        if relative.as_os_str().is_empty()
            || relative.is_absolute()
            || relative.components().any(|component| {
                matches!(
                    component,
                    std::path::Component::ParentDir
                        | std::path::Component::RootDir
                        | std::path::Component::Prefix(_)
                )
            })
        {
            return Err(ImportError::InvalidTexture {
                uri,
                message: "imageURI must be a safe relative path".into(),
            });
        }
        let extension = relative
            .extension()
            .and_then(|extension| extension.to_str())
            .map(str::to_ascii_lowercase)
            .filter(|extension| matches!(extension.as_str(), "png" | "jpg" | "jpeg"))
            .ok_or_else(|| ImportError::InvalidTexture {
                uri: uri.clone(),
                message: "only PNG and JPEG images are supported".into(),
            })?;
        let source_dir = source_dir.ok_or_else(|| ImportError::InvalidTexture {
            uri: uri.clone(),
            message: "string imports cannot resolve external Appearance images".into(),
        })?;
        let source = source_dir.join(relative);
        if !source.is_file() {
            return Err(ImportError::InvalidTexture {
                uri,
                message: format!("referenced file does not exist at {}", source.display()),
            });
        }
        paths.insert(uri, format!("textures/appearance_{index:04}.{extension}"));
    }
    Ok(paths)
}

fn copy_appearance_textures(
    source_dir: Option<&Path>,
    output_dir: &Path,
    texture_paths: &BTreeMap<String, String>,
) -> Result<(), ImportError> {
    if texture_paths.is_empty() {
        return Ok(());
    }
    let source_dir = source_dir.expect("validated while resolving texture paths");
    let textures_dir = output_dir.join("textures");
    fs::create_dir_all(&textures_dir).map_err(|error| io_error(&textures_dir, error))?;
    for (uri, generated_path) in texture_paths {
        let source = source_dir.join(uri);
        let destination = output_dir.join(generated_path);
        fs::copy(&source, &destination).map_err(|error| io_error(&source, error))?;
    }
    Ok(())
}

fn generate_building(
    building: &ParsedBuilding,
    index: usize,
    mode: CoordinateMode,
    origin: SourceOrigin,
    texture_paths: &BTreeMap<String, String>,
) -> Result<GeneratedBuilding, ImportError> {
    let local_polygons: Vec<Vec<[f64; 3]>> = building
        .polygons
        .iter()
        .map(|polygon| {
            polygon
                .points
                .iter()
                .map(|point| source_to_local(*point, mode, origin))
                .collect()
        })
        .collect();
    let (bounds_min_m, bounds_max_m) = bounds(&local_polygons);
    let translation_m = [
        (bounds_min_m[0] + bounds_max_m[0]) * 0.5,
        (bounds_min_m[1] + bounds_max_m[1]) * 0.5,
        (bounds_min_m[2] + bounds_max_m[2]) * 0.5,
    ];
    let size_m = [
        bounds_max_m[0] - bounds_min_m[0],
        bounds_max_m[1] - bounds_min_m[1],
        bounds_max_m[2] - bounds_min_m[2],
    ];
    if size_m
        .iter()
        .any(|extent| !extent.is_finite() || *extent <= 0.0)
    {
        return Err(ImportError::InvalidCoordinates {
            feature_id: building.id.clone(),
            message: format!(
                "LOD{} building has degenerate bounds {size_m:?}",
                building.lod
            ),
        });
    }

    let safe_id = sanitize_component(&building.id);
    let entity_name = format!("plateau_building_{index:04}_{safe_id}");
    let mesh_path = format!("meshes/{entity_name}.obj");
    let material_path = (building.lod == 2).then(|| format!("meshes/{entity_name}.mtl"));
    let mut obj = format!(
        "# RNE PLATEAU LOD{} building {}\n",
        building.lod, building.id
    );
    if building.lod == 2 {
        obj.push_str(&format!("mtllib {entity_name}.mtl\n"));
    }
    obj.push_str(&format!("o {entity_name}\n"));
    let mut mtl = (building.lod == 2).then(String::new);
    let mut vertex_count = 0_usize;
    let mut triangle_count = 0_usize;
    let mut surface_counts = BTreeMap::new();
    let mut used_texture_paths = BTreeSet::new();
    let mut textured_surface_count = 0;
    for (polygon_index, polygon) in local_polygons.iter().enumerate() {
        let parsed_polygon = &building.polygons[polygon_index];
        *surface_counts.entry(parsed_polygon.surface).or_insert(0) += 1;
        for point in polygon {
            obj.push_str(&format!(
                "v {:.6} {:.6} {:.6}\n",
                clean_zero(point[0] - translation_m[0]),
                clean_zero(point[1] - translation_m[1]),
                clean_zero(point[2] - translation_m[2])
            ));
        }
        if building.lod == 2 {
            let texcoords = parsed_polygon
                .texture
                .as_ref()
                .map(|texture| texture.texcoords.as_slice());
            for vertex_index in 0..polygon.len() {
                let texcoord = texcoords
                    .map(|texcoords| texcoords[vertex_index])
                    .unwrap_or([0.0, 0.0]);
                obj.push_str(&format!("vt {:.6} {:.6}\n", texcoord[0], texcoord[1]));
            }
            let material_name = format!("surface_{polygon_index:04}");
            obj.push_str(&format!("usemtl {material_name}\n"));
            let material = mtl.as_mut().expect("LOD2 material document");
            material.push_str(&format!("newmtl {material_name}\n"));
            if let Some(texture) = &parsed_polygon.texture {
                let generated_path = texture_paths
                    .get(&texture.image_uri)
                    .expect("validated texture path");
                material.push_str("Kd 1.000000 1.000000 1.000000\n");
                material.push_str(&format!("map_Kd ../{generated_path}\n\n"));
                used_texture_paths.insert(generated_path.clone());
                textured_surface_count += 1;
            } else {
                let color = semantic_surface_color(parsed_polygon.surface);
                material.push_str(&format!(
                    "Kd {:.6} {:.6} {:.6}\n\n",
                    color[0], color[1], color[2]
                ));
            }
        }
        let triangles =
            triangulate_polygon(polygon).map_err(|message| ImportError::UnsupportedGeometry {
                feature_id: building.id.clone(),
                message,
            })?;
        for triangle in triangles {
            let indices = [
                vertex_count + triangle[0] + 1,
                vertex_count + triangle[1] + 1,
                vertex_count + triangle[2] + 1,
            ];
            if building.lod == 2 {
                obj.push_str(&format!(
                    "f {0}/{0} {1}/{1} {2}/{2}\n",
                    indices[0], indices[1], indices[2]
                ));
            } else {
                obj.push_str(&format!("f {} {} {}\n", indices[0], indices[1], indices[2]));
            }
            triangle_count += 1;
        }
        vertex_count += polygon.len();
    }

    Ok(GeneratedBuilding {
        metadata: BuildingMetadata {
            source_id: building.id.clone(),
            entity_name,
            name: building.name.clone(),
            function: building.function.clone(),
            measured_height_m: building.measured_height_m,
            lod: building.lod,
            surface_counts,
            textured_surface_count,
            texture_paths: used_texture_paths.into_iter().collect(),
            mesh_path,
            material_path,
            translation_m,
            bounds_min_m,
            bounds_max_m,
            triangle_count,
        },
        obj,
        mtl,
        size_m,
    })
}

fn semantic_surface_color(surface: BuildingSurface) -> [f32; 3] {
    match surface {
        BuildingSurface::Roof => [0.42, 0.20, 0.16],
        BuildingSurface::Wall => [0.72, 0.69, 0.61],
        BuildingSurface::Ground | BuildingSurface::OuterFloor => [0.32, 0.34, 0.33],
        BuildingSurface::OuterCeiling => [0.62, 0.61, 0.56],
        BuildingSurface::Closure => [0.55, 0.57, 0.58],
        BuildingSurface::Unknown => [0.63, 0.68, 0.72],
    }
}

fn generate_road(
    road: &ParsedRoad,
    index: usize,
    mode: CoordinateMode,
    origin: SourceOrigin,
) -> Result<GeneratedRoad, ImportError> {
    let local_polygons: Vec<Vec<[f64; 3]>> = road
        .polygons
        .iter()
        .map(|polygon| {
            polygon
                .iter()
                .map(|point| source_to_local(*point, mode, origin))
                .collect()
        })
        .collect();
    let (bounds_min_m, bounds_max_m) = bounds(&local_polygons);
    let safe_id = sanitize_component(&road.id);
    let entity_name = format!("plateau_road_{index:04}_{safe_id}");
    let mesh_path = format!("meshes/{entity_name}.obj");
    let mut obj = format!("# RNE PLATEAU LOD1 road {}\no {entity_name}\n", road.id);
    let mut vertex_count = 0_usize;
    let mut triangle_count = 0_usize;
    let mut lanes = Vec::new();
    for (polygon_index, polygon) in local_polygons.iter().enumerate() {
        for point in polygon {
            obj.push_str(&format!(
                "v {:.6} {:.6} {:.6}\n",
                clean_zero(point[0]),
                clean_zero(point[1] + 0.02),
                clean_zero(point[2])
            ));
        }
        let triangles =
            triangulate_polygon(polygon).map_err(|message| ImportError::UnsupportedGeometry {
                feature_id: road.id.clone(),
                message,
            })?;
        for mut triangle in triangles {
            let a = polygon[triangle[0]];
            let b = polygon[triangle[1]];
            let c = polygon[triangle[2]];
            let normal_y = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]);
            if normal_y < 0.0 {
                triangle.swap(1, 2);
            }
            obj.push_str(&format!(
                "f {} {} {}\n",
                vertex_count + triangle[0] + 1,
                vertex_count + triangle[1] + 1,
                vertex_count + triangle[2] + 1
            ));
            triangle_count += 1;
        }
        vertex_count += polygon.len();
        lanes.extend(derive_two_way_lanes(polygon, &road.id, polygon_index));
    }

    Ok(GeneratedRoad {
        metadata: RoadMetadata {
            source_id: road.id.clone(),
            entity_name,
            name: road.name.clone(),
            function: road.function.clone(),
            mesh_path,
            bounds_min_m,
            bounds_max_m,
            triangle_count,
            lane_derivation: "deterministic principal-axis approximation from LOD1 road surface",
            lanes,
        },
        obj,
    })
}

fn derive_two_way_lanes(
    polygon: &[[f64; 3]],
    road_id: &str,
    polygon_index: usize,
) -> Vec<ImportedLane> {
    let count = polygon.len() as f64;
    let center_x = polygon.iter().map(|point| point[0]).sum::<f64>() / count;
    let center_y = polygon.iter().map(|point| point[1]).sum::<f64>() / count;
    let center_z = polygon.iter().map(|point| point[2]).sum::<f64>() / count;
    let mut covariance_xx = 0.0;
    let mut covariance_xz = 0.0;
    let mut covariance_zz = 0.0;
    for point in polygon {
        let x = point[0] - center_x;
        let z = point[2] - center_z;
        covariance_xx += x * x;
        covariance_xz += x * z;
        covariance_zz += z * z;
    }
    let angle_rad = 0.5 * (2.0 * covariance_xz).atan2(covariance_xx - covariance_zz);
    let mut axis = [angle_rad.cos(), angle_rad.sin()];
    if axis[0] < -EPSILON || (axis[0].abs() <= EPSILON && axis[1] < 0.0) {
        axis[0] = -axis[0];
        axis[1] = -axis[1];
    }
    let perpendicular = [-axis[1], axis[0]];
    let mut axis_min = f64::INFINITY;
    let mut axis_max = f64::NEG_INFINITY;
    let mut transverse_min = f64::INFINITY;
    let mut transverse_max = f64::NEG_INFINITY;
    for point in polygon {
        let relative = [point[0] - center_x, point[2] - center_z];
        let along = relative[0] * axis[0] + relative[1] * axis[1];
        let transverse = relative[0] * perpendicular[0] + relative[1] * perpendicular[1];
        axis_min = axis_min.min(along);
        axis_max = axis_max.max(along);
        transverse_min = transverse_min.min(transverse);
        transverse_max = transverse_max.max(transverse);
    }
    let length_m = axis_max - axis_min;
    let road_width_m = transverse_max - transverse_min;
    if !length_m.is_finite()
        || !road_width_m.is_finite()
        || road_width_m < 4.0
        || length_m < road_width_m * 1.5
    {
        return Vec::new();
    }
    let transverse_center = (transverse_min + transverse_max) * 0.5;
    let lane_width_m = road_width_m * 0.5;
    [-0.25, 0.25]
        .into_iter()
        .enumerate()
        .map(|(lane_index, width_fraction)| {
            let lane_transverse = transverse_center + road_width_m * width_fraction;
            let point = |along: f64| {
                [
                    center_x + axis[0] * along + perpendicular[0] * lane_transverse,
                    center_y + 0.05,
                    center_z + axis[1] * along + perpendicular[1] * lane_transverse,
                ]
            };
            let mut centerline_m = [point(axis_min), point(axis_max)];
            let travel_direction = if lane_index == 0 {
                LaneTravelDirection::PrincipalAxisPositive
            } else {
                centerline_m.swap(0, 1);
                LaneTravelDirection::PrincipalAxisNegative
            };
            ImportedLane {
                lane_id: format!("{road_id}/surface-{polygon_index:04}/lane-{lane_index}"),
                road_source_id: road_id.to_owned(),
                centerline_m,
                width_m: lane_width_m,
                travel_direction,
            }
        })
        .collect()
}

fn source_to_local(point: SourcePoint, mode: CoordinateMode, origin: SourceOrigin) -> [f64; 3] {
    match mode {
        CoordinateMode::GeographicDegrees => {
            let latitude_rad = origin.first_deg_or_m.to_radians();
            [
                (point.second_deg_or_m - origin.second_deg_or_m).to_radians()
                    * EARTH_RADIUS_M
                    * latitude_rad.cos(),
                point.height_m - origin.height_m,
                -(point.first_deg_or_m - origin.first_deg_or_m).to_radians() * EARTH_RADIUS_M,
            ]
        }
        CoordinateMode::ProjectedMeters | CoordinateMode::Auto => [
            point.first_deg_or_m - origin.first_deg_or_m,
            point.height_m - origin.height_m,
            -(point.second_deg_or_m - origin.second_deg_or_m),
        ],
    }
}

fn bounds(polygons: &[Vec<[f64; 3]>]) -> ([f64; 3], [f64; 3]) {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for point in polygons.iter().flatten() {
        for axis in 0..3 {
            min[axis] = min[axis].min(point[axis]);
            max[axis] = max[axis].max(point[axis]);
        }
    }
    (min, max)
}

fn triangulate_polygon(points: &[[f64; 3]]) -> Result<Vec<[usize; 3]>, String> {
    if points.len() == 3 {
        return Ok(vec![[0, 1, 2]]);
    }
    let projected = project_polygon(points)?;
    let area = signed_area(&projected);
    if area.abs() <= EPSILON {
        return Err("polygon has zero projected area".into());
    }
    let orientation = area.signum();
    let mut remaining: Vec<usize> = (0..points.len()).collect();
    let mut triangles = Vec::with_capacity(points.len() - 2);
    while remaining.len() > 3 {
        let mut ear = None;
        for cursor in 0..remaining.len() {
            let previous = remaining[(cursor + remaining.len() - 1) % remaining.len()];
            let current = remaining[cursor];
            let next = remaining[(cursor + 1) % remaining.len()];
            if cross_2d(projected[previous], projected[current], projected[next]) * orientation
                <= EPSILON
            {
                continue;
            }
            let contains_vertex = remaining.iter().copied().any(|candidate| {
                candidate != previous
                    && candidate != current
                    && candidate != next
                    && point_in_triangle(
                        projected[candidate],
                        projected[previous],
                        projected[current],
                        projected[next],
                        orientation,
                    )
            });
            if !contains_vertex {
                ear = Some((cursor, [previous, current, next]));
                break;
            }
        }
        let Some((cursor, triangle)) = ear else {
            return Err("polygon is self-intersecting or cannot be triangulated".into());
        };
        triangles.push(triangle);
        remaining.remove(cursor);
    }
    triangles.push([remaining[0], remaining[1], remaining[2]]);
    Ok(triangles)
}

fn project_polygon(points: &[[f64; 3]]) -> Result<Vec<[f64; 2]>, String> {
    let mut normal = [0.0; 3];
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        normal[0] += (current[1] - next[1]) * (current[2] + next[2]);
        normal[1] += (current[2] - next[2]) * (current[0] + next[0]);
        normal[2] += (current[0] - next[0]) * (current[1] + next[1]);
    }
    let drop_axis = normal
        .iter()
        .enumerate()
        .max_by(|left, right| {
            left.1
                .abs()
                .partial_cmp(&right.1.abs())
                .unwrap_or(Ordering::Equal)
        })
        .map(|(axis, _)| axis)
        .ok_or_else(|| "polygon has no normal".to_owned())?;
    if normal[drop_axis].abs() <= EPSILON {
        return Err("polygon has no stable plane normal".into());
    }
    Ok(points
        .iter()
        .map(|point| match drop_axis {
            0 => [point[1], point[2]],
            1 => [point[0], point[2]],
            _ => [point[0], point[1]],
        })
        .collect())
}

fn signed_area(points: &[[f64; 2]]) -> f64 {
    points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let next = points[(index + 1) % points.len()];
            point[0] * next[1] - next[0] * point[1]
        })
        .sum::<f64>()
        * 0.5
}

fn cross_2d(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn point_in_triangle(
    point: [f64; 2],
    a: [f64; 2],
    b: [f64; 2],
    c: [f64; 2],
    orientation: f64,
) -> bool {
    cross_2d(a, b, point) * orientation >= -EPSILON
        && cross_2d(b, c, point) * orientation >= -EPSILON
        && cross_2d(c, a, point) * orientation >= -EPSILON
}

fn generated_scene(
    buildings: &[GeneratedBuilding],
    roads: &[GeneratedRoad],
    options: &ImportOptions,
) -> SceneAsset {
    let mut objects: Vec<SceneObjectAsset> = buildings
        .iter()
        .map(|building| SceneObjectAsset {
            name: building.metadata.entity_name.clone(),
            translation_m: building.metadata.translation_m,
            rotation_rpy_rad: [0.0; 3],
            body_type: ObstacleBodyType::Fixed,
            mass_kg: 0.08,
            friction: Some(0.75),
            restitution: Some(0.0),
            visual: Some(SceneVisualAsset::Mesh {
                path: building.metadata.mesh_path.clone(),
                scale: [1.0; 3],
                color_rgba: if building.metadata.lod == 2 {
                    [1.0; 4]
                } else {
                    options.building_color_rgba
                },
            }),
            collision: Some(SceneCollisionAsset::Box {
                size_m: building.size_m,
            }),
        })
        .collect();
    objects.extend(roads.iter().map(|road| SceneObjectAsset {
        name: road.metadata.entity_name.clone(),
        translation_m: [0.0; 3],
        rotation_rpy_rad: [0.0; 3],
        body_type: ObstacleBodyType::Fixed,
        mass_kg: 0.08,
        friction: Some(0.9),
        restitution: Some(0.0),
        visual: Some(SceneVisualAsset::Mesh {
            path: road.metadata.mesh_path.clone(),
            scale: [1.0; 3],
            color_rgba: options.road_color_rgba,
        }),
        collision: None,
    }));
    SceneAsset {
        world: SceneWorldAsset {
            seed: options.world_seed,
            ..SceneWorldAsset::default()
        },
        ground: GroundAsset { enabled: true },
        robots: Vec::new(),
        obstacles: Vec::new(),
        objects,
        deformables: Vec::new(),
        task_markers: Vec::new(),
    }
}

fn sanitize_component(value: &str) -> String {
    let mut result = String::with_capacity(value.len());
    let mut previous_separator = false;
    for character in value.chars() {
        let mapped = if character.is_ascii_alphanumeric() {
            character.to_ascii_lowercase()
        } else {
            '_'
        };
        if mapped == '_' {
            if !previous_separator {
                result.push(mapped);
            }
            previous_separator = true;
        } else {
            result.push(mapped);
            previous_separator = false;
        }
    }
    let result = result.trim_matches('_');
    if result.is_empty() {
        "unnamed".into()
    } else {
        result.into()
    }
}

fn clean_zero(value: f64) -> f64 {
    if value.abs() < 0.5e-6 {
        0.0
    } else {
        value
    }
}

fn io_error(path: &Path, error: std::io::Error) -> ImportError {
    ImportError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn concave_polygon_triangulates_deterministically() {
        let polygon = vec![
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 0.0, 2.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 2.0],
        ];
        let first = triangulate_polygon(&polygon).expect("triangulate");
        let second = triangulate_polygon(&polygon).expect("repeat triangulation");
        assert_eq!(first.len(), 3);
        assert_eq!(first, second);
    }

    #[test]
    fn geographic_coordinates_map_to_y_up_local_meters() {
        let origin = SourceOrigin {
            first_deg_or_m: 35.0,
            second_deg_or_m: 139.0,
            height_m: 10.0,
        };
        let local = source_to_local(
            SourcePoint {
                first_deg_or_m: 35.000_01,
                second_deg_or_m: 139.000_01,
                height_m: 13.0,
            },
            CoordinateMode::GeographicDegrees,
            origin,
        );
        assert!(local[0] > 0.8 && local[0] < 1.0);
        assert_eq!(local[1], 3.0);
        assert!(local[2] < -1.0 && local[2] > -1.2);
    }

    #[test]
    fn auto_mode_recognizes_plateau_geographic_crs() {
        assert_eq!(
            resolve_coordinate_mode(
                CoordinateMode::Auto,
                Some("http://www.opengis.net/def/crs/EPSG/0/6697")
            ),
            CoordinateMode::GeographicDegrees
        );
        assert_eq!(
            resolve_coordinate_mode(CoordinateMode::Auto, Some("urn:ogc:def:crs:EPSG::6677")),
            CoordinateMode::ProjectedMeters
        );
    }

    #[test]
    fn polygon_holes_are_rejected_with_feature_id() {
        let xml = r#"
            <CityModel xmlns:gml="urn:gml" xmlns:bldg="urn:bldg">
              <bldg:Building gml:id="with-hole">
                <bldg:lod1Solid><gml:Solid><gml:Polygon>
                  <gml:exterior><gml:LinearRing><gml:posList>
                    0 0 0 1 0 0 1 1 0 0 0 0
                  </gml:posList></gml:LinearRing></gml:exterior>
                  <gml:interior><gml:LinearRing><gml:posList>
                    0.2 0.2 0 0.4 0.2 0 0.2 0.4 0 0.2 0.2 0
                  </gml:posList></gml:LinearRing></gml:interior>
                </gml:Polygon></gml:Solid></bldg:lod1Solid>
              </bldg:Building>
            </CityModel>
        "#;
        let document = Document::parse(xml).expect("test XML");
        let error = parse_buildings(&document, &HashMap::new()).expect_err("hole should fail");
        assert!(matches!(
            error,
            ImportError::UnsupportedGeometry { feature_id, .. } if feature_id == "with-hole"
        ));
    }

    #[test]
    fn derives_stable_opposing_lanes_from_straight_road_surface() {
        let polygon = vec![
            [-3.0, 0.0, -17.0],
            [3.0, 0.0, -17.0],
            [3.0, 0.0, 17.0],
            [-3.0, 0.0, 17.0],
        ];
        let first = derive_two_way_lanes(&polygon, "road-main", 0);
        let second = derive_two_way_lanes(&polygon, "road-main", 0);
        assert_eq!(first.len(), 2);
        assert_eq!(first[0].lane_id, "road-main/surface-0000/lane-0");
        assert!((first[0].width_m - 3.0).abs() < 1.0e-9);
        assert!((first[0].centerline_m[0][0].abs() - 1.5).abs() < 1.0e-9);
        assert!((first[1].centerline_m[0][0].abs() - 1.5).abs() < 1.0e-9);
        assert!((first[0].centerline_m[0][0] + first[1].centerline_m[0][0]).abs() < 1.0e-9);
        assert_eq!(first[0].centerline_m, second[0].centerline_m);
        assert_eq!(first[0].centerline_m[0][2], first[1].centerline_m[1][2]);
        assert_eq!(first[0].centerline_m[1][2], first[1].centerline_m[0][2]);
    }
}
