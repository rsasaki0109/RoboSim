//! Visual-only robot model manifests.
//!
//! The manifest in this module describes render geometry attached to existing
//! robot links.  It deliberately contains no collision, joint, or inertial
//! settings: those remain owned by the robot's URDF and `.rne.robot.toml`
//! asset.

use crate::error::AssetError;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};

/// Current version of the `mm_mobile_lift` visual-only manifest schema.
pub const MM_MOBILE_LIFT_VISUAL_MANIFEST_VERSION: u32 = 1;

/// Model name accepted by the version 1 visual-only contract.
pub const MM_MOBILE_LIFT_MODEL_NAME: &str = "mm_mobile_lift";

/// Coordinate convention required by the version 1 visual-only contract.
pub const MM_MOBILE_LIFT_COORDINATE_SYSTEM: &str = "rne_y_up_x_forward";

/// Required link names in the `mm_mobile_lift` visual contract.
pub const MM_MOBILE_LIFT_REQUIRED_LINKS: [&str; 10] = [
    "base_link",
    "left_wheel",
    "right_wheel",
    "torso_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_link",
    "gripper_base_link",
    "left_finger_link",
    "right_finger_link",
];

/// Parsed and validated visual-only manifest for `mm_mobile_lift`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualManifest {
    /// Manifest schema version.
    pub schema_version: u32,
    /// Robot model name this visual contract attaches to.
    pub robot_model: String,
    /// Must be `true`; visual manifests cannot define physics.
    pub visual_only: bool,
    /// Coordinate convention used by the visual meshes.
    pub coordinate_system: String,
    /// Optional companion physics asset for human/tooling provenance.
    #[serde(default)]
    pub physics_asset: Option<String>,
    /// Optional provenance document for the visual files.
    #[serde(default)]
    pub provenance: Option<String>,
    /// Triangle, material, and texture budgets for the visual model.
    pub budget: VisualBudget,
    /// Per-link visual mesh assignments.
    pub links: Vec<VisualLink>,
}

impl VisualManifest {
    /// Validates this manifest and all referenced mesh files.
    pub fn validate(&self, manifest_path: &Path) -> Result<(), AssetError> {
        validate_visual_manifest(manifest_path, self)
    }
}

/// Resource budgets enforced while loading a visual-only manifest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualBudget {
    /// Maximum aggregate triangle count of unique LOD0 meshes.
    pub max_lod0_triangles: u64,
    /// Maximum aggregate triangle count of unique LOD1 meshes.
    pub max_lod1_triangles: u64,
    /// Maximum width or height of a decoded texture in pixels.
    pub max_texture_size_px: u32,
    /// Maximum aggregate decoded RGBA8 texture bytes.
    pub max_texture_bytes: u64,
    /// Maximum number of material-homogeneous parts across unique meshes.
    pub max_materials: u32,
}

/// One visual mesh assignment for a robot link.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VisualLink {
    /// Exact URDF link name receiving this visual mesh.
    pub name: String,
    /// Relative LOD0 mesh path, normally a `.glb` file.
    pub mesh: String,
    /// Optional relative LOD1 mesh path.
    #[serde(default)]
    pub lod1_mesh: Option<String>,
    /// Positive finite mesh scale in the link visual frame.
    #[serde(default = "default_visual_scale")]
    pub scale: [f64; 3],
    /// Whether this link is required by the contract.
    #[serde(default = "default_required_link")]
    pub required: bool,
}

fn default_visual_scale() -> [f64; 3] {
    [1.0, 1.0, 1.0]
}

fn default_required_link() -> bool {
    true
}

/// Loads and validates a visual-only manifest from disk.
pub fn load_visual_manifest(path: &Path) -> Result<VisualManifest, AssetError> {
    let text = fs::read_to_string(path).map_err(|error| AssetError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    parse_visual_manifest(path, &text)
}

/// Parses and validates a visual-only manifest from TOML text.
///
/// Mesh paths are resolved relative to `path`'s parent, so callers can use a
/// virtual path in tests as long as its referenced fixture files exist.
pub fn parse_visual_manifest(path: &Path, text: &str) -> Result<VisualManifest, AssetError> {
    let manifest: VisualManifest = toml::from_str(text).map_err(|error| {
        AssetError::invalid(path.display().to_string(), format!("TOML: {error}"))
    })?;
    validate_visual_manifest(path, &manifest)?;
    Ok(manifest)
}

/// Validates a parsed visual-only manifest and all of its mesh references.
pub fn validate_visual_manifest(
    manifest_path: &Path,
    manifest: &VisualManifest,
) -> Result<(), AssetError> {
    let invalid =
        |message: String| AssetError::invalid(manifest_path.display().to_string(), message);

    if manifest.schema_version != MM_MOBILE_LIFT_VISUAL_MANIFEST_VERSION {
        return Err(invalid(format!(
            "unsupported visual manifest schema_version {}; expected {}",
            manifest.schema_version, MM_MOBILE_LIFT_VISUAL_MANIFEST_VERSION
        )));
    }
    if manifest.robot_model != MM_MOBILE_LIFT_MODEL_NAME {
        return Err(invalid(format!(
            "visual manifest robot_model must be `{MM_MOBILE_LIFT_MODEL_NAME}`, got `{}`",
            manifest.robot_model
        )));
    }
    if !manifest.visual_only {
        return Err(invalid("visual_only must be true".to_string()));
    }
    if manifest.coordinate_system != MM_MOBILE_LIFT_COORDINATE_SYSTEM {
        return Err(invalid(format!(
            "unsupported coordinate_system `{}`; expected `{MM_MOBILE_LIFT_COORDINATE_SYSTEM}`",
            manifest.coordinate_system
        )));
    }
    if let Some(physics_asset) = &manifest.physics_asset {
        resolve_auxiliary_reference(manifest_path, "physics_asset", physics_asset)?;
    }
    if let Some(provenance) = &manifest.provenance {
        resolve_auxiliary_reference(manifest_path, "provenance", provenance)?;
    }
    validate_budget(manifest_path, &manifest.budget)?;

    let mut seen_names = BTreeSet::new();
    let mut lod0_paths = BTreeSet::new();
    let mut lod1_paths = BTreeSet::new();

    for (index, link) in manifest.links.iter().enumerate() {
        if link.name.trim().is_empty() {
            return Err(invalid(format!("links[{index}].name must not be empty")));
        }
        if !MM_MOBILE_LIFT_REQUIRED_LINKS.contains(&link.name.as_str()) {
            return Err(invalid(format!(
                "links[{index}] contains unknown required link `{}`",
                link.name
            )));
        }
        if !seen_names.insert(link.name.as_str()) {
            return Err(invalid(format!(
                "links[{index}] duplicates link `{}`",
                link.name
            )));
        }
        if !link.required {
            return Err(invalid(format!(
                "links[{index}] required link `{}` must set required = true",
                link.name
            )));
        }
        validate_scale(manifest_path, index, &link.scale)?;

        let lod0_path = resolve_mesh_reference(manifest_path, index, "mesh", &link.mesh)?;
        lod0_paths.insert(lod0_path);
        if let Some(lod1_mesh) = &link.lod1_mesh {
            let lod1_path = resolve_mesh_reference(manifest_path, index, "lod1_mesh", lod1_mesh)?;
            lod1_paths.insert(lod1_path);
        }
    }

    for required_name in MM_MOBILE_LIFT_REQUIRED_LINKS {
        if !seen_names.contains(required_name) {
            return Err(invalid(format!("missing required link `{required_name}`")));
        }
    }

    let all_mesh_paths = lod0_paths
        .union(&lod1_paths)
        .cloned()
        .collect::<BTreeSet<_>>();
    let mut mesh_stats = BTreeMap::new();
    for mesh_path in &all_mesh_paths {
        mesh_stats.insert(mesh_path.clone(), inspect_mesh(manifest_path, mesh_path)?);
    }

    let lod0_triangles = aggregate_triangles(&lod0_paths, &mesh_stats, manifest_path)?;
    if lod0_triangles > manifest.budget.max_lod0_triangles {
        return Err(invalid(format!(
            "LOD0 triangle budget exceeded: {lod0_triangles} > {}",
            manifest.budget.max_lod0_triangles
        )));
    }
    let lod1_triangles = aggregate_triangles(&lod1_paths, &mesh_stats, manifest_path)?;
    if lod1_triangles > manifest.budget.max_lod1_triangles {
        return Err(invalid(format!(
            "LOD1 triangle budget exceeded: {lod1_triangles} > {}",
            manifest.budget.max_lod1_triangles
        )));
    }

    let mut material_count = 0_u64;
    let mut texture_bytes = 0_u64;
    let mut max_texture_size_px = 0_u32;
    for stats in mesh_stats.values() {
        material_count = material_count
            .checked_add(stats.material_count)
            .ok_or_else(|| invalid("material count overflow".to_string()))?;
        texture_bytes = texture_bytes
            .checked_add(stats.texture_bytes)
            .ok_or_else(|| invalid("texture byte count overflow".to_string()))?;
        max_texture_size_px = max_texture_size_px.max(stats.max_texture_size_px);
    }
    if material_count > u64::from(manifest.budget.max_materials) {
        return Err(invalid(format!(
            "material budget exceeded: {material_count} > {}",
            manifest.budget.max_materials
        )));
    }
    if texture_bytes > manifest.budget.max_texture_bytes {
        return Err(invalid(format!(
            "texture byte budget exceeded: {texture_bytes} > {}",
            manifest.budget.max_texture_bytes
        )));
    }
    if max_texture_size_px > manifest.budget.max_texture_size_px {
        return Err(invalid(format!(
            "texture size budget exceeded: {max_texture_size_px}px > {}px",
            manifest.budget.max_texture_size_px
        )));
    }

    Ok(())
}

fn validate_budget(manifest_path: &Path, budget: &VisualBudget) -> Result<(), AssetError> {
    let invalid = |field: &str| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!("budget.{field} must be positive"),
        )
    };
    if budget.max_lod0_triangles == 0 {
        return Err(invalid("max_lod0_triangles"));
    }
    if budget.max_lod1_triangles == 0 {
        return Err(invalid("max_lod1_triangles"));
    }
    if budget.max_texture_size_px == 0 {
        return Err(invalid("max_texture_size_px"));
    }
    if budget.max_texture_bytes == 0 {
        return Err(invalid("max_texture_bytes"));
    }
    if budget.max_materials == 0 {
        return Err(invalid("max_materials"));
    }
    Ok(())
}

fn validate_scale(manifest_path: &Path, index: usize, scale: &[f64; 3]) -> Result<(), AssetError> {
    if scale
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err(AssetError::invalid(
            manifest_path.display().to_string(),
            format!("links[{index}].scale must contain only finite positive values"),
        ));
    }
    Ok(())
}

fn resolve_mesh_reference(
    manifest_path: &Path,
    index: usize,
    field: &str,
    value: &str,
) -> Result<PathBuf, AssetError> {
    let relative = validate_mesh_path(manifest_path, index, field, value)?;
    let root = manifest_path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let candidate = root.join(relative);
    if !candidate.is_file() {
        return Err(AssetError::invalid(
            manifest_path.display().to_string(),
            format!(
                "links[{index}].{field} mesh not found: {}",
                candidate.display()
            ),
        ));
    }

    let canonical_root = fs::canonicalize(root).map_err(|error| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!(
                "could not resolve visual asset root `{}`: {error}",
                root.display()
            ),
        )
    })?;
    let canonical_mesh = fs::canonicalize(&candidate).map_err(|error| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!("could not resolve mesh `{}`: {error}", candidate.display()),
        )
    })?;
    if !canonical_mesh.starts_with(&canonical_root) {
        return Err(AssetError::invalid(
            manifest_path.display().to_string(),
            format!(
                "links[{index}].{field} resolves outside the manifest directory: {}",
                candidate.display()
            ),
        ));
    }
    Ok(canonical_mesh)
}

fn resolve_auxiliary_reference(
    manifest_path: &Path,
    field: &str,
    value: &str,
) -> Result<PathBuf, AssetError> {
    let relative = validate_relative_reference(manifest_path, field, value)?;
    let root = manifest_path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let candidate = root.join(relative);
    if !candidate.is_file() {
        return Err(AssetError::invalid(
            manifest_path.display().to_string(),
            format!("{field} file not found: {}", candidate.display()),
        ));
    }
    let canonical_root = fs::canonicalize(root).map_err(|error| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!(
                "could not resolve visual asset root {}: {error}",
                root.display()
            ),
        )
    })?;
    let canonical_file = fs::canonicalize(&candidate).map_err(|error| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!("could not resolve {field} {}: {error}", candidate.display()),
        )
    })?;
    if !canonical_file.starts_with(&canonical_root) {
        return Err(AssetError::invalid(
            manifest_path.display().to_string(),
            format!(
                "{field} resolves outside the manifest directory: {}",
                candidate.display()
            ),
        ));
    }
    Ok(canonical_file)
}

fn validate_relative_reference(
    manifest_path: &Path,
    field: &str,
    value: &str,
) -> Result<PathBuf, AssetError> {
    let invalid = |message: String| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!("{field} {message}"),
        )
    };
    if value.trim().is_empty() {
        return Err(invalid("must not be empty".to_string()));
    }
    if value.contains('\0') {
        return Err(invalid("must not contain NUL".to_string()));
    }
    if value.contains('\\') {
        return Err(invalid("must use forward-slash relative paths".to_string()));
    }
    if value.as_bytes().get(1) == Some(&b':') {
        return Err(invalid("must not use a drive-qualified path".to_string()));
    }
    let path = Path::new(value);
    if path.is_absolute() {
        return Err(invalid("must be relative to the manifest".to_string()));
    }
    for component in path.components() {
        match component {
            Component::Normal(_) => {}
            Component::CurDir | Component::ParentDir => {
                return Err(invalid("must not contain dot path components".to_string()));
            }
            Component::RootDir | Component::Prefix(_) => {
                return Err(invalid("must be a relative path".to_string()));
            }
        }
    }
    Ok(path.to_path_buf())
}

fn validate_mesh_path(
    manifest_path: &Path,
    index: usize,
    field: &str,
    value: &str,
) -> Result<PathBuf, AssetError> {
    let _ = validate_relative_reference(manifest_path, &format!("links[{index}].{field}"), value)?;
    let invalid = |message: String| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!("links[{index}].{field} {message}"),
        )
    };
    if value.trim().is_empty() {
        return Err(invalid("must not be empty".to_string()));
    }
    if value.contains('\0') {
        return Err(invalid("must not contain NUL".to_string()));
    }
    if value.contains('\\') {
        return Err(invalid("must use forward-slash relative paths".to_string()));
    }
    if value.as_bytes().get(1) == Some(&b':') {
        return Err(invalid("must not use a drive-qualified path".to_string()));
    }

    let path = Path::new(value);
    if path.is_absolute() {
        return Err(invalid("must be relative to the manifest".to_string()));
    }
    for component in path.components() {
        match component {
            Component::Normal(_) => {}
            Component::CurDir | Component::ParentDir => {
                return Err(invalid(
                    "must not contain `.` or `..` path components".to_string(),
                ));
            }
            Component::RootDir | Component::Prefix(_) => {
                return Err(invalid("must be a relative path".to_string()));
            }
        }
    }
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .map(str::to_ascii_lowercase);
    if !matches!(extension.as_deref(), Some("glb" | "gltf" | "obj" | "stl")) {
        return Err(invalid(
            "must use a supported .glb, .gltf, .obj, or .stl mesh extension".to_string(),
        ));
    }
    Ok(path.to_path_buf())
}

#[derive(Clone, Copy, Debug, Default)]
struct MeshStats {
    triangle_count: u64,
    material_count: u64,
    texture_bytes: u64,
    max_texture_size_px: u32,
}

fn inspect_mesh(manifest_path: &Path, mesh_path: &Path) -> Result<MeshStats, AssetError> {
    let parts = rne_render::load_mesh_parts(mesh_path).map_err(|error| {
        AssetError::invalid(
            manifest_path.display().to_string(),
            format!("invalid visual mesh `{}`: {error}", mesh_path.display()),
        )
    })?;
    if parts.is_empty() {
        return Err(AssetError::invalid(
            manifest_path.display().to_string(),
            format!("visual mesh `{}` contains no parts", mesh_path.display()),
        ));
    }

    let mut stats = MeshStats {
        material_count: u64::try_from(parts.len()).map_err(|_| {
            AssetError::invalid(
                manifest_path.display().to_string(),
                format!(
                    "visual mesh `{}` has too many material parts",
                    mesh_path.display()
                ),
            )
        })?,
        ..MeshStats::default()
    };
    for part in parts {
        stats.triangle_count = stats
            .triangle_count
            .checked_add(u64::try_from(part.mesh.triangle_count()).map_err(|_| {
                AssetError::invalid(
                    manifest_path.display().to_string(),
                    format!(
                        "visual mesh `{}` has too many triangles",
                        mesh_path.display()
                    ),
                )
            })?)
            .ok_or_else(|| {
                AssetError::invalid(
                    manifest_path.display().to_string(),
                    format!(
                        "visual mesh `{}` triangle count overflow",
                        mesh_path.display()
                    ),
                )
            })?;
        if let Some(texture) = part.base_color_texture {
            stats.texture_bytes = stats
                .texture_bytes
                .checked_add(u64::try_from(texture.rgba8.len()).map_err(|_| {
                    AssetError::invalid(
                        manifest_path.display().to_string(),
                        format!("visual mesh `{}` texture is too large", mesh_path.display()),
                    )
                })?)
                .ok_or_else(|| {
                    AssetError::invalid(
                        manifest_path.display().to_string(),
                        format!(
                            "visual mesh `{}` texture byte count overflow",
                            mesh_path.display()
                        ),
                    )
                })?;
            stats.max_texture_size_px = stats
                .max_texture_size_px
                .max(texture.width.max(texture.height));
        }
        for texture in [
            part.material.normal_texture.as_deref(),
            part.material.roughness_texture.as_deref(),
            part.material.metallic_roughness_texture.as_deref(),
            part.material.emissive_texture.as_deref(),
            part.material.occlusion_texture.as_deref(),
        ]
        .into_iter()
        .flatten()
        {
            stats.texture_bytes = stats
                .texture_bytes
                .checked_add(u64::try_from(texture.rgba8.len()).map_err(|_| {
                    AssetError::invalid(
                        manifest_path.display().to_string(),
                        format!("visual mesh {} texture is too large", mesh_path.display()),
                    )
                })?)
                .ok_or_else(|| {
                    AssetError::invalid(
                        manifest_path.display().to_string(),
                        format!(
                            "visual mesh {} texture byte count overflow",
                            mesh_path.display()
                        ),
                    )
                })?;
            stats.max_texture_size_px = stats
                .max_texture_size_px
                .max(texture.width.max(texture.height));
        }
    }
    Ok(stats)
}

fn aggregate_triangles(
    paths: &BTreeSet<PathBuf>,
    stats: &BTreeMap<PathBuf, MeshStats>,
    manifest_path: &Path,
) -> Result<u64, AssetError> {
    paths.iter().try_fold(0_u64, |total, path| {
        total
            .checked_add(stats.get(path).map_or(0, |mesh| mesh.triangle_count))
            .ok_or_else(|| {
                AssetError::invalid(
                    manifest_path.display().to_string(),
                    "triangle count overflow",
                )
            })
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn fixture_path(file_name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/mm_mobile_lift_visual")
            .join(file_name)
    }

    fn fixture_text() -> String {
        fs::read_to_string(fixture_path("mm_mobile_lift.visual.toml")).unwrap()
    }

    #[test]
    fn loads_valid_mm_mobile_lift_visual_manifest() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let manifest = load_visual_manifest(&path).unwrap();
        assert_eq!(manifest.schema_version, 1);
        assert_eq!(manifest.links.len(), MM_MOBILE_LIFT_REQUIRED_LINKS.len());
    }

    #[test]
    fn shipped_mm_mobile_lift_visual_pack_is_valid() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/robots/mm_mobile_lift/mm_mobile_lift.visual.toml");
        let manifest = load_visual_manifest(&path).expect("shipped visual pack");
        assert_eq!(manifest.robot_model, MM_MOBILE_LIFT_MODEL_NAME);
        assert_eq!(manifest.links.len(), MM_MOBILE_LIFT_REQUIRED_LINKS.len());
        for link in &manifest.links {
            let lod0 = path.parent().unwrap().join(&link.mesh);
            let parts = rne_render::load_mesh_parts(&lod0).expect("load authored GLB");
            assert!(
                parts.len() >= 2,
                "{} should retain multiple materials",
                link.name
            );
            assert!(parts.iter().all(|part| {
                part.base_color_texture.is_some()
                    && part.material.normal_texture.is_some()
                    && part.material.metallic_roughness_texture.is_some()
                    && part.material.emissive_texture.is_some()
                    && part.material.occlusion_texture.is_some()
            }));
            let lod1 = path
                .parent()
                .unwrap()
                .join(link.lod1_mesh.as_ref().unwrap());
            assert!(
                rne_render::load_mesh_parts(&lod1)
                    .expect("load authored LOD1 GLB")
                    .len()
                    >= 2
            );
        }
    }

    #[test]
    fn rejects_unknown_link() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let text = fixture_text().replacen("name = \"base_link\"", "name = \"unknown_link\"", 1);
        let error = parse_visual_manifest(&path, &text).unwrap_err();
        assert!(error.to_string().contains("unknown required link"));
    }

    #[test]
    fn rejects_duplicate_link() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let text = fixture_text().replacen("name = \"left_wheel\"", "name = \"base_link\"", 1);
        let error = parse_visual_manifest(&path, &text).unwrap_err();
        assert!(error.to_string().contains("duplicates link"));
    }

    #[test]
    fn rejects_missing_required_link() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let text = fixture_text().replacen(
            "[[links]]\nname = \"right_finger_link\"\nmesh = \"meshes/tiny.stl\"\nrequired = true\n",
            "",
            1,
        );
        let error = parse_visual_manifest(&path, &text).unwrap_err();
        assert!(error.to_string().contains("missing required link"));
    }

    #[test]
    fn rejects_path_traversal_and_missing_mesh() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let traversal =
            fixture_text().replacen("mesh = \"meshes/tiny.stl\"", "mesh = \"../tiny.stl\"", 1);
        let error = parse_visual_manifest(&path, &traversal).unwrap_err();
        assert!(error.to_string().contains("must not contain"));

        let missing = fixture_text().replacen(
            "mesh = \"meshes/tiny.stl\"",
            "mesh = \"meshes/missing.stl\"",
            1,
        );
        let error = parse_visual_manifest(&path, &missing).unwrap_err();
        assert!(error.to_string().contains("mesh not found"));
    }

    #[test]
    fn rejects_non_finite_or_non_positive_scale() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let text = fixture_text().replacen(
            "mesh = \"meshes/tiny.stl\"",
            "mesh = \"meshes/tiny.stl\"\nscale = [1.0, 0.0, 1.0]",
            1,
        );
        let error = parse_visual_manifest(&path, &text).unwrap_err();
        assert!(error.to_string().contains("finite positive"));
    }

    #[test]
    fn rejects_triangle_budget() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let text = fixture_text().replace("max_lod0_triangles = 2", "max_lod0_triangles = 1");
        let error = parse_visual_manifest(&path, &text).unwrap_err();
        assert!(error.to_string().contains("LOD0 triangle budget exceeded"));
    }

    #[test]
    fn rejects_zero_budget_and_unknown_fields() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let zero_budget = fixture_text().replace("max_materials = 1", "max_materials = 0");
        let error = parse_visual_manifest(&path, &zero_budget).unwrap_err();
        assert!(error
            .to_string()
            .contains("budget.max_materials must be positive"));

        let unknown = format!("{}\nextra = true\n", fixture_text());
        let error = parse_visual_manifest(&path, &unknown).unwrap_err();
        assert!(error.to_string().contains("unknown field"));
    }

    #[test]
    fn validates_optional_provenance_reference_containment_and_existence() {
        let path = fixture_path("mm_mobile_lift.visual.toml");
        let existing = format!(
            "provenance = \"mm_mobile_lift.visual.toml\"\n{}",
            fixture_text()
        );
        parse_visual_manifest(&path, &existing).expect("existing provenance");

        let missing = format!("provenance = \"missing.md\"\n{}", fixture_text());
        let error = parse_visual_manifest(&path, &missing).unwrap_err();
        assert!(error.to_string().contains("provenance file not found"));

        let escaped = format!("provenance = \"../provenance.md\"\n{}", fixture_text());
        let error = parse_visual_manifest(&path, &escaped).unwrap_err();
        assert!(error.to_string().contains("must not contain"));
    }
}
