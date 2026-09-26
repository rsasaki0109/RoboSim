//! Cached mesh loading for interactive render loops.

use crate::mesh::{load_mesh_parts, MeshLoadError, TriangleMesh};
use crate::path::resolve_package_uri;
use crate::scene::{RenderScene, RenderSceneItem};
use crate::visual::VisualShape;
use crate::{ImageFrame, PbrMaterial};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// Reuses loaded mesh geometry across frames and scene rebuilds.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshRenderCache {
    loaded: HashMap<MeshCacheKey, Vec<CachedMeshPart>>,
}

/// A mesh path plus whether it is wanted with reversed triangle winding.
///
/// A visual whose total scale has an odd number of negative components -- the
/// `scale="1 -1 1"` a URDF uses to mirror one arm's meshes onto the other side
/// -- transforms every triangle from counter-clockwise to clockwise. Back-face
/// culling then discards the surface that faces the camera. Winding-corrected
/// geometry is cached separately so both handednesses of the same file can be
/// on screen at once.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct MeshCacheKey {
    path: String,
    mirrored: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct CachedMeshPart {
    mesh: Arc<TriangleMesh>,
    base_color_texture: Option<Arc<ImageFrame>>,
    base_color_rgba: Option<[f32; 4]>,
    material: PbrMaterial,
}

impl MeshRenderCache {
    /// Creates an empty mesh cache.
    pub fn new() -> Self {
        Self::default()
    }

    /// Clears all cached mesh geometry.
    pub fn clear(&mut self) {
        self.loaded.clear();
    }

    /// Loads mesh assets referenced by a scene, reusing cached geometry and PBR
    /// materials when possible.
    ///
    /// Material-homogeneous OBJ/glTF parts are expanded in source order. This
    /// preserves base-color, normal, roughness, metallic-roughness, emissive,
    /// and occlusion textures across rebuilt animation frames.
    pub fn resolve_scene(
        &mut self,
        scene: &mut RenderScene,
        package_roots: &[&Path],
    ) -> Result<(), MeshLoadError> {
        let mut resolved_items = Vec::with_capacity(scene.items.len());
        for item in &scene.items {
            let VisualShape::Mesh { path, .. } = &item.shape else {
                resolved_items.push(item.clone());
                continue;
            };
            if item.mesh.is_some() {
                resolved_items.push(item.clone());
                continue;
            }

            let key = MeshCacheKey {
                path: path.clone(),
                mirrored: item_is_mirrored(item),
            };
            if !self.loaded.contains_key(&key) {
                let file_path = resolve_mesh_path(path, package_roots)?;
                let parts = load_mesh_parts(&file_path)?
                    .into_iter()
                    .map(|part| CachedMeshPart {
                        mesh: Arc::new(if key.mirrored {
                            reverse_winding(part.mesh)
                        } else {
                            part.mesh
                        }),
                        base_color_texture: part.base_color_texture.map(Arc::new),
                        base_color_rgba: part.base_color_rgba,
                        material: part.material,
                    })
                    .collect();
                self.loaded.insert(key.clone(), parts);
            }

            for part in self
                .loaded
                .get(&key)
                .expect("mesh key inserted immediately above")
            {
                resolved_items.push(resolve_part(item, part));
            }
        }
        scene.items = resolved_items;
        Ok(())
    }
}

fn resolve_part(item: &RenderSceneItem, part: &CachedMeshPart) -> RenderSceneItem {
    let mut resolved = item.clone();
    resolved.mesh = Some(part.mesh.clone());
    resolved.base_color_texture = part.base_color_texture.clone();
    resolved.material = part.material.clone();
    if let Some(base_color_rgba) = part.base_color_rgba {
        resolved.color_rgba = [1.0; 4];
        resolved.material.base_color_rgba = base_color_rgba;
    }
    resolved
}

/// Reports whether an item's model matrix flips handedness.
///
/// `RenderSceneItem::item_from_visual` folds the shape scale into the item
/// transform, so the transform alone carries the total scale the model matrix
/// is built from. Only the sign of the product matters: an even number of
/// negative components is a rotation and leaves winding intact.
fn item_is_mirrored(item: &RenderSceneItem) -> bool {
    let scale = item.transform.scale;
    scale.x * scale.y * scale.z < 0.0
}

/// Swaps two indices of every triangle so a mirrored model matrix restores
/// counter-clockwise winding.
///
/// Normals are left alone. They come from the vertex attribute rather than from
/// winding, and the inverse-transpose the vertex stage applies already maps
/// them to the mirrored surface's outward direction.
fn reverse_winding(mut mesh: TriangleMesh) -> TriangleMesh {
    for triangle in mesh.indices.chunks_exact_mut(3) {
        triangle.swap(1, 2);
    }
    mesh
}

fn resolve_mesh_path(uri: &str, package_roots: &[&Path]) -> Result<PathBuf, MeshLoadError> {
    for root in package_roots {
        let file_path = resolve_package_uri(uri, root);
        if file_path.is_file() {
            return Ok(file_path);
        }
    }

    Err(MeshLoadError::Io {
        path: uri.to_string(),
        message: format!("mesh not found in {} package root(s)", package_roots.len()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{RenderScene, RenderSceneItem};
    use rne_math::{Transform3 as MathTransform3, Vec3};
    use rne_world::Transform3 as WorldTransform3;

    #[test]
    fn a_mirrored_visual_gets_its_triangle_winding_reversed() {
        let package_root =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mesh_diff_drive");
        // `item_from_visual` folds the shape scale into the transform, so a
        // mirrored visual reaches the cache as a mirrored item transform.
        let item = |scale: Vec3| {
            RenderScene::item_from_visual(
                WorldTransform3::IDENTITY,
                VisualShape::Mesh {
                    path: "package://mesh_diff_drive/meshes/base_link.stl".into(),
                    scale,
                },
                [1.0; 4],
                WorldTransform3::IDENTITY,
            )
        };
        let mut cache = MeshRenderCache::new();
        let mut scene = RenderScene {
            items: vec![item(Vec3::ONE), item(Vec3::new(1.0, -1.0, 1.0))],
        };

        cache
            .resolve_scene(&mut scene, &[package_root.as_path()])
            .expect("resolve");

        // Both handednesses of one file stay resident at the same time.
        assert_eq!(cache.loaded.len(), 2);
        let upright = scene.items[0].mesh.as_ref().expect("upright mesh");
        let mirrored = scene.items[1].mesh.as_ref().expect("mirrored mesh");
        assert_eq!(upright.indices.len(), mirrored.indices.len());
        assert!(
            !upright.indices.is_empty(),
            "fixture must contain triangles for the comparison to mean anything"
        );
        for triangle in 0..upright.indices.len() / 3 {
            let base = triangle * 3;
            assert_eq!(mirrored.indices[base], upright.indices[base]);
            assert_eq!(mirrored.indices[base + 1], upright.indices[base + 2]);
            assert_eq!(mirrored.indices[base + 2], upright.indices[base + 1]);
        }
        // The normal matrix carries the other half of the correction.
        assert_eq!(upright.normals, mirrored.normals);
    }

    #[test]
    fn an_evenly_negated_scale_is_a_rotation_and_keeps_its_winding() {
        let package_root =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mesh_diff_drive");
        let mut cache = MeshRenderCache::new();
        let mut scene = RenderScene {
            items: vec![RenderScene::item_from_visual(
                WorldTransform3::IDENTITY,
                VisualShape::Mesh {
                    path: "package://mesh_diff_drive/meshes/base_link.stl".into(),
                    scale: Vec3::new(-1.0, -1.0, 1.0),
                },
                [1.0; 4],
                WorldTransform3::IDENTITY,
            )],
        };

        cache
            .resolve_scene(&mut scene, &[package_root.as_path()])
            .expect("resolve");

        assert!(cache.loaded.keys().all(|key| !key.mirrored));
    }

    #[test]
    fn cache_reuses_loaded_mesh() {
        let package_root =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mesh_diff_drive");
        let mut cache = MeshRenderCache::new();
        let mut scene = RenderScene {
            items: vec![RenderSceneItem {
                transform: MathTransform3::IDENTITY,
                shape: VisualShape::Mesh {
                    path: "package://mesh_diff_drive/meshes/base_link.stl".into(),
                    scale: Vec3::ONE,
                },
                color_rgba: [1.0, 1.0, 1.0, 1.0],
                mesh: None,
                base_color_texture: None,
                material: Default::default(),
            }],
        };

        cache
            .resolve_scene(&mut scene, &[package_root.as_path()])
            .expect("resolve");
        assert!(scene.items[0].mesh.is_some());
        assert_eq!(cache.loaded.len(), 1);

        scene.items[0].mesh = None;
        cache
            .resolve_scene(&mut scene, &[package_root.as_path()])
            .expect("resolve cached");
        assert!(scene.items[0].mesh.is_some());
    }

    #[test]
    fn cache_preserves_material_parts_and_pbr_values() {
        let package_root =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mesh_pbr_cache");
        let mut cache = MeshRenderCache::new();
        let scene_item = || RenderSceneItem {
            transform: MathTransform3::IDENTITY,
            shape: VisualShape::Mesh {
                path: "package://mesh_pbr_cache/panel.obj".into(),
                scale: Vec3::ONE,
            },
            color_rgba: [0.2, 0.3, 0.4, 1.0],
            mesh: None,
            base_color_texture: None,
            material: Default::default(),
        };
        let mut scene = RenderScene {
            items: vec![scene_item()],
        };

        cache
            .resolve_scene(&mut scene, &[package_root.as_path()])
            .expect("resolve material parts");
        assert!(!scene.items.is_empty());
        assert_eq!(scene.items.len(), 2);
        assert!(scene.items.iter().all(|item| item.mesh.is_some()));
        assert_ne!(scene.items[0].material, PbrMaterial::default());
        let first_material = scene.items[0].material.clone();
        let first_texture = scene.items[0].base_color_texture.clone();

        let mut replay = RenderScene {
            items: vec![scene_item()],
        };
        cache
            .resolve_scene(&mut replay, &[package_root.as_path()])
            .expect("resolve cached material parts");
        assert_eq!(replay.items[0].material, first_material);
        assert_eq!(replay.items[0].base_color_texture, first_texture);
    }
}
