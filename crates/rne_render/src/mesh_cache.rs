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
    loaded: HashMap<String, Vec<CachedMeshPart>>,
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

            if !self.loaded.contains_key(path) {
                let file_path = resolve_mesh_path(path, package_roots)?;
                let parts = load_mesh_parts(&file_path)?
                    .into_iter()
                    .map(|part| CachedMeshPart {
                        mesh: Arc::new(part.mesh),
                        base_color_texture: part.base_color_texture.map(Arc::new),
                        base_color_rgba: part.base_color_rgba,
                        material: part.material,
                    })
                    .collect();
                self.loaded.insert(path.clone(), parts);
            }

            for part in self
                .loaded
                .get(path)
                .expect("mesh path inserted immediately above")
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
