//! Cached mesh loading for interactive render loops.

use crate::mesh::{load_mesh, MeshLoadError, TriangleMesh};
use crate::path::resolve_package_uri;
use crate::scene::RenderScene;
use crate::visual::VisualShape;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// Reuses loaded mesh geometry across frames and scene rebuilds.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshRenderCache {
    loaded: HashMap<String, Arc<TriangleMesh>>,
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

    /// Loads mesh assets referenced by a scene, reusing cached geometry when possible.
    pub fn resolve_scene(
        &mut self,
        scene: &mut RenderScene,
        package_roots: &[&Path],
    ) -> Result<(), MeshLoadError> {
        for item in &mut scene.items {
            let VisualShape::Mesh { path, .. } = &item.shape else {
                continue;
            };

            if let Some(mesh) = self.loaded.get(path) {
                item.mesh = Some(mesh.clone());
                continue;
            }

            let file_path = resolve_mesh_path(path, package_roots)?;
            let mesh = Arc::new(load_mesh(&file_path)?);
            self.loaded.insert(path.clone(), mesh.clone());
            item.mesh = Some(mesh);
        }
        Ok(())
    }
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
}
