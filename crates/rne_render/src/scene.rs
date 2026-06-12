//! Primitive scene description for rendering.

use crate::mesh::{load_stl, MeshLoadError, TriangleMesh};
use crate::path::resolve_package_uri;
use crate::visual::VisualShape;
use rne_math::Transform3 as MathTransform3;
use rne_world::Transform3;
use std::path::Path;
use std::path::PathBuf;
use std::sync::Arc;

/// One renderable primitive in a scene pass.
#[derive(Clone, Debug, PartialEq)]
pub struct RenderSceneItem {
    /// World-space transform including shape scale.
    pub transform: MathTransform3,
    /// Primitive shape.
    pub shape: VisualShape,
    /// RGBA color in linear space.
    pub color_rgba: [f32; 4],
    /// Loaded mesh geometry for [`VisualShape::Mesh`] items.
    pub mesh: Option<Arc<TriangleMesh>>,
}

/// Collection of primitives rendered in one camera pass.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RenderScene {
    /// Draw items in arbitrary order.
    pub items: Vec<RenderSceneItem>,
}

impl RenderScene {
    /// Creates an empty scene.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builds a render item from a world transform and visual component.
    pub fn item_from_visual(
        world: Transform3,
        shape: VisualShape,
        color_rgba: [f32; 4],
        local_offset: Transform3,
    ) -> RenderSceneItem {
        let world = to_math_transform(world);
        let local = to_math_transform(local_offset);
        let base = world.mul_transform(&local);
        RenderSceneItem {
            transform: apply_shape_scale(base, &shape),
            shape,
            color_rgba,
            mesh: None,
        }
    }

    /// Loads STL files referenced by mesh visuals in this scene.
    pub fn resolve_mesh_assets(&mut self, package_root: &Path) -> Result<(), MeshLoadError> {
        self.resolve_mesh_assets_with_roots(&[package_root])
    }

    /// Loads STL files using the first package root that resolves each mesh URI.
    pub fn resolve_mesh_assets_with_roots(
        &mut self,
        package_roots: &[&Path],
    ) -> Result<(), MeshLoadError> {
        for item in &mut self.items {
            let VisualShape::Mesh { path, .. } = &item.shape else {
                continue;
            };
            let file_path = resolve_mesh_path(path, package_roots)?;
            item.mesh = Some(Arc::new(load_stl(&file_path)?));
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

fn to_math_transform(transform: Transform3) -> MathTransform3 {
    MathTransform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    }
}

fn apply_shape_scale(mut transform: MathTransform3, shape: &VisualShape) -> MathTransform3 {
    let shape_scale = match shape {
        VisualShape::Box { size_m } => *size_m,
        VisualShape::Sphere { radius_m } => {
            let diameter = radius_m * 2.0;
            rne_math::Vec3::splat(diameter)
        }
        VisualShape::Cylinder { radius_m, length_m } => {
            rne_math::Vec3::new(radius_m * 2.0, radius_m * 2.0, *length_m)
        }
        VisualShape::Mesh { scale, .. } => *scale,
    };
    transform.scale = rne_math::Vec3::new(
        transform.scale.x * shape_scale.x,
        transform.scale.y * shape_scale.y,
        transform.scale.z * shape_scale.z,
    );
    transform
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::Vec3;
    use rne_world::Transform3 as WorldTransform3;
    use std::path::PathBuf;

    #[test]
    fn box_visual_applies_size_scale() {
        let item = RenderScene::item_from_visual(
            WorldTransform3::IDENTITY,
            VisualShape::Box {
                size_m: Vec3::new(2.0, 1.0, 0.5),
            },
            [1.0, 1.0, 1.0, 1.0],
            WorldTransform3::IDENTITY,
        );
        assert_eq!(item.transform.scale, Vec3::new(2.0, 1.0, 0.5));
    }

    #[test]
    fn resolve_mesh_assets_loads_stl() {
        let package_root =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/mesh_diff_drive");
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
        scene
            .resolve_mesh_assets(&package_root)
            .expect("resolve mesh");
        assert!(scene.items[0].mesh.is_some());
    }
}
