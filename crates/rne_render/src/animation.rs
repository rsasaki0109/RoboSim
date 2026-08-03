//! Backend-neutral glTF node animation and CPU skinning.

use crate::mesh::{LoadedMeshPart, TriangleMesh};
use rne_math::{Mat4, Quat, Transform3, Vec3};
use thiserror::Error;

/// The supported interpolation modes for glTF keyframes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnimationInterpolation {
    /// Interpolate between adjacent keyframes.
    Linear,
    /// Hold the previous keyframe until the next keyframe.
    Step,
}

/// The local transform property targeted by an animation channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnimationProperty {
    /// Local translation in meters.
    Translation,
    /// Local rotation as an `(x, y, z, w)` unit quaternion.
    Rotation,
    /// Local non-uniform scale.
    Scale,
}

/// One node-local animation channel.
#[derive(Clone, Debug, PartialEq)]
pub struct AnimationChannel {
    /// Index of the targeted node in [`GltfSceneAsset::nodes`].
    pub node_index: usize,
    /// Transform property targeted by this channel.
    pub property: AnimationProperty,
    /// Keyframe times in seconds, in ascending order.
    pub times_s: Vec<f32>,
    /// Keyframe values. Translation and scale use the first three values;
    /// rotation uses all four values as `(x, y, z, w)`.
    pub values: Vec<[f32; 4]>,
    /// Interpolation used between keyframes.
    pub interpolation: AnimationInterpolation,
}

impl AnimationChannel {
    /// Samples the channel at a time in seconds.
    ///
    /// Values outside the channel's keyframe range are clamped to its first or
    /// last keyframe. Looping is handled by [`AnimationClip`].
    pub fn sample_value(&self, time_s: f32) -> [f32; 4] {
        if self.times_s.is_empty() || self.values.is_empty() {
            return [0.0; 4];
        }
        let count = self.times_s.len().min(self.values.len());
        if count == 1 || time_s <= self.times_s[0] {
            return self.values[0];
        }
        if time_s >= self.times_s[count - 1] {
            return self.values[count - 1];
        }

        let next = self.times_s[..count].partition_point(|time| *time <= time_s);
        let previous = next.saturating_sub(1);
        if self.interpolation == AnimationInterpolation::Step {
            return self.values[previous];
        }

        let start_s = self.times_s[previous];
        let end_s = self.times_s[next];
        let alpha = if end_s > start_s {
            ((time_s - start_s) / (end_s - start_s)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let start = self.values[previous];
        let end = self.values[next];
        std::array::from_fn(|index| start[index] + (end[index] - start[index]) * alpha)
    }
}

/// A named or unnamed glTF keyframe animation.
#[derive(Clone, Debug, PartialEq)]
pub struct AnimationClip {
    /// Optional source animation name.
    pub name: Option<String>,
    /// Duration in seconds, derived from the latest channel keyframe.
    pub duration_s: f32,
    /// Node-local animation channels.
    pub channels: Vec<AnimationChannel>,
}

impl AnimationClip {
    /// Returns the looping time used for sampling this clip.
    pub fn looped_time_s(&self, time_s: f32) -> f32 {
        if self.duration_s > 0.0 && self.duration_s.is_finite() {
            time_s.rem_euclid(self.duration_s)
        } else {
            0.0
        }
    }

    /// Samples all node-local transforms from their bind-pose values.
    pub fn sample_node_transforms(&self, nodes: &[GltfNode], time_s: f32) -> Vec<Transform3> {
        let mut transforms = nodes
            .iter()
            .map(|node| node.bind_transform)
            .collect::<Vec<_>>();
        let sample_time_s = self.looped_time_s(time_s);
        for channel in &self.channels {
            let Some(transform) = transforms.get_mut(channel.node_index) else {
                continue;
            };
            let value = channel.sample_value(sample_time_s);
            match channel.property {
                AnimationProperty::Translation => {
                    transform.translation = Vec3::new(
                        f64::from(value[0]),
                        f64::from(value[1]),
                        f64::from(value[2]),
                    );
                }
                AnimationProperty::Rotation => {
                    let rotation = Quat::from_xyzw(
                        f64::from(value[0]),
                        f64::from(value[1]),
                        f64::from(value[2]),
                        f64::from(value[3]),
                    );
                    if rotation.length_squared() > 1.0e-12 {
                        transform.rotation = rotation.normalize();
                    }
                }
                AnimationProperty::Scale => {
                    transform.scale = Vec3::new(
                        f64::from(value[0]),
                        f64::from(value[1]),
                        f64::from(value[2]),
                    );
                }
            }
        }
        transforms
    }
}

/// One node in the imported glTF hierarchy.
#[derive(Clone, Debug, PartialEq)]
pub struct GltfNode {
    /// Optional source node name.
    pub name: Option<String>,
    /// Parent node index, if the node is not a scene root.
    pub parent_index: Option<usize>,
    /// Node-local bind-pose transform.
    pub bind_transform: Transform3,
}

/// A joint and its inverse-bind matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct GltfSkinJoint {
    /// Node index containing this joint.
    pub node_index: usize,
    /// Inverse-bind matrix supplied by the glTF skin, or identity when absent.
    pub inverse_bind_matrix: Mat4,
}

/// A glTF skin binding.
#[derive(Clone, Debug, PartialEq)]
pub struct GltfSkin {
    /// Joints in the order referenced by `JOINTS_0` vertex attributes.
    pub joints: Vec<GltfSkinJoint>,
}

/// Up to four joint influences for each vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct SkinWeights {
    /// Joint indices aligned with the source mesh vertices.
    pub joints: Vec<[u16; 4]>,
    /// Joint weights aligned with the source mesh vertices.
    pub weights: Vec<[f32; 4]>,
}

/// A material part from a glTF scene with its node and optional skin binding.
#[derive(Clone, Debug, PartialEq)]
pub struct GltfScenePart {
    /// Raw mesh and material data before the node transform is applied.
    pub render_part: LoadedMeshPart,
    /// Node that instantiates the mesh.
    pub node_index: usize,
    /// Skin index referenced by the mesh node.
    pub skin_index: Option<usize>,
    /// Per-vertex skin influences, when the mesh is skinned.
    pub skin_weights: Option<SkinWeights>,
}

/// Imported glTF hierarchy, skins, animation clips, and material parts.
#[derive(Clone, Debug, PartialEq)]
pub struct GltfSceneAsset {
    /// All nodes from the source document.
    pub nodes: Vec<GltfNode>,
    /// All skins from the source document.
    pub skins: Vec<GltfSkin>,
    /// All transform animations from the source document.
    pub animations: Vec<AnimationClip>,
    /// Mesh parts in deterministic scene traversal order.
    pub parts: Vec<GltfScenePart>,
}

/// Errors produced while sampling an imported glTF scene.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum AnimationSampleError {
    /// The requested mesh part does not exist.
    #[error("glTF mesh part index {index} is out of bounds")]
    PartIndexOutOfBounds {
        /// Requested part index.
        index: usize,
    },
    /// The requested animation does not exist.
    #[error("glTF animation index {index} is out of bounds")]
    AnimationIndexOutOfBounds {
        /// Requested animation index.
        index: usize,
    },
    /// A part references a missing skin.
    #[error("glTF mesh part references missing skin {index}")]
    SkinIndexOutOfBounds {
        /// Referenced skin index.
        index: usize,
    },
    /// A skinned part did not provide both joint and weight attributes.
    #[error("skinned glTF mesh part has no joint weights")]
    MissingSkinWeights,
    /// A skin attribute count does not match the mesh vertex count.
    #[error("glTF skin weights have {weights} vertices; mesh has {vertices}")]
    SkinVertexCountMismatch {
        /// Number of vertices in the skin attributes.
        weights: usize,
        /// Number of mesh vertices.
        vertices: usize,
    },
    /// A vertex references a joint outside its skin.
    #[error("glTF vertex references joint {joint}, but skin has {joint_count} joints")]
    JointIndexOutOfBounds {
        /// Invalid joint index.
        joint: usize,
        /// Number of joints in the skin.
        joint_count: usize,
    },
    /// A skin joint references a missing node.
    #[error("glTF skin joint node {index} is out of bounds")]
    JointNodeOutOfBounds {
        /// Invalid node index.
        index: usize,
    },
}

impl GltfSceneAsset {
    /// Samples a mesh part in bind pose or at a selected animation time.
    ///
    /// The returned [`TriangleMesh`] is ready for
    /// [`crate::RenderScene::item_from_dynamic_mesh`]. Its node transform and
    /// skin deformation have already been applied. The source mesh stored in
    /// [`GltfScenePart::render_part`] remains unchanged, so callers can sample
    /// the same part repeatedly without accumulating deformation.
    pub fn sample_part(
        &self,
        part_index: usize,
        animation_index: Option<usize>,
        time_s: f32,
    ) -> Result<TriangleMesh, AnimationSampleError> {
        let part = self
            .parts
            .get(part_index)
            .ok_or(AnimationSampleError::PartIndexOutOfBounds { index: part_index })?;
        let local_transforms = animation_index
            .map(|index| {
                let animation = self
                    .animations
                    .get(index)
                    .ok_or(AnimationSampleError::AnimationIndexOutOfBounds { index })?;
                Ok(animation.sample_node_transforms(&self.nodes, time_s))
            })
            .transpose()?
            .unwrap_or_else(|| self.nodes.iter().map(|node| node.bind_transform).collect());
        let global_transforms = global_node_matrices(&self.nodes, &local_transforms);
        let node_transform = global_transforms
            .get(part.node_index)
            .copied()
            .unwrap_or(Mat4::IDENTITY);
        let raw_mesh = &part.render_part.mesh;

        let (skin, skin_weights) = match part.skin_index {
            Some(skin_index) => {
                let skin = self
                    .skins
                    .get(skin_index)
                    .ok_or(AnimationSampleError::SkinIndexOutOfBounds { index: skin_index })?;
                let weights = part
                    .skin_weights
                    .as_ref()
                    .ok_or(AnimationSampleError::MissingSkinWeights)?;
                if weights.joints.len() != raw_mesh.positions.len()
                    || weights.weights.len() != raw_mesh.positions.len()
                {
                    return Err(AnimationSampleError::SkinVertexCountMismatch {
                        weights: weights.joints.len().min(weights.weights.len()),
                        vertices: raw_mesh.positions.len(),
                    });
                }
                (Some(skin), Some(weights))
            }
            None => (None, None),
        };

        let mut positions = Vec::with_capacity(raw_mesh.positions.len());
        let mut normals = Vec::with_capacity(raw_mesh.normals.len());
        let node_normal_matrix = node_transform.inverse().transpose();
        for (vertex_index, position) in raw_mesh.positions.iter().enumerate() {
            let position = Vec3::new(
                f64::from(position[0]),
                f64::from(position[1]),
                f64::from(position[2]),
            );
            let normal = raw_mesh
                .normals
                .get(vertex_index)
                .copied()
                .unwrap_or([0.0, 1.0, 0.0]);
            let normal = Vec3::new(
                f64::from(normal[0]),
                f64::from(normal[1]),
                f64::from(normal[2]),
            );
            let (deformed_position, deformed_normal) =
                if let (Some(skin), Some(weights)) = (skin, skin_weights) {
                    let mut weighted_position = Vec3::ZERO;
                    let mut weighted_normal = Vec3::ZERO;
                    let mut total_weight = 0.0_f64;
                    for influence in 0..4 {
                        let weight = f64::from(weights.weights[vertex_index][influence]);
                        if !weight.is_finite() || weight <= 0.0 {
                            continue;
                        }
                        let joint_index = usize::from(weights.joints[vertex_index][influence]);
                        let joint = skin.joints.get(joint_index).ok_or(
                            AnimationSampleError::JointIndexOutOfBounds {
                                joint: joint_index,
                                joint_count: skin.joints.len(),
                            },
                        )?;
                        let joint_global = global_transforms.get(joint.node_index).copied().ok_or(
                            AnimationSampleError::JointNodeOutOfBounds {
                                index: joint.node_index,
                            },
                        )?;
                        let joint_matrix = joint_global * joint.inverse_bind_matrix;
                        weighted_position += joint_matrix.transform_point3(position) * weight;
                        weighted_normal +=
                            joint_matrix.inverse().transpose().transform_vector3(normal) * weight;
                        total_weight += weight;
                    }
                    if total_weight > 1.0e-12 {
                        (
                            weighted_position / total_weight,
                            weighted_normal / total_weight,
                        )
                    } else {
                        (position, normal)
                    }
                } else {
                    (position, normal)
                };
            let world_position = node_transform.transform_point3(deformed_position);
            let world_normal = node_normal_matrix
                .transform_vector3(deformed_normal)
                .normalize_or_zero();
            positions.push([
                world_position.x as f32,
                world_position.y as f32,
                world_position.z as f32,
            ]);
            normals.push([
                world_normal.x as f32,
                world_normal.y as f32,
                world_normal.z as f32,
            ]);
        }

        Ok(TriangleMesh {
            positions,
            normals,
            texcoords: raw_mesh.texcoords.clone(),
            indices: raw_mesh.indices.clone(),
        })
    }

    /// Samples a mesh part in its bind pose.
    pub fn sample_bind_pose(
        &self,
        part_index: usize,
    ) -> Result<TriangleMesh, AnimationSampleError> {
        self.sample_part(part_index, None, 0.0)
    }
}

fn global_node_matrices(nodes: &[GltfNode], local_transforms: &[Transform3]) -> Vec<Mat4> {
    let mut globals = vec![None; nodes.len()];
    let mut visiting = vec![false; nodes.len()];
    for index in 0..nodes.len() {
        resolve_global_node(index, nodes, local_transforms, &mut globals, &mut visiting);
    }
    globals
        .into_iter()
        .map(|matrix| matrix.unwrap_or(Mat4::IDENTITY))
        .collect()
}

fn resolve_global_node(
    index: usize,
    nodes: &[GltfNode],
    local_transforms: &[Transform3],
    globals: &mut [Option<Mat4>],
    visiting: &mut [bool],
) -> Mat4 {
    if let Some(matrix) = globals[index] {
        return matrix;
    }
    if visiting[index] {
        return local_transforms
            .get(index)
            .map(Transform3::to_matrix)
            .unwrap_or(Mat4::IDENTITY);
    }
    visiting[index] = true;
    let parent = nodes[index].parent_index.map_or(Mat4::IDENTITY, |parent| {
        resolve_global_node(parent, nodes, local_transforms, globals, visiting)
    });
    let local = local_transforms
        .get(index)
        .copied()
        .unwrap_or(Transform3::IDENTITY)
        .to_matrix();
    let global = parent * local;
    visiting[index] = false;
    globals[index] = Some(global);
    global
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_node_asset() -> GltfSceneAsset {
        GltfSceneAsset {
            nodes: vec![
                GltfNode {
                    name: Some("root".into()),
                    parent_index: None,
                    bind_transform: Transform3::IDENTITY,
                },
                GltfNode {
                    name: Some("joint".into()),
                    parent_index: Some(0),
                    bind_transform: Transform3::from_translation_rotation(
                        Vec3::new(0.0, 1.0, 0.0),
                        Quat::IDENTITY,
                    ),
                },
            ],
            skins: vec![GltfSkin {
                joints: vec![GltfSkinJoint {
                    node_index: 1,
                    inverse_bind_matrix: Mat4::from_translation(Vec3::new(0.0, -1.0, 0.0)),
                }],
            }],
            animations: vec![AnimationClip {
                name: Some("raise".into()),
                duration_s: 1.0,
                channels: vec![AnimationChannel {
                    node_index: 1,
                    property: AnimationProperty::Translation,
                    times_s: vec![0.0, 1.0],
                    values: vec![[0.0, 1.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0]],
                    interpolation: AnimationInterpolation::Linear,
                }],
            }],
            parts: vec![GltfScenePart {
                render_part: LoadedMeshPart {
                    mesh: TriangleMesh {
                        positions: vec![[0.0, 1.0, 0.0]],
                        normals: vec![[0.0, 1.0, 0.0]],
                        texcoords: vec![[0.0, 0.0]],
                        indices: vec![],
                    },
                    base_color_texture: None,
                    base_color_rgba: None,
                    material: Default::default(),
                },
                node_index: 0,
                skin_index: Some(0),
                skin_weights: Some(SkinWeights {
                    joints: vec![[0, 0, 0, 0]],
                    weights: vec![[1.0, 0.0, 0.0, 0.0]],
                }),
            }],
        }
    }

    #[test]
    fn animation_samples_translation_and_loops() {
        let asset = two_node_asset();
        let transforms = asset.animations[0].sample_node_transforms(&asset.nodes, 1.5);
        assert!((transforms[1].translation.y - 1.5).abs() < 1.0e-6);
    }

    #[test]
    fn skinning_applies_joint_delta_without_accumulating() {
        let asset = two_node_asset();
        let bind = asset.sample_bind_pose(0).expect("bind pose");
        let animated = asset.sample_part(0, Some(0), 0.5).expect("animated pose");
        assert!((bind.positions[0][1] - 1.0).abs() < 1.0e-6);
        assert!((animated.positions[0][1] - 1.5).abs() < 1.0e-6);
        let replay = asset.sample_part(0, Some(0), 0.5).expect("replay pose");
        assert_eq!(animated, replay);
    }
}
