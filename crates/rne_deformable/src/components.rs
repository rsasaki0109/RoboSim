//! ECS components and immutable topology for deformable bodies.

use bevy_ecs::prelude::Component;
use rne_ecs::Entity;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// One simulated material point in world space.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Particle {
    /// Current world-space position in meters.
    pub position_m: Vec3,
    /// Position at the start of the latest substep in meters.
    pub previous_position_m: Vec3,
    /// Current linear velocity in meters per second.
    pub velocity_m_s: Vec3,
    /// Reciprocal mass in inverse kilograms; zero marks a fixed particle.
    pub inverse_mass_kg_inv: f64,
}

impl Particle {
    /// Creates a dynamic particle with a positive mass.
    pub fn dynamic(position_m: Vec3, mass_kg: f64) -> Self {
        Self {
            position_m,
            previous_position_m: position_m,
            velocity_m_s: Vec3::ZERO,
            inverse_mass_kg_inv: mass_kg.recip(),
        }
    }

    /// Returns whether this particle can be moved by the solver.
    pub fn is_dynamic(&self) -> bool {
        self.inverse_mass_kg_inv > 0.0
    }
}

/// Semantic purpose of a distance constraint.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConstraintKind {
    /// Preserves cable segments or cloth grid edges.
    Structural,
    /// Preserves cloth diagonal distances.
    Shear,
    /// Resists cable or cloth bending through second-neighbor distance.
    Bending,
}

/// XPBD distance constraint between two particles.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DistanceConstraint {
    /// First particle index.
    pub first: usize,
    /// Second particle index.
    pub second: usize,
    /// Rest distance in meters.
    pub rest_length_m: f64,
    /// XPBD compliance in meters per newton; zero is rigid.
    pub compliance_m_n: f64,
    /// Constraint category.
    pub kind: ConstraintKind,
    /// Accumulated XPBD Lagrange multiplier for the current substep.
    #[serde(skip)]
    pub lambda: f64,
}

/// Particle pinned to a world-space target.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PinConstraint {
    /// Pinned particle index.
    pub particle: usize,
    /// World-space target in meters.
    pub target_position_m: Vec3,
}

/// Triangle index topology used to render a cloth surface.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TriangleTopology {
    /// Counter-clockwise triangle indices.
    pub indices: Vec<u32>,
}

/// CPU geometry extracted from deformable state for rendering.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DeformableSurfaceMesh {
    /// World-space vertex positions in meters.
    pub positions: Vec<[f32; 3]>,
    /// Smooth unit normals aligned with `positions`.
    pub normals: Vec<[f32; 3]>,
    /// Counter-clockwise triangle indices.
    pub indices: Vec<u32>,
}

/// One cable segment rendered as a cylinder between its endpoints.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CableSegment {
    /// First world-space endpoint in meters.
    pub start_m: Vec3,
    /// Second world-space endpoint in meters.
    pub end_m: Vec3,
    /// Cable radius in meters.
    pub radius_m: f64,
}

/// Render metadata attached to a deformable ECS entity.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeformableVisual {
    /// Linear RGBA surface color.
    pub color_rgba: [f32; 4],
}

/// One deformable particle anchored in a target entity's local frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeformableAttachmentPoint {
    /// Particle index in the attached [`DeformableBody`].
    pub particle: usize,
    /// Anchor position in the target entity's local frame, in meters.
    pub target_local_position_m: Vec3,
}

/// Kinematic attachment from deformable particles to another ECS entity.
///
/// Removing this component releases every point without changing authored pins.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct DeformableAttachment {
    /// Entity whose composed world transform drives the attachment anchors.
    pub target: Entity,
    /// Attached points in stable particle-index order.
    pub points: Vec<DeformableAttachmentPoint>,
}

/// Deformable topology category.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeformableKind {
    /// One-dimensional particle chain.
    Cable,
    /// Two-dimensional rectangular particle grid.
    Cloth,
}

/// Material parameters shared by cable and cloth simulation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeformableMaterial {
    /// Radius used for particle collision in meters.
    pub collision_radius_m: f64,
    /// Structural compliance in meters per newton.
    pub structural_compliance_m_n: f64,
    /// Shear compliance in meters per newton.
    pub shear_compliance_m_n: f64,
    /// Bending compliance in meters per newton.
    pub bending_compliance_m_n: f64,
    /// Fraction of velocity retained per second, in `(0, 1]`.
    pub velocity_retention_per_s: f64,
    /// Whether non-adjacent particle and cloth vertex-triangle contacts are solved.
    #[serde(default)]
    pub self_collision: bool,
}

impl Default for DeformableMaterial {
    fn default() -> Self {
        Self {
            collision_radius_m: 0.01,
            structural_compliance_m_n: 1.0e-7,
            shear_compliance_m_n: 2.0e-7,
            bending_compliance_m_n: 2.0e-4,
            velocity_retention_per_s: 0.995,
            self_collision: false,
        }
    }
}

/// Immutable description used to build an evenly spaced cable.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CableSpec {
    /// First cable endpoint in world-space meters.
    pub start_m: Vec3,
    /// Second cable endpoint in world-space meters.
    pub end_m: Vec3,
    /// Number of simulated particles, at least two.
    pub particle_count: usize,
    /// Total cable mass in kilograms.
    pub total_mass_kg: f64,
    /// Pin the first particle to `start_m`.
    pub pin_start: bool,
    /// Pin the last particle to `end_m`.
    pub pin_end: bool,
    /// Cable material parameters.
    pub material: DeformableMaterial,
}

/// Immutable description used to build a rectangular cloth grid.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ClothSpec {
    /// Grid origin in world-space meters.
    pub origin_m: Vec3,
    /// Vector from the first to the last grid column in meters.
    pub width_direction_m: Vec3,
    /// Vector from the first to the last grid row in meters.
    pub height_direction_m: Vec3,
    /// Number of grid columns, at least two.
    pub columns: usize,
    /// Number of grid rows, at least two.
    pub rows: usize,
    /// Total cloth mass in kilograms.
    pub total_mass_kg: f64,
    /// Pin every particle in the first row.
    pub pin_top_edge: bool,
    /// Cloth material parameters.
    pub material: DeformableMaterial,
}

/// Complete deformable simulation state stored on one ECS entity.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeformableBody {
    /// Cable or cloth topology category.
    pub kind: DeformableKind,
    /// Particle state in stable index order.
    pub particles: Vec<Particle>,
    /// Distance constraints in stable solve order.
    pub distance_constraints: Vec<DistanceConstraint>,
    /// Pin constraints in stable solve order.
    pub pin_constraints: Vec<PinConstraint>,
    /// Optional cloth triangle topology.
    pub triangles: TriangleTopology,
    /// Physical material parameters.
    pub material: DeformableMaterial,
}

impl DeformableBody {
    /// Returns a stable FNV-1a hash of all externally visible particle state.
    pub fn stable_state_hash(&self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        for particle in &self.particles {
            for value in [
                particle.position_m.x,
                particle.position_m.y,
                particle.position_m.z,
                particle.velocity_m_s.x,
                particle.velocity_m_s.y,
                particle.velocity_m_s.z,
                particle.inverse_mass_kg_inv,
            ] {
                for byte in value.to_bits().to_le_bytes() {
                    hash ^= u64::from(byte);
                    hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
                }
            }
        }
        hash
    }

    /// Extracts cable segments in particle index order.
    pub fn cable_segments(&self) -> Vec<CableSegment> {
        self.particles
            .windows(2)
            .map(|particles| CableSegment {
                start_m: particles[0].position_m,
                end_m: particles[1].position_m,
                radius_m: self.material.collision_radius_m,
            })
            .collect()
    }

    /// Builds a cloth triangle mesh with deterministic smooth normals.
    ///
    /// Returns `None` for cable bodies or missing triangle topology.
    pub fn cloth_surface_mesh(&self) -> Option<DeformableSurfaceMesh> {
        if self.kind != DeformableKind::Cloth || self.triangles.indices.is_empty() {
            return None;
        }
        let positions = self
            .particles
            .iter()
            .map(|particle| {
                [
                    particle.position_m.x as f32,
                    particle.position_m.y as f32,
                    particle.position_m.z as f32,
                ]
            })
            .collect::<Vec<_>>();
        let mut normal_sums = vec![Vec3::ZERO; self.particles.len()];
        for triangle in self.triangles.indices.chunks_exact(3) {
            let first = triangle[0] as usize;
            let second = triangle[1] as usize;
            let third = triangle[2] as usize;
            let edge_a = self.particles[second].position_m - self.particles[first].position_m;
            let edge_b = self.particles[third].position_m - self.particles[first].position_m;
            let face_normal = edge_a.cross(edge_b);
            normal_sums[first] += face_normal;
            normal_sums[second] += face_normal;
            normal_sums[third] += face_normal;
        }
        let normals = normal_sums
            .into_iter()
            .map(|normal| {
                let normal = if normal.length_squared() > f64::EPSILON {
                    normal.normalize()
                } else {
                    Vec3::Z
                };
                [normal.x as f32, normal.y as f32, normal.z as f32]
            })
            .collect();
        Some(DeformableSurfaceMesh {
            positions,
            normals,
            indices: self.triangles.indices.clone(),
        })
    }
}
