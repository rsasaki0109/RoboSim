//! Backend-neutral articulated self-collision checking.
//!
//! This is the RNE counterpart of Choreonoid's `BodyCollisionDetector`: it
//! evaluates forward kinematics for a robot and tests collider pairs against
//! each other without stepping a physics backend. Colliders are represented as
//! convex primitives and tested with closed-form distances (sphere / capsule)
//! and a separating-axis test (cuboid / cuboid). Cuboid pairs that mix with a
//! rounded primitive use an exact point / segment to box distance.
//!
//! Pairs that are structurally adjacent (same link, or connected within
//! `min_link_distance` joints) are skipped, mirroring
//! the usual "ignore parent/child contact" behavior of robot models. Explicit
//! [`CollisionGroups`] are honored using the same mask semantics as the physics
//! backends.

use crate::kinematics::{ForwardKinematics, KinematicModel, KinematicsError};
use bevy_ecs::prelude::World;
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_physics::{Collider, ColliderShape, CollisionGroups};
use rne_world::Transform3;
use std::collections::HashSet;

/// Convex collision primitive in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CollisionPrimitive {
    /// Sphere with a center and radius in meters.
    Sphere {
        /// Center in meters.
        center_m: Vec3,
        /// Radius in meters.
        radius_m: f64,
    },
    /// Capsule with a world-space segment and radius in meters.
    Capsule {
        /// First segment endpoint in meters.
        a_m: Vec3,
        /// Second segment endpoint in meters.
        b_m: Vec3,
        /// Radius in meters.
        radius_m: f64,
    },
    /// Oriented box with a center, orthonormal axes, and half extents in meters.
    Cuboid {
        /// Center in meters.
        center_m: Vec3,
        /// Orthonormal body axes; `axes[0]` pairs with `half_extents_m.x`.
        axes: [Vec3; 3],
        /// Half extents along `axes` in meters.
        half_extents_m: Vec3,
    },
}

impl CollisionPrimitive {
    /// Builds a world-space primitive from a collider shape and pose.
    ///
    /// Infinite planes are not supported by the self-collision checker and
    /// return `None`.
    pub fn from_shape(shape: &ColliderShape, transform: &Transform3) -> Option<Self> {
        match *shape {
            ColliderShape::Sphere { radius_m } => Some(Self::Sphere {
                center_m: transform.translation,
                radius_m: radius_m.abs(),
            }),
            ColliderShape::Capsule {
                half_height_m,
                radius_m,
            } => {
                let a = transform_point(transform, Vec3::new(0.0, -half_height_m, 0.0));
                let b = transform_point(transform, Vec3::new(0.0, half_height_m, 0.0));
                Some(Self::Capsule {
                    a_m: a,
                    b_m: b,
                    radius_m: radius_m.abs(),
                })
            }
            ColliderShape::Cuboid { half_extents_m } => {
                let axes = [
                    (transform.rotation * Vec3::X).normalize_or_zero(),
                    (transform.rotation * Vec3::Y).normalize_or_zero(),
                    (transform.rotation * Vec3::Z).normalize_or_zero(),
                ];
                Some(Self::Cuboid {
                    center_m: transform.translation,
                    axes,
                    half_extents_m: Vec3::new(
                        (half_extents_m.x * transform.scale.x).abs(),
                        (half_extents_m.y * transform.scale.y).abs(),
                        (half_extents_m.z * transform.scale.z).abs(),
                    ),
                })
            }
            ColliderShape::Plane { .. } => None,
        }
    }
}

/// A reported self-collision pair.
#[derive(Clone, Debug, PartialEq)]
pub struct SelfCollisionPair {
    /// First link entity.
    pub link_a: Entity,
    /// Second link entity.
    pub link_b: Entity,
    /// Penetration depth in meters; positive when the pair overlaps.
    pub depth_m: f64,
    /// Attached body name on the first side, if any.
    pub body_a: Option<String>,
    /// Attached body name on the second side, if any.
    pub body_b: Option<String>,
}

/// A collision body rigidly attached to a robot link.
///
/// This is the RNE analogue of MoveIt's `AttachedBody`: a grasped or mounted
/// object whose geometry moves with the link. It is tested against other links
/// and world objects, except for the links in `touch_links` (for example the
/// gripper that holds it).
#[derive(Clone, Debug, PartialEq)]
pub struct AttachedBody {
    name: String,
    link: Entity,
    shape: ColliderShape,
    local_offset: Transform3,
    groups: CollisionGroups,
    touch_links: Vec<Entity>,
}

impl AttachedBody {
    /// Body name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Link the body is attached to.
    pub fn link(&self) -> Entity {
        self.link
    }

    /// Links the body is allowed to touch.
    pub fn touch_links(&self) -> &[Entity] {
        &self.touch_links
    }
}

/// Result of a self-collision query.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SelfCollisionReport {
    pairs: Vec<SelfCollisionPair>,
}

impl SelfCollisionReport {
    /// Whether any pair overlaps.
    pub fn is_colliding(&self) -> bool {
        !self.pairs.is_empty()
    }

    /// Number of overlapping pairs.
    pub fn len(&self) -> usize {
        self.pairs.len()
    }

    /// Whether no pair overlaps.
    pub fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }

    /// Reported pairs in deterministic link order.
    pub fn pairs(&self) -> &[SelfCollisionPair] {
        &self.pairs
    }
}

/// Set of link pairs whose collision checks are explicitly skipped.
///
/// This is the RNE analogue of MoveIt's allowed collision matrix (ACM). The
/// checker tests every otherwise-eligible pair unless the pair is present here.
/// Pairs are stored in canonical entity order, so `allow(a, b)` and
/// `allow(b, a)` describe the same entry.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AllowedCollisionMatrix {
    allowed: HashSet<(Entity, Entity)>,
}

impl AllowedCollisionMatrix {
    /// Creates an empty matrix that checks every pair.
    pub fn new() -> Self {
        Self::default()
    }

    /// Marks a link pair as allowed to skip.
    pub fn allow(&mut self, link_a: Entity, link_b: Entity) {
        self.allowed.insert(canonical_pair(link_a, link_b));
    }

    /// Removes a link pair from the allowed set.
    pub fn deny(&mut self, link_a: Entity, link_b: Entity) {
        self.allowed.remove(&canonical_pair(link_a, link_b));
    }

    /// Whether a link pair is allowed to skip.
    pub fn is_allowed(&self, link_a: Entity, link_b: Entity) -> bool {
        self.allowed.contains(&canonical_pair(link_a, link_b))
    }

    /// Number of allowed pairs.
    pub fn len(&self) -> usize {
        self.allowed.len()
    }

    /// Whether no pair is allowed.
    pub fn is_empty(&self) -> bool {
        self.allowed.is_empty()
    }

    /// Removes all entries.
    pub fn clear(&mut self) {
        self.allowed.clear();
    }
}

fn canonical_pair(link_a: Entity, link_b: Entity) -> (Entity, Entity) {
    if link_b < link_a {
        (link_b, link_a)
    } else {
        (link_a, link_b)
    }
}

/// A link pair and its signed distance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionPairDistance {
    /// First link entity.
    pub link_a: Entity,
    /// Second link entity.
    pub link_b: Entity,
    /// Signed distance in meters; negative when the pair penetrates.
    pub distance_m: f64,
}

/// Result of a minimum-distance query over the robot's checked pairs.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SelfCollisionDistanceReport {
    closest: Option<CollisionPairDistance>,
}

impl SelfCollisionDistanceReport {
    /// Minimum signed distance in meters, or `None` when no pair was evaluated.
    pub fn min_distance_m(&self) -> Option<f64> {
        self.closest.map(|pair| pair.distance_m)
    }

    /// Closest checked pair, if any.
    pub fn closest(&self) -> Option<CollisionPairDistance> {
        self.closest
    }

    /// Whether the closest pair penetrates.
    pub fn is_colliding(&self) -> bool {
        self.closest.is_some_and(|pair| pair.distance_m < 0.0)
    }
}

/// Configuration for joint-space path collision checking.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PathCollisionConfig {
    /// Number of interpolation segments between the two endpoints.
    pub steps: usize,
}

impl Default for PathCollisionConfig {
    fn default() -> Self {
        Self { steps: 32 }
    }
}

impl PathCollisionConfig {
    /// Creates a configuration with the given number of segments.
    pub fn new(steps: usize) -> Self {
        Self { steps }
    }
}

/// First colliding configuration found along a joint-space path.
#[derive(Clone, Debug, PartialEq)]
pub struct PathCollisionSample {
    /// Interpolation parameter in `[0, 1]` between start and goal.
    pub interpolation: f64,
    /// The colliding pair at this sample.
    pub pair: SelfCollisionPair,
}

/// Result of a path collision query.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PathCollisionReport {
    first: Option<PathCollisionSample>,
    samples_checked: usize,
}

impl PathCollisionReport {
    /// Whether the whole path is collision free.
    pub fn is_valid(&self) -> bool {
        self.first.is_none()
    }

    /// The first collision, if the path is invalid.
    pub fn first_collision(&self) -> Option<PathCollisionSample> {
        self.first.clone()
    }

    /// Number of configurations checked, including both endpoints.
    pub fn samples_checked(&self) -> usize {
        self.samples_checked
    }
}

/// A static collision object in world space.
///
/// This is the RNE analogue of MoveIt's `CollisionObject`: a named primitive in
/// the world that planners can add, look up, and remove.
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionWorldObject {
    /// Object name, or empty for an anonymous object.
    pub name: String,
    /// Collision primitive in world coordinates.
    pub primitive: CollisionPrimitive,
    /// Interaction masks; the default interacts with every group.
    pub groups: CollisionGroups,
}

impl CollisionWorldObject {
    /// Creates an anonymous object that interacts with every collision group.
    pub fn new(primitive: CollisionPrimitive) -> Self {
        Self {
            name: String::new(),
            primitive,
            groups: CollisionGroups::default(),
        }
    }

    /// Creates a named object that interacts with every collision group.
    pub fn named(name: impl Into<String>, primitive: CollisionPrimitive) -> Self {
        Self {
            name: name.into(),
            primitive,
            groups: CollisionGroups::default(),
        }
    }

    /// Object name, if any.
    pub fn name(&self) -> Option<&str> {
        if self.name.is_empty() {
            None
        } else {
            Some(&self.name)
        }
    }
}

/// A triangle-mesh collision object in world space.
///
/// This is the RNE approximation of MoveIt's mesh collision geometry. Meshes are
/// tested against robot spheres and capsules (exact point/segment-to-triangle
/// distance) and against line-of-sight segments (ray/triangle intersection);
/// cuboid-vs-mesh uses the mesh AABB, which is conservative.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshCollisionObject {
    name: String,
    vertices: Vec<Vec3>,
    triangles: Vec<[usize; 3]>,
}

impl MeshCollisionObject {
    /// Creates a named mesh, validating triangle indices.
    pub fn new(
        name: impl Into<String>,
        vertices: Vec<Vec3>,
        triangles: Vec<[usize; 3]>,
    ) -> Result<Self, KinematicsError> {
        if vertices.is_empty() || triangles.is_empty() {
            return Err(KinematicsError::NonFiniteInput);
        }
        if triangles
            .iter()
            .flatten()
            .any(|&index| index >= vertices.len())
        {
            return Err(KinematicsError::NonFiniteInput);
        }
        Ok(Self {
            name: name.into(),
            vertices,
            triangles,
        })
    }

    /// Mesh name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Mesh vertices in world coordinates.
    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    /// Triangle vertex-index triples.
    pub fn triangles(&self) -> &[[usize; 3]] {
        &self.triangles
    }

    /// Axis-aligned bounds of the mesh.
    pub fn aabb(&self) -> (Vec3, Vec3) {
        let mut min = self.vertices[0];
        let mut max = self.vertices[0];
        for vertex in &self.vertices {
            min = min.min(*vertex);
            max = max.max(*vertex);
        }
        (min, max)
    }

    fn minimum_point_distance(&self, point: Vec3) -> f64 {
        self.triangles
            .iter()
            .map(|triangle| {
                point_triangle_distance(
                    point,
                    self.vertices[triangle[0]],
                    self.vertices[triangle[1]],
                    self.vertices[triangle[2]],
                )
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn minimum_segment_distance(&self, a: Vec3, b: Vec3) -> f64 {
        const SAMPLES: usize = 16;
        let mut minimum = f64::INFINITY;
        for step in 0..=SAMPLES {
            let point = a.lerp(b, step as f64 / SAMPLES as f64);
            minimum = minimum.min(self.minimum_point_distance(point));
        }
        minimum
    }

    fn segment_intersects(&self, a: Vec3, b: Vec3) -> bool {
        self.triangles.iter().any(|triangle| {
            segment_triangle_intersects(
                a,
                b,
                self.vertices[triangle[0]],
                self.vertices[triangle[1]],
                self.vertices[triangle[2]],
            )
        })
    }
}

/// A dense voxel occupancy grid in world space.
///
/// This is the RNE analogue of an Octomap collision map: a regular voxel grid
/// whose occupied cells are tested against robot primitives using the same
/// closed-form box distances as `CollisionWorld` cuboids. It is deterministic
/// and backend-neutral.
#[derive(Clone, Debug, PartialEq)]
pub struct VoxelGridObject {
    name: String,
    origin_m: Vec3,
    resolution_m: f64,
    dims: [usize; 3],
    occupied: Vec<bool>,
}

impl VoxelGridObject {
    /// Creates a grid from an explicit occupancy bitmap.
    pub fn new(
        name: impl Into<String>,
        origin_m: Vec3,
        resolution_m: f64,
        dims: [usize; 3],
        occupied: Vec<bool>,
    ) -> Result<Self, KinematicsError> {
        if !origin_m.is_finite()
            || !resolution_m.is_finite()
            || resolution_m <= 0.0
            || dims.contains(&0)
            || occupied.len() != dims[0] * dims[1] * dims[2]
        {
            return Err(KinematicsError::NonFiniteInput);
        }
        Ok(Self {
            name: name.into(),
            origin_m,
            resolution_m,
            dims,
            occupied,
        })
    }

    /// Builds a grid that occupies every voxel containing one of `points`.
    ///
    /// The grid covers the padded point bounds. Useful for point-cloud or
    /// depth-derived obstacles.
    pub fn from_points(
        name: impl Into<String>,
        resolution_m: f64,
        padding_m: f64,
        points: &[Vec3],
    ) -> Result<Self, KinematicsError> {
        if points.is_empty() || !resolution_m.is_finite() || resolution_m <= 0.0 {
            return Err(KinematicsError::NonFiniteInput);
        }
        let mut min = points[0];
        let mut max = points[0];
        for point in points {
            if !point.is_finite() {
                return Err(KinematicsError::NonFiniteInput);
            }
            min = min.min(*point);
            max = max.max(*point);
        }
        let padding = padding_m.max(0.0);
        let origin = min - Vec3::splat(padding + resolution_m);
        let extent = (max - min) + Vec3::splat(2.0 * (padding + resolution_m));
        let dims = [
            (extent.x / resolution_m).ceil().max(1.0) as usize,
            (extent.y / resolution_m).ceil().max(1.0) as usize,
            (extent.z / resolution_m).ceil().max(1.0) as usize,
        ];
        let mut grid = Self::new(
            name,
            origin,
            resolution_m,
            dims,
            vec![false; dims[0] * dims[1] * dims[2]],
        )?;
        for point in points {
            let local = (*point - origin) / resolution_m;
            let coords = [
                (local.x.floor() as isize).clamp(0, dims[0] as isize - 1) as usize,
                (local.y.floor() as isize).clamp(0, dims[1] as isize - 1) as usize,
                (local.z.floor() as isize).clamp(0, dims[2] as isize - 1) as usize,
            ];
            let index = coords[2] * dims[0] * dims[1] + coords[1] * dims[0] + coords[0];
            grid.occupied[index] = true;
        }
        Ok(grid)
    }

    /// Grid name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Number of occupied voxels.
    pub fn occupied_count(&self) -> usize {
        self.occupied.iter().filter(|occupied| **occupied).count()
    }

    /// Voxel resolution in meters.
    pub fn resolution_m(&self) -> f64 {
        self.resolution_m
    }

    fn occupied_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.occupied
            .iter()
            .enumerate()
            .filter_map(|(index, occupied)| occupied.then_some(index))
    }

    fn voxel_cuboid(&self, index: usize) -> CollisionPrimitive {
        let width = self.dims[0];
        let depth = self.dims[1];
        let coords = [
            index % width,
            (index / width) % depth,
            index / (width * depth),
        ];
        let half = self.resolution_m * 0.5;
        let center = self.origin_m
            + Vec3::new(coords[0] as f64, coords[1] as f64, coords[2] as f64) * self.resolution_m
            + Vec3::splat(half);
        CollisionPrimitive::Cuboid {
            center_m: center,
            axes: [Vec3::X, Vec3::Y, Vec3::Z],
            half_extents_m: Vec3::splat(half),
        }
    }
}

/// A robot-vs-world collision pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WorldCollisionPair {
    /// Robot link entity.
    pub link: Entity,
    /// Index of the primitive object in the [`CollisionWorld`], or the mesh
    /// index when `mesh` is set.
    pub object: usize,
    /// Penetration depth in meters.
    pub depth_m: f64,
    /// Whether `object` indexes a mesh object rather than a primitive.
    pub mesh: bool,
    /// Whether `object` indexes a voxel grid rather than a primitive.
    pub voxel: bool,
}

/// Result of a robot-vs-world collision query.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WorldCollisionReport {
    pairs: Vec<WorldCollisionPair>,
}

impl WorldCollisionReport {
    /// Whether any link overlaps a world object.
    pub fn is_colliding(&self) -> bool {
        !self.pairs.is_empty()
    }

    /// Number of overlapping pairs.
    pub fn len(&self) -> usize {
        self.pairs.len()
    }

    /// Whether no link overlaps a world object.
    pub fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }

    /// Reported pairs in deterministic link then object order.
    pub fn pairs(&self) -> &[WorldCollisionPair] {
        &self.pairs
    }
}

/// Closest robot link to a world object and the signed distance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WorldCollisionDistance {
    /// Robot link entity.
    pub link: Entity,
    /// Index of the primitive object in the [`CollisionWorld`], or the mesh
    /// index when `mesh` is set.
    pub object: usize,
    /// Signed distance in meters; negative when the pair penetrates.
    pub distance_m: f64,
    /// Whether `object` indexes a mesh object rather than a primitive.
    pub mesh: bool,
    /// Whether `object` indexes a voxel grid rather than a primitive.
    pub voxel: bool,
}

/// Result of a robot-vs-world minimum-distance query.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WorldCollisionDistanceReport {
    closest: Option<WorldCollisionDistance>,
}

impl WorldCollisionDistanceReport {
    /// Minimum signed distance in meters, or `None` when no pair was evaluated.
    pub fn min_distance_m(&self) -> Option<f64> {
        self.closest.map(|pair| pair.distance_m)
    }

    /// Closest checked pair, if any.
    pub fn closest(&self) -> Option<WorldCollisionDistance> {
        self.closest
    }

    /// Whether the closest pair penetrates.
    pub fn is_colliding(&self) -> bool {
        self.closest.is_some_and(|pair| pair.distance_m < 0.0)
    }
}

/// Backend-neutral container of world collision objects.
///
/// This is the MoveIt `CollisionWorld` analogue: objects live in world space and
/// are tested against a robot's link colliders evaluated through
/// [`SelfCollisionChecker`]. It is deliberately independent of any physics
/// backend.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionWorld {
    objects: Vec<CollisionWorldObject>,
    meshes: Vec<MeshCollisionObject>,
    voxels: Vec<VoxelGridObject>,
}

impl CollisionWorld {
    /// Creates an empty collision world.
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates a collision world from the given primitive objects.
    pub fn with_objects(objects: Vec<CollisionWorldObject>) -> Self {
        Self {
            objects,
            meshes: Vec::new(),
            voxels: Vec::new(),
        }
    }

    /// Adds or replaces a named voxel occupancy grid.
    pub fn add_voxel_grid(&mut self, grid: VoxelGridObject) -> usize {
        let name = grid.name.to_string();
        self.voxels.retain(|existing| existing.name != name);
        self.voxels.push(grid);
        self.voxels.len() - 1
    }

    /// Removes a voxel grid by name, returning whether it existed.
    pub fn remove_voxel_grid(&mut self, name: &str) -> bool {
        let before = self.voxels.len();
        self.voxels.retain(|grid| grid.name != name);
        self.voxels.len() != before
    }

    /// Looks up a voxel grid by name.
    pub fn voxel_grid(&self, name: &str) -> Option<&VoxelGridObject> {
        self.voxels.iter().find(|grid| grid.name == name)
    }

    /// Voxel grids in insertion order.
    pub fn voxel_grids(&self) -> &[VoxelGridObject] {
        &self.voxels
    }

    /// Adds or replaces a named mesh object.
    pub fn add_mesh_object(
        &mut self,
        name: impl Into<String>,
        vertices: Vec<Vec3>,
        triangles: Vec<[usize; 3]>,
    ) -> Result<usize, KinematicsError> {
        let mesh = MeshCollisionObject::new(name, vertices, triangles)?;
        self.meshes.retain(|existing| existing.name != mesh.name);
        self.meshes.push(mesh);
        Ok(self.meshes.len() - 1)
    }

    /// Removes a mesh object by name, returning whether it existed.
    pub fn remove_mesh_object(&mut self, name: &str) -> bool {
        let before = self.meshes.len();
        self.meshes.retain(|mesh| mesh.name != name);
        self.meshes.len() != before
    }

    /// Looks up a mesh object by name.
    pub fn mesh_object(&self, name: &str) -> Option<&MeshCollisionObject> {
        self.meshes.iter().find(|mesh| mesh.name == name)
    }

    /// Mesh objects in insertion order.
    pub fn meshes(&self) -> &[MeshCollisionObject] {
        &self.meshes
    }

    /// World objects in insertion order.
    pub fn objects(&self) -> &[CollisionWorldObject] {
        &self.objects
    }

    /// Appends an object and returns its index.
    pub fn add_object(&mut self, object: CollisionWorldObject) -> usize {
        self.objects.push(object);
        self.objects.len() - 1
    }

    /// Appends a named object, replacing any object with the same name.
    pub fn add_named_object(
        &mut self,
        name: impl Into<String>,
        primitive: CollisionPrimitive,
    ) -> usize {
        let name = name.into();
        self.objects.retain(|object| object.name != name);
        self.add_object(CollisionWorldObject::named(name, primitive))
    }

    /// Removes an object by name, returning whether it existed.
    pub fn remove_object(&mut self, name: &str) -> bool {
        let before = self.objects.len();
        self.objects.retain(|object| object.name != name);
        self.objects.len() != before
    }

    /// Looks up an object by name.
    pub fn object(&self, name: &str) -> Option<&CollisionWorldObject> {
        self.objects.iter().find(|object| object.name == name)
    }

    /// Name of the object at `index`, if it is named.
    pub fn object_name(&self, index: usize) -> Option<&str> {
        self.objects.get(index).and_then(|object| object.name())
    }

    /// Number of world objects.
    pub fn len(&self) -> usize {
        self.objects.len()
    }

    /// Whether the world has no objects.
    pub fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }

    /// Tests every robot link primitive against every world object.
    pub fn check(
        &self,
        checker: &SelfCollisionChecker,
        q: &[f64],
    ) -> Result<WorldCollisionReport, KinematicsError> {
        let links = checker.link_primitives(q)?;
        let mut pairs = Vec::new();
        for (link, primitive, groups) in &links {
            for (object_index, object) in self.objects.iter().enumerate() {
                if !groups_interact(groups, &object.groups) {
                    continue;
                }
                if let Some(depth_m) = penetration(primitive, &object.primitive) {
                    pairs.push(WorldCollisionPair {
                        link: *link,
                        object: object_index,
                        depth_m,
                        mesh: false,
                        voxel: false,
                    });
                }
            }
            for (mesh_index, mesh) in self.meshes.iter().enumerate() {
                if let Some(depth_m) = penetration_primitive_mesh(primitive, mesh) {
                    pairs.push(WorldCollisionPair {
                        link: *link,
                        object: mesh_index,
                        depth_m,
                        mesh: true,
                        voxel: false,
                    });
                }
            }
            for (voxel_index, grid) in self.voxels.iter().enumerate() {
                if let Some(depth_m) = penetration_primitive_voxels(primitive, grid) {
                    pairs.push(WorldCollisionPair {
                        link: *link,
                        object: voxel_index,
                        depth_m,
                        mesh: false,
                        voxel: true,
                    });
                }
            }
        }
        Ok(WorldCollisionReport { pairs })
    }

    /// Minimum signed distance from any robot link to any world object.
    pub fn distance(
        &self,
        checker: &SelfCollisionChecker,
        q: &[f64],
    ) -> Result<WorldCollisionDistanceReport, KinematicsError> {
        let links = checker.link_primitives(q)?;
        let mut closest: Option<WorldCollisionDistance> = None;
        for (link, primitive, groups) in &links {
            for (object_index, object) in self.objects.iter().enumerate() {
                if !groups_interact(groups, &object.groups) {
                    continue;
                }
                let distance_m = signed_distance(primitive, &object.primitive);
                if closest.is_none_or(|current| distance_m < current.distance_m) {
                    closest = Some(WorldCollisionDistance {
                        link: *link,
                        object: object_index,
                        distance_m,
                        mesh: false,
                        voxel: false,
                    });
                }
            }
            for (mesh_index, mesh) in self.meshes.iter().enumerate() {
                let distance_m = signed_distance_primitive_mesh(primitive, mesh);
                if closest.is_none_or(|current| distance_m < current.distance_m) {
                    closest = Some(WorldCollisionDistance {
                        link: *link,
                        object: mesh_index,
                        distance_m,
                        mesh: true,
                        voxel: false,
                    });
                }
            }
            for (voxel_index, grid) in self.voxels.iter().enumerate() {
                let distance_m = signed_distance_primitive_voxels(primitive, grid);
                if closest.is_none_or(|current| distance_m < current.distance_m) {
                    closest = Some(WorldCollisionDistance {
                        link: *link,
                        object: voxel_index,
                        distance_m,
                        mesh: false,
                        voxel: true,
                    });
                }
            }
        }
        Ok(WorldCollisionDistanceReport { closest })
    }

    /// Whether the segment `a`-`b` is occluded by any world object.
    pub fn segment_blocked(&self, a: Vec3, b: Vec3) -> bool {
        self.objects
            .iter()
            .any(|object| segment_intersects_primitive(&object.primitive, a, b))
            || self.meshes.iter().any(|mesh| mesh.segment_intersects(a, b))
            || self
                .voxels
                .iter()
                .any(|grid| voxel_segment_intersects(grid, a, b))
    }
}

#[derive(Clone, Copy, Debug)]
struct LinkCollider {
    link: Entity,
    link_index: usize,
    shape: ColliderShape,
    local_offset: Transform3,
    groups: CollisionGroups,
}

/// Checks self-collision for a robot across its joint configuration.
#[derive(Clone, Debug)]
pub struct SelfCollisionChecker {
    model: KinematicModel,
    colliders: Vec<LinkCollider>,
    excluded: Vec<Vec<bool>>,
    allowed: AllowedCollisionMatrix,
    attached: Vec<AttachedBody>,
}

impl SelfCollisionChecker {
    /// Builds a checker for a robot, defaulting to excluding same-link and
    /// directly connected link pairs.
    pub fn from_robot(world: &World, robot: Entity) -> Result<Self, KinematicsError> {
        Self::from_robot_with_min_link_distance(world, robot, 1)
    }

    /// Builds a checker that ignores link pairs connected within
    /// `min_link_distance` joints. A value of `0` only excludes same-link
    /// colliders; `1` additionally excludes parent/child pairs.
    pub fn from_robot_with_min_link_distance(
        world: &World,
        robot: Entity,
        min_link_distance: usize,
    ) -> Result<Self, KinematicsError> {
        let model = KinematicModel::from_robot(world, robot)?;
        let mut colliders: Vec<LinkCollider> = Vec::new();
        for &link in &model_links(&model) {
            let Some(index) = model.link_index(link) else {
                continue;
            };
            let Some(collider) = world.get::<Collider>(link) else {
                continue;
            };
            let groups = world
                .get::<CollisionGroups>(link)
                .copied()
                .unwrap_or_default();
            colliders.push(LinkCollider {
                link,
                link_index: index,
                shape: collider.shape,
                local_offset: collider.local_offset,
                groups,
            });
        }
        colliders.sort_by(|a, b| {
            a.link_index
                .cmp(&b.link_index)
                .then_with(|| a.link.index().cmp(&b.link.index()))
        });

        let distances = link_distances(&model);
        let excluded = distances
            .iter()
            .map(|row| row.iter().map(|&d| d <= min_link_distance).collect())
            .collect();

        Ok(Self {
            model,
            colliders,
            excluded,
            allowed: AllowedCollisionMatrix::new(),
            attached: Vec::new(),
        })
    }

    /// Attaches a collision body to `link`.
    ///
    /// The body is tested against other links (except `touch_links`) and world
    /// objects. An existing body with the same name is replaced.
    pub fn attach_body(
        &mut self,
        name: impl Into<String>,
        link: Entity,
        shape: ColliderShape,
        local_offset: Transform3,
        touch_links: Vec<Entity>,
    ) -> Result<(), KinematicsError> {
        if self.model.link_index(link).is_none() {
            return Err(KinematicsError::UnknownLink(link));
        }
        let name = name.into();
        self.attached.retain(|body| body.name != name);
        self.attached.push(AttachedBody {
            name,
            link,
            shape,
            local_offset,
            groups: CollisionGroups::default(),
            touch_links,
        });
        Ok(())
    }

    /// Removes an attached body by name, returning whether it existed.
    pub fn detach_body(&mut self, name: &str) -> bool {
        let before = self.attached.len();
        self.attached.retain(|body| body.name != name);
        self.attached.len() != before
    }

    /// Attached bodies in insertion order.
    pub fn attached_bodies(&self) -> &[AttachedBody] {
        &self.attached
    }

    fn attached_primitive(
        &self,
        attached: &AttachedBody,
        state: &ForwardKinematics,
    ) -> Option<CollisionPrimitive> {
        let index = self.model.link_index(attached.link)?;
        let link_transform = state.transform_at(index)?;
        let world_transform = link_transform.mul_transform(&attached.local_offset);
        CollisionPrimitive::from_shape(&attached.shape, &world_transform)
    }

    /// Whether the segment `a`-`b` is occluded by a robot link or attached body.
    ///
    /// Links in `excluded_links` (for example the sensor link) are ignored.
    pub fn segment_blocked(
        &self,
        q: &[f64],
        a: Vec3,
        b: Vec3,
        excluded_links: &[Entity],
    ) -> Result<bool, KinematicsError> {
        let state = self.model.forward_kinematics(q)?;
        for collider in &self.colliders {
            if excluded_links.contains(&collider.link) {
                continue;
            }
            let Some(primitive) = primitive_for(collider, &state) else {
                continue;
            };
            if segment_intersects_primitive(&primitive, a, b) {
                return Ok(true);
            }
        }
        for attached in &self.attached {
            if excluded_links.contains(&attached.link) {
                continue;
            }
            let Some(primitive) = self.attached_primitive(attached, &state) else {
                continue;
            };
            if segment_intersects_primitive(&primitive, a, b) {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Builds a checker that additionally skips every pair in `allowed`.
    ///
    /// Structural exclusion (same link and parent/child pairs) still applies.
    pub fn from_robot_with_allowed_collision_matrix(
        world: &World,
        robot: Entity,
        allowed: AllowedCollisionMatrix,
    ) -> Result<Self, KinematicsError> {
        Ok(Self::from_robot(world, robot)?.with_allowed_collision_matrix(allowed))
    }

    /// Replaces the allowed collision matrix, consuming and returning the checker.
    pub fn with_allowed_collision_matrix(mut self, allowed: AllowedCollisionMatrix) -> Self {
        self.allowed = allowed;
        self
    }

    /// The allowed collision matrix used to skip explicit link pairs.
    pub fn allowed_collision_matrix(&self) -> &AllowedCollisionMatrix {
        &self.allowed
    }

    /// The underlying kinematic model.
    pub fn model(&self) -> &KinematicModel {
        &self.model
    }

    /// Evaluates link collision primitives at a joint configuration.
    ///
    /// Primitives that cannot be represented (infinite planes) are omitted.
    pub(crate) fn link_primitives(
        &self,
        q: &[f64],
    ) -> Result<Vec<(Entity, CollisionPrimitive, CollisionGroups)>, KinematicsError> {
        let state = self.model.forward_kinematics(q)?;
        let mut primitives = Vec::with_capacity(self.colliders.len() + self.attached.len());
        for collider in &self.colliders {
            let Some(primitive) = primitive_for(collider, &state) else {
                continue;
            };
            primitives.push((collider.link, primitive, collider.groups));
        }
        for attached in &self.attached {
            let Some(primitive) = self.attached_primitive(attached, &state) else {
                continue;
            };
            primitives.push((attached.link, primitive, attached.groups));
        }
        Ok(primitives)
    }

    /// Evaluates self-collision at a joint configuration.
    pub fn check(&self, q: &[f64]) -> Result<SelfCollisionReport, KinematicsError> {
        let state = self.model.forward_kinematics(q)?;
        let primitives: Vec<Option<CollisionPrimitive>> = self
            .colliders
            .iter()
            .map(|collider| primitive_for(collider, &state))
            .collect();

        let mut pairs = Vec::new();
        for (i, a) in self.colliders.iter().enumerate() {
            for (j, b) in self.colliders.iter().enumerate().skip(i + 1) {
                if self.pair_excluded(a, b) {
                    continue;
                }
                if !groups_interact(&a.groups, &b.groups) {
                    continue;
                }
                let (Some(pa), Some(pb)) = (&primitives[i], &primitives[j]) else {
                    continue;
                };
                if let Some(depth_m) = penetration(pa, pb) {
                    pairs.push(link_pair(a.link, b.link, depth_m));
                }
            }
        }

        for attached in &self.attached {
            let Some(attached_primitive) = self.attached_primitive(attached, &state) else {
                continue;
            };
            for (index, collider) in self.colliders.iter().enumerate() {
                if collider.link == attached.link || attached.touch_links.contains(&collider.link) {
                    continue;
                }
                if !groups_interact(&attached.groups, &collider.groups) {
                    continue;
                }
                let Some(link_primitive) = &primitives[index] else {
                    continue;
                };
                if let Some(depth_m) = penetration(&attached_primitive, link_primitive) {
                    pairs.push(SelfCollisionPair {
                        link_a: attached.link,
                        link_b: collider.link,
                        depth_m,
                        body_a: Some(attached.name.clone()),
                        body_b: None,
                    });
                }
            }
        }

        for (i, a) in self.attached.iter().enumerate() {
            let Some(pa) = self.attached_primitive(a, &state) else {
                continue;
            };
            for b in self.attached.iter().skip(i + 1) {
                if a.link == b.link {
                    continue;
                }
                if a.touch_links.contains(&b.link) || b.touch_links.contains(&a.link) {
                    continue;
                }
                if !groups_interact(&a.groups, &b.groups) {
                    continue;
                }
                let Some(pb) = self.attached_primitive(b, &state) else {
                    continue;
                };
                if let Some(depth_m) = penetration(&pa, &pb) {
                    pairs.push(SelfCollisionPair {
                        link_a: a.link,
                        link_b: b.link,
                        depth_m,
                        body_a: Some(a.name.clone()),
                        body_b: Some(b.name.clone()),
                    });
                }
            }
        }

        Ok(SelfCollisionReport { pairs })
    }

    /// Finds the closest checked link pair and its signed distance.
    ///
    /// This is the MoveIt `distanceRobot` analogue: the reported value is
    /// negative when the closest pair penetrates and positive when separated.
    pub fn distance(&self, q: &[f64]) -> Result<SelfCollisionDistanceReport, KinematicsError> {
        let state = self.model.forward_kinematics(q)?;
        let primitives: Vec<Option<CollisionPrimitive>> = self
            .colliders
            .iter()
            .map(|collider| primitive_for(collider, &state))
            .collect();

        let mut closest: Option<CollisionPairDistance> = None;
        for (i, a) in self.colliders.iter().enumerate() {
            for (j, b) in self.colliders.iter().enumerate().skip(i + 1) {
                if self.pair_excluded(a, b) || !groups_interact(&a.groups, &b.groups) {
                    continue;
                }
                let (Some(pa), Some(pb)) = (&primitives[i], &primitives[j]) else {
                    continue;
                };
                let distance_m = signed_distance(pa, pb);
                if closest.is_none_or(|current| distance_m < current.distance_m) {
                    closest = Some(CollisionPairDistance {
                        link_a: a.link,
                        link_b: b.link,
                        distance_m,
                    });
                }
            }
        }
        Ok(SelfCollisionDistanceReport { closest })
    }

    /// Tests a joint-space path by sampling uniformly between two endpoints.
    ///
    /// This is the MoveIt `isPathValid` analogue. Sampling is deterministic and
    /// stops at the first colliding configuration. `steps` is clamped to at
    /// least one segment, so both endpoints are always tested.
    pub fn check_path(
        &self,
        start: &[f64],
        goal: &[f64],
        config: &PathCollisionConfig,
    ) -> Result<PathCollisionReport, KinematicsError> {
        let dof = self.model.dof();
        if start.len() != dof {
            return Err(KinematicsError::JointCountMismatch {
                provided: start.len(),
                expected: dof,
            });
        }
        if goal.len() != dof {
            return Err(KinematicsError::JointCountMismatch {
                provided: goal.len(),
                expected: dof,
            });
        }

        let steps = config.steps.max(1);
        let mut q = vec![0.0; dof];
        let mut samples_checked = 0;
        for step in 0..=steps {
            let interpolation = step as f64 / steps as f64;
            for (index, value) in q.iter_mut().enumerate() {
                *value = start[index] + (goal[index] - start[index]) * interpolation;
            }
            let report = self.check(&q)?;
            samples_checked += 1;
            if let Some(pair) = report.pairs().first().cloned() {
                return Ok(PathCollisionReport {
                    first: Some(PathCollisionSample {
                        interpolation,
                        pair,
                    }),
                    samples_checked,
                });
            }
        }
        Ok(PathCollisionReport {
            first: None,
            samples_checked,
        })
    }

    fn pair_excluded(&self, a: &LinkCollider, b: &LinkCollider) -> bool {
        self.excluded[a.link_index][b.link_index] || self.allowed.is_allowed(a.link, b.link)
    }
}

/// Convenience wrapper that builds a checker and evaluates a configuration.
pub fn check_self_collisions(
    world: &World,
    robot: Entity,
    q: &[f64],
) -> Result<SelfCollisionReport, KinematicsError> {
    SelfCollisionChecker::from_robot(world, robot)?.check(q)
}

fn model_links(model: &KinematicModel) -> Vec<Entity> {
    (0..model.link_count())
        .filter_map(|index| model.link_entity(index))
        .collect()
}

fn link_distances(model: &KinematicModel) -> Vec<Vec<usize>> {
    let n = model.link_count();
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
    for index in 0..n {
        if let Some(parent) = model.link_parent(index) {
            adjacency[index].push(parent);
            adjacency[parent].push(index);
        }
    }
    for neighbors in &mut adjacency {
        neighbors.sort_unstable();
    }

    let mut distances = vec![vec![usize::MAX; n]; n];
    for (source, row) in distances.iter_mut().enumerate() {
        row[source] = 0;
        let mut frontier = vec![source];
        while !frontier.is_empty() {
            let mut next = Vec::new();
            for node in frontier {
                let node_distance = row[node];
                for &neighbor in &adjacency[node] {
                    if row[neighbor] == usize::MAX {
                        row[neighbor] = node_distance + 1;
                        next.push(neighbor);
                    }
                }
            }
            frontier = next;
        }
    }
    distances
}

/// Contact tolerance in meters below which a pair is treated as non-penetrating.
pub const CONTACT_EPSILON_M: f64 = 1.0e-9;

fn groups_interact(a: &CollisionGroups, b: &CollisionGroups) -> bool {
    (a.memberships & b.filter) != 0 && (b.memberships & a.filter) != 0
}

fn transform_point(transform: &Transform3, point: Vec3) -> Vec3 {
    transform.translation + transform.rotation * (transform.scale * point)
}

fn primitive_for(collider: &LinkCollider, state: &ForwardKinematics) -> Option<CollisionPrimitive> {
    let link_transform = state.transform_at(collider.link_index)?;
    let world_transform = link_transform.mul_transform(&collider.local_offset);
    CollisionPrimitive::from_shape(&collider.shape, &world_transform)
}

fn link_pair(link_a: Entity, link_b: Entity, depth_m: f64) -> SelfCollisionPair {
    SelfCollisionPair {
        link_a,
        link_b,
        depth_m,
        body_a: None,
        body_b: None,
    }
}

/// Returns the penetration depth when two primitives overlap, otherwise `None`.
pub fn penetration(a: &CollisionPrimitive, b: &CollisionPrimitive) -> Option<f64> {
    let distance_m = signed_distance(a, b);
    if distance_m < -CONTACT_EPSILON_M {
        Some(-distance_m)
    } else {
        None
    }
}

/// Whether the segment `a`-`b` intersects a collision primitive.
///
/// This is the occlusion test used for line-of-sight and visibility queries.
pub fn segment_intersects_primitive(primitive: &CollisionPrimitive, a: Vec3, b: Vec3) -> bool {
    match primitive {
        CollisionPrimitive::Sphere { center_m, radius_m } => {
            point_segment_distance(*center_m, a, b) <= *radius_m
        }
        CollisionPrimitive::Capsule { a_m, b_m, radius_m } => {
            segment_segment_distance(a, b, *a_m, *b_m) <= *radius_m
        }
        CollisionPrimitive::Cuboid { .. } => segment_box_distance(a, b, primitive) <= 1.0e-9,
    }
}

/// Signed distance between two convex primitives in meters.
///
/// Positive when separated, negative when penetrating. All supported primitives
/// are convex, so every pair has a closed-form or separating-axis solution.
pub fn signed_distance(a: &CollisionPrimitive, b: &CollisionPrimitive) -> f64 {
    match (a, b) {
        (
            CollisionPrimitive::Sphere {
                center_m: ca,
                radius_m: ra,
            },
            CollisionPrimitive::Sphere {
                center_m: cb,
                radius_m: rb,
            },
        ) => ca.distance(*cb) - (ra + rb),
        (
            CollisionPrimitive::Sphere {
                center_m: c,
                radius_m: r,
            },
            CollisionPrimitive::Capsule {
                a_m: a,
                b_m: b,
                radius_m: rc,
            },
        )
        | (
            CollisionPrimitive::Capsule {
                a_m: a,
                b_m: b,
                radius_m: rc,
            },
            CollisionPrimitive::Sphere {
                center_m: c,
                radius_m: r,
            },
        ) => point_segment_distance(*c, *a, *b) - (r + rc),
        (
            CollisionPrimitive::Capsule {
                a_m: a1,
                b_m: b1,
                radius_m: r1,
            },
            CollisionPrimitive::Capsule {
                a_m: a2,
                b_m: b2,
                radius_m: r2,
            },
        ) => segment_segment_distance(*a1, *b1, *a2, *b2) - (r1 + r2),
        (CollisionPrimitive::Sphere { center_m, radius_m }, cuboid)
        | (cuboid, CollisionPrimitive::Sphere { center_m, radius_m }) => {
            if let CollisionPrimitive::Cuboid { .. } = cuboid {
                point_box_distance(*center_m, cuboid) - *radius_m
            } else {
                f64::MAX
            }
        }
        (
            CollisionPrimitive::Capsule {
                a_m: a,
                b_m: b,
                radius_m: r,
            },
            cuboid,
        )
        | (
            cuboid,
            CollisionPrimitive::Capsule {
                a_m: a,
                b_m: b,
                radius_m: r,
            },
        ) => {
            if let CollisionPrimitive::Cuboid { .. } = cuboid {
                segment_box_distance(*a, *b, cuboid) - *r
            } else {
                f64::MAX
            }
        }
        (CollisionPrimitive::Cuboid { .. }, CollisionPrimitive::Cuboid { .. }) => {
            cuboid_cuboid_signed_distance(a, b)
        }
    }
}

/// Signed distance from a robot primitive to a triangle mesh.
///
/// Sphere and capsule distances are exact (point/segment samples against every
/// triangle). Cuboid uses the mesh AABB, which is conservative.
pub fn signed_distance_primitive_mesh(
    primitive: &CollisionPrimitive,
    mesh: &MeshCollisionObject,
) -> f64 {
    match primitive {
        CollisionPrimitive::Sphere { center_m, radius_m } => {
            mesh.minimum_point_distance(*center_m) - *radius_m
        }
        CollisionPrimitive::Capsule { a_m, b_m, radius_m } => {
            mesh.minimum_segment_distance(*a_m, *b_m) - *radius_m
        }
        CollisionPrimitive::Cuboid {
            center_m,
            axes,
            half_extents_m,
        } => {
            let (min, max) = mesh.aabb();
            let mesh_box = CollisionPrimitive::Cuboid {
                center_m: (min + max) * 0.5,
                axes: [Vec3::X, Vec3::Y, Vec3::Z],
                half_extents_m: (max - min) * 0.5,
            };
            let box_shape = CollisionPrimitive::Cuboid {
                center_m: *center_m,
                axes: *axes,
                half_extents_m: *half_extents_m,
            };
            cuboid_cuboid_signed_distance(&mesh_box, &box_shape)
        }
    }
}

/// Signed distance from a robot primitive to the nearest occupied voxel.
///
/// Returns `f64::MAX` when the grid has no occupied voxels.
pub fn signed_distance_primitive_voxels(
    primitive: &CollisionPrimitive,
    grid: &VoxelGridObject,
) -> f64 {
    let mut minimum = f64::MAX;
    for index in grid.occupied_indices() {
        let voxel = grid.voxel_cuboid(index);
        let distance = match primitive {
            CollisionPrimitive::Sphere { center_m, radius_m } => {
                point_box_distance(*center_m, &voxel) - *radius_m
            }
            CollisionPrimitive::Capsule { a_m, b_m, radius_m } => {
                segment_box_distance(*a_m, *b_m, &voxel) - *radius_m
            }
            CollisionPrimitive::Cuboid { .. } => cuboid_cuboid_signed_distance(primitive, &voxel),
        };
        if distance < minimum {
            minimum = distance;
        }
    }
    minimum
}

fn penetration_primitive_voxels(
    primitive: &CollisionPrimitive,
    grid: &VoxelGridObject,
) -> Option<f64> {
    let distance = signed_distance_primitive_voxels(primitive, grid);
    if distance < -CONTACT_EPSILON_M {
        Some(-distance)
    } else {
        None
    }
}

fn voxel_segment_intersects(grid: &VoxelGridObject, a: Vec3, b: Vec3) -> bool {
    grid.occupied_indices()
        .any(|index| segment_intersects_primitive(&grid.voxel_cuboid(index), a, b))
}

fn penetration_primitive_mesh(
    primitive: &CollisionPrimitive,
    mesh: &MeshCollisionObject,
) -> Option<f64> {
    let distance = signed_distance_primitive_mesh(primitive, mesh);
    if distance < -CONTACT_EPSILON_M {
        Some(-distance)
    } else {
        None
    }
}

fn point_triangle_distance(point: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = point - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return point.distance(a);
    }
    let bp = point - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return point.distance(b);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return point.distance(a + ab * (d1 / (d1 - d3)));
    }
    let cp = point - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return point.distance(c);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return point.distance(a + ac * (d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return point.distance(b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }
    let denominator = 1.0 / (va + vb + vc);
    let v = vb * denominator;
    let w = vc * denominator;
    point.distance(a + ab * v + ac * w)
}

fn segment_triangle_intersects(a: Vec3, b: Vec3, v0: Vec3, v1: Vec3, v2: Vec3) -> bool {
    let direction = b - a;
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let pvec = direction.cross(edge2);
    let determinant = edge1.dot(pvec);
    if determinant.abs() < 1.0e-12 {
        return false;
    }
    let inverse = 1.0 / determinant;
    let tvec = a - v0;
    let u = inverse * tvec.dot(pvec);
    if !(-1.0e-9..=1.0 + 1.0e-9).contains(&u) {
        return false;
    }
    let qvec = tvec.cross(edge1);
    let v = inverse * direction.dot(qvec);
    if v < -1.0e-9 || u + v > 1.0 + 1.0e-9 {
        return false;
    }
    let t = inverse * edge2.dot(qvec);
    (-1.0e-9..=1.0 + 1.0e-9).contains(&t)
}

fn point_segment_distance(point: Vec3, a: Vec3, b: Vec3) -> f64 {
    point.distance(closest_point_on_segment(point, a, b))
}

fn closest_point_on_segment(point: Vec3, a: Vec3, b: Vec3) -> Vec3 {
    let ab = b - a;
    let denominator = ab.length_squared();
    if denominator <= 1.0e-18 {
        return a;
    }
    let t = ((point - a).dot(ab) / denominator).clamp(0.0, 1.0);
    a + ab * t
}

fn segment_segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> f64 {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.length_squared();
    let e = d2.length_squared();
    let f = d2.dot(r);
    let epsilon = 1.0e-18;

    let (mut s, mut t);
    if a <= epsilon && e <= epsilon {
        return p1.distance(p2);
    } else if a <= epsilon {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= epsilon {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denominator = a * e - b * b;
            s = if denominator > epsilon {
                ((b * f - c * e) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }
    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    c1.distance(c2)
}

fn to_local(point: Vec3, cuboid: &CollisionPrimitive) -> Vec3 {
    let CollisionPrimitive::Cuboid {
        center_m,
        axes,
        half_extents_m: _,
    } = cuboid
    else {
        return point;
    };
    let delta = point - *center_m;
    Vec3::new(delta.dot(axes[0]), delta.dot(axes[1]), delta.dot(axes[2]))
}

fn point_box_distance(point: Vec3, cuboid: &CollisionPrimitive) -> f64 {
    let CollisionPrimitive::Cuboid { half_extents_m, .. } = cuboid else {
        return f64::MAX;
    };
    let local = to_local(point, cuboid);
    let clamp = |value: f64, half: f64| value.clamp(-half, half);
    let closest = Vec3::new(
        clamp(local.x, half_extents_m.x),
        clamp(local.y, half_extents_m.y),
        clamp(local.z, half_extents_m.z),
    );
    (local - closest).length()
}

fn segment_box_distance(a: Vec3, b: Vec3, cuboid: &CollisionPrimitive) -> f64 {
    let local_a = to_local(a, cuboid);
    let local_b = to_local(b, cuboid);
    let evaluate = |t: f64| point_box_distance_local(local_a.lerp(local_b, t), cuboid);
    golden_section_minimize(evaluate, 0.0, 1.0)
}

fn point_box_distance_local(point: Vec3, cuboid: &CollisionPrimitive) -> f64 {
    let CollisionPrimitive::Cuboid { half_extents_m, .. } = cuboid else {
        return f64::MAX;
    };
    let dx = (point.x.abs() - half_extents_m.x).max(0.0);
    let dy = (point.y.abs() - half_extents_m.y).max(0.0);
    let dz = (point.z.abs() - half_extents_m.z).max(0.0);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn golden_section_minimize<F: Fn(f64) -> f64>(f: F, mut lo: f64, mut hi: f64) -> f64 {
    const INV_PHI: f64 = 0.618_033_988_749_894_9;
    let mut c = hi - INV_PHI * (hi - lo);
    let mut d = lo + INV_PHI * (hi - lo);
    let mut fc = f(c);
    let mut fd = f(d);
    for _ in 0..80 {
        if fc < fd {
            hi = d;
            d = c;
            fd = fc;
            c = hi - INV_PHI * (hi - lo);
            fc = f(c);
        } else {
            lo = c;
            c = d;
            fc = fd;
            d = lo + INV_PHI * (hi - lo);
            fd = f(d);
        }
    }
    fc.min(fd)
}

fn cuboid_cuboid_signed_distance(a: &CollisionPrimitive, b: &CollisionPrimitive) -> f64 {
    let (
        CollisionPrimitive::Cuboid {
            center_m: ca,
            axes: axes_a,
            half_extents_m: ha,
        },
        CollisionPrimitive::Cuboid {
            center_m: cb,
            axes: axes_b,
            half_extents_m: hb,
        },
    ) = (a, b)
    else {
        return f64::MAX;
    };

    let mut axes: Vec<Vec3> = Vec::with_capacity(15);
    axes.extend_from_slice(axes_a);
    axes.extend_from_slice(axes_b);
    for &axis_a in axes_a {
        for &axis_b in axes_b {
            let cross = axis_a.cross(axis_b);
            if cross.length_squared() > 1.0e-12 {
                axes.push(cross.normalize());
            }
        }
    }

    // Along each separating-axis candidate the gap is
    // `center_gap - (radius_a + radius_b)`. For overlapping boxes the maximum
    // gap is the negated minimum translation; for disjoint boxes it is the
    // Euclidean distance, because the optimal direction is a face normal or an
    // edge-edge normal, both of which are candidates.
    let center_delta = *cb - *ca;
    let mut max_gap = f64::NEG_INFINITY;
    for axis in axes {
        let radius_a = projected_radius(axes_a, ha, axis);
        let radius_b = projected_radius(axes_b, hb, axis);
        let gap = center_delta.dot(axis).abs() - (radius_a + radius_b);
        if gap > max_gap {
            max_gap = gap;
        }
    }
    if max_gap.is_finite() {
        max_gap
    } else {
        f64::MAX
    }
}

fn projected_radius(axes: &[Vec3; 3], half_extents: &Vec3, axis: Vec3) -> f64 {
    let halves = [half_extents.x, half_extents.y, half_extents.z];
    axes.iter()
        .zip(halves)
        .map(|(body_axis, half)| body_axis.dot(axis).abs() * half)
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sphere(center: Vec3, radius: f64) -> CollisionPrimitive {
        CollisionPrimitive::Sphere {
            center_m: center,
            radius_m: radius,
        }
    }

    fn capsule(a: Vec3, b: Vec3, radius: f64) -> CollisionPrimitive {
        CollisionPrimitive::Capsule {
            a_m: a,
            b_m: b,
            radius_m: radius,
        }
    }

    fn cuboid(center: Vec3, half: Vec3) -> CollisionPrimitive {
        CollisionPrimitive::Cuboid {
            center_m: center,
            axes: [Vec3::X, Vec3::Y, Vec3::Z],
            half_extents_m: half,
        }
    }

    #[test]
    fn sphere_sphere_reports_depth() {
        let depth = penetration(&sphere(Vec3::ZERO, 0.5), &sphere(Vec3::X * 0.8, 0.5)).unwrap();
        assert_relative_eq!(depth, 0.2, epsilon = 1e-12);
        assert!(penetration(&sphere(Vec3::ZERO, 0.5), &sphere(Vec3::X * 2.0, 0.5)).is_none());
    }

    #[test]
    fn capsule_capsule_parallel_overlap() {
        let a = capsule(vec3(0.0, -1.0, 0.0), vec3(0.0, 1.0, 0.0), 0.25);
        let b = capsule(vec3(0.4, -1.0, 0.0), vec3(0.4, 1.0, 0.0), 0.25);
        let depth = penetration(&a, &b).unwrap();
        assert_relative_eq!(depth, 0.1, epsilon = 1e-9);
    }

    #[test]
    fn sphere_box_penetration_and_separation() {
        let box_shape = cuboid(vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        let inside = penetration(&sphere(vec3(1.2, 0.0, 0.0), 0.5), &box_shape).unwrap();
        assert_relative_eq!(inside, 0.3, epsilon = 1e-9);
        assert!(penetration(&sphere(vec3(3.0, 0.0, 0.0), 0.5), &box_shape).is_none());
    }

    #[test]
    fn box_box_sat_detects_overlap() {
        let a = cuboid(vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        let b = cuboid(vec3(1.5, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        let depth = penetration(&a, &b).unwrap();
        assert_relative_eq!(depth, 0.5, epsilon = 1e-9);
        let separated = cuboid(vec3(3.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        assert!(penetration(&a, &separated).is_none());
    }

    fn colliding_robot() -> (World, Entity) {
        use crate::components::{Joint, JointKind, JointLimits, Link, Robot};
        use rne_ecs::spawn_named;

        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let link1 = spawn_named(&mut world, "link1");
        let link2 = spawn_named(&mut world, "link2");

        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
            rne_physics::Collider::sphere(0.1),
        ));
        world.entity_mut(link1).insert((
            Link {
                robot,
                name: "link1".into(),
            },
            Transform3::from_translation_rotation(
                Vec3::new(0.3, 0.0, 0.0),
                rne_math::Quat::IDENTITY,
            ),
            rne_physics::Collider::sphere(0.2),
        ));
        world.entity_mut(link2).insert((
            Link {
                robot,
                name: "link2".into(),
            },
            Transform3::from_translation_rotation(
                Vec3::new(0.3, 0.0, 0.0),
                rne_math::Quat::IDENTITY,
            ),
            rne_physics::Collider::sphere(0.2),
        ));
        world.entity_mut(robot).insert(Robot {
            robot_id: Default::default(),
            model_name: "robot".into(),
            base_link: base,
        });

        for (parent, child, kind) in [
            (base, link1, JointKind::Fixed),
            (link1, link2, JointKind::Revolute),
        ] {
            let joint = spawn_named(&mut world, "joint");
            world.entity_mut(joint).insert(Joint {
                robot,
                parent_link: parent,
                child_link: child,
                kind,
                limits: JointLimits::default(),
                axis: Vec3::Z,
                position: 0.0,
                velocity: 0.0,
            });
        }

        (world, robot)
    }

    #[test]
    fn ecs_checker_excludes_adjacent_links_by_default() {
        let (world, robot) = colliding_robot();
        let checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        // link1 and link2 are directly connected, so they are excluded.
        let report = checker.check(&[0.0]).unwrap();
        assert!(report.is_empty(), "pairs={:?}", report.pairs());
    }

    #[test]
    fn ecs_checker_reports_previously_excluded_pair() {
        let (world, robot) = colliding_robot();
        let checker =
            SelfCollisionChecker::from_robot_with_min_link_distance(&world, robot, 0).unwrap();
        let report = checker.check(&[0.0]).unwrap();
        assert_eq!(report.len(), 1);
        let pair = report.pairs()[0].clone();
        // link1 sphere at x=0.3 and link2 sphere at x=0.3+0.3=0.6 overlap by 0.1 m.
        assert_relative_eq!(pair.depth_m, 0.1, epsilon = 1e-9);
    }

    #[test]
    fn cuboid_signed_distance_reports_separation_and_overlap() {
        let a = cuboid(vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        let separated = cuboid(vec3(3.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        assert_relative_eq!(signed_distance(&a, &separated), 1.0, epsilon = 1e-9);
        let overlapping = cuboid(vec3(1.5, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
        assert_relative_eq!(signed_distance(&a, &overlapping), -0.5, epsilon = 1e-9);
    }

    #[test]
    fn allowed_collision_matrix_skips_explicit_pairs() {
        let (world, robot) = colliding_robot();
        let checker =
            SelfCollisionChecker::from_robot_with_min_link_distance(&world, robot, 0).unwrap();
        let report = checker.check(&[0.0]).unwrap();
        assert_eq!(report.len(), 1);
        let pair = report.pairs()[0].clone();

        let mut allowed = AllowedCollisionMatrix::new();
        allowed.allow(pair.link_a, pair.link_b);
        assert!(allowed.is_allowed(pair.link_b, pair.link_a));
        assert_eq!(allowed.len(), 1);

        let filtered = checker.with_allowed_collision_matrix(allowed);
        assert!(filtered.check(&[0.0]).unwrap().is_empty());
    }

    #[test]
    fn distance_reports_signed_minimum() {
        let (world, robot) = colliding_robot();
        let checker =
            SelfCollisionChecker::from_robot_with_min_link_distance(&world, robot, 0).unwrap();
        let report = checker.distance(&[0.0]).unwrap();
        let closest = report.closest().expect("a checked pair");
        assert_relative_eq!(closest.distance_m, -0.1, epsilon = 1e-9);
        assert!(report.is_colliding());
    }

    #[test]
    fn path_check_reports_first_collision() {
        let (world, robot) = colliding_robot();
        let checker =
            SelfCollisionChecker::from_robot_with_min_link_distance(&world, robot, 0).unwrap();
        let report = checker
            .check_path(&[0.0], &[2.0], &PathCollisionConfig::new(8))
            .unwrap();
        assert!(!report.is_valid());
        assert_eq!(report.samples_checked(), 1);
        let first = report.first_collision().expect("a collision");
        assert_relative_eq!(first.interpolation, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn collision_world_reports_robot_object_pair() {
        let (world, robot) = colliding_robot();
        let checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        let collision_world = CollisionWorld::with_objects(vec![CollisionWorldObject::new(
            CollisionPrimitive::Sphere {
                center_m: vec3(0.45, 0.0, 0.0),
                radius_m: 0.2,
            },
        )]);

        let report = collision_world.check(&checker, &[0.0]).unwrap();
        assert!(report.is_colliding());
        let pair = report.pairs().first().expect("a world collision");
        assert_eq!(pair.object, 0);
        assert_relative_eq!(pair.depth_m, 0.25, epsilon = 1e-9);
    }

    #[test]
    fn collision_world_distance_reports_separation() {
        let (world, robot) = colliding_robot();
        let checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        let collision_world = CollisionWorld::with_objects(vec![CollisionWorldObject::new(
            CollisionPrimitive::Sphere {
                center_m: vec3(2.0, 0.0, 0.0),
                radius_m: 0.1,
            },
        )]);
        let report = collision_world.distance(&checker, &[0.0]).unwrap();
        // The farthest link is link2 at x = 0.6 with radius 0.2.
        assert_relative_eq!(report.min_distance_m().unwrap(), 1.1, epsilon = 1e-9);
        assert!(!report.is_colliding());
    }

    fn link_by_name(model: &KinematicModel, name: &str) -> Entity {
        (0..model.link_count())
            .find(|&index| model.link_name(index) == Some(name))
            .and_then(|index| model.link_entity(index))
            .expect("link")
    }

    fn payload_offset() -> Transform3 {
        Transform3::from_translation_rotation(Vec3::new(0.3, 0.0, 0.0), rne_math::Quat::IDENTITY)
    }

    #[test]
    fn attached_body_collides_with_other_link_and_detaches() {
        let (world, robot) = colliding_robot();
        let mut checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        let link1 = link_by_name(checker.model(), "link1");
        let link2 = link_by_name(checker.model(), "link2");
        checker
            .attach_body(
                "payload",
                link1,
                ColliderShape::Sphere { radius_m: 0.1 },
                payload_offset(),
                Vec::new(),
            )
            .unwrap();
        assert_eq!(checker.attached_bodies().len(), 1);

        let report = checker.check(&[0.0]).unwrap();
        let body_pair = report
            .pairs()
            .iter()
            .find(|pair| pair.body_a.as_deref() == Some("payload"))
            .expect("attached body collision");
        assert_eq!(body_pair.link_b, link2);
        assert!(body_pair.depth_m > 0.0);

        assert!(checker.detach_body("payload"));
        assert!(!checker.detach_body("payload"));
        assert!(checker
            .check(&[0.0])
            .unwrap()
            .pairs()
            .iter()
            .all(|pair| pair.body_a.is_none()));
    }

    #[test]
    fn attach_body_rejects_unknown_link() {
        let (mut world, robot) = colliding_robot();
        let foreign = rne_ecs::spawn_named(&mut world, "foreign");
        let mut checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        assert!(checker
            .attach_body(
                "payload",
                foreign,
                ColliderShape::Sphere { radius_m: 0.1 },
                Transform3::IDENTITY,
                Vec::new(),
            )
            .is_err());
    }

    #[test]
    fn collision_world_includes_attached_bodies() {
        let (world, robot) = colliding_robot();
        let mut checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        let link1 = link_by_name(checker.model(), "link1");
        checker
            .attach_body(
                "payload",
                link1,
                ColliderShape::Sphere { radius_m: 0.1 },
                payload_offset(),
                Vec::new(),
            )
            .unwrap();

        let collision_world = CollisionWorld::with_objects(vec![CollisionWorldObject::new(
            CollisionPrimitive::Sphere {
                center_m: vec3(0.6, 0.0, 0.0),
                radius_m: 0.05,
            },
        )]);
        let report = collision_world.check(&checker, &[0.0]).unwrap();
        assert!(report.pairs().iter().any(|pair| pair.link == link1));
    }

    #[test]
    fn segment_intersects_sphere_and_cuboid() {
        let sphere = sphere(Vec3::ZERO, 0.5);
        assert!(segment_intersects_primitive(
            &sphere,
            vec3(-1.0, 0.0, 0.0),
            vec3(1.0, 0.0, 0.0)
        ));
        assert!(!segment_intersects_primitive(
            &sphere,
            vec3(2.0, 0.0, 0.0),
            vec3(3.0, 0.0, 0.0)
        ));
        let box_shape = cuboid(Vec3::ZERO, vec3(1.0, 1.0, 1.0));
        assert!(segment_intersects_primitive(
            &box_shape,
            vec3(0.0, -2.0, 0.0),
            vec3(0.0, 2.0, 0.0)
        ));
        assert!(!segment_intersects_primitive(
            &box_shape,
            vec3(2.0, 0.0, 0.0),
            vec3(3.0, 0.0, 0.0)
        ));
    }

    #[test]
    fn segment_blocked_by_robot_link() {
        let (world, robot) = colliding_robot();
        let checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        assert!(checker
            .segment_blocked(&[0.0], vec3(0.55, 0.0, 0.0), vec3(0.65, 0.0, 0.0), &[])
            .unwrap());
        assert!(!checker
            .segment_blocked(&[0.0], vec3(0.55, 0.5, 0.0), vec3(0.65, 0.5, 0.0), &[])
            .unwrap());
    }

    #[test]
    fn named_world_objects_add_lookup_and_remove() {
        let mut world = CollisionWorld::new();
        world.add_named_object(
            "obstacle",
            CollisionPrimitive::Sphere {
                center_m: Vec3::ZERO,
                radius_m: 0.5,
            },
        );
        assert_eq!(world.object_name(0), Some("obstacle"));
        assert_eq!(
            world.object("obstacle").and_then(|object| object.name()),
            Some("obstacle")
        );
        assert!(world.remove_object("obstacle"));
        assert!(!world.remove_object("obstacle"));
        assert!(world.object("obstacle").is_none());
    }

    fn unit_square_mesh() -> MeshCollisionObject {
        MeshCollisionObject::new(
            "plane",
            vec![
                vec3(0.0, 0.0, 0.0),
                vec3(1.0, 0.0, 0.0),
                vec3(1.0, 1.0, 0.0),
                vec3(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap()
    }

    #[test]
    fn mesh_object_collides_with_sphere_and_blocks_segment() {
        let mesh = unit_square_mesh();
        let intersecting = sphere(vec3(0.5, 0.5, 0.1), 0.2);
        assert_relative_eq!(
            signed_distance_primitive_mesh(&intersecting, &mesh),
            -0.1,
            epsilon = 1e-9
        );
        let far = sphere(vec3(0.5, 0.5, 0.5), 0.1);
        assert_relative_eq!(
            signed_distance_primitive_mesh(&far, &mesh),
            0.4,
            epsilon = 1e-9
        );
        assert!(mesh.segment_intersects(vec3(0.5, 0.5, -1.0), vec3(0.5, 0.5, 1.0)));
        assert!(!mesh.segment_intersects(vec3(2.0, 2.0, -1.0), vec3(2.0, 2.0, 1.0)));
    }

    #[test]
    fn mesh_object_rejects_invalid_indices() {
        let invalid = MeshCollisionObject::new("bad", vec![vec3(0.0, 0.0, 0.0)], vec![[0, 1, 2]]);
        assert!(invalid.is_err());
    }

    #[test]
    fn collision_world_reports_mesh_pair() {
        let (world, robot) = colliding_robot();
        let mut collision_world = CollisionWorld::new();
        collision_world
            .add_mesh_object(
                "floor",
                unit_square_mesh().vertices().to_vec(),
                unit_square_mesh().triangles().to_vec(),
            )
            .unwrap();
        let checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        let report = collision_world.check(&checker, &[0.0]).unwrap();
        assert!(report.pairs().iter().any(|pair| pair.mesh));
    }

    #[test]
    fn voxel_grid_from_points_collides_and_blocks_segments() {
        let grid = VoxelGridObject::from_points(
            "cloud",
            0.1,
            0.1,
            &[vec3(0.0, 0.0, 0.0), vec3(0.01, 0.0, 0.0)],
        )
        .unwrap();
        assert!(grid.occupied_count() >= 1);
        let near = sphere(vec3(0.0, 0.0, 0.0), 0.15);
        assert!(signed_distance_primitive_voxels(&near, &grid) < 0.0);
        let far = sphere(vec3(5.0, 5.0, 5.0), 0.1);
        assert!(signed_distance_primitive_voxels(&far, &grid) > 0.0);
        assert!(voxel_segment_intersects(
            &grid,
            vec3(-1.0, 0.0, 0.0),
            vec3(1.0, 0.0, 0.0)
        ));
        assert!(!voxel_segment_intersects(
            &grid,
            vec3(-1.0, 2.0, 0.0),
            vec3(1.0, 2.0, 0.0)
        ));
    }

    #[test]
    fn collision_world_reports_voxel_pair() {
        let (world, robot) = colliding_robot();
        let mut collision_world = CollisionWorld::new();
        collision_world.add_voxel_grid(
            VoxelGridObject::from_points("cloud", 0.1, 0.1, &[vec3(0.6, 0.0, 0.0)]).unwrap(),
        );
        let checker = SelfCollisionChecker::from_robot(&world, robot).unwrap();
        let report = collision_world.check(&checker, &[0.0]).unwrap();
        assert!(report.pairs().iter().any(|pair| pair.voxel));
    }

    fn vec3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }
}
