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
//! [`SelfCollisionChecker::min_link_distance`] joints) are skipped, mirroring
//! the usual "ignore parent/child contact" behavior of robot models. Explicit
//! [`CollisionGroups`] are honored using the same mask semantics as the physics
//! backends.

use crate::kinematics::{KinematicModel, KinematicsError};
use bevy_ecs::prelude::World;
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_physics::{Collider, ColliderShape, CollisionGroups};
use rne_world::Transform3;

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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SelfCollisionPair {
    /// First link entity.
    pub link_a: Entity,
    /// Second link entity.
    pub link_b: Entity,
    /// Penetration depth in meters; positive when the pair overlaps.
    pub depth_m: f64,
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
        })
    }

    /// The underlying kinematic model.
    pub fn model(&self) -> &KinematicModel {
        &self.model
    }

    /// Evaluates self-collision at a joint configuration.
    pub fn check(&self, q: &[f64]) -> Result<SelfCollisionReport, KinematicsError> {
        let state = self.model.forward_kinematics(q)?;
        let primitives: Vec<Option<CollisionPrimitive>> = self
            .colliders
            .iter()
            .map(|collider| {
                let link_transform = state.transform_at(collider.link_index)?;
                let world_transform = link_transform.mul_transform(&collider.local_offset);
                CollisionPrimitive::from_shape(&collider.shape, &world_transform)
            })
            .collect();

        let mut pairs = Vec::new();
        for (i, a) in self.colliders.iter().enumerate() {
            for (j, b) in self.colliders.iter().enumerate().skip(i + 1) {
                if self.excluded[a.link_index][b.link_index] {
                    continue;
                }
                if !groups_interact(&a.groups, &b.groups) {
                    continue;
                }
                let (Some(pa), Some(pb)) = (&primitives[i], &primitives[j]) else {
                    continue;
                };
                if let Some(depth_m) = penetration(pa, pb) {
                    pairs.push(SelfCollisionPair {
                        link_a: a.link,
                        link_b: b.link,
                        depth_m,
                    });
                }
            }
        }
        Ok(SelfCollisionReport { pairs })
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

/// Returns the penetration depth when two primitives overlap, otherwise `None`.
pub fn penetration(a: &CollisionPrimitive, b: &CollisionPrimitive) -> Option<f64> {
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
        ) => rounded(ca.distance(*cb), ra + rb),
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
        ) => rounded(point_segment_distance(*c, *a, *b), r + rc),
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
        ) => rounded(segment_segment_distance(*a1, *b1, *a2, *b2), r1 + r2),
        (CollisionPrimitive::Sphere { center_m, radius_m }, cuboid)
        | (cuboid, CollisionPrimitive::Sphere { center_m, radius_m }) => {
            if let CollisionPrimitive::Cuboid { .. } = cuboid {
                rounded(point_box_distance(*center_m, cuboid), *radius_m)
            } else {
                None
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
                rounded(segment_box_distance(*a, *b, cuboid), *r)
            } else {
                None
            }
        }
        (CollisionPrimitive::Cuboid { .. }, CollisionPrimitive::Cuboid { .. }) => {
            cuboid_cuboid_penetration(a, b)
        }
    }
}

fn rounded(distance: f64, radius_sum: f64) -> Option<f64> {
    let depth = radius_sum - distance;
    if depth > CONTACT_EPSILON_M {
        Some(depth)
    } else {
        None
    }
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

fn cuboid_cuboid_penetration(a: &CollisionPrimitive, b: &CollisionPrimitive) -> Option<f64> {
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
        return None;
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

    let center_delta = *cb - *ca;
    let mut min_overlap = f64::INFINITY;
    for axis in axes {
        let ra = projected_radius(axes_a, ha, axis);
        let rb = projected_radius(axes_b, hb, axis);
        let distance = center_delta.dot(axis).abs();
        let overlap = ra + rb - distance;
        if overlap <= 0.0 {
            return None;
        }
        if overlap < min_overlap {
            min_overlap = overlap;
        }
    }
    if min_overlap.is_finite() {
        Some(min_overlap)
    } else {
        None
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
        let pair = report.pairs()[0];
        // link1 sphere at x=0.3 and link2 sphere at x=0.3+0.3=0.6 overlap by 0.1 m.
        assert_relative_eq!(pair.depth_m, 0.1, epsilon = 1e-9);
    }

    fn vec3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }
}
