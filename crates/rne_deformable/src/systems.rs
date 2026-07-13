//! Deterministic deformable builders and XPBD stepping systems.

use crate::{
    CableSpec, ClothSpec, ConstraintKind, DeformableBody, DeformableCollider, DeformableKind,
    DeformableMaterial, DeformableSolverConfig, DeformableStepError, DistanceConstraint, Particle,
    PinConstraint, TriangleTopology,
};
use rne_math::Vec3;
use rne_physics::{Collider, ColliderShape, RigidBody, RigidBodyType};
use rne_world::Transform3 as WorldTransform3;
use std::collections::{BTreeMap, BTreeSet};

/// Builds an evenly spaced cable between two world-space endpoints.
pub fn build_cable(spec: CableSpec) -> Result<DeformableBody, DeformableStepError> {
    let CableSpec {
        start_m,
        end_m,
        particle_count,
        total_mass_kg,
        pin_start,
        pin_end,
        material,
    } = spec;
    if particle_count < 2 || !total_mass_kg.is_finite() || total_mass_kg <= 0.0 {
        return Err(DeformableStepError::InvalidState(
            "cable requires at least two particles and positive finite mass".into(),
        ));
    }
    validate_material(material)?;
    let segment = (end_m - start_m) / (particle_count - 1) as f64;
    if !segment.is_finite() || segment.length_squared() <= f64::EPSILON {
        return Err(DeformableStepError::InvalidState(
            "cable endpoints must be finite and distinct".into(),
        ));
    }
    let particle_mass_kg = total_mass_kg / particle_count as f64;
    let particles = (0..particle_count)
        .map(|index| Particle::dynamic(start_m + segment * index as f64, particle_mass_kg))
        .collect::<Vec<_>>();
    let mut constraints = Vec::with_capacity(particle_count.saturating_mul(2));
    for first in 0..particle_count - 1 {
        constraints.push(distance_constraint(
            &particles,
            first,
            first + 1,
            material.structural_compliance_m_n,
            ConstraintKind::Structural,
        ));
    }
    for first in 0..particle_count.saturating_sub(2) {
        constraints.push(distance_constraint(
            &particles,
            first,
            first + 2,
            material.bending_compliance_m_n,
            ConstraintKind::Bending,
        ));
    }
    let mut pins = Vec::with_capacity(2);
    if pin_start {
        pins.push(PinConstraint {
            particle: 0,
            target_position_m: start_m,
        });
    }
    if pin_end {
        pins.push(PinConstraint {
            particle: particle_count - 1,
            target_position_m: end_m,
        });
    }
    Ok(DeformableBody {
        kind: DeformableKind::Cable,
        particles,
        distance_constraints: constraints,
        pin_constraints: pins,
        triangles: TriangleTopology::default(),
        material,
    })
}

/// Builds a rectangular cloth grid in the X/Y plane, indexed row-major.
pub fn build_cloth(spec: ClothSpec) -> Result<DeformableBody, DeformableStepError> {
    let ClothSpec {
        origin_m,
        width_direction_m,
        height_direction_m,
        columns,
        rows,
        total_mass_kg,
        pin_top_edge,
        material,
    } = spec;
    if columns < 2
        || rows < 2
        || !total_mass_kg.is_finite()
        || total_mass_kg <= 0.0
        || !width_direction_m.is_finite()
        || !height_direction_m.is_finite()
        || width_direction_m.length_squared() <= f64::EPSILON
        || height_direction_m.length_squared() <= f64::EPSILON
    {
        return Err(DeformableStepError::InvalidState(
            "cloth requires a 2x2 grid, finite nonzero axes, and positive mass".into(),
        ));
    }
    validate_material(material)?;
    let particle_count = columns * rows;
    let particle_mass_kg = total_mass_kg / particle_count as f64;
    let mut particles = Vec::with_capacity(particle_count);
    for row in 0..rows {
        let v = row as f64 / (rows - 1) as f64;
        for column in 0..columns {
            let u = column as f64 / (columns - 1) as f64;
            particles.push(Particle::dynamic(
                origin_m + width_direction_m * u + height_direction_m * v,
                particle_mass_kg,
            ));
        }
    }
    let index = |column: usize, row: usize| row * columns + column;
    let mut constraints = Vec::new();
    for row in 0..rows {
        for column in 0..columns {
            let current = index(column, row);
            if column + 1 < columns {
                constraints.push(distance_constraint(
                    &particles,
                    current,
                    index(column + 1, row),
                    material.structural_compliance_m_n,
                    ConstraintKind::Structural,
                ));
            }
            if row + 1 < rows {
                constraints.push(distance_constraint(
                    &particles,
                    current,
                    index(column, row + 1),
                    material.structural_compliance_m_n,
                    ConstraintKind::Structural,
                ));
            }
            if column + 1 < columns && row + 1 < rows {
                for diagonal in [index(column + 1, row + 1), index(column, row + 1)] {
                    let from = if diagonal == index(column, row + 1) {
                        index(column + 1, row)
                    } else {
                        current
                    };
                    constraints.push(distance_constraint(
                        &particles,
                        from,
                        diagonal,
                        material.shear_compliance_m_n,
                        ConstraintKind::Shear,
                    ));
                }
            }
            if column + 2 < columns {
                constraints.push(distance_constraint(
                    &particles,
                    current,
                    index(column + 2, row),
                    material.bending_compliance_m_n,
                    ConstraintKind::Bending,
                ));
            }
            if row + 2 < rows {
                constraints.push(distance_constraint(
                    &particles,
                    current,
                    index(column, row + 2),
                    material.bending_compliance_m_n,
                    ConstraintKind::Bending,
                ));
            }
        }
    }
    let pin_constraints = if pin_top_edge {
        (0..columns)
            .map(|column| PinConstraint {
                particle: index(column, 0),
                target_position_m: particles[index(column, 0)].position_m,
            })
            .collect()
    } else {
        Vec::new()
    };
    let mut indices = Vec::with_capacity((columns - 1) * (rows - 1) * 6);
    for row in 0..rows - 1 {
        for column in 0..columns - 1 {
            let a = index(column, row) as u32;
            let b = index(column + 1, row) as u32;
            let c = index(column, row + 1) as u32;
            let d = index(column + 1, row + 1) as u32;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    Ok(DeformableBody {
        kind: DeformableKind::Cloth,
        particles,
        distance_constraints: constraints,
        pin_constraints,
        triangles: TriangleTopology { indices },
        material,
    })
}

/// Advances one deformable body using deterministic sequential XPBD.
pub fn step_deformable(
    body: &mut DeformableBody,
    colliders: &[DeformableCollider],
    gravity_m_s2: Vec3,
    dt_s: f64,
    config: DeformableSolverConfig,
) -> Result<(), DeformableStepError> {
    validate_body(body)?;
    if !dt_s.is_finite() || dt_s <= 0.0 || !gravity_m_s2.is_finite() {
        return Err(DeformableStepError::InvalidDuration);
    }
    if config.substeps == 0 || config.constraint_iterations == 0 {
        return Err(DeformableStepError::InvalidSolverCounts);
    }
    for collider in colliders {
        if !collider.friction.is_finite() || collider.friction < 0.0 {
            return Err(DeformableStepError::InvalidState(
                "collider friction must be finite and nonnegative".into(),
            ));
        }
    }
    let substep_s = dt_s / f64::from(config.substeps);
    let retention = body.material.velocity_retention_per_s.powf(substep_s);
    let self_collision_data = body
        .material
        .self_collision
        .then(|| (self_collision_exclusions(body), pinned_particle_mask(body)));
    for _ in 0..config.substeps {
        for constraint in &mut body.distance_constraints {
            constraint.lambda = 0.0;
        }
        for particle in &mut body.particles {
            particle.previous_position_m = particle.position_m;
            if particle.is_dynamic() {
                particle.velocity_m_s =
                    (particle.velocity_m_s + gravity_m_s2 * substep_s) * retention;
                particle.position_m += particle.velocity_m_s * substep_s;
            }
        }
        for _ in 0..config.constraint_iterations {
            solve_pins(body);
            solve_distances(body, substep_s);
            solve_collisions(body, colliders);
        }
        if let Some((exclusions, pinned)) = &self_collision_data {
            solve_self_collisions(body, exclusions, pinned);
            solve_vertex_triangle_self_collisions(body, exclusions, pinned);
            solve_collisions(body, colliders);
        }
        solve_pins(body);
        for particle in &mut body.particles {
            particle.velocity_m_s =
                (particle.position_m - particle.previous_position_m) / substep_s;
        }
    }
    Ok(())
}

fn self_collision_exclusions(body: &DeformableBody) -> BTreeSet<(usize, usize)> {
    body.distance_constraints
        .iter()
        .map(|constraint| ordered_pair(constraint.first, constraint.second))
        .collect()
}

fn pinned_particle_mask(body: &DeformableBody) -> Vec<bool> {
    let mut pinned = vec![false; body.particles.len()];
    for constraint in &body.pin_constraints {
        pinned[constraint.particle] = true;
    }
    pinned
}

fn solve_self_collisions(
    body: &mut DeformableBody,
    exclusions: &BTreeSet<(usize, usize)>,
    pinned: &[bool],
) {
    let diameter_m = body.material.collision_radius_m * 2.0;
    let mut cells = BTreeMap::<(i64, i64, i64), Vec<usize>>::new();
    for (index, particle) in body.particles.iter().enumerate() {
        cells
            .entry(particle_cell(particle.position_m, diameter_m))
            .or_default()
            .push(index);
    }

    let mut pairs = Vec::new();
    for (first, particle) in body.particles.iter().enumerate() {
        let (cell_x, cell_y, cell_z) = particle_cell(particle.position_m, diameter_m);
        for offset_x in -1_i64..=1 {
            for offset_y in -1_i64..=1 {
                for offset_z in -1_i64..=1 {
                    let neighbor = (
                        cell_x.saturating_add(offset_x),
                        cell_y.saturating_add(offset_y),
                        cell_z.saturating_add(offset_z),
                    );
                    let Some(indices) = cells.get(&neighbor) else {
                        continue;
                    };
                    for &second in indices {
                        if second > first && !exclusions.contains(&(first, second)) {
                            pairs.push((first, second));
                        }
                    }
                }
            }
        }
    }
    pairs.sort_unstable();
    pairs.dedup();

    for (first_index, second_index) in pairs {
        let (first, second) = two_mut(&mut body.particles, first_index, second_index);
        let delta = first.position_m - second.position_m;
        let distance_m = delta.length();
        if distance_m >= diameter_m {
            continue;
        }
        let first_weight = if pinned[first_index] {
            0.0
        } else {
            first.inverse_mass_kg_inv
        };
        let second_weight = if pinned[second_index] {
            0.0
        } else {
            second.inverse_mass_kg_inv
        };
        let weight = first_weight + second_weight;
        if weight <= f64::EPSILON {
            continue;
        }
        let normal = if distance_m > f64::EPSILON {
            delta / distance_m
        } else {
            deterministic_pair_normal(first_index, second_index)
        };
        let correction_m = normal * (diameter_m - distance_m);
        first.position_m += correction_m * (first_weight / weight);
        second.position_m -= correction_m * (second_weight / weight);
    }
}

fn solve_vertex_triangle_self_collisions(
    body: &mut DeformableBody,
    exclusions: &BTreeSet<(usize, usize)>,
    pinned: &[bool],
) {
    if body.kind != DeformableKind::Cloth || body.triangles.indices.is_empty() {
        return;
    }
    let thickness_m = body.material.collision_radius_m * 2.0;
    let triangles = body
        .triangles
        .indices
        .chunks_exact(3)
        .map(|triangle| {
            [
                triangle[0] as usize,
                triangle[1] as usize,
                triangle[2] as usize,
            ]
        })
        .collect::<Vec<_>>();
    let max_edge_m = triangles.iter().fold(0.0_f64, |maximum, triangle| {
        let [first, second, third] = *triangle;
        maximum
            .max(
                body.particles[first]
                    .position_m
                    .distance(body.particles[second].position_m),
            )
            .max(
                body.particles[second]
                    .position_m
                    .distance(body.particles[third].position_m),
            )
            .max(
                body.particles[third]
                    .position_m
                    .distance(body.particles[first].position_m),
            )
    });
    let cell_size_m = max_edge_m.max(thickness_m * 2.0);
    let mut cells = BTreeMap::<(i64, i64, i64), Vec<usize>>::new();
    for (triangle_index, &[first, second, third]) in triangles.iter().enumerate() {
        let positions = [
            body.particles[first].position_m,
            body.particles[second].position_m,
            body.particles[third].position_m,
        ];
        let minimum = positions
            .iter()
            .copied()
            .fold(Vec3::splat(f64::INFINITY), Vec3::min)
            - Vec3::splat(thickness_m);
        let maximum = positions
            .iter()
            .copied()
            .fold(Vec3::splat(f64::NEG_INFINITY), Vec3::max)
            + Vec3::splat(thickness_m);
        let minimum_cell = particle_cell(minimum, cell_size_m);
        let maximum_cell = particle_cell(maximum, cell_size_m);
        for cell_x in minimum_cell.0..=maximum_cell.0 {
            for cell_y in minimum_cell.1..=maximum_cell.1 {
                for cell_z in minimum_cell.2..=maximum_cell.2 {
                    cells
                        .entry((cell_x, cell_y, cell_z))
                        .or_default()
                        .push(triangle_index);
                }
            }
        }
    }

    let mut candidates = Vec::new();
    for (particle_index, particle) in body.particles.iter().enumerate() {
        let cell = particle_cell(particle.position_m, cell_size_m);
        let Some(triangle_indices) = cells.get(&cell) else {
            continue;
        };
        for &triangle_index in triangle_indices {
            let triangle = triangles[triangle_index];
            if triangle.contains(&particle_index)
                || triangle
                    .iter()
                    .any(|&vertex| exclusions.contains(&ordered_pair(particle_index, vertex)))
            {
                continue;
            }
            candidates.push((particle_index, triangle_index));
        }
    }
    candidates.sort_unstable();
    candidates.dedup();

    for (particle_index, triangle_index) in candidates {
        let [first, second, third] = triangles[triangle_index];
        let particle_position = body.particles[particle_index].position_m;
        let triangle_positions = [
            body.particles[first].position_m,
            body.particles[second].position_m,
            body.particles[third].position_m,
        ];
        let (closest, barycentric) = closest_point_on_triangle(
            particle_position,
            triangle_positions[0],
            triangle_positions[1],
            triangle_positions[2],
        );
        let delta = particle_position - closest;
        let distance_m = delta.length();
        if distance_m >= thickness_m {
            continue;
        }
        let normal = if distance_m > f64::EPSILON {
            delta / distance_m
        } else {
            deterministic_triangle_normal(triangle_positions, particle_index, triangle_index)
        };
        let particle_weight = effective_inverse_mass(body, pinned, particle_index);
        let triangle_weights = [
            effective_inverse_mass(body, pinned, first),
            effective_inverse_mass(body, pinned, second),
            effective_inverse_mass(body, pinned, third),
        ];
        let denominator = particle_weight
            + triangle_weights
                .iter()
                .zip(barycentric)
                .map(|(weight, barycentric)| weight * barycentric * barycentric)
                .sum::<f64>();
        if denominator <= f64::EPSILON {
            continue;
        }
        let correction_m = normal * ((thickness_m - distance_m) / denominator);
        body.particles[particle_index].position_m += correction_m * particle_weight;
        for ((vertex, weight), barycentric) in [first, second, third]
            .into_iter()
            .zip(triangle_weights)
            .zip(barycentric)
        {
            body.particles[vertex].position_m -= correction_m * (weight * barycentric);
        }
    }
}

fn effective_inverse_mass(body: &DeformableBody, pinned: &[bool], particle: usize) -> f64 {
    if pinned[particle] {
        0.0
    } else {
        body.particles[particle].inverse_mass_kg_inv
    }
}

fn closest_point_on_triangle(
    point: Vec3,
    first: Vec3,
    second: Vec3,
    third: Vec3,
) -> (Vec3, [f64; 3]) {
    let first_second = second - first;
    let first_third = third - first;
    if first_second.cross(first_third).length_squared() <= f64::EPSILON {
        return closest_point_on_degenerate_triangle(point, first, second, third);
    }
    let first_point = point - first;
    let d1 = first_second.dot(first_point);
    let d2 = first_third.dot(first_point);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (first, [1.0, 0.0, 0.0]);
    }

    let second_point = point - second;
    let d3 = first_second.dot(second_point);
    let d4 = first_third.dot(second_point);
    if d3 >= 0.0 && d4 <= d3 {
        return (second, [0.0, 1.0, 0.0]);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let second_weight = d1 / (d1 - d3);
        return (
            first + first_second * second_weight,
            [1.0 - second_weight, second_weight, 0.0],
        );
    }

    let third_point = point - third;
    let d5 = first_second.dot(third_point);
    let d6 = first_third.dot(third_point);
    if d6 >= 0.0 && d5 <= d6 {
        return (third, [0.0, 0.0, 1.0]);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let third_weight = d2 / (d2 - d6);
        return (
            first + first_third * third_weight,
            [1.0 - third_weight, 0.0, third_weight],
        );
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let third_weight = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (
            second + (third - second) * third_weight,
            [0.0, 1.0 - third_weight, third_weight],
        );
    }

    let inverse = (va + vb + vc).recip();
    let second_weight = vb * inverse;
    let third_weight = vc * inverse;
    (
        first + first_second * second_weight + first_third * third_weight,
        [
            1.0 - second_weight - third_weight,
            second_weight,
            third_weight,
        ],
    )
}

fn closest_point_on_degenerate_triangle(
    point: Vec3,
    first: Vec3,
    second: Vec3,
    third: Vec3,
) -> (Vec3, [f64; 3]) {
    let candidates = [
        closest_point_on_segment(point, first, second, [0, 1]),
        closest_point_on_segment(point, second, third, [1, 2]),
        closest_point_on_segment(point, third, first, [2, 0]),
    ];
    candidates
        .into_iter()
        .min_by(|left, right| {
            point
                .distance_squared(left.0)
                .total_cmp(&point.distance_squared(right.0))
        })
        .expect("three deterministic edge candidates")
}

fn closest_point_on_segment(
    point: Vec3,
    start: Vec3,
    end: Vec3,
    vertices: [usize; 2],
) -> (Vec3, [f64; 3]) {
    let segment = end - start;
    let denominator = segment.length_squared();
    let end_weight = if denominator > f64::EPSILON {
        ((point - start).dot(segment) / denominator).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut barycentric = [0.0; 3];
    barycentric[vertices[0]] = 1.0 - end_weight;
    barycentric[vertices[1]] = end_weight;
    (start + segment * end_weight, barycentric)
}

fn deterministic_triangle_normal(positions: [Vec3; 3], particle: usize, triangle: usize) -> Vec3 {
    let face_normal = (positions[1] - positions[0]).cross(positions[2] - positions[0]);
    if face_normal.length_squared() > f64::EPSILON {
        let normal = face_normal.normalize();
        if particle.wrapping_add(triangle).is_multiple_of(2) {
            normal
        } else {
            -normal
        }
    } else {
        deterministic_pair_normal(particle, triangle)
    }
}

fn particle_cell(position_m: Vec3, cell_size_m: f64) -> (i64, i64, i64) {
    (
        (position_m.x / cell_size_m).floor() as i64,
        (position_m.y / cell_size_m).floor() as i64,
        (position_m.z / cell_size_m).floor() as i64,
    )
}

fn ordered_pair(first: usize, second: usize) -> (usize, usize) {
    if first < second {
        (first, second)
    } else {
        (second, first)
    }
}

fn deterministic_pair_normal(first: usize, second: usize) -> Vec3 {
    match first.wrapping_mul(31).wrapping_add(second) % 3 {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    }
}

/// Advances every deformable ECS component against fixed and kinematic colliders.
///
/// Collider and deformable entities are sorted by stable ECS entity bits before
/// stepping, so externally visible results do not depend on query iteration
/// order. Dynamic rigid bodies are intentionally excluded by the one-way MVP.
pub fn step_deformable_world(
    world: &mut rne_ecs::World,
    gravity_m_s2: Vec3,
    dt_s: f64,
    config: DeformableSolverConfig,
) -> Result<(), DeformableStepError> {
    let mut sampled = world
        .query::<(rne_ecs::Entity, &RigidBody, &Collider, &WorldTransform3)>()
        .iter(world)
        .filter(|(_, body, collider, _)| {
            body.body_type != RigidBodyType::Dynamic && !collider.sensor
        })
        .map(|(entity, _, collider, transform)| {
            let world_transform = transform.mul_transform(&collider.local_offset);
            (
                entity.to_bits(),
                DeformableCollider {
                    shape: collider.shape,
                    world_transform: rne_math::Transform3 {
                        translation: world_transform.translation,
                        rotation: world_transform.rotation,
                        scale: world_transform.scale,
                    },
                    friction: f64::from(collider.material.friction),
                },
            )
        })
        .collect::<Vec<_>>();
    sampled.sort_by_key(|(bits, _)| *bits);
    let colliders = sampled
        .into_iter()
        .map(|(_, collider)| collider)
        .collect::<Vec<_>>();

    let mut deformables = world
        .query_filtered::<rne_ecs::Entity, bevy_ecs::query::With<DeformableBody>>()
        .iter(world)
        .collect::<Vec<_>>();
    deformables.sort_by_key(|entity| entity.to_bits());
    for entity in deformables {
        let mut body = world
            .get_mut::<DeformableBody>(entity)
            .expect("entity selected by DeformableBody filter");
        step_deformable(&mut body, &colliders, gravity_m_s2, dt_s, config)?;
    }
    Ok(())
}

fn distance_constraint(
    particles: &[Particle],
    first: usize,
    second: usize,
    compliance_m_n: f64,
    kind: ConstraintKind,
) -> DistanceConstraint {
    DistanceConstraint {
        first,
        second,
        rest_length_m: particles[first]
            .position_m
            .distance(particles[second].position_m),
        compliance_m_n,
        kind,
        lambda: 0.0,
    }
}

fn solve_pins(body: &mut DeformableBody) {
    for pin in &body.pin_constraints {
        let particle = &mut body.particles[pin.particle];
        particle.position_m = pin.target_position_m;
        particle.previous_position_m = pin.target_position_m;
        particle.velocity_m_s = Vec3::ZERO;
    }
}

fn solve_distances(body: &mut DeformableBody, substep_s: f64) {
    for constraint in &mut body.distance_constraints {
        let (first, second) = two_mut(&mut body.particles, constraint.first, constraint.second);
        let delta = first.position_m - second.position_m;
        let length = delta.length();
        if length <= f64::EPSILON {
            continue;
        }
        let weight = first.inverse_mass_kg_inv + second.inverse_mass_kg_inv;
        if weight <= f64::EPSILON {
            continue;
        }
        let alpha = constraint.compliance_m_n / (substep_s * substep_s);
        let value = length - constraint.rest_length_m;
        let delta_lambda = (-value - alpha * constraint.lambda) / (weight + alpha);
        constraint.lambda += delta_lambda;
        let correction = delta * (delta_lambda / length);
        first.position_m += correction * first.inverse_mass_kg_inv;
        second.position_m -= correction * second.inverse_mass_kg_inv;
    }
}

fn solve_collisions(body: &mut DeformableBody, colliders: &[DeformableCollider]) {
    let radius_m = body.material.collision_radius_m;
    for particle in &mut body.particles {
        if !particle.is_dynamic() {
            continue;
        }
        for collider in colliders {
            let before = particle.position_m;
            let Some((projected, normal)) =
                project_particle(particle.position_m, radius_m, collider)
            else {
                continue;
            };
            let normal_correction_m = (projected - before).length();
            particle.position_m = projected;
            let displacement = particle.position_m - particle.previous_position_m;
            let tangent = displacement - normal * displacement.dot(normal);
            let tangent_length = tangent.length();
            let max_friction_m = collider.friction * normal_correction_m;
            if tangent_length > f64::EPSILON && max_friction_m > 0.0 {
                particle.position_m -=
                    tangent * (max_friction_m.min(tangent_length) / tangent_length);
            }
        }
    }
}

fn project_particle(
    position_m: Vec3,
    particle_radius_m: f64,
    collider: &DeformableCollider,
) -> Option<(Vec3, Vec3)> {
    let inverse = collider.world_transform.inverse();
    let local = inverse.transform_point(position_m);
    let (projected_local, normal_local) = match collider.shape {
        ColliderShape::Plane { normal } => {
            let normal = normal.normalize_or_zero();
            let distance = local.dot(normal);
            if distance >= particle_radius_m {
                return None;
            }
            (local + normal * (particle_radius_m - distance), normal)
        }
        ColliderShape::Sphere { radius_m } => {
            let required = radius_m + particle_radius_m;
            let distance = local.length();
            if distance >= required {
                return None;
            }
            let normal = if distance > f64::EPSILON {
                local / distance
            } else {
                Vec3::Y
            };
            (normal * required, normal)
        }
        ColliderShape::Cuboid { half_extents_m } => {
            let expanded = half_extents_m + Vec3::splat(particle_radius_m);
            if local.x.abs() >= expanded.x
                || local.y.abs() >= expanded.y
                || local.z.abs() >= expanded.z
            {
                return None;
            }
            let distances = [
                expanded.x - local.x,
                expanded.x + local.x,
                expanded.y - local.y,
                expanded.y + local.y,
                expanded.z - local.z,
                expanded.z + local.z,
            ];
            let (face, _) = distances
                .iter()
                .enumerate()
                .min_by(|left, right| left.1.total_cmp(right.1))?;
            let normal = match face {
                0 => Vec3::X,
                1 => -Vec3::X,
                2 => Vec3::Y,
                3 => -Vec3::Y,
                4 => Vec3::Z,
                _ => -Vec3::Z,
            };
            let projected = match face {
                0 => Vec3::new(expanded.x, local.y, local.z),
                1 => Vec3::new(-expanded.x, local.y, local.z),
                2 => Vec3::new(local.x, expanded.y, local.z),
                3 => Vec3::new(local.x, -expanded.y, local.z),
                4 => Vec3::new(local.x, local.y, expanded.z),
                _ => Vec3::new(local.x, local.y, -expanded.z),
            };
            (projected, normal)
        }
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => {
            let axis_point = Vec3::new(0.0, local.y.clamp(-half_height_m, half_height_m), 0.0);
            let offset = local - axis_point;
            let distance = offset.length();
            let required = radius_m + particle_radius_m;
            if distance >= required {
                return None;
            }
            let normal = if distance > f64::EPSILON {
                offset / distance
            } else {
                Vec3::X
            };
            (axis_point + normal * required, normal)
        }
    };
    let projected_world = collider.world_transform.transform_point(projected_local);
    let normal_world = (collider.world_transform.rotation * normal_local).normalize_or_zero();
    Some((projected_world, normal_world))
}

fn two_mut<T>(values: &mut [T], first: usize, second: usize) -> (&mut T, &mut T) {
    debug_assert_ne!(first, second);
    if first < second {
        let (left, right) = values.split_at_mut(second);
        (&mut left[first], &mut right[0])
    } else {
        let (left, right) = values.split_at_mut(first);
        (&mut right[0], &mut left[second])
    }
}

fn validate_material(material: DeformableMaterial) -> Result<(), DeformableStepError> {
    if !material.collision_radius_m.is_finite()
        || material.collision_radius_m <= 0.0
        || !material.structural_compliance_m_n.is_finite()
        || material.structural_compliance_m_n < 0.0
        || !material.shear_compliance_m_n.is_finite()
        || material.shear_compliance_m_n < 0.0
        || !material.bending_compliance_m_n.is_finite()
        || material.bending_compliance_m_n < 0.0
        || !material.velocity_retention_per_s.is_finite()
        || !(0.0..=1.0).contains(&material.velocity_retention_per_s)
        || material.velocity_retention_per_s == 0.0
    {
        return Err(DeformableStepError::InvalidState(
            "material values are outside their finite physical ranges".into(),
        ));
    }
    Ok(())
}

fn validate_body(body: &DeformableBody) -> Result<(), DeformableStepError> {
    validate_material(body.material)?;
    if body.particles.is_empty()
        || body.particles.iter().any(|particle| {
            !particle.position_m.is_finite()
                || !particle.previous_position_m.is_finite()
                || !particle.velocity_m_s.is_finite()
                || !particle.inverse_mass_kg_inv.is_finite()
                || particle.inverse_mass_kg_inv < 0.0
        })
        || body.distance_constraints.iter().any(|constraint| {
            constraint.first >= body.particles.len()
                || constraint.second >= body.particles.len()
                || constraint.first == constraint.second
                || !constraint.rest_length_m.is_finite()
                || constraint.rest_length_m <= 0.0
                || !constraint.compliance_m_n.is_finite()
                || constraint.compliance_m_n < 0.0
        })
        || body
            .pin_constraints
            .iter()
            .any(|pin| pin.particle >= body.particles.len() || !pin.target_position_m.is_finite())
        || !body.triangles.indices.len().is_multiple_of(3)
        || body.triangles.indices.chunks_exact(3).any(|triangle| {
            let first = triangle[0] as usize;
            let second = triangle[1] as usize;
            let third = triangle[2] as usize;
            first >= body.particles.len()
                || second >= body.particles.len()
                || third >= body.particles.len()
                || first == second
                || second == third
                || third == first
        })
    {
        return Err(DeformableStepError::InvalidState(
            "particles or constraints are invalid".into(),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::{Quat, Transform3};

    fn step_many(body: &mut DeformableBody, colliders: &[DeformableCollider], steps: usize) {
        for _ in 0..steps {
            step_deformable(
                body,
                colliders,
                Vec3::new(0.0, -9.81, 0.0),
                1.0 / 60.0,
                DeformableSolverConfig::default(),
            )
            .expect("valid deformable step");
        }
    }

    #[test]
    fn pinned_cable_sags_and_replays_exactly() {
        let make = || {
            build_cable(CableSpec {
                start_m: Vec3::new(-0.5, 1.0, 0.0),
                end_m: Vec3::new(0.5, 1.0, 0.0),
                particle_count: 17,
                total_mass_kg: 0.2,
                pin_start: true,
                pin_end: true,
                material: DeformableMaterial::default(),
            })
            .expect("cable")
        };
        let mut first = make();
        let mut second = make();
        step_many(&mut first, &[], 180);
        step_many(&mut second, &[], 180);
        assert_eq!(first, second);
        assert_eq!(first.stable_state_hash(), second.stable_state_hash());
        assert!(first.particles[8].position_m.y < 0.98);
        assert_eq!(first.particles[0].position_m, Vec3::new(-0.5, 1.0, 0.0));
        assert_eq!(first.particles[16].position_m, Vec3::new(0.5, 1.0, 0.0));
    }

    #[test]
    fn cable_particles_rest_on_plane_without_penetrating() {
        let mut cable = build_cable(CableSpec {
            start_m: Vec3::new(-0.4, 0.8, 0.0),
            end_m: Vec3::new(0.4, 0.8, 0.0),
            particle_count: 13,
            total_mass_kg: 0.15,
            pin_start: false,
            pin_end: false,
            material: DeformableMaterial::default(),
        })
        .expect("cable");
        let ground = DeformableCollider {
            shape: ColliderShape::Plane { normal: Vec3::Y },
            world_transform: Transform3::IDENTITY,
            friction: 0.6,
        };
        step_many(&mut cable, &[ground], 240);
        assert!(cable
            .particles
            .iter()
            .all(|particle| particle.position_m.y >= cable.material.collision_radius_m - 1.0e-9));
    }

    #[test]
    fn every_supported_primitive_projects_a_particle() {
        let shapes = [
            ColliderShape::Plane { normal: Vec3::Y },
            ColliderShape::Cuboid {
                half_extents_m: Vec3::splat(0.2),
            },
            ColliderShape::Sphere { radius_m: 0.2 },
            ColliderShape::Capsule {
                half_height_m: 0.2,
                radius_m: 0.1,
            },
        ];
        for shape in shapes {
            let collider = DeformableCollider {
                shape,
                world_transform: Transform3::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY),
                friction: 0.0,
            };
            let projected = project_particle(Vec3::ZERO, 0.01, &collider)
                .expect("origin should be inside each test collider");
            assert!(projected.0.is_finite());
            assert_relative_eq!(projected.1.length(), 1.0, epsilon = 1.0e-12);
        }
    }

    #[test]
    fn cloth_builder_creates_grid_constraints_and_triangles() {
        let cloth = build_cloth(ClothSpec {
            origin_m: Vec3::new(-0.5, 1.0, 0.0),
            width_direction_m: Vec3::X,
            height_direction_m: -Vec3::Y,
            columns: 8,
            rows: 6,
            total_mass_kg: 0.25,
            pin_top_edge: true,
            material: DeformableMaterial::default(),
        })
        .expect("cloth");
        assert_eq!(cloth.particles.len(), 48);
        assert_eq!(cloth.pin_constraints.len(), 8);
        assert_eq!(cloth.triangles.indices.len(), 7 * 5 * 6);
        assert!(cloth
            .distance_constraints
            .iter()
            .any(|constraint| constraint.kind == ConstraintKind::Shear));
        assert!(cloth
            .distance_constraints
            .iter()
            .any(|constraint| constraint.kind == ConstraintKind::Bending));
    }

    #[test]
    fn pinned_cloth_edge_remains_fixed_under_gravity() {
        let mut cloth = build_cloth(ClothSpec {
            origin_m: Vec3::new(-0.5, 1.0, 0.0),
            width_direction_m: Vec3::X,
            height_direction_m: -Vec3::Y,
            columns: 6,
            rows: 5,
            total_mass_kg: 0.2,
            pin_top_edge: true,
            material: DeformableMaterial::default(),
        })
        .expect("cloth");
        let pinned = cloth
            .pin_constraints
            .iter()
            .map(|pin| (pin.particle, pin.target_position_m))
            .collect::<Vec<_>>();
        step_many(&mut cloth, &[], 120);
        for (particle, target) in pinned {
            assert_eq!(cloth.particles[particle].position_m, target);
            assert_eq!(cloth.particles[particle].velocity_m_s, Vec3::ZERO);
        }
        let surface = cloth.cloth_surface_mesh().expect("cloth surface");
        assert!(surface.normals.iter().all(|normal| {
            let length = normal.iter().map(|value| value * value).sum::<f32>().sqrt();
            (length - 1.0).abs() < 1.0e-5
        }));
    }

    #[test]
    fn positional_friction_reduces_tangential_travel() {
        let make_body = || DeformableBody {
            kind: DeformableKind::Cable,
            particles: vec![Particle {
                position_m: Vec3::new(0.0, 0.01, 0.0),
                previous_position_m: Vec3::new(0.0, 0.01, 0.0),
                velocity_m_s: Vec3::new(1.0, 0.0, 0.0),
                inverse_mass_kg_inv: 1.0,
            }],
            distance_constraints: Vec::new(),
            pin_constraints: Vec::new(),
            triangles: TriangleTopology::default(),
            material: DeformableMaterial::default(),
        };
        let collider = |friction| DeformableCollider {
            shape: ColliderShape::Plane { normal: Vec3::Y },
            world_transform: Transform3::IDENTITY,
            friction,
        };
        let mut frictionless = make_body();
        let mut frictional = make_body();
        step_many(&mut frictionless, &[collider(0.0)], 60);
        step_many(&mut frictional, &[collider(0.9)], 60);
        assert!(frictional.particles[0].position_m.x < frictionless.particles[0].position_m.x);
    }

    #[test]
    fn self_collision_separates_non_adjacent_particles_deterministically() {
        let make_body = || {
            let material = DeformableMaterial {
                self_collision: true,
                ..DeformableMaterial::default()
            };
            DeformableBody {
                kind: DeformableKind::Cloth,
                particles: vec![
                    Particle::dynamic(Vec3::ZERO, 0.5),
                    Particle::dynamic(Vec3::ZERO, 0.5),
                ],
                distance_constraints: Vec::new(),
                pin_constraints: Vec::new(),
                triangles: TriangleTopology::default(),
                material,
            }
        };
        let mut first = make_body();
        let mut replay = make_body();
        let config = DeformableSolverConfig {
            substeps: 1,
            constraint_iterations: 1,
        };
        step_deformable(&mut first, &[], Vec3::ZERO, 1.0 / 60.0, config)
            .expect("self-collision step");
        step_deformable(&mut replay, &[], Vec3::ZERO, 1.0 / 60.0, config).expect("replay step");

        assert_eq!(first, replay);
        assert_eq!(first.stable_state_hash(), replay.stable_state_hash());
        assert_relative_eq!(
            first.particles[0]
                .position_m
                .distance(first.particles[1].position_m),
            first.material.collision_radius_m * 2.0,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn self_collision_does_not_inflate_constraint_neighbors() {
        let material = DeformableMaterial {
            collision_radius_m: 0.01,
            self_collision: true,
            ..DeformableMaterial::default()
        };
        let mut cable = build_cable(CableSpec {
            start_m: Vec3::ZERO,
            end_m: Vec3::new(0.02, 0.0, 0.0),
            particle_count: 3,
            total_mass_kg: 0.03,
            pin_start: false,
            pin_end: false,
            material,
        })
        .expect("short cable");
        let initial = cable
            .particles
            .iter()
            .map(|particle| particle.position_m)
            .collect::<Vec<_>>();
        step_deformable(
            &mut cable,
            &[],
            Vec3::ZERO,
            1.0 / 60.0,
            DeformableSolverConfig::default(),
        )
        .expect("self-collision step");
        assert_eq!(
            cable
                .particles
                .iter()
                .map(|particle| particle.position_m)
                .collect::<Vec<_>>(),
            initial
        );
    }

    #[test]
    fn vertex_triangle_self_collision_stops_surface_penetration_and_replays() {
        let make_body = || DeformableBody {
            kind: DeformableKind::Cloth,
            particles: vec![
                Particle::dynamic(Vec3::new(-0.5, 0.0, -0.5), 0.25),
                Particle::dynamic(Vec3::new(0.5, 0.0, -0.5), 0.25),
                Particle::dynamic(Vec3::new(0.0, 0.0, 0.5), 0.25),
                Particle::dynamic(Vec3::new(0.0, 0.005, 0.0), 0.25),
            ],
            distance_constraints: Vec::new(),
            pin_constraints: Vec::new(),
            triangles: TriangleTopology {
                indices: vec![0, 1, 2],
            },
            material: DeformableMaterial {
                collision_radius_m: 0.01,
                self_collision: true,
                ..DeformableMaterial::default()
            },
        };
        let mut first = make_body();
        let mut replay = make_body();
        for body in [&mut first, &mut replay] {
            step_deformable(
                body,
                &[],
                Vec3::ZERO,
                1.0 / 60.0,
                DeformableSolverConfig::default(),
            )
            .expect("vertex-triangle self-collision step");
        }

        assert_eq!(first, replay);
        let triangle = [
            first.particles[0].position_m,
            first.particles[1].position_m,
            first.particles[2].position_m,
        ];
        let (closest, _) = closest_point_on_triangle(
            first.particles[3].position_m,
            triangle[0],
            triangle[1],
            triangle[2],
        );
        assert!(
            first.particles[3].position_m.distance(closest)
                >= first.material.collision_radius_m * 2.0 - 1.0e-10
        );
    }

    #[test]
    fn vertex_triangle_self_collision_ignores_topological_neighbors() {
        let material = DeformableMaterial {
            collision_radius_m: 0.2,
            self_collision: true,
            ..DeformableMaterial::default()
        };
        let mut cloth = build_cloth(ClothSpec {
            origin_m: Vec3::ZERO,
            width_direction_m: Vec3::X * 0.1,
            height_direction_m: Vec3::Z * 0.1,
            columns: 2,
            rows: 2,
            total_mass_kg: 0.04,
            pin_top_edge: false,
            material,
        })
        .expect("small cloth");
        let initial = cloth
            .particles
            .iter()
            .map(|particle| particle.position_m)
            .collect::<Vec<_>>();
        step_deformable(
            &mut cloth,
            &[],
            Vec3::ZERO,
            1.0 / 60.0,
            DeformableSolverConfig::default(),
        )
        .expect("adjacent cloth step");
        assert_eq!(
            cloth
                .particles
                .iter()
                .map(|particle| particle.position_m)
                .collect::<Vec<_>>(),
            initial
        );
    }

    #[test]
    fn malformed_triangle_topology_is_rejected_before_stepping() {
        let mut body = DeformableBody {
            kind: DeformableKind::Cloth,
            particles: vec![Particle::dynamic(Vec3::ZERO, 1.0)],
            distance_constraints: Vec::new(),
            pin_constraints: Vec::new(),
            triangles: TriangleTopology {
                indices: vec![0, 1, 2],
            },
            material: DeformableMaterial::default(),
        };
        let error = step_deformable(
            &mut body,
            &[],
            Vec3::ZERO,
            1.0 / 60.0,
            DeformableSolverConfig::default(),
        )
        .expect_err("out-of-range triangle must fail validation");
        assert!(matches!(error, DeformableStepError::InvalidState(_)));
    }

    #[test]
    fn world_step_samples_kinematic_colliders() {
        use rne_physics::{Collider, RigidBody};
        use rne_world::Transform3 as WorldTransform3;

        let mut world = rne_ecs::World::new();
        let collider = world
            .spawn((
                RigidBody {
                    body_type: RigidBodyType::Kinematic,
                    ..RigidBody::default()
                },
                Collider::sphere(0.2),
                WorldTransform3::IDENTITY,
            ))
            .id();
        let body = DeformableBody {
            kind: DeformableKind::Cable,
            particles: vec![Particle::dynamic(Vec3::ZERO, 1.0)],
            distance_constraints: Vec::new(),
            pin_constraints: Vec::new(),
            triangles: TriangleTopology::default(),
            material: DeformableMaterial::default(),
        };
        let deformable = world.spawn(body).id();
        step_deformable_world(
            &mut world,
            Vec3::ZERO,
            1.0 / 60.0,
            DeformableSolverConfig::default(),
        )
        .expect("world step");
        assert!(
            world
                .get::<DeformableBody>(deformable)
                .expect("deformable")
                .particles[0]
                .position_m
                .length()
                >= 0.21 - 1.0e-10
        );
        assert!(world.get::<RigidBody>(collider).is_some());
    }
}
