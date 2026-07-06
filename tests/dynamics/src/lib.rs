//! Shared harness, analytic references, and documented tolerances for dynamics validation.

#![allow(missing_docs)]

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsMaterial, PhysicsWorldDesc, PhysicsWorldId,
    RevoluteJointDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_world::{world_transform_of, Transform3};

/// Standard gravity magnitude used in validation scenarios (m/s²).
pub const G: f64 = 9.81;

/// Default Rapier step rate for most scenarios (Hz).
pub const DEFAULT_HZ: f64 = 60.0;

/// Bare Rapier world + ECS for analytic comparison tests.
pub struct PhysicsHarness {
    /// Rapier backend instance.
    pub backend: RapierBackend,
    /// Active physics world handle.
    pub physics_world: PhysicsWorldId,
    /// ECS world mirrored into Rapier.
    pub world: World,
    /// Gravity vector from the world descriptor (negative Y by default).
    pub gravity_m_s2: Vec3,
}

impl PhysicsHarness {
    /// Creates an empty physics harness with the given world description.
    pub fn new(desc: PhysicsWorldDesc) -> Self {
        let gravity_m_s2 = desc.gravity_m_s2;
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(desc).expect("physics world creation");
        Self {
            backend,
            physics_world,
            world: World::new(),
            gravity_m_s2,
        }
    }

    /// Advances the simulation for `steps` fixed substeps at `hz`.
    pub fn step_hz(&mut self, hz: f64, steps: u32) {
        let dt = SimDuration::from_hertz(Hertz::new(hz));
        for _ in 0..steps {
            step_physics(&mut self.backend, &mut self.world, self.physics_world, dt)
                .expect("physics step");
        }
    }

    /// Advances the simulation for `steps` substeps of duration `dt`.
    pub fn step_dt(&mut self, dt: SimDuration, steps: u32) {
        for _ in 0..steps {
            step_physics(&mut self.backend, &mut self.world, self.physics_world, dt)
                .expect("physics step");
        }
    }

    /// Spawns a dynamic body with a cuboid collider at `transform`.
    pub fn spawn_dynamic_cuboid(
        &mut self,
        name: &str,
        half_extents_m: Vec3,
        transform: Transform3,
        material: PhysicsMaterial,
        mass_kg: f64,
    ) -> Entity {
        let entity = spawn_named(&mut self.world, name);
        self.world.entity_mut(entity).insert((
            RigidBody {
                mass_kg,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid { half_extents_m },
                material,
                ..Collider::default()
            },
            transform,
        ));
        entity
    }

    /// Spawns a fixed cuboid collider body.
    pub fn spawn_fixed_cuboid(
        &mut self,
        name: &str,
        half_extents_m: Vec3,
        transform: Transform3,
        material: PhysicsMaterial,
    ) -> Entity {
        let entity = spawn_named(&mut self.world, name);
        self.world.entity_mut(entity).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid { half_extents_m },
                material,
                ..Collider::default()
            },
            transform,
        ));
        entity
    }

    /// Spawns a dynamic sphere.
    pub fn spawn_dynamic_sphere(
        &mut self,
        name: &str,
        radius_m: f64,
        transform: Transform3,
        material: PhysicsMaterial,
        mass_kg: f64,
    ) -> Entity {
        let entity = spawn_named(&mut self.world, name);
        self.world.entity_mut(entity).insert((
            RigidBody {
                mass_kg,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Sphere { radius_m },
                material,
                ..Collider::default()
            },
            transform,
        ));
        entity
    }

    /// World-space translation of an entity.
    pub fn translation(&self, entity: Entity) -> Vec3 {
        world_transform_of(&self.world, entity).translation
    }

    /// World-space linear velocity of a dynamic body.
    pub fn linear_velocity(&self, entity: Entity) -> Vec3 {
        self.world
            .get::<RigidBody>(entity)
            .expect("rigid body")
            .linear_velocity_m_s
    }
}

/// Continuous free-fall reference: `y(t) = y₀ + ½ g t²` with `g` negative.
pub fn continuous_free_fall_y(y0_m: f64, g_m_s2: f64, t_s: f64) -> f64 {
    y0_m + 0.5 * g_m_s2 * t_s * t_s
}

/// Symplectic-Euler discrete sum after `n` steps from rest: `yₙ = y₀ + g·Δt²·n(n+1)/2`.
///
/// Rapier integrates unconstrained bodies with velocity then position using the *new*
/// velocity each substep, which matches this discrete sum rather than the continuous
/// parabola `½ g t²`.
pub fn symplectic_euler_free_fall_y(y0_m: f64, g_m_s2: f64, dt_s: f64, steps: u32) -> f64 {
    let n = steps as f64;
    y0_m + g_m_s2 * dt_s * dt_s * n * (n + 1.0) / 2.0
}

/// Symplectic-Euler horizontal displacement with constant `vₓ`: `xₙ = x₀ + vₓ·Δt·n`.
pub fn symplectic_euler_projectile_x(x0_m: f64, vx_m_s: f64, dt_s: f64, steps: u32) -> f64 {
    x0_m + vx_m_s * dt_s * steps as f64
}

/// Pendulum bob height above the lowest point (pivot at origin, length `L`).
pub fn pendulum_height_above_lowest_m(bob_y_m: f64, length_m: f64) -> f64 {
    bob_y_m + length_m
}

/// Small-angle pendulum period: `T = 2π √(L/g)`.
pub fn small_angle_pendulum_period_s(length_m: f64, g_m_s2: f64) -> f64 {
    2.0 * std::f64::consts::PI * (length_m / g_m_s2).sqrt()
}

/// Sliding deceleration on a flat Coulomb surface: `a = μ g`.
pub fn friction_deceleration_m_s2(mu: f64, g_m_s2: f64) -> f64 {
    mu * g_m_s2
}

/// Stopping distance from initial speed on a flat Coulomb surface: `s = v₀² / (2 μ g)`.
pub fn friction_stopping_distance_m(v0_m_s: f64, mu: f64, g_m_s2: f64) -> f64 {
    v0_m_s * v0_m_s / (2.0 * mu * g_m_s2)
}

/// Component of gravity along an incline of angle `θ` (radians) from horizontal.
pub fn incline_gravity_along_m_s2(theta_rad: f64, g_m_s2: f64) -> f64 {
    g_m_s2 * theta_rad.sin()
}

/// Sliding acceleration when `tan θ > μ`: `a = g (sin θ − μ cos θ)`.
pub fn incline_slide_acceleration_m_s2(theta_rad: f64, mu: f64, g_m_s2: f64) -> f64 {
    g_m_s2 * (theta_rad.sin() - mu * theta_rad.cos())
}

/// Builds a simple revolute pendulum: fixed pivot + dynamic bob on a Z-axis hinge.
pub fn spawn_pendulum(
    harness: &mut PhysicsHarness,
    length_m: f64,
    initial_angle_rad: f64,
    bob_mass_kg: f64,
    bob_radius_m: f64,
) -> (Entity, Entity) {
    let pivot = spawn_named(&mut harness.world, "pivot");
    harness.world.entity_mut(pivot).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider::sphere(0.02),
        Transform3::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY),
    ));

    let sin_a = initial_angle_rad.sin();
    let cos_a = initial_angle_rad.cos();
    let bob_center = Vec3::new(length_m * sin_a, -length_m * cos_a, 0.0);

    let bob = spawn_named(&mut harness.world, "bob");
    harness.world.entity_mut(bob).insert((
        RigidBody {
            mass_kg: bob_mass_kg,
            ..RigidBody::default()
        },
        Collider::sphere(bob_radius_m),
        Transform3::from_translation_rotation(bob_center, Quat::IDENTITY),
        RevoluteJointDesc {
            parent: pivot,
            axis: Vec3::new(0.0, 0.0, 1.0),
            anchor_parent_m: Vec3::ZERO,
            anchor_child_m: Vec3::new(0.0, length_m, 0.0),
        },
    ));

    (pivot, bob)
}

/// Pendulum angle (radians) from horizontal displacement and rope length.
pub fn pendulum_angle_rad(pivot: Vec3, bob: Vec3, length_m: f64) -> f64 {
    let dx = bob.x - pivot.x;
    let _dy = bob.y - pivot.y;
    (dx / length_m).asin()
}

/// Instants where `angle(t)` crosses zero with positive slope (deterministic scan).
pub fn positive_zero_crossings(times_s: &[f64], angles_rad: &[f64]) -> Vec<f64> {
    assert_eq!(times_s.len(), angles_rad.len());
    let mut crossings = Vec::new();
    for i in 1..angles_rad.len() {
        let prev = angles_rad[i - 1];
        let curr = angles_rad[i];
        if prev <= 0.0 && curr > 0.0 {
            let frac = if (curr - prev).abs() > f64::EPSILON {
                (-prev) / (curr - prev)
            } else {
                0.5
            };
            let t = times_s[i - 1] + frac * (times_s[i] - times_s[i - 1]);
            crossings.push(t);
        }
    }
    crossings
}

/// Mean spacing of consecutive values.
pub fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<f64>() / values.len() as f64
}

/// Spawns a flat ground for tilted-gravity incline tests (`theta_rad` is encoded in `PhysicsWorldDesc`).
pub fn spawn_flat_ground(harness: &mut PhysicsHarness, material: PhysicsMaterial) -> Entity {
    harness.spawn_fixed_cuboid(
        "ground",
        Vec3::new(30.0, 0.5, 10.0),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        material,
    )
}

/// Gravity vector for an effective incline of `theta_rad` on a horizontal plane.
pub fn tilted_gravity_m_s2(theta_rad: f64) -> Vec3 {
    Vec3::new(-G * theta_rad.sin(), -G * theta_rad.cos(), 0.0)
}

/// Box on a flat surface under tilted gravity.
pub fn spawn_box_on_flat_incline(
    harness: &mut PhysicsHarness,
    half_height_m: f64,
    material: PhysicsMaterial,
) -> Entity {
    harness.spawn_dynamic_cuboid(
        "incline_box",
        Vec3::new(0.25, half_height_m, 0.25),
        Transform3::from_translation_rotation(Vec3::new(0.0, half_height_m, 0.0), Quat::IDENTITY),
        material,
        1.0,
    )
}

/// Horizontal displacement magnitude along the ramp (signed along +X).
pub fn incline_displacement_m(initial: Vec3, current: Vec3) -> f64 {
    current.x - initial.x
}
