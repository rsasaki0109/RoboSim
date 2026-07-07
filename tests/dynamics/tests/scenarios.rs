//! Analytic-reference dynamics validation scenarios for the Rapier backend.

use approx::assert_relative_eq;
use rne_dynamics_tests::{
    continuous_free_fall_y, friction_deceleration_m_s2, friction_stopping_distance_m,
    incline_displacement_m, incline_slide_acceleration_m_s2, mean, pendulum_angle_rad,
    pendulum_height_above_lowest_m, positive_zero_crossings, small_angle_pendulum_period_s,
    spawn_box_on_flat_incline, spawn_flat_ground, spawn_pendulum, symplectic_euler_free_fall_y,
    symplectic_euler_projectile_x, tilted_gravity_m_s2, PhysicsHarness, DEFAULT_HZ, G,
};
use rne_math::{Quat, Vec3};
use rne_physics::{PhysicsMaterial, PhysicsWorldDesc, RigidBody};
use rne_world::Transform3;

// ---------------------------------------------------------------------------
// Free fall
// ---------------------------------------------------------------------------

/// Maximum |y_sim − y_continuous| after 1 s of free fall at 60 Hz.
///
/// Rapier's unconstrained integrator tracks the continuous parabola `y₀ + ½gt²`
/// more closely than the textbook symplectic-Euler discrete sum `gΔt²n(n+1)/2`
/// (measured gap ≈ 2 cm vs ≈ 6 cm at 60 Hz over 1 s). The remaining slack covers
/// f32 gravity storage (`9.81`) and per-body pipeline rounding.
const FREE_FALL_POSITION_EPS_M: f64 = 0.03;

/// Free fall matches the continuous reference; symplectic-Euler sum is documented as looser.
#[test]
fn free_fall_matches_continuous_reference() {
    let g = -G;
    let y0 = 50.0;
    let hz = DEFAULT_HZ;
    let steps = hz as u32; // 1 s
    let dt = 1.0 / hz;

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, g, 0.0),
        ..PhysicsWorldDesc::default()
    });
    let body = harness.spawn_dynamic_cuboid(
        "falling",
        Vec3::splat(0.1),
        Transform3::from_translation_rotation(Vec3::new(0.0, y0, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: 0.0,
            restitution: 0.0,
        },
        1.0,
    );

    harness.step_hz(hz, steps);

    let y_sim = harness.translation(body).y;
    let y_continuous = continuous_free_fall_y(y0, g, 1.0);
    let y_symplectic = symplectic_euler_free_fall_y(y0, g, dt, steps);

    assert_relative_eq!(y_sim, y_continuous, epsilon = FREE_FALL_POSITION_EPS_M);
    // Characterize integrator: Rapier is closer to continuous than to the symplectic sum.
    let err_continuous = (y_sim - y_continuous).abs();
    let err_symplectic = (y_sim - y_symplectic).abs();
    assert!(
        err_continuous < err_symplectic,
        "Rapier should be nearer continuous (err={err_continuous:.4}) than symplectic (err={err_symplectic:.4})"
    );
}

// ---------------------------------------------------------------------------
// Projectile
// ---------------------------------------------------------------------------

/// Horizontal projectile displacement tolerance at 60 Hz over 1 s.
///
/// Constant horizontal velocity integration is exact to f32 storage precision.
const PROJECTILE_HORIZONTAL_EPS_M: f64 = 1e-4;

/// Vertical component uses the same continuous free-fall bound as [`FREE_FALL_POSITION_EPS_M`].
const PROJECTILE_VERTICAL_EPS_M: f64 = FREE_FALL_POSITION_EPS_M;

#[test]
fn projectile_horizontal_linear_vertical_continuous() {
    let g = -G;
    let vx = 4.0;
    let y0 = 30.0;
    let hz = DEFAULT_HZ;
    let steps = hz as u32;

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, g, 0.0),
        ..PhysicsWorldDesc::default()
    });
    let body = harness.spawn_dynamic_cuboid(
        "projectile",
        Vec3::splat(0.1),
        Transform3::from_translation_rotation(Vec3::new(0.0, y0, 0.0), Quat::IDENTITY),
        PhysicsMaterial::default(),
        1.0,
    );
    harness
        .world
        .get_mut::<RigidBody>(body)
        .unwrap()
        .linear_velocity_m_s = Vec3::new(vx, 0.0, 0.0);

    harness.step_hz(hz, steps);

    let pos = harness.translation(body);
    let x_ref = symplectic_euler_projectile_x(0.0, vx, 1.0 / hz, steps);
    let y_ref = continuous_free_fall_y(y0, g, 1.0);

    assert_relative_eq!(pos.x, x_ref, epsilon = PROJECTILE_HORIZONTAL_EPS_M);
    assert_relative_eq!(pos.y, y_ref, epsilon = PROJECTILE_VERTICAL_EPS_M);
}

// ---------------------------------------------------------------------------
// Pendulum period
// ---------------------------------------------------------------------------

/// Pendulum length for period test (m).
const PENDULUM_LENGTH_M: f64 = 2.0;

/// Small release angle (rad) ≈ 5°.
const PENDULUM_INITIAL_ANGLE_RAD: f64 = 5.0_f64.to_radians();

/// Measured period may deviate from `2π√(L/g)` due to finite amplitude (≈0.1% at 5°)
/// plus 60 Hz constraint error on the revolute joint. Allow 3% relative slack.
const PENDULUM_PERIOD_REL_EPS: f64 = 0.03;

#[test]
fn pendulum_period_near_small_angle_formula() {
    let hz = DEFAULT_HZ;
    let dt = 1.0 / hz;
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());

    let (_pivot, bob) = spawn_pendulum(
        &mut harness,
        PENDULUM_LENGTH_M,
        PENDULUM_INITIAL_ANGLE_RAD,
        1.0,
        0.1,
    );

    let mut times_s = Vec::new();
    let mut angles_rad = Vec::new();
    let total_steps = (25.0 * hz) as u32;
    for step in 0..=total_steps {
        if step > 0 {
            harness.step_hz(hz, 1);
        }
        let t = step as f64 * dt;
        let bob_pos = harness.translation(bob);
        let angle = pendulum_angle_rad(Vec3::ZERO, bob_pos, PENDULUM_LENGTH_M);
        times_s.push(t);
        angles_rad.push(angle);
    }

    let crossings = positive_zero_crossings(&times_s, &angles_rad);
    assert!(
        crossings.len() >= 4,
        "need multiple positive zero crossings for period estimate, got {}",
        crossings.len()
    );

    let periods: Vec<f64> = crossings.windows(2).map(|w| w[1] - w[0]).collect();
    let measured = mean(&periods);
    let expected = small_angle_pendulum_period_s(PENDULUM_LENGTH_M, G);

    assert_relative_eq!(measured, expected, max_relative = PENDULUM_PERIOD_REL_EPS);
}

// ---------------------------------------------------------------------------
// Friction stick / slip (tilted gravity on flat plane)
// ---------------------------------------------------------------------------

/// Coulomb friction coefficient for incline scenarios.
const INCLINE_MU: f32 = 0.4;

/// Static hold: displacement bound over 2 s when tan θ < μ (after 1 s settle).
///
/// Measured envelope on Rapier 0.22 with tilted gravity: micro-slip from contact
/// solver warm-up stays below 3 mm once settled.
const INCLINE_STATIC_MAX_DISPLACEMENT_M: f64 = 0.003;

/// Sliding acceleration tolerance when tan θ > μ.
///
/// Velocity-based measurement after 1 s avoids spawn transient; impulse friction
/// still adds ~20% bias vs `g(sin θ − μ cos θ)`.
const INCLINE_SLIDE_ACCEL_REL_EPS: f64 = 0.22;

fn incline_material() -> PhysicsMaterial {
    PhysicsMaterial {
        friction: INCLINE_MU,
        restitution: 0.0,
    }
}

#[test]
fn friction_incline_stays_static_below_mu() {
    let theta = 15.0_f64.to_radians();
    assert!(theta.tan() < INCLINE_MU as f64);

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc {
        gravity_m_s2: tilted_gravity_m_s2(theta),
        ..PhysicsWorldDesc::default()
    });
    spawn_flat_ground(&mut harness, incline_material());
    let box_body = spawn_box_on_flat_incline(&mut harness, 0.15, incline_material());

    harness.step_hz(DEFAULT_HZ, DEFAULT_HZ as u32); // 1 s settle
    let initial = harness.translation(box_body);
    harness.step_hz(DEFAULT_HZ, (2.0 * DEFAULT_HZ) as u32);

    let displacement = incline_displacement_m(initial, harness.translation(box_body)).abs();
    assert!(
        displacement < INCLINE_STATIC_MAX_DISPLACEMENT_M,
        "box should remain static on shallow effective incline, displacement={displacement:.4} m"
    );
}

#[test]
fn friction_incline_slides_above_mu() {
    let theta = 35.0_f64.to_radians();
    let mu = INCLINE_MU as f64;
    assert!(theta.tan() > mu);

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc {
        gravity_m_s2: tilted_gravity_m_s2(theta),
        ..PhysicsWorldDesc::default()
    });
    spawn_flat_ground(&mut harness, incline_material());
    let box_body = spawn_box_on_flat_incline(&mut harness, 0.15, incline_material());
    let initial = harness.translation(box_body);

    harness.step_hz(DEFAULT_HZ, (0.5 * DEFAULT_HZ) as u32); // brief settle
    let v0 = harness.linear_velocity(box_body).x;
    let measure_s = 0.5;
    harness.step_hz(DEFAULT_HZ, (measure_s * DEFAULT_HZ) as u32);
    let v1 = harness.linear_velocity(box_body).x;

    let displacement = incline_displacement_m(initial, harness.translation(box_body));
    assert!(
        displacement.abs() > 0.05,
        "box should slide on steep effective incline, displacement={displacement:.4} m"
    );

    let expected_accel = incline_slide_acceleration_m_s2(theta, mu, G);
    let measured_accel = (v1 - v0).abs() / measure_s;
    assert_relative_eq!(
        measured_accel.abs(),
        expected_accel,
        max_relative = INCLINE_SLIDE_ACCEL_REL_EPS
    );
}

// ---------------------------------------------------------------------------
// Sliding deceleration on flat plane
// ---------------------------------------------------------------------------

/// Friction coefficient for flat sliding test.
const SLIDE_MU: f32 = 0.3;

/// Initial horizontal speed (m/s).
const SLIDE_V0_M_S: f64 = 3.0;

/// Stopping-distance tolerance: contact solver + dt integration add ~10% vs `v₀²/(2μg)`.
const SLIDE_STOPPING_DISTANCE_REL_EPS: f64 = 0.12;

/// Velocity near rest threshold to declare stopped (m/s).
const SLIDE_REST_SPEED_M_S: f64 = 0.05;

#[test]
fn sliding_box_decelerates_and_stops_at_reference_distance() {
    let mu = SLIDE_MU as f64;
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    harness.spawn_fixed_cuboid(
        "ground",
        Vec3::new(30.0, 0.5, 30.0),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: SLIDE_MU,
            restitution: 0.0,
        },
    );
    let box_body = harness.spawn_dynamic_cuboid(
        "slider",
        Vec3::new(0.25, 0.15, 0.25),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.15, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: SLIDE_MU,
            restitution: 0.0,
        },
        2.0,
    );
    harness
        .world
        .get_mut::<RigidBody>(box_body)
        .unwrap()
        .linear_velocity_m_s = Vec3::new(SLIDE_V0_M_S, 0.0, 0.0);

    let initial_x = harness.translation(box_body).x;
    let expected_stop = friction_stopping_distance_m(SLIDE_V0_M_S, mu, G);

    let max_steps = (5.0 * DEFAULT_HZ) as u32;
    let mut stop_x = initial_x;
    for _ in 0..max_steps {
        harness.step_hz(DEFAULT_HZ, 1);
        let speed = harness.linear_velocity(box_body).x.abs();
        stop_x = harness.translation(box_body).x;
        if speed < SLIDE_REST_SPEED_M_S {
            break;
        }
    }

    let distance = (stop_x - initial_x).abs();
    assert_relative_eq!(
        distance,
        expected_stop,
        max_relative = SLIDE_STOPPING_DISTANCE_REL_EPS
    );

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    harness.spawn_fixed_cuboid(
        "ground",
        Vec3::new(30.0, 0.5, 30.0),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: SLIDE_MU,
            restitution: 0.0,
        },
    );
    let box_body = harness.spawn_dynamic_cuboid(
        "slider",
        Vec3::new(0.25, 0.15, 0.25),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.15, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: SLIDE_MU,
            restitution: 0.0,
        },
        2.0,
    );
    harness
        .world
        .get_mut::<RigidBody>(box_body)
        .unwrap()
        .linear_velocity_m_s = Vec3::new(SLIDE_V0_M_S, 0.0, 0.0);
    harness.step_hz(DEFAULT_HZ, (0.5 * DEFAULT_HZ) as u32);
    let vx = harness.linear_velocity(box_body).x;
    let expected_vx = SLIDE_V0_M_S - friction_deceleration_m_s2(mu, G) * 0.5;
    assert_relative_eq!(vx, expected_vx, max_relative = 0.12);
}

// ---------------------------------------------------------------------------
// Restitution
// ---------------------------------------------------------------------------

/// Coefficient of restitution for bounce test.
const RESTITUTION_E: f32 = 0.6;

/// Loose bound on apex ratio `h₂/h₁` vs `e²`. Impulse solvers exhibit 10–20% scatter
/// on restitution because of penetration correction and simultaneous contact points.
const RESTITUTION_APEX_RATIO_REL_EPS: f64 = 0.20;

#[test]
fn restitution_rebound_apex_scales_with_e_squared() {
    let drop_height = 2.0;
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    harness.spawn_fixed_cuboid(
        "ground",
        Vec3::new(10.0, 0.5, 10.0),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: 0.0,
            restitution: RESTITUTION_E,
        },
    );
    let ball = harness.spawn_dynamic_sphere(
        "ball",
        0.15,
        Transform3::from_translation_rotation(Vec3::new(0.0, drop_height, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: 0.0,
            restitution: RESTITUTION_E,
        },
        0.5,
    );

    let mut peak_after_bounce = 0.0_f64;
    let mut bounced = false;
    for _ in 0..(3.0 * DEFAULT_HZ) as u32 {
        harness.step_hz(DEFAULT_HZ, 1);
        let y = harness.translation(ball).y;
        let vy = harness.linear_velocity(ball).y;
        if bounced {
            peak_after_bounce = peak_after_bounce.max(y);
        }
        if y < 0.3 && vy > 0.1 {
            bounced = true;
        }
    }

    assert!(bounced, "ball should rebound off the plane");
    let ratio = peak_after_bounce / drop_height;
    let expected_ratio = (RESTITUTION_E as f64) * (RESTITUTION_E as f64);
    assert_relative_eq!(
        ratio,
        expected_ratio,
        max_relative = RESTITUTION_APEX_RATIO_REL_EPS
    );
}

// ---------------------------------------------------------------------------
// Energy drift
// ---------------------------------------------------------------------------

/// Conservative pendulum tracked for 10 s at 60 Hz.
const ENERGY_TRACK_DURATION_S: f64 = 10.0;

/// Maximum relative mechanical-energy drift over the tracking window.
///
/// Discrete integration plus joint constraint stabilization bleeds energy slowly;
/// 5% over 10 s is a characterization bound, not an exactness claim.
const ENERGY_DRIFT_MAX_REL: f64 = 0.05;

#[test]
fn pendulum_energy_drift_bounded_over_long_horizon() {
    let hz = DEFAULT_HZ;
    let steps = (ENERGY_TRACK_DURATION_S * hz) as u32;
    let length = PENDULUM_LENGTH_M;
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    let (_pivot, bob) = spawn_pendulum(&mut harness, length, PENDULUM_INITIAL_ANGLE_RAD, 1.0, 0.1);

    let mut energies = Vec::with_capacity(steps as usize + 1);
    for step in 0..=steps {
        if step > 0 {
            harness.step_hz(hz, 1);
        }
        let pos = harness.translation(bob);
        let vel = harness.linear_velocity(bob);
        let h = pendulum_height_above_lowest_m(pos.y, length);
        let ke = 0.5 * vel.length_squared();
        let pe = G * h;
        energies.push(ke + pe);
    }

    let e0 = energies[0];
    let max_drift = energies
        .iter()
        .map(|e| ((e - e0) / e0).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_drift < ENERGY_DRIFT_MAX_REL,
        "energy drift {max_drift:.4} exceeds bound over {ENERGY_TRACK_DURATION_S} s"
    );
}

// ---------------------------------------------------------------------------
// dt convergence
// ---------------------------------------------------------------------------

/// Coarse (60 Hz) error must exceed fine (240 Hz) error by at least this factor,
/// demonstrating O(dt) integration-order bias rather than a model mismatch.
const DT_CONVERGENCE_MIN_RATIO: f64 = 1.5;

#[test]
fn free_fall_error_shrinks_with_smaller_dt() {
    let g = -G;
    let y0 = 40.0;
    let t_s = 1.0;
    let y_continuous = continuous_free_fall_y(y0, g, t_s);

    let error_at_hz = |hz: f64| {
        let steps = (t_s * hz) as u32;
        let mut harness = PhysicsHarness::new(PhysicsWorldDesc {
            gravity_m_s2: Vec3::new(0.0, g, 0.0),
            ..PhysicsWorldDesc::default()
        });
        let body = harness.spawn_dynamic_cuboid(
            "falling",
            Vec3::splat(0.1),
            Transform3::from_translation_rotation(Vec3::new(0.0, y0, 0.0), Quat::IDENTITY),
            PhysicsMaterial {
                friction: 0.0,
                restitution: 0.0,
            },
            1.0,
        );
        harness.step_hz(hz, steps);
        (harness.translation(body).y - y_continuous).abs()
    };

    let err_60 = error_at_hz(60.0);
    let err_240 = error_at_hz(240.0);
    assert!(
        err_60 > err_240 * DT_CONVERGENCE_MIN_RATIO,
        "expected dt convergence: err_60={err_60:.4} err_240={err_240:.4}"
    );
}
