//! Prints measured error magnitudes for the architecture doc (`cargo test report -- --nocapture`).

use rne_dynamics_tests::{
    continuous_free_fall_y, friction_stopping_distance_m, incline_slide_acceleration_m_s2, mean,
    pendulum_angle_rad, pendulum_height_above_lowest_m, positive_zero_crossings,
    small_angle_pendulum_period_s, spawn_box_on_flat_incline, spawn_flat_ground, spawn_pendulum,
    symplectic_euler_free_fall_y, tilted_gravity_m_s2, PhysicsHarness, G,
};
use rne_math::{Quat, Vec3};
use rne_physics::{PhysicsMaterial, PhysicsWorldDesc, RigidBody};
use rne_world::Transform3;

#[test]
fn report_measured_errors() {
    let g = -G;
    let y0 = 50.0;
    let hz = 60.0;
    let steps = 60u32;
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
    let y_cont = continuous_free_fall_y(y0, g, 1.0);
    let y_sym = symplectic_euler_free_fall_y(y0, g, dt, steps);
    eprintln!(
        "free_fall_1s: y_sim={y_sim:.6} err_vs_continuous={:.6} err_vs_symplectic={:.6}",
        (y_sim - y_cont).abs(),
        (y_sim - y_sym).abs()
    );

    let err = |hz: f64| {
        let mut h = PhysicsHarness::new(PhysicsWorldDesc {
            gravity_m_s2: Vec3::new(0.0, g, 0.0),
            ..PhysicsWorldDesc::default()
        });
        let b = h.spawn_dynamic_cuboid(
            "f",
            Vec3::splat(0.1),
            Transform3::from_translation_rotation(Vec3::new(0.0, 40.0, 0.0), Quat::IDENTITY),
            PhysicsMaterial {
                friction: 0.0,
                restitution: 0.0,
            },
            1.0,
        );
        h.step_hz(hz, hz as u32);
        (h.translation(b).y - continuous_free_fall_y(40.0, g, 1.0)).abs()
    };
    eprintln!(
        "dt_convergence: err_60={:.6} err_240={:.6}",
        err(60.0),
        err(240.0)
    );

    let length = 2.0;
    let theta0 = 5.0_f64.to_radians();
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    let (_pivot, bob) = spawn_pendulum(&mut harness, length, theta0, 1.0, 0.1);
    let mut times = Vec::new();
    let mut angles = Vec::new();
    for step in 0..=(25.0 * hz) as u32 {
        if step > 0 {
            harness.step_hz(hz, 1);
        }
        times.push(step as f64 / hz);
        angles.push(pendulum_angle_rad(
            Vec3::ZERO,
            harness.translation(bob),
            length,
        ));
    }
    let periods: Vec<f64> = positive_zero_crossings(&times, &angles)
        .windows(2)
        .map(|w| w[1] - w[0])
        .collect();
    let t_meas = mean(&periods);
    let t_exp = small_angle_pendulum_period_s(length, G);
    eprintln!(
        "pendulum_period: measured={t_meas:.4} expected={t_exp:.4} rel_err={:.4}%",
        ((t_meas - t_exp) / t_exp * 100.0).abs()
    );

    let mu = 0.4f64;
    let theta_static = 15.0_f64.to_radians();
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc {
        gravity_m_s2: tilted_gravity_m_s2(theta_static),
        ..PhysicsWorldDesc::default()
    });
    spawn_flat_ground(
        &mut harness,
        PhysicsMaterial {
            friction: 0.4,
            restitution: 0.0,
        },
    );
    let box_body = spawn_box_on_flat_incline(
        &mut harness,
        0.15,
        PhysicsMaterial {
            friction: 0.4,
            restitution: 0.0,
        },
    );
    harness.step_hz(hz, hz as u32);
    let x0 = harness.translation(box_body).x;
    harness.step_hz(hz, (2.0 * hz) as u32);
    let static_disp = (harness.translation(box_body).x - x0).abs();
    eprintln!("incline_static_15deg_2s: displacement={static_disp:.6} m");

    let theta_slide = 35.0_f64.to_radians();
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc {
        gravity_m_s2: tilted_gravity_m_s2(theta_slide),
        ..PhysicsWorldDesc::default()
    });
    spawn_flat_ground(
        &mut harness,
        PhysicsMaterial {
            friction: 0.4,
            restitution: 0.0,
        },
    );
    let box_body = spawn_box_on_flat_incline(
        &mut harness,
        0.15,
        PhysicsMaterial {
            friction: 0.4,
            restitution: 0.0,
        },
    );
    harness.step_hz(hz, 30);
    let v0 = harness.linear_velocity(box_body).x;
    harness.step_hz(hz, 30);
    let v1 = harness.linear_velocity(box_body).x;
    let a_meas = (v1 - v0).abs() / 0.5;
    let a_exp = incline_slide_acceleration_m_s2(theta_slide, mu, G);
    eprintln!(
        "incline_slide_35deg: a_meas={a_meas:.4} a_exp={a_exp:.4} rel_err={:.2}%",
        ((a_meas - a_exp) / a_exp * 100.0).abs()
    );

    let slide_mu = 0.3;
    let v0_slide = 3.0;
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    harness.spawn_fixed_cuboid(
        "ground",
        Vec3::new(30.0, 0.5, 30.0),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: slide_mu,
            restitution: 0.0,
        },
    );
    let slider = harness.spawn_dynamic_cuboid(
        "s",
        Vec3::new(0.25, 0.15, 0.25),
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.15, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: slide_mu,
            restitution: 0.0,
        },
        2.0,
    );
    harness
        .world
        .get_mut::<RigidBody>(slider)
        .unwrap()
        .linear_velocity_m_s = Vec3::new(v0_slide, 0.0, 0.0);
    let x_start = harness.translation(slider).x;
    for _ in 0..300 {
        harness.step_hz(hz, 1);
        if harness.linear_velocity(slider).x.abs() < 0.05 {
            break;
        }
    }
    let stop_dist = (harness.translation(slider).x - x_start).abs();
    let stop_exp = friction_stopping_distance_m(v0_slide, slide_mu as f64, G);
    eprintln!(
        "sliding_stop: dist={stop_dist:.4} expected={stop_exp:.4} rel_err={:.2}%",
        ((stop_dist - stop_exp) / stop_exp * 100.0).abs()
    );

    let drop_h = 2.0;
    let e = 0.6f32;
    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    harness.spawn_fixed_cuboid(
        "ground",
        Vec3::new(10.0, 0.5, 10.0),
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: 0.0,
            restitution: e,
        },
    );
    let ball = harness.spawn_dynamic_sphere(
        "ball",
        0.15,
        Transform3::from_translation_rotation(Vec3::new(0.0, drop_h, 0.0), Quat::IDENTITY),
        PhysicsMaterial {
            friction: 0.0,
            restitution: e,
        },
        0.5,
    );
    let mut peak = 0.0_f64;
    let mut bounced = false;
    for _ in 0..180 {
        harness.step_hz(hz, 1);
        let y = harness.translation(ball).y;
        let vy = harness.linear_velocity(ball).y;
        if bounced {
            peak = peak.max(y);
        }
        if y < 0.3 && vy > 0.1 {
            bounced = true;
        }
    }
    let ratio = peak / drop_h;
    eprintln!(
        "restitution: apex_ratio={ratio:.4} expected_e2={:.4}",
        (e as f64) * (e as f64)
    );

    let mut harness = PhysicsHarness::new(PhysicsWorldDesc::default());
    let (_pivot, bob) = spawn_pendulum(&mut harness, length, theta0, 1.0, 0.1);
    let steps = (10.0 * hz) as u32;
    let mut energies = Vec::new();
    for step in 0..=steps {
        if step > 0 {
            harness.step_hz(hz, 1);
        }
        let pos = harness.translation(bob);
        let vel = harness.linear_velocity(bob);
        let h = pendulum_height_above_lowest_m(pos.y, length);
        energies.push(0.5 * vel.length_squared() + G * h);
    }
    let e0 = energies[0];
    let max_drift = energies
        .iter()
        .map(|e| ((e - e0) / e0).abs())
        .fold(0.0_f64, f64::max);
    eprintln!("energy_drift_10s: max_rel={max_drift:.4}");
}
