//! Motor-only transfer probe in the native Rapier world; not a qualified flip.

use super::*;
use rne_ai::UrdfJointEffortTarget;
use rne_core::SimDuration;
use rne_robot::Joint;
use serde_json::json;

pub(super) fn run() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let summary: serde_json::Value = serde_json::from_slice(
        &fs::read(root.join("docs/evidence/g1-contact-backflip/edu/step-125us/summary.json"))
            .expect("read source candidate"),
    )
    .expect("source summary");
    let p: Vec<f64> = serde_json::from_value(summary["parameters"].clone()).unwrap();
    let args: Vec<_> = std::env::args().collect();
    let dt_us: u64 = args
        .iter()
        .position(|arg| arg == "--native-dt-us")
        .map(|i| args[i + 1].parse().expect("integer step in microseconds"))
        .unwrap_or(125);
    assert!(
        [125, 250, 500, 1000].contains(&dt_us),
        "supported steps: 125, 250, 500, 1000 us"
    );
    let dt_s = dt_us as f64 * 1e-6;
    let control_steps = 2000 / dt_us;
    let implicit = args.iter().any(|arg| arg == "--native-implicit");
    let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
        &unitree_g1_dynamic_scene_path(),
        16,
        SimDuration::from_ticks(dt_us * 1000),
    )
    .expect("native G1 scene");
    let (model, names) = build_chain(&mut sim);
    let limits: Vec<_> = model
        .kinematic()
        .movable_joint_entities()
        .iter()
        .map(|id| sim.world().get::<Joint>(*id).expect("joint limits").limits)
        .collect();
    let torque: Vec<_> = names
        .iter()
        .zip(&limits)
        .map(|(name, limit)| {
            if name.contains("knee") {
                120.0
            } else {
                limit.max_effort
            }
        })
        .collect();
    for (name, limit) in names.iter().zip(&torque) {
        assert!(sim.configure_named_revolute_effort_actuation(name, *limit));
    }
    let stand: Vec<_> = names
        .iter()
        .map(|name| {
            if name.contains("hip_pitch") || name.contains("ankle_pitch") {
                -0.18
            } else if name.contains("knee") {
                0.36
            } else if name.contains("shoulder_roll") {
                0.2 * side_sign(name)
            } else if name.contains("elbow") {
                0.42
            } else {
                0.0
            }
        })
        .collect();
    let pose = |hip, knee, ankle, shoulder| -> Vec<f64> {
        names
            .iter()
            .zip(&stand)
            .map(|(name, value)| {
                if name.contains("hip_pitch") {
                    hip
                } else if name.contains("knee") {
                    knee
                } else if name.contains("ankle_pitch") {
                    ankle
                } else if name.contains("shoulder_pitch") {
                    shoulder
                } else if name.contains("shoulder_roll") {
                    p[13] * side_sign(name)
                } else {
                    *value
                }
            })
            .collect()
    };
    let crouch = pose(
        -p[0] / 2.0 + p[12],
        p[0],
        (-p[0] / 2.0 - p[1]).clamp(-0.85, 0.5),
        -0.4,
    );
    let launch = pose(p[3], 0.02, p[4], p[9]);
    let tuck = pose(p[6], p[7], -0.4, 0.0);
    let land = pose(
        -p[8] / 2.0 + p[10],
        p[8],
        (-p[8] / 2.0 - p[10]).clamp(-0.85, 0.5),
        p[15],
    );
    let mix = |a: &[f64], b: &[f64], t: f64| -> Vec<f64> {
        a.iter()
            .zip(b)
            .map(|(a, b)| lerp(*a, *b, t.clamp(0.0, 1.0)))
            .collect()
    };
    let mut target = stand.clone();
    let mut pending = (stand.clone(), 300.0, 10.0);
    let (mut kp, mut kd) = (300.0, 10.0);
    let (mut angle, mut previous, mut air_s) = (0.0_f64, 0.0_f64, 0.0_f64);
    let (mut takeoff, mut touchdown) = (None::<f64>, None::<f64>);
    let mut history = Vec::new();
    let mut failure = None;
    let mut last_position = sim.named_transform("pelvis").unwrap().translation;
    let mut velocity = Vec3::ZERO;
    let mut contact = true;
    let mut min_tail_upright = 1.0_f64;
    let mut max_tail_speed = 0.0_f64;
    let mut longest_air_s = 0.0_f64;
    // One second of actual motor-driven settling, followed by the maneuver.
    // No body pose or velocity is written after scene construction.
    let mut completed_s = 0.0;
    for step in 0..6_000_000 / dt_us {
        let t = step as f64 * dt_s - 1.0;
        if names.iter().zip(&limits).any(|(name, limit)| {
            let q = sim.named_joint_position(name).unwrap();
            let v = sim.named_joint_velocity(name).unwrap();
            !q.is_finite() || !v.is_finite() || v.abs() > 5.0 * limit.max_velocity
        }) {
            failure = Some(format!("Joint state diverged at maneuver time {t:.6} s"));
            break;
        }
        if step % (500_000 / dt_us) == 0 {
            eprintln!(
                "native probe t={t:.3} s, base y={:.3} m, pitch={angle:.3} rad",
                last_position.y
            );
        }
        if step % control_steps == 0 {
            let (next, next_kp, next_kd) = if t < 0.0 {
                (stand.clone(), 300.0, 10.0)
            } else if t < 0.5 {
                (mix(&stand, &crouch, smoothstep(t / 0.4)), 300.0, 10.0)
            } else if let Some(landing) = touchdown {
                let mut q = mix(&land, &stand, (t - landing) / 0.5);
                for (name, q) in names.iter().zip(&mut q) {
                    if name.contains("ankle_pitch") {
                        *q =
                            (*q + (previous + 0.6 * velocity.x).clamp(-0.6, 0.6)).clamp(-0.85, 0.5);
                    }
                }
                (q, 1000.0, 20.0)
            } else if let Some(start) = takeoff {
                let elapsed = t - start;
                if elapsed < p[5] && angle > -p[11] {
                    (mix(&launch, &tuck, elapsed / 0.08), 300.0, 10.0)
                } else {
                    let blend = if angle <= -p[11] {
                        1.0
                    } else {
                        (elapsed - p[5]) / 0.1
                    };
                    (mix(&tuck, &land, blend), 1000.0, 20.0)
                }
            } else {
                (mix(&crouch, &launch, (t - 0.5) / p[2]), 300.0, 10.0)
            };
            (target, kp, kd) = pending;
            pending = (next, next_kp, next_kd);
            if t > 0.5 && takeoff.is_none() && air_s > 0.012 && velocity.y > 0.3 {
                takeoff = Some(t - air_s);
            }
            if takeoff.is_some_and(|start| t - start > 0.1) && touchdown.is_none() && contact {
                touchdown = Some(t);
            }
        }
        let efforts: Vec<_> = names
            .iter()
            .enumerate()
            .map(|(i, name)| {
                let q = sim.named_joint_position(name).expect("joint position");
                let v = sim.named_joint_velocity(name).expect("joint velocity");
                let ratio = v / limits[i].max_velocity;
                let lower = -torque[i] * ((1.0 + ratio) * 10.0).clamp(0.0, 1.0)
                    + torque[i] * ((-ratio - 1.0) * 20.0).clamp(0.0, 1.0);
                let upper = torque[i] * ((1.0 - ratio) * 10.0).clamp(0.0, 1.0)
                    - torque[i] * ((ratio - 1.0) * 20.0).clamp(0.0, 1.0);
                UrdfJointEffortTarget {
                    link_name: name,
                    effort_nm: (kp * (target[i] - q) - kd * v).clamp(lower, upper),
                }
            })
            .collect();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            if implicit {
                let targets: Vec<_> = names
                    .iter()
                    .enumerate()
                    .map(|(i, name)| {
                        assert!(sim
                            .configure_named_revolute_position_actuation(name, kp, kd, torque[i]));
                        rne_ai::UrdfJointPositionTarget {
                            link_name: name,
                            position: target[i],
                        }
                    })
                    .collect();
                sim.step_joint_position_actuation_targets(&targets);
            } else {
                sim.step_joint_effort_actuation_targets_substeps(&efforts, 1)
                    .expect("physics step");
            }
        }));
        if result.is_err() {
            failure = Some(format!("Native physics panicked at maneuver time {t:.6} s"));
            break;
        }
        completed_s = t + dt_s;
        let base = sim.named_transform("pelvis").unwrap();
        if !base.translation.is_finite()
            || !base.rotation.is_finite()
            || base.translation.y < -0.2
            || base.translation.length() > 5.0
        {
            failure = Some(format!(
                "Native base left valid scene bounds at maneuver time {t:.6} s"
            ));
            break;
        }
        velocity = (base.translation - last_position) / dt_s;
        last_position = base.translation;
        let forward = base.rotation * Vec3::X;
        let pitch = (-forward.y).atan2(forward.x);
        angle += (pitch - previous + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        previous = pitch;
        contact = ["left_ankle_roll_link", "right_ankle_roll_link"]
            .iter()
            .any(|name| sim.link_contact_impulse_ns(name) > 0.0);
        air_s = if contact { 0.0 } else { air_s + dt_s };
        if t >= 0.0 {
            longest_air_s = longest_air_s.max(air_s);
        }
        let upright = (base.rotation * Vec3::Z).y;
        if t >= 4.0 {
            min_tail_upright = min_tail_upright.min(upright);
            max_tail_speed = max_tail_speed.max(velocity.length());
        }
        if step % (10_000 / dt_us) == 0 {
            history.push(json!({"time_s":t+dt_s,"base_translation_m":base.translation.to_array(),
                "base_rotation_xyzw":base.rotation.to_array(),"pitch_rad":angle,"upright":upright,
                "joint_positions_rad":names.iter().map(|n|sim.named_joint_position(n).unwrap()).collect::<Vec<_>>(),
                "foot_contact":contact}));
        }
    }
    let output = json!({"backend":"RoboSim/Rapier","qualified_backflip":false,
        "failure":failure,"completed_maneuver_time_s":completed_s,"implicit_position_motors":implicit,"note":"Transfer probe: native primitive collisions, self-collision disabled; qualification pending",
        "dt_s":dt_s,"knee_limit_nm":120.0,"signed_rotation_rad":angle,
        "takeoff_s":takeoff,"touchdown_s":touchdown,"longest_air_s":longest_air_s,
        "final_second_min_upright":(completed_s>=4.999).then_some(min_tail_upright),"final_second_max_base_speed_m_s":(completed_s>=4.999).then_some(max_tail_speed),
        "joint_link_names":names,"frames":history});
    let path = root.join(format!(
        "target/research/g1-native-probe-{dt_us}us-{}.json",
        if implicit { "implicit" } else { "effort" }
    ));
    fs::create_dir_all(path.parent().unwrap()).unwrap();
    fs::write(&path, serde_json::to_vec(&output).unwrap()).unwrap();
    println!("native transfer probe: rotation={angle:.3} rad, takeoff={takeoff:?}, touchdown={touchdown:?}, completed maneuver time={completed_s:.6} s, failure={failure:?}; {}",path.display());
}
