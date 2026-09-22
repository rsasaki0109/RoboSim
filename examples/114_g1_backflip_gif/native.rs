//! Motor-only transfer probe in the native Rapier world; not a qualified flip.

use super::*;
use rne_ai::UrdfJointEffortTarget;
use rne_core::SimDuration;
use rne_physics::RevoluteJointArmature;
use rne_physics::{
    ColliderShape, CompoundCollider, JointActuation, JointMotorGainModel, RigidBody,
};
use rne_robot::{Joint, Link};
use serde_json::json;

pub(super) fn run() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let summary: serde_json::Value = serde_json::from_slice(
        &fs::read(root.join("docs/evidence/g1-contact-backflip/edu/step-125us/summary.json"))
            .expect("read source candidate"),
    )
    .expect("source summary");
    let args: Vec<_> = std::env::args().collect();
    let candidate = args
        .iter()
        .position(|arg| arg == "--native-candidate")
        .map(|i| {
            let bytes = fs::read(args.get(i + 1).expect("candidate path")).expect("candidate file");
            serde_json::from_slice::<serde_json::Value>(&bytes).expect("candidate JSON")
        });
    let p: Vec<f64> =
        serde_json::from_value(candidate.as_ref().unwrap_or(&summary)["parameters"].clone())
            .expect("candidate parameters");
    assert!(
        p.len() == 16 && p.iter().all(|v| v.is_finite()) && p[2] > 0.0 && p[5] > 0.0,
        "expected 16 finite parameters with positive phase durations"
    );
    assert!(
        p[14] == 0.0,
        "native probe currently requires zero hip-extension delay"
    );
    let recovery_s = candidate
        .as_ref()
        .and_then(|value| value.get("recovery_s"))
        .map(|value| value.as_f64().expect("numeric recovery_s"))
        .unwrap_or(0.5);
    assert!(
        recovery_s.is_finite() && recovery_s > 0.0,
        "recovery duration must be positive"
    );
    let roll_balance = candidate
        .as_ref()
        .and_then(|value| value.get("roll_balance"))
        .map(|value| value.as_bool().expect("boolean roll_balance"))
        .unwrap_or(false);
    let landing_stance_rad = candidate
        .as_ref()
        .and_then(|value| value.get("landing_stance_rad"))
        .map(|value| value.as_f64().expect("numeric landing_stance_rad"))
        .unwrap_or(0.0);
    assert!(
        landing_stance_rad.is_finite() && (0.0..=0.2).contains(&landing_stance_rad),
        "landing stance must be in 0..=0.2 rad"
    );
    let contact_friction = candidate
        .as_ref()
        .and_then(|value| value.get("contact_friction"))
        .map(|value| value.as_f64().expect("numeric contact_friction"))
        .unwrap_or(0.5);
    assert!(
        contact_friction.is_finite() && (0.0..=1.5).contains(&contact_friction),
        "contact friction must be in 0..=1.5"
    );
    let joint_armature_kg_m2 = candidate
        .as_ref()
        .and_then(|v| v.get("joint_armature_kg_m2"))
        .map(|v| v.as_f64().expect("numeric joint_armature_kg_m2"))
        .unwrap_or(0.0);
    assert!(
        joint_armature_kg_m2.is_finite() && (0.0..=1.0).contains(&joint_armature_kg_m2),
        "joint armature must be in 0..=1 kg m²"
    );
    let stop_on_fall = args.iter().any(|arg| arg == "--native-stop-on-fall");
    let dt_us: u64 = args
        .iter()
        .position(|arg| arg == "--native-dt-us")
        .map(|i| args[i + 1].parse().expect("integer step in microseconds"))
        .unwrap_or(125);
    assert!(
        [125, 250, 500, 1000].contains(&dt_us),
        "supported steps: 125, 250, 500, 1000 us"
    );
    let solver_iterations: usize = args
        .iter()
        .position(|arg| arg == "--native-solver-iterations")
        .map(|i| {
            args.get(i + 1)
                .expect("solver iteration count")
                .parse()
                .expect("integer solver iteration count")
        })
        .unwrap_or(16);
    assert!(
        [16, 32, 64].contains(&solver_iterations),
        "supported solver iterations: 16, 32, 64"
    );
    let explicit_path = args
        .iter()
        .position(|arg| arg == "--native-output")
        .map(|i| PathBuf::from(args.get(i + 1).expect("output path")));
    assert!(
        !explicit_path.as_ref().is_some_and(|p| p.exists()),
        "output must not already exist"
    );
    let velocity_servo = args.iter().any(|arg| arg == "--native-velocity-servo");
    let dt_s = dt_us as f64 * 1e-6;
    let control_steps = 2000 / dt_us;
    let implicit = args.iter().any(|arg| arg == "--native-implicit");
    assert!(
        !(implicit && velocity_servo),
        "choose only one implicit motor mode"
    );
    let declared = args.iter().any(|arg| arg == "--native-declared");
    let standing_only = args.iter().any(|arg| arg == "--native-stand");
    let custom_scene = args
        .iter()
        .position(|arg| arg == "--native-scene")
        .map(|i| PathBuf::from(args.get(i + 1).expect("native scene path")));
    assert!(
        custom_scene.is_none() || declared,
        "custom comparison scene requires --native-declared"
    );
    let scene_path = if let Some(path) = custom_scene {
        path
    } else if declared {
        root.join("assets/scenes/unitree_g1_backflip_probe.rne.scene.toml")
    } else {
        unitree_g1_dynamic_scene_path()
    };
    let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
        &scene_path,
        solver_iterations,
        SimDuration::from_ticks(dt_us * 1000),
    )
    .expect("native G1 scene");
    for name in ["ground", "left_ankle_roll_link", "right_ankle_roll_link"] {
        assert!(sim.set_named_collider_friction(name, contact_friction));
    }
    let (model, names) = build_chain(&mut sim);
    let mass_kg: f64 = (0..model.link_count())
        .map(|i| model.link_inertia(i).unwrap().mass_kg)
        .sum();
    let mut compound_part_counts: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<CompoundCollider>().map(|c| c.parts.len()))
        .collect();
    compound_part_counts.sort_unstable();
    if args.iter().any(|arg| arg == "--native-model-check") {
        assert_eq!(sim.sim_time().ticks(), 0);
        println!(
            "{}",
            json!({"mass_kg":mass_kg,"compound_part_counts":compound_part_counts,"movable_joint_count":names.len(),"scene":scene_path,"simulation_ticks":0,"qualified_backflip":false})
        );
        return;
    }
    let limits: Vec<_> = model
        .kinematic()
        .movable_joint_entities()
        .iter()
        .map(|id| sim.world().get::<Joint>(*id).expect("joint limits").limits)
        .collect();
    let actuator_entities: Vec<_> = model
        .kinematic()
        .movable_joint_entities()
        .iter()
        .map(|id| sim.world().get::<Joint>(*id).unwrap().child_link)
        .collect();
    if joint_armature_kg_m2 > 0.0 {
        for entity in &actuator_entities {
            sim.world_mut()
                .entity_mut(*entity)
                .insert(RevoluteJointArmature {
                    inertia_kg_m2: joint_armature_kg_m2,
                });
        }
    }
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
    let mut land = pose(
        -p[8] / 2.0 + p[10],
        p[8],
        (-p[8] / 2.0 - p[10]).clamp(-0.85, 0.5),
        p[15],
    );
    let mut recovered_stand = stand.clone();
    for q in [&mut land, &mut recovered_stand] {
        for (name, value) in names.iter().zip(q) {
            if name.contains("hip_roll") {
                *value = landing_stance_rad * side_sign(name);
            }
            if name.contains("ankle_roll") {
                *value = -landing_stance_rad * side_sign(name);
            }
        }
    }
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
    let mut com_velocity = Vec3::ZERO;
    let mut contact = true;
    let mut pitch_rate_rad_s = 0.0;
    let (mut roll_error, mut roll_rate_rad_s) = (0.0_f64, 0.0_f64);
    // Read-only diagnostics: retain the positive-impulse standing gate.
    let sole_spheres: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            let link = entity.get::<Link>()?;
            if !["left_ankle_roll_link", "right_ankle_roll_link"].contains(&link.name.as_str()) {
                return None;
            }
            let compound = entity.get::<CompoundCollider>()?;
            let spheres: Option<Vec<_>> = compound
                .parts
                .iter()
                .map(|part| match part.shape {
                    ColliderShape::Sphere { radius_m } => {
                        Some((part.local_offset.translation, radius_m))
                    }
                    _ => None,
                })
                .collect();
            Some((link.name.clone(), spheres?))
        })
        .collect();
    let mut tail_diagnostics = ContactDiagnostics::default();
    let mut tail_contact = true;
    let mut configured_gains = None;
    let mut min_tail_upright = 1.0_f64;
    let mut max_tail_speed = 0.0_f64;
    let mut longest_air_s = 0.0_f64;
    // One second of actual motor-driven settling, followed by the maneuver.
    // No body pose or velocity is written after scene construction.
    let mut completed_s = 0.0;
    let mut peak_speed_ratio = 0.0_f64;
    let mut peak_speed_joint = None;
    let mut peak_speed_time_s = 0.0;
    let mut max_position_excess_rad = 0.0_f64;
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
            let (next, next_kp, next_kd) = if t < 0.0 || standing_only {
                let mut q = if standing_only {
                    recovered_stand.clone()
                } else {
                    stand.clone()
                };
                if declared {
                    let correction = (previous + 0.3 * pitch_rate_rad_s).clamp(-0.4, 0.4);
                    for (name, value) in names.iter().zip(&mut q) {
                        if name.contains("ankle_pitch") {
                            *value += correction;
                        }
                    }
                }
                (q, 300.0, 10.0)
            } else if t < 0.5 {
                (mix(&stand, &crouch, smoothstep(t / 0.4)), 300.0, 10.0)
            } else if let Some(landing) = touchdown {
                let mut q = mix(&land, &recovered_stand, (t - landing) / recovery_s);
                for (name, q) in names.iter().zip(&mut q) {
                    if name.contains("ankle_pitch") {
                        let blend = ((t - landing - recovery_s) / 0.3).clamp(0.0, 1.0);
                        let correction = (1.0 - blend) * (previous + 0.3 * pitch_rate_rad_s)
                            + blend
                                * (previous
                                    + 0.6 * if declared { com_velocity.x } else { velocity.x });
                        *q = (*q + correction.clamp(-0.6, 0.6)).clamp(-0.85, 0.5);
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
            let mut next = next;
            if roll_balance && touchdown.is_some() {
                let ankle_correction = (roll_error + 0.3 * roll_rate_rad_s).clamp(-0.22, 0.22);
                for (name, value) in names.iter().zip(&mut next) {
                    if name.contains("ankle_roll") {
                        *value += ankle_correction;
                    }
                }
            }
            for (value, limit) in next.iter_mut().zip(&limits) {
                *value = value.clamp(limit.lower + 0.005, limit.upper - 0.005);
            }
            (target, kp, kd) = pending;
            pending = (next, next_kp, next_kd);
            if t > 0.5 && takeoff.is_none() && air_s > 0.012 && velocity.y > 0.3 {
                takeoff = Some(t - air_s);
            }
            if takeoff.is_some_and(|start| t - start > 0.1) && touchdown.is_none() && contact {
                touchdown = Some(t);
            }
        }
        let efforts: Vec<_> = if implicit || velocity_servo {
            Vec::new()
        } else {
            names
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
                .collect()
        };
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            if velocity_servo {
                for (i, entity) in actuator_entities.iter().enumerate() {
                    let q = sim.named_joint_position(&names[i]).unwrap();
                    let desired = velocity_target(target[i] - q, kp, kd, limits[i].max_velocity);
                    sim.world_mut().entity_mut(*entity).insert((
                        JointActuation::RevoluteVelocity {
                            target_velocity_rad_s: desired,
                            gain_nm_s_per_rad: kd,
                            max_effort_nm: torque[i],
                        },
                        JointMotorGainModel::ForceBased,
                    ));
                }
                sim.step_joint_position_actuation_targets(&[]);
            } else if implicit {
                if configured_gains != Some((kp, kd)) {
                    for (name, limit) in names.iter().zip(&torque) {
                        assert!(
                            sim.configure_named_revolute_position_actuation(name, kp, kd, *limit)
                        );
                    }
                    configured_gains = Some((kp, kd));
                }
                let targets: Vec<_> = names
                    .iter()
                    .enumerate()
                    .map(|(i, name)| rne_ai::UrdfJointPositionTarget {
                        link_name: name,
                        position: target[i],
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
        for (name, limit) in names.iter().zip(&limits) {
            let q = sim.named_joint_position(name).unwrap();
            let v = sim.named_joint_velocity(name).unwrap();
            let ratio = v.abs() / limit.max_velocity;
            if ratio > peak_speed_ratio {
                peak_speed_ratio = ratio;
                peak_speed_joint = Some(name.clone());
                peak_speed_time_s = completed_s;
            }
            max_position_excess_rad =
                max_position_excess_rad.max((q - limit.upper).max(limit.lower - q));
        }
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
        if stop_on_fall && takeoff.is_some() && t > 1.2 && base.translation.y < 0.25 {
            failure = Some(format!("Landing collapsed at maneuver time {t:.6} s"));
            break;
        }
        com_velocity = mass_weighted_velocity((0..model.link_count()).map(|index| {
            let entity = model.link_entity(index).unwrap();
            let mass = model.link_inertia(index).unwrap().mass_kg;
            // Rapier's completed-step linvel is the link COM velocity.
            let body = sim.world().get::<RigidBody>(entity).expect("physical link");
            (mass, body.linear_velocity_m_s)
        }));
        velocity = (base.translation - last_position) / dt_s;
        last_position = base.translation;
        // Use a body-frame gravity direction and angular velocity: Euler roll
        // is ill-conditioned while the sagittal flip passes through vertical.
        roll_error = body_roll_error(base.rotation);
        roll_rate_rad_s = (base.rotation.inverse()
            * sim
                .world()
                .get::<RigidBody>(model.link_entity(0).unwrap())
                .expect("base body")
                .angular_velocity_rad_s)
            .x;
        let forward = base.rotation * Vec3::X;
        let pitch = (-forward.y).atan2(forward.x);
        let pitch_delta = (pitch - previous + std::f64::consts::PI)
            .rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        angle += pitch_delta;
        pitch_rate_rad_s = pitch_delta / dt_s;
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
            let pair = ["left_ankle_roll_link", "right_ankle_roll_link"]
                .iter()
                .any(|name| sim.named_entities_in_contact(name, "ground"));
            // World-space sphere bottom, relative to y=0. This is geometry,
            // not Rapier's pre-integration contact manifold distance.
            let bottom = (sole_spheres.len() == 2).then(|| {
                sole_spheres
                    .iter()
                    .flat_map(|(name, spheres)| {
                        let pose = sim.named_transform(name).expect("sole transform");
                        spheres.iter().map(move |(center, radius)| {
                            (pose.translation + pose.rotation * *center).y - radius
                        })
                    })
                    .fold(f64::INFINITY, f64::min)
            });
            tail_diagnostics.record(contact, pair, bottom);
            tail_contact &= contact;
            min_tail_upright = min_tail_upright.min(upright);
            max_tail_speed = max_tail_speed.max(velocity.length());
        }
        if step % (10_000 / dt_us) == 0 {
            history.push(json!({"time_s":t+dt_s,"base_translation_m":base.translation.to_array(),
                "base_rotation_xyzw":base.rotation.to_array(),"pitch_rad":angle,"upright":upright,
                "joint_positions_rad":names.iter().map(|n|sim.named_joint_position(n).unwrap()).collect::<Vec<_>>(),
                "foot_contact":contact,"com_velocity_m_s":com_velocity.to_array()}));
        }
    }
    let output = json!({"backend":"RoboSim/Rapier","scene":scene_path,"compound_part_counts":compound_part_counts,"qualified_backflip":false,
        "velocity_servo":velocity_servo,"peak_joint_speed_ratio":peak_speed_ratio,"peak_speed_joint":peak_speed_joint,"peak_speed_time_s":peak_speed_time_s,"max_joint_position_excess_rad":max_position_excess_rad,
        "failure":failure,"completed_maneuver_time_s":completed_s,"implicit_position_motors":implicit,"declared_inertial_scene":declared,"standing_only":standing_only,"note":"Transfer probe: native primitive collisions, self-collision disabled; qualification pending",
        "joint_armature_kg_m2":joint_armature_kg_m2,"solver_iterations":solver_iterations,"final_second_contact_diagnostics":tail_diagnostics.report(dt_s),
        "dt_s":dt_s,"contact_friction":contact_friction,"balance_velocity_source":if declared {"whole_robot_com"} else {"base_origin"},"parameters":p,"recovery_s":recovery_s,"roll_balance":roll_balance,"landing_stance_rad":landing_stance_rad,"stop_on_fall":stop_on_fall,"mass_kg":mass_kg,"knee_limit_nm":120.0,
        "standing_passed":standing_only && failure.is_none() && completed_s>=4.999
            && min_tail_upright>0.99 && max_tail_speed<0.1 && tail_contact
            && last_position.y>0.65,"signed_rotation_rad":angle,
        "takeoff_s":takeoff,"touchdown_s":touchdown,"longest_air_s":longest_air_s,
        "final_second_continuous_foot_contact":(completed_s>=4.999).then_some(tail_contact),
        "final_second_min_upright":(completed_s>=4.999).then_some(min_tail_upright),"final_second_max_base_speed_m_s":(completed_s>=4.999).then_some(max_tail_speed),
        "joint_link_names":names,"frames":history});
    let default_path = root.join(format!(
        "target/research/g1-native-probe-{dt_us}us-{}-{}-{}.json",
        if declared { "declared" } else { "legacy" },
        if standing_only { "stand" } else { "flip" },
        if velocity_servo {
            "velocity"
        } else if implicit {
            "implicit"
        } else {
            "effort"
        }
    ));
    let path = explicit_path.as_ref().unwrap_or(&default_path);
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent).unwrap();
    }
    if explicit_path.is_some() {
        use std::io::Write;
        fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .expect("output must not already exist")
            .write_all(&serde_json::to_vec(&output).unwrap())
            .unwrap();
    } else {
        fs::write(path, serde_json::to_vec(&output).unwrap()).unwrap();
    }
    println!("native transfer probe: rotation={angle:.3} rad, takeoff={takeoff:?}, touchdown={touchdown:?}, completed maneuver time={completed_s:.6} s, failure={failure:?}; {}",path.display());
}

// This bounds the command, not the measured velocity under external impulses.
// Every measured velocity is checked separately in the rollout metrics.
fn velocity_target(error_rad: f64, kp: f64, kd: f64, rated_rad_s: f64) -> f64 {
    (kp / kd * error_rad).clamp(-0.9 * rated_rad_s, 0.9 * rated_rad_s)
}

fn body_roll_error(rotation: Quat) -> f64 {
    (rotation.inverse() * Vec3::Y).y
}

fn mass_weighted_velocity(links: impl IntoIterator<Item = (f64, Vec3)>) -> Vec3 {
    let (mass, momentum) = links
        .into_iter()
        .fold((0.0, Vec3::ZERO), |(mass, momentum), (m, v)| {
            (mass + m, momentum + m * v)
        });
    assert!(
        mass > 0.0 && mass.is_finite(),
        "positive robot mass required"
    );
    momentum / mass
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn velocity_command_is_bounded_and_retains_small_error_gain() {
        assert_eq!(velocity_target(0.1, 300.0, 10.0, 20.0), 3.0);
        assert_eq!(velocity_target(2.0, 300.0, 10.0, 20.0), 18.0);
        assert_eq!(velocity_target(-2.0, 300.0, 10.0, 20.0), -18.0);
    }
    #[test]
    fn roll_feedback_stays_bounded_through_vertical_pitch() {
        for pitch in [0.0, 1.2, std::f64::consts::FRAC_PI_2, 2.0] {
            let rotation = Quat::from_rotation_x(BASE_ROTATION_X_RAD)
                * Quat::from_rotation_y(pitch)
                * Quat::from_rotation_x(0.2);
            assert!((body_roll_error(rotation) - 0.2_f64.sin() * pitch.cos()).abs() < 1e-12);
        }
    }
    #[test]
    fn internal_motion_does_not_change_center_of_mass_velocity() {
        let internal = [(2.0, Vec3::X * 3.0), (3.0, -Vec3::X * 2.0)];
        assert!(mass_weighted_velocity(internal).length() < 1e-12);
        let translation = Vec3::new(0.4, -0.2, 0.1);
        assert!(
            (mass_weighted_velocity(
                internal.map(|(mass, velocity)| (mass, velocity + translation))
            ) - translation)
                .length()
                < 1e-12
        );
    }
}

// Counts every physics step, independently of the decimated render frames.
#[derive(Debug, Default)]
struct ContactDiagnostics {
    steps: u64,
    zero_impulse_steps: u64,
    no_pair_steps: u64,
    zero_run: u64,
    longest_zero_run: u64,
    min_bottom_m: Option<f64>,
    max_bottom_m: Option<f64>,
    max_unloaded_bottom_m: Option<f64>,
}

impl ContactDiagnostics {
    fn record(&mut self, loaded: bool, pair: bool, bottom_m: Option<f64>) {
        self.steps += 1;
        self.no_pair_steps += u64::from(!pair);
        self.zero_impulse_steps += u64::from(!loaded);
        self.zero_run = if loaded { 0 } else { self.zero_run + 1 };
        self.longest_zero_run = self.longest_zero_run.max(self.zero_run);
        if let Some(bottom) = bottom_m {
            self.min_bottom_m = Some(self.min_bottom_m.map_or(bottom, |v| v.min(bottom)));
            self.max_bottom_m = Some(self.max_bottom_m.map_or(bottom, |v| v.max(bottom)));
            if !loaded {
                self.max_unloaded_bottom_m =
                    Some(self.max_unloaded_bottom_m.map_or(bottom, |v| v.max(bottom)));
            }
        }
    }

    fn report(&self, dt_s: f64) -> serde_json::Value {
        json!({"sampled_steps":self.steps,"zero_impulse_steps":self.zero_impulse_steps,
            "no_active_ground_pair_steps":self.no_pair_steps,
            "longest_zero_impulse_duration_s":self.longest_zero_run as f64 * dt_s,
            "min_sphere_bottom_world_y_m":self.min_bottom_m,
            "max_sphere_bottom_world_y_m":self.max_bottom_m,
            "max_unloaded_sphere_bottom_world_y_m":self.max_unloaded_bottom_m})
    }
}

#[cfg(test)]
mod diagnostic_tests {
    use super::*;
    #[test]
    fn native_link_poses_close_the_authored_joint_frames() {
        use rne_physics::{FixedJointDesc, RevoluteJointDesc, RigidBodyInertia};
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let scene = std::env::var_os("RNE_BACKFLIP_KINEMATIC_SCENE")
            .map(PathBuf::from)
            .unwrap_or_else(|| root.join("assets/scenes/unitree_g1_backflip_probe.rne.scene.toml"));
        let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
            &scene,
            16,
            SimDuration::from_ticks(500_000),
        )
        .unwrap();
        for name in ["left_knee_link", "right_knee_link"] {
            assert!(sim.configure_named_revolute_effort_actuation(name, 10.0));
        }
        let mut maximum_position_error_m = 0.0_f64;
        let mut maximum_rotation_error = 0.0_f64;
        let mut maximum_rotation_norm_error = 0.0_f64;
        let mut maximum_joint_motion_rad = 0.0_f64;
        for _ in 0..100 {
            sim.step_joint_effort_actuation_targets_substeps(
                &[
                    UrdfJointEffortTarget {
                        link_name: "left_knee_link",
                        effort_nm: 5.0,
                    },
                    UrdfJointEffortTarget {
                        link_name: "right_knee_link",
                        effort_nm: 4.0,
                    },
                ],
                1,
            )
            .unwrap();
            for entity in sim.world().iter_entities() {
                let Some(link) = entity.get::<Link>() else {
                    continue;
                };
                let (parent, anchor_parent, anchor_child, relative_rotation) = if let Some(desc) =
                    entity.get::<RevoluteJointDesc>()
                {
                    let q = sim.named_joint_position(&link.name).unwrap();
                    maximum_joint_motion_rad = maximum_joint_motion_rad.max(q.abs());
                    (
                        desc.parent,
                        desc.anchor_parent_m,
                        desc.anchor_child_m,
                        desc.relative_rotation * Quat::from_axis_angle(desc.axis.normalize(), q),
                    )
                } else if let Some(desc) = entity.get::<FixedJointDesc>() {
                    (
                        desc.parent,
                        desc.anchor_parent_m,
                        desc.anchor_child_m,
                        desc.relative_rotation,
                    )
                } else {
                    continue;
                };
                let parent_name = &sim.world().get::<Link>(parent).unwrap().name;
                let parent_pose = sim.named_transform(parent_name).unwrap();
                let actual = sim.named_transform(&link.name).unwrap();
                let rotation = parent_pose.rotation * relative_rotation;
                let translation = parent_pose.translation + parent_pose.rotation * anchor_parent
                    - rotation * anchor_child;
                maximum_position_error_m =
                    maximum_position_error_m.max((translation - actual.translation).length());
                maximum_rotation_norm_error =
                    maximum_rotation_norm_error.max((actual.rotation.length_squared() - 1.0).abs());
                maximum_rotation_error = maximum_rotation_error
                    .max(1.0 - rotation.normalize().dot(actual.rotation.normalize()).abs());
            }
        }
        if let Some(path) = std::env::var_os("RNE_BACKFLIP_KINEMATIC_AUDIT") {
            use std::io::Write;
            let mut links: Vec<_> = sim.world().iter_entities().filter_map(|entity| {
                let link = entity.get::<Link>()?;
                let body = entity.get::<RigidBody>()?;
                let pose = sim.named_transform(&link.name)?;
                Some(json!({"name":link.name,"mass_kg":body.mass_kg,
                    "translation_m":pose.translation.to_array(),"rotation_xyzw":pose.rotation.to_array(),
                    "com_local_m":entity.get::<RigidBodyInertia>().map(|i|i.center_of_mass_local_m.to_array()),
                    "joint_position_rad":sim.named_joint_position(&link.name)}))
            }).collect();
            links.sort_by(|a, b| a["name"].as_str().cmp(&b["name"].as_str()));
            let report = json!({"scene":scene,"maximum_position_error_m":maximum_position_error_m,
                "maximum_rotation_error":maximum_rotation_error,"maximum_rotation_norm_error":maximum_rotation_norm_error,"maximum_joint_motion_rad":maximum_joint_motion_rad,"links":links});
            fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .unwrap()
                .write_all(&serde_json::to_vec_pretty(&report).unwrap())
                .unwrap();
        }
        assert!(
            maximum_joint_motion_rad > 0.01,
            "fixture must excite joints"
        );
        assert!(
            maximum_position_error_m < 1e-4,
            "joint-frame translation mismatch: {maximum_position_error_m}"
        );
        assert!(
            maximum_rotation_norm_error < 1e-10,
            "non-unit world rotation: {maximum_rotation_norm_error}"
        );
        assert!(
            maximum_rotation_error < 1e-6,
            "joint-frame rotation mismatch: {maximum_rotation_error}"
        );
    }

    #[test]
    fn separates_unloaded_pairs_from_missing_contacts_and_counts_runs() {
        let mut stats = ContactDiagnostics::default();
        for (loaded, pair, bottom) in [
            (true, true, -0.001),
            (false, true, 0.002),
            (false, false, 0.003),
            (true, true, -0.002),
            (false, true, 0.001),
        ] {
            stats.record(loaded, pair, Some(bottom));
        }
        let report = stats.report(0.0005);
        assert_eq!(report["sampled_steps"], 5);
        assert_eq!(report["zero_impulse_steps"], 3);
        assert_eq!(report["no_active_ground_pair_steps"], 1);
        assert_eq!(report["longest_zero_impulse_duration_s"], 0.001);
        assert_eq!(report["min_sphere_bottom_world_y_m"], -0.002);
        assert_eq!(report["max_unloaded_sphere_bottom_world_y_m"], 0.003);
        assert!(
            ContactDiagnostics::default().report(0.0005)["min_sphere_bottom_world_y_m"].is_null()
        );
    }
}
