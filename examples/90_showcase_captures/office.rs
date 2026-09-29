//! Office AGV desk-place showcase source and capture.

use super::media::{
    capture_frames, push_box, push_box_material, push_cylinder, push_sphere, CameraEvidence,
    CaptureFrame, ShowcaseMetadata, SimulationEvidence, FRAME_COUNT,
};
use anyhow::{Context, Result};
use rne_ai::{
    build_visual_render_scene, office_agv_delivery_scene_path, DiffDriveAction,
    DiffDriveObservation, DiffDriveSim, OfficeAgvSharedAisleCourse,
};
use rne_ecs::{spawn_named, Entity};
use rne_math::{Quat, Vec3};
use rne_physics::{
    hash_physics_state, Collider, ColliderShape, CommandedKinematicPose, PhysicsMaterial,
    RigidBody, RigidBodyType,
};
use rne_render::{PbrMaterial, RenderScene, VisualShape};
use rne_render_wgpu::CameraOrbit;
use rne_world::Transform3;
use serde_json::to_vec_pretty;
use std::f64::consts::FRAC_PI_2;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "office";
const SUBJECT: &str = "office AGV tote pickup, shared-aisle yield and desk delivery by contact";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.40,
    pitch_rad: 0.86,
    distance_m: 5.0,
};

/// Drives the office AGV through a pickup at the dock, a yield to the
/// oncoming AGV at the single-lane section, and a delivery to the desk, with
/// the tote moved by contact at every stage, then optionally renders evenly
/// sampled post-step states with wgpu.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(false, None)?;
    let replay = rollout(false, Some(first.steps))?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "Office replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    let e = &first.evidence;
    anyhow::ensure!(
        e.loaded_offset_m < 0.05,
        "the dock pusher left the tote {:.3} m off the deck centre",
        e.loaded_offset_m
    );
    anyhow::ensure!(
        e.carry_slip_m < 0.02,
        "the tote slid {:.3} m on the deck while carried",
        e.carry_slip_m
    );
    anyhow::ensure!(e.delivered, "the tote did not end on the desk tray: {e:?}");
    anyhow::ensure!(
        e.lowest_tote_m > 0.3,
        "the tote dropped to {:.3} m",
        e.lowest_tote_m
    );
    anyhow::ensure!(
        e.yield_steps > 30,
        "the AGV did not wait for the oncoming AGV: {e:?}"
    );
    anyhow::ensure!(
        e.min_gap_m > 0.0 && !e.agv_contact,
        "the AGVs touched: min gap {:.3} m",
        e.min_gap_m
    );
    let evidence = SimulationEvidence {
        scenario: "office AGV tote transfer (examples/90_showcase_captures/office.rs)",
        steps: first.steps,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: format!(
            "tote_loaded=true; tote_delivered=true; loaded_offset_m={:.3}; carry_slip_m={:.4}; delivered_on_tray_m=[{:.3}, {:.3}]; lowest_tote_m={:.3}; yield_steps={}; min_agv_gap_m={:.3}",
            e.loaded_offset_m,
            e.carry_slip_m,
            e.delivered_at.0,
            e.delivered_at.1,
            e.lowest_tote_m,
            e.yield_steps,
            e.min_gap_m
        ),
    };
    let capture_evidence = if capture {
        let captured = rollout(true, Some(first.steps))?;
        let orbit = CameraOrbit {
            focus: Vec3::new(4.65, 0.55, 0.0),
            yaw_rad: CAMERA.yaw_rad,
            pitch_rad: CAMERA.pitch_rad,
            distance_m: CAMERA.distance_m,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &captured.frames,
            orbit,
            [0.32, 0.36, 0.42, 1.0],
            FRAME_COUNT / 2,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "The delivery AGV is the office scene's physics diff-drive robot; its deck, fences and pusher are kinematic bodies commanded to follow it. The tote is a dynamic body: the dock's pusher slides it onto the deck, friction carries it, and the AGV's pusher slides it onto the desk tray. The oncoming AGV is a kinematic body in the same world; the totes it carries are drawn only.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/office_agv_delivery.rne.scene.toml",
            "assets/robots/office_agv_delivery.rne.robot.toml",
            "crates/rne_ai/src/env/office_agv_shared_aisle.rs",
            "examples/90_showcase_captures/office.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment office",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment office",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-office.json");
        fs::write(&path, to_vec_pretty(&metadata)?)
            .with_context(|| format!("write {}", path.display()))?;
    }
    Ok(metadata)
}

/// Wheel radius of `office_agv_delivery.rne.robot.toml`.
const WHEEL_RADIUS_M: f64 = 0.1;
const CRUISE_M_S: f64 = 0.5;
const ACCEL_M_S2: f64 = 0.5;
/// Top of the AGV's physics body above its base centre (half height 0.15),
/// and the deck plate that rides a few millimetres above it.
const BODY_TOP_M: f64 = 0.15;
const DECK_HALF_M: Vec3 = Vec3::new(0.24, 0.01, 0.19);
const DECK_GAP_M: f64 = 0.005;
/// The tote: size and mass.
const TOTE_HALF_M: Vec3 = Vec3::new(0.17, 0.09, 0.14);
const TOTE_MASS_KG: f64 = 2.0;
/// Where the AGV stops for pickup and delivery.
const DOCK_X_M: f64 = 2.5;
const DESK_STOP_X_M: f64 = 6.45;
/// The dock shelf beside the lane, and the tray in front of the desk.
const DOCK_SHELF_Z_M: (f64, f64) = (0.24, 0.76);
const DESK_TRAY_X_M: (f64, f64) = (6.76, 7.1);
/// The pushers: stroke and speed.
const PUSH_M_S: f64 = 0.2;
/// The oncoming AGV sets off later than in the scenario so that it meets
/// the delivery AGV, which spends a few seconds loading, at the single-lane
/// section.
const ONCOMING_DEPARTURE_S: f64 = 9.5;

#[derive(Clone, Debug, Default)]
struct Evidence {
    loaded_offset_m: f64,
    carry_slip_m: f64,
    delivered: bool,
    delivered_at: (f64, f64),
    lowest_tote_m: f64,
    yield_steps: u64,
    min_gap_m: f64,
    agv_contact: bool,
}

struct Rollout {
    steps: u64,
    initial_digest: u64,
    final_digest: u64,
    evidence: Evidence,
    frames: Vec<CaptureFrame>,
}

/// A kinematic body placed each step.
#[derive(Clone, Copy)]
struct Body {
    entity: Entity,
}

impl Body {
    fn spawn(
        sim: &mut DiffDriveSim,
        name: &str,
        half: Vec3,
        at: Vec3,
        body_type: RigidBodyType,
        mass_kg: f64,
        friction: f64,
    ) -> Self {
        let world = sim.world_mut();
        let entity = spawn_named(world, name);
        world.entity_mut(entity).insert((
            RigidBody {
                body_type,
                mass_kg,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: half,
                },
                material: PhysicsMaterial {
                    friction: friction as f32,
                    ..PhysicsMaterial::default()
                },
                ..Collider::default()
            },
            Transform3::from_translation_rotation(at, Quat::IDENTITY),
        ));
        if body_type == RigidBodyType::Kinematic {
            world.entity_mut(entity).insert(CommandedKinematicPose);
        }
        Self { entity }
    }

    fn place(self, sim: &mut DiffDriveSim, at: Vec3, rotation: Quat) {
        if let Some(mut transform) = sim.world_mut().get_mut::<Transform3>(self.entity) {
            transform.translation = at;
            transform.rotation = rotation;
        }
    }

    fn pose(self, sim: &DiffDriveSim) -> Option<Transform3> {
        sim.world().get::<Transform3>(self.entity).copied()
    }
}

/// The bodies the transfer adds to the office scene.
struct Cell {
    deck: Body,
    side_fence: Body,
    rear_fence: Body,
    agv_pusher: Body,
    dock_pusher: Body,
    tote: Body,
    oncoming: Body,
    fixed: Vec<Body>,
}

/// Height of the deck's top face above the floor for a base at `base_y`.
fn deck_top(base_y: f64) -> f64 {
    base_y + BODY_TOP_M + DECK_GAP_M + 2.0 * DECK_HALF_M.y
}

impl Cell {
    fn spawn(sim: &mut DiffDriveSim, base_y: f64) -> Self {
        let top = deck_top(base_y);
        let kinematic = RigidBodyType::Kinematic;
        let deck = Body::spawn(
            sim,
            "agv_deck",
            DECK_HALF_M,
            Vec3::ZERO,
            kinematic,
            1.0,
            0.7,
        );
        let side_fence = Body::spawn(
            sim,
            "agv_side_fence",
            Vec3::new(DECK_HALF_M.x, 0.03, 0.01),
            Vec3::ZERO,
            kinematic,
            1.0,
            0.3,
        );
        let rear_fence = Body::spawn(
            sim,
            "agv_rear_fence",
            Vec3::new(0.01, 0.03, DECK_HALF_M.z),
            Vec3::ZERO,
            kinematic,
            1.0,
            0.3,
        );
        let agv_pusher = Body::spawn(
            sim,
            "agv_pusher",
            Vec3::new(0.01, 0.05, 0.12),
            Vec3::ZERO,
            kinematic,
            1.0,
            0.3,
        );
        // Dock shelf: a fixed top at deck height beside the lane.
        let fixed_type = RigidBodyType::Fixed;
        let (z0, z1) = DOCK_SHELF_Z_M;
        let dock_shelf = Body::spawn(
            sim,
            "dock_shelf",
            Vec3::new(0.25, 0.01, 0.5 * (z1 - z0)),
            Vec3::new(DOCK_X_M, top - 0.01, 0.5 * (z0 + z1)),
            fixed_type,
            1.0,
            0.35,
        );
        let (x0, x1) = DESK_TRAY_X_M;
        let desk_tray = Body::spawn(
            sim,
            "desk_tray",
            Vec3::new(0.5 * (x1 - x0), 0.01, 0.25),
            Vec3::new(0.5 * (x0 + x1), top - 0.01, 0.0),
            fixed_type,
            1.0,
            0.6,
        );
        let tote_z = 0.5 * (z0 + z1);
        let tote = Body::spawn(
            sim,
            "tote",
            TOTE_HALF_M,
            Vec3::new(DOCK_X_M, top + TOTE_HALF_M.y + 0.002, tote_z),
            RigidBodyType::Dynamic,
            TOTE_MASS_KG,
            0.6,
        );
        let dock_pusher = Body::spawn(
            sim,
            "dock_pusher",
            Vec3::new(0.15, 0.05, 0.01),
            Vec3::new(DOCK_X_M, top + 0.05, tote_z + TOTE_HALF_M.z + 0.02),
            kinematic,
            1.0,
            0.3,
        );
        let course = OfficeAgvSharedAisleCourse::default();
        let oncoming = Body::spawn(
            sim,
            "oncoming_agv",
            Vec3::new(course.other_half_x_m, 0.15, course.other_half_z_m),
            Vec3::new(
                course.other_start_x_m,
                base_y,
                course.other_lane_z_m(course.other_start_x_m),
            ),
            kinematic,
            40.0,
            0.5,
        );
        Self {
            deck,
            side_fence,
            rear_fence,
            agv_pusher,
            dock_pusher,
            tote,
            oncoming,
            fixed: vec![dock_shelf, desk_tray],
        }
    }

    /// Places the AGV's deck, fences and pusher (at `push` along its stroke)
    /// on the AGV at `(x, y, z, yaw)`.
    fn ride(&self, sim: &mut DiffDriveSim, agv: (f64, f64, f64, f64), push: f64) {
        let rotation = Quat::from_rotation_y(agv.3);
        let base = Vec3::new(agv.0, agv.1, agv.2);
        let top = deck_top(agv.1) - agv.1;
        let at = |local: Vec3| base + rotation * local;
        self.deck
            .place(sim, at(Vec3::new(0.0, top - DECK_HALF_M.y, 0.0)), rotation);
        self.side_fence.place(
            sim,
            at(Vec3::new(0.0, top + 0.03, -(DECK_HALF_M.z + 0.01))),
            rotation,
        );
        self.rear_fence.place(
            sim,
            at(Vec3::new(-(DECK_HALF_M.x + 0.01), top + 0.03, 0.0)),
            rotation,
        );
        let pusher_x = -0.225 + push * AGV_PUSH_STROKE_M;
        self.agv_pusher
            .place(sim, at(Vec3::new(pusher_x, top + 0.05, 0.0)), rotation);
    }

    fn entities(&self) -> Vec<Entity> {
        let mut all = vec![
            self.deck.entity,
            self.side_fence.entity,
            self.rear_fence.entity,
            self.agv_pusher.entity,
            self.dock_pusher.entity,
            self.tote.entity,
            self.oncoming.entity,
        ];
        all.extend(self.fixed.iter().map(|body| body.entity));
        all
    }
}

/// How far the AGV's pusher travels: from behind the tote to past the
/// deck's front edge, which sets the tote onto the desk tray.
const AGV_PUSH_STROKE_M: f64 = 0.6;
/// How far the dock pusher travels across the lane.
const DOCK_PUSH_STROKE_M: f64 = 0.51;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Stage {
    ToDock,
    Load { step: u64 },
    ToYield,
    Yield,
    ToDesk,
    Unload { step: u64 },
    Settle { step: u64 },
}

impl Stage {
    fn label(self) -> &'static str {
        match self {
            Stage::ToDock => "drive-to-dock",
            Stage::Load { .. } => "load-at-dock",
            Stage::ToYield => "drive-to-yield-line",
            Stage::Yield => "yield-to-oncoming",
            Stage::ToDesk => "drive-to-desk",
            Stage::Unload { .. } => "unload-at-desk",
            Stage::Settle { .. } => "delivered",
        }
    }
}

/// The speed that drives the AGV straight to `target_x`, slowing to stop on
/// it, changed from `current` by at most [`ACCEL_M_S2`] so a tote riding
/// the deck on friction alone is not thrown.
fn drive_to(current: f64, x: f64, target_x: f64, dt_s: f64) -> f64 {
    let remaining = target_x - x;
    let stopping = (2.0 * ACCEL_M_S2 * remaining.abs()).sqrt().min(CRUISE_M_S);
    let wanted = stopping.min(1.5 * remaining.abs()) * remaining.signum();
    current + (wanted - current).clamp(-ACCEL_M_S2 * dt_s, ACCEL_M_S2 * dt_s)
}

#[allow(clippy::too_many_lines)]
fn rollout(capture: bool, expected_steps: Option<u64>) -> Result<Rollout> {
    let mut sim =
        DiffDriveSim::from_scene_path(&office_agv_delivery_scene_path()).context("office scene")?;
    let first = sim.step_action(DiffDriveAction::forward(0.0));
    let cell = Cell::spawn(&mut sim, first.base_y_m);
    let dt_s = sim.fixed_delta().as_seconds().value();
    let course = OfficeAgvSharedAisleCourse::default();
    let initial_digest = hash_physics_state(sim.world());
    let push_steps = |stroke_m: f64| (stroke_m / PUSH_M_S / dt_s).ceil() as u64;
    let (dock_push, agv_push) = (
        push_steps(DOCK_PUSH_STROKE_M),
        push_steps(AGV_PUSH_STROKE_M),
    );
    let retract = 60_u64;
    let mut stage = Stage::ToDock;
    let mut observed = first;
    let mut oncoming_x = course.other_start_x_m;
    let mut evidence = Evidence {
        lowest_tote_m: f64::INFINITY,
        min_gap_m: f64::INFINITY,
        ..Evidence::default()
    };
    let mut loaded_local: Option<Vec3> = None;
    let mut frames = Vec::new();
    let mut sample_steps = Vec::new();
    if capture {
        let total = expected_steps.context("office capture needs discovered step count")?;
        sample_steps = (1..=FRAME_COUNT)
            .map(|index| ((index as u64 * total).div_ceil(FRAME_COUNT as u64)).max(1))
            .collect();
    }
    let mut step = 0_u64;
    let mut speed = 0.0_f64;
    loop {
        step += 1;
        let time_s = step as f64 * dt_s;
        // The oncoming AGV follows its lane toward -X once it has set off.
        if time_s >= ONCOMING_DEPARTURE_S {
            oncoming_x = (oncoming_x - course.other_speed_m_s * dt_s).max(course.other_clear_x_m);
        }
        let oncoming_z = course.other_lane_z_m(oncoming_x);
        cell.oncoming.place(
            &mut sim,
            Vec3::new(oncoming_x, observed.base_y_m, oncoming_z),
            Quat::from_rotation_y(oncoming_yaw(oncoming_x)),
        );

        // The delivery AGV's own script.
        let target_x = match stage {
            Stage::ToDock => Some(DOCK_X_M),
            Stage::ToYield => Some(course.shared_min_x_m - course.delivery.robot_half_x_m - 0.15),
            Stage::ToDesk => Some(DESK_STOP_X_M),
            _ => None,
        };
        speed = match target_x {
            Some(target) => drive_to(speed, observed.base_x_m, target, dt_s),
            None => 0.0,
        };
        let drive = DiffDriveAction::forward(speed / WHEEL_RADIUS_M);
        let (action, agv_push_fraction, dock_push_fraction) = match stage {
            Stage::ToDock => (drive, 0.0, 0.0),
            Stage::Load { step } => {
                let push = if step < dock_push {
                    step as f64 / dock_push as f64
                } else {
                    1.0 - ((step - dock_push) as f64 / retract as f64).min(1.0)
                };
                (DiffDriveAction::forward(0.0), 0.0, push)
            }
            Stage::ToYield => (drive, 0.0, 0.0),
            Stage::Yield => (DiffDriveAction::forward(0.0), 0.0, 0.0),
            Stage::ToDesk => (drive, 0.0, 0.0),
            Stage::Unload { step } => {
                let push = if step < agv_push {
                    step as f64 / agv_push as f64
                } else {
                    1.0 - ((step - agv_push) as f64 / retract as f64).min(1.0)
                };
                (DiffDriveAction::forward(0.0), push, 0.0)
            }
            Stage::Settle { .. } => (DiffDriveAction::forward(0.0), 0.0, 0.0),
        };
        cell.ride(
            &mut sim,
            (
                observed.base_x_m,
                observed.base_y_m,
                observed.base_z_m,
                observed.base_yaw_rad,
            ),
            agv_push_fraction,
        );
        let tote_z0 = 0.5 * (DOCK_SHELF_Z_M.0 + DOCK_SHELF_Z_M.1);
        cell.dock_pusher.place(
            &mut sim,
            Vec3::new(
                DOCK_X_M,
                deck_top(observed.base_y_m) + 0.05,
                tote_z0 + TOTE_HALF_M.z + 0.02 - dock_push_fraction * DOCK_PUSH_STROKE_M,
            ),
            Quat::IDENTITY,
        );
        observed = sim.step_action(action);

        let tote = cell.tote.pose(&sim).context("tote")?;
        evidence.lowest_tote_m = evidence.lowest_tote_m.min(tote.translation.y);
        let agv_rotation = Quat::from_rotation_y(observed.base_yaw_rad);
        let agv_base = Vec3::new(observed.base_x_m, observed.base_y_m, observed.base_z_m);
        let tote_local = agv_rotation.inverse() * (tote.translation - agv_base);
        let gap = footprint_gap(
            (observed.base_x_m, observed.base_z_m),
            (oncoming_x, oncoming_z),
            &course,
        );
        evidence.min_gap_m = evidence.min_gap_m.min(gap);
        evidence.agv_contact |= sim.last_contacts().iter().any(|contact| {
            [contact.entity_a, contact.entity_b].contains(&cell.oncoming.entity)
                && [contact.entity_a, contact.entity_b].contains(&sim.robot().base_link)
        });

        stage = match stage {
            Stage::ToDock => {
                if (observed.base_x_m - DOCK_X_M).abs() < 0.004
                    && observed.left_wheel_velocity_rad_s.abs() < 0.05
                {
                    Stage::Load { step: 0 }
                } else {
                    Stage::ToDock
                }
            }
            Stage::Load { step } => {
                if step + 1 >= dock_push + retract + 30 {
                    evidence.loaded_offset_m = tote_local.x.hypot(tote_local.z);
                    loaded_local = Some(tote_local);
                    Stage::ToYield
                } else {
                    Stage::Load { step: step + 1 }
                }
            }
            Stage::ToYield | Stage::Yield | Stage::ToDesk => {
                if let Some(start) = loaded_local {
                    evidence.carry_slip_m =
                        evidence.carry_slip_m.max((tote_local - start).length());
                }
                match stage {
                    Stage::ToYield => {
                        let stop = course.shared_min_x_m - course.delivery.robot_half_x_m - 0.15;
                        if (observed.base_x_m - stop).abs() < 0.01 {
                            Stage::Yield
                        } else {
                            Stage::ToYield
                        }
                    }
                    Stage::Yield => {
                        // Wait until the oncoming AGV has left the single
                        // lane and is past the yield line.
                        let clear = !course.other_occupies_shared(oncoming_x)
                            && oncoming_x < course.shared_min_x_m;
                        if clear {
                            Stage::ToDesk
                        } else {
                            evidence.yield_steps += 1;
                            Stage::Yield
                        }
                    }
                    _ => {
                        if (observed.base_x_m - DESK_STOP_X_M).abs() < 0.004
                            && observed.left_wheel_velocity_rad_s.abs() < 0.05
                        {
                            Stage::Unload { step: 0 }
                        } else {
                            Stage::ToDesk
                        }
                    }
                }
            }
            Stage::Unload { step } => {
                if step + 1 >= agv_push + retract {
                    Stage::Settle { step: 0 }
                } else {
                    Stage::Unload { step: step + 1 }
                }
            }
            Stage::Settle { step } => Stage::Settle { step: step + 1 },
        };

        if capture && frames.len() < sample_steps.len() && step >= sample_steps[frames.len()] {
            frames.push(CaptureFrame {
                step,
                phase: stage.label().into(),
                scene: render_scene(&sim, &cell, &observed, oncoming_x, agv_push_fraction)?,
            });
        }
        let done = matches!(stage, Stage::Settle { step } if step >= 20);
        if done || expected_steps.is_some_and(|steps| step >= steps) {
            break;
        }
        anyhow::ensure!(step < 20_000, "office rollout did not finish: {stage:?}");
    }
    let tote = cell.tote.pose(&sim).context("tote")?;
    let (x0, x1) = DESK_TRAY_X_M;
    let upright = (tote.rotation * Vec3::Y).y > 0.97;
    evidence.delivered = tote.translation.x > x0
        && tote.translation.x < x1
        && tote.translation.z.abs() < 0.2
        && (tote.translation.y - (deck_top(first.base_y_m) + TOTE_HALF_M.y)).abs() < 0.02
        && upright;
    evidence.delivered_at = (tote.translation.x, tote.translation.z);
    anyhow::ensure!(
        !capture || frames.len() == FRAME_COUNT,
        "office capture sampled {} of {} frames",
        frames.len(),
        FRAME_COUNT
    );
    Ok(Rollout {
        steps: step,
        initial_digest,
        final_digest: hash_physics_state(sim.world()),
        evidence,
        frames,
    })
}

/// Clearance between the two AGVs' footprints in the ground plane.
fn footprint_gap(ego: (f64, f64), other: (f64, f64), course: &OfficeAgvSharedAisleCourse) -> f64 {
    let dx = (ego.0 - other.0).abs() - course.delivery.robot_half_x_m - course.other_half_x_m;
    let dz = (ego.1 - other.1).abs() - 0.2 - course.other_half_z_m;
    if dx > 0.0 && dz > 0.0 {
        dx.hypot(dz)
    } else {
        dx.max(dz)
    }
}

fn render_scene(
    sim: &DiffDriveSim,
    cell: &Cell,
    observed: &DiffDriveObservation,
    oncoming_x: f64,
    agv_push: f64,
) -> Result<RenderScene> {
    let course = OfficeAgvSharedAisleCourse::default();
    let oncoming_z = course.other_lane_z_m(oncoming_x);
    let mut scene = render_office(
        sim.world(),
        (observed.base_x_m, observed.base_z_m, observed.base_yaw_rad),
        (oncoming_x, oncoming_z, oncoming_yaw(oncoming_x)),
    );
    // The transfer's bodies carry colliders but no visuals, which the scene
    // builder would draw as plain boxes; they are drawn in detail below.
    let own: Vec<Vec3> = cell
        .entities()
        .into_iter()
        .filter_map(|entity| sim.world().get::<Transform3>(entity).map(|t| t.translation))
        .collect();
    scene.items.retain(|item| {
        !own.iter()
            .any(|at| (item.transform.translation - *at).length() < 1e-9)
    });
    // The shared renderer's thin desk shelving stands where the single-lane
    // section's deep units and the far charging bay go.
    scene.items.retain(|item| {
        !matches!(
            item.shape,
            VisualShape::Box { size_m }
                if (size_m.x - 0.72).abs() < 1e-6 && (size_m.y - 1.3).abs() < 1e-6
        )
    });
    push_single_lane_section(&mut scene);
    push_charging_bays(&mut scene);
    push_totes(&mut scene, oncoming_x, oncoming_z, oncoming_yaw(oncoming_x));
    push_transfer(&mut scene, sim, cell, observed, agv_push)?;
    Ok(scene)
}

/// The deck, fences and pusher on the delivery AGV, the dock shelf with its
/// pusher, the desk tray, and the tote, all at their simulated poses.
fn push_transfer(
    scene: &mut RenderScene,
    sim: &DiffDriveSim,
    cell: &Cell,
    observed: &DiffDriveObservation,
    _agv_push: f64,
) -> Result<()> {
    const DECK: [f32; 4] = [0.18, 0.19, 0.21, 1.0];
    const FENCE: [f32; 4] = [0.95, 0.55, 0.08, 1.0];
    const STEEL: [f32; 4] = [0.55, 0.58, 0.62, 1.0];
    const TOTE: [f32; 4] = [0.18, 0.46, 0.82, 1.0];
    let draw = |scene: &mut RenderScene, body: Body, half: Vec3, color: [f32; 4]| -> Result<()> {
        let pose = body.pose(sim).context("transfer body pose")?;
        push_box_material(
            scene,
            pose.translation,
            half * 2.0,
            pose.rotation,
            color,
            PbrMaterial::new(color, 0.45, 0.35, [0.0; 3]),
        );
        Ok(())
    };
    draw(scene, cell.deck, DECK_HALF_M, DECK)?;
    draw(
        scene,
        cell.side_fence,
        Vec3::new(DECK_HALF_M.x, 0.03, 0.01),
        FENCE,
    )?;
    draw(
        scene,
        cell.rear_fence,
        Vec3::new(0.01, 0.03, DECK_HALF_M.z),
        FENCE,
    )?;
    draw(scene, cell.agv_pusher, Vec3::new(0.01, 0.05, 0.12), STEEL)?;
    draw(scene, cell.dock_pusher, Vec3::new(0.15, 0.05, 0.01), STEEL)?;
    // Scissor posts from the AGV body up to the deck.
    let rotation = Quat::from_rotation_y(observed.base_yaw_rad);
    let base = Vec3::new(observed.base_x_m, observed.base_y_m, observed.base_z_m);
    let top = deck_top(observed.base_y_m) - observed.base_y_m;
    for (x, z) in [(-0.16, -0.12), (-0.16, 0.12), (0.16, -0.12), (0.16, 0.12)] {
        push_box(
            scene,
            base + rotation * Vec3::new(x, 0.5 * (0.09 + top - 2.0 * DECK_HALF_M.y), z),
            Vec3::new(0.03, top - 2.0 * DECK_HALF_M.y - 0.09, 0.03),
            STEEL,
        );
    }
    // Dock shelf and the desk tray, each on legs.
    let deck_height = deck_top(observed.base_y_m);
    for body in &cell.fixed {
        let pose = body.pose(sim).context("fixed body pose")?;
        let half = match sim
            .world()
            .get::<Collider>(body.entity)
            .map(|c| c.shape.clone())
        {
            Some(ColliderShape::Cuboid { half_extents_m }) => half_extents_m,
            _ => continue,
        };
        push_box_material(
            scene,
            pose.translation,
            half * 2.0,
            Quat::IDENTITY,
            STEEL,
            PbrMaterial::new(STEEL, 0.35, 0.6, [0.0; 3]),
        );
        for (sx, sz) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
            push_box(
                scene,
                Vec3::new(
                    pose.translation.x + sx * (half.x - 0.02),
                    0.5 * (deck_height - 0.02),
                    pose.translation.z + sz * (half.z - 0.02),
                ),
                Vec3::new(0.03, deck_height - 0.02, 0.03),
                [0.25, 0.27, 0.3, 1.0],
            );
        }
    }
    let tote = cell.tote.pose(sim).context("tote pose")?;
    push_box_material(
        scene,
        tote.translation,
        TOTE_HALF_M * 2.0,
        tote.rotation,
        TOTE,
        PbrMaterial::new(TOTE, 0.55, 0.0, [0.0; 3]),
    );
    push_box_material(
        scene,
        tote.translation + tote.rotation * Vec3::new(0.0, TOTE_HALF_M.y + 0.005, 0.0),
        Vec3::new(2.0 * TOTE_HALF_M.x + 0.02, 0.01, 2.0 * TOTE_HALF_M.z + 0.02),
        tote.rotation,
        [0.12, 0.13, 0.15, 1.0],
        PbrMaterial::new([0.12, 0.13, 0.15, 1.0], 0.5, 0.0, [0.0; 3]),
    );
    Ok(())
}

fn push_agv(
    scene: &mut RenderScene,
    center: Vec3,
    yaw: f64,
    body: [f32; 4],
    accent: [f32; 4],
    light: [f32; 4],
) {
    let rot = Quat::from_rotation_y(yaw);
    let at = |local: Vec3| center + rot * local;
    let vertical = (rot * Quat::from_rotation_x(FRAC_PI_2)).normalize();
    let axle = (rot * Quat::from_rotation_y(FRAC_PI_2)).normalize();

    // Lower chassis deck and a slightly inset upper deck give the silhouette
    // a bevelled, rounded-looking profile instead of a single flat box.
    push_box_material(
        scene,
        at(Vec3::new(0.0, -0.08, 0.0)),
        Vec3::new(0.56, 0.16, 0.44),
        rot,
        body,
        PbrMaterial::new(body, 0.42, 0.28, [0.0; 3]),
    );
    push_box_material(
        scene,
        at(Vec3::new(0.0, 0.045, 0.0)),
        Vec3::new(0.46, 0.10, 0.36),
        rot,
        body,
        PbrMaterial::new(body, 0.36, 0.24, [0.0; 3]),
    );
    push_box_material(
        scene,
        at(Vec3::new(0.0, 0.005, 0.0)),
        Vec3::new(0.58, 0.018, 0.46),
        rot,
        accent,
        PbrMaterial::new(accent, 0.55, 0.10, [0.0; 3]),
    );
    // Lift deck carrying the cargo tote.
    push_box_material(
        scene,
        at(Vec3::new(0.0, 0.108, 0.0)),
        Vec3::new(0.40, 0.020, 0.32),
        rot,
        accent,
        PbrMaterial::new(accent, 0.30, 0.55, [0.0; 3]),
    );
    // Front bumper.
    push_box_material(
        scene,
        at(Vec3::new(0.275, -0.03, 0.0)),
        Vec3::new(0.045, 0.11, 0.40),
        rot,
        accent,
        PbrMaterial::new(accent, 0.20, 0.55, [0.0; 3]),
    );
    // Status light strips along both long edges of the upper deck.
    for side in [-1.0, 1.0] {
        push_box_material(
            scene,
            at(Vec3::new(-0.02, 0.098, side * 0.187)),
            Vec3::new(0.40, 0.014, 0.014),
            rot,
            light,
            PbrMaterial::new(
                light,
                0.15,
                0.05,
                [light[0] * 1.3, light[1] * 1.3, light[2] * 1.3],
            ),
        );
    }
    // LiDAR mast and puck.
    push_cylinder(
        scene,
        at(Vec3::new(-0.13, 0.155, 0.0)),
        0.014,
        0.13,
        vertical,
        [0.12, 0.12, 0.14, 1.0],
    );
    push_cylinder(
        scene,
        at(Vec3::new(-0.13, 0.232, 0.0)),
        0.034,
        0.030,
        vertical,
        [0.05, 0.05, 0.06, 1.0],
    );
    push_sphere(scene, at(Vec3::new(-0.13, 0.232, 0.0)), 0.009, accent);
    // Four wheels/casters with an axle aligned across the chassis width.
    for (dx, dz) in [
        (-0.205, 0.19),
        (-0.205, -0.19),
        (0.205, 0.19),
        (0.205, -0.19),
    ] {
        push_cylinder(
            scene,
            at(Vec3::new(dx, -0.146, dz)),
            0.074,
            0.055,
            axle,
            [0.045, 0.045, 0.05, 1.0],
        );
        push_sphere(
            scene,
            at(Vec3::new(dx, -0.146, dz)),
            0.014,
            [0.55, 0.57, 0.60, 1.0],
        );
    }
}

/// Adds a floor with tiled patches of alternating albedo so the surface
/// reads as carpet/tile instead of one flat slab.
fn push_floor_tiles(
    scene: &mut RenderScene,
    center: Vec3,
    tile_size: (f64, f64),
    counts: (i32, i32),
    y_m: f64,
    tones: ([f32; 4], [f32; 4]),
) {
    let (tw, td) = tile_size;
    let (cols, rows) = counts;
    let origin_x = center.x - (cols as f64) * tw / 2.0 + tw / 2.0;
    let origin_z = center.z - (rows as f64) * td / 2.0 + td / 2.0;
    for row in 0..rows {
        for col in 0..cols {
            let color = if (row + col) % 2 == 0 {
                tones.0
            } else {
                tones.1
            };
            push_box_material(
                scene,
                Vec3::new(origin_x + col as f64 * tw, y_m, origin_z + row as f64 * td),
                Vec3::new(tw * 0.97, 0.006, td * 0.97),
                Quat::IDENTITY,
                color,
                PbrMaterial::new(color, 0.72, 0.03, [0.0; 3]),
            );
        }
    }
}

/// Adds a desk with legs, a monitor, and a keyboard at a fixed footprint,
/// plus a nearby chair. Static office furniture, so world-space coordinates
/// are authored directly rather than derived from a moving observation.
fn push_desk(scene: &mut RenderScene) {
    let wood: [f32; 4] = [0.66, 0.49, 0.32, 1.0];
    let metal: [f32; 4] = [0.16, 0.17, 0.19, 1.0];
    let screen: [f32; 4] = [0.05, 0.10, 0.15, 1.0];

    push_box_material(
        scene,
        Vec3::new(7.45, 0.78, 0.0),
        Vec3::new(0.76, 0.045, 1.46),
        Quat::IDENTITY,
        wood,
        PbrMaterial::new(wood, 0.42, 0.10, [0.0; 3]),
    );
    for (dx, dz) in [(-0.33, 0.62), (-0.33, -0.62), (0.33, 0.62), (0.33, -0.62)] {
        push_cylinder(
            scene,
            Vec3::new(7.45 + dx, 0.38, dz),
            0.025,
            0.74,
            Quat::IDENTITY,
            metal,
        );
    }
    // Monitor: stand + screen with a faint "on" glow.
    push_box_material(
        scene,
        Vec3::new(7.30, 0.845, 0.0),
        Vec3::new(0.03, 0.09, 0.03),
        Quat::IDENTITY,
        metal,
        PbrMaterial::new(metal, 0.30, 0.65, [0.0; 3]),
    );
    push_box_material(
        scene,
        Vec3::new(7.26, 1.02, 0.0),
        Vec3::new(0.025, 0.30, 0.44),
        Quat::IDENTITY,
        screen,
        PbrMaterial::new(screen, 0.20, 0.10, [0.03, 0.09, 0.16]),
    );
    // Keyboard.
    push_box(
        scene,
        Vec3::new(7.55, 0.815, 0.0),
        Vec3::new(0.30, 0.02, 0.14),
        [0.20, 0.21, 0.23, 1.0],
    );
    // Chair on the far side of the desk, tucked away from the AGV aisle.
    let chair: [f32; 4] = [0.14, 0.18, 0.26, 1.0];
    push_box(
        scene,
        Vec3::new(7.45, 0.46, 0.98),
        Vec3::new(0.42, 0.05, 0.42),
        chair,
    );
    push_box(
        scene,
        Vec3::new(7.45, 0.72, 1.16),
        Vec3::new(0.42, 0.46, 0.05),
        chair,
    );
    for (dx, dz) in [(-0.18, 0.80), (-0.18, 1.16), (0.18, 0.80), (0.18, 1.16)] {
        push_cylinder(
            scene,
            Vec3::new(7.45 + dx, 0.23, dz),
            0.018,
            0.46,
            Quat::IDENTITY,
            metal,
        );
    }
}

/// Adds a shelving unit with a potted plant near the pickup dock.
fn push_shelf_and_plant(scene: &mut RenderScene) {
    let shelf: [f32; 4] = [0.42, 0.46, 0.51, 1.0];
    push_box(
        scene,
        Vec3::new(1.55, 0.90, 1.00),
        Vec3::new(0.40, 1.65, 0.32),
        shelf,
    );
    for y in [0.30, 0.70, 1.10, 1.50] {
        push_box(
            scene,
            Vec3::new(1.55, y, 1.00),
            Vec3::new(0.42, 0.025, 0.34),
            [0.30, 0.33, 0.37, 1.0],
        );
    }
    for (idx, y) in [0.42, 0.82, 1.22].into_iter().enumerate() {
        let tone = if idx % 2 == 0 {
            [0.62, 0.30, 0.10, 1.0]
        } else {
            [0.10, 0.28, 0.46, 1.0]
        };
        push_box(
            scene,
            Vec3::new(1.42, y, 1.00),
            Vec3::new(0.14, 0.18, 0.20),
            tone,
        );
    }
    // Potted plant.
    let pot: [f32; 4] = [0.58, 0.32, 0.19, 1.0];
    push_cylinder(
        scene,
        Vec3::new(1.95, 0.14, 1.02),
        0.11,
        0.28,
        Quat::IDENTITY,
        pot,
    );
    let leaf_dark: [f32; 4] = [0.10, 0.40, 0.16, 1.0];
    let leaf_light: [f32; 4] = [0.20, 0.56, 0.24, 1.0];
    push_sphere(scene, Vec3::new(1.95, 0.34, 1.02), 0.13, leaf_dark);
    push_sphere(scene, Vec3::new(1.88, 0.46, 0.97), 0.10, leaf_light);
    push_sphere(scene, Vec3::new(2.02, 0.44, 1.08), 0.10, leaf_light);
    push_sphere(scene, Vec3::new(1.96, 0.52, 1.01), 0.09, leaf_dark);
}

/// Adds a low frosted-glass partition divider off the AGV's driving lane.
///
/// The renderer used for this showcase composites opaque geometry only (no
/// alpha blending), so a translucent-looking glass panel is approximated
/// with a pale, low, glossy opaque pane rather than a true alpha value —
/// a large near-white alpha-blended pane would otherwise render as a solid
/// opaque slab and block the shot.
fn push_glass_partition(scene: &mut RenderScene) {
    let glass: [f32; 4] = [0.80, 0.92, 0.95, 1.0];
    push_box_material(
        scene,
        Vec3::new(6.35, 0.42, 1.02),
        Vec3::new(0.62, 0.62, 0.025),
        Quat::IDENTITY,
        glass,
        PbrMaterial::new(glass, 0.04, 0.05, [0.0; 3]),
    );
    let frame: [f32; 4] = [0.30, 0.32, 0.35, 1.0];
    for dx in [-0.31, 0.31] {
        push_box(
            scene,
            Vec3::new(6.35 + dx, 0.42, 1.02),
            Vec3::new(0.025, 0.64, 0.04),
            frame,
        );
    }
}

/// Adds a painted dock outline and corner markers at the pickup dock.
fn push_dock_markings(scene: &mut RenderScene) {
    let paint: [f32; 4] = [0.96, 0.72, 0.10, 1.0];
    for x in [2.10, 2.90] {
        push_box(
            scene,
            Vec3::new(x, 0.029, 0.0),
            Vec3::new(0.03, 0.006, 1.05),
            paint,
        );
    }
    for z in [-0.52, 0.52] {
        push_box(
            scene,
            Vec3::new(2.5, 0.029, z),
            Vec3::new(0.83, 0.006, 0.03),
            paint,
        );
    }
}

/// Heading of the oncoming AGV at `x`: it drives toward -X along its lane,
/// which bends onto the centre line through the single-lane section, so it
/// faces along the lane's slope there. (A fixed heading of PI had it sliding
/// sideways through the bend.)
fn oncoming_yaw(x_m: f64) -> f64 {
    let course = OfficeAgvSharedAisleCourse::default();
    let step = 0.05;
    let dx = -2.0 * step;
    let dz = course.other_lane_z_m(x_m - step) - course.other_lane_z_m(x_m + step);
    // Yaw about +Y turns the AGV's +X toward (cos yaw, -sin yaw) in (x, z).
    (-dz).atan2(dx)
}

/// The single-lane section the scenario enforces (x 3.5 to 5.5 m), drawn as
/// deep shelving units that narrow the far side of the aisle to one lane,
/// with hatched floor at both ends. They are drawn only: the scenario's rule,
/// not a collider, is what makes the oncoming AGV take the centre line there
/// and the delivery AGV wait at the yield line.
fn push_single_lane_section(scene: &mut RenderScene) {
    let course = OfficeAgvSharedAisleCourse::default();
    let (x0, x1) = (course.shared_min_x_m, course.shared_max_x_m);
    let depth_m = 0.62;
    let back_z = -1.04;
    let carcass = [0.30, 0.36, 0.44, 1.0];
    let units = 3;
    let unit_m = (x1 - x0) / f64::from(units);
    for unit in 0..units {
        let x = x0 + (f64::from(unit) + 0.5) * unit_m;
        push_box(
            scene,
            Vec3::new(x, 0.75, back_z + 0.5 * depth_m),
            Vec3::new(unit_m - 0.04, 1.5, depth_m),
            carcass,
        );
        for shelf in 0..4 {
            let y = 0.18 + 0.36 * f64::from(shelf);
            push_box(
                scene,
                Vec3::new(x, y, back_z + depth_m + 0.005),
                Vec3::new(unit_m - 0.1, 0.03, 0.01),
                [0.92, 0.62, 0.14, 1.0],
            );
            // Archive boxes on each shelf, facing the aisle.
            for slot in 0..3 {
                let bx = x - 0.5 * unit_m + 0.2 + f64::from(slot) * (unit_m - 0.4) / 2.0;
                push_box(
                    scene,
                    Vec3::new(bx, y + 0.13, back_z + depth_m - 0.12),
                    Vec3::new(0.24, 0.22, 0.2),
                    if (unit + shelf + slot) % 3 == 0 {
                        [0.85, 0.82, 0.74, 1.0]
                    } else {
                        [0.62, 0.48, 0.32, 1.0]
                    },
                );
            }
        }
    }
    // Hatched floor marking both ends of the section.
    for x in [x0, x1] {
        for stripe in 0..5 {
            push_box(
                scene,
                Vec3::new(x + 0.06 * f64::from(stripe) - 0.12, 0.031, -0.1),
                Vec3::new(0.03, 0.004, 1.8),
                if stripe % 2 == 0 {
                    [0.96, 0.78, 0.10, 1.0]
                } else {
                    [0.12, 0.12, 0.13, 1.0]
                },
            );
        }
    }
}

/// Charging bays at both ends of the oncoming AGV's run: it charges at the
/// far bay until it departs and parks at the near one once clear.
fn push_charging_bays(scene: &mut RenderScene) {
    let course = OfficeAgvSharedAisleCourse::default();
    for x in [course.other_start_x_m, course.other_clear_x_m] {
        let z = course.other_lane_z_m(x);
        push_box(
            scene,
            Vec3::new(x, 0.03, z),
            Vec3::new(0.72, 0.006, 0.56),
            [0.10, 0.45, 0.85, 1.0],
        );
        push_box(
            scene,
            Vec3::new(x, 0.032, z),
            Vec3::new(0.62, 0.006, 0.46),
            [0.16, 0.18, 0.22, 1.0],
        );
        push_box_material(
            scene,
            Vec3::new(x, 0.25, -1.02),
            Vec3::new(0.3, 0.5, 0.08),
            Quat::IDENTITY,
            [0.85, 0.87, 0.9, 1.0],
            PbrMaterial::new([0.85, 0.87, 0.9, 1.0], 0.35, 0.2, [0.0; 3]),
        );
        push_box_material(
            scene,
            Vec3::new(x, 0.4, -0.975),
            Vec3::new(0.12, 0.05, 0.01),
            Quat::IDENTITY,
            [0.2, 0.95, 0.45, 1.0],
            PbrMaterial::new([0.2, 0.95, 0.45, 1.0], 0.3, 0.0, [0.2, 0.95, 0.45]),
        );
    }
}

/// A stack of two lidded totes on the oncoming AGV's deck.
fn push_totes(scene: &mut RenderScene, x_m: f64, z_m: f64, yaw: f64) {
    let rotation = Quat::from_rotation_y(yaw);
    for (level, color) in [
        (0.0, [0.12, 0.42, 0.78, 1.0]),
        (1.0, [0.95, 0.60, 0.10, 1.0]),
    ] {
        let centre = Vec3::new(x_m, 0.42 + 0.2 * level, z_m);
        push_box_material(
            scene,
            centre,
            Vec3::new(0.4, 0.18, 0.3),
            rotation,
            color,
            PbrMaterial::new(color, 0.55, 0.0, [0.0; 3]),
        );
        push_box_material(
            scene,
            centre + Vec3::new(0.0, 0.1, 0.0),
            Vec3::new(0.42, 0.02, 0.32),
            rotation,
            [0.2, 0.2, 0.22, 1.0],
            PbrMaterial::new([0.2, 0.2, 0.22, 1.0], 0.5, 0.0, [0.0; 3]),
        );
    }
}

/// Renders the office corridor with the ego AGV and the second AGV at the given
/// `(x, z, yaw)` poses. Both vehicles are drawn from the poses the caller
/// passes, so any rollout driving the office scene can reuse the set dressing.
pub(crate) fn render_office(
    world: &rne_ecs::World,
    ego: (f64, f64, f64),
    other: (f64, f64, f64),
) -> RenderScene {
    let mut scene = build_visual_render_scene(world);
    // The authored walls and the flat collision-box desk are collision
    // boundaries, but a low eye-level showcase camera would hide every
    // actor behind them, and a bare box desk reads poorly. Drop both render
    // items and rebuild richer render-only geometry from the same footprint.
    scene.items.retain(|item| {
        let is_wall = matches!(
            item.shape,
            VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z < 0.2
        );
        let is_flat_desk = matches!(
            item.shape,
            VisualShape::Box { size_m }
                if (size_m.x - 0.7).abs() < 0.01
                    && (size_m.y - 0.8).abs() < 0.01
                    && (size_m.z - 1.4).abs() < 0.01
        );
        !(is_wall || is_flat_desk)
    });
    // Extend the authored corridor floor toward the close camera. The source
    // scene deliberately stops at the south wall; the extension keeps the
    // lower half of the poster an office floor instead of a background void.
    push_box(
        &mut scene,
        Vec3::new(4.65, -0.035, 2.25),
        Vec3::new(6.0, 0.05, 4.5),
        [0.80, 0.78, 0.73, 1.0],
    );
    push_floor_tiles(
        &mut scene,
        Vec3::new(4.65, 0.022, 0.0),
        (0.9, 0.95),
        (7, 2),
        0.022,
        ([0.84, 0.82, 0.77, 1.0], [0.77, 0.75, 0.70, 1.0]),
    );
    push_floor_tiles(
        &mut scene,
        Vec3::new(4.65, -0.006, 2.1),
        (1.1, 1.05),
        (6, 3),
        -0.006,
        ([0.83, 0.81, 0.76, 1.0], [0.76, 0.74, 0.69, 1.0]),
    );
    push_agv(
        &mut scene,
        Vec3::new(ego.0, 0.24, ego.1),
        ego.2,
        [0.94, 0.33, 0.07, 1.0],
        [0.14, 0.15, 0.17, 1.0],
        [1.0, 0.55, 0.06, 1.0],
    );
    push_agv(
        &mut scene,
        Vec3::new(other.0, 0.24, other.1),
        other.2,
        [0.10, 0.32, 0.72, 1.0],
        [0.13, 0.14, 0.16, 1.0],
        [0.10, 0.55, 0.92, 1.0],
    );
    // Desk shelving, aisle dividers, and a destination halo make the task
    // legible from the single fixed camera without adding physics entities.
    for x_m in [3.4, 4.7, 5.8] {
        push_box(
            &mut scene,
            Vec3::new(x_m, 0.65, -0.92),
            Vec3::new(0.72, 1.3, 0.10),
            [0.32, 0.38, 0.45, 1.0],
        );
    }
    // Far-side office wall and ceiling fixtures remove the empty sky band in
    // the poster while keeping the driving aisle open to the camera.
    push_box(
        &mut scene,
        Vec3::new(4.6, 1.80, -1.08),
        Vec3::new(5.9, 3.60, 0.08),
        [0.38, 0.44, 0.52, 1.0],
    );
    for x_m in [2.9, 4.4, 5.9, 7.2] {
        push_box_material(
            &mut scene,
            Vec3::new(x_m, 1.58, -1.02),
            Vec3::new(0.72, 0.10, 0.04),
            Quat::IDENTITY,
            [0.96, 0.98, 1.0, 1.0],
            PbrMaterial::new([0.96, 0.98, 1.0, 1.0], 0.30, 0.02, [0.55, 0.57, 0.60]),
        );
        push_box(
            &mut scene,
            Vec3::new(x_m, 1.28, -1.035),
            Vec3::new(0.56, 0.34, 0.025),
            [0.09, 0.36, 0.50, 1.0],
        );
    }
    push_dock_markings(&mut scene);
    // Yield line at the shared aisle and a desk-top/monitor silhouette at the
    // destination make the mission semantics readable without text labels.
    push_box(
        &mut scene,
        Vec3::new(3.45, 0.035, 0.0),
        Vec3::new(0.06, 0.025, 1.60),
        [0.96, 0.70, 0.08, 1.0],
    );
    push_desk(&mut scene);
    push_shelf_and_plant(&mut scene);
    push_glass_partition(&mut scene);
    push_box_material(
        &mut scene,
        Vec3::new(6.5, 0.08, 0.0),
        Vec3::new(0.75, 0.03, 0.75),
        Quat::IDENTITY,
        [0.10, 0.82, 0.42, 0.92],
        PbrMaterial::new([0.10, 0.82, 0.42, 0.92], 0.42, 0.10, [0.0; 3]),
    );
    scene
}

/// A doorway in the far wall at `x_m`, for the navigation showcase's
/// pedestrian: a dark opening, a frame and an exit sign. The shelving and
/// window on that stretch of wall are removed so the doorway is not blocked.
pub(crate) fn push_doorway(scene: &mut RenderScene, x_m: f64) {
    scene.items.retain(|item| {
        // Shelving, windows and light fittings on that stretch of wall; the
        // wall itself is wider than a meter and stays.
        // Box items carry their extent in the transform's scale; `size_m` is
        // the unit box.
        let small_box = matches!(
            item.shape,
            VisualShape::Box { size_m } if size_m.x * item.transform.scale.x < 1.0
        );
        let at = item.transform.translation;
        !(small_box && (at.x - x_m).abs() < 0.5 && at.z < -0.85 && at.y > 0.3)
    });
    const FRAME: [f32; 4] = [0.78, 0.80, 0.84, 1.0];
    push_box(
        scene,
        Vec3::new(x_m, 1.0, -1.035),
        Vec3::new(0.82, 2.0, 0.01),
        [0.05, 0.06, 0.08, 1.0],
    );
    for dx in [-0.45, 0.45] {
        push_box(
            scene,
            Vec3::new(x_m + dx, 1.02, -1.03),
            Vec3::new(0.08, 2.04, 0.05),
            FRAME,
        );
    }
    push_box(
        scene,
        Vec3::new(x_m, 2.06, -1.03),
        Vec3::new(0.98, 0.08, 0.05),
        FRAME,
    );
    push_box_material(
        scene,
        Vec3::new(x_m, 2.24, -1.03),
        Vec3::new(0.34, 0.14, 0.03),
        Quat::IDENTITY,
        [0.10, 0.80, 0.35, 1.0],
        PbrMaterial::new([0.10, 0.80, 0.35, 1.0], 0.3, 0.0, [0.12, 0.95, 0.40]),
    );
}
