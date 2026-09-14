//! Demonstrates the generic robot kinematics layer added to `rne_robot`:
//! forward kinematics, a geometric Jacobian, damped least-squares inverse
//! kinematics, self-collision checking, link devices, and keyframe motion.
//!
//! Run headless with `cargo run -p robot_kinematics --example 94_robot_kinematics`.

use rne_ecs::World;
use rne_math::Pose3;
use rne_robot::{
    body_motion_from_world, devices_of_link, spawn_device, BodyMotion, DeviceKind, IkOptions,
    JointKeyframe, JointTrack, KinematicModel, SelfCollisionChecker,
};
use rne_urdf_import::{parse_urdf, spawn_urdf_robot};

fn main() {
    let xml = include_str!("../../crates/rne_urdf_import/tests/fixtures/mm_minimal_arm.urdf");
    let urdf = parse_urdf(xml).expect("parse URDF");
    let mut world = World::new();
    let spawned = spawn_urdf_robot(&mut world, &urdf).expect("spawn URDF robot");

    let model = KinematicModel::from_robot(&world, spawned.robot).expect("build kinematic model");
    println!(
        "robot '{}': links={} dof={}",
        urdf.name,
        model.link_count(),
        model.dof()
    );
    println!("movable joints: {:?}", model.movable_joint_names());

    let end_link = model.link_entity(model.link_count() - 1).expect("end link");
    let configuration = vec![0.4, -0.8, 0.2, -0.2];
    let state = model
        .forward_kinematics(&configuration)
        .expect("forward kinematics");
    let end = state.link_transform(end_link).expect("end transform");
    println!(
        "FK end translation = ({:.3}, {:.3}, {:.3})",
        end.translation.x, end.translation.y, end.translation.z
    );

    let jacobian = model
        .jacobian(&configuration, end_link, rne_math::Vec3::ZERO)
        .expect("jacobian");
    println!("Jacobian shape = {}x{}", jacobian.rows(), jacobian.cols());

    let target = Pose3 {
        translation: end.translation,
        rotation: end.rotation,
    };
    let solution = model
        .inverse_kinematics(
            &target,
            end_link,
            &vec![0.0; model.dof()],
            &IkOptions::default(),
        )
        .expect("inverse kinematics");
    println!(
        "IK converged in {} iterations, residual position {:.2e} m, orientation {:.2e} rad",
        solution.iterations, solution.position_error_m, solution.orientation_error_rad
    );

    let forearm = spawned.links["forearm_link"];
    spawn_device(&mut world, forearm, "wrist_imu", DeviceKind::Sensor).expect("attach device");
    println!(
        "forearm devices = {}",
        devices_of_link(&world, forearm).len()
    );

    let mut motion = BodyMotion::new();
    let shoulder = model.movable_joint_entities()[0];
    motion
        .add_track(JointTrack::new(
            shoulder,
            "shoulder_joint",
            vec![
                JointKeyframe::new(0.0, 0.0),
                JointKeyframe::new(1.0, 0.5),
                JointKeyframe::new(2.0, 0.0),
            ],
        ))
        .expect("add track");
    for time_s in [0.0, 0.5, 1.0, 2.0] {
        let sample = motion.sample(time_s).expect("sample motion");
        println!(
            "motion t={:.1}s shoulder={:.3} rad",
            sample.time_s,
            sample.position(shoulder).unwrap_or_default()
        );
    }
    let seeded = body_motion_from_world(&world, spawned.robot, 0.0);
    println!("seeded motion tracks = {}", seeded.tracks.len());

    let checker = SelfCollisionChecker::from_robot_with_min_link_distance(&world, spawned.robot, 0)
        .expect("self-collision checker");
    let report = checker.check(&configuration).expect("self-collision check");
    println!("self-collision pairs = {}", report.len());
    for pair in report.pairs() {
        println!(
            "  link {} <-> link {}: depth {:.4} m",
            pair.link_a.index(),
            pair.link_b.index(),
            pair.depth_m
        );
    }
}
