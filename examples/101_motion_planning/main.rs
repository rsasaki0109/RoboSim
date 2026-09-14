//! Demonstrates the MoveIt-inspired native motion planning stack in
//! `rne_planning`: a planning scene, joint interpolation and RRT-Connect
//! planners, a planner pipeline, and robot-vs-world distance queries.
//!
//! Run headless with `cargo run -p motion_planning --example 101_motion_planning`.

use rne_ecs::World;
use rne_math::Vec3;
use rne_planning::{
    GoalConstraint, JointInterpolationPlanner, MotionPlanRequest, MotionPlanner, PlanningOptions,
    PlanningPipeline, PlanningScene,
};
use rne_robot::{CollisionPrimitive, CollisionWorld, CollisionWorldObject};
use rne_urdf_import::{parse_urdf, spawn_urdf_robot};

fn main() {
    let xml = include_str!("../../crates/rne_urdf_import/tests/fixtures/mm_minimal_arm.urdf");
    let urdf = parse_urdf(xml).expect("parse URDF");
    let mut world = World::new();
    let spawned = spawn_urdf_robot(&mut world, &urdf).expect("spawn URDF robot");

    let scene = PlanningScene::from_world(&world, spawned.robot).expect("planning scene");
    let dof = scene.model().dof();
    println!("dof = {dof}");

    let pipeline = PlanningPipeline::with_builtins();
    println!("planners = {:?}", pipeline.planner_names());

    let start = vec![0.0; dof];
    let goal = vec![0.2, -0.3, 0.1, 0.0];

    let request = MotionPlanRequest::new(
        start.clone(),
        GoalConstraint::joint(goal.clone()),
        PlanningOptions {
            waypoint_count: 5,
            ..PlanningOptions::default()
        },
    );
    let interpolation = JointInterpolationPlanner::new()
        .plan(&scene, &request)
        .expect("joint interpolation plan");
    println!(
        "joint_interpolation: {} points, duration {:.2} s",
        interpolation.trajectory.len(),
        interpolation.trajectory.duration_s()
    );

    let options = PlanningOptions {
        seed: 42,
        step_size: 0.3,
        collision_check_steps: 8,
        acceleration_limits: vec![2.0; dof],
        ..PlanningOptions::default()
    };
    let request = MotionPlanRequest::new(start.clone(), GoalConstraint::joint(goal), options);
    let pipeline = PlanningPipeline::with_builtins();
    println!("pipeline adapters = {:?}", pipeline.adapter_names());
    let planned = pipeline.plan(&scene, &request).expect("pipeline plan");
    println!(
        "pipeline({}): {} iterations, {} waypoints, duration {:.2} s",
        planned.planner,
        planned.iterations,
        planned.trajectory.len(),
        planned.trajectory.duration_s()
    );

    let collision_world = CollisionWorld::with_objects(vec![CollisionWorldObject::new(
        CollisionPrimitive::Sphere {
            center_m: Vec3::new(0.5, 0.5, 0.0),
            radius_m: 0.05,
        },
    )]);
    let scene = PlanningScene::from_world(&world, spawned.robot)
        .expect("scene")
        .with_collision_world(collision_world);
    let distance = scene
        .collision_world()
        .distance(scene.self_checker(), &start)
        .expect("distance query");
    println!(
        "min robot-obstacle distance = {:.3} m",
        distance.min_distance_m().unwrap_or(f64::INFINITY)
    );
}
