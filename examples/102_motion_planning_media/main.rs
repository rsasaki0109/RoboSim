//! Emits a JSON description of `rne_planning` plans for a planar 2R arm in
//! front of a workspace obstacle. `tools/generate_motion_planning_media.py`
//! renders the README figure and animation from this output.
//!
//! Run headless with
//! `cargo run -p motion_planning_media --example 102_motion_planning_media`.

use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Vec3};
use rne_planning::{
    BitStarPlanner, GoalConstraint, HybridPlanner, InformedRrtStarPlanner,
    JointInterpolationPlanner, MotionPlanRequest, MotionPlanner, PlanningOptions, PlanningScene,
    PrmPlanner, RrtConnectPlanner, RrtStarPlanner, StompPlanner,
};
use rne_robot::{CollisionPrimitive, Joint, JointKind, JointLimits, Link, Robot, Transform3};
use serde_json::json;

const START: [f64; 2] = [0.2, 0.2];
const GOAL: [f64; 2] = [1.4, -1.0];
const OBSTACLE: [f64; 3] = [1.5, 1.0, 0.18];

fn main() {
    let mut world = World::new();
    let robot = spawn_named(&mut world, "arm");
    let base = spawn_named(&mut world, "base");
    let link1 = spawn_named(&mut world, "link1");
    let ee = spawn_named(&mut world, "ee");
    let tool = spawn_named(&mut world, "tool");

    for (link, name, local) in [
        (base, "base", Vec3::ZERO),
        (link1, "link1", Vec3::ZERO),
        (ee, "ee", Vec3::new(1.0, 0.0, 0.0)),
        (tool, "tool", Vec3::new(1.0, 0.0, 0.0)),
    ] {
        world.entity_mut(link).insert((
            Link {
                robot,
                name: name.to_string(),
            },
            Transform3::from_translation_rotation(local, Quat::IDENTITY),
            rne_physics::Collider::sphere(0.03),
        ));
    }
    world.entity_mut(robot).insert(Robot {
        robot_id: Default::default(),
        model_name: "arm".into(),
        base_link: base,
    });

    let limits = JointLimits {
        lower: -std::f64::consts::PI,
        upper: std::f64::consts::PI,
        max_velocity: 1.0,
        ..JointLimits::default()
    };
    for (parent, child, kind, name) in [
        (base, link1, JointKind::Revolute, "joint1"),
        (link1, ee, JointKind::Revolute, "joint2"),
        (ee, tool, JointKind::Fixed, "tool_joint"),
    ] {
        let joint = spawn_named(&mut world, name);
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: parent,
            child_link: child,
            kind,
            limits,
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });
    }

    let mut scene = PlanningScene::from_world(&world, robot).expect("planning scene");
    scene.add_collision_object(
        "obstacle",
        CollisionPrimitive::Sphere {
            center_m: Vec3::new(OBSTACLE[0], OBSTACLE[1], 0.0),
            radius_m: OBSTACLE[2],
        },
    );

    let options = PlanningOptions {
        seed: 7,
        step_size: 0.4,
        goal_bias: 0.2,
        max_iterations: 4_000,
        collision_check_steps: 8,
        neighbor_radius: 1.5,
        roadmap_neighbors: 8,
        waypoint_count: 40,
        acceleration_limits: vec![2.0; 2],
        ..PlanningOptions::default()
    };
    let request = MotionPlanRequest::new(
        START.to_vec(),
        GoalConstraint::joint(GOAL.to_vec()),
        options,
    );

    let planners: Vec<(&str, Box<dyn MotionPlanner>)> = vec![
        (
            "joint_interpolation",
            Box::new(JointInterpolationPlanner::new()),
        ),
        ("rrt_connect", Box::new(RrtConnectPlanner::new())),
        ("rrt_star", Box::new(RrtStarPlanner::new())),
        ("informed_rrt_star", Box::new(InformedRrtStarPlanner::new())),
        ("prm", Box::new(PrmPlanner::new())),
        ("hybrid", Box::new(HybridPlanner::new())),
        ("stomp", Box::new(StompPlanner::new())),
        ("bit_star", Box::new(BitStarPlanner::new())),
    ];

    let results: Vec<serde_json::Value> = planners
        .iter()
        .map(|(name, planner)| plan(name, planner.as_ref(), &scene, &request))
        .collect();

    let straight_line: Vec<[f64; 2]> = (0..40)
        .map(|index| {
            let t = index as f64 / 39.0;
            [
                START[0] + (GOAL[0] - START[0]) * t,
                START[1] + (GOAL[1] - START[1]) * t,
            ]
        })
        .collect();
    let straight_line_valid = scene.is_motion_valid(&START, &GOAL, 40).unwrap_or(false);

    let output = json!({
        "link_lengths_m": [1.0, 1.0],
        "joint_limits_rad": [-std::f64::consts::PI, std::f64::consts::PI],
        "start": START,
        "goal": GOAL,
        "obstacle": { "center": [OBSTACLE[0], OBSTACLE[1]], "radius": OBSTACLE[2] },
        "straight_line": straight_line,
        "straight_line_valid": straight_line_valid,
        "planners": results,
    });
    println!("{}", serde_json::to_string_pretty(&output).expect("json"));
}

fn plan(
    name: &str,
    planner: &dyn MotionPlanner,
    scene: &PlanningScene,
    request: &MotionPlanRequest,
) -> serde_json::Value {
    match planner.plan(scene, request) {
        Ok(response) => {
            let points: Vec<[f64; 2]> = response
                .trajectory
                .points()
                .iter()
                .map(|point| [point.positions[0], point.positions[1]])
                .collect();
            let mut valid = true;
            for window in response.trajectory.points().windows(2) {
                if !scene
                    .is_motion_valid(&window[0].positions, &window[1].positions, 8)
                    .unwrap_or(false)
                {
                    valid = false;
                    break;
                }
            }
            json!({
                "name": name,
                "ok": valid,
                "error": if valid { serde_json::Value::Null } else { json!("trajectory in collision") },
                "iterations": response.iterations,
                "duration_s": response.trajectory.duration_s(),
                "waypoints": points,
            })
        }
        Err(error) => json!({
            "name": name,
            "ok": false,
            "error": error.to_string(),
        }),
    }
}
