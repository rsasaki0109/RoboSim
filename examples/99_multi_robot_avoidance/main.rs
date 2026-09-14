//! Deterministic multi-robot goal seeking with local sense-and-avoid.
//!
//! Several differential-drive robots cross a shared plane. Each one seeks its
//! goal and uses `rne_nav::avoid_velocities` against the predicted positions of
//! the others. The simulation is pure and reproducible: running it twice yields
//! identical final poses and minimum separation.
//!
//! Run with `cargo run -p multi_robot_avoidance --example 99_multi_robot_avoidance`.

use rne_math::Vec3;
use rne_nav::{avoid_velocities, AvoidanceConfig, CircularObstacle, Pose2d, VelocityCommand2d};
use std::f64::consts::PI;

const ROBOT_RADIUS_M: f64 = 0.2;
const DT_S: f64 = 0.1;
const MAX_STEPS: usize = 800;
const MAX_LINEAR_M_S: f64 = 0.8;
const MAX_ANGULAR_RAD_S: f64 = 1.5;
const GOAL_TOLERANCE_M: f64 = 0.15;

#[derive(Clone, Copy)]
struct Robot {
    pose: Pose2d,
    goal: Vec3,
    command: VelocityCommand2d,
}

#[derive(Clone, Debug, PartialEq)]
struct Report {
    steps: usize,
    reached: usize,
    min_separation_m: f64,
    final_poses: Vec<Pose2d>,
}

fn main() {
    let first = simulate();
    let second = simulate();
    assert_eq!(first, second, "fleet simulation must be deterministic");

    println!("robots            = {}", first.final_poses.len());
    println!("steps             = {}", first.steps);
    println!("reached           = {}", first.reached);
    println!("min separation    = {:.3} m", first.min_separation_m);
    for (index, pose) in first.final_poses.iter().enumerate() {
        println!(
            "  robot {index}: ({:.2}, {:.2}, {:.3})",
            pose.x_m, pose.y_m, pose.yaw_rad
        );
    }

    // Collision-free (never closer than the summed radii) and all goals met.
    assert!(
        first.min_separation_m >= 2.0 * ROBOT_RADIUS_M,
        "robots collided: min separation {:.3}",
        first.min_separation_m
    );
    assert_eq!(
        first.reached,
        first.final_poses.len(),
        "not all robots reached"
    );
    println!("multi-robot avoidance passed");
}

fn simulate() -> Report {
    let mut robots = [
        Robot {
            pose: Pose2d::new(-1.5, -0.25, 0.0),
            goal: Vec3::new(1.5, -0.25, 0.0),
            command: VelocityCommand2d::ZERO,
        },
        Robot {
            pose: Pose2d::new(1.5, 0.25, PI),
            goal: Vec3::new(-1.5, 0.25, 0.0),
            command: VelocityCommand2d::ZERO,
        },
        Robot {
            pose: Pose2d::new(0.0, -1.5, PI / 2.0),
            goal: Vec3::new(0.0, 1.5, 0.0),
            command: VelocityCommand2d::ZERO,
        },
    ];
    let config = AvoidanceConfig {
        time_horizon_s: 2.0,
        safety_margin_m: 0.1,
        ..AvoidanceConfig::default()
    };
    let mut min_separation = f64::INFINITY;
    let mut steps = 0;

    for step in 0..MAX_STEPS {
        steps = step + 1;
        let mut commands = Vec::with_capacity(robots.len());
        for (index, robot) in robots.iter().enumerate() {
            let obstacles: Vec<CircularObstacle> = robots
                .iter()
                .enumerate()
                .filter(|(other, _)| *other != index)
                .map(|(_, other)| CircularObstacle {
                    center_m: Vec3::new(other.pose.x_m, other.pose.y_m, 0.0),
                    velocity_m_s: Vec3::new(
                        other.command.linear_m_s * other.pose.yaw_rad.cos(),
                        other.command.linear_m_s * other.pose.yaw_rad.sin(),
                        0.0,
                    ),
                    radius_m: ROBOT_RADIUS_M,
                })
                .collect();
            let desired = seek(robot.pose, robot.goal);
            commands.push(avoid_velocities(
                robot.pose,
                ROBOT_RADIUS_M,
                &obstacles,
                desired,
                MAX_LINEAR_M_S,
                MAX_ANGULAR_RAD_S,
                &config,
            ));
        }
        for (robot, command) in robots.iter_mut().zip(commands) {
            robot.command = command;
            robot.pose.x_m += command.linear_m_s * robot.pose.yaw_rad.cos() * DT_S;
            robot.pose.y_m += command.linear_m_s * robot.pose.yaw_rad.sin() * DT_S;
            robot.pose.yaw_rad += command.angular_rad_s * DT_S;
        }

        for i in 0..robots.len() {
            for j in (i + 1)..robots.len() {
                let dx = robots[i].pose.x_m - robots[j].pose.x_m;
                let dy = robots[i].pose.y_m - robots[j].pose.y_m;
                min_separation = min_separation.min(dx.hypot(dy));
            }
        }

        let all_reached = robots.iter().all(|robot| {
            (robot.pose.x_m - robot.goal.x).hypot(robot.pose.y_m - robot.goal.y) <= GOAL_TOLERANCE_M
        });
        if all_reached {
            break;
        }
    }

    let reached = robots
        .iter()
        .filter(|robot| {
            (robot.pose.x_m - robot.goal.x).hypot(robot.pose.y_m - robot.goal.y) <= GOAL_TOLERANCE_M
        })
        .count();
    Report {
        steps,
        reached,
        min_separation_m: min_separation,
        final_poses: robots.iter().map(|robot| robot.pose).collect(),
    }
}

fn seek(pose: Pose2d, goal: Vec3) -> VelocityCommand2d {
    let dx = goal.x - pose.x_m;
    let dy = goal.y - pose.y_m;
    let distance = dx.hypot(dy);
    if distance <= GOAL_TOLERANCE_M {
        return VelocityCommand2d::ZERO;
    }
    let heading_error = wrap_angle(dy.atan2(dx) - pose.yaw_rad);
    let alignment = (1.0 - heading_error.abs() / PI).clamp(0.0, 1.0);
    let linear = MAX_LINEAR_M_S * alignment * (distance / 0.5).clamp(0.2, 1.0);
    let angular = (1.5 * heading_error).clamp(-MAX_ANGULAR_RAD_S, MAX_ANGULAR_RAD_S);
    VelocityCommand2d::new(linear.max(0.0), angular)
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}
