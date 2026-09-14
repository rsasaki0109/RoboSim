//! Global A* planning plus pure-pursuit and DWA local control over a costmap,
//! all in the ROS-free `rne_nav` core.
//!
//! Run with `cargo run -p nav_planning --example 96_nav_planning`.

use rne_math::Vec3;
use rne_nav::{
    plan_path, Costmap, CostmapConfig, DwaConfig, DwaOutcome, DwaPlanner, GlobalPlannerConfig,
    GridCoord, OccupancyGrid, Path2d, Pose2d, PurePursuitConfig, VelocityCommand2d,
};

const RESOLUTION_M: f64 = 0.05;

fn main() {
    let costmap = build_costmap();
    let start = Pose2d::new(-4.0, 0.0, 0.0);
    let goal = Vec3::new(4.0, 0.0, 0.0);

    let path = plan_path(
        &costmap,
        Vec3::new(start.x_m, start.y_m, 0.0),
        goal,
        &GlobalPlannerConfig::default(),
    )
    .expect("global plan");
    println!(
        "A* path: {} waypoints, {:.2} m",
        path.len(),
        path.length_m()
    );

    let (pp_steps, pp_reached, pp_pose) = simulate_pure_pursuit(&path, start);
    println!(
        "pure pursuit: {} steps, reached={}, final=({:.2}, {:.2})",
        pp_steps, pp_reached, pp_pose.x_m, pp_pose.y_m
    );

    let (dwa_steps, dwa_reached, dwa_pose) = simulate_dwa(&costmap, &path, start);
    println!(
        "DWA: {} steps, reached={}, final=({:.2}, {:.2})",
        dwa_steps, dwa_reached, dwa_pose.x_m, dwa_pose.y_m
    );

    println!("\nplanned route ('#' obstacle, '*' path, '.' free, ' ' unknown):");
    print!("{}", render(&costmap, &path, 72, 24));
}

fn build_costmap() -> Costmap {
    let grid = OccupancyGrid::new(
        (12.0 / RESOLUTION_M) as usize,
        (8.0 / RESOLUTION_M) as usize,
        RESOLUTION_M,
        Pose2d::new(-6.0, -4.0, 0.0),
    )
    .expect("grid");
    let mut grid = grid;
    let (width, height) = (grid.width(), grid.height());
    for y in 0..height {
        for x in 0..width {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            grid.mark_free(coord);
            grid.mark_free(coord);
        }
    }
    // Vertical wall at x = 0 with a gap above y = 0.6 m.
    mark_rectangle(&mut grid, -0.1, -3.0, 0.1, 0.6);
    // A free-standing block to force a wider detour.
    mark_rectangle(&mut grid, 1.8, -1.4, 2.6, -0.6);
    Costmap::from_occupancy(
        &grid,
        &CostmapConfig {
            inscribed_radius_m: 0.15,
            inflation_radius_m: 0.45,
            ..CostmapConfig::default()
        },
    )
    .expect("costmap")
}

fn mark_rectangle(grid: &mut OccupancyGrid, min_x: f64, min_y: f64, max_x: f64, max_y: f64) {
    let steps_x = ((max_x - min_x) / RESOLUTION_M).ceil() as i64;
    let steps_y = ((max_y - min_y) / RESOLUTION_M).ceil() as i64;
    for iy in 0..=steps_y {
        for ix in 0..=steps_x {
            let world = Vec3::new(
                min_x + ix as f64 * RESOLUTION_M,
                min_y + iy as f64 * RESOLUTION_M,
                0.0,
            );
            if let Some(coord) = grid.world_to_grid(world) {
                grid.reset(coord);
                grid.mark_occupied(coord);
            }
        }
    }
}

fn simulate_pure_pursuit(path: &Path2d, start: Pose2d) -> (usize, bool, Pose2d) {
    let config = PurePursuitConfig {
        max_linear_m_s: 1.2,
        lookahead_m: 0.5,
        ..PurePursuitConfig::default()
    };
    let dt = 0.05;
    let mut pose = start;
    for step in 0..6000 {
        let Ok(result) = rne_nav::pure_pursuit_follow(path, pose, &config) else {
            return (step, false, pose);
        };
        if result.reached {
            return (step, true, pose);
        }
        integrate(&mut pose, result.command, dt);
    }
    (6000, false, pose)
}

fn simulate_dwa(costmap: &Costmap, path: &Path2d, start: Pose2d) -> (usize, bool, Pose2d) {
    let config = DwaConfig {
        max_linear_m_s: 1.2,
        goal_tolerance_m: 0.15,
        ..DwaConfig::default()
    };
    let planner = DwaPlanner::new(config);
    let dt = config.simulation_step_s;
    let mut pose = start;
    let mut velocity = VelocityCommand2d::ZERO;
    for step in 0..3000 {
        match planner.compute_command(costmap, path, pose, velocity) {
            Ok(DwaOutcome::Reached) => return (step, true, pose),
            Ok(DwaOutcome::Command(command)) => {
                velocity = command;
                integrate(&mut pose, command, dt);
            }
            Err(_) => return (step, false, pose),
        }
    }
    (3000, false, pose)
}

fn integrate(pose: &mut Pose2d, command: VelocityCommand2d, dt: f64) {
    pose.x_m += command.linear_m_s * pose.yaw_rad.cos() * dt;
    pose.y_m += command.linear_m_s * pose.yaw_rad.sin() * dt;
    pose.yaw_rad += command.angular_rad_s * dt;
}

fn render(costmap: &Costmap, path: &Path2d, columns: usize, rows: usize) -> String {
    let path_cells: std::collections::HashSet<(usize, usize)> = path
        .waypoints()
        .iter()
        .filter_map(|waypoint| {
            costmap
                .world_to_grid(Vec3::new(waypoint.x_m, waypoint.y_m, 0.0))
                .map(|coord| {
                    (
                        coord.x as usize * columns / costmap.width(),
                        coord.y as usize * rows / costmap.height(),
                    )
                })
        })
        .collect();

    let mut output = String::new();
    for row in (0..rows).rev() {
        for column in 0..columns {
            let x0 = column * costmap.width() / columns;
            let x1 = ((column + 1) * costmap.width() / columns).max(x0 + 1);
            let y0 = row * costmap.height() / rows;
            let y1 = ((row + 1) * costmap.height() / rows).max(y0 + 1);
            let mut lethal = false;
            let mut known = false;
            for y in y0..y1 {
                for x in x0..x1 {
                    let coord = GridCoord {
                        x: x as isize,
                        y: y as isize,
                    };
                    lethal |= costmap.is_lethal(coord);
                    known |= costmap.cost_at(coord) != Some(rne_nav::COST_NO_INFORMATION);
                }
            }
            if path_cells.contains(&(column, row)) {
                output.push('*');
            } else if lethal {
                output.push('#');
            } else if known {
                output.push('.');
            } else {
                output.push(' ');
            }
        }
        output.push('\n');
    }
    output
}
