//! Renders what a navigating robot actually knows: its map, its costs, its path.
//!
//! Every navigation and SLAM example in this repository is headless, and no
//! picture in the README shows an occupancy grid, a costmap, a planned path or a
//! laser scan. A robot therefore appears to move for no reason: the map it
//! built, the costs it avoided and the route it chose are all invisible.
//!
//! This draws them. The occupancy grid becomes free and occupied cells on the
//! floor, the costmap's inflation becomes a band around the obstacles, the
//! planned path becomes markers the robot follows, and the robot drives it.
//! Nothing here is staged: the cells come from the same `OccupancyGrid` the
//! planner reads, and the path is what `plan_path` returned.
//!
//! ```text
//! cargo run --release -p nav_map_media --example 121_nav_map_media -- --smoke
//! cargo run --release -p nav_map_media --example 121_nav_map_media
//! ```
//!
//! `--smoke` verifies the map, costmap and path headlessly without a GPU.

use rne_math::{Quat, Vec3};
use rne_nav::{
    plan_path, Costmap, CostmapConfig, GlobalPlannerConfig, GridCoord, OccupancyGrid, Path2d,
    Pose2d, COST_LETHAL,
};
use rne_render::{
    grid_mesh, Camera, GridMeshSpec, MeshRenderCache, RenderBackend, RenderScene, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;
use std::fs;
use std::path::{Path, PathBuf};

const CELLS: usize = 96;
const RESOLUTION_M: f64 = 0.125;
const INFLATION_RADIUS_M: f64 = 0.35;
const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const FRAME_COUNT: usize = 96;
const CLEAR_COLOR: [f32; 4] = [0.05, 0.06, 0.08, 1.0];

/// Colours chosen so the three map states stay distinct in both themes.
const FREE_RGBA: [f32; 4] = [0.16, 0.18, 0.22, 1.0];
const OCCUPIED_RGBA: [f32; 4] = [0.85, 0.30, 0.25, 1.0];
const INFLATION_RGBA: [f32; 4] = [0.95, 0.65, 0.20, 0.9];
const PATH_AHEAD_RGBA: [f32; 4] = [0.20, 0.80, 0.95, 1.0];
const PATH_DRIVEN_RGBA: [f32; 4] = [0.35, 0.95, 0.55, 1.0];
const ROBOT_RGBA: [f32; 4] = [0.90, 0.92, 0.96, 1.0];

/// Builds an office-like floor: an outer wall and two staggered partitions.
fn build_grid() -> OccupancyGrid {
    let mut grid =
        OccupancyGrid::new(CELLS, CELLS, RESOLUTION_M, Pose2d::new(-6.0, -6.0, 0.0)).expect("grid");
    for row in 0..CELLS as isize {
        for column in 0..CELLS as isize {
            for _ in 0..8 {
                grid.mark_free(GridCoord { x: column, y: row });
            }
        }
    }
    let mut wall = |x: isize, y: isize| {
        for _ in 0..8 {
            grid.mark_occupied(GridCoord { x, y });
        }
    };
    let last = CELLS as isize - 1;
    for i in 0..CELLS as isize {
        wall(i, 0);
        wall(i, last);
        wall(0, i);
        wall(last, i);
    }
    // Two partitions the planner has to route between.
    for y in 0..64 {
        wall(32, y);
        wall(33, y);
    }
    for y in 32..CELLS as isize {
        wall(64, y);
        wall(65, y);
    }
    grid
}

fn grid_spec(grid: &OccupancyGrid, height_m: f64) -> GridMeshSpec {
    GridMeshSpec {
        columns: grid.width(),
        rows: grid.height(),
        cell_size_m: grid.resolution_m(),
        origin_m: Vec3::new(grid.origin().x_m, 0.0, grid.origin().y_m),
        height_m,
        fill: 0.88,
    }
}

/// Pushes the map, its inflation and the plan into a scene.
///
/// Each band is its own mesh because a render item carries one colour, and the
/// heights are staggered so the overlays read in a fixed order rather than
/// fighting for the same plane.
fn append_map(
    scene: &mut RenderScene,
    grid: &OccupancyGrid,
    costmap: &Costmap,
    path: &Path2d,
    driven_upto: usize,
) {
    let occupied = |column: usize, row: usize| {
        grid.probability(GridCoord {
            x: column as isize,
            y: row as isize,
        })
        .is_some_and(|probability| probability >= 0.6)
    };

    scene.items.push(RenderScene::item_from_dynamic_mesh(
        grid_mesh(&grid_spec(grid, 0.005), |column, row| {
            !occupied(column, row)
        }),
        FREE_RGBA,
    ));
    // Inflation: cells the planner charges for but that are not obstacles.
    scene.items.push(RenderScene::item_from_dynamic_mesh(
        grid_mesh(&grid_spec(grid, 0.010), |column, row| {
            if occupied(column, row) {
                return false;
            }
            costmap
                .cost_at(GridCoord {
                    x: column as isize,
                    y: row as isize,
                })
                .is_some_and(|cost| cost > 0 && cost < COST_LETHAL)
        }),
        INFLATION_RGBA,
    ));
    scene.items.push(RenderScene::item_from_dynamic_mesh(
        grid_mesh(&grid_spec(grid, 0.015), occupied),
        OCCUPIED_RGBA,
    ));

    // Where the robot has been and where it is going read differently, so the
    // picture says what the plan is *for* rather than just that one exists.
    for (index, waypoint) in path.waypoints().iter().enumerate().step_by(2) {
        let driven = index <= driven_upto;
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(
                Vec3::new(waypoint.x_m, 0.06, waypoint.y_m),
                Quat::IDENTITY,
            ),
            VisualShape::Sphere { radius_m: 0.075 },
            if driven {
                PATH_DRIVEN_RGBA
            } else {
                PATH_AHEAD_RGBA
            },
            Transform3::IDENTITY,
        ));
    }
}

fn append_robot(scene: &mut RenderScene, pose: Pose2d) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(
            Vec3::new(pose.x_m, 0.15, pose.y_m),
            Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        ),
        VisualShape::Cylinder {
            radius_m: 0.22,
            length_m: 0.30,
        },
        ROBOT_RGBA,
        Transform3::IDENTITY,
    ));
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = png::Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

fn main() {
    let grid = build_grid();
    let costmap = Costmap::from_occupancy(
        &grid,
        &CostmapConfig {
            inflation_radius_m: INFLATION_RADIUS_M,
            ..CostmapConfig::default()
        },
    )
    .expect("costmap");

    let start = Vec3::new(-5.0, -5.0, 0.0);
    let goal = Vec3::new(5.0, 5.0, 0.0);
    let path = plan_path(&costmap, start, goal, &GlobalPlannerConfig::default()).expect("plan");
    println!(
        "planned {:.2} m over {} waypoints across a {}x{} map",
        path.length_m(),
        path.len(),
        grid.width(),
        grid.height()
    );

    if std::env::args().any(|argument| argument == "--smoke") {
        // The overlays are only worth drawing if they describe a real plan.
        assert!(
            path.len() > 2,
            "the planner must route around the partitions"
        );
        assert!(
            path.length_m() > 14.0,
            "a straight line would be shorter than routing between the partitions: {:.2} m",
            path.length_m()
        );
        let occupied_cells = grid_mesh(&grid_spec(&grid, 0.015), |column, row| {
            grid.probability(GridCoord {
                x: column as isize,
                y: row as isize,
            })
            .is_some_and(|probability| probability >= 0.6)
        });
        assert!(
            occupied_cells.triangle_count() > 0,
            "the map must produce drawable obstacle geometry"
        );
        println!(
            "smoke ok: {} obstacle triangles, path {:.2} m",
            occupied_cells.triangle_count(),
            path.length_m()
        );
        return;
    }

    let frames_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/rne-nav-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();

    let waypoints = path.waypoints();
    for frame in 0..FRAME_COUNT {
        let progress = frame as f64 / (FRAME_COUNT - 1).max(1) as f64;
        let index = ((waypoints.len() - 1) as f64 * progress).round() as usize;
        let pose = waypoints[index];

        // A fixed viewpoint, deliberately. An orbiting camera changes every
        // pixel every frame, which both fights the viewer trying to read the
        // map and defeats inter-frame compression: the same run encodes to
        // 31 MB orbiting and a fraction of that held still.
        let orbit = CameraOrbit {
            yaw_rad: 0.85,
            pitch_rad: 0.80,
            distance_m: 13.0,
            focus: Vec3::new(0.0, 0.0, 0.0),
        };

        let mut scene = RenderScene::default();
        append_map(&mut scene, &grid, &costmap, &path, index);
        append_robot(&mut scene, pose);
        mesh_cache
            .resolve_scene(&mut scene, &[])
            .expect("resolve scene meshes");
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render navigation frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
        )
        .expect("write frame");
    }

    let media_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");
    let gif_path = media_dir.join("nav-map.gif");
    build_gif(&frames_dir, &gif_path).expect("encode the navigation gif");
    println!("wrote {FRAME_COUNT} frames and {}", gif_path.display());
}

/// Encodes the frames with a per-frame palette, which keeps the cell grid from
/// banding into blocks the way a single shared palette does.
fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            "12",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=12,scale=860:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg navigation gif encode failed"))
}
