//! A delivery across three floors of a building, planned and then executed.
//!
//! Every other map in this repository is one horizontal surface, which is the
//! right model for a floor and the wrong model for a building. This example
//! plans a route through a [`BuildingMap`] — drive, cross, drive — and then
//! executes each crossing on the real [`Elevator`] device, so the plan's
//! crossing cost meets the car's actual timing.
//!
//! The delivery goes from the loading dock on 1F to a desk on 3F, past a
//! blocked corridor that forces the planner to prefer the far staircase over
//! the near one.
//!
//! No renderer is involved. Run with:
//!
//! ```text
//! cargo run --release -p multi_floor_mission --example 120_multi_floor_mission
//! ```

use rne_math::Vec3;
use rne_nav::{
    plan_building_route, save_map, BuildingDescription, BuildingMap, Elevator, ElevatorSpec,
    FloorEntry, FloorId, FloorPosition, GlobalPlannerConfig, GridCoord, OccupancyGrid, Pose2d,
    RouteCosts, RouteLeg, TransitionEntry, TransitionKind,
};
use std::path::{Path, PathBuf};

/// Floor extent in cells and meters per cell.
const CELLS: usize = 48;
const RESOLUTION_M: f64 = 0.25;
/// Physics-free control step used to run the elevator, in seconds.
const DT_S: f64 = 1.0 / 60.0;

/// Site directory holding the building description and its floor maps.
const SITE_DIR: &str = "assets/buildings/office_three_floor";
/// Costmap inflation each floor declares, in meters.
const INFLATION_RADIUS_M: f64 = 0.2;

/// Writes one floor's occupancy map, with optional wall segments.
///
/// A freshly created grid is entirely unknown and the planner refuses unknown
/// space, so the open floor has to be declared free rather than merely empty.
fn write_floor_map(path: &Path, walls: &[(usize, usize, usize, usize)]) {
    let mut grid = OccupancyGrid::new(CELLS, CELLS, RESOLUTION_M, Pose2d::new(0.0, 0.0, 0.0))
        .expect("floor grid");
    for y in 0..CELLS {
        for x in 0..CELLS {
            for _ in 0..8 {
                grid.mark_free(GridCoord {
                    x: x as isize,
                    y: y as isize,
                });
            }
        }
    }
    for (x0, y0, x1, y1) in walls {
        for y in *y0..=*y1 {
            for x in *x0..=*x1 {
                for _ in 0..8 {
                    grid.mark_occupied(GridCoord {
                        x: x as isize,
                        y: y as isize,
                    });
                }
            }
        }
    }
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).expect("site directory");
    }
    save_map(path, &grid).expect("write floor map");
}

fn lift(name: &str, from: usize, to: usize, point_m: [f64; 3]) -> TransitionEntry {
    TransitionEntry {
        name: name.to_string(),
        kind: TransitionKind::Elevator,
        from,
        to,
        from_point_m: point_m,
        to_point_m: point_m,
        cost_s: 20.0,
        bidirectional: true,
    }
}

/// Regenerates the committed site: three floor maps and the building file.
///
/// 2F carries a wall across the middle that seals off the near lift's landing,
/// so a route that lands there cannot continue.
fn emit_site(site_dir: &Path) {
    write_floor_map(&site_dir.join("1f.rne.map"), &[]);
    write_floor_map(&site_dir.join("2f.rne.map"), &[(0, 20, 30, 22)]);
    write_floor_map(&site_dir.join("3f.rne.map"), &[]);
    let description = BuildingDescription {
        name: "office three floor".to_string(),
        floors: vec![
            FloorEntry {
                id: 0,
                name: "1F".to_string(),
                elevation_m: 0.0,
                map: PathBuf::from("1f.rne.map"),
                inflation_radius_m: INFLATION_RADIUS_M,
            },
            FloorEntry {
                id: 1,
                name: "2F".to_string(),
                elevation_m: 3.5,
                map: PathBuf::from("2f.rne.map"),
                inflation_radius_m: INFLATION_RADIUS_M,
            },
            FloorEntry {
                id: 2,
                name: "3F".to_string(),
                elevation_m: 7.0,
                map: PathBuf::from("3f.rne.map"),
                inflation_radius_m: INFLATION_RADIUS_M,
            },
        ],
        transitions: vec![
            lift("near lift", 0, 1, [2.0, 2.0, 0.0]),
            lift("far lift", 0, 1, [10.0, 10.0, 0.0]),
            lift("upper lift", 1, 2, [10.0, 10.0, 0.0]),
        ],
    };
    description
        .save(&site_dir.join("office.rne.building"))
        .expect("write building description");
}

fn elevator_spec() -> ElevatorSpec {
    ElevatorSpec {
        floor_heights_m: vec![0.0, 3.5, 7.0],
        car_speed_m_s: 1.0,
        car_acceleration_m_s2: 0.8,
        door_travel_m: 0.6,
        door_speed_m_s: 0.6,
        door_hold_s: 2.0,
    }
}

/// Runs the car to `floor` and returns the seconds until boarding is allowed.
fn summon(elevator: &mut Elevator, floor: usize) -> f64 {
    elevator.call(floor).expect("call the floor");
    let mut elapsed_s = 0.0;
    for _ in 0..200_000 {
        if elevator.is_boardable(floor) {
            return elapsed_s;
        }
        elevator.update(DT_S).expect("elevator update");
        elapsed_s += DT_S;
    }
    panic!("the car never answered the call to floor {floor}");
}

fn main() {
    // The building is data, not code: a caller edits the site directory rather
    // than this file. `--emit-site` regenerates the committed copy.
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("repository root");
    let site_dir = repo_root.join(SITE_DIR);
    if std::env::args().any(|argument| argument == "--emit-site") {
        emit_site(&site_dir);
        println!("wrote the site to {}", site_dir.display());
    }
    let map: BuildingMap = BuildingDescription::load_from(&site_dir.join("office.rne.building"))
        .expect("load the committed building description");
    println!(
        "loaded {} floors and {} crossings from {}",
        map.floors().len(),
        map.transitions().len(),
        SITE_DIR
    );

    let dock = FloorPosition::new(FloorId(0), Vec3::new(1.0, 1.0, 0.0));
    let desk = FloorPosition::new(FloorId(2), Vec3::new(9.0, 3.0, 0.0));
    let route = plan_building_route(
        &map,
        dock,
        desk,
        RouteCosts::default(),
        &GlobalPlannerConfig::default(),
    )
    .expect("plan the delivery");

    println!(
        "planned {} legs at cost {:.2} from {} to {}",
        route.legs.len(),
        route.total_cost,
        map.floor(dock.floor).expect("dock floor").name,
        map.floor(desk.floor).expect("desk floor").name
    );

    let mut elevator = Elevator::new(elevator_spec(), 0).expect("elevator");
    let mut driven_m = 0.0;
    let mut waited_s = 0.0;
    let mut crossings = 0usize;

    for leg in &route.legs {
        match leg {
            RouteLeg::Drive { floor: id, path } => {
                let name = &map.floor(*id).expect("planned floor").name;
                println!(
                    "  drive {name}: {:.2} m over {} waypoints",
                    path.length_m(),
                    path.len()
                );
                driven_m += path.length_m();
            }
            RouteLeg::Cross {
                transition,
                from,
                to,
            } => {
                let crossing = &map.transitions()[*transition];
                // Execute the crossing on the real device: summon the car to
                // the floor being left, then ride to the floor being reached.
                let summon_s = summon(&mut elevator, from.0);
                let ride_s = summon(&mut elevator, to.0);
                println!(
                    "  cross `{}` {} -> {}: summoned in {summon_s:.2} s, rode in {ride_s:.2} s",
                    crossing.name,
                    map.floor(*from).expect("from floor").name,
                    map.floor(*to).expect("to floor").name,
                );
                waited_s += summon_s + ride_s;
                crossings += 1;
            }
        }
    }

    println!("executed: {driven_m:.2} m driven, {waited_s:.2} s in lifts, {crossings} crossings");

    // The delivery has to actually change floors, twice, to reach 3F.
    assert_eq!(crossings, 2, "a 1F to 3F delivery must cross twice");
    assert!(driven_m > 0.0);
    // The wall on 2F seals the near lift's landing, so the planner must route
    // through the far lift even though it is further from the dock.
    let used: Vec<&str> = route
        .legs
        .iter()
        .filter_map(|leg| match leg {
            RouteLeg::Cross { transition, .. } => {
                Some(map.transitions()[*transition].name.as_str())
            }
            RouteLeg::Drive { .. } => None,
        })
        .collect();
    assert_eq!(
        used,
        vec!["far lift", "upper lift"],
        "the blocked 2F landing must push the route onto the far lift"
    );

    // The car ends where the route left it, with its doors open for the robot.
    assert!(elevator.is_boardable(2));
    println!("car parked at 3F with doors open");

    // Planning is deterministic for the same building and endpoints.
    let repeat = plan_building_route(
        &map,
        dock,
        desk,
        RouteCosts::default(),
        &GlobalPlannerConfig::default(),
    )
    .expect("replan");
    assert_eq!(repeat, route, "the same delivery must replan identically");
    println!("replan is identical");
}
