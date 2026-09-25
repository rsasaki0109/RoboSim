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
//! cargo run --release -p multi_floor_mission --example 121_multi_floor_mission
//! ```

use rne_math::Vec3;
use rne_nav::{
    plan_building_route, BuildingMap, Costmap, CostmapConfig, Elevator, ElevatorSpec, Floor,
    FloorId, FloorPosition, FloorTransition, GlobalPlannerConfig, GridCoord, OccupancyGrid, Pose2d,
    RouteCosts, RouteLeg, TransitionKind,
};

/// Floor extent in cells and meters per cell.
const CELLS: usize = 48;
const RESOLUTION_M: f64 = 0.25;
/// Physics-free control step used to run the elevator, in seconds.
const DT_S: f64 = 1.0 / 60.0;

/// Builds a floor whose free space is fully known, with optional wall segments.
///
/// A freshly created grid is entirely unknown and the planner refuses unknown
/// space, so the open floor has to be declared free rather than merely empty.
fn floor(id: usize, name: &str, elevation_m: f64, walls: &[(usize, usize, usize, usize)]) -> Floor {
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
    Floor {
        id: FloorId(id),
        name: name.to_string(),
        elevation_m,
        costmap: Costmap::from_occupancy(&grid, &CostmapConfig::default()).expect("costmap"),
    }
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
    // 2F has a wall across the middle that seals off the near lift's landing,
    // so a route that lands there cannot continue.
    let blocking_wall = [(0usize, 20usize, 30usize, 22usize)];
    let map = BuildingMap::new(
        vec![
            floor(0, "1F", 0.0, &[]),
            floor(1, "2F", 3.5, &blocking_wall),
            floor(2, "3F", 7.0, &[]),
        ],
        vec![
            FloorTransition {
                name: "near lift".to_string(),
                kind: TransitionKind::Elevator,
                from: FloorId(0),
                to: FloorId(1),
                from_point_m: Vec3::new(2.0, 2.0, 0.0),
                to_point_m: Vec3::new(2.0, 2.0, 0.0),
                cost_s: 20.0,
                bidirectional: true,
            },
            FloorTransition {
                name: "far lift".to_string(),
                kind: TransitionKind::Elevator,
                from: FloorId(0),
                to: FloorId(1),
                from_point_m: Vec3::new(10.0, 10.0, 0.0),
                to_point_m: Vec3::new(10.0, 10.0, 0.0),
                cost_s: 20.0,
                bidirectional: true,
            },
            FloorTransition {
                name: "upper lift".to_string(),
                kind: TransitionKind::Elevator,
                from: FloorId(1),
                to: FloorId(2),
                from_point_m: Vec3::new(10.0, 10.0, 0.0),
                to_point_m: Vec3::new(10.0, 10.0, 0.0),
                cost_s: 20.0,
                bidirectional: true,
            },
        ],
    )
    .expect("building map");

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
