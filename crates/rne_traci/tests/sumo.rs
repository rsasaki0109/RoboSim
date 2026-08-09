//! Integration tests against a real SUMO process.
//!
//! Skips (with a message) when the `sumo` binary is not on `PATH`, so the
//! suite stays green without SUMO installed. CI installs `eclipse-sumo` via
//! pip so these tests run there.

use rne_ecs::World;
use rne_traci::{CoSimulation, TraciClient};
use rne_traffic::TrafficPose;
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::Duration;

fn sumo_available() -> bool {
    Command::new("sumo")
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}

fn free_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind free port");
    listener.local_addr().expect("local address").port()
}

struct SumoFixture {
    child: Child,
    stderr_log: PathBuf,
    port: u16,
}

impl SumoFixture {
    fn spawn(net: &Path, routes: Option<&Path>) -> Self {
        let port = free_port();
        let stderr_log = std::env::temp_dir().join(format!("rne-traci-sumo-{port}.log"));
        let stderr_file = std::fs::File::create(&stderr_log).expect("create stderr log");
        let mut args = vec![
            "--net-file".to_string(),
            net.to_str().expect("net path").to_string(),
            "--remote-port".to_string(),
            port.to_string(),
            "--start".to_string(),
            "--no-warnings".to_string(),
            "--no-step-log".to_string(),
        ];
        if let Some(routes) = routes {
            args.extend([
                "--route-files".to_string(),
                routes.to_str().expect("route path").to_string(),
            ]);
        }
        let child = Command::new("sumo")
            .args(&args)
            .stdout(Stdio::null())
            .stderr(Stdio::from(stderr_file))
            .spawn()
            .expect("spawn sumo");
        Self {
            child,
            stderr_log,
            port,
        }
    }

    fn log(&self) -> String {
        std::fs::read_to_string(&self.stderr_log).unwrap_or_default()
    }

    fn connect(&mut self) -> TraciClient {
        for _ in 0..100 {
            match TraciClient::connect("127.0.0.1", self.port) {
                Ok(client) => return client,
                Err(_) => std::thread::sleep(Duration::from_millis(100)),
            }
        }
        self.fail(
            "could not connect to SUMO",
            "connection never established".to_string(),
        );
    }

    fn fail(&mut self, context: &str, error: String) -> ! {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let log = self.log();
        panic!("{context}: {error}\nsumo stderr:\n{log}");
    }
}

fn fixture_paths() -> (PathBuf, PathBuf) {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let net = root.join("assets/networks/minimal_cross.net.xml");
    let routes = root.join("assets/networks/sumo_cross_flow.rou.xml");
    (net, routes)
}

fn expect_version(fixture: &mut SumoFixture, client: &mut TraciClient) {
    let (api, _name) = match client.get_version() {
        Ok(version) => version,
        Err(error) => fixture.fail("get version from SUMO", error.to_string()),
    };
    assert!(api > 0, "TraCI API version must be positive, got {api}");
}

#[test]
fn co_simulates_a_moving_vehicle_with_a_real_sumo_when_available() {
    if !sumo_available() {
        eprintln!(
            "skipping real-SUMO test: `sumo` is not on PATH (install with `pip install eclipse-sumo`)"
        );
        return;
    }
    let (net, routes) = fixture_paths();
    let mut fixture = SumoFixture::spawn(&net, Some(&routes));
    let mut client = fixture.connect();
    expect_version(&mut fixture, &mut client);

    let mut previous_z = None;
    for _ in 0..5 {
        if let Err(error) = client.simulation_step() {
            fixture.fail("simulation step", error.to_string());
        }
        let ids = match client.vehicle_ids() {
            Ok(ids) => ids,
            Err(error) => fixture.fail("vehicle id list", error.to_string()),
        };
        assert!(
            ids.iter().any(|id| id == "v0"),
            "the fixture vehicle must be present after stepping, got {ids:?}"
        );
        let position = match client.vehicle_position_rne("v0") {
            Ok(position) => position,
            Err(error) => fixture.fail("vehicle position", error.to_string()),
        };
        assert!(
            position[0].is_finite() && position[1] == 0.0 && position[2].is_finite(),
            "RNE position must be finite with Y up, got {position:?}"
        );
        assert!(
            (-300.0..=-200.0).contains(&position[2]),
            "the vehicle must stay on the northbound edge, got {position:?}"
        );
        if let Some(previous) = previous_z {
            assert!(
                position[2] > previous,
                "the vehicle must move toward the intersection, got z {} then {}",
                previous,
                position[2]
            );
        }
        previous_z = Some(position[2]);
    }

    if let Err(error) = client.close() {
        fixture.fail("close SUMO connection", error.to_string());
    }
    let status = fixture.child.wait().expect("wait for sumo");
    assert!(status.success(), "sumo should exit cleanly after close");
    let _ = std::fs::remove_file(&fixture.stderr_log);
}

#[test]
fn co_simulation_bridge_mirrors_a_real_sumo_vehicle() {
    if !sumo_available() {
        eprintln!(
            "skipping real-SUMO test: `sumo` is not on PATH (install with `pip install eclipse-sumo`)"
        );
        return;
    }
    let (net, routes) = fixture_paths();
    let mut fixture = SumoFixture::spawn(&net, Some(&routes));
    let mut co_sim = match CoSimulation::connect("127.0.0.1", fixture.port) {
        Ok(co_sim) => co_sim,
        Err(_) => fixture.fail(
            "could not connect to SUMO",
            "connection never established".to_string(),
        ),
    };
    let mut world = World::new();
    for _ in 0..5 {
        if let Err(error) = co_sim.step(&mut world) {
            fixture.fail("co-simulation step", error.to_string());
        }
    }
    assert_eq!(
        co_sim.actors().len(),
        1,
        "the bridge must mirror the fixture vehicle"
    );
    let entity = co_sim.actors()["v0"];
    let pose = world.get::<TrafficPose>(entity).expect("mirror pose");
    assert!(
        pose.position_m[1] == 0.0 && (-300.0..=-200.0).contains(&pose.position_m[2]),
        "the mirror must track the vehicle on the northbound edge, got {:?}",
        pose.position_m
    );

    if let Err(error) = co_sim.close() {
        fixture.fail("close SUMO connection", error.to_string());
    }
    let status = fixture.child.wait().expect("wait for sumo");
    assert!(status.success(), "sumo should exit cleanly after close");
    let _ = std::fs::remove_file(&fixture.stderr_log);
}
