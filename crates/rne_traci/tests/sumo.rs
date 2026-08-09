//! Integration test against a real SUMO process.
//!
//! Skips (with a message) when the `sumo` binary is not on `PATH`, so the
//! suite stays green without SUMO installed. CI installs `eclipse-sumo` via
//! pip so this test runs there.

use rne_traci::TraciClient;
use std::net::TcpListener;
use std::path::PathBuf;
use std::process::{Command, Stdio};
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

#[test]
fn co_simulates_a_moving_vehicle_with_a_real_sumo_when_available() {
    if !sumo_available() {
        eprintln!(
            "skipping real-SUMO test: `sumo` is not on PATH (install with `pip install eclipse-sumo`)"
        );
        return;
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let net = root.join("assets/networks/minimal_cross.net.xml");
    let routes = root.join("assets/networks/sumo_cross_flow.rou.xml");
    let port = free_port();
    let stderr_log = std::env::temp_dir().join(format!("rne-traci-sumo-{port}.log"));
    let stderr_file = std::fs::File::create(&stderr_log).expect("create stderr log");
    let mut child = Command::new("sumo")
        .args([
            "--net-file",
            net.to_str().expect("net path"),
            "--route-files",
            routes.to_str().expect("route path"),
            "--remote-port",
            &port.to_string(),
            "--start",
            "--no-warnings",
            "--no-step-log",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::from(stderr_file))
        .spawn()
        .expect("spawn sumo");

    let sumo_log = || std::fs::read_to_string(&stderr_log).unwrap_or_default();
    let mut fail = |context: &str, error: String| -> ! {
        let _ = child.kill();
        let _ = child.wait();
        let log = sumo_log();
        panic!("{context}: {error}\nsumo stderr:\n{log}");
    };

    let mut client = None;
    for _ in 0..100 {
        match TraciClient::connect("127.0.0.1", port) {
            Ok(connected) => {
                client = Some(connected);
                break;
            }
            Err(_) => std::thread::sleep(Duration::from_millis(100)),
        }
    }
    let mut client = client.unwrap_or_else(|| {
        fail(
            "could not connect to SUMO",
            "connection never established".to_string(),
        )
    });

    let (api, _name) = match client.get_version() {
        Ok(version) => version,
        Err(error) => fail("get version from SUMO", error.to_string()),
    };
    assert!(api > 0, "TraCI API version must be positive, got {api}");

    // The vehicle departs at t=0 on the northbound edge (SUMO y from 300 down
    // to 200, i.e. RNE z from -300 up to -200) at up to 10 m/s. Step a few
    // times and require it to appear and move in RNE coordinates.
    let mut previous_z = None;
    for _ in 0..5 {
        if let Err(error) = client.simulation_step() {
            fail("simulation step", error.to_string());
        }
        let ids = match client.vehicle_ids() {
            Ok(ids) => ids,
            Err(error) => fail("vehicle id list", error.to_string()),
        };
        assert!(
            ids.iter().any(|id| id == "v0"),
            "the fixture vehicle must be present after stepping, got {ids:?}"
        );
        let position = match client.vehicle_position_rne("v0") {
            Ok(position) => position,
            Err(error) => fail("vehicle position", error.to_string()),
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
        fail("close SUMO connection", error.to_string());
    }
    let status = child.wait().expect("wait for sumo");
    assert!(status.success(), "sumo should exit cleanly after close");
    let _ = std::fs::remove_file(&stderr_log);
}
