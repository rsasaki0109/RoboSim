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
fn connects_and_co_simulates_with_a_real_sumo_when_available() {
    if !sumo_available() {
        eprintln!(
            "skipping real-SUMO test: `sumo` is not on PATH (install with `pip install eclipse-sumo`)"
        );
        return;
    }
    let net = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/networks/minimal_cross.net.xml");
    let port = free_port();
    let stderr_log = std::env::temp_dir().join(format!("rne-traci-sumo-{port}.log"));
    let stderr_file = std::fs::File::create(&stderr_log).expect("create stderr log");
    let mut child = Command::new("sumo")
        .args([
            "--net-file",
            net.to_str().expect("net path"),
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
    let mut client = match client {
        Some(client) => client,
        None => {
            let log = std::fs::read_to_string(&stderr_log).unwrap_or_default();
            let _ = child.kill();
            panic!("could not connect to SUMO on port {port}; stderr:\n{log}");
        }
    };

    let (api, _name) = client.get_version().expect("get version from SUMO");
    assert!(api > 0, "TraCI API version must be positive, got {api}");

    client.simulation_step().expect("simulation step");
    let ids = client.vehicle_ids().expect("vehicle id list");
    assert!(
        ids.is_empty(),
        "the fixture net has no routes, so no vehicles exist"
    );

    client.close().expect("close SUMO connection");
    let status = child.wait().expect("wait for sumo");
    assert!(status.success(), "sumo should exit cleanly after close");
    let _ = std::fs::remove_file(&stderr_log);
}
