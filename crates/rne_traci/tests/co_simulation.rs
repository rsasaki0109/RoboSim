//! Co-simulation bridge tests against a stateful mock TraCI server.

use rne_ecs::World;
use rne_traci::CoSimulation;
use rne_traffic::{TrafficActor, TrafficPose};
use std::io::{BufReader, Read, Write};
use std::net::TcpListener;
use std::thread;

fn string_bytes(value: &str) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(4 + value.len());
    bytes.extend_from_slice(&(value.len() as u32).to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    bytes
}

fn status_bytes(command_id: u8) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(0x00);
    payload.extend_from_slice(&string_bytes(""));
    let mut command = Vec::with_capacity(2 + payload.len());
    command.push((2 + payload.len()) as u8);
    command.push(command_id);
    command.extend_from_slice(&payload);
    command
}

fn command_bytes(command_id: u8, payload: &[u8]) -> Vec<u8> {
    let mut command = Vec::with_capacity(2 + payload.len());
    command.push((2 + payload.len()) as u8);
    command.push(command_id);
    command.extend_from_slice(payload);
    command
}

fn message(commands: &[Vec<u8>]) -> Vec<u8> {
    let body_len: usize = commands.iter().map(Vec::len).sum();
    let mut message = Vec::with_capacity(4 + body_len);
    message.extend_from_slice(&((body_len + 4) as u32).to_be_bytes());
    for command in commands {
        message.extend_from_slice(command);
    }
    message
}

/// Serves a scripted vehicle: `v0` appears at (10,-20), moves to (20,-30),
/// then leaves after two simulation steps.
fn start_stateful_mock() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind mock listener");
    let port = listener.local_addr().expect("local address").port();
    thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("accept mock client");
        let mut reader = BufReader::new(stream.try_clone().expect("clone stream"));
        let mut sim_step = 0_usize;
        loop {
            let mut length_bytes = [0_u8; 4];
            if reader.read_exact(&mut length_bytes).is_err() {
                return;
            }
            let length = u32::from_be_bytes(length_bytes) as usize;
            let mut body = vec![0_u8; length.saturating_sub(4)];
            if reader.read_exact(&mut body).is_err() {
                return;
            }
            let mut responses = Vec::new();
            match body[1] {
                0x02 => {
                    responses.push(status_bytes(0x02));
                    responses.push(command_bytes(0x02, &0_u32.to_be_bytes()));
                    sim_step += 1;
                }
                0xa4 => {
                    responses.push(status_bytes(0xa4));
                    let mut payload = Vec::new();
                    payload.push(body[2]);
                    payload.extend_from_slice(&string_bytes(""));
                    match body[2] {
                        0x00 => {
                            let ids: &[&str] = if sim_step >= 3 { &[] } else { &["v0"] };
                            payload.push(0x0e);
                            payload.extend_from_slice(&(ids.len() as u32).to_be_bytes());
                            for id in ids {
                                payload.extend_from_slice(&string_bytes(id));
                            }
                        }
                        0x42 => {
                            let (x, y): (f64, f64) = if sim_step == 1 {
                                (10.0, -20.0)
                            } else {
                                (20.0, -30.0)
                            };
                            payload.push(0x01);
                            payload.extend_from_slice(&x.to_be_bytes());
                            payload.extend_from_slice(&y.to_be_bytes());
                        }
                        _ => {}
                    }
                    responses.push(command_bytes(0xb4, &payload));
                }
                other => responses.push(status_bytes(other)),
            }
            let bytes = message(&responses);
            if stream.write_all(&bytes).is_err() {
                return;
            }
            let _ = stream.flush();
        }
    });
    port
}

#[test]
fn mirrors_vehicle_create_update_and_remove() {
    let port = start_stateful_mock();
    let mut world = World::new();
    let mut co_sim = CoSimulation::connect("127.0.0.1", port).expect("connect");
    assert!(co_sim.actors().is_empty());

    co_sim.step(&mut world).expect("first step");
    assert_eq!(co_sim.actors().len(), 1, "the vehicle must be mirrored");
    let entity = co_sim.actors()["v0"];
    assert!(world.get::<TrafficActor>(entity).is_some());
    let pose = world.get::<TrafficPose>(entity).expect("pose");
    assert_eq!(pose.position_m, [10.0, 0.0, 20.0]);

    co_sim.step(&mut world).expect("second step");
    let pose = world.get::<TrafficPose>(entity).expect("pose");
    assert_eq!(
        pose.position_m,
        [20.0, 0.0, 30.0],
        "the mirror must track the SUMO vehicle's motion"
    );

    co_sim.step(&mut world).expect("third step");
    assert!(
        co_sim.actors().is_empty(),
        "a departed SUMO vehicle must be despawned"
    );
}
