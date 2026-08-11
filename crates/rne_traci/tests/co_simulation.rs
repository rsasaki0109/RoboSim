//! Co-simulation bridge tests against a stateful mock TraCI server.

use rne_ecs::World;
use rne_traci::{CoSimulation, CoSimulationSessionState, ReconnectPolicy};
use rne_traffic::{TrafficActor, TrafficPose, TrafficPoseSource};
use std::io::{BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
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

fn error_status_bytes(command_id: u8, description: &str) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(0xff);
    payload.extend_from_slice(&string_bytes(description));
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

fn read_request(reader: &mut BufReader<std::net::TcpStream>) -> Option<Vec<u8>> {
    let mut length_bytes = [0_u8; 4];
    reader.read_exact(&mut length_bytes).ok()?;
    let length = u32::from_be_bytes(length_bytes) as usize;
    let mut body = vec![0_u8; length.saturating_sub(4)];
    reader.read_exact(&mut body).ok()?;
    Some(body)
}

fn write_message(stream: &mut std::net::TcpStream, commands: &[Vec<u8>]) {
    stream
        .write_all(&message(commands))
        .expect("write mock response");
    stream.flush().expect("flush mock response");
}

fn sim_step_response() -> Vec<Vec<u8>> {
    vec![
        status_bytes(0x02),
        command_bytes(0x02, &0_u32.to_be_bytes()),
    ]
}

fn vehicle_ids_response(ids: &[&str]) -> Vec<Vec<u8>> {
    let mut payload = vec![0x00];
    payload.extend_from_slice(&string_bytes(""));
    payload.push(0x0e);
    payload.extend_from_slice(&(ids.len() as u32).to_be_bytes());
    for id in ids {
        payload.extend_from_slice(&string_bytes(id));
    }
    vec![status_bytes(0xa4), command_bytes(0xb4, &payload)]
}

fn vehicle_position_response(id: &str, x_m: f64, y_m: f64) -> Vec<Vec<u8>> {
    let mut payload = vec![0x42];
    payload.extend_from_slice(&string_bytes(id));
    payload.push(0x01);
    payload.extend_from_slice(&x_m.to_be_bytes());
    payload.extend_from_slice(&y_m.to_be_bytes());
    vec![status_bytes(0xa4), command_bytes(0xb4, &payload)]
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

/// Serves `v0` on the first step, then returns `v0` and `v1` while failing to
/// provide `v1`'s position. The successful `v0` read must not be mirrored.
fn start_transactional_failure_mock() -> u16 {
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
                    let variable = body[2];
                    if variable == 0x00 {
                        responses.push(status_bytes(0xa4));
                        let ids: &[&str] = if sim_step == 1 {
                            &["v0"]
                        } else {
                            &["v0", "v1"]
                        };
                        let mut payload = Vec::new();
                        payload.push(variable);
                        payload.extend_from_slice(&string_bytes(""));
                        payload.push(0x0e);
                        payload.extend_from_slice(&(ids.len() as u32).to_be_bytes());
                        for id in ids {
                            payload.extend_from_slice(&string_bytes(id));
                        }
                        responses.push(command_bytes(0xb4, &payload));
                    } else if variable == 0x42 {
                        let id_length =
                            u32::from_be_bytes(body[3..7].try_into().expect("vehicle id length"))
                                as usize;
                        let id =
                            std::str::from_utf8(&body[7..7 + id_length]).expect("vehicle id utf-8");
                        if sim_step >= 2 && id == "v1" {
                            responses.push(error_status_bytes(0xa4, "position unavailable"));
                        } else {
                            responses.push(status_bytes(0xa4));
                            let (x, y): (f64, f64) = if sim_step == 1 {
                                (1.0, -2.0)
                            } else {
                                (9.0, -8.0)
                            };
                            let mut payload = Vec::new();
                            payload.push(variable);
                            payload.extend_from_slice(&string_bytes(id));
                            payload.push(0x01);
                            payload.extend_from_slice(&x.to_be_bytes());
                            payload.extend_from_slice(&y.to_be_bytes());
                            responses.push(command_bytes(0xb4, &payload));
                        }
                    }
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

/// Drops a connection after receiving an ambiguous second step, then accepts a
/// replacement client whose first command must be a snapshot query.
fn start_recoverable_mock() -> (u16, Arc<AtomicUsize>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind recovery listener");
    let port = listener.local_addr().expect("local address").port();
    let simulation_steps = Arc::new(AtomicUsize::new(0));
    let thread_steps = Arc::clone(&simulation_steps);
    thread::spawn(move || {
        let (mut first, _) = listener.accept().expect("accept initial client");
        let mut first_reader = BufReader::new(first.try_clone().expect("clone initial client"));

        let request = read_request(&mut first_reader).expect("initial simulation step");
        assert_eq!(request[1], 0x02);
        thread_steps.fetch_add(1, Ordering::SeqCst);
        write_message(&mut first, &sim_step_response());

        let request = read_request(&mut first_reader).expect("initial id query");
        assert_eq!((request[1], request[2]), (0xa4, 0x00));
        write_message(&mut first, &vehicle_ids_response(&["v0"]));
        let request = read_request(&mut first_reader).expect("initial position query");
        assert_eq!((request[1], request[2]), (0xa4, 0x42));
        write_message(&mut first, &vehicle_position_response("v0", 1.0, -2.0));

        let request = read_request(&mut first_reader).expect("ambiguous second step");
        assert_eq!(request[1], 0x02);
        thread_steps.fetch_add(1, Ordering::SeqCst);
        drop(first_reader);
        drop(first);

        let (mut replacement, _) = listener.accept().expect("accept replacement client");
        let mut replacement_reader =
            BufReader::new(replacement.try_clone().expect("clone replacement client"));

        let request = read_request(&mut replacement_reader).expect("recovery id query");
        assert_eq!(
            (request[1], request[2]),
            (0xa4, 0x00),
            "recovery must resynchronize before sending another simulation step"
        );
        write_message(&mut replacement, &vehicle_ids_response(&["v0", "v1"]));
        for (id, x_m, y_m) in [("v0", 2.0, -3.0), ("v1", 4.0, -5.0)] {
            let request = read_request(&mut replacement_reader).expect("recovery position query");
            assert_eq!((request[1], request[2]), (0xa4, 0x42));
            write_message(&mut replacement, &vehicle_position_response(id, x_m, y_m));
        }

        let request = read_request(&mut replacement_reader).expect("post-recovery step");
        assert_eq!(request[1], 0x02);
        thread_steps.fetch_add(1, Ordering::SeqCst);
        write_message(&mut replacement, &sim_step_response());
        let request = read_request(&mut replacement_reader).expect("post-recovery id query");
        assert_eq!((request[1], request[2]), (0xa4, 0x00));
        write_message(&mut replacement, &vehicle_ids_response(&["v0", "v1"]));
        for (id, x_m, y_m) in [("v0", 3.0, -4.0), ("v1", 5.0, -6.0)] {
            let _request =
                read_request(&mut replacement_reader).expect("post-recovery position query");
            write_message(&mut replacement, &vehicle_position_response(id, x_m, y_m));
        }
    });
    (port, simulation_steps)
}

#[test]
fn mirrors_vehicle_create_update_and_remove() {
    let port = start_stateful_mock();
    let mut world = World::new();
    let mut co_sim = CoSimulation::connect("127.0.0.1", port).expect("connect");
    assert!(co_sim.actors().is_empty());

    co_sim
        .set_vehicle_speed_m_s("v0", 4.0)
        .expect("explicit SUMO speed command");

    co_sim.step(&mut world).expect("first step");
    assert_eq!(co_sim.actors().len(), 1, "the vehicle must be mirrored");
    let entity = co_sim.actors()["v0"];
    assert!(world.get::<TrafficActor>(entity).is_some());
    assert_eq!(
        world.get::<TrafficPoseSource>(entity),
        Some(&TrafficPoseSource::External)
    );
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
    assert_eq!(co_sim.metrics().successful_steps, 3);
    co_sim.close().expect("close session");
    assert_eq!(co_sim.state(), CoSimulationSessionState::Closed);
    assert!(
        co_sim.step(&mut world).is_err(),
        "closed sessions reject steps"
    );
}

#[test]
fn position_failure_does_not_partially_update_the_mirror() {
    let port = start_transactional_failure_mock();
    let mut world = World::new();
    let mut co_sim = CoSimulation::connect("127.0.0.1", port).expect("connect");

    co_sim.step(&mut world).expect("initial step");
    let entity = co_sim.actors()["v0"];
    let pose_before = *world.get::<TrafficPose>(entity).expect("initial pose");
    let actors_before = co_sim.actors().clone();

    let error = co_sim
        .step(&mut world)
        .expect_err("the second vehicle position must fail");
    assert!(
        error.to_string().contains("position unavailable"),
        "unexpected error: {error}"
    );
    assert_eq!(co_sim.actors(), &actors_before);
    assert_eq!(
        world.get::<TrafficPose>(entity),
        Some(&pose_before),
        "a successful earlier position read must not update ECS"
    );
    assert_eq!(
        world.query::<&TrafficPose>().iter(&world).count(),
        1,
        "a new actor must not be spawned before every position read succeeds"
    );
    assert_eq!(
        world.get::<TrafficPoseSource>(entity),
        Some(&TrafficPoseSource::External)
    );
}

#[test]
fn reconnect_resynchronizes_without_resending_the_ambiguous_step() {
    let (port, simulation_steps) = start_recoverable_mock();
    let mut world = World::new();
    let mut co_sim = CoSimulation::connect("127.0.0.1", port).expect("connect");

    co_sim.step(&mut world).expect("initial step");
    let original_v0 = co_sim.actors()["v0"];
    let pose_before = *world.get::<TrafficPose>(original_v0).expect("initial pose");
    co_sim
        .step(&mut world)
        .expect_err("the mock drops the ambiguous second step response");
    assert_eq!(co_sim.state(), CoSimulationSessionState::Disconnected);
    assert_eq!(co_sim.actors()["v0"], original_v0);
    assert_eq!(world.get::<TrafficPose>(original_v0), Some(&pose_before));
    assert_eq!(simulation_steps.load(Ordering::SeqCst), 2);

    let recovery = co_sim
        .recover(
            &mut world,
            ReconnectPolicy::new(1).expect("one bounded attempt"),
        )
        .expect("recover session");
    assert_eq!(recovery.generation, 2);
    assert_eq!(recovery.attempts, 1);
    assert_eq!(recovery.created_actor_count, 1);
    assert_eq!(recovery.updated_actor_count, 1);
    assert_eq!(recovery.removed_actor_count, 0);
    assert_eq!(co_sim.state(), CoSimulationSessionState::Connected);
    assert_eq!(co_sim.actors()["v0"], original_v0);
    assert_eq!(co_sim.actors().len(), 2);
    assert_eq!(
        world
            .get::<TrafficPose>(original_v0)
            .expect("recovered v0")
            .position_m,
        [2.0, 0.0, 3.0]
    );
    assert_eq!(
        simulation_steps.load(Ordering::SeqCst),
        2,
        "snapshot recovery must not issue simulationStep"
    );

    let metrics = co_sim.metrics();
    assert_eq!(metrics.successful_steps, 1);
    assert_eq!(metrics.failed_steps, 1);
    assert_eq!(metrics.reconnect_attempts, 1);
    assert_eq!(metrics.successful_recoveries, 1);
    assert_eq!(metrics.generation, 2);

    co_sim.step(&mut world).expect("post-recovery step");
    assert_eq!(simulation_steps.load(Ordering::SeqCst), 3);
    assert_eq!(co_sim.metrics().successful_steps, 2);
}
