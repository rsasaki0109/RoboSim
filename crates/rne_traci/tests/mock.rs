//! Round-trip tests against an in-process mock TraCI server.

use rne_traci::{TraciClient, TraciError};
use std::io::{BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::thread;

fn command_bytes(command_id: u8, payload: &[u8]) -> Vec<u8> {
    let mut command = Vec::with_capacity(1 + 1 + payload.len());
    command.push((1 + payload.len()) as u8);
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

fn string_bytes(value: &str) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(4 + value.len());
    bytes.extend_from_slice(&(value.len() as u32).to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    bytes
}

fn status_bytes(command_id: u8, result: u8, description: &str) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.push(result);
    payload.extend_from_slice(&string_bytes(description));
    let mut command = Vec::with_capacity(1 + 1 + payload.len());
    command.push((1 + payload.len()) as u8);
    command.push(command_id);
    command.extend_from_slice(&payload);
    command
}

/// Spawns a mock TraCI server that answers one connection, returning its port.
fn start_mock(respond: impl Fn(&mut MockReply) + Send + 'static) -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind mock listener");
    let port = listener.local_addr().expect("local address").port();
    thread::spawn(move || {
        let (stream, _) = listener.accept().expect("accept mock client");
        mock_loop(stream, respond);
    });
    port
}

/// Builds mock TraCI response commands.
struct MockReply {
    responses: Vec<Vec<u8>>,
}

impl MockReply {
    fn new() -> Self {
        Self {
            responses: Vec::new(),
        }
    }

    fn status(&mut self, command_id: u8) {
        let command = status_bytes(command_id, 0x00, "");
        self.responses.push(command);
    }

    fn version(&mut self, api_version: u32, name: &str) {
        let mut payload = Vec::new();
        payload.extend_from_slice(&api_version.to_be_bytes());
        payload.extend_from_slice(&string_bytes(name));
        self.responses.push(command_bytes(0x00, &payload));
    }

    fn subscription_count(&mut self) {
        self.responses
            .push(command_bytes(0x02, &0_u32.to_be_bytes()));
    }

    fn string_list(&mut self, variable: u8, object_id: &str, values: &[&str]) {
        let mut payload = Vec::new();
        payload.push(variable);
        payload.extend_from_slice(&string_bytes(object_id));
        payload.push(0x0e);
        payload.extend_from_slice(&(values.len() as u32).to_be_bytes());
        for value in values {
            payload.extend_from_slice(&string_bytes(value));
        }
        self.responses.push(command_bytes(0xb4, &payload));
    }

    fn position_2d(&mut self, variable: u8, object_id: &str, x: f64, y: f64) {
        let mut payload = Vec::new();
        payload.push(variable);
        payload.extend_from_slice(&string_bytes(object_id));
        payload.push(0x01);
        payload.extend_from_slice(&x.to_be_bytes());
        payload.extend_from_slice(&y.to_be_bytes());
        self.responses.push(command_bytes(0xb4, &payload));
    }
}

fn read_request(reader: &mut BufReader<TcpStream>) -> Option<Vec<u8>> {
    let mut length_bytes = [0_u8; 4];
    reader.read_exact(&mut length_bytes).ok()?;
    let length = u32::from_be_bytes(length_bytes) as usize;
    let mut body = vec![0_u8; length.saturating_sub(4)];
    reader.read_exact(&mut body).ok()?;
    Some(body)
}

fn request_command_id(body: &[u8]) -> u8 {
    // Requests from this client always use the 1-byte command length form.
    body[1]
}

fn mock_loop(mut stream: TcpStream, respond: impl Fn(&mut MockReply)) {
    let mut reader = BufReader::new(stream.try_clone().expect("clone stream"));
    loop {
        let Some(body) = read_request(&mut reader) else {
            return;
        };
        let mut reply = MockReply::new();
        match request_command_id(&body) {
            0x00 => {
                reply.status(0x00);
                reply.version(22, "SUMO test v1_0");
            }
            0x02 => {
                reply.status(0x02);
                reply.subscription_count();
            }
            0xa4 => {
                reply.status(0xa4);
                respond(&mut reply);
            }
            0x7f => {
                reply.status(0x7f);
                let bytes = message(&reply.responses);
                let _ = stream.write_all(&bytes);
                let _ = stream.flush();
                return;
            }
            other => {
                reply.status(other);
            }
        }
        let bytes = message(&reply.responses);
        if stream.write_all(&bytes).is_err() {
            return;
        }
        let _ = stream.flush();
    }
}

fn connect(port: u16) -> TraciClient {
    TraciClient::connect("127.0.0.1", port).expect("connect to mock")
}

#[test]
fn version_round_trip() {
    let port = start_mock(|_| {});
    let mut client = connect(port);
    let (api, name) = client.get_version().expect("get version");
    assert_eq!(api, 22);
    assert_eq!(name, "SUMO test v1_0");
}

#[test]
fn simulation_step_round_trip() {
    let port = start_mock(|_| {});
    let mut client = connect(port);
    client.simulation_step().expect("simulation step");
}

#[test]
fn vehicle_id_list_round_trip() {
    let port = start_mock(|reply| reply.string_list(0x00, "", &["veh_0", "veh_1"]));
    let mut client = connect(port);
    let ids = client.vehicle_ids().expect("vehicle ids");
    assert_eq!(ids, vec!["veh_0".to_string(), "veh_1".to_string()]);
}

#[test]
fn vehicle_position_round_trip() {
    let port = start_mock(|reply| reply.position_2d(0x40, "veh_0", 10.0, -20.0));
    let mut client = connect(port);
    let position = client.vehicle_position("veh_0").expect("position");
    assert_eq!(position, [10.0, -20.0]);
}

#[test]
fn close_round_trip() {
    let port = start_mock(|_| {});
    let mut client = connect(port);
    client.close().expect("close");
}

#[test]
fn command_failure_is_reported() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let port = listener.local_addr().expect("local address").port();
    thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("accept");
        let mut reader = BufReader::new(stream.try_clone().expect("clone"));
        let body = read_request(&mut reader).expect("read request");
        let _ = request_command_id(&body);
        let failed = message(&[status_bytes(0x00, 0xff, "boom")]);
        let _ = stream.write_all(&failed);
    });
    let mut client = connect(port);
    let error = client.get_version().expect_err("must fail");
    assert!(
        matches!(error, TraciError::Command(_)),
        "expected a command error, got {error}"
    );
}
