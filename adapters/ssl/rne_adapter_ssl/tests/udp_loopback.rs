use prost::Message;
use rne_adapter_ssl::proto::robot_move_command;
use rne_adapter_ssl::{
    bind_ssl_ports, decode_simulator_response, encode_robot_control, encode_simulator_command,
    parse_robot_control, serve_robot_control_once, serve_simulator_control_once,
    teleport_ball_command, MoveLocalVelocity, RobotCommand, RobotControl, RobotControlResponse,
    RobotMoveCommand, SslMoveCommand, SslSimulationPorts, SslTeam, SSL_ROBOT_CONTROL_BLUE_PORT,
    SSL_ROBOT_CONTROL_YELLOW_PORT, SSL_SIM_CONTROL_PORT,
};
use std::net::UdpSocket;
use std::thread;
use std::time::Duration;

fn sample_robot_control() -> RobotControl {
    RobotControl {
        robot_commands: vec![RobotCommand {
            id: 3,
            move_command: Some(RobotMoveCommand {
                command: Some(robot_move_command::Command::LocalVelocity(
                    MoveLocalVelocity {
                        forward: 0.4,
                        left: -0.1,
                        angular: 0.2,
                    },
                )),
            }),
            kick_speed: Some(6.0),
            kick_angle: None,
            dribbler_speed: Some(1_000.0),
        }],
    }
}

#[test]
fn default_ports_match_ssl_simulation_protocol() {
    let ports = SslSimulationPorts::default();
    assert_eq!(ports.control, SSL_SIM_CONTROL_PORT);
    assert_eq!(ports.blue, SSL_ROBOT_CONTROL_BLUE_PORT);
    assert_eq!(ports.yellow, SSL_ROBOT_CONTROL_YELLOW_PORT);
    assert_eq!(SSL_SIM_CONTROL_PORT, 10_300);
    assert_eq!(SSL_ROBOT_CONTROL_BLUE_PORT, 10_301);
    assert_eq!(SSL_ROBOT_CONTROL_YELLOW_PORT, 10_302);
}

#[test]
fn robot_control_round_trips_through_prost() {
    let encoded = encode_robot_control(&sample_robot_control());
    let parsed = parse_robot_control(SslTeam::Blue, &encoded).expect("parse");
    assert_eq!(parsed.team, SslTeam::Blue);
    assert_eq!(parsed.commands.len(), 1);
    assert_eq!(parsed.commands[0].id, 3);
    assert_eq!(
        parsed.commands[0].move_command,
        Some(SslMoveCommand::LocalVelocity {
            forward_m_s: 0.4,
            left_m_s: -0.1,
            angular_rad_s: 0.2,
        })
    );
    assert_eq!(parsed.commands[0].kick_speed_m_s, Some(6.0));
    assert_eq!(parsed.commands[0].dribbler_speed_rpm, Some(1_000.0));
}

#[test]
fn blue_robot_control_udp_loopback() {
    let bound = bind_ssl_ports(SslSimulationPorts::ephemeral()).expect("bind");
    let ports = bound.local_ports().expect("local ports");
    let server = thread::spawn(move || serve_robot_control_once(&bound.blue, SslTeam::Blue));

    thread::sleep(Duration::from_millis(20));
    let client = UdpSocket::bind("127.0.0.1:0").expect("client");
    client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("timeout");
    let payload = encode_robot_control(&sample_robot_control());
    client
        .send_to(&payload, ("127.0.0.1", ports.blue))
        .expect("send");
    let mut buffer = [0_u8; 2048];
    let (len, _) = client.recv_from(&mut buffer).expect("recv response");
    let response = RobotControlResponse::decode(&buffer[..len]).expect("decode");
    assert!(response.errors.is_empty());
    assert_eq!(response.feedback.len(), 1);
    assert_eq!(response.feedback[0].id, 3);

    let (parsed, _) = server.join().expect("join").expect("serve");
    assert_eq!(parsed.commands[0].id, 3);
}

#[test]
fn simulator_control_teleport_ball_udp_loopback() {
    let bound = bind_ssl_ports(SslSimulationPorts::ephemeral()).expect("bind");
    let ports = bound.local_ports().expect("local ports");
    let server = thread::spawn(move || serve_simulator_control_once(&bound.control));

    thread::sleep(Duration::from_millis(20));
    let client = UdpSocket::bind("127.0.0.1:0").expect("client");
    client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("timeout");
    let payload = encode_simulator_command(&teleport_ball_command(1.5, -0.25, 0.0));
    client
        .send_to(&payload, ("127.0.0.1", ports.control))
        .expect("send");
    let mut buffer = [0_u8; 2048];
    let (len, _) = client.recv_from(&mut buffer).expect("recv response");
    let response = decode_simulator_response(&buffer[..len]).expect("decode");
    assert!(response.errors.is_empty());

    let (command, _) = server.join().expect("join").expect("serve");
    let ball = command
        .control
        .as_ref()
        .and_then(|control| control.teleport_ball.as_ref())
        .expect("teleport_ball");
    assert_eq!(ball.x, Some(1.5));
    assert_eq!(ball.y, Some(-0.25));
    assert_eq!(ball.z, Some(0.0));
}
