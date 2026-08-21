//! Smoke test for the SSL simulation-protocol UDP adapter spike.
//!
//! Binds ephemeral ports, round-trips `RobotControl` on the blue socket and
//! `SimulatorCommand` teleport-ball on the control socket. This does not run
//! physics or speak ssl-vision.

use prost::Message;
use rne_adapter_ssl::proto::robot_move_command;
use rne_adapter_ssl::{
    bind_ssl_ports, decode_simulator_response, encode_robot_control, encode_simulator_command,
    serve_robot_control_once, serve_simulator_control_once, teleport_ball_command,
    MoveLocalVelocity, RobotCommand, RobotControl, RobotControlResponse, RobotMoveCommand,
    SslSimulationPorts, SslTeam, SSL_ROBOT_CONTROL_BLUE_PORT, SSL_ROBOT_CONTROL_YELLOW_PORT,
    SSL_SIM_CONTROL_PORT,
};
use std::net::UdpSocket;
use std::thread;
use std::time::Duration;

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    if !smoke {
        eprintln!("usage: 80_ssl_adapter_smoke -- --smoke");
        std::process::exit(2);
    }

    assert_eq!(SSL_SIM_CONTROL_PORT, 10_300);
    assert_eq!(SSL_ROBOT_CONTROL_BLUE_PORT, 10_301);
    assert_eq!(SSL_ROBOT_CONTROL_YELLOW_PORT, 10_302);

    let bound = bind_ssl_ports(SslSimulationPorts::ephemeral()).expect("bind ssl ports");
    let ports = bound.local_ports().expect("local ports");
    println!(
        "bound control={} blue={} yellow={}",
        ports.control, ports.blue, ports.yellow
    );

    let blue_server = thread::spawn({
        let blue = bound.blue.try_clone().expect("clone blue");
        move || serve_robot_control_once(&blue, SslTeam::Blue)
    });
    let control_server = thread::spawn({
        let control = bound.control.try_clone().expect("clone control");
        move || serve_simulator_control_once(&control)
    });
    thread::sleep(Duration::from_millis(30));

    let client = UdpSocket::bind("127.0.0.1:0").expect("client");
    client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("timeout");

    let robot_payload = encode_robot_control(&RobotControl {
        robot_commands: vec![RobotCommand {
            id: 0,
            move_command: Some(RobotMoveCommand {
                command: Some(robot_move_command::Command::LocalVelocity(
                    MoveLocalVelocity {
                        forward: 1.0,
                        left: 0.0,
                        angular: 0.0,
                    },
                )),
            }),
            kick_speed: None,
            kick_angle: None,
            dribbler_speed: None,
        }],
    });
    client
        .send_to(&robot_payload, ("127.0.0.1", ports.blue))
        .expect("send robot control");
    let mut buffer = [0_u8; 2048];
    let (len, _) = client.recv_from(&mut buffer).expect("robot response");
    let robot_response = RobotControlResponse::decode(&buffer[..len]).expect("decode robot");
    assert!(robot_response.errors.is_empty());
    assert_eq!(robot_response.feedback[0].id, 0);
    let (parsed, _) = blue_server.join().expect("join blue").expect("serve blue");
    println!(
        "robot-control: team={:?} id={} cmds={}",
        parsed.team,
        parsed.commands[0].id,
        parsed.commands.len()
    );

    let sim_payload = encode_simulator_command(&teleport_ball_command(0.0, 0.0, 0.0));
    client
        .send_to(&sim_payload, ("127.0.0.1", ports.control))
        .expect("send sim control");
    let (len, _) = client.recv_from(&mut buffer).expect("sim response");
    let sim_response = decode_simulator_response(&buffer[..len]).expect("decode sim");
    assert!(sim_response.errors.is_empty());
    let (command, _) = control_server
        .join()
        .expect("join control")
        .expect("serve control");
    let ball = command
        .control
        .as_ref()
        .and_then(|control| control.teleport_ball.as_ref())
        .expect("teleport_ball");
    println!(
        "sim-control: teleport_ball=({:?}, {:?}, {:?})",
        ball.x, ball.y, ball.z
    );
    println!("smoke: ok");
}
