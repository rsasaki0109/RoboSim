//! Couple SSL simulation-protocol UDP commands into the small-pitch physics plant.
//!
//! Decodes loopback `TeleportBall` / `RobotControl` datagrams with
//! `rne_adapter_ssl`, then applies them to `SslSmallPitchScenario` without
//! pulling protobuf into core crates.

use rne_adapter_ssl::proto::robot_move_command;
use rne_adapter_ssl::{
    bind_ssl_ports, encode_robot_control, encode_simulator_command, parse_robot_control,
    serve_robot_control_once, serve_simulator_control_once, ssl_ball_teleport_to_rne_m,
    ssl_move_to_diff_drive, teleport_ball_command, MoveLocalVelocity, RobotCommand, RobotControl,
    RobotMoveCommand, SslSimulationPorts, SslTeam, SSL_STAND_IN_TRACK_WIDTH_M,
    SSL_STAND_IN_WHEEL_RADIUS_M,
};
use rne_ai::{DiffDriveAction, SslBallRegion, SslSmallPitchScenario};
use std::net::UdpSocket;
use std::thread;
use std::time::Duration;

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    if !smoke {
        eprintln!("usage: 87_ssl_physics_coupling -- --smoke");
        std::process::exit(2);
    }

    let mut scenario = SslSmallPitchScenario::success(1).expect("load ssl pitch");
    let attacker = scenario
        .robot_entity("ssl_blue_0")
        .expect("ssl_blue_0 attacker");

    let bound = bind_ssl_ports(SslSimulationPorts::ephemeral()).expect("bind ssl ports");
    let ports = bound.local_ports().expect("local ports");
    let client = UdpSocket::bind("127.0.0.1:0").expect("client");
    client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .expect("timeout");
    let mut buffer = [0_u8; 2048];

    // Teleport first so the control server does not idle through the drive loop.
    let control_server = thread::spawn({
        let control = bound.control.try_clone().expect("clone control");
        move || serve_simulator_control_once(&control)
    });
    thread::sleep(Duration::from_millis(30));
    let teleport = teleport_ball_command(4.6, 0.0, 0.0215);
    client
        .send_to(
            &encode_simulator_command(&teleport),
            ("127.0.0.1", ports.control),
        )
        .expect("send teleport");
    let _ = client.recv_from(&mut buffer); // may be empty protobuf bytes
    let (command, _) = control_server
        .join()
        .expect("join control")
        .expect("serve control");
    let ball = command
        .control
        .as_ref()
        .and_then(|control| control.teleport_ball.as_ref())
        .expect("teleport ball");
    let translation =
        ssl_ball_teleport_to_rne_m(ball.x.expect("x"), ball.y.expect("y"), ball.z.expect("z"));
    assert!(scenario.teleport_ball_m(translation));
    let scored = scenario.current_observation();
    assert_eq!(scored.ball_region, SslBallRegion::YellowGoal);
    println!(
        "teleport couple: region={:?} ball=({:.3},{:.3},{:.3})",
        scored.ball_region, scored.ball_x_m, scored.ball_y_m, scored.ball_z_m
    );

    // Reset the ball to kickoff and drive via UDP RobotControl.
    assert!(scenario.teleport_ball_m([0.0, 0.0215, 0.0]));
    let blue_server = thread::spawn({
        let blue = bound.blue.try_clone().expect("clone blue");
        move || serve_robot_control_once(&blue, SslTeam::Blue)
    });
    thread::sleep(Duration::from_millis(30));

    let robot_payload = encode_robot_control(&RobotControl {
        robot_commands: vec![RobotCommand {
            id: 0,
            move_command: Some(RobotMoveCommand {
                command: Some(robot_move_command::Command::LocalVelocity(
                    MoveLocalVelocity {
                        forward: 0.5,
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
    let (len, _) = client.recv_from(&mut buffer).expect("robot response");
    assert!(len > 0);
    let (parsed, _) = blue_server.join().expect("join blue").expect("serve blue");
    let command = parsed.commands[0]
        .move_command
        .expect("local velocity command");
    let yaw_rad = scenario.current_observation().attacker_yaw_rad;
    let wheels = ssl_move_to_diff_drive(
        command,
        yaw_rad,
        SSL_STAND_IN_WHEEL_RADIUS_M,
        SSL_STAND_IN_TRACK_WIDTH_M,
    );
    let start_x = scenario.current_observation().attacker_x_m;
    for _ in 0..90 {
        scenario.step_with_actions(&[(
            attacker,
            DiffDriveAction {
                left_velocity_rad_s: wheels.left_rad_s,
                right_velocity_rad_s: wheels.right_rad_s,
            },
        )]);
    }
    let after_drive = scenario.current_observation();
    assert!(
        after_drive.attacker_x_m > start_x + 0.3,
        "UDP local velocity should advance the attacker"
    );
    println!(
        "robot-control couple: start_x={start_x:.3} after_x={:.3}",
        after_drive.attacker_x_m
    );

    let _ = parse_robot_control(SslTeam::Blue, &robot_payload).expect("parse");
    println!("smoke: ok");
}
