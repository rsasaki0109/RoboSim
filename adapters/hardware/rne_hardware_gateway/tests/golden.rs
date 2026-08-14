use rne_ai::{diff_drive_goal_task_spec, DiffDriveRewardConfig};
use rne_hardware_gateway::{GatewayConfig, HardwareGateway, HardwareMode};
use std::fs;
use std::path::Path;

#[test]
fn fail_closed_session_matches_golden() {
    let task = diff_drive_goal_task_spec(180, DiffDriveRewardConfig::default());
    let mut gateway = HardwareGateway::new(
        task,
        GatewayConfig {
            mode: HardwareMode::Live,
            max_observation_age_ms: 50,
            command_deadline_ms: 10,
            max_command_age_ms: 20,
            observation_capacity: 2,
            actuation_capacity: 2,
            event_capacity: 32,
        },
    )
    .unwrap();
    gateway.connect(0).unwrap();
    gateway
        .ingest_observation(0, 0, vec![0.0; gateway.observation_width()])
        .unwrap();
    gateway.arm(0).unwrap();
    assert!(gateway.submit_action(1, 0, 0, vec![100.0, 0.0]).is_err());
    assert!(gateway.poll_actuation(1).unwrap().unwrap().safety_stop);

    let actual = format!(
        "{}\n",
        serde_json::to_string_pretty(&gateway.take_evidence()).unwrap()
    );
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..");
    let expected =
        fs::read_to_string(root.join("tests/golden/hardware/gateway-fail-closed-session-v1.json"))
            .unwrap();
    assert_eq!(actual, expected);
}
