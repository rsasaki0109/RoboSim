use rne_ai::TaskSpec;
use rne_hardware_gateway::mock::{
    MockConformanceCase, MockConformanceCaseResult, MockConformanceReport,
};
use rne_hardware_gateway::wire::{
    DeviceWireFrame, DeviceWirePayload, HardwareWireCodec, HostWireFrame, HostWirePayload,
    WireDisconnectReason,
};
use rne_hardware_gateway::{
    ActuationFrame, CommandDisposition, GatewayConfig, GatewayConnectionState, GatewayError,
    HardwareGateway, HardwareMode, SafetyReason,
};
use std::io::{BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

const TASK_JSON: &str = include_str!("../../../../assets/tasks/diff_drive_goal.task.json");
const GOLDEN: &str =
    include_str!("../../../../tests/golden/hardware/gateway-mock-conformance-v1.json");

#[test]
fn process_mock_fault_matrix_matches_golden() {
    let task: TaskSpec = serde_json::from_str(TASK_JSON).expect("task json");
    let cases = vec![
        command_deadline_case(task.clone()),
        disconnect_case(task.clone()),
        reconnect_case(task.clone()),
        command_stale_case(task.clone()),
        actuator_limit_case(task.clone()),
        emergency_stop_case(task),
    ];
    let report = MockConformanceReport::new(cases).expect("conformance report");
    report.validate().expect("validate report");
    let actual = format!(
        "{}\n",
        serde_json::to_string_pretty(&report).expect("serialize report")
    );
    assert_eq!(actual, GOLDEN);
}

fn command_deadline_case(task: TaskSpec) -> MockConformanceCaseResult {
    let mut gateway = gateway(task);
    let mut process = MockProcess::spawn("deadline", &[]);
    process.open(&gateway);
    gateway.connect(0).unwrap();
    let observation = process.poll_observation();
    gateway
        .ingest_observation(1, observation.0, observation.1)
        .unwrap();
    gateway.arm(2).unwrap();
    assert!(matches!(
        gateway.submit_action(12, 1, 1, vec![0.1, -0.1]),
        Err(GatewayError::CommandDeadlineMissed { .. })
    ));
    let stop = take_stop(&mut gateway, 12, SafetyReason::CommandDeadlineMissed);
    let device_stop_confirmed = process.apply_stop(stop);
    process.close();
    case_result(
        MockConformanceCase::CommandDeadline,
        Some(SafetyReason::CommandDeadlineMissed),
        device_stop_confirmed,
        true,
        false,
    )
}

fn disconnect_case(task: TaskSpec) -> MockConformanceCaseResult {
    let mut gateway = gateway(task);
    let mut process = MockProcess::spawn("disconnect", &["--disconnect-after-actuations", "1"]);
    process.open(&gateway);
    gateway.connect(0).unwrap();
    let observation = process.poll_observation();
    gateway
        .ingest_observation(1, observation.0, observation.1)
        .unwrap();
    gateway.arm(2).unwrap();
    gateway.submit_action(3, 1, 1, vec![0.1, -0.1]).unwrap();
    let command = gateway.poll_actuation(3).unwrap().unwrap();
    let response = process.exchange(HostWirePayload::Actuate { frame: command });
    let device_stop_confirmed = matches!(
        response.payload,
        DeviceWirePayload::Disconnected {
            reason: WireDisconnectReason::InjectedFault,
            safe_stop_applied: true,
        }
    );
    process.wait();
    gateway.disconnect(4).unwrap();
    let _stop = take_stop(&mut gateway, 4, SafetyReason::Disconnected);
    case_result(
        MockConformanceCase::Disconnect,
        Some(SafetyReason::Disconnected),
        device_stop_confirmed,
        true,
        false,
    )
}

fn reconnect_case(task: TaskSpec) -> MockConformanceCaseResult {
    let mut gateway = gateway(task);
    let mut first = MockProcess::spawn("reconnect-first", &["--disconnect-after-actuations", "1"]);
    first.open(&gateway);
    gateway.connect(0).unwrap();
    let observation = first.poll_observation();
    gateway
        .ingest_observation(1, observation.0, observation.1)
        .unwrap();
    gateway.arm(2).unwrap();
    gateway.submit_action(3, 1, 1, vec![0.1, -0.1]).unwrap();
    let command = gateway.poll_actuation(3).unwrap().unwrap();
    assert!(matches!(
        first
            .exchange(HostWirePayload::Actuate { frame: command })
            .payload,
        DeviceWirePayload::Disconnected {
            safe_stop_applied: true,
            ..
        }
    ));
    first.wait();
    gateway.disconnect(4).unwrap();
    let _ = take_stop(&mut gateway, 4, SafetyReason::Disconnected);

    let mut second = MockProcess::spawn("reconnect-second", &[]);
    second.open(&gateway);
    gateway.connect(5).unwrap();
    gateway.clear_safety_latch(6).unwrap();
    let observation = second.poll_observation();
    gateway
        .ingest_observation(7, observation.0, observation.1)
        .unwrap();
    gateway.arm(8).unwrap();
    let reconnect_rearmed = gateway.connection_state() == GatewayConnectionState::Armed
        && gateway.snapshot().safety_latch.is_none();
    assert_eq!(
        gateway
            .submit_action(9, 1, observation.0, vec![0.2, -0.2])
            .unwrap(),
        CommandDisposition::Queued
    );
    let command = gateway.poll_actuation(9).unwrap().unwrap();
    assert!(matches!(
        second
            .exchange(HostWirePayload::Actuate { frame: command })
            .payload,
        DeviceWirePayload::ActuationAccepted {
            action_sequence: Some(1),
            safety_stop: false,
        }
    ));
    gateway.disarm(10).unwrap();
    let stop = gateway.poll_actuation(10).unwrap().unwrap();
    let gateway_stop_delivered =
        stop.safety_stop && stop.reason == Some(SafetyReason::ManualDisarm);
    let device_stop_confirmed = second.apply_stop(stop);
    second.close();
    case_result(
        MockConformanceCase::Reconnect,
        None,
        device_stop_confirmed,
        gateway_stop_delivered,
        reconnect_rearmed,
    )
}

fn command_stale_case(task: TaskSpec) -> MockConformanceCaseResult {
    let mut gateway = gateway(task);
    let mut process = MockProcess::spawn("stale", &[]);
    process.open(&gateway);
    gateway.connect(0).unwrap();
    let observation = process.poll_observation();
    gateway
        .ingest_observation(1, observation.0, observation.1)
        .unwrap();
    gateway.arm(2).unwrap();
    gateway
        .submit_action(3, 1, observation.0, vec![0.1, -0.1])
        .unwrap();
    let stop = take_stop(&mut gateway, 24, SafetyReason::CommandStale);
    let device_stop_confirmed = process.apply_stop(stop);
    process.close();
    case_result(
        MockConformanceCase::CommandStale,
        Some(SafetyReason::CommandStale),
        device_stop_confirmed,
        true,
        false,
    )
}

fn actuator_limit_case(task: TaskSpec) -> MockConformanceCaseResult {
    let mut gateway = gateway(task);
    let mut process = MockProcess::spawn("limit", &[]);
    process.open(&gateway);
    gateway.connect(0).unwrap();
    let observation = process.poll_observation();
    gateway
        .ingest_observation(1, observation.0, observation.1)
        .unwrap();
    gateway.arm(2).unwrap();
    assert!(matches!(
        gateway.submit_action(3, 1, observation.0, vec![11.0, 0.0]),
        Err(GatewayError::ActuatorLimit { .. })
    ));
    let stop = take_stop(&mut gateway, 3, SafetyReason::ActuatorLimit);
    let device_stop_confirmed = process.apply_stop(stop);
    process.close();
    case_result(
        MockConformanceCase::ActuatorLimit,
        Some(SafetyReason::ActuatorLimit),
        device_stop_confirmed,
        true,
        false,
    )
}

fn emergency_stop_case(task: TaskSpec) -> MockConformanceCaseResult {
    let mut gateway = gateway(task);
    let mut process = MockProcess::spawn(
        "emergency-stop",
        &["--emergency-stop-after-observations", "1"],
    );
    process.open(&gateway);
    gateway.connect(0).unwrap();
    let response = process.exchange(HostWirePayload::PollObservation);
    let device_stop_confirmed = matches!(
        response.payload,
        DeviceWirePayload::SafetySignal {
            reason: SafetyReason::EmergencyStop,
            safe_stop_applied: true,
        }
    );
    process.wait();
    gateway.emergency_stop(1).unwrap();
    let _stop = take_stop(&mut gateway, 1, SafetyReason::EmergencyStop);
    case_result(
        MockConformanceCase::EmergencyStop,
        Some(SafetyReason::EmergencyStop),
        device_stop_confirmed,
        true,
        false,
    )
}

fn gateway(task: TaskSpec) -> HardwareGateway {
    HardwareGateway::new(
        task,
        GatewayConfig {
            mode: HardwareMode::Hil,
            max_observation_age_ms: 100,
            command_deadline_ms: 10,
            max_command_age_ms: 20,
            observation_capacity: 2,
            actuation_capacity: 2,
            event_capacity: 64,
        },
    )
    .unwrap()
}

fn take_stop(gateway: &mut HardwareGateway, now_ms: u64, reason: SafetyReason) -> ActuationFrame {
    let stop = gateway
        .poll_actuation(now_ms)
        .unwrap()
        .expect("gateway safety stop");
    assert!(stop.safety_stop);
    assert_eq!(stop.reason, Some(reason));
    assert_eq!(gateway.snapshot().safety_latch, Some(reason));
    stop
}

fn case_result(
    case: MockConformanceCase,
    gateway_reason: Option<SafetyReason>,
    device_stop_confirmed: bool,
    gateway_stop_delivered: bool,
    reconnect_rearmed: bool,
) -> MockConformanceCaseResult {
    MockConformanceCaseResult {
        case,
        gateway_reason,
        device_stop_confirmed,
        gateway_stop_delivered,
        reconnect_rearmed,
        passed: device_stop_confirmed && gateway_stop_delivered,
    }
}

struct MockProcess {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
    codec: HardwareWireCodec,
    session_id: String,
    next_sequence: u64,
}

impl MockProcess {
    fn spawn(session_suffix: &str, fault_args: &[&str]) -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_rne-hardware-mock-device"))
            .args(["--device-id", "rne-mock-conformance-v1"])
            .args(fault_args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .expect("spawn mock device");
        Self {
            stdin: child.stdin.take().unwrap(),
            stdout: BufReader::new(child.stdout.take().unwrap()),
            child,
            codec: HardwareWireCodec::default(),
            session_id: format!("rne.mock.conformance.{session_suffix}"),
            next_sequence: 1,
        }
    }

    fn open(&mut self, gateway: &HardwareGateway) {
        let response = self.exchange(HostWirePayload::Open {
            task_id: gateway.task_spec().task_id.clone(),
            mode: HardwareMode::Hil,
            observation_width: gateway.observation_width(),
            action_width: gateway.action_width(),
        });
        assert!(matches!(response.payload, DeviceWirePayload::Ready { .. }));
    }

    fn poll_observation(&mut self) -> (u64, Vec<f64>) {
        let response = self.exchange(HostWirePayload::PollObservation);
        let DeviceWirePayload::Observation { sequence, values } = response.payload else {
            panic!("mock did not return observation");
        };
        (sequence, values)
    }

    fn apply_stop(&mut self, frame: ActuationFrame) -> bool {
        matches!(
            self.exchange(HostWirePayload::Actuate { frame }).payload,
            DeviceWirePayload::ActuationAccepted {
                action_sequence: None,
                safety_stop: true,
            }
        )
    }

    fn exchange(&mut self, payload: HostWirePayload) -> DeviceWireFrame {
        let request = HostWireFrame::new(&self.session_id, self.next_sequence, payload);
        self.next_sequence += 1;
        self.stdin
            .write_all(&self.codec.encode_host(&request).unwrap())
            .unwrap();
        self.stdin.flush().unwrap();
        let line = self.codec.read_line(&mut self.stdout).unwrap().unwrap();
        let response = self.codec.decode_device(&line).unwrap();
        assert_eq!(response.request_sequence, request.sequence);
        response
    }

    fn close(&mut self) {
        assert!(matches!(
            self.exchange(HostWirePayload::Close).payload,
            DeviceWirePayload::Closed
        ));
        self.wait();
    }

    fn wait(&mut self) {
        assert!(self.child.wait().unwrap().success());
    }
}
