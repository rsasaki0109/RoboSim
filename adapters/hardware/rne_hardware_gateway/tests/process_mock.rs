use rne_ai::TaskSpec;
use rne_hardware_gateway::wire::{
    DeviceWireFrame, DeviceWirePayload, HardwareSessionEvidence, HardwareWireCodec,
    HardwareWireTraceEntry, HardwareWireTraceOutcome, HardwareWireTraceRecorder, HostWireFrame,
    HostWirePayload, WireDisconnectReason,
};
use rne_hardware_gateway::{
    CommandDisposition, GatewayConfig, HardwareGateway, HardwareMode, SafetyReason,
};
use std::io::{BufReader, Write};
use std::process::{ChildStdin, ChildStdout, Command, Stdio};

const TASK_JSON: &str = include_str!("../../../../assets/tasks/diff_drive_goal.task.json");
const GOLDEN: &str =
    include_str!("../../../../tests/golden/hardware/gateway-process-disconnect-session-v1.json");

#[test]
fn process_disconnect_session_matches_golden() {
    let task: TaskSpec = serde_json::from_str(TASK_JSON).expect("task json");
    let config = GatewayConfig {
        mode: HardwareMode::Hil,
        max_observation_age_ms: 100,
        command_deadline_ms: 20,
        max_command_age_ms: 100,
        observation_capacity: 2,
        actuation_capacity: 2,
        event_capacity: 32,
    };
    let mut gateway = HardwareGateway::new(task.clone(), config).expect("gateway");
    let codec = HardwareWireCodec::default();
    let session_id = "rne.mock.disconnect.v1";
    let task_id = gateway.task_spec().task_id.clone();
    let mut recorder =
        HardwareWireTraceRecorder::new(session_id, &task_id, 6).expect("trace recorder");

    let mut child = Command::new(env!("CARGO_BIN_EXE_rne-hardware-mock-device"))
        .args(["--device-id", "rne-mock-process-v1"])
        .args(["--disconnect-after-actuations", "1"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("spawn mock process");
    let mut stdin = child.stdin.take().expect("mock stdin");
    let mut stdout = BufReader::new(child.stdout.take().expect("mock stdout"));

    let ready = exchange(
        &codec,
        &mut recorder,
        &mut stdin,
        &mut stdout,
        HostWireFrame::new(
            session_id,
            1,
            HostWirePayload::Open {
                task_id: task_id.clone(),
                mode: HardwareMode::Hil,
                observation_width: gateway.observation_width(),
                action_width: gateway.action_width(),
            },
        ),
    );
    assert!(matches!(ready.payload, DeviceWirePayload::Ready { .. }));
    gateway.connect(0).expect("connect gateway");

    let observation = exchange(
        &codec,
        &mut recorder,
        &mut stdin,
        &mut stdout,
        HostWireFrame::new(session_id, 2, HostWirePayload::PollObservation),
    );
    let DeviceWirePayload::Observation { sequence, values } = observation.payload else {
        panic!("mock must return an observation");
    };
    gateway
        .ingest_observation(1, sequence, values)
        .expect("ingest observation");
    gateway.arm(2).expect("arm HIL gateway");
    assert_eq!(
        gateway
            .submit_action(3, 1, sequence, vec![0.25, -0.25])
            .expect("submit bounded action"),
        CommandDisposition::Queued
    );
    let actuation = gateway
        .poll_actuation(3)
        .expect("poll actuation")
        .expect("queued actuation");

    let disconnected = exchange(
        &codec,
        &mut recorder,
        &mut stdin,
        &mut stdout,
        HostWireFrame::new(session_id, 3, HostWirePayload::Actuate { frame: actuation }),
    );
    assert_eq!(
        disconnected.payload,
        DeviceWirePayload::Disconnected {
            reason: WireDisconnectReason::InjectedFault,
            safe_stop_applied: true,
        }
    );
    gateway.disconnect(4).expect("disconnect gateway");
    let stop = gateway
        .poll_actuation(4)
        .expect("poll safety stop")
        .expect("queued safety stop");
    assert!(stop.safety_stop);
    assert_eq!(stop.reason, Some(SafetyReason::Disconnected));

    drop(stdin);
    assert!(child.wait().expect("wait for mock").success());
    let trace = recorder
        .finish(HardwareWireTraceOutcome::Disconnected {
            reason: WireDisconnectReason::InjectedFault,
        })
        .expect("complete wire trace");
    let evidence = HardwareSessionEvidence::new(trace, gateway.take_evidence())
        .expect("correlate session evidence");
    evidence
        .validate_against(&task)
        .expect("rebind session to TaskSpec");
    let mut wrong_task = task.clone();
    wrong_task.task_id = "rne.other.task.v1".to_string();
    assert!(evidence.validate_against(&wrong_task).is_err());
    let mut wrong_width = evidence.clone();
    let HardwareWireTraceEntry::Host { frame } = &mut wrong_width.wire_trace.entries[0] else {
        panic!("validated trace starts with a host frame");
    };
    let HostWirePayload::Open {
        observation_width, ..
    } = &mut frame.payload
    else {
        panic!("validated trace starts with open");
    };
    *observation_width += 1;
    let HardwareWireTraceEntry::Device { frame } = &mut wrong_width.wire_trace.entries[1] else {
        panic!("validated trace continues with a device frame");
    };
    let DeviceWirePayload::Ready {
        observation_width, ..
    } = &mut frame.payload
    else {
        panic!("validated trace continues with ready");
    };
    *observation_width += 1;
    assert!(wrong_width.validate_against(&task).is_err());
    let actual = format!(
        "{}\n",
        serde_json::to_string_pretty(&evidence).expect("serialize evidence")
    );
    assert_eq!(actual, GOLDEN);
}

fn exchange(
    codec: &HardwareWireCodec,
    recorder: &mut HardwareWireTraceRecorder,
    stdin: &mut ChildStdin,
    stdout: &mut BufReader<ChildStdout>,
    request: HostWireFrame,
) -> DeviceWireFrame {
    recorder
        .record_host(request.clone())
        .expect("record host frame");
    stdin
        .write_all(&codec.encode_host(&request).expect("encode host frame"))
        .expect("write host frame");
    stdin.flush().expect("flush host frame");
    let line = codec
        .read_line(stdout)
        .expect("read device frame")
        .expect("device did not close early");
    let response = codec.decode_device(&line).expect("decode device frame");
    recorder
        .record_device(response.clone())
        .expect("record device frame");
    response
}
