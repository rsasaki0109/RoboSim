use rne_hardware_gateway::wire::{
    DeviceWireFrame, DeviceWirePayload, HardwareWireCodec, HostWireFrame, HostWirePayload,
    WireRejectionCode,
};
use rne_hardware_gateway::{ActuationFrame, HardwareMode, SafetyReason};
use rne_hardware_lekiwi::LEKIWI_BASE_TASK_ID;
use std::io::{BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::time::Duration;

#[test]
fn python_bridge_maps_observations_actions_and_safe_stop() {
    let mut process = BridgeProcess::spawn(&[]);
    let ready = process.exchange(HostWirePayload::Open {
        task_id: LEKIWI_BASE_TASK_ID.to_string(),
        mode: HardwareMode::Live,
        observation_width: 9,
        action_width: 3,
    });
    assert!(matches!(ready.payload, DeviceWirePayload::Ready { .. }));

    let observation = process.exchange(HostWirePayload::PollObservation);
    let DeviceWirePayload::Observation { sequence, values } = observation.payload else {
        panic!("bridge did not return an observation");
    };
    assert_eq!(sequence, 1);
    assert_eq!(values.len(), 9);
    assert!((values[0] - 10_f64.to_radians()).abs() < 1.0e-12);
    assert_eq!(values[5], 60.0);

    let accepted = process.exchange(HostWirePayload::Actuate {
        frame: ActuationFrame {
            action_sequence: Some(1),
            queued_at_ms: 2,
            values: vec![0.05, -0.05, 0.1],
            safety_stop: false,
            reason: None,
        },
    });
    assert_eq!(
        accepted.payload,
        DeviceWirePayload::ActuationAccepted {
            action_sequence: Some(1),
            safety_stop: false,
        }
    );

    let stopped = process.exchange(HostWirePayload::Actuate {
        frame: ActuationFrame {
            action_sequence: None,
            queued_at_ms: 3,
            values: vec![0.0; 3],
            safety_stop: true,
            reason: Some(SafetyReason::ManualDisarm),
        },
    });
    assert_eq!(
        stopped.payload,
        DeviceWirePayload::ActuationAccepted {
            action_sequence: None,
            safety_stop: true,
        }
    );
    process.close();
}

#[test]
fn python_bridge_denies_shadow_actuation() {
    let mut process = BridgeProcess::spawn(&[]);
    assert!(matches!(
        process
            .exchange(HostWirePayload::Open {
                task_id: LEKIWI_BASE_TASK_ID.to_string(),
                mode: HardwareMode::Shadow,
                observation_width: 9,
                action_width: 3,
            })
            .payload,
        DeviceWirePayload::Ready { .. }
    ));
    let _ = process.exchange(HostWirePayload::PollObservation);
    assert_eq!(
        process
            .exchange(HostWirePayload::Actuate {
                frame: ActuationFrame {
                    action_sequence: Some(1),
                    queued_at_ms: 2,
                    values: vec![0.0; 3],
                    safety_stop: false,
                    reason: None,
                },
            })
            .payload,
        DeviceWirePayload::Rejected {
            code: WireRejectionCode::AuthorityDenied,
        }
    );
    process.close();
}

#[test]
fn python_bridge_watchdog_stops_without_a_host_command() {
    let mut process = BridgeProcess::spawn(&["--mock-watchdog-timeout-ms", "40"]);
    assert!(matches!(
        process
            .exchange(HostWirePayload::Open {
                task_id: LEKIWI_BASE_TASK_ID.to_string(),
                mode: HardwareMode::Live,
                observation_width: 9,
                action_width: 3,
            })
            .payload,
        DeviceWirePayload::Ready { .. }
    ));
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(
        process.exchange(HostWirePayload::PollObservation).payload,
        DeviceWirePayload::SafetySignal {
            reason: SafetyReason::CommandStale,
            safe_stop_applied: true,
        }
    );
    process.wait();
}

struct BridgeProcess {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
    codec: HardwareWireCodec,
    next_sequence: u64,
}

impl BridgeProcess {
    fn spawn(extra_args: &[&str]) -> Self {
        let script =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("python/rne_hardware_lekiwi_device.py");
        let mut child = Command::new(python_command())
            .arg(script)
            .arg("--mock")
            .args(extra_args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .expect("spawn LeKiwi Python bridge");
        Self {
            stdin: child.stdin.take().expect("bridge stdin"),
            stdout: BufReader::new(child.stdout.take().expect("bridge stdout")),
            child,
            codec: HardwareWireCodec::default(),
            next_sequence: 1,
        }
    }

    fn exchange(&mut self, payload: HostWirePayload) -> DeviceWireFrame {
        let request = HostWireFrame::new("rne.lekiwi.process.test", self.next_sequence, payload);
        self.next_sequence += 1;
        self.stdin
            .write_all(&self.codec.encode_host(&request).unwrap())
            .unwrap();
        self.stdin.flush().unwrap();
        let line = self
            .codec
            .read_line(&mut self.stdout)
            .unwrap()
            .expect("bridge closed before responding");
        let response = self.codec.decode_device(&line).unwrap();
        assert_eq!(response.session_id, request.session_id);
        assert_eq!(response.request_sequence, request.sequence);
        response
    }

    fn close(&mut self) {
        assert_eq!(
            self.exchange(HostWirePayload::Close).payload,
            DeviceWirePayload::Closed
        );
        self.wait();
    }

    fn wait(&mut self) {
        assert!(self.child.wait().unwrap().success());
    }
}

fn python_command() -> PathBuf {
    let candidates = std::env::var_os("PYTHON")
        .map(PathBuf::from)
        .into_iter()
        .chain([PathBuf::from("python3"), PathBuf::from("python")]);
    for candidate in candidates {
        let status = Command::new(&candidate)
            .arg("--version")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        if status.is_ok_and(|status| status.success()) {
            return candidate;
        }
    }
    panic!("Python 3 is required to test the LeKiwi device bridge");
}
