use rne_hardware_gateway::wire::HardwareWireTraceOutcome;
use rne_hardware_gateway::{GatewayConnectionState, HardwareMode};
use rne_hardware_lekiwi::session::LeKiwiReferenceSessionEvidence;
use std::path::PathBuf;
use std::process::Command;

#[test]
fn session_cli_runs_the_real_python_mock_protocol_and_writes_evidence() {
    for (mode, expected_entries) in [("shadow", 8), ("live", 14)] {
        let output = output_path(mode);
        if output.exists() {
            std::fs::remove_file(&output).unwrap();
        }
        let status = Command::new(env!("CARGO_BIN_EXE_rne-lekiwi-session"))
            .args([
                "--mock",
                "--mode",
                mode,
                "--samples",
                "2",
                "--sample-period-ms",
                "1",
                "--session-id",
                &format!("rne.lekiwi.cli.{mode}"),
                "--output",
            ])
            .arg(&output)
            .status()
            .expect("run LeKiwi session host");
        assert!(status.success());

        let evidence: LeKiwiReferenceSessionEvidence =
            serde_json::from_slice(&std::fs::read(&output).unwrap()).unwrap();
        evidence.validate().unwrap();
        assert_eq!(
            evidence.device_id,
            rne_hardware_lekiwi::LEKIWI_MOCK_DEVICE_ID
        );
        assert_eq!(
            evidence.session.mode,
            if mode == "shadow" {
                HardwareMode::Shadow
            } else {
                HardwareMode::Live
            }
        );
        assert_eq!(
            evidence.session.wire_trace.outcome,
            HardwareWireTraceOutcome::Completed
        );
        assert_eq!(evidence.session.wire_trace.entries.len(), expected_entries);
        assert_eq!(
            evidence.session.gateway.final_snapshot.connection_state,
            GatewayConnectionState::Disconnected
        );
        std::fs::remove_file(output).unwrap();
    }
}

#[test]
fn session_cli_persists_gateway_safety_terminal_before_nonzero_exit() {
    let output = output_path("stale");
    if output.exists() {
        std::fs::remove_file(&output).unwrap();
    }
    let status = Command::new(env!("CARGO_BIN_EXE_rne-lekiwi-session"))
        .args([
            "--mock",
            "--mode",
            "live",
            "--samples",
            "2",
            "--sample-period-ms",
            "150",
            "--session-id",
            "rne.lekiwi.cli.stale",
            "--output",
        ])
        .arg(&output)
        .status()
        .expect("run stale-command LeKiwi session host");
    assert_eq!(status.code(), Some(3));

    let evidence: LeKiwiReferenceSessionEvidence =
        serde_json::from_slice(&std::fs::read(&output).unwrap()).unwrap();
    evidence.validate().unwrap();
    assert_eq!(
        evidence.session.wire_trace.outcome,
        HardwareWireTraceOutcome::GatewaySafetyStopped {
            reason: rne_hardware_gateway::SafetyReason::ObservationStale,
        }
    );
    std::fs::remove_file(output).unwrap();
}

fn output_path(mode: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "rne-lekiwi-session-cli-{}-{mode}.json",
        std::process::id()
    ))
}
