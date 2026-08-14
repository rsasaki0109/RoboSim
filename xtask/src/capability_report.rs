//! Machine-readable inventory of the RNE capabilities advertised by the
//! 0.2 trust foundation.

use anyhow::Context;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process::Command;

/// Schema version for the capability inventory.
pub(crate) const CAPABILITY_REPORT_SCHEMA_VERSION: u32 = 1;
/// Stable artifact discriminator for the capability inventory.
pub(crate) const CAPABILITY_REPORT_KIND: &str = "rne_capability_report";

const DEFAULT_OUTPUT: &str = "artifacts/capability-report/report.json";

#[derive(Debug, Clone, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct CapabilityReport {
    kind: String,
    schema_version: u32,
    release_version: String,
    git_commit: String,
    capabilities: Vec<Capability>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Capability {
    id: String,
    name: String,
    status: String,
    evidence: Vec<CapabilityEvidence>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct CapabilityEvidence {
    path: String,
    command: String,
}

struct CapabilityDefinition {
    id: &'static str,
    name: &'static str,
    status: &'static str,
    evidence: &'static [CapabilityEvidenceDefinition],
}

struct CapabilityEvidenceDefinition {
    path: &'static str,
    command: &'static str,
}

// Keep this list explicitly ordered. It follows the workflow order in
// docs/OSS_PARITY_MATRIX.md, not lexical ordering, so the report remains a
// stable product contract as new capabilities are added.
const ADVERTISED_CAPABILITIES: &[CapabilityDefinition] = &[
    CapabilityDefinition {
        id: "world_robot_authoring",
        name: "World and robot authoring",
        status: "parity",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/OSS_PARITY_MATRIX.md",
                command: "cargo run --locked -p xtask -- parity",
            },
            CapabilityEvidenceDefinition {
                path: "crates/rne_urdf_import/tests/fixtures/minimal_diff_drive.urdf",
                command: "cargo test --locked -p rne_urdf_import",
            },
        ],
    },
    CapabilityDefinition {
        id: "fixed_step_simulation",
        name: "Fixed-step simulation",
        status: "parity",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/OSS_PARITY.md",
                command: "cargo run --locked -p rne_asset_cli -- run assets/runs/mesh_diff_drive.rne.run.toml",
            },
            CapabilityEvidenceDefinition {
                path: "assets/runs/mesh_diff_drive.rne.run.toml",
                command: "cargo run --locked -p rne_asset_cli -- run assets/runs/mesh_diff_drive.rne.run.toml",
            },
        ],
    },
    CapabilityDefinition {
        id: "controller_actuator_io",
        name: "Controller and actuator I/O",
        status: "parity",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "crates/rne_robot/src/actuator.rs",
                command: "cargo test --locked -p rne_robot",
            },
            CapabilityEvidenceDefinition {
                path: "crates/rne_plugin/tests/load.rs",
                command: "cargo test --locked -p rne_plugin --test load",
            },
        ],
    },
    CapabilityDefinition {
        id: "sensor_simulation",
        name: "Sensor simulation",
        status: "parity",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "crates/rne_sensor/src/lib.rs",
                command: "cargo test --locked -p rne_sensor",
            },
            CapabilityEvidenceDefinition {
                path: "docs/PLAN_SENSOR_FRONTEND_TRANSPORT.md",
                command: "cargo run --locked -p xtask -- parity",
            },
        ],
    },
    CapabilityDefinition {
        id: "physics_selection_conformance",
        name: "Physics selection and conformance",
        status: "complete",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/PLAN_PHYSICS_CONFORMANCE.md",
                command: "cargo run --locked -p xtask -- physics-conformance",
            },
            CapabilityEvidenceDefinition {
                path: "tests/physics_conformance/src/lib.rs",
                command: "cargo test --locked -p rne_physics_conformance",
            },
            CapabilityEvidenceDefinition {
                path: "docs/PLAN_MUJOCO_SPIKE.md",
                command: "cargo test --locked -p rne_physics_mujoco",
            },
        ],
    },
    CapabilityDefinition {
        id: "scenario_authoring",
        name: "Scenario authoring",
        status: "complete",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/PLAN_SCENARIO_TRAFFIC_SCALE.md",
                command: "cargo run --locked -p xtask -- scenario-scale",
            },
            CapabilityEvidenceDefinition {
                path: "crates/rne_openscenario/tests/runtime.rs",
                command: "cargo test --locked -p rne_openscenario --test runtime",
            },
        ],
    },
    CapabilityDefinition {
        id: "native_traffic_runtime",
        name: "Native traffic runtime",
        status: "complete",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/TRAFFIC_RUNTIME.md",
                command: "cargo run --locked --release -p rne_scenario_scale -- --output artifacts/scenario-scale/report.json",
            },
            CapabilityEvidenceDefinition {
                path: "crates/rne_traffic/tests/fleet_replay.rs",
                command: "cargo test --locked -p rne_traffic --test fleet_replay",
            },
        ],
    },
    CapabilityDefinition {
        id: "sumo_road_import",
        name: "SUMO road import",
        status: "parity",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/TRAFFIC_ASSET.md",
                command: "cargo run --release -p rne_asset_cli -- sumo-net assets/networks/minimal_cross.net.xml --out target/runs/minimal_cross.rne.traffic.json",
            },
            CapabilityEvidenceDefinition {
                path: "crates/rne_sumo/src/lib.rs",
                command: "cargo test --locked -p rne_sumo",
            },
        ],
    },
    CapabilityDefinition {
        id: "external_traffic_cosimulation",
        name: "External traffic co-simulation",
        status: "complete",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/TRAFFIC_RUNTIME.md",
                command: "cargo test --locked -p rne_traci --test co_simulation",
            },
            CapabilityEvidenceDefinition {
                path: "crates/rne_traci/tests/co_simulation.rs",
                command: "cargo test --locked -p rne_traci --test co_simulation",
            },
        ],
    },
    CapabilityDefinition {
        id: "runner_control_remote_inspection",
        name: "Runner control and remote inspection",
        status: "parity",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "crates/rne_core/src/control.rs",
                command: "cargo test --locked -p rne_core",
            },
            CapabilityEvidenceDefinition {
                path: "docs/COMPATIBILITY.md",
                command: "cargo run --locked -p xtask -- parity",
            },
        ],
    },
    CapabilityDefinition {
        id: "frontend_rendering",
        name: "Frontend and rendering",
        status: "complete",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/PLAN_SENSOR_FRONTEND_TRANSPORT.md",
                command: "cargo run --locked -p xtask -- parity",
            },
            CapabilityEvidenceDefinition {
                path: "examples/14_interactive_viewer/main.rs",
                command: "cargo check --locked -p interactive_viewer --example 14_interactive_viewer",
            },
        ],
    },
    CapabilityDefinition {
        id: "extension_integration",
        name: "Extension and integration",
        status: "parity",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/COMPATIBILITY.md",
                command: "cargo test --locked -p rne_plugin",
            },
            CapabilityEvidenceDefinition {
                path: "crates/rne_plugin/src/lib.rs",
                command: "cargo test --locked -p rne_plugin",
            },
        ],
    },
    CapabilityDefinition {
        id: "ci_evaluation",
        name: "CI and evaluation",
        status: "partial",
        evidence: &[
            CapabilityEvidenceDefinition {
                path: "docs/OSS_PARITY_MATRIX.md",
                command: "cargo run --locked -p xtask -- parity",
            },
            CapabilityEvidenceDefinition {
                path: "tests/determinism/tests/scenarios.rs",
                command: "cargo test --locked -p rne_determinism_tests",
            },
            CapabilityEvidenceDefinition {
                path: "docs/BENCHMARKS.md",
                command: "cargo run --locked -p xtask -- benchmark",
            },
            CapabilityEvidenceDefinition {
                path: "docs/FAILURE_CAPSULE.md",
                command: "cargo test --locked -p xtask failure_capsule",
            },
            CapabilityEvidenceDefinition {
                path: "docs/adr/011-determinism-contract.md",
                command: "cargo test --locked -p rne_core determinism",
            },
            CapabilityEvidenceDefinition {
                path: "docs/EVIDENCE_QUICKSTART.md",
                command: "cargo run --locked -p xtask -- evidence",
            },
        ],
    },
];

/// Emit the deterministic capability report.
pub(crate) fn capability_report(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let root = workspace_root()?;
    let output = parse_output(args, &root)?;
    let git_commit = git_commit(&root)?;
    let report = build_report(&git_commit);

    validate_report(&report)?;
    validate_committed_evidence_paths(&root, &report)?;

    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("create capability report directory {}", parent.display()))?;
    }
    let mut json = serde_json::to_vec_pretty(&report)?;
    json.push(b'\n');
    fs::write(&output, json)
        .with_context(|| format!("write capability report {}", output.display()))?;
    println!(
        "capability report ok: capabilities={} output={}",
        report.capabilities.len(),
        output.display()
    );
    Ok(())
}

fn build_report(git_commit: &str) -> CapabilityReport {
    CapabilityReport {
        kind: CAPABILITY_REPORT_KIND.to_string(),
        schema_version: CAPABILITY_REPORT_SCHEMA_VERSION,
        release_version: crate::RELEASE_VERSION.to_string(),
        git_commit: git_commit.to_string(),
        capabilities: ADVERTISED_CAPABILITIES
            .iter()
            .map(|definition| Capability {
                id: definition.id.to_string(),
                name: definition.name.to_string(),
                status: definition.status.to_string(),
                evidence: definition
                    .evidence
                    .iter()
                    .map(|evidence| CapabilityEvidence {
                        path: evidence.path.to_string(),
                        command: evidence.command.to_string(),
                    })
                    .collect(),
            })
            .collect(),
    }
}

fn parse_output(args: &mut impl Iterator<Item = String>, root: &Path) -> anyhow::Result<PathBuf> {
    let mut output = PathBuf::from(DEFAULT_OUTPUT);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--output" | "--json" => {
                output = PathBuf::from(
                    args.next()
                        .ok_or_else(|| anyhow::anyhow!("{argument} requires a path"))?,
                );
            }
            other => anyhow::bail!("unknown capability-report argument: {other}"),
        }
    }
    Ok(absolute_from(root, output))
}

fn absolute_from(root: &Path, path: PathBuf) -> PathBuf {
    if path.is_absolute() {
        path
    } else {
        root.join(path)
    }
}

fn build_capability_ids() -> Vec<&'static str> {
    ADVERTISED_CAPABILITIES
        .iter()
        .map(|capability| capability.id)
        .collect()
}

fn validate_report(report: &CapabilityReport) -> anyhow::Result<()> {
    anyhow::ensure!(
        report.kind == CAPABILITY_REPORT_KIND,
        "capability report kind must be {CAPABILITY_REPORT_KIND}"
    );
    anyhow::ensure!(
        report.schema_version == CAPABILITY_REPORT_SCHEMA_VERSION,
        "capability report schema_version must be {CAPABILITY_REPORT_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        report.release_version == crate::RELEASE_VERSION,
        "capability report release_version must be {}",
        crate::RELEASE_VERSION
    );
    anyhow::ensure!(
        !report.git_commit.trim().is_empty(),
        "capability report git_commit must be non-empty"
    );

    let expected_ids = build_capability_ids();
    let actual_ids = report
        .capabilities
        .iter()
        .map(|capability| capability.id.as_str())
        .collect::<Vec<_>>();
    anyhow::ensure!(
        actual_ids == expected_ids,
        "capability order differs from the advertised contract: expected={expected_ids:?} actual={actual_ids:?}"
    );

    let expected_capabilities = build_report(&report.git_commit).capabilities;
    anyhow::ensure!(
        report.capabilities == expected_capabilities,
        "capability metadata differs from the advertised contract"
    );

    let mut ids = BTreeSet::new();
    for capability in &report.capabilities {
        anyhow::ensure!(
            ids.insert(capability.id.as_str()),
            "capability id is duplicated: {}",
            capability.id
        );
        anyhow::ensure!(
            !capability.name.trim().is_empty(),
            "capability {} must have a name",
            capability.id
        );
        anyhow::ensure!(
            matches!(
                capability.status.as_str(),
                "complete" | "parity" | "partial"
            ),
            "capability {} has unsupported status {:?}",
            capability.id,
            capability.status
        );
        anyhow::ensure!(
            !capability.evidence.is_empty(),
            "capability {} must have evidence",
            capability.id
        );
        for evidence in &capability.evidence {
            anyhow::ensure!(
                !evidence.path.trim().is_empty(),
                "capability {} has an empty evidence path",
                capability.id
            );
            anyhow::ensure!(
                !evidence.command.trim().is_empty(),
                "capability {} has an empty evidence command",
                capability.id
            );
        }
    }
    Ok(())
}

fn validate_committed_evidence_paths(root: &Path, report: &CapabilityReport) -> anyhow::Result<()> {
    validate_evidence_paths(root, report)?;
    for capability in &report.capabilities {
        for evidence in &capability.evidence {
            let output = Command::new("git")
                .current_dir(root)
                .args(["ls-files", "--error-unmatch", "--", evidence.path.as_str()])
                .output()
                .with_context(|| format!("check tracked evidence path {}", evidence.path))?;
            anyhow::ensure!(
                output.status.success(),
                "evidence path {} must be committed",
                evidence.path
            );
        }
    }
    Ok(())
}

fn validate_evidence_paths(root: &Path, report: &CapabilityReport) -> anyhow::Result<()> {
    for capability in &report.capabilities {
        for evidence in &capability.evidence {
            let path = Path::new(&evidence.path);
            anyhow::ensure!(
                !path.is_absolute(),
                "evidence path {} must be relative to the repository",
                evidence.path
            );
            anyhow::ensure!(
                !evidence.path.contains('\\'),
                "evidence path {} must use '/' separators",
                evidence.path
            );
            anyhow::ensure!(
                path.components().all(|component| {
                    !matches!(
                        component,
                        Component::ParentDir | Component::RootDir | Component::Prefix(_)
                    )
                }),
                "evidence path {} must not escape the repository",
                evidence.path
            );
            let absolute = root.join(path);
            anyhow::ensure!(
                absolute.is_file(),
                "evidence path {} does not exist as a file",
                evidence.path
            );
        }
    }
    Ok(())
}

fn git_commit(root: &Path) -> anyhow::Result<String> {
    let output = Command::new("git")
        .current_dir(root)
        .args(["rev-parse", "HEAD"])
        .output()
        .context("read the current git commit")?;
    anyhow::ensure!(
        output.status.success(),
        "git rev-parse HEAD failed: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    );
    let commit = String::from_utf8(output.stdout)?.trim().to_string();
    anyhow::ensure!(
        !commit.is_empty(),
        "git rev-parse HEAD returned an empty commit"
    );
    Ok(commit)
}

fn workspace_root() -> anyhow::Result<PathBuf> {
    let output = Command::new("cargo")
        .args(["metadata", "--locked", "--format-version", "1", "--no-deps"])
        .output()
        .context("read workspace metadata")?;
    anyhow::ensure!(
        output.status.success(),
        "cargo metadata failed: {}",
        String::from_utf8_lossy(&output.stderr).trim()
    );
    let metadata: serde_json::Value = serde_json::from_slice(&output.stdout)?;
    metadata["workspace_root"]
        .as_str()
        .map(PathBuf::from)
        .ok_or_else(|| anyhow::anyhow!("missing workspace_root in cargo metadata"))
}

#[cfg(test)]
mod tests {
    use super::{
        build_capability_ids, build_report, validate_evidence_paths, validate_report,
        CapabilityEvidence, CapabilityReport, CAPABILITY_REPORT_KIND,
        CAPABILITY_REPORT_SCHEMA_VERSION,
    };
    use std::fs;
    use tempfile::tempdir;

    #[test]
    fn advertised_capability_order_is_explicit_and_stable() {
        let report = build_report("0123456789012345678901234567890123456789");
        let ids = report
            .capabilities
            .iter()
            .map(|capability| capability.id.as_str())
            .collect::<Vec<_>>();
        assert_eq!(ids, build_capability_ids());
        assert_eq!(
            ids,
            vec![
                "world_robot_authoring",
                "fixed_step_simulation",
                "controller_actuator_io",
                "sensor_simulation",
                "physics_selection_conformance",
                "scenario_authoring",
                "native_traffic_runtime",
                "sumo_road_import",
                "external_traffic_cosimulation",
                "runner_control_remote_inspection",
                "frontend_rendering",
                "extension_integration",
                "ci_evaluation",
            ]
        );
        assert_eq!(report.schema_version, CAPABILITY_REPORT_SCHEMA_VERSION);
    }

    #[test]
    fn report_schema_serialization_is_stable() {
        let report = CapabilityReport {
            kind: CAPABILITY_REPORT_KIND.to_string(),
            schema_version: CAPABILITY_REPORT_SCHEMA_VERSION,
            release_version: "0.1.0".to_string(),
            git_commit: "0123456789012345678901234567890123456789".to_string(),
            capabilities: vec![super::Capability {
                id: "example".to_string(),
                name: "Example".to_string(),
                status: "complete".to_string(),
                evidence: vec![CapabilityEvidence {
                    path: "docs/example.md".to_string(),
                    command: "cargo test --locked -p example".to_string(),
                }],
            }],
        };
        let json = serde_json::to_string(&report).expect("serialize capability report");
        let golden = include_str!("../../tests/golden/evidence/capability-report-v1.json");
        assert_eq!(json, golden.trim_end());
        let decoded: CapabilityReport = serde_json::from_str(golden).expect("parse golden");
        assert_eq!(decoded, report);
    }

    #[test]
    fn report_schema_rejects_unknown_top_level_fields() {
        let json = r#"{
            "kind": "rne_capability_report",
            "schema_version": 1,
            "release_version": "0.1.0",
            "git_commit": "0123456789012345678901234567890123456789",
            "capabilities": [],
            "unexpected": true
        }"#;
        assert!(serde_json::from_str::<CapabilityReport>(json).is_err());
    }

    #[test]
    fn report_validation_rejects_reordered_capabilities() {
        let mut report = build_report("0123456789012345678901234567890123456789");
        report.capabilities.swap(0, 1);
        assert!(validate_report(&report).is_err());
    }

    #[test]
    fn report_validation_rejects_status_drift() {
        let mut report = build_report("0123456789012345678901234567890123456789");
        report.capabilities[0].status = "complete".to_string();
        assert!(validate_report(&report).is_err());
    }

    #[test]
    fn report_validation_rejects_kind_drift() {
        let mut report = build_report("0123456789012345678901234567890123456789");
        report.kind = "other_artifact".to_string();
        assert!(validate_report(&report).is_err());
    }

    #[test]
    fn evidence_validation_requires_existing_repository_files() {
        let root = tempdir().expect("temporary repository");
        fs::create_dir_all(root.path().join("docs")).expect("docs directory");
        fs::write(root.path().join("docs/evidence.md"), "evidence\n").expect("evidence file");
        let mut report = build_report("0123456789012345678901234567890123456789");
        report.capabilities.clear();
        report.capabilities.push(super::Capability {
            id: "world_robot_authoring".to_string(),
            name: "World and robot authoring".to_string(),
            status: "parity".to_string(),
            evidence: vec![CapabilityEvidence {
                path: "docs/evidence.md".to_string(),
                command: "true".to_string(),
            }],
        });
        assert!(validate_evidence_paths(root.path(), &report).is_ok());
        report.capabilities[0].evidence[0].path = "docs/missing.md".to_string();
        assert!(validate_evidence_paths(root.path(), &report).is_err());
    }

    #[test]
    fn evidence_validation_rejects_repository_escape() {
        let root = tempdir().expect("temporary repository");
        let mut report = build_report("0123456789012345678901234567890123456789");
        report.capabilities.clear();
        report.capabilities.push(super::Capability {
            id: "world_robot_authoring".to_string(),
            name: "World and robot authoring".to_string(),
            status: "parity".to_string(),
            evidence: vec![CapabilityEvidence {
                path: "../outside.md".to_string(),
                command: "true".to_string(),
            }],
        });
        assert!(validate_evidence_paths(root.path(), &report).is_err());
    }
}
