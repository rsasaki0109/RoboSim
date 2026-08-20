use rne_physics::{PhysicsBackendManifest, PhysicsBackendRepeatability, PhysicsCapability};
use rne_physics_analytic::AnalyticBackend;
use rne_physics_conformance::{
    run_external_backend_conformance, ExternalPhysicsBackendConformanceConfig,
    ExternalPhysicsBackendSubject,
};
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let output = match (args.next().as_deref(), args.next(), args.next()) {
        (None, None, None) => None,
        (Some("--output"), Some(path), None) => Some(PathBuf::from(path)),
        _ => return Err("usage: reference_external_backend [--output REPORT.json]".into()),
    };
    let subject_bytes = b"reference external backend source bundle v1";
    let subject = ExternalPhysicsBackendSubject::from_bytes(
        "reference-external-backend-source.tar.zst",
        subject_bytes,
    )?;
    let manifest = PhysicsBackendManifest::new(
        "reference_external_backend",
        "0.1.0",
        "independent_ballistic_engine",
        "1",
        [
            PhysicsCapability::RigidBody,
            PhysicsCapability::DeterministicStep,
            PhysicsCapability::KinematicBody,
        ],
        PhysicsBackendRepeatability::SameRuntimeExact,
    )?;
    let report = run_external_backend_conformance::<AnalyticBackend, _>(
        ExternalPhysicsBackendConformanceConfig::new(subject, manifest),
        AnalyticBackend::new,
    )?;
    if let Some(output) = output {
        if let Some(parent) = output.parent() {
            std::fs::create_dir_all(parent)?;
        }
        report.write_json(&output)?;
        println!("wrote {}", output.display());
    } else {
        print!("{}", report.to_json_pretty()?);
    }
    if report.passed() {
        Ok(())
    } else {
        Err("external physics backend conformance failed".into())
    }
}
