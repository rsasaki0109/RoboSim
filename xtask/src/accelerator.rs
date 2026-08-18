//! Validation for the one selected optional accelerator adapter.

use anyhow::{Context, Result};
use rne_accelerator_contract::{
    AcceleratorCapabilityReport, AcceleratorConformanceReport, AcceleratorManifest,
    AcceleratorRuntimeContract, AcceleratorRuntimePackages, AcceleratorScaleReport,
};
pub(crate) use rne_accelerator_contract::{
    ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION, ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
    ACCELERATOR_MANIFEST_SCHEMA_VERSION, ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
    ACCELERATOR_RUNTIME_CONTRACT_SCHEMA_VERSION, ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION,
};
use std::fs;
use std::path::Path;
use std::process::Command;

const SELECTED_MANIFEST: &str = "adapters/mjx/accelerator.toml";
const BATCH_WIDTHS: [usize; 4] = [1, 16, 256, 4096];
const TASK_ID: &str = "rne.physics.free_fall.mjx.v1";

pub(crate) fn accelerator_check(args: &mut impl Iterator<Item = String>) -> Result<()> {
    anyhow::ensure!(
        args.next().is_none(),
        "accelerator-check accepts no arguments"
    );
    let root = super::workspace_root()?;
    let manifest = validate_contract(&root)?;
    println!(
        "accelerator selection ok: id={} runtime={} status={}",
        manifest.id, manifest.runtime, manifest.status
    );
    Ok(())
}

pub(crate) fn accelerator_conformance(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = super::workspace_root()?;
    let manifest = validate_selected_manifest(&root)?;
    validate_task_binding(&root, &manifest)?;
    validate_runtime_contract(&root, &manifest)?;
    let python = super::python_command()?;
    let forwarded: Vec<String> = args.collect();
    let status = Command::new(python)
        .current_dir(&root)
        .arg(root.join("adapters/mjx/conformance.py"))
        .args(forwarded)
        .status()
        .context("run MJX adapter conformance")?;
    anyhow::ensure!(status.success(), "MJX adapter conformance failed");
    Ok(())
}

pub(crate) fn accelerator_scale(args: &mut impl Iterator<Item = String>) -> Result<()> {
    run_python_evidence_command("scale.py", "scale", args)
}

fn run_python_evidence_command(
    script_name: &str,
    label: &str,
    args: &mut impl Iterator<Item = String>,
) -> Result<()> {
    let root = super::workspace_root()?;
    let manifest = validate_selected_manifest(&root)?;
    validate_task_binding(&root, &manifest)?;
    validate_runtime_contract(&root, &manifest)?;
    let python = super::python_command()?;
    let forwarded: Vec<String> = args.collect();
    let status = Command::new(python)
        .current_dir(&root)
        .arg(root.join("adapters/mjx").join(script_name))
        .args(forwarded)
        .status()
        .with_context(|| format!("run MJX adapter {label}"))?;
    anyhow::ensure!(status.success(), "MJX adapter {label} failed");
    Ok(())
}

pub(crate) fn validate_contract(root: &Path) -> Result<AcceleratorManifest> {
    let manifest = validate_selected_manifest(root)?;
    validate_task_binding(root, &manifest)?;
    validate_runtime_contract(root, &manifest)?;
    validate_capability_fixture(root, &manifest)?;
    validate_conformance_fixture(root, &manifest)?;
    validate_scale_fixture(root, &manifest)?;
    run_python_contract_tests(root)?;
    Ok(manifest)
}

pub(crate) fn validate_selected_manifest(root: &Path) -> Result<AcceleratorManifest> {
    let path = root.join(SELECTED_MANIFEST);
    let text = fs::read_to_string(&path)
        .with_context(|| format!("read accelerator manifest {}", path.display()))?;
    let manifest: AcceleratorManifest = toml::from_str(&text)
        .with_context(|| format!("parse accelerator manifest {}", path.display()))?;
    validate_manifest(root, &manifest)?;
    Ok(manifest)
}

fn validate_manifest(root: &Path, manifest: &AcceleratorManifest) -> Result<()> {
    manifest.validate()?;
    anyhow::ensure!(
        manifest.schema_version == ACCELERATOR_MANIFEST_SCHEMA_VERSION,
        "accelerator manifest schema must be {ACCELERATOR_MANIFEST_SCHEMA_VERSION}"
    );
    anyhow::ensure!(
        manifest.id == "mjx_warp",
        "selected accelerator must be mjx_warp"
    );
    anyhow::ensure!(manifest.selected, "accelerator manifest must be selected");
    anyhow::ensure!(
        manifest.status == "experimental",
        "accelerator remains experimental until GPU conformance evidence exists"
    );
    anyhow::ensure!(manifest.runtime == "mujoco_mjx_warp", "runtime mismatch");
    anyhow::ensure!(
        manifest.precision == "f64",
        "accelerator precision mismatch"
    );
    anyhow::ensure!(
        manifest.execution_boundary == "out_of_process_python",
        "accelerator must remain behind the Python process boundary"
    );
    anyhow::ensure!(
        !manifest.core_dependency,
        "accelerator dependencies must not enter core crates"
    );
    anyhow::ensure!(
        manifest.requires_nvidia_gpu,
        "MJX-Warp requires an NVIDIA GPU"
    );
    anyhow::ensure!(
        manifest.task_spec_schema == rne_ai::TASK_SPEC_SCHEMA_VERSION,
        "accelerator TaskSpec schema mismatch"
    );
    anyhow::ensure!(
        manifest.batch_checkpoint_schema == rne_ai::PORTABLE_BATCH_CHECKPOINT_VERSION,
        "accelerator checkpoint schema mismatch"
    );
    anyhow::ensure!(
        manifest.protocol_schema == ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
        "accelerator protocol schema mismatch"
    );
    anyhow::ensure!(
        manifest.capability_report_schema == ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION,
        "accelerator capability-report schema mismatch"
    );
    anyhow::ensure!(
        manifest.conformance_report_schema == ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
        "accelerator conformance-report schema mismatch"
    );
    anyhow::ensure!(
        manifest.scale_report_schema == ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION,
        "accelerator scale-report schema mismatch"
    );
    anyhow::ensure!(
        manifest.supported_batch_widths == BATCH_WIDTHS,
        "accelerator batch-width evidence points must be {BATCH_WIDTHS:?}"
    );
    anyhow::ensure!(
        manifest.official_sources.len() == 3
            && manifest
                .official_sources
                .iter()
                .all(|source| source.starts_with("https://")),
        "accelerator selection must retain three official HTTPS sources"
    );
    let adr = root.join(&manifest.selection_adr);
    anyhow::ensure!(
        adr.is_file(),
        "accelerator selection ADR is missing: {}",
        adr.display()
    );
    Ok(())
}

fn validate_task_binding(root: &Path, manifest: &AcceleratorManifest) -> Result<()> {
    let task_path = root.join(&manifest.binding_task_spec);
    let task_text = fs::read_to_string(&task_path)
        .with_context(|| format!("read accelerator TaskSpec {}", task_path.display()))?;
    let task_spec: rne_ai::TaskSpec = serde_json::from_str(&task_text)
        .with_context(|| format!("parse accelerator TaskSpec {}", task_path.display()))?;
    task_spec
        .validate()
        .with_context(|| format!("validate accelerator TaskSpec {}", task_path.display()))?;
    anyhow::ensure!(
        task_spec.task_id == TASK_ID,
        "accelerator task binding ID mismatch"
    );

    let model_path = root.join(&manifest.binding_model);
    let model = fs::read_to_string(&model_path)
        .with_context(|| format!("read accelerator model {}", model_path.display()))?;
    anyhow::ensure!(
        model.len() <= 1024 * 1024
            && model.contains("<mujoco model=\"rne-free-fall\">")
            && model.contains("timestep=\"0.016666666\"")
            && model.contains("gravity=\"0 -9.81 0\"")
            && model.contains("<freejoint name=\"rne_free_fall_joint\"/>")
            && model.contains("<geom name=\"rne_free_fall_sphere\""),
        "accelerator model must remain the bounded free-fall binding"
    );
    anyhow::ensure!(
        root.join("adapters/mjx/conformance.py").is_file(),
        "accelerator conformance runner is missing"
    );
    anyhow::ensure!(
        root.join("adapters/mjx/scale.py").is_file(),
        "accelerator scale runner is missing"
    );
    Ok(())
}

fn validate_runtime_contract(root: &Path, manifest: &AcceleratorManifest) -> Result<()> {
    let contract_path = root.join(&manifest.runtime_contract);
    let contract_text = fs::read_to_string(&contract_path)
        .with_context(|| format!("read accelerator runtime {}", contract_path.display()))?;
    let contract: AcceleratorRuntimeContract = toml::from_str(&contract_text)
        .with_context(|| format!("parse accelerator runtime {}", contract_path.display()))?;
    contract.validate()?;
    anyhow::ensure!(
        contract.schema_version == ACCELERATOR_RUNTIME_CONTRACT_SCHEMA_VERSION,
        "accelerator runtime-contract schema mismatch"
    );
    anyhow::ensure!(
        contract.operating_system == "linux"
            && contract.architecture == "x86_64"
            && contract.python == "3.12"
            && contract.cuda_major == 13
            && contract.nvidia_driver_minimum == 580,
        "accelerator runtime host contract drifted"
    );
    anyhow::ensure!(
        contract.packages
            == (AcceleratorRuntimePackages {
                jax: "0.10.2".to_string(),
                jaxlib: "0.10.2".to_string(),
                jax_cuda_plugin: "0.10.2".to_string(),
                mujoco: "3.9.0".to_string(),
                mujoco_mjx: "3.9.0".to_string(),
                warp_lang: "1.12.1".to_string(),
            }),
        "accelerator package pins drifted"
    );
    anyhow::ensure!(
        contract.official_sources.len() == 3
            && contract
                .official_sources
                .iter()
                .all(|source| source.starts_with("https://")),
        "accelerator runtime contract must retain three official sources"
    );

    let requirements_path = root.join(&manifest.requirements);
    let requirements = fs::read_to_string(&requirements_path).with_context(|| {
        format!(
            "read accelerator requirements {}",
            requirements_path.display()
        )
    })?;
    for pin in [
        "jax[cuda13]==0.10.2",
        "jaxlib==0.10.2",
        "jax-cuda13-plugin[with-cuda]==0.10.2",
        "mujoco==3.9.0",
        "mujoco-mjx[warp]==3.9.0",
        "warp-lang==1.12.1",
    ] {
        anyhow::ensure!(
            requirements.lines().any(|line| line.trim() == pin),
            "accelerator requirements are missing {pin}"
        );
    }
    Ok(())
}

fn validate_capability_fixture(root: &Path, manifest: &AcceleratorManifest) -> Result<()> {
    let runtime: AcceleratorRuntimeContract =
        toml::from_str(&fs::read_to_string(root.join(&manifest.runtime_contract))?)?;
    let task: rne_ai::TaskSpec =
        serde_json::from_slice(&fs::read(root.join(&manifest.binding_task_spec))?)?;
    let report_path = root.join("tests/golden/accelerators/capability-report-v1.json");
    let report: AcceleratorCapabilityReport = serde_json::from_slice(&fs::read(&report_path)?)
        .with_context(|| {
            format!(
                "parse accelerator capability report {}",
                report_path.display()
            )
        })?;
    report.validate_against(manifest, &runtime, &task)?;
    Ok(())
}

fn validate_conformance_fixture(root: &Path, manifest: &AcceleratorManifest) -> Result<()> {
    let runtime: AcceleratorRuntimeContract =
        toml::from_str(&fs::read_to_string(root.join(&manifest.runtime_contract))?)?;
    let task: rne_ai::TaskSpec =
        serde_json::from_slice(&fs::read(root.join(&manifest.binding_task_spec))?)?;
    let model = fs::read(root.join(&manifest.binding_model))?;
    let report_path = root.join("tests/golden/accelerators/conformance-report-v1.json");
    let report = AcceleratorConformanceReport::from_json_slice(&fs::read(&report_path)?)
        .with_context(|| {
            format!(
                "parse accelerator conformance report {}",
                report_path.display()
            )
        })?;
    report.validate_against(manifest, &runtime, &task, &model)?;
    Ok(())
}

fn validate_scale_fixture(root: &Path, manifest: &AcceleratorManifest) -> Result<()> {
    let runtime: AcceleratorRuntimeContract =
        toml::from_str(&fs::read_to_string(root.join(&manifest.runtime_contract))?)?;
    let task: rne_ai::TaskSpec =
        serde_json::from_slice(&fs::read(root.join(&manifest.binding_task_spec))?)?;
    let model = fs::read(root.join(&manifest.binding_model))?;
    let report_path = root.join("tests/golden/accelerators/scale-report-v1.json");
    let report = AcceleratorScaleReport::from_json_slice(&fs::read(&report_path)?)
        .with_context(|| format!("parse accelerator scale report {}", report_path.display()))?;
    report.validate_against(manifest, &runtime, &task, &model)?;
    Ok(())
}

fn run_python_contract_tests(root: &Path) -> Result<()> {
    let python = super::python_command()?;
    let status = Command::new(python)
        .current_dir(root)
        .args([
            "-m",
            "unittest",
            "discover",
            "-s",
            "adapters/mjx/tests",
            "-v",
        ])
        .status()
        .context("run MJX adapter protocol tests")?;
    anyhow::ensure!(status.success(), "MJX adapter protocol tests failed");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_accelerator_manifest_is_selected_and_core_free() {
        let root = super::super::workspace_root().expect("workspace root");
        let manifest = validate_selected_manifest(&root).expect("selected accelerator manifest");
        assert!(manifest.selected);
        assert!(!manifest.core_dependency);
        assert_eq!(manifest.supported_batch_widths, BATCH_WIDTHS);
        validate_task_binding(&root, &manifest).expect("task binding");
        validate_runtime_contract(&root, &manifest).expect("runtime contract");
        validate_capability_fixture(&root, &manifest).expect("capability fixture");
        validate_conformance_fixture(&root, &manifest).expect("conformance fixture");
        validate_scale_fixture(&root, &manifest).expect("scale fixture");
    }

    #[test]
    fn core_dependency_and_multiple_runtimes_are_rejected() {
        let root = super::super::workspace_root().expect("workspace root");
        let mut manifest = validate_selected_manifest(&root).expect("selected manifest");
        manifest.core_dependency = true;
        assert!(validate_manifest(&root, &manifest).is_err());
        manifest.core_dependency = false;
        manifest.runtime = "mjx_jax_and_warp".to_string();
        assert!(validate_manifest(&root, &manifest).is_err());
    }
}
