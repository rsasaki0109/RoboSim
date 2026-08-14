//! Validation for the one selected optional accelerator adapter.

use anyhow::{Context, Result};
use serde::Deserialize;
use std::fs;
use std::path::Path;

pub(crate) const ACCELERATOR_MANIFEST_SCHEMA_VERSION: u32 = 1;
const SELECTED_MANIFEST: &str = "adapters/mjx/accelerator.toml";
const BATCH_WIDTHS: [usize; 4] = [1, 16, 256, 4096];

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub(crate) struct AcceleratorManifest {
    schema_version: u32,
    id: String,
    selected: bool,
    status: String,
    runtime: String,
    execution_boundary: String,
    core_dependency: bool,
    requires_nvidia_gpu: bool,
    task_spec_schema: u32,
    batch_checkpoint_schema: u32,
    supported_batch_widths: Vec<usize>,
    selection_adr: String,
    official_sources: Vec<String>,
}

pub(crate) fn accelerator_check(args: &mut impl Iterator<Item = String>) -> Result<()> {
    anyhow::ensure!(
        args.next().is_none(),
        "accelerator-check accepts no arguments"
    );
    let root = super::workspace_root()?;
    let manifest = validate_selected_manifest(&root)?;
    println!(
        "accelerator selection ok: id={} runtime={} status={}",
        manifest.id, manifest.runtime, manifest.status
    );
    Ok(())
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
