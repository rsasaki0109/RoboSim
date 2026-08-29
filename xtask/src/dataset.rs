//! Headless dataset bundle verification and offline evaluation commands.

use anyhow::{Context, Result};
use rne_data::{
    DatasetBundle, DatasetVerificationReport, DepthPairMetricSpec, RendererDatasetCaptureReport,
    StreamId,
};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fs;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::{Path, PathBuf};

pub(crate) const RENDERER_CAPTURE_REPORT_SCHEMA_VERSION: u32 =
    rne_data::RENDERER_DATASET_CAPTURE_REPORT_SCHEMA_VERSION;
const RENDERER_CAPTURE_REPORT: &str = "renderer-capture-report.json";
const WGPU_G1_DATASET_ID: &str = "rne-unitree-g1-wgpu-rgbd-v1";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RendererContract {
    kind: String,
    schema_version: u32,
    backend: String,
    capture_mode: String,
}

pub(crate) fn dataset_reference_smoke() -> Result<()> {
    let root = super::workspace_root()?;
    let relative = PathBuf::from(format!(
        "target/rne-dataset-reference-smoke-{}",
        std::process::id()
    ));
    let output = root.join(&relative);
    anyhow::ensure!(
        !output.exists(),
        "dataset reference smoke target already exists: {}",
        output.display()
    );
    let portable = relative.to_string_lossy().replace('\\', "/");
    let capture = super::run_step(&format!(
        "cargo run --locked -p diff_drive_dataset_capture --example 73_diff_drive_dataset_capture -- {portable} --verify-golden"
    ));
    if let Err(error) = capture {
        if output.exists() {
            std::fs::remove_dir_all(&output)
                .with_context(|| format!("remove failed reference capture {}", output.display()))?;
        }
        return Err(error);
    }

    let verification = DatasetBundle::open(&output)
        .and_then(|bundle| bundle.verify())
        .with_context(|| format!("verify generated reference capture {}", output.display()));
    std::fs::remove_dir_all(&output)
        .with_context(|| format!("remove reference capture {}", output.display()))?;
    let verification = verification?;
    anyhow::ensure!(
        verification.stream_count == 8
            && verification.record_count == 471
            && verification.sample_count == 471
            && verification.dropped_count == 0,
        "reference capture counts changed: {verification:?}"
    );
    println!(
        "dataset reference smoke ok: records={} manifest={}",
        verification.record_count, verification.manifest_sha256
    );
    Ok(())
}

pub(crate) fn dataset_check(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = super::workspace_root()?;
    let bundle_argument = args
        .next()
        .ok_or_else(|| anyhow::anyhow!("dataset-check requires a bundle directory"))?;
    anyhow::ensure!(
        args.next().is_none(),
        "dataset-check accepts exactly one bundle directory"
    );
    let path = resolve_path(&root, &bundle_argument);
    let bundle = DatasetBundle::open(&path)
        .with_context(|| format!("open dataset bundle {}", path.display()))?;
    let report = bundle
        .verify()
        .with_context(|| format!("verify dataset bundle {}", path.display()))?;
    if bundle.manifest().dataset_id == WGPU_G1_DATASET_ID {
        anyhow::ensure!(
            path.join(RENDERER_CAPTURE_REPORT).is_file(),
            "WGPU G1 renderer dataset is missing its capture report"
        );
        verify_renderer_capture(&path, &root, &bundle, &report)?;
    } else if path.join(RENDERER_CAPTURE_REPORT).exists() {
        verify_renderer_capture(&path, &root, &bundle, &report)?;
    }
    println!(
        "dataset verified: id={} streams={} records={} samples={} dropped={} manifest={}",
        bundle.manifest().dataset_id,
        report.stream_count,
        report.record_count,
        report.sample_count,
        report.dropped_count,
        report.manifest_sha256
    );
    Ok(())
}

fn verify_renderer_capture(
    dataset_root: &Path,
    workspace_root: &Path,
    bundle: &DatasetBundle,
    verification: &DatasetVerificationReport,
) -> Result<()> {
    let report_bytes = read_regular_file(&dataset_root.join(RENDERER_CAPTURE_REPORT), 64 * 1024)?;
    let report: RendererDatasetCaptureReport = serde_json::from_slice(&report_bytes)?;
    validate_wgpu_renderer_report(&report, verification, &bundle.manifest().content_sha256)?;

    let task_asset = bundle
        .manifest()
        .assets
        .iter()
        .find(|asset| asset.role == "task_spec")
        .context("renderer dataset has no TaskSpec asset")?;
    anyhow::ensure!(
        task_asset.path == "task-spec.json",
        "unexpected TaskSpec path"
    );
    let task_bytes = read_regular_file(&dataset_root.join(&task_asset.path), 1024 * 1024)?;
    let task_digest = sha256(&task_bytes);
    anyhow::ensure!(
        task_digest == task_asset.sha256
            && task_digest == report.task_spec_sha256
            && task_digest == bundle.manifest().task_spec_sha256,
        "renderer dataset TaskSpec identity mismatch"
    );
    let task: rne_ai::TaskSpec = serde_json::from_slice(&task_bytes)?;
    task.validate()
        .context("validate renderer dataset TaskSpec")?;

    let renderer_asset = bundle
        .manifest()
        .assets
        .iter()
        .find(|asset| asset.role == "renderer_contract")
        .context("renderer dataset has no renderer contract asset")?;
    anyhow::ensure!(
        renderer_asset.path == "renderer-contract.json",
        "unexpected renderer contract path"
    );
    let renderer_bytes = read_regular_file(&dataset_root.join(&renderer_asset.path), 64 * 1024)?;
    anyhow::ensure!(
        sha256(&renderer_bytes) == renderer_asset.sha256,
        "renderer contract digest mismatch"
    );
    let renderer: RendererContract = serde_json::from_slice(&renderer_bytes)?;
    anyhow::ensure!(
        renderer.kind == "rne_renderer_capture_contract"
            && renderer.schema_version == 1
            && renderer.backend == report.renderer
            && renderer.capture_mode == "offscreen_rgbd",
        "renderer contract identity is invalid"
    );

    let workspace_assets = bundle
        .manifest()
        .assets
        .iter()
        .filter(|asset| asset.role != "task_spec" && asset.role != "renderer_contract")
        .collect::<Vec<_>>();
    let roles = workspace_assets
        .iter()
        .map(|asset| asset.role.as_str())
        .collect::<BTreeSet<_>>();
    anyhow::ensure!(
        workspace_assets.len() >= 10
            && [
                "scene",
                "robot_model",
                "robot_source",
                "environment",
                "renderer_texture"
            ]
            .into_iter()
            .all(|role| roles.contains(role)),
        "renderer dataset does not bind its complete workspace input families"
    );
    let canonical_workspace = workspace_root.canonicalize()?;
    for asset in workspace_assets {
        let source = workspace_root.join(Path::new(&asset.path));
        let source_metadata = fs::symlink_metadata(&source)
            .with_context(|| format!("inspect renderer source asset {}", asset.path))?;
        anyhow::ensure!(
            source_metadata.file_type().is_file() && !source_metadata.file_type().is_symlink(),
            "renderer source asset must be a regular non-symlink file: {}",
            asset.path
        );
        let canonical_source = source
            .canonicalize()
            .with_context(|| format!("resolve renderer source asset {}", asset.path))?;
        anyhow::ensure!(
            canonical_source.starts_with(&canonical_workspace),
            "renderer source asset escapes the workspace: {}",
            asset.path
        );
        anyhow::ensure!(
            sha256_regular_file(&source, 256 * 1024 * 1024)? == asset.sha256,
            "renderer source asset digest mismatch: {}",
            asset.path
        );
    }
    println!(
        "renderer capture verified: renderer={} frames={} assets={} task={}",
        report.renderer,
        report.frame_count,
        bundle.manifest().assets.len(),
        report.task_spec_sha256
    );
    Ok(())
}

fn validate_wgpu_renderer_report(
    report: &RendererDatasetCaptureReport,
    verification: &DatasetVerificationReport,
    manifest_sha256: &str,
) -> Result<()> {
    report.validate_against(manifest_sha256, verification)?;
    anyhow::ensure!(
        report.renderer == "rne_render_wgpu",
        "renderer capture report identity is invalid"
    );
    Ok(())
}

fn read_regular_file(path: &Path, maximum_bytes: u64) -> Result<Vec<u8>> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect renderer evidence {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink(),
        "renderer evidence must be a regular non-symlink file: {}",
        path.display()
    );
    anyhow::ensure!(
        metadata.len() <= maximum_bytes,
        "renderer evidence exceeds its byte limit: {}",
        path.display()
    );
    fs::read(path).with_context(|| format!("read renderer evidence {}", path.display()))
}

fn sha256_regular_file(path: &Path, maximum_bytes: u64) -> Result<String> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect renderer source asset {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink(),
        "renderer source asset must be a regular non-symlink file: {}",
        path.display()
    );
    anyhow::ensure!(
        metadata.len() <= maximum_bytes,
        "renderer source asset exceeds its byte limit: {}",
        path.display()
    );
    let mut reader = BufReader::new(File::open(path)?);
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let count = reader.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        hasher.update(&buffer[..count]);
    }
    Ok(format!("sha256:{:x}", hasher.finalize()))
}

fn sha256(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

pub(crate) fn dataset_evaluate_depth(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let root = super::workspace_root()?;
    let bundle_argument = args.next().ok_or_else(|| {
        anyhow::anyhow!(
            "dataset-evaluate-depth requires BUNDLE PREDICTED_STREAM GROUND_TRUTH_STREAM TOLERANCE_M [OUTPUT]"
        )
    })?;
    let predicted = parse_stream(args.next(), "predicted stream")?;
    let ground_truth = parse_stream(args.next(), "ground-truth stream")?;
    let tolerance = args
        .next()
        .ok_or_else(|| anyhow::anyhow!("missing tolerance in metres"))?
        .parse::<f64>()
        .context("parse tolerance in metres")?;
    let output = args.next();
    anyhow::ensure!(
        args.next().is_none(),
        "dataset-evaluate-depth accepts at most one output path"
    );

    let bundle_path = resolve_path(&root, &bundle_argument);
    let bundle = DatasetBundle::open(&bundle_path)
        .with_context(|| format!("open dataset bundle {}", bundle_path.display()))?;
    bundle
        .verify()
        .with_context(|| format!("verify dataset bundle {}", bundle_path.display()))?;
    let report = bundle.evaluate_depth_pair(DepthPairMetricSpec {
        predicted_stream: predicted,
        ground_truth_stream: ground_truth,
        tolerance_m: tolerance,
    })?;
    bundle.verify_depth_pair_report(&report)?;
    if let Some(output) = output {
        let output = resolve_path(&root, &output);
        report
            .write_json(&output)
            .with_context(|| format!("write evaluation report {}", output.display()))?;
        println!(
            "depth evaluation written: passed={} frames={} pixels={} max_error_m={} output={}",
            report.passed,
            report.compared_frames,
            report.compared_pixels,
            report.max_absolute_error_m,
            output.display()
        );
    } else {
        println!("{}", serde_json::to_string_pretty(&report)?);
    }
    anyhow::ensure!(report.passed, "depth evaluation exceeded tolerance");
    Ok(())
}

fn parse_stream(value: Option<String>, label: &str) -> Result<StreamId> {
    let value = value.ok_or_else(|| anyhow::anyhow!("missing {label}"))?;
    Ok(StreamId::new(
        value
            .parse::<u64>()
            .with_context(|| format!("parse {label}"))?,
    ))
}

fn resolve_path(root: &Path, value: &str) -> PathBuf {
    let path = PathBuf::from(value);
    if path.is_absolute() {
        path
    } else {
        root.join(path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renderer_capture_report_rejects_identity_digest_and_count_drift() {
        let manifest = format!("sha256:{}", "a".repeat(64));
        let verification = DatasetVerificationReport {
            schema_version: 1,
            manifest_sha256: manifest.clone(),
            stream_count: 2,
            record_count: 24,
            sample_count: 24,
            dropped_count: 0,
            passed: true,
        };
        let report = RendererDatasetCaptureReport {
            kind: "rne_renderer_dataset_capture_report".into(),
            schema_version: RENDERER_CAPTURE_REPORT_SCHEMA_VERSION,
            status: "passed".into(),
            renderer: "rne_render_wgpu".into(),
            dataset_manifest_sha256: manifest.clone(),
            task_spec_sha256: format!("sha256:{}", "b".repeat(64)),
            stream_count: 2,
            record_count: 24,
            sample_count: 24,
            frame_count: 12,
        };
        validate_wgpu_renderer_report(&report, &verification, &manifest).unwrap();

        let mut wrong_renderer = report.clone();
        wrong_renderer.renderer = "headless".into();
        assert!(validate_wgpu_renderer_report(&wrong_renderer, &verification, &manifest).is_err());
        let mut wrong_digest = report.clone();
        wrong_digest.dataset_manifest_sha256 = format!("sha256:{}", "c".repeat(64));
        assert!(validate_wgpu_renderer_report(&wrong_digest, &verification, &manifest).is_err());
        let mut wrong_count = report;
        wrong_count.record_count = 22;
        assert!(validate_wgpu_renderer_report(&wrong_count, &verification, &manifest).is_err());

        let unknown = format!(
            "{{\"kind\":\"rne_renderer_dataset_capture_report\",\"schema_version\":1,\"status\":\"passed\",\"renderer\":\"rne_render_wgpu\",\"dataset_manifest_sha256\":\"{manifest}\",\"task_spec_sha256\":\"sha256:{}\",\"stream_count\":2,\"record_count\":24,\"sample_count\":24,\"frame_count\":12,\"unknown\":true}}",
            "b".repeat(64)
        );
        assert!(serde_json::from_str::<RendererDatasetCaptureReport>(&unknown).is_err());
    }
}
