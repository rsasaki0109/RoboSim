//! Headless dataset bundle verification and offline evaluation commands.

use anyhow::{Context, Result};
use rne_data::{DatasetBundle, DepthPairMetricSpec, StreamId};
use std::path::{Path, PathBuf};

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
