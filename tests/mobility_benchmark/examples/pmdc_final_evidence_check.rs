//! Verify the sole retained PMDC final result without rerunning its rollout.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_pmdc::final_evaluation::{
    decode_pmdc_final_evaluation, PMDC_FINAL_EVALUATION_ARTIFACT_BYTES,
};
use serde_json::json;
use std::{fs::File, io::Read, path::PathBuf};

fn main() -> Result<()> {
    let input = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .context("usage: pmdc_final_evidence_check <final-evidence.json>")?;
    ensure!(std::env::args_os().nth(2).is_none(), "unexpected argument");
    let metadata = input.metadata().context("stat PMDC final evidence")?;
    ensure!(
        metadata.is_file() && metadata.len() == PMDC_FINAL_EVALUATION_ARTIFACT_BYTES as u64,
        "PMDC final evidence byte length drift"
    );
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    File::open(input)?
        .take(PMDC_FINAL_EVALUATION_ARTIFACT_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    let evidence = decode_pmdc_final_evaluation(&bytes)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "kind": "rne_pmdc_final_evaluation_check",
            "content_sha256": evidence.content_sha256,
            "gate_count": evidence.metrics.len(),
            "passed": evidence.passed,
            "reran_rollout": false,
        }))?
    );
    Ok(())
}
