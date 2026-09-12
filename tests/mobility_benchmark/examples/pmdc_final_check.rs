//! Verify the sealed PMDC final partition without evaluating response values.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_pmdc::{
    data::{decode_pmdc_final, MAX_PMDC_FINAL_BYTES},
    final_protocol::{PMDC_FINAL_ARTIFACT_SHA256, PMDC_FINAL_RECORDS_SHA256},
};
use serde_json::json;
use std::{
    fs::File,
    io::Read,
    path::{Path, PathBuf},
};

fn read_bounded(path: &Path) -> Result<Vec<u8>> {
    let metadata = path.metadata().context("stat PMDC final artifact")?;
    ensure!(
        metadata.is_file() && metadata.len() <= MAX_PMDC_FINAL_BYTES as u64,
        "PMDC final artifact is not a bounded regular file"
    );
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    File::open(path)?
        .take(MAX_PMDC_FINAL_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= MAX_PMDC_FINAL_BYTES,
        "PMDC final artifact grew during read"
    );
    Ok(bytes)
}

fn main() -> Result<()> {
    let input = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .context("usage: pmdc_final_check <final.jsonl>")?;
    ensure!(std::env::args_os().nth(2).is_none(), "unexpected argument");
    let final_set = decode_pmdc_final(&read_bounded(&input)?)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "kind": "rne_pmdc_final_partition_check",
            "final_protocol_sha256": final_set.final_protocol_sha256,
            "artifact_sha256": PMDC_FINAL_ARTIFACT_SHA256,
            "records_sha256": PMDC_FINAL_RECORDS_SHA256,
            "run_ids": final_set.runs.iter().map(|run| &run.run_id).collect::<Vec<_>>(),
            "sample_counts": final_set.runs.iter().map(|run| run.samples.len()).collect::<Vec<_>>(),
            "final_responses_evaluated": false,
        }))?
    );
    Ok(())
}
