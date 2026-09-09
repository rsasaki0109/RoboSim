//! Read-only raw-log audit; no calibration fit or physical qualification.

use anyhow::{ensure, Result};
use rne_mobility_benchmark::recorded_openmct::read_openmct;
use serde_json::json;
use std::{collections::BTreeSet, fs::File};

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    ensure!(args.len() == 1, "usage: openmct_audit <raw-log.txt>");
    let source = read_openmct(File::open(&args[0])?)?;
    let mut ids = BTreeSet::new();
    let mut missing = 0;
    let mut reused = 0;
    let mut minimum_age_ms = f64::INFINITY;
    let mut maximum_age_ms = f64::NEG_INFINITY;
    for row in source.samples() {
        if let Some(dmm) = &row.dmm {
            if !ids.insert(dmm.sample_id) {
                reused += 1;
            }
            minimum_age_ms = minimum_age_ms.min(dmm.age_ms);
            maximum_age_ms = maximum_age_ms.max(dmm.age_ms);
        } else {
            missing += 1;
        }
    }
    let ages = if ids.is_empty() {
        None
    } else {
        Some([minimum_age_ms, maximum_age_ms])
    };
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "kind": "rne_openmct_source_audit", "schema_version": 1,
            "source_sha256": source.source_sha256(), "mode": source.mode(),
            "declared_date": source.date(), "rows": source.samples().len(),
            "missing_dmm_rows": missing, "unique_dmm_ids": ids.len(),
            "reused_dmm_rows": reused, "dmm_age_range_ms": ages,
            "physical_calibration_qualified": false
        }))?
    );
    Ok(())
}
