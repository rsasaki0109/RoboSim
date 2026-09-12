//! Execute and exclusively retain the one permitted PMDC final evaluation.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_pmdc::{
    data::{
        decode_pmdc_final, decode_pmdc_training, MAX_PMDC_FINAL_BYTES, MAX_PMDC_TRAINING_BYTES,
    },
    final_evaluation::evaluate_pmdc_final,
    identification::identify_pmdc_training,
};
use std::{
    fs::{File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
};

fn read_bounded(path: &Path, maximum: usize) -> Result<Vec<u8>> {
    let metadata = path.metadata().context("stat PMDC partition")?;
    ensure!(
        metadata.is_file() && metadata.len() <= maximum as u64,
        "PMDC partition is not a bounded regular file"
    );
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    File::open(path)?
        .take(maximum as u64 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(bytes.len() <= maximum, "PMDC partition grew during read");
    Ok(bytes)
}

fn main() -> Result<()> {
    let training_path = std::env::args_os().nth(1).map(PathBuf::from).context(
        "usage: pmdc_final_evaluation <training.jsonl> <final.jsonl> <new-evidence.json>",
    )?;
    let final_path = std::env::args_os()
        .nth(2)
        .map(PathBuf::from)
        .context("missing final input")?;
    let output_path = std::env::args_os()
        .nth(3)
        .map(PathBuf::from)
        .context("missing new evidence output")?;
    ensure!(std::env::args_os().nth(4).is_none(), "unexpected argument");

    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)
        .context("reserve one-shot PMDC final evidence path")?;
    let training = decode_pmdc_training(&read_bounded(&training_path, MAX_PMDC_TRAINING_BYTES)?)?;
    let identification = identify_pmdc_training(&training)?;
    let final_set = decode_pmdc_final(&read_bounded(&final_path, MAX_PMDC_FINAL_BYTES)?)?;
    let evidence = evaluate_pmdc_final(&training, &identification, &final_set)?;
    output.write_all(&serde_json::to_vec_pretty(&evidence)?)?;
    output.write_all(b"\n")?;
    println!("{}", serde_json::to_string_pretty(&evidence)?);
    ensure!(
        evidence.passed,
        "PMDC final gates failed; evidence retained"
    );
    Ok(())
}
