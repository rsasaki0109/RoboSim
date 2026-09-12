//! Produce content-bound training-only PMDC effective-identification evidence.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_pmdc::{
    data::{decode_pmdc_training, MAX_PMDC_TRAINING_BYTES},
    identification::identify_pmdc_training,
};
use std::{
    fs::{File, OpenOptions},
    io::{Read, Write},
    path::PathBuf,
};

fn main() -> Result<()> {
    let input = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .context("usage: pmdc_training_identification <training.jsonl> <new-evidence.json>")?;
    let output = std::env::args_os()
        .nth(2)
        .map(PathBuf::from)
        .context("missing new evidence output")?;
    ensure!(std::env::args_os().nth(3).is_none(), "unexpected argument");
    let metadata = input.metadata().context("stat PMDC training artifact")?;
    ensure!(
        metadata.is_file() && metadata.len() <= MAX_PMDC_TRAINING_BYTES as u64,
        "PMDC training artifact is not a bounded regular file"
    );
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    File::open(input)?
        .take(MAX_PMDC_TRAINING_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    let evidence = identify_pmdc_training(&decode_pmdc_training(&bytes)?)?;
    let rendered = serde_json::to_vec_pretty(&evidence)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)
        .context("exclusively create PMDC identification evidence")?;
    file.write_all(&rendered)?;
    file.write_all(b"\n")?;
    println!("{}", serde_json::to_string_pretty(&evidence)?);
    Ok(())
}
