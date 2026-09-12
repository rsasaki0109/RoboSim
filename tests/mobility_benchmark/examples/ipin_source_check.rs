//! Read-only bounded IPIN static IMU audit; use '-' for a decompressed stdin stream.
use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_ipin::audit_ipin_imu;
use std::{
    env,
    fs::File,
    io::{self, BufReader},
};

fn main() -> Result<()> {
    let args: Vec<_> = env::args().skip(1).collect();
    ensure!(args.len() == 1, "usage: ipin_source_check <csv-path|->");
    let report = if args[0] == "-" {
        audit_ipin_imu(io::stdin().lock())?
    } else {
        audit_ipin_imu(BufReader::new(
            File::open(&args[0]).context("open IPIN source")?,
        ))?
    };
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
