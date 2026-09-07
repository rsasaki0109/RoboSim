//! Fixed-period operation replay CLI; all file reads are bounded.

use anyhow::{bail, ensure, Context, Result};
use rne_mobility_benchmark::observed_fixed_batch::{
    decode_fixed_batch_replay, record_fixed_batch_replay, verify_fixed_batch_replay,
    FixedBatchOperation, MAX_FIXED_REPLAY_BYTES,
};
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use std::{
    io::{Read, Write},
    path::Path,
};

pub(crate) fn run(
    mode: &str,
    input: &Path,
    output: Option<&Path>,
    seed: Option<u64>,
    width: Option<usize>,
    workers: usize,
) -> Result<()> {
    let record = match mode {
        "fixed-record-rapier" | "fixed-record-mujoco" => true,
        "fixed-verify-rapier" | "fixed-verify-mujoco" => false,
        _ => bail!("unknown fixed replay command {mode}"),
    };
    if record {
        ensure!(
            output.is_some() && seed.is_some() && width.is_some(),
            "fixed record requires --output, --seed and --num-envs"
        );
    } else {
        ensure!(
            output.is_none() && seed.is_none() && width.is_none(),
            "fixed verify accepts only --input and --workers"
        );
    }
    let mut bytes = Vec::new();
    std::fs::File::open(input)
        .with_context(|| format!("open {}", input.display()))?
        .take(MAX_FIXED_REPLAY_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() <= MAX_FIXED_REPLAY_BYTES,
        "fixed input exceeds byte limit"
    );
    let process = |factory| execute(factory, record, &bytes, output, seed, width, workers);
    if mode.ends_with("-rapier") {
        use rne_physics_rapier::RapierBackend;
        process(|| Ok((RapierBackend::new(), RapierBackend::manifest())))
    } else {
        #[cfg(feature = "mujoco")]
        {
            use rne_physics_mujoco::MuJoCoBackend;
            execute(
                || {
                    Ok((
                        MuJoCoBackend::new(rne_core::SimDuration::from_ticks(1_000_000))?,
                        MuJoCoBackend::manifest(),
                    ))
                },
                record,
                &bytes,
                output,
                seed,
                width,
                workers,
            )
        }
        #[cfg(not(feature = "mujoco"))]
        {
            bail!("fixed MuJoCo replay requires --features mujoco")
        }
    }
}

fn execute<B, F>(
    factory: F,
    record: bool,
    bytes: &[u8],
    output: Option<&Path>,
    seed: Option<u64>,
    width: Option<usize>,
    workers: usize,
) -> Result<()>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    if record {
        let operations: Vec<FixedBatchOperation> =
            serde_json::from_slice(bytes).context("decode fixed operation array")?;
        let replay = record_fixed_batch_replay(
            factory,
            seed.context("seed required")?,
            width.context("width required")?,
            workers,
            &operations,
        )?;
        let path = output.context("output required")?;
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .with_context(|| format!("create new replay {}", path.display()))?;
        file.write_all(&serde_json::to_vec_pretty(&replay)?)?;
        file.write_all(b"\n")?;
        println!(
            "fixed replay evidence written: {} operations={} digest={}",
            path.display(),
            replay.events.len(),
            replay.content_sha256
        );
    } else {
        let replay = decode_fixed_batch_replay(bytes)?;
        verify_fixed_batch_replay(factory, workers, &replay)?;
        println!(
            "fixed physical replay exact: operations={} digest={}",
            replay.events.len(),
            replay.content_sha256
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_records_verifies_and_never_overwrites_existing_evidence() {
        let directory = tempfile::tempdir().unwrap();
        let operations = directory.path().join("operations.json");
        std::fs::write(
            &operations,
            include_bytes!("../fixtures/fixed-batch-operations.json"),
        )
        .unwrap();
        let output = directory.path().join("replay.json");
        run(
            "fixed-record-rapier",
            &operations,
            Some(&output),
            Some(42),
            Some(2),
            1,
        )
        .unwrap();
        let original = std::fs::read(&output).unwrap();
        run("fixed-verify-rapier", &output, None, None, None, 2).unwrap();
        assert!(run(
            "fixed-record-rapier",
            &operations,
            Some(&output),
            Some(43),
            Some(2),
            2
        )
        .is_err());
        assert_eq!(std::fs::read(&output).unwrap(), original);
        assert!(run("fixed-verify-rapier", &output, None, Some(42), None, 2).is_err());
        assert!(run(
            "fixed-record-rapier",
            &operations,
            Some(&output),
            None,
            Some(2),
            2
        )
        .is_err());
        assert!(run("fixed-unknown", &operations, None, None, None, 1).is_err());
    }
}
