//! Export preliminary caster closed-loop evidence without overwriting prior runs.
use anyhow::{bail, Context, Result};
use rne_mobility_benchmark::diff_caster_observed::{
    nominal_caster_feedforward_spec, run_caster_observed_with_control_spec,
};
use rne_physics_rapier::RapierBackend;
use std::io::Write;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let backend = args.next().context("expected rapier or mujoco")?;
    let path = args.next().context("expected external output path")?;
    let mut blackout = false;
    let mut feedforward = false;
    for argument in args {
        match argument.as_str() {
            "--imu-blackout" if !blackout => blackout = true,
            "--nominal-feedforward" if !feedforward => feedforward = true,
            _ => bail!("unknown or repeated option"),
        }
    }
    let spec = if feedforward {
        nominal_caster_feedforward_spec()
    } else {
        Default::default()
    };
    if std::path::Path::new(&path).exists() {
        bail!("refusing to overwrite evidence");
    }
    let run = match backend.as_str() {
        "rapier" => run_caster_observed_with_control_spec(
            RapierBackend::new(),
            RapierBackend::manifest(),
            blackout,
            spec,
        )?,
        #[cfg(feature = "mujoco")]
        "mujoco" => {
            use rne_physics_mujoco::MuJoCoBackend;
            run_caster_observed_with_control_spec(
                MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    rne_mobility_benchmark::diff_caster::DIFF_CASTER_FIXED_DELTA_TICKS,
                ))?,
                MuJoCoBackend::manifest(),
                blackout,
                spec,
            )?
        }
        _ => bail!("unsupported backend (MuJoCo requires --features mujoco)"),
    };
    let bytes = serde_json::to_vec_pretty(&run)?;
    let mut file = std::fs::OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(&path)?;
    file.write_all(&bytes)?;
    file.sync_all()?;
    println!("{} decisions written to {path}", run.samples.len());
    Ok(())
}
