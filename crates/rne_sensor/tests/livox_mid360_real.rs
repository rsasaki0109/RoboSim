//! Checks the Livox Mid-360 model against measurements from real recordings.

use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    ContactEvent, PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc,
    PhysicsWorldId, RaycastHit, RaycastQuery,
};
use rne_sensor::{
    livox_mid360_spec, sample_livox_mid360, LidarRigOcclusion, LidarSweep, LivoxMid360Pattern,
    SensorNoiseKey,
};
use rne_world::Transform3;
use serde::Deserialize;
use std::path::PathBuf;

#[derive(Deserialize)]
struct DirectionFixture {
    reference_median_error_deg: f64,
    samples: Vec<DirectionSample>,
}

#[derive(Deserialize)]
struct DirectionSample {
    firing: u64,
    line: u16,
    rotor_phase_offset_rad: f64,
    nod_phase_offset_rad: f64,
    direction: [f64; 3],
}

fn repo_path(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(relative)
}

fn go2_rig() -> LidarRigOcclusion {
    let text = std::fs::read_to_string(repo_path(
        "../../assets/sensors/livox_mid360/go2_rig_occlusion.json",
    ))
    .expect("rig asset");
    serde_json::from_str(&text).expect("rig json")
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}

#[test]
fn pattern_matches_held_out_real_returns() {
    // 1,500 returns from a recording the shape fit never saw, with the per-frame phase
    // offsets tracked on that recording.
    let text = std::fs::read_to_string(repo_path(
        "tests/fixtures/livox_mid360_eil_mask2_directions.json",
    ))
    .expect("fixture");
    let fixture: DirectionFixture = serde_json::from_str(&text).expect("fixture json");
    let errors_deg = fixture
        .samples
        .iter()
        .map(|sample| {
            let predicted = LivoxMid360Pattern::new()
                .with_phase_offsets_rad(sample.rotor_phase_offset_rad, sample.nod_phase_offset_rad)
                .direction(sample.firing, sample.line);
            let measured = Vec3::new(
                sample.direction[0],
                sample.direction[1],
                sample.direction[2],
            )
            .normalize();
            predicted.dot(measured).clamp(-1.0, 1.0).acos().to_degrees()
        })
        .collect::<Vec<_>>();
    let median_deg = median(errors_deg);
    // The Rust evaluation reproduces the fitting script, and the error stays below the
    // datasheet's 0.15° angular precision.
    assert!(
        (median_deg - fixture.reference_median_error_deg).abs() < 1e-3,
        "median {median_deg} vs reference {}",
        fixture.reference_median_error_deg
    );
    assert!(median_deg < 0.15, "median error {median_deg} deg");
}

#[test]
fn rig_blocks_the_four_mount_posts() {
    // Four posts 90° apart block every ray at low elevation in both recordings.
    let rig = go2_rig();
    for post_deg in [-136.0, -46.0, 44.0, 134.0] {
        let azimuth_bin = ((post_deg - rig.min_azimuth_deg) / rig.azimuth_bin_deg) as u32;
        let elevation_bin = ((10.0 - rig.min_elevation_deg) / rig.elevation_bin_deg) as u32;
        let cell = rig
            .cells
            .iter()
            .find(|cell| cell.azimuth_bin == azimuth_bin && cell.elevation_bin == elevation_bin)
            .unwrap_or_else(|| panic!("no rig cell at {post_deg} deg"));
        assert!(cell.block_probability > 0.9, "{post_deg}: {cell:?}");
    }
}

/// A backend that never hits anything, so every return comes from the rig.
struct EmptyPhysics;

impl PhysicsBackend for EmptyPhysics {
    type BodyHandle = ();
    type ColliderHandle = ();

    fn create_world(&mut self, _: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
        Ok(PhysicsWorldId::DEFAULT)
    }
    fn sync_from_ecs(&mut self, _: &mut World, _: PhysicsWorldId) -> Result<(), PhysicsError> {
        Ok(())
    }
    fn step(&mut self, _: PhysicsWorldId, _: SimDuration) -> Result<(), PhysicsError> {
        Ok(())
    }
    fn sync_to_ecs(&mut self, _: &mut World, _: PhysicsWorldId) -> Result<(), PhysicsError> {
        Ok(())
    }
    fn raycast(&self, _: PhysicsWorldId, _: RaycastQuery) -> Result<Vec<RaycastHit>, PhysicsError> {
        Ok(Vec::new())
    }
    fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        Ok(&[])
    }
    fn capabilities(&self) -> &[PhysicsCapability] {
        &[]
    }
}

#[test]
fn self_returns_match_the_recorded_rate_and_replay() {
    // Real Go2 recordings return 5.49 % (EIL_Box) and 5.48 % (EIL_Mask2) of all slots
    // from the robot itself within 0.35 m; the model returns 5.37 % over these frames.
    let rig = go2_rig();
    let spec = livox_mid360_spec();
    let world = World::new();
    let sweep = LidarSweep::stationary(Transform3::from_translation_rotation(
        Vec3::ZERO,
        Quat::IDENTITY,
    ));
    let pattern = LivoxMid360Pattern::new();
    let mut returns = 0_usize;
    let mut slots = 0_usize;
    for frame in 0..10 {
        let key = SensorNoiseKey::new(7, spec.seed, 1, frame);
        let cloud = sample_livox_mid360(
            &EmptyPhysics,
            PhysicsWorldId::DEFAULT,
            &world,
            &sweep,
            &spec,
            &pattern,
            frame,
            Some(&rig),
            key,
        );
        assert!(cloud.points_m.iter().all(|point| point.length() < 0.35));
        assert!(cloud.attributes_are_aligned());
        let replay = sample_livox_mid360(
            &EmptyPhysics,
            PhysicsWorldId::DEFAULT,
            &world,
            &sweep,
            &spec,
            &pattern,
            frame,
            Some(&rig),
            key,
        );
        assert_eq!(cloud, replay);
        returns += cloud.points_m.len();
        slots += pattern.frame_rays(frame).len();
    }
    let fraction = returns as f64 / slots as f64;
    assert!(
        (0.050..0.060).contains(&fraction),
        "self-return fraction {fraction}"
    );
}
