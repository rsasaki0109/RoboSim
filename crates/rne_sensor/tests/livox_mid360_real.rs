//! Checks the Livox Mid-360 model against measurements from real recordings.

use rne_core::SimDuration;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::{
    ContactEvent, PhysicsBackend, PhysicsCapability, PhysicsError, PhysicsWorldDesc,
    PhysicsWorldId, RaycastHit, RaycastQuery,
};
use rne_sensor::{
    livox_mid360_near_blanking_probability, livox_mid360_spec, sample_livox_mid360,
    LidarRigOcclusion, LidarSweep, LivoxMid360Pattern, SensorNoiseKey,
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
    // from the robot itself within 0.35 m; the model returns 5.42 % over these frames.
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

/// A flat floor at `y = 0` made of one entity.
struct FloorPhysics {
    entity: rne_ecs::Entity,
}

impl PhysicsBackend for FloorPhysics {
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
    fn raycast(
        &self,
        _: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        if query.direction.y >= -1e-9 {
            return Ok(Vec::new());
        }
        let distance_m = -query.origin_m.y / query.direction.y;
        if distance_m <= 0.0 || distance_m > query.max_distance_m {
            return Ok(Vec::new());
        }
        Ok(vec![RaycastHit {
            entity: self.entity,
            point_m: query.origin_m + query.direction * distance_m,
            normal: Vec3::Y,
            distance_m,
        }])
    }
    fn contacts(&self, _: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        Ok(&[])
    }
    fn capabilities(&self) -> &[PhysicsCapability] {
        &[]
    }
}

/// Per 4° Livox-elevation band from 24°: (no-return fraction, self-return fraction).
fn floor_bands(material: rne_sensor::LidarMaterial, frames: u64) -> Vec<(f64, f64)> {
    let mut world = World::new();
    let floor = rne_ecs::spawn_named(&mut world, "floor");
    world.entity_mut(floor).insert(material);
    let physics = FloorPhysics { entity: floor };
    let rig = go2_rig();
    let spec = livox_mid360_spec();
    // Upside down at the measured 0.447 m: Livox +z points at the floor.
    let pose = Transform3::from_translation_rotation(
        Vec3::new(0.0, 0.447, 0.0),
        Quat::from_rotation_x(std::f64::consts::PI),
    );
    let sweep = LidarSweep::stationary(pose);
    let pattern = LivoxMid360Pattern::new();
    let mut slots = [0_usize; 7];
    let mut empty = [0_usize; 7];
    let mut selfs = [0_usize; 7];
    for frame in 0..frames {
        let rays = pattern.frame_rays(frame);
        let cloud = sample_livox_mid360(
            &physics,
            PhysicsWorldId::DEFAULT,
            &world,
            &sweep,
            &spec,
            &pattern,
            frame,
            Some(&rig),
            SensorNoiseKey::new(3, spec.seed, 2, frame),
        );
        let mut returned = std::collections::HashMap::new();
        for (index, point) in cloud.points_m.iter().enumerate() {
            let key = (cloud.ray_indices[index], cloud.channel_indices[index]);
            returned.insert(key, (*point - pose.translation).length());
        }
        for ray in &rays {
            // Livox elevation is the engine elevation (both are asin of the up component).
            let band = ((ray.elevation_rad.to_degrees() - 24.0) / 4.0).floor();
            if !(0.0..7.0).contains(&band) {
                continue;
            }
            let band = band as usize;
            slots[band] += 1;
            match returned.get(&(ray.column, ray.channel)) {
                None => empty[band] += 1,
                Some(range_m) if *range_m < 0.35 => selfs[band] += 1,
                Some(_) => {}
            }
        }
    }
    (0..7)
        .map(|band| {
            (
                empty[band] as f64 / slots[band] as f64,
                selfs[band] as f64 / slots[band] as f64,
            )
        })
        .collect()
}

#[test]
fn steep_floor_bands_match_the_recordings() {
    // With the sensor upside down 0.447 m above a flat floor, every ray from 24° to 52°
    // Livox elevation lands on the floor within 1.1 m or on the robot, so these bands
    // compare with the recordings without modelling the rooms. Floor returns this
    // close saturate, so the floor material does not matter here; the loss comes from
    // the rig table and near-range blanking.
    //
    // Recorded no-return fraction per 4° band (EIL_Box; EIL_Mask2 agrees from 40° up
    // and is higher below, where its room leaves more directions empty) and self-return
    // fraction (mean of both recordings).
    const RECORDED_NO_RETURN: [f64; 7] = [0.338, 0.343, 0.410, 0.533, 0.673, 0.772, 0.991];
    const RECORDED_SELF: [f64; 7] = [0.065, 0.088, 0.127, 0.139, 0.111, 0.060, 0.001];
    let bands = floor_bands(rne_sensor::LidarMaterial::new(0.05, 0.0, 1.0), 10);
    for (index, (empty, selfs)) in bands.iter().enumerate() {
        let band_deg = 24 + 4 * index;
        assert!(
            (empty - RECORDED_NO_RETURN[index]).abs() < 0.04,
            "band {band_deg}: no-return {empty:.3} vs recorded {}",
            RECORDED_NO_RETURN[index]
        );
        assert!(
            (selfs - RECORDED_SELF[index]).abs() < 0.015,
            "band {band_deg}: self {selfs:.3} vs recorded {}",
            RECORDED_SELF[index]
        );
    }
}

#[test]
fn near_blanking_alternates_returns_on_a_close_surface() {
    // Recorded: a surface closer than 0.55 m returns on every other firing of a line.
    assert_eq!(livox_mid360_near_blanking_probability(0.3), 1.0);
    assert!(livox_mid360_near_blanking_probability(0.65) > 0.4);
    assert!(livox_mid360_near_blanking_probability(0.65) < 0.55);
    assert_eq!(livox_mid360_near_blanking_probability(1.0), 0.0);
}
