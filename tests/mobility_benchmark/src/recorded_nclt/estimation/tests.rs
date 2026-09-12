use super::*;
use crate::recorded_nclt::replay::NcltReplay;
use rne_core::SimTime;

fn config() -> VelocityEstimatorConfig {
    VelocityEstimatorConfig {
        wheel_hold_us: 2_000_000,
        imu_hold_us: 2_000_000,
        wheel_scale: 1.0,
        gyro_z_bias_rad_s: 0.0,
        segment_origin_m: [0.0, 0.0],
        segment_heading_rad: 0.0,
        queue_capacity: 16,
    }
}

fn replay(times: &[u64], speeds: &[f64], yaw: f64, policy: NcltReplayPolicy) -> NcltReplay {
    let mut w = String::new();
    let mut i = String::new();
    for (&t, &v) in times.iter().zip(speeds) {
        let stamp = 1_350_000_000_000_000 + t;
        w.push_str(&format!("{stamp},{v},{v}\n"));
        i.push_str(&format!("{stamp},0,0,0,0,0,0,0,0,{yaw}\n"));
    }
    let entity = rne_ecs::spawn_named(&mut rne_ecs::World::new(), "synthetic");
    NcltReplay::new(w.as_bytes(), i.as_bytes(), policy, entity).unwrap()
}

fn run(mut replay: NcltReplay, cfg: VelocityEstimatorConfig, end_us: u64) -> VelocityEstimate {
    let policy = replay.policy();
    let mut estimator = MeasuredVelocityEstimator::new(replay.origin_us(), policy, cfg).unwrap();
    while let Some(time) = replay.next_delivery_time() {
        estimator.update(&replay.advance_to(time).unwrap()).unwrap();
    }
    let end = SimTime::from_ticks((end_us + policy.wheel_delay_us.max(policy.imu_delay_us)) * 1000);
    estimator.update(&replay.advance_to(end).unwrap()).unwrap()
}

#[test]
fn delayed_step_does_not_apply_future_speed_to_previous_interval() {
    let p = NcltReplayPolicy {
        wheel_delay_us: 100_000,
        imu_delay_us: 100_000,
    };
    let r = run(
        replay(&[0, 1_000_000, 2_000_000], &[1.0, 2.0, 4.0], 0.0, p),
        config(),
        2_000_000,
    );
    assert!((r.pose.unwrap().position_m[0] - 3.0).abs() < 1e-12);
    assert_eq!(r.estimate_time_ticks, Some(2_000_000_000));
    assert_eq!(r.decision_time_ticks, 2_100_000_000);
    assert_eq!(r.integrated_ticks, 2_000_000_000);
    assert_eq!(r.unobserved_ticks, 0);
    assert!(!r.uncertainty_calibrated);
}

#[test]
fn unequal_delay_reordering_and_terminal_flush_preserve_source_pose() {
    let a = run(
        replay(
            &[0, 500_000, 1_000_000],
            &[1.0, 2.0, 3.0],
            0.25,
            NcltReplayPolicy {
                wheel_delay_us: 0,
                imu_delay_us: 0,
            },
        ),
        config(),
        1_000_000,
    );
    let b = run(
        replay(
            &[0, 500_000, 1_000_000],
            &[1.0, 2.0, 3.0],
            0.25,
            NcltReplayPolicy {
                wheel_delay_us: 100_000,
                imu_delay_us: 900_000,
            },
        ),
        config(),
        1_000_000,
    );
    let ap = a.pose.unwrap();
    let bp = b.pose.unwrap();
    for (x, y) in ap.position_m.iter().zip(bp.position_m) {
        assert!((x - y).abs() < 1e-12);
    }
    assert!((ap.heading_rad - bp.heading_rad).abs() < 1e-12);
    assert_eq!(a.integrated_ticks, b.integrated_ticks);
    assert_eq!(a.unobserved_ticks, b.unobserved_ticks);
    assert_eq!(b.estimate_time_ticks, Some(1_000_000_000));
}

#[test]
fn stale_interval_splits_segments_without_fabricating_motion() {
    let cfg = VelocityEstimatorConfig {
        wheel_hold_us: 200_000,
        imu_hold_us: 200_000,
        ..config()
    };
    let r = run(
        replay(
            &[0, 300_000],
            &[1.0, 1.0],
            0.0,
            NcltReplayPolicy {
                wheel_delay_us: 0,
                imu_delay_us: 0,
            },
        ),
        cfg,
        400_000,
    );
    let pose = r.pose.unwrap();
    assert_eq!(pose.segment_id, 2);
    assert!((pose.position_m[0] - 0.1).abs() < 1e-12);
    assert_eq!(r.integrated_ticks, 300_000_000);
    assert_eq!(r.unobserved_ticks, 100_000_000);
}

#[test]
fn invalid_frames_and_backward_calls_are_transactional() {
    let p = NcltReplayPolicy {
        wheel_delay_us: 0,
        imu_delay_us: 0,
    };
    let mut r = replay(&[0, 100], &[1.0, 2.0], 0.0, p);
    let mut e = MeasuredVelocityEstimator::new(r.origin_us(), p, config()).unwrap();
    let first = r.advance_to(SimTime::ZERO).unwrap();
    let a = e.update(&first).unwrap();
    assert_eq!(a, e.update(&first).unwrap());
    let before = e.clone();
    let mut changed = first.clone();
    changed.wheels.as_mut().unwrap().payload.left_speed_m_s = 9.0;
    assert!(e.update(&changed).is_err());
    assert_eq!(e, before);
    let next = r.advance_to(SimTime::from_ticks(100_000)).unwrap();
    let mut gap = next.clone();
    gap.wheels.as_mut().unwrap().sequence = 7;
    assert!(e.update(&gap).is_err());
    assert_eq!(e, before);
    let mut bad = next.clone();
    bad.imu.as_mut().unwrap().payload.angular_velocity_rad_s[2] = f64::NAN;
    assert!(e.update(&bad).is_err());
    assert_eq!(e, before);
    e.update(&next).unwrap();
    let after = e.clone();
    assert!(e.update(&first).is_err());
    assert_eq!(e, after);
}

#[test]
fn pending_queue_is_bounded_and_failed_admission_is_atomic() {
    let p = NcltReplayPolicy {
        wheel_delay_us: 0,
        imu_delay_us: 1_000_000,
    };
    let mut r = replay(&[0, 100], &[1.0, 2.0], 0.0, p);
    let mut e = MeasuredVelocityEstimator::new(
        r.origin_us(),
        p,
        VelocityEstimatorConfig {
            queue_capacity: 1,
            ..config()
        },
    )
    .unwrap();
    let first = e.update(&r.advance_to(SimTime::ZERO).unwrap()).unwrap();
    assert!(first.estimate_time_ticks.is_none() && first.pose.is_none());
    let before = e.clone();
    assert!(e
        .update(&r.advance_to(SimTime::from_ticks(100_000)).unwrap())
        .is_err());
    assert_eq!(e, before);
}

#[test]
fn expiry_boundary_has_no_pose_but_replacement_without_gap_keeps_segment() {
    let p = NcltReplayPolicy {
        wheel_delay_us: 0,
        imu_delay_us: 0,
    };
    let mut r = replay(&[0, 200_000], &[1.0, 1.0], 0.0, p);
    let cfg = VelocityEstimatorConfig {
        wheel_hold_us: 200_000,
        imu_hold_us: 200_000,
        ..config()
    };
    let mut e = MeasuredVelocityEstimator::new(r.origin_us(), p, cfg).unwrap();
    let mut first = r.advance_to(SimTime::ZERO).unwrap();
    e.update(&first).unwrap();
    first.decision_time = SimTime::from_ticks(200_000_000);
    let expired = e.update(&first).unwrap();
    assert!(expired.pose.is_none());
    assert_eq!(expired.integrated_ticks, 200_000_000);
    assert_eq!(expired.unobserved_ticks, 0);
    let replaced = e
        .update(&r.advance_to(first.decision_time).unwrap())
        .unwrap();
    assert_eq!(replaced.pose.unwrap().segment_id, 1);
    assert_eq!(replaced.unobserved_ticks, 0);
}

#[test]
fn invalid_assumptions_are_rejected_at_construction() {
    let p = NcltReplayPolicy {
        wheel_delay_us: 0,
        imu_delay_us: 0,
    };
    for cfg in [
        VelocityEstimatorConfig {
            wheel_hold_us: 0,
            ..config()
        },
        VelocityEstimatorConfig {
            imu_hold_us: 10_000_001,
            ..config()
        },
        VelocityEstimatorConfig {
            queue_capacity: 4097,
            ..config()
        },
        VelocityEstimatorConfig {
            wheel_scale: -1.0,
            ..config()
        },
        VelocityEstimatorConfig {
            gyro_z_bias_rad_s: f64::NAN,
            ..config()
        },
        VelocityEstimatorConfig {
            segment_origin_m: [f64::INFINITY, 0.0],
            ..config()
        },
    ] {
        assert!(MeasuredVelocityEstimator::new(1_350_000_000_000_000, p, cfg).is_err());
    }
}

#[test]
fn geometry_has_stable_straight_reverse_and_turn_limits() {
    for (v, rate, x, y) in [
        (1.0, 0.0, 1.0, 0.0),
        (-1.0, 0.0, -1.0, 0.0),
        (
            1.0,
            std::f64::consts::FRAC_PI_2,
            2.0 / std::f64::consts::PI,
            2.0 / std::f64::consts::PI,
        ),
        (
            1.0,
            -std::f64::consts::FRAC_PI_2,
            2.0 / std::f64::consts::PI,
            -2.0 / std::f64::consts::PI,
        ),
        (1.0, 1e-14, 1.0, 0.0),
    ] {
        let mut pose = VelocitySegmentPose {
            segment_id: 1,
            position_m: [0.0, 0.0],
            heading_rad: 0.0,
        };
        integrate_pose(&mut pose, v, rate, 1.0).unwrap();
        assert!((pose.position_m[0] - x).abs() < 1e-12);
        assert!((pose.position_m[1] - y).abs() < 1e-12);
    }
}
