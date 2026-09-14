//! ECS systems for online SLAM.

use crate::resources::{PendingSlamScans, SlamState};
use bevy_ecs::prelude::World;
use rne_nav::{GridError, Pose2d};

/// Aggregate result of one [`slam_step`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SlamStepReport {
    /// Number of scans processed.
    pub scans: usize,
    /// Number of scans that were successfully matched.
    pub matched: usize,
    /// Pose after the last processed scan.
    pub last_pose: Pose2d,
}

/// Drains the pending scan queue into the [`SlamState`] estimator.
pub fn slam_step(world: &mut World) -> Result<SlamStepReport, GridError> {
    let pending = match world.get_resource_mut::<PendingSlamScans>() {
        Some(mut queue) => std::mem::take(&mut queue.0),
        None => return Ok(SlamStepReport::default()),
    };
    if pending.is_empty() {
        return Ok(SlamStepReport::default());
    }
    let Some(mut state) = world.get_resource_mut::<SlamState>() else {
        return Ok(SlamStepReport::default());
    };

    let mut report = SlamStepReport::default();
    for input in pending {
        let update =
            state
                .estimator
                .process(&input.scan, input.odom_pose, input.sensor_from_base)?;
        report.scans += 1;
        if update.matched {
            report.matched += 1;
        }
        report.last_pose = update.pose;
    }
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resources::{PendingSlamScans, SlamState};
    use crate::slam::{Slam2d, SlamConfig};
    use rne_nav::{FrameId, OccupancyGrid};
    use std::f64::consts::TAU;

    #[test]
    fn system_drains_queue_into_estimator() {
        let mut world = World::new();
        let grid = OccupancyGrid::new(120, 80, 0.1, Pose2d::new(-6.0, -4.0, 0.0)).unwrap();
        world.insert_resource(SlamState::new(Slam2d::new(grid, SlamConfig::default())));
        let beams = 180;
        let scan = rne_nav::LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / beams as f64,
            range_min_m: 0.05,
            range_max_m: 30.0,
            ranges_m: vec![2.0; beams],
        };
        let mut pending = PendingSlamScans::new();
        pending.push(scan.clone(), Pose2d::IDENTITY, Pose2d::IDENTITY);
        pending.push(scan, Pose2d::new(0.1, 0.0, 0.0), Pose2d::IDENTITY);
        world.insert_resource(pending);

        let report = slam_step(&mut world).unwrap();
        assert_eq!(report.scans, 2);
        assert!(world.resource::<PendingSlamScans>().is_empty());
        assert_eq!(world.resource::<SlamState>().estimator.scans_processed(), 2);
    }
}
