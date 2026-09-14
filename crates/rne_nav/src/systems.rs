//! Navigation ECS systems.

use crate::grid::GridError;
use crate::resources::{NavMap, PendingScans};
use crate::scan::ScanIntegrationReport;
use bevy_ecs::prelude::World;

/// Integrates every queued scan into the [`NavMap`] and rebuilds its costmap.
///
/// The queue is drained even when empty. Returns the aggregate report; an
/// invalid scan or costmap configuration surfaces as a [`GridError`].
pub fn integrate_pending_scans(world: &mut World) -> Result<ScanIntegrationReport, GridError> {
    let pending = match world.get_resource_mut::<PendingScans>() {
        Some(mut queue) => std::mem::take(&mut queue.0),
        None => return Ok(ScanIntegrationReport::default()),
    };
    if pending.is_empty() {
        return Ok(ScanIntegrationReport::default());
    }

    let Some(mut map) = world.get_resource_mut::<NavMap>() else {
        return Ok(ScanIntegrationReport::default());
    };

    let mut total = ScanIntegrationReport::default();
    for scan in pending {
        let report = map.integrate(&scan.scan, scan.sensor_pose_world)?;
        total.beams_processed += report.beams_processed;
        total.beams_skipped += report.beams_skipped;
        total.free_updates += report.free_updates;
        total.occupied_updates += report.occupied_updates;
    }
    map.refresh_costmap()?;
    Ok(total)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::CostmapConfig;
    use crate::grid::OccupancyGrid;
    use crate::pose2d::Pose2d;
    use crate::resources::PendingScans;
    use crate::scan::{LaserScan2d, ScanIntegrationConfig};
    use crate::tf::FrameId;

    #[test]
    fn system_integrates_and_refreshes_costmap() {
        let mut world = World::new();
        let grid = OccupancyGrid::new(21, 21, 0.2, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
        let map = NavMap::new(
            grid,
            CostmapConfig::default(),
            ScanIntegrationConfig::default(),
        )
        .unwrap();
        world.insert_resource(map);
        let mut pending = PendingScans::new();
        let beam_count = 72;
        pending.push(
            LaserScan2d {
                time_s: 0.0,
                frame: FrameId::new("laser"),
                angle_min_rad: 0.0,
                angle_increment_rad: std::f64::consts::TAU / beam_count as f64,
                range_min_m: 0.1,
                range_max_m: 10.0,
                ranges_m: vec![1.0; beam_count],
            },
            Pose2d::IDENTITY,
        );
        world.insert_resource(pending);

        let report = integrate_pending_scans(&mut world).unwrap();
        assert_eq!(report.beams_processed, beam_count);
        assert!(world.resource::<NavMap>().costmap.lethal_count() > 0);
        assert!(world.resource::<PendingScans>().is_empty());
    }
}
