//! Runs the Unitree G1 factory inspection task headlessly.

use rne_ai::{
    Episode, UnitreeG1InspectionAction, UnitreeG1InspectionEpisode,
    UnitreeG1InspectionEpisodeConfig,
};

fn main() {
    let mut episode = UnitreeG1InspectionEpisode::new(UnitreeG1InspectionEpisodeConfig::default())
        .expect("load G1 factory inspection episode");
    let mut total_reward = 0.0;
    loop {
        let step = episode.step(UnitreeG1InspectionAction { advance: true });
        total_reward += step.reward;
        if episode.step_in_episode() == 1 {
            println!(
                "inspection route start: marker {}/{} distance={:.3} m radius={:.3} m",
                step.observation.current_marker_index + 1,
                step.observation.marker_count,
                step.observation.marker_distance_m,
                step.observation.marker_radius_m
            );
        }
        if episode.step_in_episode().is_multiple_of(30) || step.is_done() {
            println!(
                "step {:3}: marker={}/{} distance={:.3} m gesture={:.0}% completed={} reward={:.3}",
                episode.step_in_episode(),
                step.observation.current_marker_index + 1,
                step.observation.marker_count,
                step.observation.marker_distance_m,
                step.observation.gesture_progress * 100.0,
                step.observation.completed_markers,
                step.reward,
            );
        }
        if step.is_done() {
            assert!(
                step.terminated,
                "inspection should succeed before truncation"
            );
            println!(
                "factory inspection route complete: total_reward={total_reward:.3}, markers={}",
                step.observation.completed_markers
            );
            break;
        }
    }
}
