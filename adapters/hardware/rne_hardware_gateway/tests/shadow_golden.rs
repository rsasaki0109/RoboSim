use rne_ai::TaskSpec;
use rne_hardware_gateway::shadow::{
    ShadowComparator, ShadowComparisonConfig, ShadowTensorTolerance,
};
use rne_hardware_gateway::HardwareObservation;

const TASK_JSON: &str = include_str!("../../../../assets/tasks/diff_drive_goal.task.json");
const GOLDEN: &str =
    include_str!("../../../../tests/golden/hardware/gateway-shadow-comparison-v1.json");

#[test]
fn shadow_comparison_matches_golden() {
    let task: TaskSpec = serde_json::from_str(TASK_JSON).expect("task json");
    let config = ShadowComparisonConfig {
        sample_capacity: 2,
        tensors: vec![
            tolerance("base_position_m", 0.01),
            tolerance("base_yaw_rad", 0.01),
            tolerance("wheel_velocity_rad_s", 0.1),
            tolerance("imu_linear_acceleration_y_m_s2", 0.1),
            tolerance("lidar_point_count", 0.0),
            tolerance("goal_delta_x_m", 0.02),
        ],
    };
    let mut comparator = ShadowComparator::new(task.clone(), config).expect("comparator");
    comparator
        .compare(
            HardwareObservation {
                sequence: 10,
                received_at_ms: 100,
                values: vec![0.0; 9],
            },
            50,
            833_333_333,
            vec![0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .expect("passing comparison");
    comparator
        .compare(
            HardwareObservation {
                sequence: 11,
                received_at_ms: 117,
                values: vec![0.0; 9],
            },
            51,
            850_000_000,
            vec![0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .expect("failing comparison");
    let report = comparator.finish().expect("report");
    report.validate_against(&task).expect("validate report");
    let actual = format!(
        "{}\n",
        serde_json::to_string_pretty(&report).expect("serialize report")
    );
    assert_eq!(actual, GOLDEN);
}

fn tolerance(tensor_name: &str, absolute_tolerance: f64) -> ShadowTensorTolerance {
    ShadowTensorTolerance {
        tensor_name: tensor_name.to_string(),
        absolute_tolerance,
    }
}
