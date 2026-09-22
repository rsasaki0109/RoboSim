//! Load a versioned learned-policy artifact and evaluate it deterministically.
//!
//! Learned weights used to be frozen as Rust constants. This artifact format
//! makes them loadable data: a small dense network with explicit activations and
//! output clamps. The example authors a linear policy that steers toward a goal
//! from the fixed diff-drive observation encoding, saves it, reloads it, and
//! evaluates it.

use rne_ai::{
    Activation, DenseLayer, DiffDriveArtifactPolicy, DiffDriveObservation, PolicyArtifact,
    DIFF_DRIVE_ACTION_WIDTH, DIFF_DRIVE_OBSERVATION_WIDTH,
};

fn main() {
    // Row 0 drives forward by +10 * goal_delta_x; row 1 turns the other way.
    let mut weights = vec![0.0; DIFF_DRIVE_OBSERVATION_WIDTH * DIFF_DRIVE_ACTION_WIDTH];
    weights[8] = 10.0;
    weights[DIFF_DRIVE_OBSERVATION_WIDTH + 8] = -10.0;
    let artifact = PolicyArtifact {
        kind: "rne_policy".to_string(),
        schema_version: 1,
        name: "diff_drive_goal_linear".to_string(),
        source: "example".to_string(),
        observation_size: DIFF_DRIVE_OBSERVATION_WIDTH as u32,
        action_size: DIFF_DRIVE_ACTION_WIDTH as u32,
        layers: vec![DenseLayer {
            input_size: DIFF_DRIVE_OBSERVATION_WIDTH as u32,
            output_size: DIFF_DRIVE_ACTION_WIDTH as u32,
            weights,
            biases: vec![0.0, 0.0],
            activation: Activation::Identity,
        }],
        action_lower: vec![-3.0, -3.0],
        action_upper: vec![3.0, 3.0],
    };

    let out = std::path::Path::new("artifacts/policy-artifact/diff_drive_goal.rne.policy.json");
    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent).expect("create artifact directory");
    }
    artifact.save(out).expect("save policy artifact");
    println!("wrote {}", out.display());

    let loaded = PolicyArtifact::load(out).expect("load policy artifact");
    let policy = DiffDriveArtifactPolicy::new(loaded).expect("diff-drive policy");
    let observation = DiffDriveObservation {
        goal_delta_x_m: Some(0.25),
        ..DiffDriveObservation::default()
    };
    let action = policy.act(&observation);
    println!(
        "goal_delta_x=0.25 m -> left={:.3} rad/s right={:.3} rad/s",
        action.left_velocity_rad_s, action.right_velocity_rad_s
    );
    assert!((action.left_velocity_rad_s - 2.5).abs() < 1e-12);
    assert!((action.right_velocity_rad_s + 2.5).abs() < 1e-12);
}
