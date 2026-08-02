//! Headless vectorized Go2/G1 locomotion and policy-contract smoke.
//!
//! The example exercises the same seeded batch, replay checkpoint, and typed
//! policy boundaries that a future CEM/PPO adapter can consume. It intentionally
//! does not require a renderer or an external robotics middleware.

use rne_ai::{
    LocomotionPolicy, UnitreeG1GaitEpisodeConfig, UnitreeG1TorqueOverlay,
    UnitreeG1TorquePolicyInput, UnitreeGo2Action, UnitreeGo2EpisodeConfig,
    UnitreeGo2PureTorquePolicy, UnitreeGo2TerrainObservation, UnitreeGo2VelocityCommand,
    UnitreeGo2VelocityPolicyConfig, UnitreeGo2VelocityPolicyInput, VectorizedUnitreeG1GaitConfig,
    VectorizedUnitreeG1GaitEnv, VectorizedUnitreeGo2GaitConfig, VectorizedUnitreeGo2GaitEnv,
};

const DEFAULT_STEPS: usize = 24;

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let steps = if smoke { 8 } else { DEFAULT_STEPS };

    let go2_config = VectorizedUnitreeGo2GaitConfig {
        episode: UnitreeGo2EpisodeConfig {
            max_steps: (steps as u64) + 8,
            cycle_steps: 45,
            ..UnitreeGo2EpisodeConfig::default()
        },
        num_envs: 2,
        seed: 6602,
        auto_reset: false,
    };
    let mut go2 = VectorizedUnitreeGo2GaitEnv::new(go2_config).expect("Go2 vectorized env");
    let reset = go2.reset();
    assert_eq!(reset.observations.len(), 2);
    for _ in 0..steps {
        let step = go2.step(&[UnitreeGo2Action::default(); 2]);
        assert!(step.observations.iter().all(|observation| {
            observation.base_y_m.is_finite()
                && observation.base_relative_pitch_rad.is_finite()
                && observation.base_relative_roll_rad.is_finite()
        }));
    }
    let checkpoint = go2.checkpoint().expect("Go2 replay checkpoint");
    let digest = go2.replay_digest();
    go2.step(&[UnitreeGo2Action::default(); 2]);
    go2.restore_checkpoint(&checkpoint)
        .expect("Go2 replay checkpoint restore");
    assert_eq!(go2.replay_digest(), digest);

    let g1_config = VectorizedUnitreeG1GaitConfig {
        episode: UnitreeG1GaitEpisodeConfig {
            max_steps: (steps as u64) + 8,
            ..UnitreeG1GaitEpisodeConfig::default()
        },
        num_envs: 2,
        seed: 6601,
        auto_reset: false,
    };
    let mut g1 = VectorizedUnitreeG1GaitEnv::new(g1_config).expect("G1 vectorized env");
    g1.reset();
    for _ in 0..steps {
        let step = g1.step(&[Default::default(); 2]);
        assert!(step.observations.iter().all(|observation| {
            observation.base_y_m.is_finite()
                && observation.base_relative_pitch_rad.is_finite()
                && observation.base_relative_roll_rad.is_finite()
        }));
    }

    let mut go2_policy = UnitreeGo2PureTorquePolicy::LEARNED_WALK;
    let go2_torques = go2_policy.act(&UnitreeGo2VelocityPolicyInput {
        phase: 0.25,
        joint_positions_rad: [
            0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5,
        ],
        joint_velocities_rad_s: [0.0; 12],
        command: UnitreeGo2VelocityCommand { forward_m_s: 0.14 },
        measured_forward_velocity_m_s: 0.0,
        terrain: UnitreeGo2TerrainObservation::default(),
        config: UnitreeGo2VelocityPolicyConfig::default(),
    });
    assert!(go2_torques
        .iter()
        .all(|torque| torque.is_finite() && torque.abs() <= 23.7));

    let mut g1_policy = UnitreeG1TorqueOverlay::LEARNED_STRIDE;
    let g1_torques = g1_policy.act(&UnitreeG1TorquePolicyInput {
        two_cycle_phase: 0.25,
        stance: [true, false],
    });
    assert!(g1_torques
        .iter()
        .all(|torque| torque.is_finite() && torque.abs() <= 40.0));

    println!(
        "locomotion vectorized smoke ok: envs go2={} g1={} go2_digest=0x{:016x} go2_tau_max={:.2} g1_tau_max={:.2}",
        go2.num_envs(),
        g1.num_envs(),
        digest,
        go2_torques.iter().map(|torque| torque.abs()).fold(0.0, f64::max),
        g1_torques.iter().map(|torque| torque.abs()).fold(0.0, f64::max),
    );
}
