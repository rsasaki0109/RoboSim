//! Train a native policy with the deterministic CEM trainer and export it.
//!
//! The fitness function is a deterministic regression objective: learn a linear
//! policy whose action equals `2 * observation`. The trainer is seeded, so the
//! run is reproducible, and the winning parameters are exported as a versioned
//! `.rne.policy.json` artifact.

use rne_ai::{cem_train, Activation, CemConfig, MlpPolicyTemplate, PolicyArtifact};

fn main() {
    let samples: Vec<f64> = (0..21).map(|index| -1.0 + index as f64 * 0.1).collect();
    let template = MlpPolicyTemplate {
        observation_size: 1,
        action_size: 1,
        hidden_sizes: Vec::new(),
        hidden_activation: Activation::Identity,
        action_lower: vec![-10.0],
        action_upper: vec![10.0],
    };
    let config = CemConfig {
        population: 64,
        elite_fraction: 0.2,
        iterations: 200,
        initial_std: 1.0,
        min_std: 1e-4,
        seed: 42,
    };

    let result = cem_train(
        |parameters| {
            let (weight, bias) = (parameters[0], parameters[1]);
            samples
                .iter()
                .map(|observation| {
                    let predicted = weight * observation + bias;
                    let target = 2.0 * observation;
                    -(predicted - target).powi(2)
                })
                .sum::<f64>()
                / samples.len() as f64
        },
        template.parameter_count(),
        config,
    )
    .expect("train");

    println!(
        "CEM best fitness {:.6}, parameters w={:.4} b={:.4}",
        result.best_fitness, result.best_parameters[0], result.best_parameters[1]
    );
    assert!(
        (result.best_parameters[0] - 2.0).abs() < 2e-2,
        "weight did not converge"
    );
    assert!(
        result.best_parameters[1].abs() < 2e-2,
        "bias did not converge to zero"
    );

    let artifact = template
        .to_artifact("linear_2x", "cem", &result.best_parameters)
        .expect("export artifact");
    let out = std::path::Path::new("artifacts/policy-artifact/linear_2x.rne.policy.json");
    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent).expect("create artifact directory");
    }
    artifact.save(out).expect("save artifact");
    println!("wrote {}", out.display());

    let loaded = PolicyArtifact::load(out).expect("load artifact");
    let action = loaded.evaluate(&[0.5]).expect("evaluate");
    println!("policy(0.5) = {:.4}", action[0]);
    assert!((action[0] - 1.0).abs() < 1e-2);
}
