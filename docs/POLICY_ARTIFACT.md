# Policy Artifacts

Versioned, backend-neutral storage for learned policies. Training (the
dependency-free CEM recipes in examples, or external PPO) previously froze its
winning weights as Rust constants. `rne_ai::PolicyArtifact` makes those weights
loadable data instead.

## Format

`.rne.policy.json`, schema version 1, kind `rne_policy`:

- `name`, `source` (provenance such as `cem`, `ppo`, or `linear`);
- `observation_size`, `action_size`;
- `layers`: dense layers with row-major `weights` (`input_size * output_size`),
  `biases`, and an `activation` (`identity`, `tanh`, `relu`, `sigmoid`);
- `action_lower`, `action_upper`: inclusive per-output clamps.

`validate()` enforces a consistent layer chain, finite weights, matching bias and
bound widths, and ordering of bounds. `evaluate()` runs a fixed-order `f64`
forward pass and clamps the action, so replaying the same artifact and
observation yields bit-identical actions. `save`/`load` wrap the JSON codec.

A single identity layer expresses a linear policy, so linear CEM outputs can be
serialized unchanged.

## Diff-drive binding

`DiffDriveArtifactPolicy` binds a `PolicyArtifact` to the fixed 12-value
diff-drive observation encoding (`DIFF_DRIVE_OBSERVATION_WIDTH`) and 2-value
action (`DIFF_DRIVE_ACTION_WIDTH`), implementing both `LocomotionPolicy` and
`Policy<DiffDriveEpisode>`. `diff_drive_observation_vector` documents the exact
encoding; missing goal/peer fields encode as `0.0`.

## Training

`rne_ai::cem_train` is a reusable, dependency-free, seeded cross-entropy method
optimizer. It maximizes a caller-supplied deterministic fitness function over a
flat parameter vector and returns the best parameters plus per-iteration elite
fitness. `MlpPolicyTemplate` describes a dense MLP and maps between the flat
vector and a `PolicyArtifact` (`parameter_count`, `to_artifact`), so CEM output
exports directly to the artifact format. `CemConfig` requires an even population
`>= 4`, an elite fraction in `(0, 1)`, and positive standard deviations.

This replaces the ad-hoc CEM loops that were copied into examples; those recipes
can now call the library and emit a loadable artifact. Neural PPO/SAC with
analytic gradients is a later increment and would reuse the same export path.

## Native networks

`rne_ai::neural` adds a small, dependency-free differentiable core: `NeuralNet`
is a deterministic dense MLP with a hand-written backward pass, `Adam` is a
seeded optimizer, and `to_policy_artifact` exports the trained weights to the
`.rne.policy.json` format. Weights are initialized from `DeterministicRng`, and
`backward` returns per-layer gradients (`LayerGradient`). Gradients are verified
against central finite differences in the tests, and Adam drives a regression
loss down. On-policy optimizers (PPO/SAC) build on this module.

`rne_ai::ppo` adds a deterministic PPO optimizer on top: `PpoTrainer` holds a
Gaussian-mean policy network and a value network, collects seeded rollouts from
a `PpoEnv`, computes GAE advantages, and optimizes the clipped surrogate plus
value and entropy terms. `to_policy_artifact` exports the deterministic mean
policy. The tests train a continuous bandit to its target and check determinism.

## Examples

`examples/114_policy_artifact` authors a linear policy, saves it, reloads it,
and evaluates it deterministically: `cargo run -p policy_artifact --example
114_policy_artifact`. `examples/115_trainer` runs the seeded `cem_train`
optimizer against a deterministic regression objective and exports the winner
as a `.rne.policy.json` artifact: `cargo run -p trainer --example
115_trainer`.

## Limits

- The format is dense feed-forward only. Recurrent, convolutional, or branching
  policies are not represented; add a new schema version rather than changing v1.
- Inference is CPU `f64`. There is no ONNX/TensorRT interop yet; exporting a
  trained network to this format is the intended bridge.
