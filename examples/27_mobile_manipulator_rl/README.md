# 27 — Mobile manipulator RL environment

A gymnasium-style reinforcement-learning wrapper around the RNE mobile manipulator,
built on the `rne_py` Python bindings (`MobileManipulatorEpisode`). The same physics,
contact-triggered weld grasp, and `Place` reward used by the Rust example 26 are driven
from Python here.

## Build the bindings

```bash
.venv/bin/pip install maturin
.venv/bin/maturin develop -m crates/rne_py/Cargo.toml
```

## Run the smokes

```bash
# gym-style env wrapper + scripted pick-and-place rollout
.venv/bin/python examples/27_mobile_manipulator_rl/run.py --smoke

# self-contained training loop that LEARNS the reach task (no external deps)
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --smoke
```

`run.py` wraps the episode as a gymnasium-style env (scripted pick-and-place rollout).
`gymnasium` and `numpy` are optional: with them installed the env subclasses
`gymnasium.Env` and exposes `action_space` / `observation_space`; without them it returns
plain Python lists.

## Training loop (`train.py`)

`train.py` optimizes a small linear state-feedback policy on the dense-reward **reach**
task (`MobileManipulatorEpisode("reach")`) with the Cross-Entropy Method (CEM) — a
derivative-free RL algorithm needing only `rne_py` and the standard library. The mean
episode reward climbs from a failing policy (~2) to a solved reach (~12) over a handful of
iterations, demonstrating an end-to-end learning loop without `torch`/`gymnasium`:

```bash
.venv/bin/python examples/27_mobile_manipulator_rl/train.py          # 20 iterations
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --smoke  # short, asserts learning
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --checkpoint cem_checkpoint.json
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --checkpoint cem_checkpoint.json --resume --iterations 30
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-out best_policy.json
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --eval-episodes 12
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --rollout-csv rollout.csv
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --rollout-csv rollout.csv --rollout-house-gif rollout_house.gif --rollout-house-gif-metadata rollout_house.json
.venv/bin/python examples/27_mobile_manipulator_rl/plot_rollout.py rollout.csv --out rollout.svg
.venv/bin/python examples/27_mobile_manipulator_rl/animate_rollout.py rollout.csv --out rollout.html
.venv/bin/python examples/27_mobile_manipulator_rl/render_house_gif.py rollout.csv --out rollout_house.gif --metadata-out rollout_house.json
python examples/27_mobile_manipulator_rl/render_house_gif.py --demo --demo-rollout-csv house_mobile_manipulator.csv --out house_mobile_manipulator.gif --metadata-out house_mobile_manipulator.json
python examples/27_mobile_manipulator_rl/render_house_gif.py --verify-metadata house_mobile_manipulator.json
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --report-dir reports/reach
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --report-dir reports/reach_gif --report-house-gif --report-house-gif-max-frames 48
.venv/bin/python examples/27_mobile_manipulator_rl/compare_reports.py reports --html reports/leaderboard.html --csv reports/leaderboard.csv --json reports/leaderboard.json --best-policy-out reports/best_policy.json --best-report-out reports/best_report.json --best-house-gif-out reports/best_rollout_house.gif --best-house-gif-metadata-out reports/best_rollout_house.json
.venv/bin/python examples/27_mobile_manipulator_rl/sweep.py --out reports/sweep --runs 4 --iterations 12 --jobs 2
.venv/bin/python examples/27_mobile_manipulator_rl/sweep.py --out reports/sweep --runs 4 --require-final-error-at-most 0.15
.venv/bin/python examples/27_mobile_manipulator_rl/sweep.py --out reports/sweep --runs 4 --iterations 20 --resume
.venv/bin/python examples/27_mobile_manipulator_rl/sweep.py --out reports/sweep --runs 8 --skip-complete
.venv/bin/python examples/27_mobile_manipulator_rl/sweep.py --out reports/sweep_gif --runs 4 --report-house-gif --report-house-gif-width 240 --report-house-gif-height 160 --report-house-gif-max-frames 36
python -m unittest examples/27_mobile_manipulator_rl/test_report_tools.py
```

The reach target is placed where the arm only reaches it under active control (passive
settling fails), so the reward signal is meaningful for learning. Each CEM iteration
evaluates the whole candidate population in lock-step with
`rne_py.VectorizedMobileManipulatorEnv`, advancing every policy with a single batched
`step` call (each policy's reward is frozen the moment its episode ends).

Vectorized environments can be checkpointed and restored from Python as JSON, which is
intended for long training jobs that need deterministic resume:

```python
env = rne_py.VectorizedMobileManipulatorEnv("reach_random", 32)
env.reset()
env.save_checkpoint("rne_checkpoint.json")

# ... more rollout / training work ...

env.load_checkpoint("rne_checkpoint.json")
checkpoint_json = env.checkpoint_json()
env.restore_checkpoint_json(checkpoint_json)
```

The dependency-free CEM trainer also writes optimizer state with `--checkpoint`, including
the sampling distribution, best policy, reward history, and Python RNG state. Use
`--resume` with the same path to continue from the next unfinished iteration. The
`--iterations` value is the total target count, not an additional count.

For a portable learned artifact, use `--policy-out` to save only the best linear policy.
`--policy-in ... --eval-only` evaluates a saved policy without rerunning CEM, and
`--policy-in` without `--eval-only` uses that policy as the initial CEM mean.
`--rollout-csv` records one randomized reach rollout with observations, actions, per-step
reward, cumulative reward, and done flags for plotting or debugging. Add
`--rollout-house-gif` to render a house-scene GIF from that same CSV in one command,
and `--rollout-house-gif-metadata` to write the checksum JSON at the same time.
`plot_rollout.py`
turns that CSV into a standalone SVG report showing the end-effector X-Z path, target
error, reward, and actions. `animate_rollout.py` turns the same CSV into a standalone
HTML replay with play/pause and scrubbing controls. `render_house_gif.py` turns the
same CSV into a dependency-free animated GIF of the mobile manipulator moving through a
small house scene and can write a checksum metadata JSON with `--metadata-out`; `--demo`
renders a synthetic scene without `rne_py`, and `--demo-rollout-csv` saves that same
synthetic trajectory as canonical rollout CSV for re-rendering or debugging. Use
`--verify-metadata` to validate an existing GIF metadata JSON against its referenced GIF.
`--report-dir` writes `index.html`, `manifest.json`, `policy.json`, `rollout.csv`,
`rollout.svg`, and `rollout.html` as one bundle. Add `--report-house-gif` to include
`rollout_house.gif` and `rollout_house.json` in that bundle and embed the GIF in
`index.html`; use
`--report-house-gif-width`, `--report-house-gif-height`,
`--report-house-gif-max-frames`, and `--report-house-gif-fps` to tune output size.
`compare_reports.py` scans those manifests and builds Markdown, HTML, CSV, and JSON
leaderboards ranked by final target error, with links back to each report's `index.html`.
It validates required report artifacts and checks each `policy.json` schema,
algorithm, and parameter shape before ranking. It can also copy the top-ranked report's
`policy.json` to a stable `best_policy.json` path for evaluation or deployment, and
write `best_report.json` with the winning report's metrics and source artifact paths.
When a report includes `rollout_house.gif`, the leaderboard JSON/CSV/HTML exposes that
optional artifact path, metadata path, dimensions, frame count, FPS, byte size, and
SHA-256 as well. The HTML leaderboard also embeds a small GIF thumbnail for quick
visual comparison and links to its metadata JSON.
Use `--best-house-gif-out` and `--best-house-gif-metadata-out` to copy the
winning report's GIF to a stable path with checksum/provenance JSON beside it.
`sweep.py` automates multiple CEM seeds, writes one report bundle per seed, then builds
HTML, CSV, and JSON leaderboards plus `best_policy.json`, `best_report.json`, and
`best_rollout_house.gif` plus `best_rollout_house.json` when GIF reports are enabled
for the whole sweep, then records the run configuration in `sweep_manifest.json`. Add
`--jobs N`
to train seeds in parallel, and `--resume` to continue seeds that already have a
`checkpoint.json`. Use `--skip-complete` to leave seeds with an existing `manifest.json`
untouched while still rebuilding the leaderboard. The sweep compares only the expected
`seed_XXXX/manifest.json` files for that run, so stale reports elsewhere under the
output directory do not affect ranking or gates. Use
`--require-final-error-at-most` or `--require-final-reward-at-least` when a sweep should
act as a regression gate. Add `--report-house-gif` when each seed report should include
the derived house-scene GIF; the same GIF width, height, frame count, and FPS options are
forwarded to each seed's `train.py` run.
The JSON/CSV artifact contracts are documented in
[`SCHEMA.md`](SCHEMA.md).

## Spaces

- **Action** (`shape=(5,)`): `[left_wheel, right_wheel, shoulder, elbow, gripper]`
  velocities (rad/s; gripper is m/s, negative closes).
- **Observation** (`shape=(12,)`): base pose `(x, y, z, yaw)`, end-effector `(x, y, z)`,
  `shoulder`, `elbow`, `gripper` joint positions, `wrist_camera_pixels`,
  `joint_state_count`.

## Training with Stable-Baselines3 (`train_ppo.py`)

`stable-baselines3` is not a project dependency; install the RL extras into the venv:

```bash
.venv/bin/pip install gymnasium numpy stable-baselines3
.venv/bin/python examples/27_mobile_manipulator_rl/train_ppo.py          # full PPO run
.venv/bin/python examples/27_mobile_manipulator_rl/train_ppo.py --smoke  # integration check
```

`train_ppo.py` plugs the `reach` gym env into SB3 PPO. It is the *integration* example —
the `--smoke` mode only verifies the training pipeline runs end-to-end (deep RL needs a
long run and some tuning to beat a random policy on this task). For a deterministic,
dependency-free demonstration that learning actually works, use `train.py` (CEM) above.

```python
from stable_baselines3 import PPO
from run import MobileManipulatorPlaceEnv

env = MobileManipulatorPlaceEnv("reach")
model = PPO("MlpPolicy", env, verbose=1, device="cpu")
model.learn(total_timesteps=200_000)
```

Note: the `mm_minimal` arm is a horizontal SCARA, so `Place` is a horizontal carry and
release calibrated to a deterministic target. Reward is shaped by the object's horizontal
distance to that target plus a success bonus on a settled placement.
