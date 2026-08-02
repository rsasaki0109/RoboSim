# Unitree locomotion RL adapters

This directory is the Python learning boundary for the headless Unitree Go2 gait
episode. The Rust episode owns the physics, observations, deterministic seed, and
termination; the Python layer only supplies Gymnasium/SB3 or dependency-free CEM
adapters.

Build the extension and run the CPU smokes from the repository root:

```text
.venv/bin/maturin develop -m crates/rne_py/Cargo.toml --release
.venv/bin/python examples/66_locomotion_rl/run.py --smoke
.venv/bin/python examples/66_locomotion_rl/train_cem.py --smoke
.venv/bin/python examples/66_locomotion_rl/train_ppo.py --smoke
```

The observation is a 21-element vector containing base pose/velocities, relative
tilt, foot impulses, gait phase, and episode progress. The five action values are
stride, swing lift, roll correction, pitch correction, and lateral calf extension.
