#!/usr/bin/env python3
"""Episode API example: drive to a goal and print reward/termination."""

from __future__ import annotations

import sys

try:
    import rne_py
except ImportError:
    print(
        "rne_py is not installed. Build it with:\n"
        "  pip install maturin\n"
        "  maturin develop -m ../../crates/rne_py/Cargo.toml",
        file=sys.stderr,
    )
    raise


def main() -> None:
    env = rne_py.DiffDriveEpisode(goal_x_m=2.0, max_steps=300)
    step = env.reset()
    wheel_velocity_rad_s = 6.0

    while not step.done:
        step = env.step(wheel_velocity_rad_s, wheel_velocity_rad_s)

    print(
        f"done: base_x={step.observation.base_x:.2f} m, "
        f"terminated={step.terminated}, total_reward={env.total_reward:.3f}"
    )
    if not step.terminated:
        raise SystemExit("expected success termination")


if __name__ == "__main__":
    main()
