#!/usr/bin/env python3
"""Minimal Python policy controlling a differential drive robot."""

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


def run_policy(steps: int = 180, wheel_velocity_rad_s: float = 6.0) -> float:
    sim = rne_py.DiffDriveSim()
    obs = sim.reset()

    for _ in range(steps):
        obs = sim.step(wheel_velocity_rad_s, wheel_velocity_rad_s)

    return obs.base_x


def main() -> None:
    final_x = run_policy()
    print(f"final forward travel = {final_x:.2f} m")
    if final_x < 1.0:
        raise SystemExit(f"expected forward motion, got x={final_x:.2f}")


if __name__ == "__main__":
    main()
