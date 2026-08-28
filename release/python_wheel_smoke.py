#!/usr/bin/env python3
"""Installed ABI3-wheel smoke for the 0.2.0 release rehearsal."""

from __future__ import annotations

from importlib import metadata

import rne_py


def main() -> None:
    version = metadata.version("rne_py")
    if version != "0.2.0":
        raise SystemExit(f"unexpected rne_py wheel version: {version}")
    if rne_py.__version__ != "0.2.0":
        raise SystemExit(f"unexpected rne_py module version: {rne_py.__version__}")

    episode = rne_py.DiffDriveEpisode(goal_x_m=1.0, max_steps=300)
    step = episode.reset()
    while not step.done:
        step = episode.step(6.0, 6.0)
    if not step.terminated:
        raise SystemExit("installed wheel did not complete the diff-drive episode")
    print(
        f"rne_py wheel smoke passed: version={version} "
        f"base_x_m={step.observation.base_x:.3f}"
    )


if __name__ == "__main__":
    main()
