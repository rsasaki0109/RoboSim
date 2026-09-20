#!/usr/bin/env python3
"""Jittered-grid scene: dense corners for SfM, aperiodic for matching.

Keeps the corner density of a checkerboard (good for SfM/SIFT) but jitters
positions/sizes/luminance (seeded) to break periodicity (good for global
descriptor matching), plus unique landmark boxes for retrieval.
"""

import colorsys
import random
import sys

TILE = 0.5
X_MAX = 20.0
Z_HALF = 2.0
WALL_Z = 1.8
SEED = 2026


def gray(rng):
    g = round(rng.uniform(0.05, 0.95), 4)
    return [g, g, round(g * 0.98, 4), 1.0]


def vivid(rng):
    h = rng.random()
    s = rng.uniform(0.7, 1.0)
    v = rng.uniform(0.55, 0.95)
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return [round(r, 4), round(g, 4), round(b, 4), 1.0]


def main() -> int:
    rng = random.Random(SEED)
    lines = [
        "[world]",
        "gravity_m_s2 = [0.0, -9.81, 0.0]",
        "seed = 2026",
        "",
        "[ground]",
        "enabled = true",
        "",
        "[[robots]]",
        'path = "../robots/dataset_diff_drive.rne.robot.toml"',
        "",
    ]
    name = 0
    nx = int(X_MAX / TILE) + 1
    nz = int(Z_HALF / TILE)

    def jitter():
        return round(rng.uniform(-0.12, 0.12), 3)

    # Jittered floor tiles (aperiodic luminance).
    for ix in range(nx):
        for iz in range(-nz, nz + 1):
            x = round(ix * TILE + jitter(), 3)
            z = round(iz * TILE + jitter(), 3)
            s = round(TILE * rng.uniform(0.8, 0.98), 3)
            colour = vivid(rng) if rng.random() < 0.04 else gray(rng)
            lines += [
                "[[objects]]",
                f'name = "floor_{name}"',
                f"translation_m = [{x}, 0.004, {z}]",
                'visual = {{ shape = "box", size_m = [{s}, 0.008, {s}], color_rgba = {c} }}'.format(
                    s=s, c=colour
                ),
                "",
            ]
            name += 1

    # Jittered wall panels.
    for side in (-1.0, 1.0):
        for ix in range(nx):
            for iy in range(5):
                x = round(ix * TILE + jitter(), 3)
                y = round(0.2 + iy * TILE + jitter(), 3)
                s = round(TILE * rng.uniform(0.8, 0.98), 3)
                colour = vivid(rng) if rng.random() < 0.06 else gray(rng)
                lines += [
                    "[[objects]]",
                    f'name = "wall_{name}"',
                    f"translation_m = [{x}, {y}, {side * WALL_Z:.3f}]",
                    'visual = {{ shape = "box", size_m = [{s}, {s}, 0.02], color_rgba = {c} }}'.format(
                        s=s, c=colour
                    ),
                    "",
                ]
                name += 1

    # Unique landmarks, clear of start and corridor.
    for _ in range(24):
        x = round(rng.uniform(3.0, 19.0), 3)
        z = round(rng.uniform(1.0, 1.6) * rng.choice((-1.0, 1.0)), 3)
        sx = round(rng.uniform(0.4, 0.9), 3)
        sy = round(rng.uniform(0.5, 1.4), 3)
        sz = round(rng.uniform(0.4, 0.9), 3)
        lines += [
            "[[objects]]",
            f'name = "landmark_{name}"',
            f"translation_m = [{x}, {round(sy / 2, 3)}, {z}]",
            'visual = {{ shape = "box", size_m = [{sx}, {sy}, {sz}], color_rgba = {c} }}'.format(
                sx=sx, sy=sy, sz=sz, c=vivid(rng)
            ),
            "",
        ]
        name += 1

    sys.stdout.write("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
