#!/usr/bin/env python3
"""Render the README motion-planning figure and animation.

The planner results come from the real ``rne_planning`` stack through
``examples/102_motion_planning_media``.  This script only draws them so the
documented motion is produced by the same code the tests exercise.

Run from the repository root::

    python tools/generate_motion_planning_media.py

Outputs:

* ``docs/media/motion-planning.png``  (workspace + joint-space paths)
* ``docs/media/motion-planning.gif``  (arm following the RRT-Connect plan)
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle

ROOT = Path(__file__).resolve().parents[1]
PNG_PATH = ROOT / "docs" / "media" / "motion-planning.png"
GIF_PATH = ROOT / "docs" / "media" / "motion-planning.gif"

BACKGROUND = "#0d1117"
FOREGROUND = "#e6edf3"
MUTED = "#8b949e"

# Collision-free planners we overlay, with display colors.
PLANNER_COLORS = {
    "rrt_connect": "#22d3ee",
    "rrt_star": "#a78bfa",
    "informed_rrt_star": "#f472b6",
    "prm": "#34d399",
    "bit_star": "#fbbf24",
}


def run_planner() -> dict:
    completed = subprocess.run(
        [
            "cargo",
            "run",
            "-q",
            "-p",
            "motion_planning_media",
            "--example",
            "102_motion_planning_media",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(completed.stdout)


def forward(q: Sequence[float], lengths: Sequence[float]) -> tuple[float, float]:
    first = q[0]
    elbow = (lengths[0] * math.cos(first), lengths[0] * math.sin(first))
    second = first + q[1]
    tip = (
        elbow[0] + lengths[1] * math.cos(second),
        elbow[1] + lengths[1] * math.sin(second),
    )
    return tip


def end_effector_path(waypoints: Iterable[Sequence[float]], lengths) -> tuple[list, list]:
    xs, ys = [], []
    for q in waypoints:
        x, y = forward(q, lengths)
        xs.append(x)
        ys.append(y)
    return xs, ys


def style_axis(ax) -> None:
    ax.set_facecolor(BACKGROUND)
    for spine in ax.spines.values():
        spine.set_color("#30363d")
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color="#21262d", linewidth=0.6)


def draw_arm(ax, q, lengths, color, alpha=1.0, linewidth=4.0, label=None) -> None:
    first = q[0]
    elbow = (lengths[0] * math.cos(first), lengths[0] * math.sin(first))
    second = first + q[1]
    tip = (
        elbow[0] + lengths[1] * math.cos(second),
        elbow[1] + lengths[1] * math.sin(second),
    )
    ax.plot([0.0, elbow[0]], [0.0, elbow[1]], color=color, lw=linewidth, alpha=alpha, solid_capstyle="round")
    ax.plot(
        [elbow[0], tip[0]],
        [elbow[1], tip[1]],
        color=color,
        lw=linewidth,
        alpha=alpha,
        solid_capstyle="round",
        label=label,
    )


def render_figure(data: dict) -> None:
    lengths = data["link_lengths_m"]
    start, goal = data["start"], data["goal"]
    obstacle = data["obstacle"]
    okay = {entry["name"]: entry for entry in data["planners"] if entry["ok"]}

    figure, (workspace, joint_space) = plt.subplots(1, 2, figsize=(13.0, 5.6), dpi=140)
    figure.patch.set_facecolor(BACKGROUND)

    for ax in (workspace, joint_space):
        style_axis(ax)

    # Workspace view.
    workspace.add_patch(
        Circle(obstacle["center"], obstacle["radius"], color="#f85149", alpha=0.75, zorder=2)
    )
    draw_arm(workspace, start, lengths, "#6e7681", alpha=0.85, linewidth=3.0)
    draw_arm(workspace, goal, lengths, "#484f58", alpha=0.85, linewidth=3.0)
    sx, sy = end_effector_path([start], lengths)
    gx, gy = end_effector_path([goal], lengths)
    workspace.scatter(sx, sy, color="#58a6ff", s=70, zorder=5, label="start")
    workspace.scatter(gx, gy, color="#3fb950", s=70, zorder=5, label="goal")

    straight_x, straight_y = end_effector_path(data["straight_line"], lengths)
    workspace.plot(
        straight_x,
        straight_y,
        color="#f85149",
        lw=1.8,
        ls="--",
        zorder=3,
        label="straight-line (blocked)",
    )

    for name, color in PLANNER_COLORS.items():
        entry = okay.get(name)
        if entry is None:
            continue
        xs, ys = end_effector_path(entry["waypoints"], lengths)
        workspace.plot(xs, ys, color=color, lw=2.2, zorder=4, label=name.replace("_", " "))

    workspace.set_title("Workspace end-effector paths", color=FOREGROUND, fontsize=13)
    workspace.set_xlabel("x (m)", color=MUTED)
    workspace.set_ylabel("y (m)", color=MUTED)
    workspace.set_aspect("equal", adjustable="box")
    workspace.set_xlim(-2.3, 2.3)
    workspace.set_ylim(-2.3, 2.3)
    legend = workspace.legend(
        loc="upper left", fontsize=8, facecolor="#161b22", edgecolor="#30363d", labelcolor=FOREGROUND
    )

    # Joint-space view.
    bound = math.pi
    joint_space.add_patch(
        plt.Rectangle((-bound, -bound), 2 * bound, 2 * bound, fill=False, edgecolor="#30363d")
    )
    joint_space.plot(
        [q[0] for q in data["straight_line"]],
        [q[1] for q in data["straight_line"]],
        color="#f85149",
        lw=1.8,
        ls="--",
        zorder=3,
    )
    for name, color in PLANNER_COLORS.items():
        entry = okay.get(name)
        if entry is None:
            continue
        joint_space.plot(
            [q[0] for q in entry["waypoints"]],
            [q[1] for q in entry["waypoints"]],
            color=color,
            lw=2.2,
            zorder=4,
        )
    joint_space.scatter([start[0]], [start[1]], color="#58a6ff", s=70, zorder=5)
    joint_space.scatter([goal[0]], [goal[1]], color="#3fb950", s=70, zorder=5)
    joint_space.set_title("Joint-space paths", color=FOREGROUND, fontsize=13)
    joint_space.set_xlabel("joint 1 (rad)", color=MUTED)
    joint_space.set_ylabel("joint 2 (rad)", color=MUTED)
    joint_space.set_xlim(-bound, bound)
    joint_space.set_ylim(-bound, bound)

    figure.tight_layout()
    figure.savefig(PNG_PATH, facecolor=BACKGROUND)
    plt.close(figure)
    print(f"wrote {PNG_PATH.relative_to(ROOT)}")


def render_gif(data: dict) -> None:
    lengths = data["link_lengths_m"]
    obstacle = data["obstacle"]
    plan = next(entry for entry in data["planners"] if entry["name"] == "rrt_connect" and entry["ok"])
    waypoints = plan["waypoints"]

    # Resample the joint path so the animation is smooth and constant-rate.
    frames = 90
    samples = []
    for frame in range(frames):
        t = frame / (frames - 1) * (len(waypoints) - 1)
        low = min(int(math.floor(t)), len(waypoints) - 2)
        blend = t - low
        q = [
            waypoints[low][index] * (1 - blend) + waypoints[low + 1][index] * blend
            for index in range(2)
        ]
        samples.append(q)

    figure, ax = plt.subplots(figsize=(6.4, 6.4), dpi=110)
    figure.patch.set_facecolor(BACKGROUND)
    style_axis(ax)
    ax.add_patch(Circle(obstacle["center"], obstacle["radius"], color="#f85149", alpha=0.75, zorder=2))
    draw_arm(ax, data["start"], lengths, "#6e7681", alpha=0.4, linewidth=3.0)
    draw_arm(ax, data["goal"], lengths, "#484f58", alpha=0.4, linewidth=3.0)
    for label, q, color in (("start", data["start"], "#58a6ff"), ("goal", data["goal"], "#3fb950")):
        x, y = forward(q, lengths)
        ax.scatter([x], [y], color=color, s=60, zorder=5, label=label)

    trail_x, trail_y = [], []
    arm_lines = [
        ax.plot([], [], color="#22d3ee", lw=5, solid_capstyle="round", zorder=4)[0]
        for _ in range(2)
    ]
    (trail,) = ax.plot([], [], color="#22d3ee", lw=1.6, alpha=0.7, zorder=3)
    ax.scatter([], [], color="#22d3ee", s=45, zorder=6)

    ax.set_title("RRT-Connect plan", color=FOREGROUND, fontsize=13)
    ax.set_xlim(-2.3, 2.3)
    ax.set_ylim(-2.3, 2.3)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper left", fontsize=9, facecolor="#161b22", edgecolor="#30363d", labelcolor=FOREGROUND)

    def update(frame):
        q = samples[frame]
        first = q[0]
        elbow = (lengths[0] * math.cos(first), lengths[0] * math.sin(first))
        second = first + q[1]
        tip = (elbow[0] + lengths[1] * math.cos(second), elbow[1] + lengths[1] * math.sin(second))
        arm_lines[0].set_data([0.0, elbow[0]], [0.0, elbow[1]])
        arm_lines[1].set_data([elbow[0], tip[0]], [elbow[1], tip[1]])
        trail_x.append(tip[0])
        trail_y.append(tip[1])
        trail.set_data(trail_x, trail_y)
        return arm_lines + [trail]

    animation = FuncAnimation(figure, update, frames=frames, interval=45, blit=False)
    animation.save(GIF_PATH, writer=PillowWriter(fps=22))
    plt.close(figure)
    print(f"wrote {GIF_PATH.relative_to(ROOT)}")


def main() -> int:
    data = run_planner()
    render_figure(data)
    render_gif(data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
