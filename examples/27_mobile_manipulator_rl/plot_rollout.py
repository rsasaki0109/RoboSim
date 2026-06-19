"""Render a saved mobile-manipulator rollout CSV as a standalone SVG report.

The input is produced by:

    python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --rollout-csv rollout.csv

The output SVG is dependency-free and can be opened directly in a browser.
"""

import argparse
import csv
import html
import math
import os
import sys


def _float(row, key):
    try:
        return float(row[key])
    except KeyError as error:
        raise ValueError(f"rollout CSV missing column: {key}") from error


def load_rollout(path):
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"rollout CSV has no rows: {path}")

    samples = []
    for row in rows:
        target_error = math.sqrt(
            _float(row, "target_dx") ** 2
            + _float(row, "target_dy") ** 2
            + _float(row, "target_dz") ** 2
        )
        samples.append(
            {
                "step": int(float(row["step"])),
                "ee_x": _float(row, "ee_x"),
                "ee_z": _float(row, "ee_z"),
                "target_x": _float(row, "ee_x") + _float(row, "target_dx"),
                "target_z": _float(row, "ee_z") + _float(row, "target_dz"),
                "target_error": target_error,
                "reward": _float(row, "reward"),
                "total_reward": _float(row, "total_reward"),
                "shoulder_action": _float(row, "shoulder_action"),
                "elbow_action": _float(row, "elbow_action"),
                "done": row.get("done", "").lower() == "true",
            }
        )
    return samples


def _series_bounds(series):
    lo = min(series)
    hi = max(series)
    if math.isclose(lo, hi):
        pad = max(1.0, abs(lo) * 0.1)
        return lo - pad, hi + pad
    pad = (hi - lo) * 0.08
    return lo - pad, hi + pad


def _polyline(samples, key, rect):
    x0, y0, width, height = rect
    steps = [sample["step"] for sample in samples]
    values = [sample[key] for sample in samples]
    sx0, sx1 = min(steps), max(steps)
    vy0, vy1 = _series_bounds(values)
    if sx0 == sx1:
        sx1 += 1

    points = []
    for step, value in zip(steps, values):
        x = x0 + ((step - sx0) / (sx1 - sx0)) * width
        y = y0 + height - ((value - vy0) / (vy1 - vy0)) * height
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points), vy0, vy1


def _panel(samples, title, series, rect):
    x, y, width, height = rect
    colors = ["#1f77b4", "#d62728", "#2ca02c"]
    body = [
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="#ffffff" stroke="#cbd5e1"/>',
        f'<text x="{x}" y="{y - 10}" class="panel-title">{html.escape(title)}</text>',
    ]

    for i in range(5):
        gy = y + (height * i / 4)
        body.append(f'<line x1="{x}" y1="{gy:.2f}" x2="{x + width}" y2="{gy:.2f}" class="grid"/>')

    for index, (key, label) in enumerate(series):
        points, lo, hi = _polyline(samples, key, rect)
        color = colors[index % len(colors)]
        body.append(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.5"/>'
        )
        body.append(
            f'<text x="{x + width + 16}" y="{y + 18 + index * 20}" fill="{color}" class="legend">'
            f"{html.escape(label)} [{lo:.2f}, {hi:.2f}]</text>"
        )

    done_steps = [sample["step"] for sample in samples if sample["done"]]
    if done_steps:
        sx0 = samples[0]["step"]
        sx1 = samples[-1]["step"] if samples[-1]["step"] != sx0 else sx0 + 1
        done_x = x + ((done_steps[0] - sx0) / (sx1 - sx0)) * width
        body.append(f'<line x1="{done_x:.2f}" y1="{y}" x2="{done_x:.2f}" y2="{y + height}" class="done"/>')

    body.append(f'<text x="{x}" y="{y + height + 22}" class="axis">step {samples[0]["step"]}</text>')
    body.append(
        f'<text x="{x + width - 70}" y="{y + height + 22}" class="axis">step {samples[-1]["step"]}</text>'
    )
    return "\n".join(body)


def _path_bounds(samples):
    xs = [sample["ee_x"] for sample in samples] + [sample["target_x"] for sample in samples]
    zs = [sample["ee_z"] for sample in samples] + [sample["target_z"] for sample in samples]
    x0, x1 = _series_bounds(xs)
    z0, z1 = _series_bounds(zs)
    return x0, x1, z0, z1


def _map_path_point(x_value, z_value, rect, bounds):
    x, y, width, height = rect
    x0, x1, z0, z1 = bounds
    px = x + ((x_value - x0) / (x1 - x0)) * width
    py = y + height - ((z_value - z0) / (z1 - z0)) * height
    return px, py


def _trajectory_panel(samples, rect):
    x, y, width, height = rect
    bounds = _path_bounds(samples)
    points = [
        _map_path_point(sample["ee_x"], sample["ee_z"], rect, bounds) for sample in samples
    ]
    point_text = " ".join(f"{px:.2f},{py:.2f}" for px, py in points)
    start_x, start_y = points[0]
    end_x, end_y = points[-1]
    target_x, target_y = _map_path_point(
        samples[-1]["target_x"], samples[-1]["target_z"], rect, bounds
    )

    body = [
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="#ffffff" stroke="#cbd5e1"/>',
        f'<text x="{x}" y="{y - 10}" class="panel-title">End-effector X-Z path</text>',
    ]
    for i in range(5):
        gx = x + (width * i / 4)
        gy = y + (height * i / 4)
        body.append(f'<line x1="{gx:.2f}" y1="{y}" x2="{gx:.2f}" y2="{y + height}" class="grid"/>')
        body.append(f'<line x1="{x}" y1="{gy:.2f}" x2="{x + width}" y2="{gy:.2f}" class="grid"/>')

    body.extend(
        [
            f'<polyline points="{point_text}" fill="none" stroke="#0f766e" stroke-width="3"/>',
            f'<circle cx="{start_x:.2f}" cy="{start_y:.2f}" r="5" fill="#1f77b4"/>',
            f'<circle cx="{end_x:.2f}" cy="{end_y:.2f}" r="5" fill="#0f766e"/>',
            f'<circle cx="{target_x:.2f}" cy="{target_y:.2f}" r="7" fill="none" stroke="#d62728" stroke-width="3"/>',
            f'<text x="{x + width + 16}" y="{y + 18}" class="legend" fill="#1f77b4">start</text>',
            f'<text x="{x + width + 16}" y="{y + 38}" class="legend" fill="#0f766e">end path</text>',
            f'<text x="{x + width + 16}" y="{y + 58}" class="legend" fill="#d62728">target</text>',
            f'<text x="{x}" y="{y + height + 22}" class="axis">x={bounds[0]:.2f} z={bounds[2]:.2f}</text>',
            f'<text x="{x + width - 130}" y="{y + height + 22}" class="axis">x={bounds[1]:.2f} z={bounds[3]:.2f}</text>',
        ]
    )
    return "\n".join(body)


def render_svg(samples, title):
    width = 1080
    height = 1010
    panels = [
        ("End-effector target error", (("target_error", "error m"),), (72, 420, 720, 130)),
        ("Reward", (("reward", "step"), ("total_reward", "total")), (72, 620, 720, 130)),
        ("Policy actions", (("shoulder_action", "shoulder"), ("elbow_action", "elbow")), (72, 820, 720, 130)),
    ]
    escaped_title = html.escape(title)
    body = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>",
        "body{margin:0}",
        ".title{font:700 26px system-ui, sans-serif; fill:#0f172a}",
        ".subtitle{font:14px system-ui, sans-serif; fill:#475569}",
        ".panel-title{font:700 15px system-ui, sans-serif; fill:#0f172a}",
        ".legend,.axis{font:12px system-ui, sans-serif; fill:#475569}",
        ".grid{stroke:#e2e8f0; stroke-width:1}",
        ".done{stroke:#7c3aed; stroke-width:2; stroke-dasharray:5 5}",
        "</style>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="72" y="52" class="title">{escaped_title}</text>',
        f'<text x="72" y="78" class="subtitle">samples={len(samples)} final_reward={samples[-1]["total_reward"]:.3f}</text>',
    ]
    body.append(_trajectory_panel(samples, (72, 120, 720, 220)))
    for panel_title, series, rect in panels:
        body.append(_panel(samples, panel_title, series, rect))
    body.append("</svg>")
    return "\n".join(body) + "\n"


def default_output_path(input_path):
    root, _ = os.path.splitext(input_path)
    return f"{root}.svg"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", help="rollout CSV produced by train.py --rollout-csv")
    parser.add_argument("--out", help="SVG output path; defaults to CSV path with .svg")
    return parser.parse_args()


def main():
    args = parse_args()
    output_path = args.out or default_output_path(args.csv_path)
    samples = load_rollout(args.csv_path)
    svg = render_svg(samples, os.path.basename(args.csv_path))
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write(svg)
    print(f"rollout svg saved: {output_path} samples={len(samples)}")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        sys.exit(str(error))
