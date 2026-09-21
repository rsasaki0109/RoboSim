#!/usr/bin/env python3
"""Render a passing contact-plant recording; never generate a motion trajectory."""

import argparse
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1_backflip_plant as plant
import mujoco
import numpy as np
from PIL import Image, ImageDraw


def render(directory, output):
    """Render measured configurations only, with a fixed camera and timestamp."""
    if shutil.disk_usage(directory).free < 30 * 1024**3:
        raise RuntimeError("disk reserve below 30 GiB")
    summary = json.loads((directory / "summary.json").read_text())
    if not summary["passed"] or summary["backend"] != "mujoco":
        raise ValueError("GIF requires a passing MuJoCo contact rollout")
    recording = (directory / "rollout.json").read_bytes()
    if hashlib.sha256(recording).hexdigest() != summary["rollout_sha256"]:
        raise ValueError("recording checksum differs from the validated rollout")
    history = json.loads(recording)
    manifest = json.loads((plant.MODEL_MANIFEST).read_text())
    manifest["mass_policy"] = summary["mass_policy"]
    manifest["joint_limit_time_constant_s"] = summary["joint_limit_time_constant_s"]
    manifest["joint_limit_margin_rad"] = summary["joint_limit_margin_rad"]
    model = plant.build_model(manifest, summary["dt_s"], 300, 10)
    model.vis.headlight.ambient[:] = 0.55
    model.vis.headlight.diffuse[:] = 0.8
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    positions = np.asarray([frame["qpos"][:3] for frame in history])
    camera.lookat[:] = [0.5 * (positions[:, 0].min() + positions[:, 0].max()), 0, 0.8]
    camera.distance = 2.8 + np.ptp(positions[:, 0]) * 0.4
    camera.azimuth = 100
    camera.elevation = -12
    frames = []
    with mujoco.Renderer(model, 480, 480) as renderer:
        for index in range(0, len(history), 3):
            frame = history[index]
            data.qpos[:] = frame["qpos"]
            data.qvel[:] = frame["qvel"]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera)
            # Decorative floor grid; it never participates in the contact plant.
            for offset in np.arange(-3, 3.01, 0.5):
                for start, end in (
                    ([-3, offset, 0.001], [3, offset, 0.001]),
                    ([offset, -3, 0.001], [offset, 3, 0.001]),
                ):
                    geom = renderer.scene.geoms[renderer.scene.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_LINE,
                        np.zeros(3),
                        np.zeros(3),
                        np.eye(3).ravel(),
                        np.array([0.2, 0.3, 0.4, 1], dtype=np.float32),
                    )
                    mujoco.mjv_connector(
                        geom,
                        mujoco.mjtGeom.mjGEOM_LINE,
                        1,
                        np.asarray(start),
                        np.asarray(end),
                    )
                    renderer.scene.ngeom += 1
            picture = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, 480, 44), fill=(15, 20, 28))
            draw.text(
                (12, 8), "G1 | Non-RL optimization | MuJoCo physics", fill="white"
            )
            draw.text(
                (12, 25),
                f"t = {frame['time_s']:.2f} s   |   {summary['mass_kg']:.2f} kg",
                fill=(170, 210, 240),
            )
            frames.append(picture)
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=30,
        loop=0,
        optimize=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    render(args.directory, args.output)


if __name__ == "__main__":
    main()
