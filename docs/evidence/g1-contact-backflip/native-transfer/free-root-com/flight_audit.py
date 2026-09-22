"""Offline source-model FK audit of native frames; never advances MuJoCo time."""

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

root = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root / "scripts"))
import mujoco
from g1_backflip_plant import build_model

manifest = json.loads((root / "scripts/fixtures/g1_backflip_model.json").read_text())
manifest["mass_policy"] = "declared"
model = build_model(manifest, 0.000125, 300, 10)
if isinstance(model, tuple):
    model = model[0]
data = mujoco.MjData(model)
joints = {
    j.find("child").get("link"): j.get("name")
    for j in ET.parse(root / "assets/robots/g1_description/g1_23dof.urdf")
    .getroot()
    .findall("joint")
    if j.get("type") != "fixed"
}
change = Rotation.from_euler("x", np.pi / 2)


def com(frame, names):
    pos = frame["base_translation_m"]
    data.qpos[:3] = [pos[0], -pos[2], pos[1]]
    quat = (change * Rotation.from_quat(frame["base_rotation_xyzw"])).as_quat()
    data.qpos[3:7] = quat[[3, 0, 1, 2]]
    for link, q in zip(names, frame["joint_positions_rad"], strict=True):
        data.qpos[model.jnt_qposadr[model.joint(joints[link]).id]] = q
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    assert data.time == 0
    c = data.subtree_com[0]
    return np.array([c[0], c[2], -c[1]])


for path_text in sys.argv[2:]:
    path = Path(path_text)
    p = json.loads(path.read_text())
    fs = [
        f
        for f in p["frames"]
        if p["takeoff_s"] + 0.025 <= f["time_s"] < p["touchdown_s"] - 0.025
    ]
    assert len(fs) >= 3 and not any(f["foot_contact"] for f in fs)
    positions = np.array([com(f, p["joint_link_names"]) for f in fs])
    times = np.array([f["time_s"] for f in fs]) - fs[0]["time_s"]
    gravity = np.array([0, -9.81, 0])
    velocity = np.array(fs[0]["com_velocity_m_s"])
    expected = (
        positions[0] + times[:, None] * velocity + 0.5 * times[:, None] ** 2 * gravity
    )
    velocity_expected = velocity + times[:, None] * gravity
    velocity_actual = np.array([f["com_velocity_m_s"] for f in fs])
    result = {
        "rollout": str(path),
        "rollout_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "audit": "source URDF forward kinematics only; external simulation never stepped",
        "native_mass_kg": p["mass_kg"],
        "fk_model_mass_kg": float(model.body_mass.sum()),
        "start_s": fs[0]["time_s"],
        "end_s": fs[-1]["time_s"],
        "sample_count": len(fs),
        "max_ballistic_position_residual_m": float(
            np.linalg.norm(positions - expected, axis=1).max()
        ),
        "max_ballistic_velocity_residual_m_s": float(
            np.linalg.norm(velocity_actual - velocity_expected, axis=1).max()
        ),
        "external_time_s": data.time,
    }
    print(json.dumps(result), flush=True)
