#!/usr/bin/env python3
"""Reintegrate a saved G1 flight with ABA and held joint torques, without contacts."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

# Explicit sibling import also works with Python -I.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1_trajopt_reference as reference
import numpy as np
import pinocchio as pin


def midpoint_step(model, data, q, v, torque, dt_s):
    """Explicit midpoint in the local configuration chart using forward dynamics."""
    acceleration = pin.aba(model, data, q, v, torque).copy()
    middle_q = pin.integrate(model, q, 0.5 * dt_s * v)
    middle_v = v + 0.5 * dt_s * acceleration
    middle_acceleration = pin.aba(model, data, middle_q, middle_v, torque).copy()
    return pin.integrate(model, q, dt_s * middle_v), v + dt_s * middle_acceleration


def check(directory, dt_s, kp_nm_rad=0.0, kd_nm_s_rad=0.0):
    """Report open-loop free-flight error; this does not validate landing/contact."""
    if not np.isfinite(dt_s) or dt_s <= 0:
        raise ValueError("dt_s must be finite and positive")
    if any(not np.isfinite(gain) or gain < 0 for gain in (kp_nm_rad, kd_nm_s_rad)):
        raise ValueError("feedback gains must be finite and nonnegative")
    summary = json.loads((directory / "summary.json").read_text())
    raw = (directory / "trajectory.json").read_bytes()
    trajectory = json.loads(raw)
    if (
        summary["urdf_sha256"]
        != hashlib.sha256(reference.URDF.read_bytes()).hexdigest()
    ):
        raise ValueError("URDF checksum mismatch")
    if summary["trajectory_sha256"] != hashlib.sha256(raw).hexdigest():
        raise ValueError("trajectory checksum mismatch")
    model, _, names, _, _ = reference.build_model(
        summary["contacts"], summary["mass_policy"]
    )
    if names != trajectory["joint_names"] or trajectory["world_up"] != "Z":
        raise ValueError("joint order or world convention mismatch")
    nodes = trajectory["nodes"]
    indices = [k for k, node in enumerate(nodes) if not node["contact_forces_world_n"]]
    if (
        not indices
        or indices != list(range(indices[0], indices[-1] + 1))
        or indices[-1] + 1 >= len(nodes)
    ):
        raise ValueError("one complete flight interval is required")
    data = model.createData()
    q = np.array(nodes[indices[0]]["q_xyzw"])
    v = np.array(nodes[indices[0]]["v_body"])
    initial_h = pin.computeCentroidalMomentum(model, data, q, v).vector.copy()
    initial_com = pin.centerOfMass(model, data, q).copy()
    mass = pin.computeTotalMass(model)
    start_s = nodes[indices[0]]["time_s"]
    max_joint_violation = 0.0
    substeps = 0
    saturated_steps = 0
    peak_torque_nm = 0.0
    for k in indices:
        duration = nodes[k + 1]["time_s"] - nodes[k]["time_s"]
        if duration <= 0:
            raise ValueError("timestamps must increase")
        count = math.ceil(duration / dt_s)
        step_s = duration / count
        feedforward = np.array(nodes[k]["tau_nm"])
        for step in range(count):
            alpha = step / count
            desired_q = (1 - alpha) * np.array(nodes[k]["q_xyzw"])[
                7:
            ] + alpha * np.array(nodes[k + 1]["q_xyzw"])[7:]
            desired_v = (1 - alpha) * np.array(nodes[k]["v_body"])[
                6:
            ] + alpha * np.array(nodes[k + 1]["v_body"])[6:]
            commanded = (
                feedforward
                + kp_nm_rad * (desired_q - q[7:])
                + kd_nm_s_rad * (desired_v - v[6:])
            )
            limited = np.clip(commanded, -model.effortLimit[6:], model.effortLimit[6:])
            saturated_steps += int(np.any(np.abs(commanded) > model.effortLimit[6:]))
            peak_torque_nm = max(peak_torque_nm, float(np.max(np.abs(limited))))
            torque = np.concatenate((np.zeros(6), limited))
            q, v = midpoint_step(model, data, q, v, torque, step_s)
            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
                raise ValueError("non-finite free-flight rollout")
            max_joint_violation = max(
                max_joint_violation,
                float(np.max(model.lowerPositionLimit[7:] - q[7:])),
                float(np.max(q[7:] - model.upperPositionLimit[7:])),
            )
            substeps += 1
    target = nodes[indices[-1] + 1]
    duration_s = target["time_s"] - start_s
    com = pin.centerOfMass(model, data, q).copy()
    h = pin.computeCentroidalMomentum(model, data, q, v).vector.copy()
    gravity = np.array([0.0, 0.0, -9.81])
    expected_com = (
        initial_com + initial_h[:3] / mass * duration_s + 0.5 * gravity * duration_s**2
    )
    error = pin.difference(model, np.array(target["q_xyzw"]), q)
    conservation = reference.flight_momentum_errors(
        [
            {"time_s": 0.0, "momentum_world": initial_h.tolist()},
            {"time_s": duration_s, "momentum_world": h.tolist()},
        ],
        mass,
    )
    return {
        "schema": "rne.g1.free_flight_check.v1",
        "trajectory_sha256": summary["trajectory_sha256"],
        "max_step_s": dt_s,
        "substeps": substeps,
        "flight_duration_s": duration_s,
        "torque_interpolation": "hold left node; zero external contacts",
        "kp_nm_rad": kp_nm_rad,
        "kd_nm_s_rad": kd_nm_s_rad,
        "feedback_reference": "linear joint position/velocity interpolation",
        "saturated_steps": saturated_steps,
        "peak_torque_nm": peak_torque_nm,
        "final_root_position_error_m": float(
            np.linalg.norm(q[:3] - target["q_xyzw"][:3])
        ),
        "final_root_rotation_error_rad": float(np.linalg.norm(error[3:6])),
        "final_joint_position_error_rad": float(np.max(np.abs(error[6:]))),
        "final_joint_velocity_error_rad_s": float(
            np.max(np.abs(v[6:] - np.array(target["v_body"])[6:]))
        ),
        "max_joint_limit_violation_rad": max_joint_violation,
        "final_com_reference_error_m": float(
            np.linalg.norm(com - target["com_world_m"])
        ),
        "ballistic_com_error_m": float(np.linalg.norm(com - expected_com)),
        "linear_momentum_error_ns": float(
            np.linalg.norm(h[:3] - initial_h[:3] - mass * gravity * duration_s)
        ),
        "angular_momentum_error_nms": float(np.linalg.norm(h[3:] - initial_h[3:])),
        "numerical_momentum_screen_passed": conservation[
            "flight_momentum_screen_passed"
        ],
        "plant_validated": False,
        "contact_landing_validated": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--dt-s", type=float, default=0.001)
    parser.add_argument("--kp-nm-rad", type=float, default=0.0)
    parser.add_argument("--kd-nm-s-rad", type=float, default=0.0)
    args = parser.parse_args()
    print(
        json.dumps(
            check(args.directory, args.dt_s, args.kp_nm_rad, args.kd_nm_s_rad),
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
