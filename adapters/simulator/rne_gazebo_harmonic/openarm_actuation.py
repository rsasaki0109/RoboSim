"""Pure validation and command realization for the Gazebo OpenArm adapter."""

from __future__ import annotations

import math
from typing import Any


def validate_actuation(config: dict[str, Any], joint_count: int) -> tuple[str, int, int]:
    mode = config.get("actuation_mode", "velocity_servo")
    substeps = config.get("physics_substeps_per_control_step", 1)
    if mode not in {"velocity_servo", "effort_pd"}:
        raise ValueError("unsupported Gazebo actuation mode")
    if not isinstance(substeps, int) or substeps < 1:
        raise ValueError("physics substeps must be a positive integer")
    if mode == "velocity_servo":
        return mode, substeps, 0
    if config.get("saturation_behavior") != "clamp_each_joint_effort_before_pre_update":
        raise ValueError("effort-PD saturation behavior is not declared")
    if config.get("failure_behavior") != "reject_invalid_configuration_before_simulator_start":
        raise ValueError("effort-PD failure behavior is not declared")
    for key in ("stiffness_nm_per_rad", "damping_nm_s_per_rad", "maximum_effort_nm"):
        values = config.get(key)
        if (
            not isinstance(values, list)
            or len(values) != joint_count
            or not all(
                isinstance(value, (int, float))
                and math.isfinite(value)
                and value >= 0.0
                for value in values
            )
        ):
            raise ValueError(f"invalid effort-PD field {key}")
    effort_joint_count = config.get("effort_joint_count")
    if (
        not isinstance(effort_joint_count, int)
        or effort_joint_count < 1
        or effort_joint_count > joint_count
    ):
        raise ValueError("invalid effort-controlled joint count")
    return mode, substeps, effort_joint_count


def realize_joint_command(
    config: dict[str, Any],
    mode: str,
    effort_joint_count: int,
    index: int,
    target_rad: float,
    position_rad: float,
    velocity_rad_s: float,
) -> tuple[str, float]:
    if mode == "effort_pd" and index < effort_joint_count:
        effort_nm = (
            config["stiffness_nm_per_rad"][index] * (target_rad - position_rad)
            - config["damping_nm_s_per_rad"][index] * velocity_rad_s
        )
        limit_nm = config["maximum_effort_nm"][index]
        return "effort_nm", max(-limit_nm, min(limit_nm, effort_nm))
    velocity_rad_s = config["position_gain_s_inv"] * (target_rad - position_rad)
    limit_rad_s = config["maximum_velocity_rad_s"]
    return "velocity_rad_s", max(-limit_rad_s, min(limit_rad_s, velocity_rad_s))
