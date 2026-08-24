"""Pure validation and command realization for the Gazebo OpenArm adapter."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


def validate_actuation(
    config: dict[str, Any], joint_count: int
) -> tuple[str, int, frozenset[int]]:
    mode = config.get("actuation_mode", "velocity_servo")
    substeps = config.get("physics_substeps_per_control_step", 1)
    if mode not in {"velocity_servo", "effort_pd"}:
        raise ValueError("unsupported Gazebo actuation mode")
    if not isinstance(substeps, int) or substeps < 1:
        raise ValueError("physics substeps must be a positive integer")
    if mode == "velocity_servo":
        return mode, substeps, frozenset()
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
    effort_joint_indices = config.get("effort_joint_indices")
    if (
        not isinstance(effort_joint_indices, list)
        or not effort_joint_indices
        or any(
            not isinstance(index, int) or index < 0 or index >= joint_count
            for index in effort_joint_indices
        )
        or len(set(effort_joint_indices)) != len(effort_joint_indices)
    ):
        raise ValueError("invalid effort-controlled joint indices")
    return mode, substeps, frozenset(effort_joint_indices)


def realize_joint_command(
    config: dict[str, Any],
    mode: str,
    effort_joint_indices: frozenset[int],
    index: int,
    target_rad: float,
    position_rad: float,
    velocity_rad_s: float,
) -> tuple[str, float]:
    command = realize_joint_command_diagnostic(
        config,
        mode,
        effort_joint_indices,
        index,
        target_rad,
        position_rad,
        velocity_rad_s,
    )
    return command.kind, command.applied


@dataclass(frozen=True)
class RealizedJointCommand:
    """One bounded command exactly as presented to a Gazebo joint."""

    kind: str
    raw: float
    applied: float
    limit: float
    saturated: bool


def realize_joint_command_diagnostic(
    config: dict[str, Any],
    mode: str,
    effort_joint_indices: frozenset[int],
    index: int,
    target_rad: float,
    position_rad: float,
    velocity_rad_s: float,
) -> RealizedJointCommand:
    """Returns raw and bounded commands without changing realization semantics."""
    if mode == "effort_pd" and index in effort_joint_indices:
        raw = (
            config["stiffness_nm_per_rad"][index] * (target_rad - position_rad)
            - config["damping_nm_s_per_rad"][index] * velocity_rad_s
        )
        limit = config["maximum_effort_nm"][index]
        kind = "effort_nm"
    else:
        raw = config["position_gain_s_inv"] * (target_rad - position_rad)
        limit = config["maximum_velocity_rad_s"]
        kind = "velocity_rad_s"
    return RealizedJointCommand(
        kind=kind,
        raw=raw,
        applied=max(-limit, min(limit, raw)),
        limit=limit,
        saturated=abs(raw) > limit,
    )


@dataclass
class ActuationDiagnosticAccumulator:
    """Accumulates commands actually issued across one fixed control step."""

    joint_count: int
    kinds: list[str | None] = field(init=False)
    raw_commands: list[list[float]] = field(init=False)
    applied_commands: list[list[float]] = field(init=False)
    saturation_counts: list[int] = field(init=False)
    initial_position_error_rad: list[float | None] = field(init=False)

    def __post_init__(self) -> None:
        self.kinds = [None] * self.joint_count
        self.raw_commands = [[] for _ in range(self.joint_count)]
        self.applied_commands = [[] for _ in range(self.joint_count)]
        self.saturation_counts = [0] * self.joint_count
        self.initial_position_error_rad = [None] * self.joint_count

    def record(
        self,
        index: int,
        command: RealizedJointCommand,
        position_error_rad: float,
    ) -> None:
        """Records one pre-update realization for one joint."""
        if not 0 <= index < self.joint_count:
            raise ValueError("diagnostic joint index is out of range")
        prior_kind = self.kinds[index]
        if prior_kind is not None and prior_kind != command.kind:
            raise ValueError("joint command kind changed within one control step")
        self.kinds[index] = command.kind
        self.raw_commands[index].append(command.raw)
        self.applied_commands[index].append(command.applied)
        self.saturation_counts[index] += int(command.saturated)
        if self.initial_position_error_rad[index] is None:
            self.initial_position_error_rad[index] = position_error_rad

    def finish(
        self,
        expected_substeps: int,
        final_position_error_rad: list[float],
    ) -> dict[str, Any]:
        """Builds a deterministic, unit-labelled diagnostic for one step."""
        if len(final_position_error_rad) != self.joint_count:
            raise ValueError("final diagnostic error width mismatch")
        if any(len(values) != expected_substeps for values in self.applied_commands):
            raise ValueError("Gazebo pre-update count differs from declared substeps")
        if any(kind is None for kind in self.kinds):
            raise ValueError("missing realized joint command diagnostic")
        return {
            "schema_version": 1,
            "substep_count": expected_substeps,
            "joint_command_kind": self.kinds,
            "joint_initial_position_error_rad": self.initial_position_error_rad,
            "joint_final_position_error_rad": final_position_error_rad,
            "joint_raw_command_peak_abs": [
                max(abs(value) for value in values) for values in self.raw_commands
            ],
            "joint_applied_command_min": [min(values) for values in self.applied_commands],
            "joint_applied_command_max": [max(values) for values in self.applied_commands],
            "joint_applied_command_mean": [
                sum(values) / len(values) for values in self.applied_commands
            ],
            "joint_saturation_substep_count": self.saturation_counts,
            "joint_saturation_fraction": [
                count / expected_substeps for count in self.saturation_counts
            ],
        }
