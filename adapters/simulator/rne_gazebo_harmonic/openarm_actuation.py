"""Pure validation and command realization for the Gazebo OpenArm adapter."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


def write_actuation_diagnostics(path: Path, output: dict[str, Any]) -> None:
    """Persist a compact diagnostic sidecar atomically for slow shared mounts."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as sink:
        json.dump(output, sink, separators=(",", ":"), allow_nan=False)
        sink.write("\n")
    temporary.replace(path)


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
    if config.get("derivative_filter_kind") not in {
        None,
        "first_order_low_pass_backward_euler_v1",
    }:
        raise ValueError("unsupported effort derivative filter")
    if config.get("derivative_filter_kind") is not None:
        time_constant_s = config.get("derivative_filter_time_constant_s")
        if (
            not isinstance(time_constant_s, (int, float))
            or not math.isfinite(time_constant_s)
            or time_constant_s <= 0.0
        ):
            raise ValueError("invalid effort derivative-filter time constant")
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
    velocity_limits = config.get("maximum_velocity_rad_s_by_joint")
    if velocity_limits is not None and (
        not isinstance(velocity_limits, list)
        or len(velocity_limits) != joint_count
        or not all(
            isinstance(value, (int, float))
            and math.isfinite(value)
            and value > 0.0
            for value in velocity_limits
        )
    ):
        raise ValueError("invalid effort-PD field maximum_velocity_rad_s_by_joint")
    friction = config.get("plant_coulomb_friction_nm", [0.0] * joint_count)
    transition = config.get(
        "plant_coulomb_transition_velocity_rad_s", [0.0] * joint_count
    )
    for key, values in (
        ("plant_coulomb_friction_nm", friction),
        ("plant_coulomb_transition_velocity_rad_s", transition),
    ):
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
    if any(magnitude > 0.0 and width <= 0.0 for magnitude, width in zip(friction, transition)):
        raise ValueError("nonzero Coulomb friction requires a positive transition velocity")
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
    if any(
        magnitude > 0.0 and index not in effort_joint_indices
        for index, magnitude in enumerate(friction)
    ):
        raise ValueError("Coulomb friction requires an effort-controlled joint")
    return mode, substeps, frozenset(effort_joint_indices)


def regularized_coulomb_effort(
    magnitude_nm: float, transition_velocity_rad_s: float, velocity_rad_s: float
) -> float:
    """Returns `-magnitude*tanh(velocity/transition)` in newton-metres."""
    if not all(
        isinstance(value, (int, float)) and math.isfinite(value)
        for value in (magnitude_nm, transition_velocity_rad_s, velocity_rad_s)
    ) or magnitude_nm < 0.0 or transition_velocity_rad_s < 0.0:
        raise ValueError("invalid regularized Coulomb-friction input")
    if magnitude_nm == 0.0:
        return 0.0
    if transition_velocity_rad_s == 0.0:
        raise ValueError("nonzero Coulomb friction requires a positive transition velocity")
    return -magnitude_nm * math.tanh(velocity_rad_s / transition_velocity_rad_s)


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


def low_pass_velocity(
    previous_rad_s: float,
    measured_rad_s: float,
    substep_period_s: float,
    time_constant_s: float,
) -> float:
    """Applies a deterministic backward-Euler first-order low-pass filter."""
    if (
        not all(
            math.isfinite(value)
            for value in (
                previous_rad_s,
                measured_rad_s,
                substep_period_s,
                time_constant_s,
            )
        )
        or substep_period_s <= 0.0
        or time_constant_s <= 0.0
    ):
        raise ValueError("invalid derivative-filter input")
    measurement_weight = substep_period_s / (time_constant_s + substep_period_s)
    return previous_rad_s + measurement_weight * (measured_rad_s - previous_rad_s)


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
    applied = max(-limit, min(limit, raw))
    velocity_limits = config.get("maximum_velocity_rad_s_by_joint")
    if (
        kind == "effort_nm"
        and velocity_limits is not None
        and applied * velocity_rad_s > 0.0
    ):
        drive_fraction = max(
            0.0, min(1.0, 1.0 - abs(velocity_rad_s) / velocity_limits[index])
        )
        applied *= drive_fraction
    return RealizedJointCommand(
        kind=kind,
        raw=raw,
        applied=applied,
        limit=limit,
        saturated=abs(raw) > limit or applied != raw,
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
    measured_velocities_rad_s: list[list[float]] = field(init=False)
    feedback_velocities_rad_s: list[list[float]] = field(init=False)
    passive_coulomb_efforts_nm: list[list[float]] = field(init=False)
    backend_commands: list[list[float]] = field(init=False)

    def __post_init__(self) -> None:
        self.kinds = [None] * self.joint_count
        self.raw_commands = [[] for _ in range(self.joint_count)]
        self.applied_commands = [[] for _ in range(self.joint_count)]
        self.saturation_counts = [0] * self.joint_count
        self.initial_position_error_rad = [None] * self.joint_count
        self.measured_velocities_rad_s = [[] for _ in range(self.joint_count)]
        self.feedback_velocities_rad_s = [[] for _ in range(self.joint_count)]
        self.passive_coulomb_efforts_nm = [[] for _ in range(self.joint_count)]
        self.backend_commands = [[] for _ in range(self.joint_count)]

    def record(
        self,
        index: int,
        command: RealizedJointCommand,
        position_error_rad: float,
        measured_velocity_rad_s: float,
        feedback_velocity_rad_s: float,
        passive_coulomb_effort_nm: float = 0.0,
        backend_command: float | None = None,
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
        self.measured_velocities_rad_s[index].append(measured_velocity_rad_s)
        self.feedback_velocities_rad_s[index].append(feedback_velocity_rad_s)
        self.passive_coulomb_efforts_nm[index].append(passive_coulomb_effort_nm)
        self.backend_commands[index].append(
            command.applied if backend_command is None else backend_command
        )
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
            "schema_version": 2,
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
            "joint_measured_velocity_peak_abs_rad_s": [
                max(abs(value) for value in values)
                for values in self.measured_velocities_rad_s
            ],
            "joint_derivative_feedback_velocity_peak_abs_rad_s": [
                max(abs(value) for value in values)
                for values in self.feedback_velocities_rad_s
            ],
            "joint_passive_coulomb_effort_min_nm": [
                min(values) for values in self.passive_coulomb_efforts_nm
            ],
            "joint_passive_coulomb_effort_max_nm": [
                max(values) for values in self.passive_coulomb_efforts_nm
            ],
            "joint_passive_coulomb_effort_mean_nm": [
                sum(values) / len(values) for values in self.passive_coulomb_efforts_nm
            ],
            "joint_backend_command_min": [min(values) for values in self.backend_commands],
            "joint_backend_command_max": [max(values) for values in self.backend_commands],
            "joint_backend_command_mean": [
                sum(values) / len(values) for values in self.backend_commands
            ],
        }
