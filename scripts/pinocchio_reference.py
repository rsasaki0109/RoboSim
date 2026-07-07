#!/usr/bin/env python3
"""Generate Pinocchio (or equivalent) reference trajectories for dynamics validation.

Writes JSON goldens under ``tests/golden/dynamics/``. On Linux CI, install Pinocchio
with ``pip install pin`` and run without ``--lagrangian-fallback``. On Windows and
other hosts without ``pin``, pass ``--lagrangian-fallback`` to integrate the same
serial-chain equations of motion (equivalent to Pinocchio ABA for these models).

Usage::

    python scripts/pinocchio_reference.py --write-golden
    python scripts/pinocchio_reference.py --write-golden --lagrangian-fallback
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, List, Sequence

G = 9.81
HZ = 60.0
DURATION_SINGLE_PENDULUM_S = 2.0
DURATION_TWO_LINK_S = 0.25

REPO_ROOT = Path(__file__).resolve().parents[1]
GOLDEN_DIR = REPO_ROOT / "tests" / "golden" / "dynamics"

try:
    import pinocchio as pin
    import numpy as np

    HAS_PIN = True
except ImportError:
    HAS_PIN = False


@dataclass(frozen=True)
class SphereLink:
    """Point-like link with sphere inertia (matches RNE sphere colliders)."""

    length_m: float
    mass_kg: float
    radius_m: float

    @property
    def inertia_com(self) -> float:
        return 2.0 / 5.0 * self.mass_kg * self.radius_m**2


def sphere_inertia_pin(mass_kg: float, radius_m: float) -> Any:
    i = 2.0 / 5.0 * mass_kg * radius_m**2
    return np.diag([i, i, i])


def integrate_pin(
    model: Any,
    q0: Sequence[float],
    lengths: Sequence[float],
    hz: float,
    duration_s: float,
) -> list[dict[str, Any]]:
    data = model.createData()
    dt = 1.0 / hz
    steps = int(duration_s * hz)
    q = pin.neutral(model).copy()
    q[: len(q0)] = np.asarray(q0, dtype=float)
    v = np.zeros(model.nv)

    samples: list[dict[str, Any]] = []
    for step in range(steps + 1):
        q_list = [float(x) for x in q]
        tip = tip_from_angles_planar(lengths, q_list)
        samples.append(
            {
                "step": step,
                "t_s": step * dt,
                "joint_q_rad": q_list,
                "tip_m": [tip[0], tip[1], tip[2]],
            }
        )
        if step == steps:
            break
        tau = np.zeros(model.nv)
        a = pin.aba(model, data, q, v, tau)
        v = v + a * dt
        q = pin.integrate(model, q, v * dt)
    return samples


def build_single_pendulum_pin(link: SphereLink) -> Any:
    model = pin.Model()
    axis = np.array([0.0, 0.0, 1.0])
    joint_id = model.addJoint(0, pin.JointModelRevoluteUnaligned(axis), pin.SE3.Identity(), "j1")
    com = np.array([0.0, -link.length_m, 0.0])
    inertia = pin.Inertia(link.mass_kg, com, sphere_inertia_pin(link.mass_kg, link.radius_m))
    model.appendBodyToJoint(joint_id, inertia, pin.SE3.Identity())
    model.gravity.linear = np.array([0.0, -G, 0.0])
    return model


def build_two_link_pin(link1: SphereLink, link2: SphereLink) -> Any:
    model = pin.Model()
    axis = np.array([0.0, 0.0, 1.0])
    j1 = model.addJoint(0, pin.JointModelRevoluteUnaligned(axis), pin.SE3.Identity(), "j1")
    com1 = np.array([0.0, -link1.length_m, 0.0])
    inertia1 = pin.Inertia(link1.mass_kg, com1, sphere_inertia_pin(link1.mass_kg, link1.radius_m))
    model.appendBodyToJoint(j1, inertia1, pin.SE3.Identity())

    placement_j2 = pin.SE3(np.eye(3), np.array([0.0, -link1.length_m, 0.0]))
    j2 = model.addJoint(j1, pin.JointModelRevoluteUnaligned(axis), placement_j2, "j2")
    com2 = np.array([0.0, -link2.length_m, 0.0])
    inertia2 = pin.Inertia(link2.mass_kg, com2, sphere_inertia_pin(link2.mass_kg, link2.radius_m))
    model.appendBodyToJoint(j2, inertia2, pin.SE3.Identity())
    model.gravity.linear = np.array([0.0, -G, 0.0])
    return model


def tip_from_angles_planar(lengths: Sequence[float], q: Sequence[float]) -> tuple[float, float, float]:
    """Planar tip position; q[0] from vertical, q[i] relative for i > 0."""
    angle = q[0]
    x = lengths[0] * math.sin(angle)
    y = -lengths[0] * math.cos(angle)
    for i in range(1, len(lengths)):
        angle += q[i]
        x += lengths[i] * math.sin(angle)
        y -= lengths[i] * math.cos(angle)
    return (x, y, 0.0)


def mass_matrix_two_link(
    m1: float, m2: float, l1: float, l2: float, i1: float, i2: float, q1: float, q2: float
) -> tuple[float, float, float]:
    """2-DOF planar serial chain inertia matrix about revolute joints."""
    c2 = math.cos(q2)
    h1 = i1 + m1 * l1 * l1
    h2 = i2 + m2 * l2 * l2
    h3 = m2 * l1 * l2 * c2
    m11 = h1 + h2 + 2.0 * h3
    m12 = h2 + h3
    m22 = h2
    return m11, m12, m22


def coriolis_two_link(
    m2: float, l1: float, l2: float, q2: float, dq1: float, dq2: float
) -> tuple[float, float]:
    s2 = math.sin(q2)
    h = -m2 * l1 * l2 * s2
    c1 = h * (2.0 * dq1 * dq2 + dq2 * dq2)
    c2 = h * dq1 * dq1
    return c1, c2


def gravity_two_link(
    m1: float, m2: float, l1: float, l2: float, q1: float, q2: float
) -> tuple[float, float]:
    g1 = (m1 + m2) * G * l1 * math.sin(q1) + m2 * G * l2 * math.sin(q1 + q2)
    g2 = m2 * G * l2 * math.sin(q1 + q2)
    return g1, g2


def integrate_lagrangian(
    lengths: Sequence[float],
    masses: Sequence[float],
    inertias_com: Sequence[float],
    q0: Sequence[float],
    hz: float,
    duration_s: float,
) -> list[dict[str, Any]]:
    """Semi-implicit Euler on the serial-chain Lagrangian (ABA-equivalent)."""
    dt = 1.0 / hz
    steps = int(duration_s * hz)
    n = len(lengths)
    q = list(q0)
    dq = [0.0] * n

    def step_one_dof(m: float, i_com: float, length: float, angle: float, dangle: float) -> tuple[float, float]:
        inertia = i_com + m * length * length
        ddq = -(G / length) * math.sin(angle)
        dangle_n = dangle + ddq * dt
        angle_n = angle + dangle_n * dt
        return angle_n, dangle_n

    samples: list[dict[str, Any]] = []
    for step in range(steps + 1):
        tip = tip_from_angles_planar(lengths, q)
        samples.append(
            {
                "step": step,
                "t_s": step * dt,
                "joint_q_rad": list(q),
                "tip_m": [tip[0], tip[1], tip[2]],
            }
        )
        if step == steps:
            break

        if n == 1:
            q[0], dq[0] = step_one_dof(masses[0], inertias_com[0], lengths[0], q[0], dq[0])
        elif n == 2:
            m1, m2 = masses
            l1, l2 = lengths
            i1, i2 = inertias_com
            q1, q2 = q
            dq1, dq2 = dq
            m11, m12, m22 = mass_matrix_two_link(m1, m2, l1, l2, i1, i2, q1, q2)
            c1, c2 = coriolis_two_link(m2, l1, l2, q2, dq1, dq2)
            g1, g2 = gravity_two_link(m1, m2, l1, l2, q1, q2)
            rhs1 = -c1 - g1
            rhs2 = -c2 - g2
            det = m11 * m22 - m12 * m12
            ddq1 = (m22 * rhs1 - m12 * rhs2) / det
            ddq2 = (-m12 * rhs1 + m11 * rhs2) / det
            dq1 += ddq1 * dt
            dq2 += ddq2 * dt
            q1 += dq1 * dt
            q2 += dq2 * dt
            q[0], q[1] = q1, q2
            dq[0], dq[1] = dq1, dq2
        else:
            raise ValueError(f"lagrangian fallback supports 1-2 DOF, got {n}")
    return samples


def write_golden(
    name: str,
    parameters: dict[str, Any],
    samples: list[dict[str, Any]],
    engine: str,
    hz: float,
    duration_s: float,
) -> None:
    GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "generator": "scripts/pinocchio_reference.py",
        "generator_version": 1,
        "engine": engine,
        "scenario": name,
        "hz": hz,
        "gravity_m_s2": [0.0, -G, 0.0],
        "duration_s": duration_s,
        "parameters": parameters,
        "samples": samples,
    }
    path = GOLDEN_DIR / f"pinocchio_{name}.json"
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {path} ({len(samples)} samples, engine={engine})")


def scenario_single_pendulum(integrate: Callable[..., list[dict[str, Any]]], engine: str) -> None:
    link = SphereLink(length_m=2.0, mass_kg=1.0, radius_m=0.1)
    q0 = [math.radians(5.0)]
    if HAS_PIN and engine == "pinocchio":
        model = build_single_pendulum_pin(link)
        samples = integrate(model, q0, [link.length_m], HZ, DURATION_SINGLE_PENDULUM_S)
    else:
        samples = integrate(
            [link.length_m],
            [link.mass_kg],
            [link.inertia_com],
            q0,
            HZ,
            DURATION_SINGLE_PENDULUM_S,
        )
    write_golden(
        "single_pendulum",
        {
            "length_m": link.length_m,
            "mass_kg": link.mass_kg,
            "radius_m": link.radius_m,
            "initial_angle_rad": q0[0],
        },
        samples,
        engine,
        HZ,
        DURATION_SINGLE_PENDULUM_S,
    )


def scenario_two_link_planar(integrate: Callable[..., list[dict[str, Any]]], engine: str) -> None:
    link1 = SphereLink(length_m=1.0, mass_kg=1.0, radius_m=0.08)
    link2 = SphereLink(length_m=1.0, mass_kg=0.8, radius_m=0.08)
    q0 = [math.radians(30.0), math.radians(-20.0)]
    if HAS_PIN and engine == "pinocchio":
        model = build_two_link_pin(link1, link2)
        samples = integrate(model, q0, [link1.length_m, link2.length_m], HZ, DURATION_TWO_LINK_S)
    else:
        samples = integrate(
            [link1.length_m, link2.length_m],
            [link1.mass_kg, link2.mass_kg],
            [link1.inertia_com, link2.inertia_com],
            q0,
            HZ,
            DURATION_TWO_LINK_S,
        )
    write_golden(
        "two_link_planar",
        {
            "link1_length_m": link1.length_m,
            "link1_mass_kg": link1.mass_kg,
            "link1_radius_m": link1.radius_m,
            "link2_length_m": link2.length_m,
            "link2_mass_kg": link2.mass_kg,
            "link2_radius_m": link2.radius_m,
            "initial_q_rad": q0,
        },
        samples,
        engine,
        HZ,
        DURATION_TWO_LINK_S,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--write-golden",
        action="store_true",
        help="Write reference JSON files to tests/golden/dynamics/",
    )
    parser.add_argument(
        "--lagrangian-fallback",
        action="store_true",
        help="Use explicit Lagrangian integrator when pin is unavailable",
    )
    args = parser.parse_args()

    if not args.write_golden:
        parser.print_help()
        return 1

    if HAS_PIN and not args.lagrangian_fallback:
        engine = "pinocchio"
        integrate_fn: Callable[..., list[dict[str, Any]]] = integrate_pin
    else:
        if not args.lagrangian_fallback and not HAS_PIN:
            print(
                "pinocchio not installed; re-run with --lagrangian-fallback "
                "or install with: pip install pin",
                file=sys.stderr,
            )
            return 1
        engine = "lagrangian_fallback"
        integrate_fn = integrate_lagrangian

    scenario_single_pendulum(integrate_fn, engine)
    scenario_two_link_planar(integrate_fn, engine)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
