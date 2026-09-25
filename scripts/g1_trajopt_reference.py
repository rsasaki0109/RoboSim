#!/usr/bin/env python3
"""Offline, non-RL G1 direct-transcription benchmark against se3_trajopt.

External dependencies stay outside the Rust workspace. See
docs/G1_TRAJOPT_REFERENCE.md for setup, conventions, and interpretation.
The exit gate evaluates constraints AND the requested maneuver; a trajectory
is never presented as a validated physics-backend replay.
"""

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

# Keep linear algebra deterministic within the pinned environment.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import cyipopt
import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM_REVISION = "1bbadc9573b2989a0f414888d4fa4af137d57db9"
URDF = ROOT / "assets/robots/g1_description/g1_23dof.urdf"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--upstream", type=Path, default=ROOT / "target/research/se3_trajopt"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--task", choices=("stand", "jump", "backflip"), default="jump")
    parser.add_argument("--contacts", choices=("sole", "toes"), default="sole")
    parser.add_argument("--mass-policy", choices=("rne", "declared"), default="rne")
    parser.add_argument(
        "--integration",
        choices=("body-euler", "world", "momentum"),
        default="body-euler",
    )
    parser.add_argument("--dt-s", type=float, default=0.03)
    parser.add_argument("--push-steps", type=int, default=15)
    parser.add_argument("--flight-steps", type=int, default=10)
    parser.add_argument("--landing-steps", type=int, default=15)
    parser.add_argument("--max-iterations", type=int, default=300)
    parser.add_argument("--max-wall-time-s", type=float, default=180.0)
    parser.add_argument("--tolerance", type=float, default=1e-4)
    parser.add_argument("--check-jacobian", action="store_true")
    parser.add_argument(
        "--feasibility-only",
        action="store_true",
        help="solve constraints with zero trajectory cost",
    )
    parser.add_argument(
        "--project-warm-start-dynamics",
        action="store_true",
        help="recompute warm-start acceleration from interpolated torque and forces",
    )
    parser.add_argument(
        "--warm-start",
        type=Path,
        help="previous output directory with matching model and phase durations",
    )
    args = parser.parse_args()
    if any(
        not np.isfinite(x) or x <= 0
        for x in (args.dt_s, args.max_wall_time_s, args.tolerance)
    ):
        parser.error(
            "time steps, time limit, and tolerance must be finite and positive"
        )
    if (
        min(args.push_steps, args.flight_steps, args.landing_steps, args.max_iterations)
        < 1
    ):
        parser.error("step counts and iteration budget must be positive")
    return args


def load_upstream(path):
    """Load a pinned, unmodified checkout; isolate its import-time CLI parser."""
    revision = subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()
    dirty = subprocess.check_output(
        ["git", "-C", str(path), "status", "--porcelain", "--untracked-files=no"],
        text=True,
    ).strip()
    if revision != UPSTREAM_REVISION or dirty:
        raise ValueError(
            "se3_trajopt must be an unmodified checkout of " + UPSTREAM_REVISION
        )
    sys.path.insert(0, str(path.resolve() / "src"))
    saved_argv = sys.argv
    try:
        sys.argv = [saved_argv[0]]
        from nltrajopt import utils
        from nltrajopt.constraint_models import (
            SemiEulerIntegration,
            TerrainGridContactConstraints,
            TerrainGridFrictionConstraints,
            TimeConstraint,
            WholeBodyDynamics,
        )
        from nltrajopt.cost_models import ConfigurationCost, JointAccelerationCost
        from nltrajopt.node import Node
        from nltrajopt.trajectory_optimization import NLTrajOpt
        from terrain.terrain_grid import TerrainGrid
    finally:
        sys.argv = saved_argv

    class CompleteTerminalFriction(TerrainGridFrictionConstraints):
        """Evaluate every terminal contact despite upstream's in-loop return.

        A next node without contacts makes the upstream loop visit every frame.
        Force-difference rows remain unbounded, as at all other stages here.
        """

        @staticmethod
        def successor(node):
            return (
                node if node is not None else SimpleNamespace(contact_phase_fnames=[])
            )

        def compute_constraints(self, node_curr, node_next, *args):
            return super().compute_constraints(
                node_curr, self.successor(node_next), *args
            )

        def compute_jacobians(self, node_curr, node_next, *args):
            return super().compute_jacobians(
                node_curr, self.successor(node_next), *args
            )

        def get_structure_ids(self, node_curr, node_next, *args):
            return super().get_structure_ids(
                node_curr, self.successor(node_next), *args
            )

    class WorldVelocityIntegration(SemiEulerIntegration):
        """Integrate base velocities in world coordinates, including frame rotation.

        Only this inexpensive kinematic constraint uses numerical derivatives;
        the inverse-dynamics Jacobian remains analytic.
        """

        def residual(self, current, following, x, model):
            q = utils.rep2pin(x[current.q_id])
            qn = utils.rep2pin(x[following.q_id])
            qp, vp = world_step(
                model, q, x[current.vq_id], x[current.aq_id], x[current.dt_id][0]
            )
            vn = x[following.vq_id].copy()
            rotation = pin.Quaternion(qn[3:7]).matrix()
            vn[:3], vn[3:6] = rotation @ vn[:3], rotation @ vn[3:6]
            return np.concatenate((pin.difference(model, qn, qp), vn - vp))

        def compute_constraints(self, current, following, x, c, model, data):
            if following is not None:
                residual = self.residual(current, following, x, model)
                c[current.c_q_integration_id] = residual[: model.nv]
                c[current.c_vq_integration_id] = residual[model.nv :]

        def compute_jacobians(self, current, following, x, jac, model, data):
            if following is None:
                return
            step = 1e-6
            trial = x.copy()
            for field in self.variable_fields(current, following):
                for column in range(field.start, field.stop):
                    trial[column] = x[column] + step
                    plus = self.residual(current, following, trial, model)
                    trial[column] = x[column] - step
                    minus = self.residual(current, following, trial, model)
                    trial[column] = x[column]
                    derivative = (plus - minus) / (2 * step)
                    jac[current.c_q_integration_id, column] = derivative[: model.nv]
                    jac[current.c_vq_integration_id, column] = derivative[model.nv :]

        def variable_fields(self, current, following):
            return (
                current.q_id,
                current.vq_id,
                current.aq_id,
                current.dt_id,
                following.q_id,
                following.vq_id,
            )

        def get_structure_ids(self, current, following, rows, cols):
            super().get_structure_ids(current, following, rows, cols)
            if following is not None:
                for field in (current.q_id, following.q_id):
                    for row in range(
                        current.c_vq_integration_id.start,
                        current.c_vq_integration_id.start + 6,
                    ):
                        for column in range(field.start, field.start + 6):
                            rows.append(row)
                            cols.append(column)

    class MomentumIntegration(WorldVelocityIntegration):
        """Use centroidal impulse balance and a ballistic CoM position update.

        Orientation and joints retain the world scheme's update. Stance uses
        a left-endpoint contact wrench; flight conserves centroidal momentum
        exactly up to the nonlinear constraint tolerance.
        """

        def residual(self, current, following, x, model):
            if not hasattr(self, "scratch"):
                self.scratch = model.createData()
            return momentum_residual(
                model,
                self.scratch,
                utils.rep2pin(x[current.q_id]),
                x[current.vq_id],
                x[current.aq_id],
                utils.rep2pin(x[following.q_id]),
                x[following.vq_id],
                x[current.dt_id][0],
                {name: x[field] for name, field in current.forces_ids.items()},
            )

        def variable_fields(self, current, following):
            return super().variable_fields(current, following) + tuple(
                current.forces_ids.values()
            )

        def get_structure_ids(self, current, following, rows, cols):
            super().get_structure_ids(current, following, rows, cols)
            if following is not None:
                affected = list(
                    range(
                        current.c_q_integration_id.start,
                        current.c_q_integration_id.start + 3,
                    )
                )
                affected += list(
                    range(
                        current.c_vq_integration_id.start,
                        current.c_vq_integration_id.start + 6,
                    )
                )
                for field in self.variable_fields(current, following):
                    for row in affected:
                        for column in range(field.start, field.stop):
                            rows.append(row)
                            cols.append(column)

    return {
        "NLTrajOpt": NLTrajOpt,
        "Node": Node,
        "utils": utils,
        "WholeBodyDynamics": WholeBodyDynamics,
        "TimeConstraint": TimeConstraint,
        "SemiEulerIntegration": SemiEulerIntegration,
        "WorldVelocityIntegration": WorldVelocityIntegration,
        "MomentumIntegration": MomentumIntegration,
        "TerrainGridContactConstraints": TerrainGridContactConstraints,
        "CompleteTerminalFriction": CompleteTerminalFriction,
        "ConfigurationCost": ConfigurationCost,
        "JointAccelerationCost": JointAccelerationCost,
        "TerrainGrid": TerrainGrid,
    }


def world_step(model, q, v, a, dt):
    """Predict configuration and velocities (base in world, joint rates unchanged).

    A spatial body-twist derivative a becomes the classical world linear
    acceleration R (a_linear + omega x v_linear).
    """
    rotation = pin.Quaternion(q[3:7]).matrix()
    linear_acceleration = rotation @ (a[:3] + np.cross(v[3:6], v[:3]))
    predicted_velocity = v + dt * a
    predicted_velocity[:3] = rotation @ v[:3] + dt * linear_acceleration
    predicted_velocity[3:6] = rotation @ (v[3:6] + dt * a[3:6])
    predicted_q = pin.integrate(model, q, (v + dt * a) * dt)
    predicted_q[:3] = (
        q[:3] + dt * (rotation @ v[:3]) + 0.5 * dt * dt * linear_acceleration
    )
    predicted_q[3:7] = pin.Quaternion(
        rotation @ pin.exp3(dt * v[3:6] + 0.5 * dt * dt * a[3:6])
    ).coeffs()
    return predicted_q, predicted_velocity


def momentum_residual(model, data, q, v, a, qn, vn, dt, forces_local):
    """Mixed position/joint-rate/centroidal-impulse residual, with explicit units.

    The first three position rows are world CoM displacement (m). The first six
    velocity rows are linear/angular impulse errors (N s / N m s).
    """
    mass = pin.computeTotalMass(model)
    h = pin.computeCentroidalMomentum(model, data, q, v).vector.copy()
    com = data.com[0].copy()
    pin.updateFramePlacements(model, data)
    force = np.array([0.0, 0.0, -mass * 9.81])
    moment = np.zeros(3)
    for name, local in forces_local.items():
        pose = data.oMf[model.getFrameId(name)]
        world = pose.rotation @ local
        force += world
        moment += np.cross(pose.translation - com, world)
    hn = pin.computeCentroidalMomentum(model, data, qn, vn).vector.copy()
    comn = data.com[0].copy()
    qp, _ = world_step(model, q, v, a, dt)
    dq = pin.difference(model, qn, qp)
    dq[:3] = comn - com - dt * h[:3] / mass - 0.5 * dt**2 * force / mass
    dv = vn - v - dt * a
    dv[:6] = hn - h - dt * np.concatenate((force, moment))
    return np.concatenate((dq, dv))


def native_torque_limit(name):
    """Match example 113's constant ceilings, not a hardware torque-speed envelope."""
    if "knee" in name:
        return 139.0
    if "ankle" in name:
        return 35.0
    if any(part in name for part in ("shoulder", "elbow", "wrist")):
        return 25.0
    return 88.0


def build_model(contact_kind, mass_policy="rne"):
    model = pin.buildModelFromUrdf(str(URDF), pin.JointModelFreeFlyer())
    if model.nv != 29 or model.nq != 30:
        raise ValueError("benchmark requires 23 actuated joints plus a free base")
    names = list(model.names)[2:]
    if mass_policy == "rne":
        # RNE's declared-inertia importer retains its 1 kg fallback for links
        # without inertial data. Pinocchio gives those fixed frames zero mass.
        # Mirror that explicit policy rather than silently comparing two robots.
        for link in ET.parse(URDF).getroot().findall("link"):
            if link.find("inertial") is None:
                frame = model.frames[model.getFrameId(link.attrib["name"])]
                model.appendBodyToJoint(
                    frame.parentJoint,
                    pin.Inertia(1.0, np.zeros(3), np.zeros((3, 3))),
                    frame.placement,
                )
    model.effortLimit[6:] = [native_torque_limit(name) for name in names]
    q = pin.neutral(model)
    for jid, name in enumerate(model.names):
        if jid < 2:
            continue
        value = 0.0
        for part, angle in (
            ("hip_pitch", -0.18),
            ("knee", 0.36),
            ("ankle_pitch", -0.18),
            ("elbow", 0.42),
        ):
            if part in name:
                value = angle
        if "shoulder_roll" in name:
            value = 0.20 if "left" in name else -0.20
        q[model.joints[jid].idx_q] = value
    # Flat feet for a consistent initial sole contact. The native gait's small
    # roll offsets are deliberately removed and recorded in the manifest.
    offsets = (
        [(0.09, 0.0, -0.035)]
        if contact_kind == "toes"
        else [(x, y, -0.035) for x in (-0.05, 0.09) for y in (-0.025, 0.025)]
    )
    frames = []
    for side in ("left", "right"):
        parent_id = model.getFrameId(side + "_ankle_roll_link")
        parent = model.frames[parent_id]
        for index, offset in enumerate(offsets):
            name = f"rne_{side}_contact_{index}"
            placement = parent.placement * pin.SE3(np.eye(3), np.array(offset))
            model.addFrame(
                pin.Frame(name, parent.parentJoint, parent_id, placement, pin.OP_FRAME)
            )
            frames.append(name)
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    q[2] -= min(data.oMf[model.getFrameId(name)].translation[2] for name in frames)
    return model, q, names, frames, offsets


def build_problem(args, api):
    model, q, names, frames, offsets = build_model(args.contacts, args.mass_policy)
    terrain = api["TerrainGrid"](10, 10, 0.7, -5.0, -5.0, 5.0, 5.0)
    flight_start = args.push_steps
    flight_end = flight_start + args.flight_steps
    horizon = flight_end + args.landing_steps
    nodes = []
    for k in range(horizon + 1):
        active = (
            [] if args.task != "stand" and flight_start <= k < flight_end else frames
        )
        node = api["Node"](model.nv, active, frames)
        node.dynamics_type = "whole_body_dynamics"
        node.constraints_list = [
            api["WholeBodyDynamics"](),
            api["TimeConstraint"](min_dt=args.dt_s, max_dt=args.dt_s, total_time=None),
            api[
                {
                    "world": "WorldVelocityIntegration",
                    "momentum": "MomentumIntegration",
                    "body-euler": "SemiEulerIntegration",
                }[args.integration]
            ](),
            api["TerrainGridContactConstraints"](terrain),
            api["CompleteTerminalFriction"](terrain, max_delta_force=-1.0),
        ]
        node.costs_list = [
            api["ConfigurationCost"](q[7:].copy(), np.eye(23) * 1e-3),
            api["JointAccelerationCost"](np.zeros(23), np.eye(23) * 1e-6),
        ]
        if getattr(args, "feasibility_only", False):
            node.costs_list = []
        nodes.append(node)
    problem = api["NLTrajOpt"](model, nodes, args.dt_s)
    problem.set_initial_pose(q)
    problem.set_target_pose(q)
    data = model.createData()
    mass = pin.computeTotalMass(model)
    for k, node in enumerate(nodes):
        problem.x0[node.dt_id] = args.dt_s
        guess = q.copy()
        if args.task != "stand" and flight_start <= k <= flight_end:
            t = (k - flight_start) * args.dt_s
            duration = args.flight_steps * args.dt_s
            guess[2] += 0.5 * 9.81 * t * (duration - t)
            problem.x0[node.vq_id.start + 2] = 9.81 * (duration / 2 - t)
            if args.task == "backflip":
                rotation = pin.rpy.rpyToMatrix(0.0, -2 * np.pi * t / duration, 0.0)
                guess[3:7] = pin.Quaternion(rotation).coeffs()
                # Body-frame translational velocity, not world vertical velocity.
                problem.x0[node.vq_id.start : node.vq_id.start + 3] = (
                    rotation.T @ np.array([0.0, 0.0, 9.81 * (duration / 2 - t)])
                )
                problem.x0[node.vq_id.start + 4] = -2 * np.pi / duration
        problem.x0[node.q_id] = api["utils"].pin2rep(guess)
        pin.framesForwardKinematics(model, data, guess)
        for name in node.contact_phase_fnames:
            pose = data.oMf[model.getFrameId(name)]
            problem.x0[node.contact_pos_ids[name]] = pose.translation
            problem.x0[node.forces_ids[name]] = pose.rotation.T @ np.array(
                [0.0, 0.0, mass * 9.81 / len(frames)]
            )
    # Upstream lists the total-time Jacobian entries twice. Return each sparse
    # entry once; duplicate coordinates are summed by IPOPT.
    pairs = sorted(set(zip(problem.row_ids, problem.col_ids)))
    problem.row_ids, problem.col_ids = map(list, zip(*pairs))
    return problem, model, q, names, frames, offsets


def bound_violation(values, lower, upper):
    if not np.all(np.isfinite(values)):
        return float("inf")
    lo = np.array([-np.inf if x is None else x for x in lower])
    hi = np.array([np.inf if x is None else x for x in upper])
    return float(max(0.0, np.max(lo - values), np.max(values - hi)))


def apply_warm_start(problem, model, names, args, api):
    """Interpolate an earlier solve on the same phase durations onto a finer mesh."""
    summary = json.loads((args.warm_start / "summary.json").read_text())
    path = args.warm_start / "trajectory.json"
    trajectory = json.loads(path.read_text())
    if (
        summary["urdf_sha256"] != hashlib.sha256(URDF.read_bytes()).hexdigest()
        or summary["contacts"] != args.contacts
        or summary["task"] != args.task
        or summary.get("mass_policy", "declared") != args.mass_policy
        or trajectory["joint_names"] != names
    ):
        raise ValueError("warm-start model, contacts, joint order, or task mismatch")
    for phase in ("push", "flight", "landing"):
        if not np.isclose(
            summary[phase + "_steps"] * summary["dt_s"],
            getattr(args, phase + "_steps") * args.dt_s,
            rtol=0.0,
            atol=1e-10,
        ):
            raise ValueError("warm-start phase durations must match")
    records = trajectory["nodes"]
    times = np.array([record["time_s"] for record in records])
    if len(times) < 2 or not np.all(np.diff(times) > 0):
        raise ValueError("warm-start timestamps must be strictly increasing")
    data = model.createData()
    for k, node in enumerate(problem.nodes):
        t = k * args.dt_s
        index = int(
            np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 2)
        )
        alpha = float(
            np.clip((t - times[index]) / (times[index + 1] - times[index]), 0.0, 1.0)
        )
        left, right = records[index : index + 2]
        q = pin.interpolate(
            model, np.array(left["q_xyzw"]), np.array(right["q_xyzw"]), alpha
        )
        problem.x0[node.q_id] = api["utils"].pin2rep(q)
        for key, field in (("v_body", node.vq_id), ("a_body", node.aq_id)):
            problem.x0[field] = (1 - alpha) * np.array(left[key]) + alpha * np.array(
                right[key]
            )
        pin.framesForwardKinematics(model, data, q)
        for name in node.contact_phase_fnames:
            pose = data.oMf[model.getFrameId(name)]
            problem.x0[node.contact_pos_ids[name]] = pose.translation
            lf = left["contact_forces_world_n"].get(name)
            rf = right["contact_forces_world_n"].get(name)
            if lf is not None or rf is not None:
                # At takeoff the following node has no contact force. Do not
                # discard the last push force even when alpha is exactly zero.
                force = (1 - alpha) * np.array(
                    lf if lf is not None else [0.0, 0.0, 0.0]
                ) + alpha * np.array(rf if rf is not None else [0.0, 0.0, 0.0])
                problem.x0[node.forces_ids[name]] = pose.rotation.T @ force
        if getattr(args, "project_warm_start_dynamics", False):
            torque = np.concatenate(
                (
                    np.zeros(6),
                    (1 - alpha) * np.array(left["tau_nm"])
                    + alpha * np.array(right["tau_nm"]),
                )
            )
            for name, field in node.forces_ids.items():
                jacobian = pin.computeFrameJacobian(
                    model, data, q, model.getFrameId(name), pin.LOCAL
                )
                torque += jacobian[:3].T @ problem.x0[field]
            problem.x0[node.aq_id] = pin.aba(
                model, data, q, problem.x0[node.vq_id], torque
            )
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(problem, model, q0, frames, x, args, api):
    """Recompute dynamics/integration with Pinocchio, outside the NLP callbacks."""
    data = model.createData()
    records = []
    errors = {
        key: 0.0
        for key in (
            "base_force_n",
            "base_torque_nm",
            "torque_limit_nm",
            "position_m",
            "rotation_rad",
            "joint_position_rad",
            "linear_velocity_m_s",
            "angular_velocity_rad_s",
            "joint_velocity_rad_s",
            "linear_impulse_ns",
            "angular_impulse_nms",
            "ground_penetration_m",
            "friction_n",
        )
    }
    for k, node in enumerate(problem.nodes):
        q = api["utils"].rep2pin(x[node.q_id])
        v, a = x[node.vq_id], x[node.aq_id]
        pin.framesForwardKinematics(model, data, q)
        generalized = pin.rnea(model, data, q, v, a).copy()
        forces = {}
        heights = []
        for name in frames:
            fid = model.getFrameId(name)
            pose = data.oMf[fid].copy()
            heights.append(float(pose.translation[2]))
            if name in node.forces_ids:
                force = x[node.forces_ids[name]]
                jac = pin.computeFrameJacobian(model, data, q, fid, pin.LOCAL)
                generalized -= jac[:3].T @ force
                fw = pose.rotation @ force
                forces[name] = fw.tolist()
                errors["friction_n"] = max(
                    errors["friction_n"],
                    -fw[2],
                    abs(fw[0]) - 0.7 * fw[2],
                    abs(fw[1]) - 0.7 * fw[2],
                )
        errors["base_force_n"] = max(
            errors["base_force_n"], float(np.max(np.abs(generalized[:3])))
        )
        errors["base_torque_nm"] = max(
            errors["base_torque_nm"], float(np.max(np.abs(generalized[3:6])))
        )
        errors["torque_limit_nm"] = max(
            errors["torque_limit_nm"],
            float(np.max(np.abs(generalized[6:]) - model.effortLimit[6:])),
        )
        errors["ground_penetration_m"] = max(
            errors["ground_penetration_m"], -min(heights)
        )
        if k + 1 < len(problem.nodes):
            nxt = problem.nodes[k + 1]
            vn = x[nxt.vq_id]
            qn = api["utils"].rep2pin(x[nxt.q_id])
            if args.integration == "world":
                qp, vp = world_step(model, q, v, a, args.dt_s)
                vn = vn.copy()
                rn = pin.Quaternion(qn[3:7]).matrix()
                vn[:3], vn[3:6] = rn @ vn[:3], rn @ vn[3:6]
                dv = vn - vp
            else:
                qp = pin.integrate(model, q, (v + a * args.dt_s) * args.dt_s)
                dv = vn - v - a * args.dt_s
            dq = pin.difference(model, qn, qp)
            linear_key, angular_key = "linear_velocity_m_s", "angular_velocity_rad_s"
            if args.integration == "momentum":
                residual = momentum_residual(
                    model,
                    data,
                    q,
                    v,
                    a,
                    qn,
                    vn,
                    args.dt_s,
                    {name: x[field] for name, field in node.forces_ids.items()},
                )
                dq, dv = residual[: model.nv], residual[model.nv :]
                linear_key, angular_key = "linear_impulse_ns", "angular_impulse_nms"
            for key, value in (
                ("position_m", dq[:3]),
                ("rotation_rad", dq[3:6]),
                ("joint_position_rad", dq[6:]),
                (linear_key, dv[:3]),
                (angular_key, dv[3:6]),
                ("joint_velocity_rad_s", dv[6:]),
            ):
                errors[key] = max(errors[key], float(np.max(np.abs(value))))
        com = pin.centerOfMass(model, data, q).copy()
        momentum = pin.computeCentroidalMomentum(model, data, q, v).vector.copy()
        rotation = pin.Quaternion(q[3:7]).matrix()
        if not all(
            np.all(np.isfinite(values))
            for values in (q, v, a, generalized, heights, com, momentum, rotation)
        ):
            raise ValueError("non-finite value in independent dynamics audit")
        records.append(
            {
                "time_s": k * args.dt_s,
                "q_xyzw": q.tolist(),
                "v_body": v.tolist(),
                "a_body": a.tolist(),
                "tau_nm": generalized[6:].tolist(),
                "contact_forces_world_n": forces,
                "lowest_contact_m": min(heights),
                "com_world_m": com.tolist(),
                "momentum_world": momentum.tolist(),
                "sagittal_angle_rad": float(np.arctan2(rotation[0, 2], rotation[0, 0])),
            }
        )
    rotation = float(np.unwrap([r["sagittal_angle_rad"] for r in records])[-1])
    com_rise = max(r["com_world_m"][2] for r in records) - records[0]["com_world_m"][2]
    flight = [r for r in records if not r["contact_forces_world_n"]]
    clearance = max((r["lowest_contact_m"] for r in flight), default=0.0)
    maneuver = args.task == "stand" or (com_rise >= 0.05 and clearance >= 0.02)
    if args.task == "backflip":
        maneuver = maneuver and abs(rotation + 2 * np.pi) <= 0.2
    # The last flight interval ends at the first landing state. This model has
    # no impact reset there, so that endpoint must also obey conservation.
    flight_indices = [
        k for k, r in enumerate(records) if not r["contact_forces_world_n"]
    ]
    momentum_records = (
        records[flight_indices[0] : flight_indices[-1] + 2] if flight_indices else []
    )
    momentum_errors = flight_momentum_errors(
        momentum_records, float(pin.computeTotalMass(model))
    )
    return (
        errors,
        records,
        dict(
            com_rise_m=com_rise,
            flight_clearance_m=clearance,
            signed_rotation_rad=rotation,
            maneuver_passed=bool(maneuver),
            **momentum_errors,
        ),
    )


def flight_momentum_errors(flight, mass_kg):
    """Check physical conservation, which a small transcription defect cannot prove."""
    linear, angular = 0.0, 0.0
    if flight:
        initial = np.array(flight[0]["momentum_world"])
        for record in flight:
            elapsed = record["time_s"] - flight[0]["time_s"]
            change = np.array(record["momentum_world"]) - initial
            gravity_impulse = np.array([0.0, 0.0, -mass_kg * 9.81 * elapsed])
            linear = max(linear, float(np.linalg.norm(change[:3] - gravity_impulse)))
            angular = max(angular, float(np.linalg.norm(change[3:])))
        duration = flight[-1]["time_s"] - flight[0]["time_s"]
        linear_scale = max(
            1.0, float(np.linalg.norm(initial[:3])), mass_kg * 9.81 * duration
        )
        angular_scale = max(1.0, float(np.linalg.norm(initial[3:])))
    else:
        linear_scale = angular_scale = 1.0
    return {
        "flight_linear_momentum_error_ns": linear,
        "flight_angular_momentum_error_nms": angular,
        "flight_linear_momentum_relative_error": linear / linear_scale,
        "flight_angular_momentum_relative_error": angular / angular_scale,
        # A declared discretization screen, not a hardware-validation gate.
        "flight_momentum_screen_passed": max(
            linear / linear_scale, angular / angular_scale
        )
        <= 0.02,
    }


def main():
    args = parse_args()
    api = load_upstream(args.upstream)
    problem, model, q0, names, frames, offsets = build_problem(args, api)
    warm_start_hash = (
        apply_warm_start(problem, model, names, args, api) if args.warm_start else None
    )
    args.output.mkdir(parents=True, exist_ok=True)
    nlp = cyipopt.Problem(
        n=problem.vars_dim,
        m=problem.cons_dim,
        problem_obj=problem,
        lb=problem.lb,
        ub=problem.ub,
        cl=problem.clb,
        cu=problem.cub,
    )
    for key, value in {
        "max_iter": args.max_iterations,
        "max_wall_time": args.max_wall_time_s,
        "tol": args.tolerance,
        "constr_viol_tol": args.tolerance,
        "hessian_approximation": "limited-memory",
        "linear_solver": "mumps",
        "print_level": 5,
        "bound_relax_factor": 0.0,
    }.items():
        nlp.add_option(key, value)
    if args.check_jacobian:
        nlp.add_option("derivative_test", "first-order")
    start = time.monotonic()
    x, info = nlp.solve(problem.x0)
    elapsed = time.monotonic() - start
    constraints = problem.constraints(x)
    finite = bool(np.all(np.isfinite(x)) and np.all(np.isfinite(constraints)))
    cv = bound_violation(constraints, problem.clb, problem.cub)
    bv = bound_violation(x, problem.lb, problem.ub)
    errors, records, task = ({}, [], {"maneuver_passed": False})
    audit_error = None
    if finite:
        try:
            errors, records, task = audit(problem, model, q0, frames, x, args, api)
        except (ValueError, RuntimeError, FloatingPointError) as error:
            audit_error = str(error)
            finite = False
    feasible = finite and max(cv, bv, *errors.values()) <= args.tolerance
    transcription_passed = (
        feasible and task["maneuver_passed"] and int(info["status"]) in (0, 1)
    )
    passed = transcription_passed and task.get("flight_momentum_screen_passed", False)
    summary = dict(
        schema="rne.g1.trajopt_reference.v1",
        task=args.task,
        upstream_revision=UPSTREAM_REVISION,
        urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
        warm_start_trajectory_sha256=warm_start_hash,
        versions={
            "pinocchio": pin.__version__,
            "numpy": np.__version__,
            "cyipopt": cyipopt.__version__,
        },
        actuated_dof=23,
        nv=model.nv,
        joint_names=names,
        mass_kg=float(pin.computeTotalMass(model)),
        mass_policy=args.mass_policy,
        integration=args.integration,
        feasibility_only=args.feasibility_only,
        project_warm_start_dynamics=args.project_warm_start_dynamics,
        world_up="Z",
        rne_world_from_reference=[[1, 0, 0], [0, 0, 1], [0, -1, 0]],
        velocity_convention="local body twist; joint rates follow joint_names",
        contacts=args.contacts,
        contact_offsets_local_m=offsets,
        initial_q_xyzw=q0.tolist(),
        initial_pose_changes="flat foot roll angles; root height grounded from FK",
        torque_limits_nm=model.effortLimit[6:].tolist(),
        joint_velocity_limits_rad_s=model.velocityLimit[6:].tolist(),
        dt_s=args.dt_s,
        push_steps=args.push_steps,
        flight_steps=args.flight_steps,
        landing_steps=args.landing_steps,
        iterations=int(problem.iter_count),
        max_iterations=args.max_iterations,
        max_wall_time_s=args.max_wall_time_s,
        tolerance=args.tolerance,
        status=int(info["status"]),
        status_message=info["status_msg"].decode(),
        elapsed_s=elapsed,
        finite=finite,
        audit_error=audit_error,
        constraint_violation=cv if finite else None,
        bound_violation=bv if finite else None,
        independent_errors=errors,
        **task,
        transcription_feasible=bool(feasible),
        transcription_passed=bool(transcription_passed),
        passed=bool(passed),
        plant_validated=False,
        limitations=[
            "finite-step smooth landing; no impact reset",
            "contact constraints at nodes only",
            "no self-collision or non-foot collision constraints",
            "constant torque limits; no torque-speed envelope",
            "no independent physics-backend replay or landing hold",
        ],
    )
    trajectory_text = (
        json.dumps(
            {"joint_names": names, "world_up": "Z", "nodes": records}, allow_nan=False
        )
        + "\n"
    )
    summary["trajectory_sha256"] = hashlib.sha256(trajectory_text.encode()).hexdigest()
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )
    (args.output / "trajectory.json").write_text(trajectory_text)
    print(json.dumps(summary, indent=2, allow_nan=False))
    return 0 if passed else 2


if __name__ == "__main__":
    sys.exit(main())
