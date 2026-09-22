#!/usr/bin/env python3
"""Derivative-free optimization of a bounded G1 maneuver in a contact plant."""

import argparse
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1_backflip_plant as plant
import mujoco
import numpy as np
from scipy.optimize import differential_evolution


class CommandClock:
    """Sample and hold joint targets and gains, with whole-tick transport delay."""

    def __init__(self, dt_s, period_s, delay_s, initial):
        values = np.asarray([dt_s, period_s, delay_s])
        if not np.all(np.isfinite(values)) or dt_s <= 0 or period_s <= 0 or delay_s < 0:
            raise ValueError("invalid command timing")
        ratio = period_s / dt_s
        ticks = delay_s / period_s
        if ratio < 1 or not np.isclose(ratio, round(ratio), atol=1e-9, rtol=0):
            raise ValueError(
                "command period must be an integer number of physics steps"
            )
        if not np.isclose(ticks, round(ticks), atol=1e-9, rtol=0):
            raise ValueError("command delay must be an integer number of command ticks")
        self.steps = round(ratio)
        self.queue = deque([initial] * round(ticks))
        self.current = initial

    def update_due(self, step):
        """Whether an outer-loop command is sampled at this physics step."""
        return step % self.steps == 0

    def submit(self, command):
        """Queue one sampled (target, kp, kd) command and return the arriving one."""
        self.queue.append(command)
        self.current = self.queue.popleft()
        return self.current


class Campaign:
    """Bounded, seeded search over joint-target trajectories in a contact plant."""

    def __init__(
        self,
        output,
        dt_s=0.001,
        stage="flip",
        target_momentum=-20.0,
        mass_policy="rne",
        joint_limit_time_constant_s=0.004,
        joint_limit_margin_rad=0.0,
        profile=None,
    ):
        if (
            shutil.disk_usage(output if output.exists() else plant.ROOT).free
            < 30 * 1024**3
        ):
            raise RuntimeError("disk reserve below 30 GiB")
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.summary = json.loads((plant.MODEL_MANIFEST).read_text())
        self.summary["mass_policy"] = mass_policy
        self.summary["joint_limit_time_constant_s"] = joint_limit_time_constant_s
        self.summary["joint_limit_margin_rad"] = joint_limit_margin_rad
        self.profile = dict(profile or {})
        allowed = {
            "name",
            "mass_policy",
            "self_collision",
            "knee_limit_nm",
            "control_period_s",
            "command_delay_s",
        }
        if set(self.profile) - allowed:
            raise ValueError("unknown screening profile fields")
        for key in ("mass_policy", "self_collision"):
            if key in self.profile:
                self.summary[key] = self.profile[key]
        if "knee_limit_nm" in self.profile:
            cap = self.profile["knee_limit_nm"]
            if not np.isfinite(cap) or cap <= 0 or cap > 139:
                raise ValueError("invalid screening knee torque cap")
            self.summary["torque_limits_nm"] = [
                min(limit, cap) if "knee" in name else limit
                for name, limit in zip(
                    self.summary["joint_names"], self.summary["torque_limits_nm"]
                )
            ]
        self.control_period_s = self.profile.get("control_period_s", dt_s)
        self.command_delay_s = self.profile.get("command_delay_s", 0.0)
        CommandClock(dt_s, self.control_period_s, self.command_delay_s, None)
        self.model = plant.build_model(self.summary, dt_s, 300, 10)
        self.data = mujoco.MjData(self.model)
        self.dt_s = dt_s
        self.names = self.summary["joint_names"]
        self.qids = np.array(
            [self.model.jnt_qposadr[self.model.joint(name).id] for name in self.names]
        )
        self.stand = np.array(self.summary["initial_q_xyzw"])[7:]
        self.ground = self.model.geom("ground").id
        self.feet = {
            self.model.body(f"{side}_ankle_roll_link").id for side in ("left", "right")
        }
        self.best = np.inf
        self.evaluations = 0
        self.stage = stage
        self.target_momentum = target_momentum
        self.balance = (1.0, 0.0, 0.6)
        self.landing_gains = (1000.0, 20.0)
        self.recovery_s = 0.5
        self.early_balance = (1.0, 0.05)
        self.vids = np.array(
            [self.model.jnt_dofadr[self.model.joint(name).id] for name in self.names]
        )
        robot = ET.parse(plant.URDF).getroot()
        self.speed_limits = np.array(
            [
                float(robot.find(f"joint[@name='{name}']/limit").get("velocity"))
                for name in self.names
            ]
        )
        self.joint_ids = np.array([self.model.joint(name).id for name in self.names])
        self.torque_limits = np.array(self.summary["torque_limits_nm"])

    def pose(self, hip, knee, ankle, shoulder=0.0):
        """Return a symmetric joint pose in radians, retaining other stand joints."""
        q = self.stand.copy()
        for index, name in enumerate(self.names):
            for part, value in (
                ("hip_pitch", hip),
                ("knee", knee),
                ("ankle_pitch", ankle),
                ("shoulder_pitch", shoulder),
            ):
                if part in name:
                    q[index] = value
        return q

    def rollout(self, parameters, record=False):
        """Simulate five seconds, applying only bounded joint motor commands."""
        if len(parameters) not in (13, 14, 15, 16) or not np.all(
            np.isfinite(parameters)
        ):
            raise ValueError("expected 13 to 16 finite maneuver parameters")
        if len(parameters) >= 15 and parameters[14] < 0:
            raise ValueError("hip extension delay must be nonnegative")
        (
            knee,
            lean,
            push_s,
            push_hip,
            push_ankle,
            tuck_s,
            tuck_hip,
            tuck_knee,
            landing_knee,
        ) = parameters[:9]
        push_shoulder = parameters[9] if len(parameters) > 9 else -2.2
        landing_bias = parameters[10] if len(parameters) > 10 else 0.0
        opening_angle = parameters[11] if len(parameters) > 11 else 5.0
        crouch_bias = parameters[12] if len(parameters) > 12 else 0.0
        crouch = self.pose(
            -knee / 2 + crouch_bias, knee, np.clip(-knee / 2 - lean, -0.85, 0.5), -0.4
        )
        launch = self.pose(push_hip, 0.02, push_ankle, push_shoulder)
        tuck = self.pose(tuck_hip, tuck_knee, -0.4, 0.0)
        land_hip = -landing_knee / 2 + landing_bias
        land = self.pose(
            land_hip,
            landing_knee,
            np.clip(-land_hip - landing_knee, -0.85, 0.5),
            parameters[15] if len(parameters) == 16 else 0.0,
        )
        if len(parameters) >= 14:
            for pose in (crouch, launch, tuck, land):
                for index, name in enumerate(self.names):
                    if "shoulder_roll" in name:
                        pose[index] = parameters[13] * (
                            1 if name.startswith("left") else -1
                        )
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        m.actuator_forcerange[:, 0] = -self.torque_limits
        m.actuator_forcerange[:, 1] = self.torque_limits
        m.actuator_gainprm[:, 0] = 300
        m.actuator_biasprm[:, 1] = -300
        m.actuator_biasprm[:, 2] = -10
        initial = np.array(self.summary["initial_q_xyzw"])
        d.qpos[:3] = initial[:3]
        d.qpos[3:7] = initial[[6, 3, 4, 5]]
        d.qpos[self.qids] = self.stand
        mujoco.mj_forward(m, d)
        angle = previous = 0.0
        maximum_backward_rotation = 0.0
        max_height = initial[2]
        takeoff = None
        takeoff_momentum = None
        takeoff_velocity = None
        touchdown = None
        air_s = longest_air_s = 0.0
        nonfoot = False
        nonfoot_bodies = set()
        peak_torque = 0.0
        peak_speed_ratio = 0.0
        peak_speed_joint = None
        peak_speed_time_s = None
        peak_position_excess = 0.0
        tail_upright = 1.0
        tail_speed = 0.0
        tail_height = np.inf
        tail_contact = True
        numerical_failure = False
        self_contact_pairs = set()
        clock = CommandClock(
            self.dt_s,
            self.control_period_s,
            self.command_delay_s,
            (self.stand.copy(), 300.0, 10.0),
        )
        command_kp, command_kd = 300.0, 10.0
        history = []
        last_stand_error = 0.0
        for step in range(round(5.0 / self.dt_s)):
            t = step * self.dt_s
            if clock.update_due(step):
                if t < 0.5:
                    alpha = min(1.0, t / 0.4)
                    alpha = alpha * alpha * (3 - 2 * alpha)
                    target = (1 - alpha) * self.stand + alpha * crouch
                elif takeoff is None:
                    alpha = min(1.0, (t - 0.5) / push_s)
                    target = (1 - alpha) * crouch + alpha * launch
                    if len(parameters) >= 15:
                        hip_alpha = np.clip((t - 0.5 - parameters[14]) / push_s, 0, 1)
                        for index, name in enumerate(self.names):
                            if "hip_pitch" in name:
                                target[index] = (1 - hip_alpha) * crouch[
                                    index
                                ] + hip_alpha * launch[index]
                elif touchdown is None:
                    elapsed = t - takeoff
                    if elapsed < tuck_s and angle > -opening_angle:
                        alpha = min(1.0, elapsed / 0.08)
                        target = (1 - alpha) * launch + alpha * tuck
                    else:
                        alpha = (
                            min(1.0, max(0.0, elapsed - tuck_s) / 0.1)
                            if angle > -opening_angle
                            else 1.0
                        )
                        target = (1 - alpha) * tuck + alpha * land
                        command_kp, command_kd = 1000.0, 20.0

                else:
                    command_kp, command_kd = self.landing_gains
                    alpha = min(1.0, (t - touchdown) / self.recovery_s)
                    target = (1 - alpha) * land + alpha * self.stand
                    mujoco.mj_subtreeVel(m, d)
                    root = m.body("pelvis").id
                    center = np.mean([d.xpos[body] for body in self.feet], axis=0)
                    gain, com_gain, velocity_gain = self.balance
                    blend = np.clip((t - touchdown - 0.5) / 0.3, 0, 1)
                    feedback = (
                        gain * previous
                        + com_gain * (d.subtree_com[root, 0] - center[0] - 0.015)
                        + velocity_gain * d.subtree_linvel[root, 0]
                    )
                    correction = np.clip(
                        (1 - blend)
                        * (
                            self.early_balance[0] * previous
                            + self.early_balance[1] * d.qvel[4]
                        )
                        + blend * feedback,
                        -0.6,
                        0.6,
                    )
                    for index, name in enumerate(self.names):
                        if "ankle_pitch" in name:
                            target[index] = np.clip(
                                target[index] + correction, -0.85, 0.5
                            )
                clock.submit((target.copy(), command_kp, command_kd))
            target, kp, kd = clock.current
            d.ctrl[:] = target
            m.actuator_gainprm[:, 0] = kp
            m.actuator_biasprm[:, 1] = -kp
            m.actuator_biasprm[:, 2] = -kd
            velocity_ratio = d.qvel[self.vids] / self.speed_limits
            # Full effort below 90% rated speed; motoring effort vanishes at the limit.
            m.actuator_forcerange[:, 0] = -self.torque_limits * np.clip(
                (1 + velocity_ratio) * 10, 0, 1
            ) + self.torque_limits * np.clip((-velocity_ratio - 1) * 20, 0, 1)
            m.actuator_forcerange[:, 1] = self.torque_limits * np.clip(
                (1 - velocity_ratio) * 10, 0, 1
            ) - self.torque_limits * np.clip((velocity_ratio - 1) * 20, 0, 1)
            mujoco.mj_step(m, d)
            numerical_failure = bool(
                any(warning.number for warning in d.warning)
                or not np.all(np.isfinite(d.qpos))
                or not np.all(np.isfinite(d.qvel))
                or abs(d.time - (step + 1) * self.dt_s) > 1e-7
            )
            if numerical_failure:
                break
            w, x, y, z = d.qpos[3:7]
            pitch = np.arctan2(2 * (x * z + w * y), 1 - 2 * (y * y + z * z))
            angle += (pitch - previous + np.pi) % (2 * np.pi) - np.pi
            previous = pitch
            maximum_backward_rotation = max(maximum_backward_rotation, -angle)
            upright = 1 - 2 * (x * x + y * y)
            contact = False
            for collision in d.contact:
                if self.ground in (collision.geom1, collision.geom2):
                    other = (
                        collision.geom2
                        if collision.geom1 == self.ground
                        else collision.geom1
                    )
                    if m.geom_bodyid[other] in self.feet:
                        contact = True
                    else:
                        nonfoot = True
                        nonfoot_bodies.add(m.body(m.geom_bodyid[other]).name)
                elif collision.dist < 0:
                    self_contact_pairs.add(
                        tuple(
                            sorted(
                                (
                                    m.body(m.geom_bodyid[collision.geom1]).name,
                                    m.body(m.geom_bodyid[collision.geom2]).name,
                                )
                            )
                        )
                    )
            air_s = 0.0 if contact else air_s + self.dt_s
            longest_air_s = max(longest_air_s, air_s)
            if (
                clock.update_due(step)
                and takeoff is None
                and t > 0.5
                and air_s > 0.012
                and d.qvel[2] > 0.3
            ):
                takeoff = t - air_s
                mujoco.mj_subtreeVel(m, d)
                root = m.body("pelvis").id
                takeoff_momentum = d.subtree_angmom[root].copy()
                takeoff_velocity = d.subtree_linvel[root].copy()
            if (
                clock.update_due(step)
                and takeoff is not None
                and touchdown is None
                and t - takeoff > 0.1
                and contact
            ):
                touchdown = t
            max_height = max(max_height, d.qpos[2])
            ranges = m.jnt_range[self.joint_ids]
            peak_position_excess = max(
                peak_position_excess,
                float(np.max(ranges[:, 0] - d.qpos[self.qids])),
                float(np.max(d.qpos[self.qids] - ranges[:, 1])),
            )
            if t >= 4.0:
                tail_upright = min(tail_upright, upright)
                tail_speed = max(tail_speed, float(np.linalg.norm(d.qvel[:6])))
                tail_height = min(tail_height, float(d.qpos[2]))
                tail_contact = tail_contact and contact
            speed_ratios = np.abs(d.qvel[self.vids]) / self.speed_limits
            fastest = int(np.argmax(speed_ratios))
            if speed_ratios[fastest] > peak_speed_ratio:
                peak_speed_ratio = float(speed_ratios[fastest])
                peak_speed_joint = self.names[fastest]
                peak_speed_time_s = float(d.time)
            peak_torque = max(peak_torque, float(np.max(np.abs(d.actuator_force))))
            last_stand_error = (
                (1 - upright) ** 2
                + 3 * (d.qpos[2] - initial[2]) ** 2
                + 0.02 * np.dot(d.qvel[:6], d.qvel[:6])
            )
            if record and step % max(1, round(0.01 / self.dt_s)) == 0:
                history.append(
                    {
                        "time_s": float(d.time),
                        "qpos": d.qpos.tolist(),
                        "qvel": d.qvel.tolist(),
                        "pitch_rad": float(angle),
                        "upright": float(upright),
                        "foot_contact": contact,
                    }
                )
            if nonfoot or self_contact_pairs or not np.all(np.isfinite(d.qpos)):
                break
            if self.stage == "launch" and takeoff is not None:
                break
        rotation_error = abs(angle + 2 * np.pi)
        loss = (
            3 * rotation_error**2
            + 20 * last_stand_error
            + 30 * nonfoot
            + 30 * (takeoff is None)
            + 15 * max(0, 0.2 - (max_height - initial[2]))
        )
        if self.stage == "launch":
            loss = (
                400.0
                if takeoff is None
                else 0.5 * (takeoff_momentum[1] - self.target_momentum) ** 2
                + 8 * (takeoff_velocity[2] - 3.2) ** 2
                + 3 * takeoff_velocity[0] ** 2
                + 3 * (takeoff_momentum[0] ** 2 + takeoff_momentum[2] ** 2)
            )
            loss += 200 * nonfoot
        loss += 200 * bool(self_contact_pairs)
        loss += 1e6 * numerical_failure
        loss += (
            30 * max(0, peak_speed_ratio - 1.05) ** 2
            + 100 * max(0, peak_position_excess - 0.02) ** 2
        )
        standing = (
            d.time >= 4.99
            and tail_upright > 0.95
            and tail_speed < 0.3
            and tail_height > 0.65
            and tail_contact
        )
        passed = (
            not numerical_failure
            and not self_contact_pairs
            and standing
            and peak_position_excess < 0.02
            and peak_speed_ratio < 1.05
            and not nonfoot
            and rotation_error < 0.25
            and last_stand_error < 0.03
            and longest_air_s > 0.25
            and touchdown is not None
        )
        result = {
            "loss": float(loss),
            "passed": bool(passed),
            "parameters": list(map(float, parameters)),
            "dt_s": self.dt_s,
            "signed_rotation_rad": float(angle),
            "maximum_backward_rotation_rad": float(maximum_backward_rotation),
            "root_rise_m": float(max_height - initial[2]),
            "takeoff_s": takeoff,
            "touchdown_s": touchdown,
            "longest_flight_s": longest_air_s,
            "nonfoot_ground_contact": nonfoot,
            "final_root_height_m": float(d.qpos[2]),
            "stand_error": float(last_stand_error),
            "peak_torque_nm": peak_torque,
        }
        result.update(
            joint_limit_time_constant_s=float(m.jnt_solref[self.joint_ids[0], 0]),
            joint_limit_margin_rad=float(m.jnt_margin[self.joint_ids[0]]),
            backend="mujoco",
            backend_version=mujoco.__version__,
            urdf_sha256=hashlib.sha256(plant.URDF.read_bytes()).hexdigest(),
            joint_names=self.names,
            balance_gains=self.balance,
            early_balance_gains=self.early_balance,
            landing_gains=self.landing_gains,
            recovery_s=self.recovery_s,
            numerical_failure=numerical_failure,
        )
        result.update(
            mass_policy=self.summary["mass_policy"],
            mass_kg=float(sum(m.body_mass)),
            stage=self.stage,
            takeoff_angular_momentum_nms=None
            if takeoff_momentum is None
            else takeoff_momentum.tolist(),
            takeoff_com_velocity_m_s=None
            if takeoff_velocity is None
            else takeoff_velocity.tolist(),
        )
        result.update(
            standing=bool(standing),
            peak_joint_position_excess_rad=peak_position_excess,
            final_second_min_upright=float(tail_upright),
            final_second_max_base_speed=float(tail_speed),
            simulation_time_s=float(d.time),
        )
        result["screening_profile"] = self.profile
        result["self_contact_pairs"] = sorted(self_contact_pairs)
        result["control_period_s"] = self.control_period_s
        result["command_delay_s"] = self.command_delay_s
        result["torque_limits_nm"] = self.torque_limits.tolist()
        result["self_collision_enabled"] = self.summary.get("self_collision", False)
        result["peak_joint_speed_ratio"] = peak_speed_ratio
        result["peak_joint_speed_joint"] = peak_speed_joint
        result["peak_joint_speed_time_s"] = peak_speed_time_s
        result["nonfoot_contact_bodies"] = sorted(nonfoot_bodies)
        return result, history

    def __call__(self, parameters):
        if shutil.disk_usage(self.output).free < 30 * 1024**3:
            raise RuntimeError("disk reserve below 30 GiB")
        result, _ = self.rollout(parameters)
        self.evaluations += 1
        if result["loss"] < self.best:
            self.best = result["loss"]
            result["evaluations"] = self.evaluations
            (self.output / "best.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result), flush=True)
        return result["loss"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--generations", type=int, default=15)
    parser.add_argument("--dt-s", type=float)
    parser.add_argument("--joint-limit-time-constant-s", type=float)
    parser.add_argument("--balance-kp", type=float)
    parser.add_argument("--balance-kd", type=float)
    parser.add_argument("--parameters", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--stage", choices=("launch", "flight", "flip"))
    parser.add_argument("--mass-policy", choices=("rne", "declared"))
    parser.add_argument("--target-momentum", type=float, default=-20.0)
    args = parser.parse_args()
    if args.generations < 0:
        parser.error("generations must be nonnegative")
    seed = json.loads(args.parameters.read_text()) if args.parameters else {}
    args.stage = args.stage or seed.get("stage", "flip")
    campaign = Campaign(
        args.output,
        args.dt_s if args.dt_s is not None else seed.get("dt_s", 0.001),
        args.stage,
        args.target_momentum,
        args.mass_policy or seed.get("mass_policy", "rne"),
        args.joint_limit_time_constant_s
        if args.joint_limit_time_constant_s is not None
        else seed.get("joint_limit_time_constant_s", 0.004),
        seed.get("joint_limit_margin_rad", 0.0),
        json.loads(args.profile.read_text())
        if args.profile
        else seed.get("screening_profile"),
    )
    for field, attribute in (
        ("early_balance_gains", "early_balance"),
        ("balance_gains", "balance"),
        ("landing_gains", "landing_gains"),
        ("recovery_s", "recovery_s"),
    ):
        if field in seed:
            setattr(campaign, attribute, seed[field])
    campaign.early_balance = (
        campaign.early_balance[0] if args.balance_kp is None else args.balance_kp,
        campaign.early_balance[1] if args.balance_kd is None else args.balance_kd,
    )
    if (
        seed.get("urdf_sha256", campaign.summary["urdf_sha256"])
        != campaign.summary["urdf_sha256"]
    ):
        raise ValueError("seed URDF checksum differs from the model")
    initial = [1.5, -0.1, 0.12, 0.1, 0.45, 0.3, -1.4, 2.5, 0.7, -2.2, 0.0, 5.0, 0.0]
    if args.parameters:
        initial = seed["parameters"]
        if len(initial) == 9:
            initial.append(-2.2)
        if len(initial) == 10:
            initial.extend([0.0, 5.0])
        if len(initial) == 12:
            initial.append(0.0)
    if args.generations:
        if len(initial) == 13:
            initial.append(0.2)
        bounds = [
            (0.8, 2.8),
            (-0.4, 0.2),
            (0.01, 0.35),
            (-0.4, 2.7),
            (-0.1, 0.52),
            (0.1, 0.65),
            (-1.7, 2.8),
            (1.0, 2.8),
            (0.1, 1.6),
            (-2.5, 2.0),
            (-1.0, 0.8),
            (3.3, 7.0),
            (-0.8, 0.8),
            (0.2, 1.2),
        ]
        if len(initial) >= 15:
            bounds.append((0.0, 0.16))
        if len(initial) == 16:
            bounds.append((-1.5, 2.5))
        if args.stage == "launch":
            for i in (5, 6, 7, 8, 10, 11):
                bounds[i] = (initial[i], initial[i])
        if args.stage == "flight":
            for i in (0, 1, 2, 3, 4, 9, 12) + ((14,) if len(initial) >= 15 else ()):
                bounds[i] = (initial[i], initial[i])
        solution = differential_evolution(
            campaign,
            bounds,
            maxiter=args.generations,
            popsize=6,
            rng=np.random.default_rng(20260921),
            x0=initial,
            polish=False,
        )
        initial = solution.x
    result, history = campaign.rollout(initial, record=True)
    recording = json.dumps(history, allow_nan=False) + "\n"
    result["rollout_sha256"] = hashlib.sha256(recording.encode()).hexdigest()
    (args.output / "rollout.json").write_text(recording)
    (args.output / "summary.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
