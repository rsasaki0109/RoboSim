"""CEM training smoke for the mobile navigate -> grasp -> place clutter task.

Re-implements ``IkMobileClutterPickPlacePolicy``'s observation-gated phase machine
(settle, drive-and-poke-grasp, straight retreat off the tabletop, carry-the-object
drive, release) in Python on the pinned `mobile_clutter_pick_place_center` episode
(`mm_mobile_clutter` scene, `clutter_cube_a`). Unlike the fixed-base arm gains tuned
by ``train_clutter.py``, the mobile task's degrees of freedom that actually separate
success from failure are the diff-drive base rates: how fast it pokes into the pick
object, how hard it backs off the table to clear the grasp weld, and how fast it
carries the object to the place target. CEM optimizes those (plus the pick-phase
gripper rate) against a weak baseline that holds the gripper open -- structurally
unable to grasp regardless of drive speed, mirroring ``train_clutter.py``'s weak
baseline -- so CEM must discover both the closing sign and workable drive speeds.

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_mobile_clutter.py --smoke
"""

import math
import random
import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

TASK = "mobile_clutter_place_center"

# Diff-drive geometry, matching `MM_MOBILE_TRACK_WIDTH_M` / `MM_MOBILE_WHEEL_RADIUS_M`
# in crates/rne_ai/src/env/mobile_manipulator/drive.rs.
TRACK_WIDTH_M = 0.45
WHEEL_RADIUS_M = 0.1
WHEEL_LIMIT_RAD_S = 3.0

# Phase step budget, matching `IkMobileClutterPickPlacePolicy`'s constants in
# crates/rne_ai/src/policy.rs (`policy.total_steps() == 1490`, under the episode's
# 1600-step budget).
SETTLE_STEPS = 80
PICK_DRIVE_STEPS = 480
RETREAT_STEPS = 300
CARRY_DRIVE_STEPS = 480
RELEASE_STEPS = 150
EPISODE_STEPS = (
    SETTLE_STEPS + PICK_DRIVE_STEPS + RETREAT_STEPS + CARRY_DRIVE_STEPS + RELEASE_STEPS
)

# Observation-gate thresholds, matching the Rust policy's constants.
RETREAT_DISTANCE_M = 1.9
RELEASE_GATE_M = 0.10
CARRY_OBJECT_LEAD_M = 0.95
CARRY_OBJECT_GAIN = 0.6
# Mobile clutter ground place target (MM_MOBILE_CLUTTER_PLACE_X_M / _Z_M in
# crates/rne_ai/src/mm_minimal_kinematics.rs).
PLACE_X_M = 1.23
PLACE_Z_M = -0.53
GRIPPER_OPEN_RAD_S = 3.0

PARAM_DIM = 4  # gripper_close, pick_drive_speed, retreat_wheel, carry_drive_speed
# Scripted reference values, matching IkMobileClutterPickPlacePolicy's Rust constants
# (MOBILE_CLUTTER_*_SPEED_M_S / MOBILE_CLUTTER_RETREAT_WHEEL_RAD_S / gripper rate).
SCRIPTED_PARAMS = [-2.5, 0.15, -1.5, 0.25]
# Weak baseline: the gripper opens during the pick drive instead of closing, so the
# contact weld can never trigger regardless of drive speed -- CEM must discover the
# closing sign as well as workable drive speeds (mirrors train_clutter.py's
# gripper-held-open baseline).
WEAK_BASELINE = [0.5, 0.05, -0.2, 0.05]


def wrap_heading_rad(angle):
    wrapped = math.fmod(angle, math.tau)
    if wrapped < 0.0:
        wrapped += math.tau
    if wrapped > math.pi:
        wrapped -= math.tau
    return wrapped


def wheel_velocities(forward_m_s, yaw_rate_rad_s):
    v_left_m_s = forward_m_s - yaw_rate_rad_s * TRACK_WIDTH_M * 0.5
    v_right_m_s = forward_m_s + yaw_rate_rad_s * TRACK_WIDTH_M * 0.5
    left = v_left_m_s / WHEEL_RADIUS_M
    right = v_right_m_s / WHEEL_RADIUS_M
    return (
        max(-WHEEL_LIMIT_RAD_S, min(WHEEL_LIMIT_RAD_S, left)),
        max(-WHEEL_LIMIT_RAD_S, min(WHEEL_LIMIT_RAD_S, right)),
    )


def drive_toward(obs, target_x_m, target_z_m, max_forward_m_s):
    """Heading-based diff-drive step toward a world XZ point (mirrors
    `mobile_drive_toward_action` in crates/rne_ai/src/policy.rs)."""
    dx_world = target_x_m - obs.base_x
    dz_world = target_z_m - obs.base_z
    distance_m = math.hypot(dx_world, dz_world)
    heading_to_target = math.atan2(dz_world, dx_world)
    heading_error = wrap_heading_rad(heading_to_target + obs.base_yaw)
    if abs(heading_error) > 0.12:
        forward_m_s = 0.0
    else:
        forward_m_s = max(0.0, min(max_forward_m_s, 0.65 * distance_m))
    yaw_rate_rad_s = max(-0.7, min(0.7, -2.0 * heading_error))
    return wheel_velocities(forward_m_s, yaw_rate_rad_s)


def carry_object_toward(obs, max_forward_m_s):
    """Diff-drive step that moves the CARRIED OBJECT toward the place target
    (mirrors `mobile_carry_object_toward_action` in crates/rne_ai/src/policy.rs)."""
    error_x, error_z = obs.target_dx, obs.target_dz
    yaw = obs.base_yaw
    forward = (math.cos(yaw), -math.sin(yaw))
    forward_yaw_derivative = (-math.sin(yaw), -math.cos(yaw))
    along_m = error_x * forward[0] + error_z * forward[1]
    lateral_m = error_x * forward_yaw_derivative[0] + error_z * forward_yaw_derivative[1]
    forward_m_s = max(
        -max_forward_m_s, min(max_forward_m_s, CARRY_OBJECT_GAIN * along_m)
    )
    yaw_rate_rad_s = max(
        -0.7, min(0.7, CARRY_OBJECT_GAIN * lateral_m / CARRY_OBJECT_LEAD_M)
    )
    return wheel_velocities(forward_m_s, yaw_rate_rad_s)


def mobile_place_base_distance(obs):
    dx = PLACE_X_M - obs.base_x
    dz = PLACE_Z_M - obs.base_z
    return math.hypot(dx, dz)


class MobileClutterPolicy:
    """Python re-implementation of `IkMobileClutterPickPlacePolicy`'s observation-
    gated phase machine, parameterized so CEM can tune the pick/retreat/carry drive
    rates. The phase boundaries are observation-gated exactly like the Rust policy:
    grasp exits the pick drive early, base-to-place distance exits the retreat early,
    and the carried-object-to-place distance exits the carry drive early."""

    def __init__(self, params):
        gripper_close, pick_speed, retreat_wheel, carry_speed = params
        self.gripper_close_rad_s = gripper_close
        self.pick_drive_speed_m_s = max(0.0, pick_speed)
        self.retreat_wheel_rad_s = retreat_wheel
        self.carry_drive_speed_m_s = max(0.0, carry_speed)
        self.step_idx = 0

    def act(self, obs, is_grasping):
        settle_end = SETTLE_STEPS
        pick_drive_end = settle_end + PICK_DRIVE_STEPS
        retreat_end = pick_drive_end + RETREAT_STEPS
        carry_drive_end = retreat_end + CARRY_DRIVE_STEPS
        release_end = carry_drive_end + RELEASE_STEPS

        s = self.step_idx
        if settle_end <= s < pick_drive_end and is_grasping:
            s = pick_drive_end
        if (
            pick_drive_end <= s < retreat_end
            and mobile_place_base_distance(obs) > RETREAT_DISTANCE_M
        ):
            s = retreat_end
        if (
            retreat_end <= s < carry_drive_end
            and is_grasping
            and math.hypot(obs.target_dx, obs.target_dz) < RELEASE_GATE_M
        ):
            s = carry_drive_end
        self.step_idx = s + 1

        # [left_wheel, right_wheel, shoulder, elbow, gripper]
        action = [0.0, 0.0, 0.0, 0.0, 0.0]
        if s < settle_end:
            pass
        elif s < pick_drive_end:
            object_x_m = obs.ee_x + obs.target_dx
            object_z_m = obs.ee_z + obs.target_dz
            action[0], action[1] = drive_toward(
                obs, object_x_m, object_z_m, self.pick_drive_speed_m_s
            )
            action[4] = self.gripper_close_rad_s
        elif s < retreat_end:
            action[0] = self.retreat_wheel_rad_s
            action[1] = self.retreat_wheel_rad_s
        elif s < carry_drive_end:
            if is_grasping:
                action[0], action[1] = carry_object_toward(
                    obs, self.carry_drive_speed_m_s
                )
            else:
                action[0], action[1] = drive_toward(
                    obs, PLACE_X_M, PLACE_Z_M, self.carry_drive_speed_m_s
                )
        elif s < release_end:
            action[4] = GRIPPER_OPEN_RAD_S
        return action


def rollout_metrics(params):
    episode = rne_py.MobileManipulatorEpisode(TASK)
    step = episode.reset()
    policy = MobileClutterPolicy(params)
    grasped = False
    placed = False
    for _ in range(EPISODE_STEPS):
        action = policy.act(step.observation, episode.is_grasping)
        step = episode.step(*action)
        if episode.is_grasping:
            grasped = True
        if step.terminated:
            placed = True
            break
    return episode.total_reward, grasped, placed


def rollout(params):
    reward, _, _ = rollout_metrics(params)
    return reward


def cem_smoke():
    population = 10
    elite = 3
    iterations = 8
    mean = [0.0, 0.12, -0.8, 0.15]
    std = [1.5, 0.08, 0.6, 0.1]
    history = []
    best_reward = float("-inf")
    best_params = mean
    best_grasped = False
    best_placed = False

    for _ in range(iterations):
        candidates = []
        for _ in range(population):
            params = [random.gauss(mean[i], std[i]) for i in range(PARAM_DIM)]
            reward, grasped, placed = rollout_metrics(params)
            candidates.append((reward, grasped, placed, params))
            if reward > best_reward:
                best_reward = reward
                best_params = params
                best_grasped = grasped
                best_placed = placed
        candidates.sort(key=lambda item: item[0], reverse=True)
        elites = candidates[:elite]
        history.append(elites[0][0])
        mean = [sum(item[3][i] for item in elites) / elite for i in range(PARAM_DIM)]
        std = [max(0.05, s * 0.85) for s in std]

    return history, best_params, best_reward, best_grasped, best_placed


def replay_best(params):
    first = rollout(params)
    second = rollout(params)
    if abs(first - second) > 1e-9:
        sys.exit(f"replay failed: rewards differ ({first} vs {second})")
    return first


def ik_policy_metrics():
    """Reference rollout using the Rust mobile clutter IK policy."""
    policy = rne_py.IkMobileClutterPickPlacePolicy()
    episode = rne_py.MobileManipulatorEpisode(TASK)
    step = episode.reset()
    grasped = False
    placed = False
    for _ in range(policy.total_steps()):
        left, right, shoulder, elbow, gripper, lift = policy.act(step.observation)
        step = episode.step(left, right, shoulder, elbow, gripper, lift)
        if episode.is_grasping:
            grasped = True
        if step.terminated:
            placed = True
            break
    return episode.total_reward, grasped, placed


def main():
    random.seed(0)
    smoke = "--smoke" in sys.argv
    baseline = rollout(WEAK_BASELINE)
    _, scripted_grasped, scripted_placed = ik_policy_metrics()
    history, best_params, best_reward, best_grasped, best_placed = cem_smoke()
    replay_reward = replay_best(best_params)
    print(
        "mobile clutter CEM: "
        f"baseline={baseline:.2f} best={best_reward:.2f} "
        f"grasped={best_grasped} placed={best_placed} "
        f"scripted_grasped={scripted_grasped} scripted_placed={scripted_placed} "
        f"replay={replay_reward:.2f} "
        f"history={[round(x, 2) for x in history]}"
    )
    if smoke:
        if best_reward > baseline + 0.5 and best_grasped and scripted_grasped:
            print(
                "mobile clutter smoke ok: CEM beat baseline, grasped, replay stable"
            )
            return
        sys.exit(
            "smoke failed: mobile clutter CEM "
            f"(baseline={baseline:.2f}, best={best_reward:.2f}, "
            f"best_grasped={best_grasped}, scripted_grasped={scripted_grasped})"
        )


if __name__ == "__main__":
    main()
