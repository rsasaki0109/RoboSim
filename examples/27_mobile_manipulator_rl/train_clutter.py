"""CEM training smoke for the fixed-base clutter pick-and-place task.

Phased policy mirrors ``IkClutterPickPlacePolicy``: settle, goal-conditioned
approach, held carry, hold, and release. CEM optimizes approach gains and the
gripper command on the pinned center cube (`clutter_place_center`); under the
stable arm dynamics the grasp weld is what separates success from failure, so
the weak baseline keeps the gripper open and the CEM must discover closing it.
The carry holds the grasp pose, matching the re-derived Rust E2E tests (the
approach hands off next to the place bearing and the derived place target is
where the held carry sets the cube down).

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_clutter.py --smoke
"""

import random
import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

ACTION_LIMIT = 6.0
SETTLE_STEPS = 20
APPROACH_STEPS = 360
CARRY_STEPS = 340
HOLD_STEPS = 80
RELEASE_STEPS = 150
EPISODE_STEPS = (
    SETTLE_STEPS + APPROACH_STEPS + CARRY_STEPS + HOLD_STEPS + RELEASE_STEPS
)
PARAM_DIM = 5  # shoulder_gain, elbow_gain, gripper_close, carry_shoulder, carry_elbow
TASK = "clutter_place_center"

GRIPPER_CLOSE_RAD_S = -2.5
GRIPPER_OPEN_RAD_S = 3.0
# Weak baseline: gripper held open (never triggers the contact weld), so it
# collects at most approach shaping; CEM should beat it by learning to close
# the gripper and carry the welded cube toward the place target.
WEAK_BASELINE = [0.6, 0.6, 0.5, 0.0, 0.0]
# Proven approach gains (carry/release timing scripted; carry velocities zero
# hold the grasp pose like the re-derived Rust fixed-carry test).
SCRIPTED_APPROACH = [4.0, 4.0, -2.5, 0.0, 0.0]


def clamp(value, limit=ACTION_LIMIT):
    return max(-limit, min(limit, value))


def act(params, obs, step_idx):
    shoulder_gain, elbow_gain, gripper_close, carry_shoulder, carry_elbow = params
    if step_idx < SETTLE_STEPS:
        return [0.0, 0.0, 0.0, 0.0, 0.0]
    approach_end = SETTLE_STEPS + APPROACH_STEPS
    carry_end = approach_end + CARRY_STEPS
    hold_end = carry_end + HOLD_STEPS
    release_end = hold_end + RELEASE_STEPS
    if step_idx < approach_end:
        return [
            0.0,
            0.0,
            clamp(shoulder_gain * obs.target_dx),
            clamp(elbow_gain * obs.target_dz),
            clamp(gripper_close),
        ]
    if step_idx < carry_end:
        return [
            0.0,
            0.0,
            clamp(carry_shoulder),
            clamp(carry_elbow),
            GRIPPER_CLOSE_RAD_S,
        ]
    if step_idx < hold_end:
        return [0.0, 0.0, 0.0, 0.0, GRIPPER_CLOSE_RAD_S]
    if step_idx < release_end:
        return [0.0, 0.0, 0.0, 0.0, GRIPPER_OPEN_RAD_S]
    return [0.0, 0.0, 0.0, 0.0, 0.0]


def rollout_metrics(params):
    episode = rne_py.MobileManipulatorEpisode(TASK)
    step = episode.reset()
    grasped = False
    placed = False
    for step_idx in range(EPISODE_STEPS):
        action = act(params, step.observation, step_idx)
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
    population = 16
    elite = 4
    iterations = 8
    mean = [0.6, 0.6, 0.0, 0.0, 0.0]
    std = [0.35, 0.35, 0.5, 0.3, 0.3]
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
        std = [max(0.12, s * 0.9) for s in std]

    return history, best_params, best_reward, best_grasped, best_placed


def replay_best(params):
    first = rollout(params)
    second = rollout(params)
    if abs(first - second) > 1e-9:
        sys.exit(f"replay failed: rewards differ ({first} vs {second})")
    return first


def ik_policy_grasps() -> bool:
    """Reference grasp check using the Rust IK clutter policy (two-finger weld)."""
    policy = rne_py.IkClutterPickPlacePolicy()
    episode = rne_py.MobileManipulatorEpisode(TASK)
    step = episode.reset()
    for _ in range(policy.total_steps()):
        left, right, shoulder, elbow, gripper, lift = policy.act(step.observation)
        step = episode.step(left, right, shoulder, elbow, gripper, lift)
        if episode.is_grasping:
            return True
    return False


def main():
    random.seed(0)
    smoke = "--smoke" in sys.argv
    baseline = rollout(WEAK_BASELINE)
    scripted_grasped = ik_policy_grasps()
    history, best_params, best_reward, best_grasped, best_placed = cem_smoke()
    replay_reward = replay_best(best_params)
    print(
        "clutter CEM: "
        f"baseline={baseline:.2f} best={best_reward:.2f} "
        f"grasped={best_grasped} placed={best_placed} "
        f"scripted_grasped={scripted_grasped} "
        f"replay={replay_reward:.2f} "
        f"history={[round(x, 2) for x in history]}"
    )
    if smoke:
        if best_reward > baseline + 0.5 and best_grasped and scripted_grasped:
            print("clutter smoke ok: CEM beat baseline, grasped, replay stable")
            return
        sys.exit(
            "smoke failed: clutter CEM "
            f"(baseline={baseline:.2f}, best={best_reward:.2f}, "
            f"best_grasped={best_grasped}, scripted_grasped={scripted_grasped})"
        )


if __name__ == "__main__":
    main()
