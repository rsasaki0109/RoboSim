"""CEM training smoke for the fixed-base clutter pick-and-place task.

Phased policy mirrors ``IkClutterPickPlacePolicy``: settle, goal-conditioned
approach, tuned fixed-velocity carry, hold, and release. CEM optimizes approach
gains on the pinned center cube (`clutter_place_center`); carry uses the
scripted winners from the Rust E2E tests.

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
PARAM_DIM = 5  # shoulder_bias, elbow_bias, shoulder_gain, elbow_gain, gripper_close
TASK = "clutter_place_center"

CARRY_SHOULDER_RAD_S = -0.50
CARRY_ELBOW_RAD_S = -0.69
GRIPPER_CLOSE_RAD_S = -2.5
GRIPPER_OPEN_RAD_S = 3.0
# Weak gains that rarely grasp; CEM should beat this baseline.
WEAK_BASELINE = [0.0, 0.0, 0.6, 0.6, -0.5]
# Proven approach gains from the Rust grasp helper (carry/release are scripted).
SCRIPTED_APPROACH = [0.0, 0.0, 4.0, 4.0, -2.5]


def clamp(value, limit=ACTION_LIMIT):
    return max(-limit, min(limit, value))


def act(params, obs, step_idx):
    shoulder_bias, elbow_bias, shoulder_gain, elbow_gain, gripper_close = params
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
            clamp(shoulder_bias + shoulder_gain * obs.target_dx),
            clamp(elbow_bias + elbow_gain * obs.target_dz),
            clamp(gripper_close),
        ]
    if step_idx < carry_end:
        return [
            0.0,
            0.0,
            CARRY_SHOULDER_RAD_S,
            CARRY_ELBOW_RAD_S,
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
    mean = list(WEAK_BASELINE)
    std = [0.15, 0.15, 0.35, 0.35, 0.25]
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


def main():
    random.seed(0)
    smoke = "--smoke" in sys.argv
    baseline = rollout(WEAK_BASELINE)
    _, scripted_grasped, _ = rollout_metrics(SCRIPTED_APPROACH)
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
