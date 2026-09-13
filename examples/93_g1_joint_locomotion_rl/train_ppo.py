"""Train a joint-space Unitree G1 forward-locomotion policy with PPO.

The environment is the native headless `rne_py` episode. The default vector env
is the native parallel Rust batch (`--vec native`), which steps every
environment on the available cores in one call. The observation, reward, and
action contract follow published OSS humanoid locomotion recipes.

Examples:
    .venv/bin/python examples/93_g1_joint_locomotion_rl/train_ppo.py --smoke
    .venv/bin/python examples/93_g1_joint_locomotion_rl/train_ppo.py \
        --timesteps 20000000 --n-envs 8 --repeat 4 --command 0.4
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run import UnitreeG1JointLocomotionEnv, evaluate_forward_transport

import numpy as np

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
except ImportError:
    sys.exit(
        "stable-baselines3 is required. Install with: "
        "uv pip install 'stable-baselines3==2.6.0' --extra-index-url "
        "https://download.pytorch.org/whl/cpu"
    )


def _make_single(seed, command_forward_m_s, max_steps):
    def _init():
        return UnitreeG1JointLocomotionEnv(
            max_steps=max_steps,
            seed=seed,
            command_forward_m_s=command_forward_m_s,
        )

    return _init


def build_vec_env(args):
    if args.vec == "native":
        from native_vec_env import NativeG1VecEnv

        return NativeG1VecEnv(
            n_envs=args.n_envs,
            seed=args.seed,
            command_forward_m_s=args.command,
            max_steps=args.max_steps,
            repeat=args.repeat,
        )
    if args.vec == "subproc":
        return SubprocVecEnv(
            [
                _make_single(args.seed + index, args.command, args.max_steps)
                for index in range(args.n_envs)
            ]
        )
    return DummyVecEnv(
        [_make_single(args.seed + index, args.command, args.max_steps) for index in range(args.n_envs)]
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=20_000_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--seed", type=int, default=93)
    parser.add_argument("--command", type=float, default=0.4)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--repeat", type=int, default=4)
    parser.add_argument("--vec", choices=["native", "subproc", "dummy"], default="native")
    parser.add_argument("--out", type=str, default="target/g1_joint_locomotion_ppo")
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()

    if args.smoke:
        args.timesteps = 2048
        args.n_envs = 2
        args.max_steps = 100

    env = build_vec_env(args)
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=256,
        batch_size=2048,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,
        max_grad_norm=1.0,
        policy_kwargs=dict(net_arch=dict(pi=[256, 128], vf=[256, 128])),
        device="cpu",
        verbose=1,
        seed=args.seed,
    )
    try:
        model.learn(total_timesteps=args.timesteps, progress_bar=False)
    finally:
        env.close()

    if not args.smoke:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        model.save(args.out)
        print(f"saved PPO policy to {args.out}")

    eval_env = UnitreeG1JointLocomotionEnv(
        max_steps=args.max_steps,
        seed=args.seed + 999,
        command_forward_m_s=args.command,
    )
    displacement, min_height = evaluate_forward_transport(
        eval_env, policy=model, steps=args.max_steps
    )
    mean_speed = displacement / (args.max_steps / 60.0)
    print(
        "eval: forward_displacement={:.3f} m mean_speed={:.3f} m/s min_height={:.3f} m".format(
            displacement, mean_speed, min_height
        )
    )
    if args.smoke:
        if not np.isfinite(displacement) or not np.isfinite(min_height):
            raise SystemExit("G1 joint locomotion PPO smoke failed")


if __name__ == "__main__":
    main()
