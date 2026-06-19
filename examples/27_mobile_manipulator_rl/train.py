"""Self-contained goal-conditioned training loop for the RNE mobile manipulator.

Optimizes a small goal-conditioned linear policy (maps the goal-relative end-effector
offset to joint velocities) with the Cross-Entropy Method (CEM), a derivative-free RL
algorithm. Each candidate is scored on several randomized reach targets, so it must
generalize rather than memorize one pose. Needs only ``rne_py`` and the Python standard
library (no gymnasium / numpy / torch); the mean reward climbs from a failing policy
(~2) to reaching varied targets (~11-12).

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py            # full run
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py --smoke    # short CI run
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py --checkpoint cem.json
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py --checkpoint cem.json --resume --iterations 30
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-out best_policy.json
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --rollout-csv rollout.csv --eval-only
"""

import argparse
import csv
import json
import os
import random
import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/pip install maturin\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

ACTION_LIMIT_RAD_S = 6.0
EPISODE_STEPS = 300
# Policy: [shoulder_bias, shoulder_gain_ee_z, elbow_bias, elbow_gain_ee_y].
PARAM_DIM = 4
TRAINING_CHECKPOINT_VERSION = 1
TRAINING_CHECKPOINT_ALGORITHM = "rne_mobile_manipulator_cem_reach_v1"
POLICY_ARTIFACT_VERSION = 1
POLICY_ARTIFACT_ALGORITHM = "rne_mobile_manipulator_linear_reach_policy_v1"
REPORT_MANIFEST_VERSION = 1
TRAINING_CHECKPOINT_REQUIRED_FIELDS = (
    "schema_version",
    "algorithm",
    "next_iteration",
    "target_iterations",
    "population",
    "elite",
    "seed",
    "param_dim",
    "targets_per_candidate",
    "episode_steps",
    "mean",
    "std",
    "best_params",
    "best_reward",
    "history",
    "rng_state",
)
POLICY_ARTIFACT_REQUIRED_FIELDS = (
    "schema_version",
    "algorithm",
    "param_dim",
    "action_limit_rad_s",
    "params",
    "best_reward",
    "training_iterations",
)
ROLLOUT_CSV_FIELDS = (
    "step",
    "base_x",
    "base_y",
    "base_yaw",
    "ee_x",
    "ee_y",
    "ee_z",
    "target_dx",
    "target_dy",
    "target_dz",
    "shoulder_action",
    "elbow_action",
    "reward",
    "total_reward",
    "done",
)


def _clamp(value, limit):
    return max(-limit, min(limit, value))


# Each CEM candidate is scored on this many freshly sampled targets, so a policy must
# generalize (use the goal) rather than memorize a single pose.
TARGETS_PER_CANDIDATE = 3


def _jsonify_random_state(value):
    if isinstance(value, tuple):
        return [_jsonify_random_state(item) for item in value]
    return value


def _tupleify_random_state(value):
    if isinstance(value, list):
        return tuple(_tupleify_random_state(item) for item in value)
    return value


def _write_json_atomic(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(tmp_path, path)


def _write_rollout_csv(path, rows):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=ROLLOUT_CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(tmp_path, path)


def _save_training_checkpoint(
    path,
    *,
    next_iteration,
    target_iterations,
    population,
    elite,
    seed,
    mean,
    std,
    best_params,
    best_reward,
    history,
    rng,
):
    payload = {
        "schema_version": TRAINING_CHECKPOINT_VERSION,
        "algorithm": TRAINING_CHECKPOINT_ALGORITHM,
        "next_iteration": next_iteration,
        "target_iterations": target_iterations,
        "population": population,
        "elite": elite,
        "seed": seed,
        "param_dim": PARAM_DIM,
        "targets_per_candidate": TARGETS_PER_CANDIDATE,
        "episode_steps": EPISODE_STEPS,
        "mean": mean,
        "std": std,
        "best_params": best_params,
        "best_reward": best_reward,
        "history": history,
        "rng_state": _jsonify_random_state(rng.getstate()),
    }
    _write_json_atomic(path, payload)


def _validate_policy_params(params):
    if len(params) != PARAM_DIM:
        raise ValueError(f"policy param_dim mismatch: {len(params)} != {PARAM_DIM}")
    return [float(value) for value in params]


def _policy_payload(params, *, best_reward, training_iterations):
    return {
        "schema_version": POLICY_ARTIFACT_VERSION,
        "algorithm": POLICY_ARTIFACT_ALGORITHM,
        "param_dim": PARAM_DIM,
        "action_limit_rad_s": ACTION_LIMIT_RAD_S,
        "params": _validate_policy_params(params),
        "best_reward": best_reward,
        "training_iterations": training_iterations,
    }


def _save_policy(path, params, *, best_reward, history):
    _write_json_atomic(
        path,
        _policy_payload(
            params,
            best_reward=best_reward,
            training_iterations=len(history),
        ),
    )


def _load_policy(path):
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    missing = [field for field in POLICY_ARTIFACT_REQUIRED_FIELDS if field not in payload]
    if missing:
        raise ValueError(f"policy missing fields: {', '.join(missing)}")
    if payload.get("schema_version") != POLICY_ARTIFACT_VERSION:
        raise ValueError(f"unsupported policy schema: {payload.get('schema_version')!r}")
    if payload.get("algorithm") != POLICY_ARTIFACT_ALGORITHM:
        raise ValueError(f"unsupported policy algorithm: {payload.get('algorithm')!r}")
    if payload.get("param_dim") != PARAM_DIM:
        raise ValueError(f"policy param_dim mismatch: {payload.get('param_dim')!r}")
    if payload.get("action_limit_rad_s") != ACTION_LIMIT_RAD_S:
        raise ValueError(
            "policy action limit mismatch: "
            f"{payload.get('action_limit_rad_s')!r} != {ACTION_LIMIT_RAD_S!r}"
        )
    payload["params"] = _validate_policy_params(payload["params"])
    return payload


def _load_training_checkpoint(path):
    with open(path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    _validate_training_checkpoint_payload(payload)
    return payload


def _validate_training_checkpoint_payload(payload):
    missing = [
        field for field in TRAINING_CHECKPOINT_REQUIRED_FIELDS if field not in payload
    ]
    if missing:
        raise ValueError(f"checkpoint missing fields: {', '.join(missing)}")
    if payload.get("schema_version") != TRAINING_CHECKPOINT_VERSION:
        raise ValueError(
            "unsupported CEM checkpoint schema: "
            f"{payload.get('schema_version')!r}"
        )
    if payload.get("algorithm") != TRAINING_CHECKPOINT_ALGORITHM:
        raise ValueError(f"unsupported CEM checkpoint algorithm: {payload.get('algorithm')!r}")
    if payload.get("param_dim") != PARAM_DIM:
        raise ValueError(f"checkpoint param_dim mismatch: {payload.get('param_dim')!r}")
    if payload.get("targets_per_candidate") != TARGETS_PER_CANDIDATE:
        raise ValueError(
            "checkpoint targets_per_candidate mismatch: "
            f"{payload.get('targets_per_candidate')!r}"
        )
    if payload.get("episode_steps") != EPISODE_STEPS:
        raise ValueError(f"checkpoint episode_steps mismatch: {payload.get('episode_steps')!r}")


def _restore_training_checkpoint_state(checkpoint, iterations, population, elite, rng):
    if checkpoint["population"] != population:
        raise ValueError(
            "checkpoint population mismatch: "
            f"{checkpoint['population']} != {population}"
        )
    if checkpoint["elite"] != elite:
        raise ValueError(f"checkpoint elite mismatch: {checkpoint['elite']} != {elite}")

    start_iteration = checkpoint["next_iteration"]
    history = [tuple(row) for row in checkpoint["history"]]
    if start_iteration != len(history):
        raise ValueError(
            "checkpoint history length mismatch: "
            f"next_iteration={start_iteration} history={len(history)}"
        )
    if iterations <= start_iteration:
        raise ValueError(
            "checkpoint has no remaining iterations: "
            f"next_iteration={start_iteration} requested_iterations={iterations}"
        )

    rng.setstate(_tupleify_random_state(checkpoint["rng_state"]))
    return {
        "seed": checkpoint["seed"],
        "start_iteration": start_iteration,
        "mean": list(checkpoint["mean"]),
        "std": list(checkpoint["std"]),
        "history": history,
        "best_params": list(checkpoint["best_params"]),
        "best_reward": checkpoint["best_reward"],
        "previous_target": checkpoint["target_iterations"],
    }


def policy_action(params, obs):
    """Goal-conditioned linear policy: maps the goal-relative offset to joint velocities."""
    shoulder = _clamp(params[0] * obs.target_dx + params[1] * obs.target_dz, ACTION_LIMIT_RAD_S)
    elbow = _clamp(params[2] * obs.target_dx + params[3] * obs.target_dz, ACTION_LIMIT_RAD_S)
    return shoulder, elbow


def evaluate_population(population, targets_per_candidate=TARGETS_PER_CANDIDATE):
    """Mean reward of each candidate over several sampled targets (goal-conditioned).

    All candidates run in lock-step on the batched env and share the same sequence of
    randomized targets (one per round), so the comparison is fair and rewards a policy
    that reaches *varied* goals. A candidate's per-round reward is frozen when its episode
    ends so repeated success bonuses cannot inflate it.
    """
    env = rne_py.VectorizedMobileManipulatorEnv("reach_random", len(population))
    totals = [0.0] * len(population)
    observations = env.reset()

    for _ in range(targets_per_candidate):
        round_reward = [None] * len(population)
        for _ in range(EPISODE_STEPS):
            batch = [
                (0.0, 0.0, *policy_action(params, obs), 0.0)
                for params, obs in zip(population, observations)
            ]
            observations, done = env.step(batch)
            for i, finished in enumerate(done):
                if finished and round_reward[i] is None:
                    round_reward[i] = env.episode_reward(i)
            if all(r is not None for r in round_reward):
                break
        for i in range(len(population)):
            totals[i] += (
                env.episode_reward(i) if round_reward[i] is None else round_reward[i]
            )
        observations = env.reset()

    return [total / targets_per_candidate for total in totals]


def evaluate_policy(params, episodes):
    return evaluate_population([_validate_policy_params(params)], targets_per_candidate=episodes)[
        0
    ]


def rollout_policy(params, max_steps=EPISODE_STEPS):
    params = _validate_policy_params(params)
    env = rne_py.VectorizedMobileManipulatorEnv("reach_random", 1)
    obs = env.reset()[0]
    rows = []

    for step in range(max_steps):
        shoulder, elbow = policy_action(params, obs)
        previous_reward = env.episode_reward(0)
        observations, done = env.step([(0.0, 0.0, shoulder, elbow, 0.0)])
        total_reward = env.episode_reward(0)
        rows.append(
            {
                "step": step,
                "base_x": obs.base_x,
                "base_y": obs.base_y,
                "base_yaw": obs.base_yaw,
                "ee_x": obs.ee_x,
                "ee_y": obs.ee_y,
                "ee_z": obs.ee_z,
                "target_dx": obs.target_dx,
                "target_dy": obs.target_dy,
                "target_dz": obs.target_dz,
                "shoulder_action": shoulder,
                "elbow_action": elbow,
                "reward": total_reward - previous_reward,
                "total_reward": total_reward,
                "done": done[0],
            }
        )
        obs = observations[0]
        if done[0]:
            break

    return rows


def _render_report_index(params, *, best_reward, training_iterations, rows):
    param_items = "\n".join(
        f"<li><code>p{index}</code>: {value:.6f}</li>"
        for index, value in enumerate(_validate_policy_params(params))
    )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RNE reach policy report</title>
  <style>
    :root {{
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f8fafc;
      color: #0f172a;
    }}
    body {{
      margin: 0;
      padding: 32px;
    }}
    main {{
      max-width: 920px;
      margin: 0 auto;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    .summary {{
      color: #475569;
      margin: 0 0 24px;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 20px;
    }}
    .card {{
      background: #fff;
      border: 1px solid #cbd5e1;
      border-radius: 8px;
      padding: 16px;
    }}
    .label {{
      color: #64748b;
      font-size: 12px;
    }}
    .value {{
      display: block;
      margin-top: 4px;
      font-size: 22px;
      font-weight: 700;
      font-variant-numeric: tabular-nums;
    }}
    .links {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 20px;
    }}
    a {{
      color: #0f766e;
      font-weight: 700;
    }}
    ul {{
      margin: 10px 0 0;
      padding-left: 20px;
    }}
    code {{
      font-family: ui-monospace, SFMono-Regular, Consolas, monospace;
    }}
  </style>
</head>
<body>
  <main>
    <h1>RNE reach policy report</h1>
    <p class="summary">Generated from a dependency-free CEM linear policy artifact.</p>
    <section class="grid">
      <div class="card"><span class="label">best reward</span><strong class="value">{best_reward:.3f}</strong></div>
      <div class="card"><span class="label">training iterations</span><strong class="value">{training_iterations}</strong></div>
      <div class="card"><span class="label">rollout rows</span><strong class="value">{rows}</strong></div>
    </section>
    <section class="links">
      <div class="card"><a href="rollout.html">Open interactive replay</a><p>Play or scrub the end-effector path.</p></div>
      <div class="card"><a href="rollout.svg">Open static SVG report</a><p>Inspect path, error, reward, and actions.</p></div>
      <div class="card"><a href="rollout.csv">Download rollout CSV</a><p>Use this for plots or offline analysis.</p></div>
      <div class="card"><a href="policy.json">Download policy JSON</a><p>Use this with <code>--policy-in</code>.</p></div>
      <div class="card"><a href="manifest.json">Download manifest JSON</a><p>Machine-readable report metadata.</p></div>
    </section>
    <section class="card">
      <span class="label">policy parameters</span>
      <ul>
        {param_items}
      </ul>
    </section>
  </main>
</body>
</html>
"""


def _write_report_dir(report_dir, params, *, best_reward, training_iterations, rollout_steps):
    from animate_rollout import render_html
    from plot_rollout import load_rollout, render_svg

    os.makedirs(report_dir, exist_ok=True)
    policy_path = os.path.join(report_dir, "policy.json")
    csv_path = os.path.join(report_dir, "rollout.csv")
    svg_path = os.path.join(report_dir, "rollout.svg")
    html_path = os.path.join(report_dir, "rollout.html")
    index_path = os.path.join(report_dir, "index.html")
    manifest_path = os.path.join(report_dir, "manifest.json")

    _write_json_atomic(
        policy_path,
        _policy_payload(
            params,
            best_reward=best_reward,
            training_iterations=training_iterations,
        ),
    )
    rows = rollout_policy(params, max_steps=rollout_steps)
    _write_rollout_csv(csv_path, rows)
    samples = load_rollout(csv_path)
    final_sample = samples[-1]

    with open(svg_path, "w", encoding="utf-8") as handle:
        handle.write(render_svg(samples, os.path.basename(csv_path)))
    with open(html_path, "w", encoding="utf-8") as handle:
        handle.write(render_html(samples, os.path.basename(csv_path)))
    with open(index_path, "w", encoding="utf-8") as handle:
        handle.write(
            _render_report_index(
                params,
                best_reward=best_reward,
                training_iterations=training_iterations,
                rows=len(rows),
            )
        )
    _write_json_atomic(
        manifest_path,
        {
            "schema_version": REPORT_MANIFEST_VERSION,
            "policy_algorithm": POLICY_ARTIFACT_ALGORITHM,
            "policy_schema_version": POLICY_ARTIFACT_VERSION,
            "best_reward": best_reward,
            "training_iterations": training_iterations,
            "rollout_rows": len(rows),
            "final_total_reward": final_sample["total_reward"],
            "final_target_error": final_sample["target_error"],
            "artifacts": {
                "index": "index.html",
                "policy": "policy.json",
                "rollout_csv": "rollout.csv",
                "rollout_svg": "rollout.svg",
                "rollout_html": "rollout.html",
            },
        },
    )

    return {
        "policy": policy_path,
        "csv": csv_path,
        "svg": svg_path,
        "html": html_path,
        "index": index_path,
        "manifest": manifest_path,
        "rows": len(rows),
    }


def cem_train(
    iterations,
    population,
    elite,
    seed,
    checkpoint_path=None,
    resume=False,
    initial_mean=None,
):
    rng = random.Random(seed)
    mean = _validate_policy_params(initial_mean) if initial_mean is not None else [0.0] * PARAM_DIM
    std = [3.0] * PARAM_DIM
    history = []
    best_params = mean
    best_reward = float("-inf")
    start_iteration = 0

    if resume:
        if checkpoint_path is None:
            raise ValueError("--resume requires --checkpoint")
        checkpoint = _load_training_checkpoint(checkpoint_path)
        restored = _restore_training_checkpoint_state(
            checkpoint, iterations, population, elite, rng
        )
        seed = restored["seed"]
        start_iteration = restored["start_iteration"]
        mean = restored["mean"]
        std = restored["std"]
        history = restored["history"]
        best_params = restored["best_params"]
        best_reward = restored["best_reward"]
        previous_target = restored["previous_target"]
        print(
            f"resumed checkpoint {checkpoint_path}: next_iter={start_iteration} "
            f"target_iter={iterations} previous_target={previous_target} seed={seed}"
        )

    for iteration in range(start_iteration, iterations):
        samples = [
            [rng.gauss(mean[d], std[d]) for d in range(PARAM_DIM)]
            for _ in range(population)
        ]
        rewards = evaluate_population(samples)
        scored = sorted(
            zip(rewards, samples), key=lambda sr: sr[0], reverse=True
        )
        elites = [p for _, p in scored[:elite]]
        if scored[0][0] > best_reward:
            best_reward, best_params = scored[0]

        # Refit the sampling distribution to the elite set.
        for d in range(PARAM_DIM):
            values = [p[d] for p in elites]
            mean[d] = sum(values) / len(values)
            var = sum((v - mean[d]) ** 2 for v in values) / len(values)
            std[d] = max(0.1, var**0.5)

        mean_reward = sum(r for r, _ in scored) / len(scored)
        history.append((iteration, mean_reward, scored[0][0]))
        print(
            f"iter {iteration:2d}: mean_reward={mean_reward:6.3f} "
            f"best_reward={scored[0][0]:6.3f}"
        )
        if checkpoint_path is not None:
            _save_training_checkpoint(
                checkpoint_path,
                next_iteration=iteration + 1,
                target_iterations=iterations,
                population=population,
                elite=elite,
                seed=seed,
                mean=mean,
                std=std,
                best_params=best_params,
                best_reward=best_reward,
                history=history,
                rng=rng,
            )
            print(f"checkpoint saved: {checkpoint_path} next_iter={iteration + 1}/{iterations}")

    return best_params, best_reward, history


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke", action="store_true", help="run a short smoke training job")
    parser.add_argument(
        "--iterations",
        type=int,
        help="override total training iteration count; resume continues until this total",
    )
    parser.add_argument("--population", type=int, default=16, help="CEM population size")
    parser.add_argument("--elite", type=int, default=4, help="number of elites per iteration")
    parser.add_argument("--seed", type=int, default=7, help="Python CEM sampling seed")
    parser.add_argument("--checkpoint", help="write CEM training state to this JSON file")
    parser.add_argument("--resume", action="store_true", help="resume from --checkpoint")
    parser.add_argument("--policy-out", help="write the best learned policy to this JSON file")
    parser.add_argument(
        "--policy-in",
        help="load a policy JSON for evaluation or as the initial CEM mean",
    )
    parser.add_argument(
        "--eval-only",
        action="store_true",
        help="evaluate --policy-in without training",
    )
    parser.add_argument(
        "--eval-episodes",
        type=int,
        default=TARGETS_PER_CANDIDATE,
        help="number of randomized reach targets for --eval-only",
    )
    parser.add_argument(
        "--rollout-csv",
        help="write one policy rollout trajectory as CSV after training or evaluation",
    )
    parser.add_argument(
        "--rollout-steps",
        type=int,
        default=EPISODE_STEPS,
        help="maximum steps to record for --rollout-csv",
    )
    parser.add_argument(
        "--report-dir",
        help="write policy.json, rollout.csv, rollout.svg, and rollout.html into this directory",
    )
    args = parser.parse_args()

    if args.population <= 0:
        parser.error("--population must be positive")
    if args.elite <= 0 or args.elite > args.population:
        parser.error("--elite must be in 1..population")
    if args.iterations is not None and args.iterations <= 0:
        parser.error("--iterations must be positive")
    if args.resume and args.checkpoint is None:
        parser.error("--resume requires --checkpoint")
    if args.resume and args.policy_in:
        parser.error("--resume cannot be combined with --policy-in")
    if args.eval_only and args.policy_in is None:
        parser.error("--eval-only requires --policy-in")
    if args.eval_episodes <= 0:
        parser.error("--eval-episodes must be positive")
    if args.rollout_steps <= 0:
        parser.error("--rollout-steps must be positive")
    return args


def main():
    args = parse_args()

    if args.eval_only:
        policy = _load_policy(args.policy_in)
        mean_reward = evaluate_policy(policy["params"], args.eval_episodes)
        print(
            f"eval policy: mean_reward={mean_reward:.3f} "
            f"episodes={args.eval_episodes} params=[{', '.join(f'{p:.2f}' for p in policy['params'])}]"
        )
        if args.rollout_csv is not None:
            rows = rollout_policy(policy["params"], max_steps=args.rollout_steps)
            _write_rollout_csv(args.rollout_csv, rows)
            print(f"rollout csv saved: {args.rollout_csv} rows={len(rows)}")
        if args.report_dir is not None:
            report = _write_report_dir(
                args.report_dir,
                policy["params"],
                best_reward=policy["best_reward"],
                training_iterations=policy["training_iterations"],
                rollout_steps=args.rollout_steps,
            )
            print(
                f"report saved: {args.report_dir} rows={report['rows']} "
                f"policy={report['policy']} html={report['html']}"
            )
        return

    initial_mean = None
    if args.policy_in is not None:
        policy = _load_policy(args.policy_in)
        initial_mean = policy["params"]
        print(
            f"loaded initial policy {args.policy_in}: "
            f"best_reward={policy['best_reward']:.3f} "
            f"params=[{', '.join(f'{p:.2f}' for p in initial_mean)}]"
        )

    iterations = args.iterations if args.iterations is not None else (6 if args.smoke else 20)
    best_params, best_reward, history = cem_train(
        iterations=iterations,
        population=args.population,
        elite=args.elite,
        seed=args.seed,
        checkpoint_path=args.checkpoint,
        resume=args.resume,
        initial_mean=initial_mean,
    )

    first_mean = history[0][1]
    peak_mean = max(mean_reward for _, mean_reward, _ in history)
    print(
        f"trained: best_reward={best_reward:.3f} "
        f"mean_reward first={first_mean:.3f} peak={peak_mean:.3f} "
        f"params=[{', '.join(f'{p:.2f}' for p in best_params)}]"
    )
    if args.policy_out is not None:
        _save_policy(args.policy_out, best_params, best_reward=best_reward, history=history)
        print(f"policy saved: {args.policy_out}")
    if args.rollout_csv is not None:
        rows = rollout_policy(best_params, max_steps=args.rollout_steps)
        _write_rollout_csv(args.rollout_csv, rows)
        print(f"rollout csv saved: {args.rollout_csv} rows={len(rows)}")
    if args.report_dir is not None:
        report = _write_report_dir(
            args.report_dir,
            best_params,
            best_reward=best_reward,
            training_iterations=len(history),
            rollout_steps=args.rollout_steps,
        )
        print(
            f"report saved: {args.report_dir} rows={report['rows']} "
            f"policy={report['policy']} html={report['html']}"
        )

    if args.smoke:
        # CEM must find a goal-conditioned policy that reaches the varied targets: a
        # mean-over-targets reward above ~11 means essentially all sampled goals were hit.
        _ = (first_mean, peak_mean)
        if best_reward > 11.0:
            print("rl train smoke ok: CEM learned a goal-conditioned reach policy")
            return
        sys.exit(f"smoke failed: no generalizing policy found (best={best_reward:.3f})")


if __name__ == "__main__":
    main()
