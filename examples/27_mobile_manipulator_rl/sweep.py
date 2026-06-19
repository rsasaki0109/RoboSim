"""Run a small seed sweep for the dependency-free CEM reach trainer.

Each run writes a report bundle, then the sweep builds a leaderboard from those
manifests. The script uses only the Python standard library.
"""

import argparse
import json
import os
import subprocess
import sys
import time

SWEEP_MANIFEST_SCHEMA_VERSION = 1


def _script_path(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)


def _relative_path(path, root):
    return os.path.relpath(path, root).replace(os.sep, "/")


def _write_json_atomic(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, path)


def _run(command, dry_run):
    printable = " ".join(command)
    print(f"$ {printable}", flush=True)
    if not dry_run:
        subprocess.run(command, check=True)


def _run_parallel(commands, jobs, dry_run):
    if dry_run:
        for command in commands:
            _run(command, dry_run=True)
        return
    if jobs == 1:
        for command in commands:
            _run(command, dry_run=False)
        return

    print(f"parallel jobs={jobs} commands={len(commands)}", flush=True)
    pending = list(commands)
    running = []
    failures = []

    while pending or running:
        while pending and len(running) < jobs:
            command = pending.pop(0)
            print(f"$ {' '.join(command)}", flush=True)
            running.append((command, subprocess.Popen(command)))

        next_running = []
        for command, process in running:
            code = process.poll()
            if code is None:
                next_running.append((command, process))
                continue
            if code != 0:
                failures.append((command, code))
        running = next_running

        if failures:
            for _, process in running:
                process.terminate()
            for _, process in running:
                process.wait()
            command, code = failures[0]
            raise subprocess.CalledProcessError(code, command)

        if running:
            time.sleep(0.1)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="reports/sweep", help="output directory for reports")
    parser.add_argument("--runs", type=int, default=4, help="number of seeds to train")
    parser.add_argument("--seed-start", type=int, default=0, help="first seed value")
    parser.add_argument("--iterations", type=int, default=12, help="CEM iterations per run")
    parser.add_argument("--population", type=int, default=16, help="CEM population size")
    parser.add_argument("--elite", type=int, default=4, help="CEM elite count")
    parser.add_argument(
        "--rollout-steps",
        type=int,
        default=300,
        help="maximum rollout rows recorded per report",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="resume any seed with an existing checkpoint.json",
    )
    parser.add_argument(
        "--skip-complete",
        action="store_true",
        help="skip any seed with an existing manifest.json report",
    )
    parser.add_argument("--jobs", type=int, default=1, help="parallel seed training jobs")
    parser.add_argument(
        "--require-final-error-at-most",
        type=float,
        help="fail unless the sweep winner has final_target_error at most this value",
    )
    parser.add_argument(
        "--require-final-reward-at-least",
        type=float,
        help="fail unless the sweep winner has final_total_reward at least this value",
    )
    parser.add_argument("--dry-run", action="store_true", help="print commands without running")
    args = parser.parse_args()

    if args.runs <= 0:
        parser.error("--runs must be positive")
    if args.iterations <= 0:
        parser.error("--iterations must be positive")
    if args.population <= 0:
        parser.error("--population must be positive")
    if args.elite <= 0 or args.elite > args.population:
        parser.error("--elite must be in 1..population")
    if args.rollout_steps <= 0:
        parser.error("--rollout-steps must be positive")
    if args.jobs <= 0:
        parser.error("--jobs must be positive")
    return args


def main():
    args = parse_args()
    train_script = _script_path("train.py")
    compare_script = _script_path("compare_reports.py")
    os.makedirs(args.out, exist_ok=True)

    commands = []
    expected_manifests = []
    for offset in range(args.runs):
        seed = args.seed_start + offset
        run_dir = os.path.join(args.out, f"seed_{seed:04d}")
        checkpoint = os.path.join(run_dir, "checkpoint.json")
        manifest = os.path.join(run_dir, "manifest.json")
        expected_manifests.append(manifest)
        if args.skip_complete and os.path.isfile(manifest):
            print(f"skip complete seed={seed} report={manifest}", flush=True)
            continue
        command = [
            sys.executable,
            train_script,
            "--seed",
            str(seed),
            "--iterations",
            str(args.iterations),
            "--population",
            str(args.population),
            "--elite",
            str(args.elite),
            "--checkpoint",
            checkpoint,
            "--report-dir",
            run_dir,
            "--rollout-steps",
            str(args.rollout_steps),
        ]
        if args.resume and os.path.isfile(checkpoint):
            command.append("--resume")
        commands.append(command)

    _run_parallel(commands, args.jobs, args.dry_run)

    leaderboard = os.path.join(args.out, "leaderboard.html")
    leaderboard_csv = os.path.join(args.out, "leaderboard.csv")
    leaderboard_json = os.path.join(args.out, "leaderboard.json")
    best_policy = os.path.join(args.out, "best_policy.json")
    best_report = os.path.join(args.out, "best_report.json")
    sweep_manifest = os.path.join(args.out, "sweep_manifest.json")
    compare_command = [
        sys.executable,
        compare_script,
        *expected_manifests,
        "--html",
        leaderboard,
        "--csv",
        leaderboard_csv,
        "--json",
        leaderboard_json,
        "--best-policy-out",
        best_policy,
        "--best-report-out",
        best_report,
        "--require-reports-at-least",
        str(args.runs),
    ]
    if args.require_final_error_at_most is not None:
        compare_command.extend(
            ["--require-final-error-at-most", str(args.require_final_error_at_most)]
        )
    if args.require_final_reward_at_least is not None:
        compare_command.extend(
            ["--require-final-reward-at-least", str(args.require_final_reward_at_least)]
        )
    _run(compare_command, args.dry_run)
    if not args.dry_run:
        _write_json_atomic(
            sweep_manifest,
            {
                "schema_version": SWEEP_MANIFEST_SCHEMA_VERSION,
                "runner": "examples/27_mobile_manipulator_rl/sweep.py",
                "config": {
                    "runs": args.runs,
                    "seed_start": args.seed_start,
                    "iterations": args.iterations,
                    "population": args.population,
                    "elite": args.elite,
                    "rollout_steps": args.rollout_steps,
                    "jobs": args.jobs,
                    "resume": args.resume,
                    "skip_complete": args.skip_complete,
                },
                "seeds": [
                    args.seed_start + offset for offset in range(args.runs)
                ],
                "quality_gates": {
                    "reports_at_least": args.runs,
                    "final_error_at_most": args.require_final_error_at_most,
                    "final_reward_at_least": args.require_final_reward_at_least,
                },
                "expected_manifests": [
                    _relative_path(path, args.out) for path in expected_manifests
                ],
                "outputs": {
                    "leaderboard_html": _relative_path(leaderboard, args.out),
                    "leaderboard_csv": _relative_path(leaderboard_csv, args.out),
                    "leaderboard_json": _relative_path(leaderboard_json, args.out),
                    "best_policy": _relative_path(best_policy, args.out),
                    "best_report": _relative_path(best_report, args.out),
                    "sweep_manifest": _relative_path(sweep_manifest, args.out),
                },
                "commands": {
                    "train": commands,
                    "compare": compare_command,
                },
            },
        )
        print(f"sweep manifest saved: {sweep_manifest}", flush=True)
    print(f"sweep output: {args.out}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
    except Exception as error:
        sys.exit(str(error))
