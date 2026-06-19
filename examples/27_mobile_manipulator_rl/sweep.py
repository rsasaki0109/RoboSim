"""Run a small seed sweep for the dependency-free CEM reach trainer.

Each run writes a report bundle, then the sweep builds a leaderboard from those
manifests. The script uses only the Python standard library.
"""

import argparse
import os
import subprocess
import sys
import time


def _script_path(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)


def _run(command, dry_run):
    printable = " ".join(command)
    print(f"$ {printable}")
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

    print(f"parallel jobs={jobs} commands={len(commands)}")
    pending = list(commands)
    running = []
    failures = []

    while pending or running:
        while pending and len(running) < jobs:
            command = pending.pop(0)
            print(f"$ {' '.join(command)}")
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
    for offset in range(args.runs):
        seed = args.seed_start + offset
        run_dir = os.path.join(args.out, f"seed_{seed:04d}")
        checkpoint = os.path.join(run_dir, "checkpoint.json")
        manifest = os.path.join(run_dir, "manifest.json")
        if args.skip_complete and os.path.isfile(manifest):
            print(f"skip complete seed={seed} report={manifest}")
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
    _run(
        [
            sys.executable,
            compare_script,
            args.out,
            "--html",
            leaderboard,
            "--csv",
            leaderboard_csv,
        ],
        args.dry_run,
    )
    print(f"sweep output: {args.out}")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
    except Exception as error:
        sys.exit(str(error))
