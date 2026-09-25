#!/usr/bin/env python3
"""Seeded joint search over launch and flight, with bounded parallel batches."""

import argparse
import hashlib
import json
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
import scipy
from g1_backflip_search import Campaign
from scipy.optimize import differential_evolution


def write_json(path, value):
    """Atomically publish one JSON artifact from the single parent writer."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def evaluate_batch(payload):
    """Evaluate an ordered batch in an isolated plant; write no worker artifacts."""
    settings, candidates = payload
    seed = settings["candidate"]
    campaign = Campaign(
        Path(settings["output"]),
        dt_s=settings["dt_s"],
        stage="flip",
        mass_policy=seed["mass_policy"],
        joint_limit_time_constant_s=seed["joint_limit_time_constant_s"],
        joint_limit_margin_rad=seed["joint_limit_margin_rad"],
        profile=seed["screening_profile"],
    )
    for field, attribute in (
        ("early_balance_gains", "early_balance"),
        ("balance_gains", "balance"),
        ("landing_gains", "landing_gains"),
        ("recovery_s", "recovery_s"),
    ):
        setattr(campaign, attribute, seed[field])
    return [campaign.rollout(candidate)[0] for candidate in candidates]


class BatchObjective:
    """Keep search evidence in one writer, in deterministic candidate order."""

    def __init__(self, settings, executor, workers):
        self.settings = settings
        self.executor = executor
        self.workers = workers
        self.evaluations = 0
        self.best = None
        self.passing = None

    def __call__(self, columns):
        if shutil.disk_usage(self.settings["output"]).free < 30 * 1024**3:
            raise RuntimeError("disk reserve below 30 GiB")
        candidates = np.asarray(columns).T
        chunks = [
            chunk.tolist()
            for chunk in np.array_split(candidates, self.workers)
            if len(chunk)
        ]
        payloads = [(self.settings, chunk) for chunk in chunks]
        batches = (
            map(evaluate_batch, payloads)
            if self.executor is None
            else self.executor.map(evaluate_batch, payloads)
        )
        losses = []
        for batch in batches:
            for result in batch:
                self.evaluations += 1
                losses.append(result["loss"])
                if self.best is None or result["loss"] < self.best["loss"]:
                    self.best = result
                    result["evaluations"] = self.evaluations
                    write_json(Path(self.settings["output"], "best.json"), result)
                if result["passed"] and (
                    self.passing is None or result["loss"] < self.passing["loss"]
                ):
                    self.passing = result
                    write_json(Path(self.settings["output"], "passing.json"), result)
        print(
            json.dumps(
                {
                    "evaluations": self.evaluations,
                    "loss": self.best["loss"],
                    "rotation_rad": self.best["signed_rotation_rad"],
                    "flight_s": self.best["longest_flight_s"],
                    "passed": self.passing is not None,
                }
            ),
            flush=True,
        )
        return np.asarray(losses)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--generations", type=int, default=4)
    parser.add_argument("--population-size", type=int, default=56)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--dt-s", type=float, default=0.0005)
    parser.add_argument("--seed", type=int, default=20260922)
    parser.add_argument("--population", type=Path)
    args = parser.parse_args()
    if args.generations < 1 or args.population_size < 5 or not 1 <= args.workers <= 8:
        parser.error("positive generations, population >= 5, and 1..8 workers required")
    seed = json.loads(args.parameters.read_text())
    if any(
        (args.output / name).exists()
        for name in ("search.json", "best.json", "population.json", "passing.json")
    ):
        parser.error(
            "output contains search artifacts; use a new directory for each search"
        )
    args.output.mkdir(parents=True, exist_ok=True)
    settings = {
        "candidate": seed,
        "dt_s": args.dt_s,
        "output": str(args.output.resolve()),
    }
    # Exercise all plant/profile validations before starting worker processes.
    evaluate_batch((settings, []))
    bounds = np.asarray(
        [
            (0.8, 2.8),
            (-0.4, 0.2),
            (0.01, 0.35),
            (-0.4, 2.7),
            (-0.1, 0.52),
            (0.1, 0.65),
            (-1.7, 2.8),
            (1, 2.8),
            (0.1, 1.6),
            (-2.5, 2),
            (-1, 0.8),
            (3.3, 7),
            (-0.8, 0.8),
            (0.2, 1.2),
            (0.0, 0.16),
            (-1.5, 2.5),
        ]
    )
    initial = list(seed["parameters"])
    if len(initial) == 13:
        initial.append(0.6)
    if len(initial) == 14:
        initial.append(0.0)
    if len(initial) == 15:
        initial.append(0.0)
    rng = np.random.default_rng(args.seed)
    if args.population:
        population = np.asarray(json.loads(args.population.read_text())["population"])
        if population.ndim == 2 and population.shape[1] in (14, 15):
            population = np.column_stack(
                (population, np.zeros((len(population), 16 - population.shape[1])))
            )
    else:
        population = rng.normal(
            initial, (bounds[:, 1] - bounds[:, 0]) * 0.15, (args.population_size, 16)
        )
        # Include folded-leg alternatives to the previous knee-torso collision.
        population[::2, 6] = rng.uniform(-1.7, 0.2, len(population[::2]))
        population = np.clip(population, bounds[:, 0], bounds[:, 1])
        population[0] = initial
    if (
        population.ndim != 2
        or population.shape[1] != 16
        or len(population) < 5
        or not np.isfinite(population).all()
    ):
        parser.error("population must contain at least five finite 16-value vectors")
    if np.any(population < bounds[:, 0]) or np.any(population > bounds[:, 1]):
        parser.error("population values must lie within the search bounds")
    configuration = {
        "seed": args.seed,
        "requested_generations": args.generations,
        "population_size": len(population),
        "workers": args.workers,
        "dt_s": args.dt_s,
        "parameters": str(args.parameters),
        "initial_population": population.tolist(),
        "completed": False,
        "bounds": bounds.tolist(),
        "candidate_config": seed,
        "numpy_version": np.__version__,
        "scipy_version": scipy.__version__,
        "controller_sha256": hashlib.sha256(
            Path(__file__).with_name("g1_backflip_search.py").read_bytes()
        ).hexdigest(),
        "optimizer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    write_json(args.output / "search.json", configuration)
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        objective = BatchObjective(settings, executor, args.workers)

        def checkpoint(intermediate_result):
            """Persist every completed generation; warm starts are explicit new searches."""
            checkpoint_data = {
                "generation": intermediate_result.nit,
                "evaluations": objective.evaluations,
                "population": intermediate_result.population.tolist(),
                "losses": intermediate_result.population_energies.tolist(),
            }
            write_json(args.output / "population.json", checkpoint_data)
            return objective.passing is not None

        result = differential_evolution(
            objective,
            bounds,
            init=population,
            maxiter=args.generations,
            rng=rng,
            updating="deferred",
            vectorized=True,
            polish=False,
            callback=checkpoint,
            tol=0,
            atol=0,
        )
        configuration.update(
            completed=True,
            passed=objective.passing is not None,
            evaluations=objective.evaluations,
            completed_generations=result.nit,
            optimizer_message=str(result.message),
        )
    write_json(args.output / "search.json", configuration)
    return 0 if objective.passing is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
