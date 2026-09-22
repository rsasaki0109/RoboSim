#!/usr/bin/env python3
"""Verify the archived native refinement campaign; no hardware claim."""

import gzip
import hashlib
import json
from pathlib import Path
import sys

DIRECTORY = Path(__file__).resolve().parent
ROOT = DIRECTORY.parents[4]
sys.path.insert(0, str(ROOT / "scripts"))
from g1_native_audit import audit


def digest(data):
    return hashlib.sha256(data).hexdigest()


def verify(directory=DIRECTORY):
    """Check hashes, identical inputs and the existing physical gates at all steps."""
    sources = json.loads((directory / "sources.json").read_text())
    for name, expected in sources["artifact_sha256"].items():
        if digest((directory / name).read_bytes()) != expected:
            raise ValueError(f"artifact checksum mismatch: {name}")
    model = json.loads((directory / "model-provenance.json").read_text())
    if model["producer_source_commit"] != sources["source_commit"]:
        raise ValueError("model and executable source commits differ")
    for row in model["source_assets"]:
        path = (ROOT / row["path"]).resolve()
        if not path.is_relative_to(ROOT) or digest(path.read_bytes()) != row["sha256"]:
            raise ValueError(f"source asset mismatch: {row['path']}")
    for name, expected in model["generated_files"].items():
        if digest((directory / ("model-" + name)).read_bytes()) != expected:
            raise ValueError(f"generated model mismatch: {name}")

    rows = []
    reference_candidate = None
    reference_settings = None
    common_fields = (
        "scene", "mass_kg", "knee_limit_nm", "solver_iterations",
        "joint_link_names", "structural_excluded_link_pairs",
        "compound_part_counts", "convex_collider_count", "roll_balance",
        "landing_stance_rad", "landing_capture_gain_rad_per_m",
        "landing_early_com_velocity_gain_s_per_m", "balance_velocity_source",
    )
    for tag, dt_s in (("500", 0.0005), ("125", 0.000125), ("62p5", 0.0000625)):
        candidate = json.loads((directory / f"{tag}-candidate-0000.json").read_text())
        search = json.loads((directory / f"{tag}-search.json").read_text())
        raw = gzip.decompress((directory / f"{tag}-rollout.json.gz").read_bytes())
        rollout = json.loads(raw)
        if search["binary_sha256"] != sources["binary_sha256"]:
            raise ValueError(f"producer mismatch: {tag}")
        evaluation = search["evaluations"][0]
        if evaluation["index"] != 0 or evaluation["rollout_sha256"] != digest(raw):
            raise ValueError(f"search/recording mismatch: {tag}")
        if evaluation["candidate"] != candidate:
            raise ValueError(f"search/candidate mismatch: {tag}")
        if any(rollout.get(key) != value for key, value in candidate.items()):
            raise ValueError(f"applied controller differs from candidate: {tag}")
        if rollout["dt_s"] != dt_s or search["dt_s"] != dt_s:
            raise ValueError(f"incorrect timestep: {tag}")
        if rollout["maneuver_duration_s"] != 15 or search["maneuver_duration_s"] != 15:
            raise ValueError(f"incorrect validation duration: {tag}")
        settings = {key: rollout[key] for key in common_fields}
        settings["effort_limits"] = [
            (row["link_name"], row["limit_nm"]) for row in rollout["joint_effort_audit"]
        ]
        if reference_candidate is not None and (
            candidate != reference_candidate or settings != reference_settings
        ):
            raise ValueError(f"candidate or model settings changed: {tag}")
        reference_candidate, reference_settings = candidate, settings
        report = audit(rollout)
        if not report["passed_recorded_gates"]:
            raise ValueError(f"physical gates failed at {tag}: {report['failed_gates']}")
        rows.append({
            "dt_us": dt_s * 1e6,
            "rollout_sha256": digest(raw),
            "signed_rotation_rad": rollout["signed_rotation_rad"],
            "peak_joint_speed_ratio": rollout["peak_joint_speed_ratio"],
            "final_second_max_base_speed_m_s": rollout["final_second_max_base_speed_m_s"],
            "final_second_min_upright": rollout["final_second_min_upright"],
            "final_standing_error": rollout["final_standing_error"],
            "passed_recorded_gates": True,
        })
    gif = json.loads((directory / "gif-provenance.json").read_text())
    gif_path = (ROOT / gif["gif_path"]).resolve()
    if not gif_path.is_relative_to(ROOT) or digest(gif_path.read_bytes()) != gif["gif_sha256"]:
        raise ValueError("rendered GIF checksum mismatch")
    if gif["rollout_sha256"] != rows[-1]["rollout_sha256"]:
        raise ValueError("GIF does not reference the finest successful recording")
    if gif["visual_urdf_sha256"] != digest(
        (ROOT / "assets/robots/g1_description/g1_23dof.urdf").read_bytes()
    ):
        raise ValueError("GIF visual model mismatch")
    return {
        "passed_recorded_refinement_gates": True,
        "same_candidate_model_settings_and_producer": True,
        "source_commit": sources["source_commit"],
        "binary_sha256": sources["binary_sha256"],
        "gif_sha256": gif["gif_sha256"],
        "hardware_validated": False,
        "scope": "Native full-contact outcome reproduced at 125 and 62.5 us, with 500 us comparison; not a proof of trajectory convergence or hardware readiness.",
        "runs": rows,
    }


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))
