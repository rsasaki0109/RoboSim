#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "openarm_physical_configuration_report",
        SCRIPT_DIR / "build_openarm_physical_configuration_report.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = load_module()


class PhysicalConfigurationReportTests(unittest.TestCase):
    def test_fixed_requirement_directions_are_enforced(self) -> None:
        manifest = json.loads(
            (SCRIPT_DIR / "openarm_physical_configuration_experiments.json").read_text()
        )
        requirements = MODULE.requirement_map(manifest)
        minimum = requirements[
            "physical_configuration.minimum_coupled_response_rms_delta_rad"
        ]
        maximum = requirements["physical_configuration.maximum_mass_delta_kg"]
        required = requirements["physical_configuration.requires_exact_replay"]
        self.assertEqual(MODULE.check(minimum, 1e-5)["status"], "passed")
        self.assertEqual(MODULE.check(minimum, 0.0)["status"], "failed")
        self.assertEqual(MODULE.check(maximum, 0.0)["status"], "passed")
        self.assertEqual(MODULE.check(maximum, 1.0)["status"], "failed")
        self.assertEqual(MODULE.check(required, True)["status"], "passed")
        self.assertEqual(MODULE.check(required, False)["status"], "failed")

    def test_model_realization_delta_detects_nested_tamper(self) -> None:
        left = [{"mass_kg": 1.0, "tensor": {"ixx": 0.1, "iyy": 0.2}}]
        right = [{"mass_kg": 1.0, "tensor": {"ixx": 0.1, "iyy": 0.25}}]
        self.assertAlmostEqual(MODULE.maximum_numeric_delta(left, right), 0.05)
        self.assertEqual(MODULE.maximum_numeric_delta(left, left), 0.0)
        self.assertEqual(MODULE.maximum_numeric_delta(left, []), float("inf"))

    def test_coupling_matrix_flattening_is_column_major_and_complete(self) -> None:
        matrix = {
            "columns": [
                {"mean_output_gain_rad_per_rad": [1.0, 2.0]},
                {"mean_output_gain_rad_per_rad": [3.0, 4.0]},
            ]
        }
        self.assertEqual(MODULE.matrix_values(matrix), [1.0, 2.0, 3.0, 4.0])

    def test_runtime_artifact_tamper_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            files = {}
            for role in ("world", "robot_model", "adapter_config"):
                path = root / f"{role}.txt"
                path.write_text(role)
                files[role] = path
            runtime = {
                "kind": "rne_external_simulator_runtime_manifest",
                "schema_version": 1,
                "simulator_id": "gazebo_sim",
                "artifacts": [
                    {
                        "role": role,
                        "file": path.name,
                        "size_bytes": path.stat().st_size,
                        "sha256": MODULE.sha256(path),
                    }
                    for role, path in files.items()
                ],
            }
            runtime_path = root / "runtime.json"
            runtime_path.write_text(json.dumps(runtime))
            self.assertEqual(set(MODULE.validate_runtime(runtime_path)), set(files))
            files["robot_model"].write_text("tampered")
            with self.assertRaisesRegex(ValueError, "differs from manifest"):
                MODULE.validate_runtime(runtime_path)


if __name__ == "__main__":
    unittest.main()
