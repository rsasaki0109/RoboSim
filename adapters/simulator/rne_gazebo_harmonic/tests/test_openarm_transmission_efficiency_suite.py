from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PAYLOAD = load_module("payload_for_transmission", SCRIPT_DIR / "build_openarm_payload_suite.py")
COULOMB = load_module(
    "coulomb_for_transmission", SCRIPT_DIR / "build_openarm_coulomb_friction_suite.py"
)
TRANSMISSION = load_module(
    "openarm_transmission_suite",
    SCRIPT_DIR / "build_openarm_transmission_efficiency_suite.py",
)


class OpenArmTransmissionEfficiencySuiteTests(unittest.TestCase):
    def compile_baseline(self, root: Path) -> Path:
        payload = root / "payload"
        PAYLOAD.compile_suite(
            SCRIPT_DIR / "openarm_payload_experiments.json",
            SCRIPT_DIR / "openarm_v2_right.rne.urdf",
            SCRIPT_DIR / "openarm_right.adapter.json",
            SCRIPT_DIR / "openarm_right.rne_actuation.json",
            payload,
        )
        coulomb = root / "coulomb"
        COULOMB.compile_suite(
            SCRIPT_DIR / "openarm_coulomb_friction_experiments.json",
            payload / "payload-0000g",
            SCRIPT_DIR / "openarm_right.rne_actuation.json",
            coulomb,
        )
        return coulomb / "joint5-coulomb-0500mn"

    def test_efficiency_grid_changes_only_typed_actuation_and_adapter(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = self.compile_baseline(root)
            first = root / "first"
            replay = root / "replay"
            suite = TRANSMISSION.compile_suite(
                SCRIPT_DIR / "openarm_transmission_efficiency_experiments.json",
                baseline,
                first,
            )
            TRANSMISSION.compile_suite(
                SCRIPT_DIR / "openarm_transmission_efficiency_experiments.json",
                baseline,
                replay,
            )
            self.assertEqual(
                [case["case_id"] for case in suite["cases"]],
                [
                    "joint5-efficiency-100pct",
                    "joint5-efficiency-090pct",
                    "joint5-efficiency-075pct",
                    "joint5-efficiency-050pct",
                    "joint5-efficiency-025pct",
                ],
            )
            self.assertEqual(
                (first / "transmission-efficiency-suite.json").read_bytes(),
                (replay / "transmission-efficiency-suite.json").read_bytes(),
            )
            lossy = suite["cases"][-1]
            self.assertEqual(lossy["portable_realized_efficiency"], 0.25)
            self.assertEqual(lossy["gazebo_realized_efficiency"], 0.25)
            for name, digest in lossy["fixed_artifact_sha256"].items():
                self.assertEqual(
                    TRANSMISSION.sha256(first / lossy["case_id"] / name), digest
                )
            actuation = json.loads(
                (first / lossy["case_id"] / TRANSMISSION.ACTUATION_FILE).read_text()
            )
            efficiencies = [item["transmission_efficiency"] for item in actuation["joints"]]
            self.assertEqual(efficiencies.count(0.25), 1)
            self.assertEqual(efficiencies.count(1.0), len(efficiencies) - 1)

    def test_invalid_efficiency_manifest_fails_closed(self) -> None:
        manifest = json.loads(
            (SCRIPT_DIR / "openarm_transmission_efficiency_experiments.json").read_text()
        )
        manifest["transmission_efficiency_grid"][-1] = 0.0
        with self.assertRaisesRegex(ValueError, "unsupported transmission"):
            TRANSMISSION.validate_manifest(manifest)


if __name__ == "__main__":
    unittest.main()
