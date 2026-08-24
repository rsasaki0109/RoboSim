from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PAYLOAD = load_module("payload_suite_for_authority", SCRIPT_DIR / "build_openarm_payload_suite.py")
AUTHORITY = load_module("openarm_authority_suite", SCRIPT_DIR / "build_openarm_authority_suite.py")


class OpenArmAuthoritySuiteTests(unittest.TestCase):
    def test_compiler_scales_only_controlled_joint_and_hashes_every_runtime(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload_root = root / "payload"
            PAYLOAD.compile_suite(
                SCRIPT_DIR / "openarm_payload_experiments.json",
                ROOT / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
                SCRIPT_DIR / "openarm_right.adapter.json",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                payload_root,
            )
            output = root / "authority"
            suite = AUTHORITY.compile_suite(
                SCRIPT_DIR / "openarm_authority_experiments.json",
                payload_root / "payload-0000g",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                output,
            )
            self.assertEqual(len(suite["cases"]), 5)
            self.assertEqual(suite["cases"][0]["authority_scale"], 1.0)
            self.assertEqual(suite["cases"][-1]["authority_scale"], 0.2)
            baseline = json.loads(
                (output / "authority-1000permille/openarm_right.rne_actuation.json").read_text()
            )
            degraded = json.loads(
                (output / "authority-0200permille/openarm_right.rne_actuation.json").read_text()
            )
            baseline_limits = [item["max_effort_nm"] for item in baseline["joints"]]
            degraded_limits = [item["max_effort_nm"] for item in degraded["joints"]]
            self.assertEqual(degraded_limits[:4], baseline_limits[:4])
            self.assertAlmostEqual(degraded_limits[4], 1.4)
            self.assertEqual(degraded_limits[5:], baseline_limits[5:])
            for case in suite["cases"]:
                case_dir = output / case["case_id"]
                runtime = json.loads((case_dir / "runtime.json").read_text())
                for item in runtime["artifacts"]:
                    path = case_dir / item["file"]
                    self.assertEqual(path.stat().st_size, item["size_bytes"])
                    self.assertEqual(AUTHORITY.sha256(path), item["sha256"])


if __name__ == "__main__":
    unittest.main()
