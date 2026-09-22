"""Negative controls for the archived refinement verifier."""

import gzip
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from verify import DIRECTORY, digest, verify


class RefinementVerificationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / "evidence"
        shutil.copytree(DIRECTORY, self.directory)

    def update_manifest(self):
        path = self.directory / "sources.json"
        value = json.loads(path.read_text())
        for name in value["artifact_sha256"]:
            value["artifact_sha256"][name] = digest((self.directory / name).read_bytes())
        path.write_text(json.dumps(value))

    def change_recording(self, change):
        path = self.directory / "62p5-rollout.json.gz"
        value = json.loads(gzip.decompress(path.read_bytes()))
        change(value)
        raw = json.dumps(value).encode()
        path.write_bytes(gzip.compress(raw, mtime=0))
        search_path = self.directory / "62p5-search.json"
        search = json.loads(search_path.read_text())
        search["evaluations"][0]["rollout_sha256"] = digest(raw)
        search_path.write_text(json.dumps(search))
        self.update_manifest()

    def test_real_campaign_passes(self):
        self.assertTrue(verify(self.directory)["passed_recorded_refinement_gates"])

    def test_changed_bytes_are_rejected(self):
        path = self.directory / "62p5-rollout.json.gz"
        path.write_bytes(path.read_bytes() + b"changed")
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            verify(self.directory)

    def test_duplicate_step_cannot_count_as_refinement(self):
        self.change_recording(lambda value: value.update(dt_s=0.000125))
        with self.assertRaisesRegex(ValueError, "incorrect timestep"):
            verify(self.directory)

    def test_stored_pass_does_not_override_speed_failure(self):
        self.change_recording(lambda value: value.update(peak_joint_speed_ratio=1.06))
        with self.assertRaisesRegex(ValueError, "physical gates failed.*peak_joint_speed_ratio"):
            verify(self.directory)

    def test_changed_controller_is_rejected_even_with_valid_checksums(self):
        self.change_recording(lambda value: value.update(effort_headroom_nm=0.002))
        with self.assertRaisesRegex(ValueError, "applied controller differs"):
            verify(self.directory)

    def test_changed_producer_is_rejected_even_with_valid_checksums(self):
        path = self.directory / "62p5-search.json"
        value = json.loads(path.read_text())
        value["binary_sha256"] = "0" * 64
        path.write_text(json.dumps(value))
        self.update_manifest()
        with self.assertRaisesRegex(ValueError, "producer mismatch"):
            verify(self.directory)


if __name__ == "__main__":
    unittest.main()
