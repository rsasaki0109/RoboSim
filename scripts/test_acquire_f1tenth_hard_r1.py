"""Synthetic tests for bounded F1TENTH hard-r1 acquisition."""

import io
import tempfile
import unittest
from pathlib import Path

from acquire_f1tenth_hard_r1 import acquire_response


class Response(io.BytesIO):
    def __init__(self, payload, declared_length=None):
        super().__init__(payload)
        self.status = 200
        self.headers = {}
        if declared_length is not None:
            self.headers["Content-Length"] = str(declared_length)


class F1TenthAcquisitionTests(unittest.TestCase):
    def test_exact_response_is_promoted_without_partial(self):
        payload = b"pinned-f1tenth-fixture"
        expected_md5 = "76a97de670f7e772d11f0a74fda82094"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run.bag"
            result = acquire_response(
                Response(payload, len(payload)), output, len(payload), expected_md5
            )
            self.assertEqual(output.read_bytes(), payload)
            self.assertFalse(Path(str(output) + ".partial").exists())
            self.assertEqual(result["bytes"], len(payload))

    def test_length_hash_overflow_and_existing_paths_fail_closed(self):
        payload = b"bounded"
        md5 = "5bd3307450f87ef12930e238afe9c7c5"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, "Content-Length"):
                acquire_response(Response(payload, 99), root / "length.bag", len(payload), md5)
            with self.assertRaisesRegex(ValueError, "exceeds"):
                acquire_response(Response(payload), root / "overflow.bag", len(payload) - 1, md5)
            with self.assertRaisesRegex(ValueError, "checksum"):
                acquire_response(Response(payload), root / "hash.bag", len(payload), "0" * 32)
            existing = root / "existing.bag"
            existing.write_bytes(b"keep")
            with self.assertRaises(FileExistsError):
                acquire_response(Response(payload), existing, len(payload), md5)
            self.assertEqual(existing.read_bytes(), b"keep")


if __name__ == "__main__":
    unittest.main()
