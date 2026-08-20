#!/usr/bin/env python3
"""Fail-closed tests for the installed Python API compatibility verifier."""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import ModuleType

import python_api_compat as api


class PythonApiCompatibilityTests(unittest.TestCase):
    def test_committed_contract_is_strict_and_canonical(self) -> None:
        fixture = Path(__file__).with_name("python-api-v1.json")
        contract = api.read_contract(fixture)
        self.assertEqual(contract["kind"], api.CONTRACT_KIND)
        self.assertEqual(contract["schema_version"], api.CONTRACT_SCHEMA_VERSION)
        self.assertEqual(len(contract["exports"]), 24)
        self.assertEqual(api.digest(contract), api.digest(copy.deepcopy(contract)))

    def test_unknown_field_and_noncanonical_order_fail_closed(self) -> None:
        fixture = Path(__file__).with_name("python-api-v1.json")
        contract = api.read_contract(fixture)
        unknown = copy.deepcopy(contract)
        unknown["unknown"] = True
        with self.assertRaisesRegex(ValueError, "unknown"):
            api.validate_contract(unknown)

        reordered = copy.deepcopy(contract)
        reordered["exports"] = list(reversed(reordered["exports"]))
        with self.assertRaisesRegex(ValueError, "sorted and unique"):
            api.validate_contract(reordered)

    def test_duplicate_json_key_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.json"
            path.write_text('{"kind":"a","kind":"b"}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
                api.read_contract(path)

    def test_mismatch_emits_a_bounded_failed_report(self) -> None:
        module = ModuleType("rne_py")
        module.__version__ = "0.1.0"
        module.VALUE = 7

        def hello() -> None:
            return None

        module.hello = hello
        expected = api.public_contract(module)
        expected["exports"][0]["value"] = 8
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixture = root / "fixture.json"
            report = root / "report.json"
            api.write_json(fixture, expected)
            self.assertFalse(api.verify(fixture, report, module))
            value = json.loads(report.read_text(encoding="utf-8"))
            self.assertFalse(value["passed"])
            self.assertNotEqual(value["fixture_sha256"], value["actual_sha256"])
            self.assertLessEqual(len(value["detail"]), api.MAX_DETAIL_CHARS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
