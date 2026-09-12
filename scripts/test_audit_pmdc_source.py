"""Synthetic contract tests for the PMDC XLSX auditor."""

import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

from audit_pmdc_source import (
    inspect_archive,
    scan_worksheet,
    verify_source,
    workbook_sheets,
)

MAIN = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
REL = "http://schemas.openxmlformats.org/package/2006/relationships"
OFFICE_REL = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"


def write_zip(path, members):
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, value in members.items():
            archive.writestr(name, value)


class PmdcAuditTests(unittest.TestCase):
    def test_matching_size_cannot_substitute_for_published_hash(self):
        path = Path(__file__)
        with patch("audit_pmdc_source.SOURCE_BYTES", path.stat().st_size):
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                verify_source(path)

    def test_archive_rejects_parent_traversal(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.xlsx"
            write_zip(path, {"../escape.xml": "x"})
            with zipfile.ZipFile(path) as archive:
                with self.assertRaisesRegex(ValueError, "unsafe"):
                    inspect_archive(archive)

    def test_archive_rejects_expansion_over_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "large.xlsx"
            write_zip(path, {"xl/workbook.xml": "x" * 20})
            with zipfile.ZipFile(path) as archive, patch("audit_pmdc_source.MAX_EXPANDED_BYTES", 19):
                with self.assertRaisesRegex(ValueError, "expanded-input"):
                    inspect_archive(archive)

    def test_workbook_requires_local_worksheet_relationship(self):
        workbook = (
            f'<workbook xmlns="{MAIN}" xmlns:r="{OFFICE_REL}">'
            '<sheets><sheet name="Data" sheetId="1" r:id="rId1"/></sheets></workbook>'
        )
        relationships = (
            f'<Relationships xmlns="{REL}"><Relationship Id="rId1" '
            'Target="https://example.invalid/data.xml" TargetMode="External"/></Relationships>'
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "external.xlsx"
            write_zip(path, {
                "xl/workbook.xml": workbook,
                "xl/_rels/workbook.xml.rels": relationships,
            })
            with zipfile.ZipFile(path) as archive:
                with self.assertRaisesRegex(ValueError, "local worksheet"):
                    workbook_sheets(archive)

    def test_trial_header_and_formula_are_retained(self):
        cells = []
        for index in range(12):
            column = chr(ord("A") + index)
            value = "time" if index == 0 else f"channel-{index}"
            cells.append(f'<c r="{column}2" t="inlineStr"><is><t>{value}</t></is></c>')
        formula = '<c r="C3"><f>(A3-B3)/1000000</f><v>0.01</v></c>'
        worksheet = f'<worksheet xmlns="{MAIN}"><sheetData><row r="2">{"".join(cells)}</row><row r="3">{formula}</row></sheetData></worksheet>'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sheet.xlsx"
            write_zip(path, {"xl/worksheets/sheet1.xml": worksheet})
            with zipfile.ZipFile(path) as archive:
                report = scan_worksheet(archive, "xl/worksheets/sheet1.xml", [])
        self.assertEqual(report["trial_count"], 1)
        self.assertEqual(report["trials"][0]["header_cell"], "A2")
        self.assertEqual(report["formulas"], [{
            "cell": "C3", "expression": "(A3-B3)/1000000", "cached_value": "0.01"
        }])

    def test_incomplete_trial_header_is_rejected(self):
        worksheet = f'<worksheet xmlns="{MAIN}"><sheetData><row r="2"><c r="A2" t="inlineStr"><is><t>time</t></is></c></row></sheetData></worksheet>'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sheet.xlsx"
            write_zip(path, {"xl/worksheets/sheet1.xml": worksheet})
            with zipfile.ZipFile(path) as archive:
                with self.assertRaisesRegex(ValueError, "incomplete"):
                    scan_worksheet(archive, "xl/worksheets/sheet1.xml", [])


if __name__ == "__main__":
    unittest.main()
