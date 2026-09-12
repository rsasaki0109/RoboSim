"""Synthetic tests for the unevaluated two-run PMDC final seal."""

import tempfile
import unittest
import zipfile
from pathlib import Path

from convert_pmdc_prbs9 import ANNOTATION_TEXT, CHANNELS
from seal_pmdc_final import FinalRunLayout, extract_final_records

MAIN = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
LAYOUT = (
    FinalRunLayout(10, 10, 11, 12, 13),
    FinalRunLayout(11, 20, 21, 22, 23),
)


def row(number, values, formula_column=None):
    cells = []
    for index, value in enumerate(values, 1):
        column = chr(ord("A") + index - 1)
        formula = "<f>A1</f>" if formula_column == index else ""
        cells.append(
            f'<c r="{column}{number}" t="inlineStr">{formula}<is><t>{value}</t></is></c>'
        )
    return f'<row r="{number}">{"".join(cells)}</row>'


def fixture(rows):
    temporary = tempfile.TemporaryDirectory()
    path = Path(temporary.name) / "fixture.xlsx"
    xml = f'<worksheet xmlns="{MAIN}"><sheetData>{"".join(rows)}</sheetData></worksheet>'
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr("xl/worksheets/sheet.xml", xml)
    return temporary, zipfile.ZipFile(path)


def valid_rows():
    rows = []
    for run in LAYOUT:
        rows.append(row(run.annotation_row, [ANNOTATION_TEXT]))
        rows.append(row(run.header_row, CHANNELS))
        for number in range(run.first_data_row, run.last_data_row + 1):
            rows.append(row(number, [f"{run.trial}:{number}:{index}" for index in range(12)]))
    return rows


class FinalSealTests(unittest.TestCase):
    def extract(self, rows):
        temporary, archive = fixture(rows)
        try:
            return extract_final_records(archive, "xl/worksheets/sheet.xml", [], LAYOUT)
        finally:
            archive.close()
            temporary.cleanup()

    def test_both_runs_and_lexical_values_are_retained(self):
        records, counts, annotations = self.extract(valid_rows())
        self.assertEqual(counts, {10: 2, 11: 2})
        self.assertEqual(len(records), 4)
        self.assertEqual(records[0]["values"]["time"], "10:12:0")
        self.assertEqual(records[-1]["run_id"], "prbs9_motor_a_trial_11")
        self.assertEqual(annotations[10]["text"], ANNOTATION_TEXT)

    def test_missing_cell_is_rejected(self):
        rows = valid_rows()
        rows[2] = row(12, ["1"] * 11)
        with self.assertRaisesRegex(ValueError, "complete 12-cell"):
            self.extract(rows)

    def test_formula_is_rejected(self):
        rows = valid_rows()
        rows[2] = row(12, ["1"] * 12, formula_column=3)
        with self.assertRaisesRegex(ValueError, "formula cells"):
            self.extract(rows)

    def test_extra_populated_row_is_rejected(self):
        rows = valid_rows() + [row(24, ["unexpected"])]
        with self.assertRaisesRegex(ValueError, "after frozen final"):
            self.extract(rows)


if __name__ == "__main__":
    unittest.main()
