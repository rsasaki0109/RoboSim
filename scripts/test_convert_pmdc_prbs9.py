"""Synthetic contract tests for lossless pre-final PMDC conversion."""

import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

from convert_pmdc_prbs9 import ANNOTATION_TEXT, CHANNELS, extract_partition

MAIN = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"


def cell(column, row, value, formula=None):
    tag = f'<c r="{column}{row}" t="inlineStr">'
    if formula is not None:
        tag += f"<f>{formula}</f>"
    return tag + f"<is><t>{value}</t></is></c>"


def row_xml(number, values, formula_column=None):
    cells = []
    for index, value in enumerate(values):
        column = chr(ord("A") + index)
        formula = "A1" if formula_column == index + 1 else None
        cells.append(cell(column, number, value, formula))
    return f'<row r="{number}">{"".join(cells)}</row>'


def workbook_xml(rows):
    return f'<worksheet xmlns="{MAIN}"><sheetData>{"".join(rows)}</sheetData></worksheet>'


class ConvertPmdcTests(unittest.TestCase):
    def open_fixture(self, rows):
        temporary = tempfile.TemporaryDirectory()
        path = Path(temporary.name) / "fixture.xlsx"
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("xl/worksheets/sheet.xml", workbook_xml(rows))
        return temporary, zipfile.ZipFile(path)

    def test_training_preserves_lexical_cells_and_source_identity(self):
        rows = []
        header_rows = (2, 2013, 4024, 6035, 8046, 10057, 12068, 14079)
        for trial, header_row in enumerate(header_rows, 1):
            rows.append(row_xml(header_row - 1, [ANNOTATION_TEXT]))
            rows.append(row_xml(header_row, CHANNELS))
            rows.append(row_xml(header_row + 1, [f"{trial}:{index}:01.00" for index in range(12)]))
        temporary, archive = self.open_fixture(rows)
        try:
            records, counts, annotations = extract_partition(
                archive, "xl/worksheets/sheet.xml", [], "training"
            )
        finally:
            archive.close()
            temporary.cleanup()
        self.assertEqual(len(records), 8)
        self.assertEqual(counts, {trial: 1 for trial in range(1, 9)})
        self.assertEqual(annotations[1]["source_row"], 1)
        self.assertEqual(records[0]["source_row"], 3)
        self.assertEqual(records[0]["values"]["time"], "1:0:01.00")
        self.assertEqual(records[-1]["run_id"], "prbs9_motor_a_trial_08")

    def test_development_stops_before_first_final_header(self):
        rows = []
        for header_row in (2, 2013, 4024, 6035, 8046, 10057, 12068, 14079):
            rows.append(row_xml(header_row - 1, [ANNOTATION_TEXT]))
            rows.append(row_xml(header_row, CHANNELS))
        rows.extend([
            row_xml(16089, [ANNOTATION_TEXT]),
            row_xml(16090, CHANNELS),
            row_xml(16091, ["dev"] * 12),
            row_xml(18101, ["not", "a", "valid", "final", "header"]),
            row_xml(18102, ["sealed-value"] * 12),
        ])
        temporary, archive = self.open_fixture(rows)
        try:
            records, counts, annotations = extract_partition(
                archive, "xl/worksheets/sheet.xml", [], "development"
            )
        finally:
            archive.close()
            temporary.cleanup()
        self.assertEqual(counts, {9: 1})
        self.assertEqual(annotations[9]["source_row"], 16089)
        self.assertEqual(records[0]["values"]["time"], "dev")

    def test_final_partition_has_no_authorized_selector(self):
        with self.assertRaisesRegex(ValueError, "only frozen"):
            extract_partition(None, "unused", [], "final")

    def test_formula_in_selected_data_is_rejected(self):
        rows = [row_xml(1, [ANNOTATION_TEXT]), row_xml(2, CHANNELS), row_xml(3, ["1"] * 12, formula_column=3)]
        for header_row in (2013, 4024, 6035, 8046, 10057, 12068, 14079):
            rows.append(row_xml(header_row - 1, [ANNOTATION_TEXT]))
            rows.append(row_xml(header_row, CHANNELS))
            rows.append(row_xml(header_row + 1, ["1"] * 12))
        temporary, archive = self.open_fixture(rows)
        try:
            with self.assertRaisesRegex(ValueError, "formula cells"):
                extract_partition(archive, "xl/worksheets/sheet.xml", [], "training")
        finally:
            archive.close()
            temporary.cleanup()

    def test_missing_source_cell_is_rejected(self):
        rows = []
        for header_row in (2, 2013, 4024, 6035, 8046, 10057, 12068, 14079):
            rows.append(row_xml(header_row - 1, [ANNOTATION_TEXT]))
            rows.append(row_xml(header_row, CHANNELS))
            values = ["1"] * (11 if header_row == 2 else 12)
            rows.append(row_xml(header_row + 1, values))
        temporary, archive = self.open_fixture(rows)
        try:
            with self.assertRaisesRegex(ValueError, "complete 12-cell"):
                extract_partition(archive, "xl/worksheets/sheet.xml", [], "training")
        finally:
            archive.close()
            temporary.cleanup()


if __name__ == "__main__":
    unittest.main()
