"""Bounded, read-only audit of the pinned PMDC geared-motor XLSX source.

The workbook is never extracted or modified.  This script verifies the immutable
source bytes before parsing OOXML, bounds both compressed and expanded input, and
reports trial boundaries and formula cells without interpreting response values.
It does not qualify calibration or identify a motor model.
"""

import argparse
import hashlib
import json
import re
import stat
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path, PurePosixPath

SOURCE_BYTES = 28_724_353
SOURCE_SHA256 = "85203c4b3ad6fbdd05221e1be7fd41ce733376c0f316d7fd5542b604a6854605"
MAX_SOURCE_BYTES = 32 * 1024 * 1024
MAX_ZIP_MEMBERS = 64
MAX_MEMBER_BYTES = 64 * 1024 * 1024
MAX_EXPANDED_BYTES = 256 * 1024 * 1024

XMLNS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
RELNS = "http://schemas.openxmlformats.org/package/2006/relationships"
OFFICE_RELNS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
CELL_REF = re.compile(r"^([A-Z]+)([1-9][0-9]*)$")

EXPECTED_SHEETS = (
    "Description",
    "PRBS7-MotorA", "PRBS7-MotorB", "PRBS9-MotorA", "PRBS9-MotorB",
    "Sine10Hz-MotorA", "Sine10Hz-MotorB", "Sine1Hz-MotorA", "Sine1Hz-MotorB",
    "Sine0.1Hz-MotorA", "Sine0.1Hz-MotorB",
    "Triangle10Hz-MotorA", "Triangle10Hz-MotorB",
    "Triangle1Hz-MotorA", "Triangle1Hz-MotorB",
    "Triangle0.1Hz-MotorA", "Triangle0.1Hz-MotorB",
    "SquareWave-MotorA", "SquareWave-MotorB",
    "StepNoLoad-MotorA", "StepNoLoad-MotorB",
    "StepLoaded-MotorA", "StepLoaded-MotorB",
)
EXPECTED_TRIAL_COUNTS = {
    "PRBS9-MotorA": 11,
    "StepNoLoad-MotorA": 122,
    "StepNoLoad-MotorB": 152,
    "StepLoaded-MotorA": 283,
    "StepLoaded-MotorB": 315,
}
EXPECTED_FORMULAS = (("StepNoLoad-MotorA", "C256", "(A256-B256)/1000000"),)


def _stream_digest(path):
    digest = hashlib.sha256()
    count = 0
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            count += len(block)
            if count > MAX_SOURCE_BYTES:
                raise ValueError("source exceeds the 32 MiB compressed-input cap")
            digest.update(block)
    return count, digest.hexdigest()


def verify_source(path):
    """Verify the exact published workbook size and digest before OOXML parsing."""
    if not path.is_file():
        raise ValueError("PMDC source is not a regular file")
    size, digest = _stream_digest(path)
    if size != SOURCE_BYTES:
        raise ValueError("PMDC source size mismatch")
    if digest != SOURCE_SHA256:
        raise ValueError("PMDC source hash mismatch; refusing unqualified input")
    return size, digest


def _safe_member_name(name):
    if "\\" in name or ":" in name:
        return False
    path = PurePosixPath(name)
    return not path.is_absolute() and all(part not in ("", ".", "..") for part in path.parts)


def inspect_archive(archive):
    """Reject unsafe or excessively expanded ZIP containers before XML parsing."""
    infos = archive.infolist()
    if not infos or len(infos) > MAX_ZIP_MEMBERS:
        raise ValueError("XLSX ZIP member count is outside the bounded contract")
    if len({info.filename for info in infos}) != len(infos):
        raise ValueError("XLSX contains duplicate ZIP member names")
    expanded = 0
    for info in infos:
        if not _safe_member_name(info.filename):
            raise ValueError("XLSX contains an unsafe ZIP member name")
        mode = info.external_attr >> 16
        if stat.S_ISLNK(mode):
            raise ValueError("XLSX contains a symbolic-link ZIP member")
        if info.flag_bits & 1:
            raise ValueError("encrypted XLSX members are unsupported")
        if info.file_size > MAX_MEMBER_BYTES:
            raise ValueError("XLSX member exceeds the expanded-member cap")
        expanded += info.file_size
        if expanded > MAX_EXPANDED_BYTES:
            raise ValueError("XLSX exceeds the total expanded-input cap")
    return {"member_count": len(infos), "expanded_bytes": expanded}


def _open_xml(archive, name):
    stream = archive.open(name)
    prefix = stream.read(4096)
    if b"<!DOCTYPE" in prefix.upper() or b"<!ENTITY" in prefix.upper():
        stream.close()
        raise ValueError(f"XML declarations with entities are forbidden: {name}")
    stream.seek(0)
    return stream


def _read_xml(archive, name):
    with _open_xml(archive, name) as stream:
        return ET.parse(stream).getroot()


def _resolve_workbook_target(target):
    path = PurePosixPath(target)
    if path.is_absolute() or ".." in path.parts or "\\" in target or ":" in target:
        raise ValueError("unsafe workbook relationship target")
    if path.parts and path.parts[0] == "xl":
        resolved = path
    else:
        resolved = PurePosixPath("xl") / path
    return resolved.as_posix()


def workbook_sheets(archive):
    """Return workbook sheet names and local worksheet members in source order."""
    workbook = _read_xml(archive, "xl/workbook.xml")
    relationships = _read_xml(archive, "xl/_rels/workbook.xml.rels")
    targets = {}
    for relationship in relationships.findall(f"{{{RELNS}}}Relationship"):
        if relationship.get("TargetMode") == "External":
            continue
        targets[relationship.get("Id")] = _resolve_workbook_target(relationship.get("Target", ""))
    sheets = []
    sheet_parent = workbook.find(f"{{{XMLNS}}}sheets")
    if sheet_parent is None:
        raise ValueError("workbook has no sheets")
    for sheet in sheet_parent.findall(f"{{{XMLNS}}}sheet"):
        rel_id = sheet.get(f"{{{OFFICE_RELNS}}}id")
        target = targets.get(rel_id)
        if target is None or not target.startswith("xl/worksheets/"):
            raise ValueError("sheet does not resolve to a local worksheet")
        sheets.append((sheet.get("name", ""), target))
    return sheets


def shared_strings(archive):
    """Load the bounded shared-string table, preserving rich-text fragments."""
    if "xl/sharedStrings.xml" not in archive.namelist():
        return []
    values = []
    with _open_xml(archive, "xl/sharedStrings.xml") as stream:
        for event, element in ET.iterparse(stream, events=("end",)):
            if element.tag == f"{{{XMLNS}}}si":
                values.append("".join(node.text or "" for node in element.iter(f"{{{XMLNS}}}t")))
                element.clear()
    return values


def _column_number(reference):
    match = CELL_REF.fullmatch(reference)
    if match is None:
        raise ValueError(f"invalid cell reference: {reference}")
    number = 0
    for char in match.group(1):
        number = number * 26 + ord(char) - ord("A") + 1
    return number


def _cell_text(cell, strings):
    cell_type = cell.get("t")
    value = cell.find(f"{{{XMLNS}}}v")
    if cell_type == "s":
        if value is None or value.text is None:
            raise ValueError("shared-string cell has no index")
        index = int(value.text)
        if not 0 <= index < len(strings):
            raise ValueError("shared-string index is outside the table")
        return strings[index]
    if cell_type == "inlineStr":
        return "".join(node.text or "" for node in cell.iter(f"{{{XMLNS}}}t"))
    return "" if value is None or value.text is None else value.text


def scan_worksheet(archive, member, strings):
    """Scan one worksheet for complete-run headers and formulas without cell repair."""
    trial_headers = []
    formulas = []
    row_count = 0
    cell_count = 0
    with _open_xml(archive, member) as stream:
        for event, element in ET.iterparse(stream, events=("end",)):
            if element.tag == f"{{{XMLNS}}}row":
                row_count += 1
                cells = {}
                for cell in element.findall(f"{{{XMLNS}}}c"):
                    reference = cell.get("r", "")
                    column = _column_number(reference)
                    text = _cell_text(cell, strings)
                    cells[column] = (reference, text)
                    formula = cell.find(f"{{{XMLNS}}}f")
                    if formula is not None:
                        formulas.append({
                            "cell": reference,
                            "expression": formula.text or "",
                            "cached_value": text,
                        })
                cell_count += len(cells)
                for column in sorted(cells):
                    reference, text = cells[column]
                    if text.strip().casefold() == "time":
                        header = [cells.get(index, (None, None))[1] for index in range(column, column + 12)]
                        if any(value is None for value in header):
                            raise ValueError(f"incomplete 12-channel trial header at {reference}")
                        trial_headers.append({"header_cell": reference, "channels": header})
                element.clear()
    return {
        "row_count": row_count,
        "cell_count": cell_count,
        "trial_count": len(trial_headers),
        "trials": trial_headers,
        "formulas": formulas,
    }


def audit(path):
    """Audit the pinned workbook and return deterministic structural evidence."""
    source_bytes, source_sha256 = verify_source(path)
    with zipfile.ZipFile(path) as archive:
        container = inspect_archive(archive)
        sheets = workbook_sheets(archive)
        names = tuple(name for name, _ in sheets)
        if names != EXPECTED_SHEETS:
            raise ValueError("workbook sheet set or order differs from the pinned contract")
        strings = shared_strings(archive)
        reports = {}
        formulas = []
        for name, member in sheets:
            report = scan_worksheet(archive, member, strings)
            reports[name] = report
            formulas.extend((name, item["cell"], item["expression"]) for item in report["formulas"])

    for name, expected in EXPECTED_TRIAL_COUNTS.items():
        if reports[name]["trial_count"] != expected:
            raise ValueError(f"{name} trial count differs from the frozen source audit")
    if tuple(formulas) != EXPECTED_FORMULAS:
        raise ValueError("formula cells differ from the frozen source audit")

    prbs9_trials = reports["PRBS9-MotorA"]["trials"]
    split = {
        "training": [item["header_cell"] for item in prbs9_trials[:8]],
        "development": [prbs9_trials[8]["header_cell"]],
        "final_untouched": [item["header_cell"] for item in prbs9_trials[9:]],
    }
    report = {
        "schema_version": 1,
        "kind": "rne_pmdc_source_audit",
        "source_sha256": source_sha256,
        "source_bytes": source_bytes,
        "zip": container,
        "sheets": reports,
        "frozen_prbs9_motor_a_split": split,
        "source_values_inspected_for_split": False,
        "source_rows_repaired": False,
        "physical_accuracy_validated": False,
        "voltage_calibration_qualified": False,
        "current_calibration_qualified": False,
        "clock_mapping_qualified": False,
        "motor_electrical_identification_qualified": False,
    }
    canonical = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
    report["audit_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workbook", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        help="write JSON with exclusive creation instead of printing it",
    )
    args = parser.parse_args()
    rendered = json.dumps(audit(args.workbook), indent=2, sort_keys=True, allow_nan=False)
    if args.output is None:
        print(rendered)
    else:
        with args.output.open("x", encoding="utf-8", newline="\n") as output:
            output.write(rendered)
            output.write("\n")
