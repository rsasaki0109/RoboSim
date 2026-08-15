#!/usr/bin/env python3
"""Freeze and verify the installed rne_py public call shape."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import stat
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Optional

CONTRACT_KIND = "rne_python_api_contract"
CONTRACT_SCHEMA_VERSION = 1
REPORT_KIND = "rne_python_api_report"
REPORT_SCHEMA_VERSION = 1
MAX_FIXTURE_BYTES = 256 * 1024
MAX_DETAIL_CHARS = 240


def _text_signature(value: object) -> Optional[str]:
    signature = getattr(value, "__text_signature__", None)
    if signature is not None and not isinstance(signature, str):
        raise TypeError("__text_signature__ must be a string or null")
    return signature


def _method_contract(owner: type, name: str) -> dict[str, Any]:
    return {
        "name": name,
        "text_signature": _text_signature(getattr(owner, name)),
    }


def public_contract(module: ModuleType) -> dict[str, Any]:
    """Return the deterministic public export, method, and property contract."""
    exports: list[dict[str, Any]] = []
    for name in sorted(item for item in dir(module) if not item.startswith("_")):
        value = getattr(module, name)
        if inspect.ismodule(value):
            continue
        if inspect.isclass(value):
            methods: list[dict[str, Any]] = []
            properties: list[str] = []
            for member_name in sorted(
                item for item in dir(value) if not item.startswith("_")
            ):
                static_member = inspect.getattr_static(value, member_name)
                if callable(static_member):
                    methods.append(_method_contract(value, member_name))
                elif inspect.isdatadescriptor(static_member):
                    properties.append(member_name)
            exports.append(
                {
                    "name": name,
                    "kind": "class",
                    "text_signature": _text_signature(value),
                    "value": None,
                    "methods": methods,
                    "properties": properties,
                }
            )
        elif callable(value):
            exports.append(
                {
                    "name": name,
                    "kind": "function",
                    "text_signature": _text_signature(value),
                    "value": None,
                    "methods": [],
                    "properties": [],
                }
            )
        elif isinstance(value, (str, int, float, bool)) or value is None:
            exports.append(
                {
                    "name": name,
                    "kind": "constant",
                    "text_signature": None,
                    "value": value,
                    "methods": [],
                    "properties": [],
                }
            )
        else:
            raise TypeError(
                f"unsupported public export {name!r} with type {type(value).__name__}"
            )
    release_version = getattr(module, "__version__", None)
    if not isinstance(release_version, str) or not release_version:
        raise ValueError("rne_py.__version__ must be a non-empty string")
    contract = {
        "kind": CONTRACT_KIND,
        "schema_version": CONTRACT_SCHEMA_VERSION,
        "module": module.__name__,
        "release_version": release_version,
        "exports": exports,
    }
    validate_contract(contract)
    return contract


def _exact_keys(value: dict[str, Any], expected: set[str], path: str) -> None:
    actual = set(value)
    if actual != expected:
        raise ValueError(
            f"{path} keys differ: missing={sorted(expected - actual)} "
            f"unknown={sorted(actual - expected)}"
        )


def _sorted_unique(names: list[str], path: str) -> None:
    if names != sorted(names) or len(names) != len(set(names)):
        raise ValueError(f"{path} must be sorted and unique")


def validate_contract(contract: object) -> None:
    """Strictly validate the Python API contract schema and ordering."""
    if not isinstance(contract, dict):
        raise ValueError("Python API contract must be an object")
    _exact_keys(
        contract,
        {"kind", "schema_version", "module", "release_version", "exports"},
        "contract",
    )
    if contract["kind"] != CONTRACT_KIND:
        raise ValueError(f"contract kind must be {CONTRACT_KIND!r}")
    if contract["schema_version"] != CONTRACT_SCHEMA_VERSION:
        raise ValueError(
            f"contract schema must be {CONTRACT_SCHEMA_VERSION}, "
            f"got {contract['schema_version']!r}"
        )
    if contract["module"] != "rne_py":
        raise ValueError("contract module must be 'rne_py'")
    if not isinstance(contract["release_version"], str) or not contract[
        "release_version"
    ]:
        raise ValueError("contract release_version must be a non-empty string")
    exports = contract["exports"]
    if not isinstance(exports, list) or not exports:
        raise ValueError("contract exports must be a non-empty list")
    export_names: list[str] = []
    for index, export in enumerate(exports):
        path = f"exports[{index}]"
        if not isinstance(export, dict):
            raise ValueError(f"{path} must be an object")
        _exact_keys(
            export,
            {"name", "kind", "text_signature", "value", "methods", "properties"},
            path,
        )
        name = export["name"]
        if not isinstance(name, str) or not name or name.startswith("_"):
            raise ValueError(f"{path}.name must be public and non-empty")
        export_names.append(name)
        kind = export["kind"]
        if kind not in {"class", "function", "constant"}:
            raise ValueError(f"{path}.kind is unsupported")
        signature = export["text_signature"]
        if signature is not None and not isinstance(signature, str):
            raise ValueError(f"{path}.text_signature must be a string or null")
        methods = export["methods"]
        properties = export["properties"]
        if not isinstance(methods, list) or not isinstance(properties, list):
            raise ValueError(f"{path} methods/properties must be lists")
        method_names: list[str] = []
        for method_index, method in enumerate(methods):
            method_path = f"{path}.methods[{method_index}]"
            if not isinstance(method, dict):
                raise ValueError(f"{method_path} must be an object")
            _exact_keys(method, {"name", "text_signature"}, method_path)
            method_name = method["name"]
            if not isinstance(method_name, str) or not method_name:
                raise ValueError(f"{method_path}.name must be non-empty")
            method_signature = method["text_signature"]
            if method_signature is not None and not isinstance(method_signature, str):
                raise ValueError(
                    f"{method_path}.text_signature must be a string or null"
                )
            method_names.append(method_name)
        _sorted_unique(method_names, f"{path}.methods")
        if not all(isinstance(item, str) and item for item in properties):
            raise ValueError(f"{path}.properties must contain non-empty strings")
        _sorted_unique(properties, f"{path}.properties")
        if kind == "class":
            if export["value"] is not None:
                raise ValueError(f"{path}.value must be null for a class")
        elif kind == "function":
            if export["value"] is not None or methods or properties:
                raise ValueError(f"{path} function-only fields are invalid")
        elif signature is not None or methods or properties:
            raise ValueError(f"{path} constant-only fields are invalid")
        try:
            json.dumps(export["value"], allow_nan=False)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{path}.value is not canonical JSON: {error}") from error
    _sorted_unique(export_names, "exports")


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def read_contract(path: Path) -> dict[str, Any]:
    """Read one bounded, regular, duplicate-free UTF-8 contract file."""
    metadata = path.lstat()
    if not stat.S_ISREG(metadata.st_mode):
        raise ValueError("Python API fixture must be a regular file")
    if metadata.st_size > MAX_FIXTURE_BYTES:
        raise ValueError(f"Python API fixture exceeds {MAX_FIXTURE_BYTES} bytes")
    contract = json.loads(
        path.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_keys
    )
    validate_contract(contract)
    return contract


def canonical_bytes(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def digest(value: object) -> str:
    return "sha256:" + hashlib.sha256(canonical_bytes(value)).hexdigest()


def first_difference(expected: object, actual: object, path: str = "contract") -> str:
    if type(expected) is not type(actual):
        return f"{path} type differs"
    if isinstance(expected, dict):
        if set(expected) != set(actual):
            return f"{path} keys differ"
        for key in expected:
            if expected[key] != actual[key]:
                return first_difference(expected[key], actual[key], f"{path}.{key}")
    elif isinstance(expected, list):
        if len(expected) != len(actual):
            return f"{path} length differs: expected {len(expected)}, got {len(actual)}"
        for index, (expected_item, actual_item) in enumerate(zip(expected, actual)):
            if expected_item != actual_item:
                return first_difference(
                    expected_item, actual_item, f"{path}[{index}]"
                )
    elif expected != actual:
        return f"{path} differs: expected {expected!r}, got {actual!r}"
    return f"{path} differs"


def write_json(path: Path, value: object, *, refuse_overwrite: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if refuse_overwrite and path.exists():
        raise FileExistsError(f"refusing to overwrite {path}")
    text = json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(text)


def verify(fixture_path: Path, output_path: Path, module: ModuleType) -> bool:
    expected = read_contract(fixture_path)
    actual = public_contract(module)
    passed = expected == actual
    detail = (
        "exact public Python API contract matched"
        if passed
        else first_difference(expected, actual)
    )
    detail = " ".join(detail.split())[:MAX_DETAIL_CHARS]
    report = {
        "kind": REPORT_KIND,
        "schema_version": REPORT_SCHEMA_VERSION,
        "module": "rne_py",
        "release_version": actual["release_version"],
        "fixture_sha256": digest(expected),
        "actual_sha256": digest(actual),
        "export_count": len(actual["exports"]),
        "passed": passed,
        "detail": detail,
    }
    write_json(output_path, report)
    print(
        f"Python API compatibility: status={'passed' if passed else 'failed'} "
        f"exports={len(actual['exports'])} report={output_path}"
    )
    if not passed:
        print(detail, file=sys.stderr)
    return passed


def main() -> int:
    parser = argparse.ArgumentParser()
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--write-fixture", type=Path)
    action.add_argument("--fixture", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    import rne_py

    if args.write_fixture is not None:
        if args.output is not None:
            parser.error("--output is not used with --write-fixture")
        write_json(
            args.write_fixture,
            public_contract(rne_py),
            refuse_overwrite=True,
        )
        print(f"wrote Python API fixture {args.write_fixture}")
        return 0
    if args.output is None:
        parser.error("--fixture requires --output")
    return 0 if verify(args.fixture, args.output, rne_py) else 1


if __name__ == "__main__":
    raise SystemExit(main())
