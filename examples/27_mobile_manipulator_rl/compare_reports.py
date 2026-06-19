"""Compare one or more RNE mobile-manipulator report bundles.

Each report is produced by:

    python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --report-dir reports/reach

This script reads each bundle's manifest.json, prints a Markdown leaderboard, can write
standalone HTML/CSV/JSON leaderboards, can copy the best policy artifact, can write a
machine-readable best-report summary, can copy the best house GIF artifact with
metadata, and can enforce CI-friendly quality gates.
"""

import argparse
import csv
import hashlib
import html
import json
import math
import os
import sys

from policy_schema import (
    POLICY_ARTIFACT_REQUIRED_FIELDS,
    POLICY_ARTIFACT_VERSION,
    stable_hash,
)
from rollout_schema import (
    ROLLOUT_CSV_FIELDS,
    ROLLOUT_CSV_SCHEMA_HASH,
    ROLLOUT_CSV_SCHEMA_VERSION,
    ROLLOUT_NUMERIC_FIELDS,
)


REQUIRED_MANIFEST_FIELDS = (
    "schema_version",
    "policy_algorithm",
    "policy_schema_version",
    "observation_schema_hash",
    "action_schema_hash",
    "rollout_schema_version",
    "rollout_schema_hash",
    "best_reward",
    "training_iterations",
    "rollout_rows",
    "final_total_reward",
    "final_target_error",
    "artifacts",
)
REQUIRED_ARTIFACT_KEYS = ("index", "policy", "rollout_csv")
REQUIRED_POLICY_FIELDS = POLICY_ARTIFACT_REQUIRED_FIELDS
REQUIRED_ROLLOUT_CSV_FIELDS = ROLLOUT_CSV_FIELDS

REPORT_MANIFEST_SCHEMA_VERSION = 2
POLICY_ARTIFACT_SCHEMA_VERSION = POLICY_ARTIFACT_VERSION
LEADERBOARD_SCHEMA_VERSION = 1
BEST_REPORT_SCHEMA_VERSION = 1
HOUSE_GIF_METADATA_VERSION = 1
METRIC_TOLERANCE = 1e-9

RANKING_CRITERIA = (
    {"field": "final_target_error", "order": "ascending"},
    {"field": "final_total_reward", "order": "descending"},
    {"field": "best_reward", "order": "descending"},
    {"field": "report", "order": "ascending"},
)


def _find_manifest_paths(paths):
    manifest_paths = []
    for path in paths:
        if os.path.isfile(path):
            if os.path.basename(path) == "manifest.json":
                manifest_paths.append(path)
            continue
        if not os.path.isdir(path):
            raise ValueError(f"report path does not exist: {path}")

        direct_manifest = os.path.join(path, "manifest.json")
        if os.path.isfile(direct_manifest):
            manifest_paths.append(direct_manifest)
            continue

        for root, _, files in os.walk(path):
            if "manifest.json" in files:
                manifest_paths.append(os.path.join(root, "manifest.json"))

    return sorted(set(os.path.abspath(path) for path in manifest_paths))


def _load_manifest(path):
    with open(path, "r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    missing = [field for field in REQUIRED_MANIFEST_FIELDS if field not in manifest]
    if missing:
        raise ValueError(f"{path}: manifest missing fields: {', '.join(missing)}")
    if manifest.get("schema_version") != REPORT_MANIFEST_SCHEMA_VERSION:
        raise ValueError(
            f"{path}: unsupported manifest schema_version: "
            f"{manifest.get('schema_version')!r}"
        )
    if manifest.get("policy_schema_version") != POLICY_ARTIFACT_SCHEMA_VERSION:
        raise ValueError(
            f"{path}: unsupported policy_schema_version: "
            f"{manifest.get('policy_schema_version')!r}"
        )
    if manifest.get("rollout_schema_version") != ROLLOUT_CSV_SCHEMA_VERSION:
        raise ValueError(
            f"{path}: unsupported rollout_schema_version: "
            f"{manifest.get('rollout_schema_version')!r}"
        )
    if manifest.get("rollout_schema_hash") != ROLLOUT_CSV_SCHEMA_HASH:
        raise ValueError(
            f"{path}: rollout_schema_hash mismatch: "
            f"{manifest.get('rollout_schema_hash')!r} != {ROLLOUT_CSV_SCHEMA_HASH!r}"
        )
    if not isinstance(manifest["artifacts"], dict):
        raise ValueError(f"{path}: manifest artifacts must be an object")
    report_dir = os.path.dirname(path)
    artifacts = manifest["artifacts"]
    resolved_artifacts = _resolve_artifacts(path, report_dir, artifacts)
    _validate_policy_artifact(path, resolved_artifacts["policy"], manifest)
    _validate_rollout_csv_artifact(path, resolved_artifacts["rollout_csv"], manifest)
    house_gif_info = None
    if "rollout_house_gif" in resolved_artifacts:
        house_gif_info = _validate_house_gif_artifact(
            path, resolved_artifacts["rollout_house_gif"], manifest
        )
    if "rollout_house_gif_metadata" in resolved_artifacts:
        if house_gif_info is None:
            raise ValueError(
                f"{path}: rollout_house_gif_metadata requires rollout_house_gif"
            )
        _validate_house_gif_metadata_artifact(
            path,
            resolved_artifacts["rollout_house_gif_metadata"],
            resolved_artifacts["rollout_house_gif"],
            resolved_artifacts["rollout_csv"],
            house_gif_info,
        )
    return {
        "name": os.path.basename(report_dir),
        "report_dir": report_dir,
        "manifest_path": path,
        "index_path": resolved_artifacts["index"],
        "policy_path": resolved_artifacts["policy"],
        "rollout_house_gif_path": resolved_artifacts.get("rollout_house_gif"),
        "rollout_house_gif_metadata_path": resolved_artifacts.get(
            "rollout_house_gif_metadata"
        ),
        "rollout_house_gif_info": house_gif_info,
        "policy_algorithm": manifest["policy_algorithm"],
        "policy_schema_version": int(manifest["policy_schema_version"]),
        "observation_schema_hash": manifest["observation_schema_hash"],
        "action_schema_hash": manifest["action_schema_hash"],
        "rollout_schema_version": int(manifest["rollout_schema_version"]),
        "rollout_schema_hash": manifest["rollout_schema_hash"],
        "best_reward": float(manifest["best_reward"]),
        "training_iterations": int(manifest["training_iterations"]),
        "rollout_rows": int(manifest["rollout_rows"]),
        "final_total_reward": float(manifest["final_total_reward"]),
        "final_target_error": float(manifest["final_target_error"]),
    }


def _resolve_artifacts(manifest_path, report_dir, artifacts):
    missing = [key for key in REQUIRED_ARTIFACT_KEYS if key not in artifacts]
    if missing:
        raise ValueError(
            f"{manifest_path}: manifest artifacts missing keys: {', '.join(missing)}"
        )

    report_root = os.path.abspath(report_dir)
    resolved = {}
    for key, value in artifacts.items():
        if not isinstance(value, str) or not value:
            raise ValueError(
                f"{manifest_path}: manifest artifact {key!r} must be a non-empty string"
            )
        if os.path.isabs(value):
            raise ValueError(
                f"{manifest_path}: manifest artifact {key!r} must be relative: {value}"
            )

        artifact_path = os.path.abspath(os.path.join(report_dir, value))
        try:
            is_inside_report = os.path.commonpath([report_root, artifact_path]) == report_root
        except ValueError:
            is_inside_report = False
        if not is_inside_report:
            raise ValueError(
                f"{manifest_path}: manifest artifact {key!r} escapes report directory: {value}"
            )
        if not os.path.isfile(artifact_path):
            raise ValueError(
                f"{manifest_path}: manifest artifact {key!r} does not exist: {value}"
            )
        resolved[key] = artifact_path
    return resolved


def _assert_close(manifest_path, field, actual, expected):
    if not math.isclose(
        actual, expected, rel_tol=METRIC_TOLERANCE, abs_tol=METRIC_TOLERANCE
    ):
        raise ValueError(
            f"{manifest_path}: {field} mismatch: {actual!r} != {expected!r}"
        )


def _float_from_mapping(mapping, key, *, context):
    try:
        return float(mapping[key])
    except KeyError as error:
        raise ValueError(f"{context}: missing field: {key}") from error
    except (TypeError, ValueError) as error:
        raise ValueError(f"{context}: field {key!r} must be numeric") from error


def _int_from_mapping(mapping, key, *, context):
    value = _float_from_mapping(mapping, key, context=context)
    if not value.is_integer():
        raise ValueError(f"{context}: field {key!r} must be an integer")
    return int(value)


def _bool_from_mapping(mapping, key, *, context):
    try:
        value = mapping[key].strip().lower()
    except KeyError as error:
        raise ValueError(f"{context}: missing field: {key}") from error
    except AttributeError as error:
        raise ValueError(f"{context}: field {key!r} must be boolean text") from error
    if value not in ("true", "false"):
        raise ValueError(f"{context}: field {key!r} must be true or false")
    return value == "true"


def _skip_gif_subblocks(data, offset, *, context):
    while offset < len(data):
        size = data[offset]
        offset += 1
        if size == 0:
            return offset
        offset += size
        if offset > len(data):
            raise ValueError(f"{context}: GIF sub-block exceeds file length")
    raise ValueError(f"{context}: GIF sub-block terminator missing")


def _positive_int_from_mapping(mapping, key, *, context):
    value = _int_from_mapping(mapping, key, context=context)
    if value <= 0:
        raise ValueError(f"{context}: field {key!r} must be positive")
    return value


def _positive_float_from_mapping(mapping, key, *, context):
    value = _float_from_mapping(mapping, key, context=context)
    if value <= 0.0:
        raise ValueError(f"{context}: field {key!r} must be positive")
    return value


def _validate_house_gif_artifact(manifest_path, gif_path, manifest):
    context = f"{manifest_path}: rollout_house_gif"
    with open(gif_path, "rb") as handle:
        data = handle.read()

    if len(data) < 14:
        raise ValueError(f"{context}: GIF artifact is too small")
    if data[:6] not in (b"GIF87a", b"GIF89a"):
        raise ValueError(f"{context}: GIF header mismatch")
    width = int.from_bytes(data[6:8], "little")
    height = int.from_bytes(data[8:10], "little")
    if width <= 0 or height <= 0:
        raise ValueError(f"{context}: GIF logical screen must be non-empty")

    offset = 13
    packed = data[10]
    if packed & 0x80:
        color_count = 1 << ((packed & 0x07) + 1)
        offset += color_count * 3
    if offset >= len(data):
        raise ValueError(f"{context}: GIF body missing")

    frames = 0
    while offset < len(data):
        marker = data[offset]
        offset += 1
        if marker == 0x3B:
            if offset != len(data):
                raise ValueError(f"{context}: GIF has trailing bytes after trailer")
            break
        if marker == 0x21:
            if offset >= len(data):
                raise ValueError(f"{context}: GIF extension label missing")
            offset += 1
            offset = _skip_gif_subblocks(data, offset, context=context)
            continue
        if marker != 0x2C:
            raise ValueError(f"{context}: GIF block marker mismatch: 0x{marker:02x}")

        if offset + 9 > len(data):
            raise ValueError(f"{context}: GIF image descriptor truncated")
        image_packed = data[offset + 8]
        offset += 9
        if image_packed & 0x80:
            color_count = 1 << ((image_packed & 0x07) + 1)
            offset += color_count * 3
            if offset > len(data):
                raise ValueError(f"{context}: GIF local color table truncated")
        if offset >= len(data):
            raise ValueError(f"{context}: GIF image data missing")
        offset += 1
        offset = _skip_gif_subblocks(data, offset, context=context)
        frames += 1
    else:
        raise ValueError(f"{context}: GIF trailer missing")

    if frames == 0:
        raise ValueError(f"{context}: GIF has no image frames")
    byte_size = len(data)
    sha256 = f"sha256:{hashlib.sha256(data).hexdigest()}"
    info = {
        "width": width,
        "height": height,
        "frame_count": frames,
        "byte_size": byte_size,
        "sha256": sha256,
    }
    manifest_info = manifest.get("rollout_house_gif")
    if manifest_info is None:
        return info
    if not isinstance(manifest_info, dict):
        raise ValueError(f"{context}: manifest metadata must be an object")

    expected_width = _positive_int_from_mapping(manifest_info, "width", context=context)
    expected_height = _positive_int_from_mapping(manifest_info, "height", context=context)
    expected_frame_count = _positive_int_from_mapping(
        manifest_info, "frame_count", context=context
    )
    sample_count = _positive_int_from_mapping(
        manifest_info, "sample_count", context=context
    )
    max_frames = _positive_int_from_mapping(manifest_info, "max_frames", context=context)
    fps = _positive_float_from_mapping(manifest_info, "fps", context=context)
    expected_byte_size = _positive_int_from_mapping(
        manifest_info, "byte_size", context=context
    )
    try:
        expected_sha256 = manifest_info["sha256"]
    except KeyError as error:
        raise ValueError(f"{context}: missing field: sha256") from error
    if not isinstance(expected_sha256, str) or not expected_sha256.startswith("sha256:"):
        raise ValueError(f"{context}: field 'sha256' must be a sha256 digest string")

    if width != expected_width:
        raise ValueError(f"{context}: width mismatch: {width!r} != {expected_width!r}")
    if height != expected_height:
        raise ValueError(f"{context}: height mismatch: {height!r} != {expected_height!r}")
    if frames != expected_frame_count:
        raise ValueError(
            f"{context}: frame_count mismatch: {frames!r} != {expected_frame_count!r}"
        )
    if frames > max_frames:
        raise ValueError(
            f"{context}: frame_count exceeds max_frames: {frames!r} > {max_frames!r}"
        )
    if sample_count != manifest["rollout_rows"]:
        raise ValueError(
            f"{context}: sample_count mismatch: "
            f"{sample_count!r} != {manifest['rollout_rows']!r}"
        )
    if byte_size != expected_byte_size:
        raise ValueError(
            f"{context}: byte_size mismatch: {byte_size!r} != {expected_byte_size!r}"
        )
    if sha256 != expected_sha256:
        raise ValueError(f"{context}: sha256 mismatch: {sha256!r} != {expected_sha256!r}")
    info["sample_count"] = sample_count
    info["max_frames"] = max_frames
    info["fps"] = fps
    return info


def _metadata_path_matches(payload_path, expected_path, metadata_dir):
    if not isinstance(payload_path, str) or not payload_path:
        return False
    if os.path.isabs(payload_path):
        resolved = os.path.abspath(payload_path)
    else:
        resolved = os.path.abspath(os.path.join(metadata_dir, payload_path))
    return resolved == os.path.abspath(expected_path)


def _validate_house_gif_metadata_artifact(
    manifest_path, metadata_path, gif_path, csv_path, house_gif_info
):
    context = f"{manifest_path}: rollout_house_gif_metadata"
    with open(metadata_path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{context}: metadata must be an object")
    if payload.get("schema_version") != HOUSE_GIF_METADATA_VERSION:
        raise ValueError(
            f"{context}: unsupported schema_version: {payload.get('schema_version')!r}"
        )
    if payload.get("artifact") != "rne_mobile_manipulator_house_gif":
        raise ValueError(f"{context}: artifact mismatch: {payload.get('artifact')!r}")

    metadata_dir = os.path.dirname(metadata_path)
    if not _metadata_path_matches(payload.get("gif_path"), gif_path, metadata_dir):
        raise ValueError(f"{context}: gif_path does not point at rollout_house_gif")

    source = payload.get("source")
    if not isinstance(source, dict):
        raise ValueError(f"{context}: source must be an object")
    if source.get("kind") != "rollout_csv":
        raise ValueError(f"{context}: source.kind must be 'rollout_csv'")
    if not _metadata_path_matches(source.get("path"), csv_path, metadata_dir):
        raise ValueError(f"{context}: source.path does not point at rollout_csv")

    for field in (
        "sample_count",
        "frame_count",
        "max_frames",
        "width",
        "height",
        "byte_size",
    ):
        actual = _positive_int_from_mapping(payload, field, context=context)
        expected = house_gif_info[field]
        if actual != expected:
            raise ValueError(
                f"{context}: {field} mismatch: {actual!r} != {expected!r}"
            )
    fps = _positive_float_from_mapping(payload, "fps", context=context)
    if not math.isclose(
        fps, house_gif_info["fps"], rel_tol=METRIC_TOLERANCE, abs_tol=METRIC_TOLERANCE
    ):
        raise ValueError(
            f"{context}: fps mismatch: {fps!r} != {house_gif_info['fps']!r}"
        )
    try:
        sha256 = payload["sha256"]
    except KeyError as error:
        raise ValueError(f"{context}: missing field: sha256") from error
    if sha256 != house_gif_info["sha256"]:
        raise ValueError(
            f"{context}: sha256 mismatch: {sha256!r} != {house_gif_info['sha256']!r}"
        )


def _validate_policy_artifact(manifest_path, policy_path, manifest):
    with open(policy_path, "r", encoding="utf-8") as handle:
        policy = json.load(handle)

    missing = [field for field in REQUIRED_POLICY_FIELDS if field not in policy]
    if missing:
        raise ValueError(
            f"{manifest_path}: policy artifact missing fields: {', '.join(missing)}"
        )
    if policy.get("schema_version") != manifest["policy_schema_version"]:
        raise ValueError(
            f"{manifest_path}: policy schema_version mismatch: "
            f"{policy.get('schema_version')!r} != {manifest['policy_schema_version']!r}"
        )
    if policy.get("schema_version") != POLICY_ARTIFACT_SCHEMA_VERSION:
        raise ValueError(
            f"{manifest_path}: unsupported policy schema_version: "
            f"{policy.get('schema_version')!r}"
        )
    if policy.get("algorithm") != manifest["policy_algorithm"]:
        raise ValueError(
            f"{manifest_path}: policy algorithm mismatch: "
            f"{policy.get('algorithm')!r} != {manifest['policy_algorithm']!r}"
        )
    if policy.get("observation_schema_hash") != manifest["observation_schema_hash"]:
        raise ValueError(
            f"{manifest_path}: policy observation schema hash mismatch: "
            f"{policy.get('observation_schema_hash')!r} != "
            f"{manifest['observation_schema_hash']!r}"
        )
    if policy.get("action_schema_hash") != manifest["action_schema_hash"]:
        raise ValueError(
            f"{manifest_path}: policy action schema hash mismatch: "
            f"{policy.get('action_schema_hash')!r} != {manifest['action_schema_hash']!r}"
        )
    if stable_hash(policy["observation_schema"]) != policy["observation_schema_hash"]:
        raise ValueError(
            f"{manifest_path}: policy embedded observation schema does not match its hash"
        )
    if stable_hash(policy["action_schema"]) != policy["action_schema_hash"]:
        raise ValueError(
            f"{manifest_path}: policy embedded action schema does not match its hash"
        )

    try:
        param_dim = int(policy["param_dim"])
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{manifest_path}: policy param_dim must be an integer"
        ) from error
    if param_dim <= 0:
        raise ValueError(f"{manifest_path}: policy param_dim must be positive")

    params = policy["params"]
    if not isinstance(params, list):
        raise ValueError(f"{manifest_path}: policy params must be an array")
    if len(params) != param_dim:
        raise ValueError(
            f"{manifest_path}: policy params length mismatch: "
            f"{len(params)} != {param_dim}"
        )
    if not isinstance(policy["policy_features"], list) or not policy["policy_features"]:
        raise ValueError(f"{manifest_path}: policy_features must be a non-empty array")
    if not isinstance(policy["policy_outputs"], list) or not policy["policy_outputs"]:
        raise ValueError(f"{manifest_path}: policy_outputs must be a non-empty array")
    for field in (
        "normalization",
        "action_scaling",
        "task_compatibility",
        "engine_compatibility",
    ):
        if not isinstance(policy[field], dict):
            raise ValueError(f"{manifest_path}: policy {field} must be an object")
    try:
        [float(value) for value in params]
        action_limit = float(policy["action_limit_rad_s"])
        best_reward = float(policy["best_reward"])
        training_iterations = int(policy["training_iterations"])
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{manifest_path}: policy numeric fields must be parseable"
        ) from error
    if training_iterations < 0:
        raise ValueError(
            f"{manifest_path}: policy training_iterations must be non-negative"
        )
    if action_limit <= 0.0:
        raise ValueError(f"{manifest_path}: policy action_limit_rad_s must be positive")
    _assert_close(
        manifest_path, "policy best_reward", best_reward, float(manifest["best_reward"])
    )
    if training_iterations != int(manifest["training_iterations"]):
        raise ValueError(
            f"{manifest_path}: policy training_iterations mismatch: "
            f"{training_iterations!r} != {int(manifest['training_iterations'])!r}"
        )


def _validate_rollout_csv_artifact(manifest_path, rollout_csv_path, manifest):
    with open(rollout_csv_path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != list(REQUIRED_ROLLOUT_CSV_FIELDS):
            raise ValueError(
                f"{manifest_path}: rollout CSV header mismatch: {reader.fieldnames!r}"
            )
        rows = list(reader)
    if not rows:
        raise ValueError(f"{manifest_path}: rollout CSV has no rows")

    running_total_reward = 0.0
    seen_done = False
    for index, row in enumerate(rows):
        context = f"{manifest_path}: rollout CSV row {index}"
        step = _int_from_mapping(row, "step", context=context)
        if step != index:
            raise ValueError(
                f"{manifest_path}: rollout step mismatch at row {index}: {step!r} != {index!r}"
            )
        numeric_values = {
            field: _float_from_mapping(row, field, context=context)
            for field in ROLLOUT_NUMERIC_FIELDS
        }
        done = _bool_from_mapping(row, "done", context=context)
        if seen_done:
            raise ValueError(f"{manifest_path}: rollout CSV has rows after done")
        seen_done = done
        running_total_reward += numeric_values["reward"]
        _assert_close(
            manifest_path,
            f"rollout total_reward row {index}",
            numeric_values["total_reward"],
            running_total_reward,
        )

    row_count = len(rows)
    expected_rows = int(manifest["rollout_rows"])
    if row_count != expected_rows:
        raise ValueError(
            f"{manifest_path}: rollout_rows mismatch: {row_count!r} != {expected_rows!r}"
        )

    final_row = rows[-1]
    context = f"{manifest_path}: rollout CSV final row"
    final_total_reward = _float_from_mapping(
        final_row, "total_reward", context=context
    )
    target_dx = _float_from_mapping(final_row, "target_dx", context=context)
    target_dy = _float_from_mapping(final_row, "target_dy", context=context)
    target_dz = _float_from_mapping(final_row, "target_dz", context=context)
    final_target_error = math.sqrt(
        target_dx * target_dx + target_dy * target_dy + target_dz * target_dz
    )

    _assert_close(
        manifest_path,
        "final_total_reward",
        final_total_reward,
        float(manifest["final_total_reward"]),
    )
    _assert_close(
        manifest_path,
        "final_target_error",
        final_target_error,
        float(manifest["final_target_error"]),
    )


def load_reports(paths):
    manifest_paths = _find_manifest_paths(paths)
    if not manifest_paths:
        raise ValueError("no manifest.json files found")
    reports = [_load_manifest(path) for path in manifest_paths]
    return sorted(
        reports,
        key=lambda report: (
            report["final_target_error"],
            -report["final_total_reward"],
            -report["best_reward"],
            report["name"],
        ),
    )


def markdown_table(reports):
    lines = [
        "| rank | report | final_error | final_reward | best_reward | iterations | rows |",
        "|---:|---|---:|---:|---:|---:|---:|",
    ]
    for index, report in enumerate(reports, start=1):
        lines.append(
            "| {rank} | {name} | {error:.4f} | {final_reward:.3f} | {best_reward:.3f} | {iterations} | {rows} |".format(
                rank=index,
                name=report["name"],
                error=report["final_target_error"],
                final_reward=report["final_total_reward"],
                best_reward=report["best_reward"],
                iterations=report["training_iterations"],
                rows=report["rollout_rows"],
            )
        )
    return "\n".join(lines)


def _prepare_output_dir(output_path):
    output_dir = os.path.dirname(os.path.abspath(output_path)) or os.getcwd()
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    return output_dir


def _relative_path(path, output_dir):
    return os.path.relpath(path, output_dir).replace(os.sep, "/")


def _write_json_atomic(output_path, payload):
    _prepare_output_dir(output_path)
    tmp_path = f"{output_path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, output_path)


def _file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def _report_payload(index, report, output_dir):
    house_gif_path = report["rollout_house_gif_path"]
    house_gif_metadata_path = report["rollout_house_gif_metadata_path"]
    house_gif_info = report["rollout_house_gif_info"] or {}
    return {
        "rank": index,
        "report": report["name"],
        "policy_algorithm": report["policy_algorithm"],
        "policy_schema_version": report["policy_schema_version"],
        "observation_schema_hash": report["observation_schema_hash"],
        "action_schema_hash": report["action_schema_hash"],
        "rollout_schema_version": report["rollout_schema_version"],
        "rollout_schema_hash": report["rollout_schema_hash"],
        "final_target_error": report["final_target_error"],
        "final_total_reward": report["final_total_reward"],
        "best_reward": report["best_reward"],
        "training_iterations": report["training_iterations"],
        "rollout_rows": report["rollout_rows"],
        "index_path": _relative_path(report["index_path"], output_dir),
        "policy_path": _relative_path(report["policy_path"], output_dir),
        "rollout_house_gif_path": (
            _relative_path(house_gif_path, output_dir) if house_gif_path else None
        ),
        "rollout_house_gif_metadata_path": (
            _relative_path(house_gif_metadata_path, output_dir)
            if house_gif_metadata_path
            else None
        ),
        "rollout_house_gif_width": house_gif_info.get("width"),
        "rollout_house_gif_height": house_gif_info.get("height"),
        "rollout_house_gif_frames": house_gif_info.get("frame_count"),
        "rollout_house_gif_sample_count": house_gif_info.get("sample_count"),
        "rollout_house_gif_fps": house_gif_info.get("fps"),
        "rollout_house_gif_byte_size": house_gif_info.get("byte_size"),
        "rollout_house_gif_sha256": house_gif_info.get("sha256"),
        "manifest_path": _relative_path(report["manifest_path"], output_dir),
    }


def write_csv(reports, output_path):
    output_dir = _prepare_output_dir(output_path)
    fieldnames = [
        "rank",
        "report",
        "final_target_error",
        "final_total_reward",
        "best_reward",
        "training_iterations",
        "rollout_rows",
        "policy_algorithm",
        "policy_schema_version",
        "observation_schema_hash",
        "action_schema_hash",
        "rollout_schema_version",
        "rollout_schema_hash",
        "index_path",
        "policy_path",
        "rollout_house_gif_path",
        "rollout_house_gif_metadata_path",
        "rollout_house_gif_width",
        "rollout_house_gif_height",
        "rollout_house_gif_frames",
        "rollout_house_gif_sample_count",
        "rollout_house_gif_fps",
        "rollout_house_gif_byte_size",
        "rollout_house_gif_sha256",
        "manifest_path",
    ]
    with open(output_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, report in enumerate(reports, start=1):
            writer.writerow(
                {
                    "rank": index,
                    "report": report["name"],
                    "final_target_error": report["final_target_error"],
                    "final_total_reward": report["final_total_reward"],
                    "best_reward": report["best_reward"],
                    "training_iterations": report["training_iterations"],
                    "rollout_rows": report["rollout_rows"],
                    "policy_algorithm": report["policy_algorithm"],
                    "policy_schema_version": report["policy_schema_version"],
                    "observation_schema_hash": report["observation_schema_hash"],
                    "action_schema_hash": report["action_schema_hash"],
                    "rollout_schema_version": report["rollout_schema_version"],
                    "rollout_schema_hash": report["rollout_schema_hash"],
                    "index_path": _relative_path(report["index_path"], output_dir),
                    "policy_path": _relative_path(report["policy_path"], output_dir),
                    "rollout_house_gif_path": (
                        _relative_path(report["rollout_house_gif_path"], output_dir)
                        if report["rollout_house_gif_path"]
                        else ""
                    ),
                    "rollout_house_gif_metadata_path": (
                        _relative_path(
                            report["rollout_house_gif_metadata_path"], output_dir
                        )
                        if report["rollout_house_gif_metadata_path"]
                        else ""
                    ),
                    "rollout_house_gif_width": (
                        report["rollout_house_gif_info"]["width"]
                        if report["rollout_house_gif_info"]
                        else ""
                    ),
                    "rollout_house_gif_height": (
                        report["rollout_house_gif_info"]["height"]
                        if report["rollout_house_gif_info"]
                        else ""
                    ),
                    "rollout_house_gif_frames": (
                        report["rollout_house_gif_info"]["frame_count"]
                        if report["rollout_house_gif_info"]
                        else ""
                    ),
                    "rollout_house_gif_sample_count": (
                        report["rollout_house_gif_info"].get("sample_count", "")
                        if report["rollout_house_gif_info"]
                        else ""
                    ),
                    "rollout_house_gif_fps": (
                        report["rollout_house_gif_info"].get("fps", "")
                        if report["rollout_house_gif_info"]
                        else ""
                    ),
                    "rollout_house_gif_byte_size": (
                        report["rollout_house_gif_info"]["byte_size"]
                        if report["rollout_house_gif_info"]
                        else ""
                    ),
                    "rollout_house_gif_sha256": (
                        report["rollout_house_gif_info"]["sha256"]
                        if report["rollout_house_gif_info"]
                        else ""
                    ),
                    "manifest_path": _relative_path(
                        report["manifest_path"], output_dir
                    ),
                }
            )


def write_json(reports, output_path):
    output_dir = _prepare_output_dir(output_path)
    payload = {
        "schema_version": LEADERBOARD_SCHEMA_VERSION,
        "reports_considered": len(reports),
        "ranking": list(RANKING_CRITERIA),
        "reports": [
            _report_payload(index, report, output_dir)
            for index, report in enumerate(reports, start=1)
        ],
    }
    _write_json_atomic(output_path, payload)


def _copy_file_atomic(source_path, output_path):
    _prepare_output_dir(output_path)
    tmp_path = f"{output_path}.tmp"

    with open(source_path, "rb") as source:
        content = source.read()
    with open(tmp_path, "wb") as target:
        target.write(content)
        target.flush()
        os.fsync(target.fileno())
    os.replace(tmp_path, output_path)


def copy_best_policy(reports, output_path):
    best = reports[0]
    source_path = best["policy_path"]
    if not os.path.isfile(source_path):
        raise ValueError(f"best report policy artifact does not exist: {source_path}")

    _copy_file_atomic(source_path, output_path)
    return best


def copy_best_house_gif(reports, output_path):
    best = reports[0]
    source_path = best["rollout_house_gif_path"]
    if not source_path:
        raise ValueError(f"best report has no rollout_house_gif artifact: {best['name']}")
    if not os.path.isfile(source_path):
        raise ValueError(f"best report house GIF artifact does not exist: {source_path}")

    _copy_file_atomic(source_path, output_path)
    return best


def write_best_house_gif_metadata(reports, gif_output_path, metadata_output_path):
    output_dir = _prepare_output_dir(metadata_output_path)
    best = reports[0]
    source_path = best["rollout_house_gif_path"]
    info = best["rollout_house_gif_info"]
    if not source_path or info is None:
        raise ValueError(f"best report has no rollout_house_gif artifact: {best['name']}")
    if not os.path.isfile(gif_output_path):
        raise ValueError(f"best house GIF output does not exist: {gif_output_path}")

    byte_size = os.path.getsize(gif_output_path)
    sha256 = _file_sha256(gif_output_path)
    if byte_size != info["byte_size"]:
        raise ValueError(
            f"best house GIF byte_size mismatch after copy: "
            f"{byte_size!r} != {info['byte_size']!r}"
        )
    if sha256 != info["sha256"]:
        raise ValueError(
            f"best house GIF sha256 mismatch after copy: {sha256!r} != {info['sha256']!r}"
        )

    payload = {
        "schema_version": HOUSE_GIF_METADATA_VERSION,
        "artifact": "rne_mobile_manipulator_house_gif",
        "gif_path": _relative_path(gif_output_path, output_dir),
        "source": {
            "kind": "best_report",
            "report": best["name"],
            "manifest_path": _relative_path(best["manifest_path"], output_dir),
            "rollout_house_gif_path": _relative_path(source_path, output_dir),
        },
        "sample_count": info["sample_count"],
        "frame_count": info["frame_count"],
        "max_frames": info["max_frames"],
        "width": info["width"],
        "height": info["height"],
        "fps": info["fps"],
        "byte_size": byte_size,
        "sha256": sha256,
    }
    _write_json_atomic(metadata_output_path, payload)
    return best


def write_best_report_summary(reports, output_path):
    output_dir = _prepare_output_dir(output_path)
    best = reports[0]
    payload = {
        "schema_version": BEST_REPORT_SCHEMA_VERSION,
        "reports_considered": len(reports),
        "ranking": list(RANKING_CRITERIA),
        "best": _report_payload(1, best, output_dir),
    }
    _write_json_atomic(output_path, payload)
    return best


def validate_requirements(
    reports, *, reports_at_least=None, final_error_at_most=None, final_reward_at_least=None
):
    best = reports[0]
    if reports_at_least is not None and len(reports) < reports_at_least:
        raise ValueError(
            f"leaderboard has {len(reports)} reports; expected at least {reports_at_least}"
        )
    if (
        final_error_at_most is not None
        and best["final_target_error"] > final_error_at_most
    ):
        raise ValueError(
            "best report failed final target error gate: "
            f"{best['final_target_error']:.6f} > {final_error_at_most:.6f} "
            f"report={best['name']}"
        )
    if (
        final_reward_at_least is not None
        and best["final_total_reward"] < final_reward_at_least
    ):
        raise ValueError(
            "best report failed final reward gate: "
            f"{best['final_total_reward']:.6f} < {final_reward_at_least:.6f} "
            f"report={best['name']}"
        )
    return best


def render_html(reports, output_path):
    rows = []
    output_dir = os.path.dirname(os.path.abspath(output_path)) or os.getcwd()
    for index, report in enumerate(reports, start=1):
        href = os.path.relpath(report["index_path"], output_dir).replace(os.sep, "/")
        house_gif_path = report["rollout_house_gif_path"]
        if house_gif_path:
            house_gif_href = os.path.relpath(house_gif_path, output_dir).replace(
                os.sep, "/"
            )
            house_gif_metadata_path = report["rollout_house_gif_metadata_path"]
            metadata_link = ""
            if house_gif_metadata_path:
                metadata_href = os.path.relpath(
                    house_gif_metadata_path, output_dir
                ).replace(os.sep, "/")
                metadata_link = (
                    f'<a class="gif-metadata" href="{html.escape(metadata_href)}">'
                    "metadata</a>"
                )
            house_gif_info = report["rollout_house_gif_info"]
            house_gif_label = "GIF"
            if house_gif_info:
                size_kib = house_gif_info["byte_size"] / 1024.0
                house_gif_label = (
                    f"GIF {house_gif_info['width']}x{house_gif_info['height']} "
                    f"/ {house_gif_info['frame_count']}f / {size_kib:.1f} KiB"
                )
            house_gif_cell = (
                f'<a class="gif-preview" href="{html.escape(house_gif_href)}">'
                f'<img src="{html.escape(house_gif_href)}" '
                f'alt="{html.escape(report["name"])} house GIF">'
                f"<span>{html.escape(house_gif_label)}</span></a>"
                f"{metadata_link}"
            )
        else:
            house_gif_cell = ""
        rows.append(
            "<tr>"
            f"<td>{index}</td>"
            f'<td><a href="{html.escape(href)}">{html.escape(report["name"])}</a></td>'
            f"<td>{report['final_target_error']:.4f}</td>"
            f"<td>{report['final_total_reward']:.3f}</td>"
            f"<td>{report['best_reward']:.3f}</td>"
            f"<td>{report['training_iterations']}</td>"
            f"<td>{report['rollout_rows']}</td>"
            f"<td>{house_gif_cell}</td>"
            f"<td>{html.escape(report['policy_algorithm'])}</td>"
            "</tr>"
        )

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RNE reach policy leaderboard</title>
  <style>
    :root {{
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f8fafc;
      color: #0f172a;
    }}
    body {{
      margin: 0;
      padding: 32px;
    }}
    main {{
      max-width: 1080px;
      margin: 0 auto;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    p {{
      margin: 0 0 20px;
      color: #475569;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      background: #fff;
      border: 1px solid #cbd5e1;
    }}
    th, td {{
      padding: 10px 12px;
      border-bottom: 1px solid #e2e8f0;
      text-align: right;
      font-variant-numeric: tabular-nums;
    }}
    th:nth-child(2), td:nth-child(2), th:last-child, td:last-child {{
      text-align: left;
    }}
    td:nth-child(8) {{
      text-align: left;
      min-width: 150px;
    }}
    th {{
      background: #f1f5f9;
      color: #334155;
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.03em;
    }}
    a {{
      color: #0f766e;
      font-weight: 700;
    }}
    .gif-preview {{
      display: grid;
      grid-template-columns: 72px minmax(0, 1fr);
      gap: 8px;
      align-items: center;
      max-width: 260px;
    }}
    .gif-preview img {{
      display: block;
      width: 72px;
      height: 48px;
      object-fit: cover;
      border: 1px solid #cbd5e1;
      border-radius: 6px;
      background: #fff;
    }}
    .gif-preview span {{
      line-height: 1.3;
    }}
    .gif-metadata {{
      display: inline-block;
      margin-top: 6px;
      font-size: 12px;
    }}
  </style>
</head>
<body>
  <main>
    <h1>RNE reach policy leaderboard</h1>
    <p>Ranked by lower final target error, then higher rollout reward.</p>
    <table>
      <thead>
        <tr>
          <th>rank</th>
          <th>report</th>
          <th>final error</th>
          <th>final reward</th>
          <th>best reward</th>
          <th>iterations</th>
          <th>rows</th>
          <th>house GIF</th>
          <th>policy</th>
        </tr>
      </thead>
      <tbody>
        {"".join(rows)}
      </tbody>
    </table>
  </main>
</body>
</html>
"""


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "paths",
        nargs="+",
        help="report directories, manifest.json files, or parent directories to scan",
    )
    parser.add_argument("--html", help="write a standalone HTML leaderboard")
    parser.add_argument("--csv", help="write a CSV leaderboard")
    parser.add_argument("--json", help="write a JSON leaderboard")
    parser.add_argument(
        "--best-policy-out",
        help="copy the top-ranked report's policy artifact to this JSON file",
    )
    parser.add_argument(
        "--best-report-out",
        help="write a JSON summary of the top-ranked report and ranking criteria",
    )
    parser.add_argument(
        "--best-house-gif-out",
        help="copy the top-ranked report's house GIF artifact to this file",
    )
    parser.add_argument(
        "--best-house-gif-metadata-out",
        help=(
            "write checksum metadata for --best-house-gif-out using the "
            "top-ranked report's house GIF provenance"
        ),
    )
    parser.add_argument(
        "--require-reports-at-least",
        type=int,
        help="fail unless at least this many reports were found",
    )
    parser.add_argument(
        "--require-final-error-at-most",
        type=float,
        help="fail unless the top-ranked report has final_target_error at most this value",
    )
    parser.add_argument(
        "--require-final-reward-at-least",
        type=float,
        help="fail unless the top-ranked report has final_total_reward at least this value",
    )
    args = parser.parse_args()
    if args.require_reports_at_least is not None and args.require_reports_at_least <= 0:
        parser.error("--require-reports-at-least must be positive")
    if args.best_house_gif_metadata_out is not None and args.best_house_gif_out is None:
        parser.error("--best-house-gif-metadata-out requires --best-house-gif-out")
    return args


def main():
    args = parse_args()
    reports = load_reports(args.paths)
    print(markdown_table(reports))
    if args.html is not None:
        output_dir = os.path.dirname(os.path.abspath(args.html))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        with open(args.html, "w", encoding="utf-8") as handle:
            handle.write(render_html(reports, args.html))
        print(f"leaderboard html saved: {args.html} reports={len(reports)}")
    if args.csv is not None:
        write_csv(reports, args.csv)
        print(f"leaderboard csv saved: {args.csv} reports={len(reports)}")
    if args.json is not None:
        write_json(reports, args.json)
        print(f"leaderboard json saved: {args.json} reports={len(reports)}")
    if args.best_policy_out is not None:
        best = copy_best_policy(reports, args.best_policy_out)
        print(
            "best policy saved: "
            f"{args.best_policy_out} source={best['policy_path']} report={best['name']}"
        )
    if args.best_report_out is not None:
        best = write_best_report_summary(reports, args.best_report_out)
        print(f"best report saved: {args.best_report_out} report={best['name']}")
    if args.best_house_gif_out is not None:
        best = copy_best_house_gif(reports, args.best_house_gif_out)
        print(
            "best house gif saved: "
            f"{args.best_house_gif_out} source={best['rollout_house_gif_path']} "
            f"report={best['name']}"
        )
        if args.best_house_gif_metadata_out is not None:
            best = write_best_house_gif_metadata(
                reports, args.best_house_gif_out, args.best_house_gif_metadata_out
            )
            print(
                "best house gif metadata saved: "
                f"{args.best_house_gif_metadata_out} "
                f"source={best['rollout_house_gif_path']} report={best['name']}"
            )
    if (
        args.require_reports_at_least is not None
        or args.require_final_error_at_most is not None
        or args.require_final_reward_at_least is not None
    ):
        best = validate_requirements(
            reports,
            reports_at_least=args.require_reports_at_least,
            final_error_at_most=args.require_final_error_at_most,
            final_reward_at_least=args.require_final_reward_at_least,
        )
        print(f"leaderboard gates passed: best_report={best['name']}")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        sys.exit(str(error))
