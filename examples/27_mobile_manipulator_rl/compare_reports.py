"""Compare one or more RNE mobile-manipulator report bundles.

Each report is produced by:

    python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --report-dir reports/reach

This script reads each bundle's manifest.json, prints a Markdown leaderboard, can write
standalone HTML/CSV/JSON leaderboards, can copy the best policy artifact, can write a
machine-readable best-report summary, and can enforce CI-friendly quality gates.
"""

import argparse
import csv
import html
import json
import os
import sys


REQUIRED_MANIFEST_FIELDS = (
    "schema_version",
    "policy_algorithm",
    "policy_schema_version",
    "best_reward",
    "training_iterations",
    "rollout_rows",
    "final_total_reward",
    "final_target_error",
    "artifacts",
)
REQUIRED_ARTIFACT_KEYS = ("index", "policy", "rollout_csv")
REQUIRED_POLICY_FIELDS = (
    "schema_version",
    "algorithm",
    "param_dim",
    "action_limit_rad_s",
    "params",
    "best_reward",
    "training_iterations",
)

REPORT_MANIFEST_SCHEMA_VERSION = 1
POLICY_ARTIFACT_SCHEMA_VERSION = 1
LEADERBOARD_SCHEMA_VERSION = 1
BEST_REPORT_SCHEMA_VERSION = 1

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
    if not isinstance(manifest["artifacts"], dict):
        raise ValueError(f"{path}: manifest artifacts must be an object")
    report_dir = os.path.dirname(path)
    artifacts = manifest["artifacts"]
    resolved_artifacts = _resolve_artifacts(path, report_dir, artifacts)
    _validate_policy_artifact(path, resolved_artifacts["policy"], manifest)
    return {
        "name": os.path.basename(report_dir),
        "report_dir": report_dir,
        "manifest_path": path,
        "index_path": resolved_artifacts["index"],
        "policy_path": resolved_artifacts["policy"],
        "policy_algorithm": manifest["policy_algorithm"],
        "policy_schema_version": int(manifest["policy_schema_version"]),
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
    try:
        [float(value) for value in params]
        float(policy["action_limit_rad_s"])
        float(policy["best_reward"])
        int(policy["training_iterations"])
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{manifest_path}: policy numeric fields must be parseable"
        ) from error
    if int(policy["training_iterations"]) < 0:
        raise ValueError(
            f"{manifest_path}: policy training_iterations must be non-negative"
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


def _report_payload(index, report, output_dir):
    return {
        "rank": index,
        "report": report["name"],
        "policy_algorithm": report["policy_algorithm"],
        "policy_schema_version": report["policy_schema_version"],
        "final_target_error": report["final_target_error"],
        "final_total_reward": report["final_total_reward"],
        "best_reward": report["best_reward"],
        "training_iterations": report["training_iterations"],
        "rollout_rows": report["rollout_rows"],
        "index_path": _relative_path(report["index_path"], output_dir),
        "policy_path": _relative_path(report["policy_path"], output_dir),
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
        "index_path",
        "policy_path",
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
                    "index_path": _relative_path(report["index_path"], output_dir),
                    "policy_path": _relative_path(report["policy_path"], output_dir),
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


def copy_best_policy(reports, output_path):
    best = reports[0]
    source_path = best["policy_path"]
    if not os.path.isfile(source_path):
        raise ValueError(f"best report policy artifact does not exist: {source_path}")

    _prepare_output_dir(output_path)
    tmp_path = f"{output_path}.tmp"

    with open(source_path, "rb") as source:
        content = source.read()
    with open(tmp_path, "wb") as target:
        target.write(content)
        target.flush()
        os.fsync(target.fileno())
    os.replace(tmp_path, output_path)
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
        rows.append(
            "<tr>"
            f"<td>{index}</td>"
            f'<td><a href="{html.escape(href)}">{html.escape(report["name"])}</a></td>'
            f"<td>{report['final_target_error']:.4f}</td>"
            f"<td>{report['final_total_reward']:.3f}</td>"
            f"<td>{report['best_reward']:.3f}</td>"
            f"<td>{report['training_iterations']}</td>"
            f"<td>{report['rollout_rows']}</td>"
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
