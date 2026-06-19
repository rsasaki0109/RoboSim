"""Compare one or more RNE mobile-manipulator report bundles.

Each report is produced by:

    python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --report-dir reports/reach

This script reads each bundle's manifest.json, prints a Markdown leaderboard, and can
also write a standalone HTML leaderboard.
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
    "best_reward",
    "training_iterations",
    "rollout_rows",
    "final_total_reward",
    "final_target_error",
    "artifacts",
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
    report_dir = os.path.dirname(path)
    artifacts = manifest["artifacts"]
    return {
        "name": os.path.basename(report_dir),
        "report_dir": report_dir,
        "manifest_path": path,
        "index_path": os.path.join(report_dir, artifacts.get("index", "index.html")),
        "policy_algorithm": manifest["policy_algorithm"],
        "best_reward": float(manifest["best_reward"]),
        "training_iterations": int(manifest["training_iterations"]),
        "rollout_rows": int(manifest["rollout_rows"]),
        "final_total_reward": float(manifest["final_total_reward"]),
        "final_target_error": float(manifest["final_target_error"]),
    }


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


def write_csv(reports, output_path):
    output_dir = os.path.dirname(os.path.abspath(output_path)) or os.getcwd()
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    fieldnames = [
        "rank",
        "report",
        "final_target_error",
        "final_total_reward",
        "best_reward",
        "training_iterations",
        "rollout_rows",
        "policy_algorithm",
        "index_path",
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
                    "index_path": os.path.relpath(
                        report["index_path"], output_dir
                    ).replace(os.sep, "/"),
                    "manifest_path": os.path.relpath(
                        report["manifest_path"], output_dir
                    ).replace(os.sep, "/"),
                }
            )


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
    return parser.parse_args()


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


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        sys.exit(str(error))
