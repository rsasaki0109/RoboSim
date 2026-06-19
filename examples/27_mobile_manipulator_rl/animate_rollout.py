"""Create an interactive HTML replay from a mobile-manipulator rollout CSV.

The input is produced by:

    python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --rollout-csv rollout.csv

The output HTML is dependency-free and can be opened directly in a browser.
"""

import argparse
import html
import json
import os
import sys

from plot_rollout import load_rollout


def _bounds(samples):
    xs = [sample["ee_x"] for sample in samples] + [sample["target_x"] for sample in samples]
    zs = [sample["ee_z"] for sample in samples] + [sample["target_z"] for sample in samples]

    def padded(values):
        lo = min(values)
        hi = max(values)
        if lo == hi:
            return lo - 1.0, hi + 1.0
        pad = (hi - lo) * 0.16
        return lo - pad, hi + pad

    x0, x1 = padded(xs)
    z0, z1 = padded(zs)
    return {"x0": x0, "x1": x1, "z0": z0, "z1": z1}


def render_html(samples, title):
    payload = json.dumps(samples, separators=(",", ":"))
    bounds = json.dumps(_bounds(samples), separators=(",", ":"))
    escaped_title = html.escape(title)
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{escaped_title}</title>
  <style>
    :root {{
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f8fafc;
      color: #0f172a;
    }}
    body {{
      margin: 0;
      padding: 28px;
    }}
    main {{
      max-width: 1120px;
      margin: 0 auto;
    }}
    h1 {{
      margin: 0 0 6px;
      font-size: 26px;
      line-height: 1.2;
    }}
    .summary {{
      margin: 0 0 20px;
      color: #475569;
      font-size: 14px;
    }}
    .surface {{
      background: #fff;
      border: 1px solid #cbd5e1;
      border-radius: 8px;
      padding: 16px;
    }}
    canvas {{
      display: block;
      width: 100%;
      aspect-ratio: 16 / 9;
      background: #fff;
      border: 1px solid #e2e8f0;
    }}
    .controls {{
      display: grid;
      grid-template-columns: auto 1fr auto;
      gap: 12px;
      align-items: center;
      margin-top: 14px;
    }}
    button {{
      min-width: 72px;
      border: 1px solid #0f766e;
      border-radius: 6px;
      background: #0f766e;
      color: #fff;
      font: 700 14px system-ui, sans-serif;
      padding: 8px 12px;
      cursor: pointer;
    }}
    input[type="range"] {{
      width: 100%;
    }}
    .metrics {{
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      margin-top: 14px;
    }}
    .metric {{
      border: 1px solid #e2e8f0;
      border-radius: 6px;
      padding: 10px;
    }}
    .metric span {{
      display: block;
      color: #64748b;
      font-size: 12px;
    }}
    .metric strong {{
      display: block;
      margin-top: 4px;
      font-size: 18px;
      font-variant-numeric: tabular-nums;
    }}
  </style>
</head>
<body>
  <main>
    <h1>{escaped_title}</h1>
    <p class="summary">samples={len(samples)} final_reward={samples[-1]["total_reward"]:.3f}</p>
    <section class="surface">
      <canvas id="scene" width="960" height="540"></canvas>
      <div class="controls">
        <button id="play">Play</button>
        <input id="scrub" type="range" min="0" max="{len(samples) - 1}" value="0" step="1">
        <output id="step">0</output>
      </div>
      <div class="metrics">
        <div class="metric"><span>target error</span><strong id="error">0.000</strong></div>
        <div class="metric"><span>step reward</span><strong id="reward">0.000</strong></div>
        <div class="metric"><span>total reward</span><strong id="total">0.000</strong></div>
        <div class="metric"><span>actions</span><strong id="actions">0.00 / 0.00</strong></div>
      </div>
    </section>
  </main>
  <script>
    const samples = {payload};
    const bounds = {bounds};
    const canvas = document.getElementById('scene');
    const context = canvas.getContext('2d');
    const play = document.getElementById('play');
    const scrub = document.getElementById('scrub');
    const stepLabel = document.getElementById('step');
    const errorLabel = document.getElementById('error');
    const rewardLabel = document.getElementById('reward');
    const totalLabel = document.getElementById('total');
    const actionsLabel = document.getElementById('actions');
    let index = 0;
    let timer = null;

    function mapPoint(x, z) {{
      const pad = 44;
      const width = canvas.width - pad * 2;
      const height = canvas.height - pad * 2;
      const px = pad + ((x - bounds.x0) / (bounds.x1 - bounds.x0)) * width;
      const py = pad + height - ((z - bounds.z0) / (bounds.z1 - bounds.z0)) * height;
      return [px, py];
    }}

    function drawGrid() {{
      context.strokeStyle = '#e2e8f0';
      context.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {{
        const t = i / 4;
        const x = 44 + t * (canvas.width - 88);
        const y = 44 + t * (canvas.height - 88);
        context.beginPath();
        context.moveTo(x, 44);
        context.lineTo(x, canvas.height - 44);
        context.stroke();
        context.beginPath();
        context.moveTo(44, y);
        context.lineTo(canvas.width - 44, y);
        context.stroke();
      }}
    }}

    function strokePath(upto) {{
      context.strokeStyle = '#0f766e';
      context.lineWidth = 4;
      context.beginPath();
      for (let i = 0; i <= upto; i++) {{
        const [x, y] = mapPoint(samples[i].ee_x, samples[i].ee_z);
        if (i === 0) context.moveTo(x, y);
        else context.lineTo(x, y);
      }}
      context.stroke();
    }}

    function drawCircle(x, y, radius, stroke, fill) {{
      context.beginPath();
      context.arc(x, y, radius, 0, Math.PI * 2);
      if (fill) {{
        context.fillStyle = fill;
        context.fill();
      }}
      if (stroke) {{
        context.strokeStyle = stroke;
        context.lineWidth = 3;
        context.stroke();
      }}
    }}

    function draw(i) {{
      const sample = samples[i];
      context.clearRect(0, 0, canvas.width, canvas.height);
      drawGrid();
      strokePath(i);
      const [targetX, targetY] = mapPoint(sample.target_x, sample.target_z);
      const [eeX, eeY] = mapPoint(sample.ee_x, sample.ee_z);
      drawCircle(targetX, targetY, 12, '#d62728', null);
      drawCircle(eeX, eeY, 8, '#0f766e', '#0f766e');
      context.strokeStyle = '#94a3b8';
      context.lineWidth = 2;
      context.setLineDash([5, 5]);
      context.beginPath();
      context.moveTo(eeX, eeY);
      context.lineTo(targetX, targetY);
      context.stroke();
      context.setLineDash([]);
      context.fillStyle = '#475569';
      context.font = '14px system-ui, sans-serif';
      context.fillText(`x-z path step ${{sample.step}}`, 54, 28);

      scrub.value = i;
      stepLabel.value = `step ${{sample.step}}`;
      errorLabel.textContent = sample.target_error.toFixed(3);
      rewardLabel.textContent = sample.reward.toFixed(3);
      totalLabel.textContent = sample.total_reward.toFixed(3);
      actionsLabel.textContent = `${{sample.shoulder_action.toFixed(2)}} / ${{sample.elbow_action.toFixed(2)}}`;
    }}

    function stop() {{
      if (timer !== null) {{
        clearInterval(timer);
        timer = null;
      }}
      play.textContent = 'Play';
    }}

    play.addEventListener('click', () => {{
      if (timer !== null) {{
        stop();
        return;
      }}
      play.textContent = 'Pause';
      timer = setInterval(() => {{
        index = Math.min(index + 1, samples.length - 1);
        draw(index);
        if (index === samples.length - 1) stop();
      }}, 90);
    }});

    scrub.addEventListener('input', () => {{
      index = Number(scrub.value);
      draw(index);
    }});

    draw(index);
  </script>
</body>
</html>
"""


def default_output_path(input_path):
    root, _ = os.path.splitext(input_path)
    return f"{root}.html"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", help="rollout CSV produced by train.py --rollout-csv")
    parser.add_argument("--out", help="HTML output path; defaults to CSV path with .html")
    return parser.parse_args()


def main():
    args = parse_args()
    output_path = args.out or default_output_path(args.csv_path)
    samples = load_rollout(args.csv_path)
    html_report = render_html(samples, os.path.basename(args.csv_path))
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write(html_report)
    print(f"rollout html saved: {output_path} samples={len(samples)}")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        sys.exit(str(error))
