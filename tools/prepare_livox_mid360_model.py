#!/usr/bin/env python3
"""Fit the Livox Mid-360 scan model and Go2 rig mask from real rosbag2 recordings.

The Mid-360 does not publish its scan pattern. This script recovers it from real
`/livox/lidar` PointCloud2 frames recorded by livox_ros_driver2 (xfer format 0:
`x y z intensity t`, no-return slots kept as zero points):

1. ``extract``: decode frames from a rosbag2 sqlite bag into compact arrays.
2. ``fit``: estimate the rotor and elevation-nod rates, then alternate between a
   2D Fourier fit of each emitter line's direction over (rotor phase, nod phase)
   and per-frame phase tracking.
3. ``track``: hold the fitted shape fixed and track per-frame phases on another bag.
4. ``build``: write the Rust coefficient table, the rig occlusion asset, and the
   held-out direction fixture.

Firing structure used throughout (measured, see docs/LIVOX_MID360.md): the
message order is the emission order, points come in 96-point packets, and point
``n`` of the stream belongs to firing ``n // 4`` and emitter line ``n % 4``.

Example (the recordings are private Go2 lab bags, not redistributed)::

    python3 tools/prepare_livox_mid360_model.py extract BAG_DIR box.npz --frames 400
    python3 tools/prepare_livox_mid360_model.py fit box.npz model.npz
    python3 tools/prepare_livox_mid360_model.py extract BAG2_DIR mask2.npz --frames 300
    python3 tools/prepare_livox_mid360_model.py track mask2.npz model.npz mask2_track.npz
    python3 tools/prepare_livox_mid360_model.py build box.npz mask2.npz model.npz mask2_track.npz
"""

from __future__ import annotations

import argparse
import glob
import json
import sqlite3
import subprocess
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LINES = 4
KA, KP = 3, 6
NOMINAL_NOD_PERIOD_FIRINGS = 5100.0
AZ_BIN_DEG, EL_BIN_DEG, EL_MIN_DEG, EL_BINS = 2.0, 2.0, -8.0, 32
# Near-range blanking knots (range_m, probability); keep in sync with
# NEAR_BLANKING_KNOTS in crates/rne_sensor/src/livox.rs.
NEAR_BLANKING_KNOTS = [
    (0.5125, 0.997), (0.5375, 0.985), (0.5625, 0.957), (0.5875, 0.867),
    (0.6125, 0.735), (0.6375, 0.580), (0.6625, 0.429), (0.6875, 0.265),
    (0.7125, 0.151), (0.7375, 0.098), (0.7625, 0.060), (0.7875, 0.028),
    (0.8125, 0.010), (0.8375, 0.003), (0.8625, 0.002), (0.8875, 0.0),
]
# On the Go2 the upside-down sensor sees the floor within 1.2 m above this elevation;
# there the whole row is fixed to the mount, so no environment baseline is removed.
FLOOR_ZONE_MIN_ELEVATION_DEG = 22.0


def near_blanking_probability(r):
    r = np.asarray(r, float)
    knots_r = np.array([k[0] for k in NEAR_BLANKING_KNOTS])
    knots_p = np.array([k[1] for k in NEAR_BLANKING_KNOTS])
    return np.where(r <= knots_r[0], 1.0, np.interp(r, knots_r, knots_p, right=0.0))


# --------------------------------------------------------------------------- extract
def extract(bag: str, out: str, frames: int) -> None:
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    con = sqlite3.connect(glob.glob(bag + "/*.db3")[0])
    tid = con.execute("select id from topics where name='/livox/lidar'").fetchone()[0]
    xyz, intensity, t, frame, stamp = [], [], [], [], []
    rows = con.execute(
        "select data from messages where topic_id=? order by timestamp", (tid,)
    )
    for k, (raw,) in enumerate(rows):
        if k >= frames:
            break
        msg = deserialize_message(raw, PointCloud2)
        fields = {f.name: f.offset for f in msg.fields}
        dtype = np.dtype(
            {
                "names": ["x", "y", "z", "intensity", "t"],
                "formats": ["f4", "f4", "f4", "f4", "f8"],
                "offsets": [fields[n] for n in ["x", "y", "z", "intensity", "t"]],
                "itemsize": msg.point_step,
            }
        )
        a = np.frombuffer(bytes(msg.data), dtype=dtype)
        xyz.append(np.stack([a["x"], a["y"], a["z"]], 1))
        intensity.append(a["intensity"])
        t.append(a["t"])
        frame.append(np.full(len(a), k, np.int32))
        stamp.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    np.savez_compressed(
        out,
        xyz=np.concatenate(xyz),
        intensity=np.concatenate(intensity),
        t=np.concatenate(t),
        frame=np.concatenate(frame),
        stamp=np.array(stamp),
    )
    print(f"frames {len(stamp)} points {sum(len(x) for x in xyz)}")


# --------------------------------------------------------------------------- model
HARMONICS = [
    (i, j) for i in range(-KA, KA + 1) for j in range(-KP, KP + 1) if (i, j) >= (0, 0)
]


def design(a: np.ndarray, p: np.ndarray) -> np.ndarray:
    cols = []
    for i, j in HARMONICS:
        ph = i * a + j * p
        if (i, j) == (0, 0):
            cols.append(np.ones_like(a))
        else:
            cols += [np.cos(ph), np.sin(ph)]
    return np.stack(cols, 1)


def predict(coef, a, p):
    """Unit directions in the Livox frame for rotor phase ``a`` and nod phase ``p``."""
    x = design(a, p)
    q = np.stack([x @ k for k in coef], 1)
    d = np.stack(
        [np.cos(a) * q[:, 0] - np.sin(a) * q[:, 1], np.sin(a) * q[:, 0] + np.cos(a) * q[:, 1], q[:, 2]],
        1,
    )
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def directions(model, firing, line, a_off, p_off):
    firing = np.asarray(firing, float)
    out = np.zeros((len(firing), 3))
    for l in range(LINES):
        m = line == l
        if not m.any():
            continue
        ao = a_off[m] if np.ndim(a_off) else a_off
        po = p_off[m] if np.ndim(p_off) else p_off
        out[m] = predict(model["coef"][l], model["wa"] * firing[m] + ao, model["wp"] * firing[m] + po)
    return out


def load_model(path):
    m = np.load(path)
    return {
        "wa": float(m["wa"]),
        "wp": float(m["wp"]),
        "coef": {l: [m[f"l{l}_{c}"] for c in range(3)] for l in range(LINES)},
    }


def unit_returns(d):
    xyz = d["xyz"].astype(float)
    r = np.linalg.norm(xyz, axis=1)
    valid = r > 1e-6
    return xyz / np.where(valid, r, 1)[:, None], valid, r


def angle_deg(a, b):
    return np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", a, b), -1, 1)))


def refine_rate(g, signal, w0, spans, weights=None):
    """Maximizes coherence |mean(signal * exp(-i w g))| around ``w0``."""
    best = w0
    for span in spans:
        ws = np.linspace(best - span, best + span, 401)
        c = [np.abs(np.mean(signal * np.exp(-1j * w * g))) for w in ws]
        best = ws[int(np.argmax(c))]
    return best


def track_frame(model, firing, line, u, a0, p0, iters=10):
    a, p = a0, p0
    for _ in range(iters):
        f0 = directions(model, firing, line, a, p)
        h = 1e-4
        ja = (directions(model, firing, line, a + h, p) - f0) / h
        jp = (directions(model, firing, line, a, p + h) - f0) / h
        step = np.linalg.lstsq(np.stack([ja.ravel(), jp.ravel()], 1), (u - f0).ravel(), rcond=None)[0]
        a += step[0]
        p += step[1]
        if np.abs(step).max() < 1e-7:
            break
    return a, p


# --------------------------------------------------------------------------- fit
def fit(data: str, out: str, train_frames: int, iterations: int) -> None:
    d = np.load(data)
    u, valid, _ = unit_returns(d)
    frame = d["frame"]
    n = len(u)
    firing = (np.arange(n) // LINES).astype(float)
    line = np.arange(n) % LINES
    az = np.arctan2(u[:, 1], u[:, 0])
    el = np.arcsin(np.clip(u[:, 2], -1, 1))

    m = valid & (line == 0)
    sub = np.nonzero(m)[0][::7]
    wa = refine_rate(firing[sub], np.exp(1j * az[sub]), np.radians(1.3), [np.radians(0.1), 1e-5, 1e-7])
    wa = refine_rate(firing[m], np.exp(1j * az[m]), wa, [1e-7, 1e-9])
    e0 = el[m] - el[m].mean()
    w0 = 2 * np.pi / NOMINAL_NOD_PERIOD_FIRINGS
    wp = refine_rate(firing[m], e0, w0, [w0 * 0.1, w0 * 1e-3, w0 * 1e-5, w0 * 1e-7])
    print(f"rotor {np.degrees(wa):.9f} deg/firing, nod period {2 * np.pi / wp:.4f} firings", flush=True)

    nf = frame.max() + 1
    a_off = np.zeros(nf)
    p_off = np.zeros(nf)
    per_frame = [np.nonzero(valid & (frame == k))[0] for k in range(nf)]
    model = {"wa": wa, "wp": wp, "coef": {}}
    for it in range(iterations):
        sel = valid & (frame < (3 if it == 0 else train_frames))
        a = wa * firing + a_off[frame]
        p = wp * firing + p_off[frame]
        for l in range(LINES):
            mm = sel & (line == l)
            am = a[mm]
            cx = np.cos(-am) * u[mm, 0] - np.sin(-am) * u[mm, 1]
            cy = np.sin(-am) * u[mm, 0] + np.cos(-am) * u[mm, 1]
            x = design(am, p[mm])
            model["coef"][l] = [np.linalg.lstsq(x, t, rcond=None)[0] for t in (cx, cy, u[mm, 2])]
        for k in range(nf):
            if it == 0 and k > 0:
                a_off[k], p_off[k] = a_off[k - 1], p_off[k - 1]
            idx = per_frame[k][::3]
            a_off[k], p_off[k] = track_frame(model, firing[idx], line[idx], u[idx], a_off[k], p_off[k], 8)
        for name, s in [("train", valid & (frame < train_frames)), ("held-out", valid & (frame >= train_frames))]:
            idx = np.nonzero(s)[0][::5]
            e = angle_deg(directions(model, firing[idx], line[idx], a_off[frame[idx]], p_off[frame[idx]]), u[idx])
            print(f"iter {it} {name} angle error deg p50/p90/p99 {np.percentile(e, [50, 90, 99]).round(3)}", flush=True)
    np.savez(
        out,
        wa=wa,
        wp=wp,
        a_off=a_off,
        p_off=p_off,
        **{f"l{l}_{c}": model["coef"][l][c] for l in range(LINES) for c in range(3)},
    )


# --------------------------------------------------------------------------- track
def track(data: str, model_path: str, out: str) -> None:
    model = load_model(model_path)
    d = np.load(data)
    u, valid, _ = unit_returns(d)
    frame = d["frame"]
    nf = frame.max() + 1
    firing = np.arange(len(u)) // LINES
    line = np.arange(len(u)) % LINES
    idx0 = np.nonzero(valid & (frame == 0))[0][::20]
    best = (1e9, 0.0, 0.0)
    for a in np.linspace(-np.pi, np.pi, 72, endpoint=False):
        for p in np.linspace(-np.pi, np.pi, 72, endpoint=False):
            e = np.median(angle_deg(directions(model, firing[idx0], line[idx0], a, p), u[idx0]))
            if e < best[0]:
                best = (e, a, p)
    a_off = np.zeros(nf)
    p_off = np.zeros(nf)
    a_off[0], p_off[0] = best[1], best[2]
    for k in range(nf):
        if k > 0:
            a_off[k], p_off[k] = a_off[k - 1], p_off[k - 1]
        idx = np.nonzero(valid & (frame == k))[0][::3]
        a_off[k], p_off[k] = track_frame(model, firing[idx], line[idx], u[idx], a_off[k], p_off[k])
        if k % 50 == 0:
            e = angle_deg(directions(model, firing[idx], line[idx], a_off[k], p_off[k]), u[idx])
            print(f"frame {k} median angle error deg {np.median(e):.3f}", flush=True)
    np.savez(out, a_off=a_off, p_off=p_off)


# --------------------------------------------------------------------------- build
def slot_grid(data, model, a_off, p_off, frames, stride=2):
    d = np.load(data)
    frame = d["frame"]
    sel = np.nonzero((frame >= frames[0]) & (frame < frames[1]))[0][::stride]
    xyz = d["xyz"][sel].astype(float)
    r = np.linalg.norm(xyz, axis=1)
    valid = r > 1e-6
    dirs = directions(model, sel // LINES, sel % LINES, a_off[frame[sel]], p_off[frame[sel]])
    el = np.degrees(np.arcsin(dirs[:, 2]))
    az = np.degrees(np.arctan2(dirs[:, 1], dirs[:, 0]))
    ai = np.clip(((az + 180) // AZ_BIN_DEG).astype(int), 0, int(360 / AZ_BIN_DEG) - 1)
    ei = np.clip(((el - EL_MIN_DEG) // EL_BIN_DEG).astype(int), 0, EL_BINS - 1)
    shape = (int(360 / AZ_BIN_DEG), EL_BINS)
    total = np.zeros(shape)
    zero = np.zeros(shape)
    self_hits = np.zeros(shape)
    np.add.at(total, (ai, ei), 1)
    np.add.at(zero, (ai, ei), ~valid)
    near = valid & (r < 0.35)
    np.add.at(self_hits, (ai[near], ei[near]), 1)
    ranges = [[[] for _ in range(EL_BINS)] for _ in range(shape[0])]
    for a, e, rr in zip(ai[near], ei[near], r[near]):
        ranges[a][e].append(rr)
    med = np.array([[np.median(c) if c else 0.0 for c in row] for row in ranges])
    # Blanking probability each non-self return imposes on the next firing.
    cast = valid & ~near
    blanking = np.zeros(shape)
    cast_count = np.zeros(shape)
    np.add.at(cast_count, (ai[cast], ei[cast]), 1)
    np.add.at(blanking, (ai[cast], ei[cast]), near_blanking_probability(r[cast]))
    return total, zero, self_hits, med, cast_count, blanking


def build(box: str, mask2: str, model_path: str, mask2_track: str) -> None:
    model = load_model(model_path)
    mb = np.load(model_path)
    m2 = np.load(mask2_track)
    grids = [
        slot_grid(box, model, mb["a_off"], mb["p_off"], (0, 400)),
        slot_grid(mask2, model, m2["a_off"], m2["p_off"], (20, 300)),
    ]
    # A slot is blocked, returns from the robot, or reaches a surface. Blanking hides
    # some surface returns: with x the fraction of slots reaching a surface and p the
    # mean blanking probability of the bin's returns, the observed surface-return
    # fraction is R = x / (1 + p x), so x = R / (1 - p R). Outside the floor zone the
    # environment can leave slots empty too, so each row's 20th-percentile block level
    # is removed there; the lower of the two recordings keeps what is fixed to the
    # sensor.
    blocks, selfs = [], []
    row_el = EL_MIN_DEG + EL_BIN_DEG * np.arange(EL_BINS)
    for total, zero, self_hits, _, cast_count, blanking in grids:
        n = np.maximum(total, 1)
        s_frac = self_hits / n
        r_frac = cast_count / n
        p_mean = blanking / np.maximum(cast_count, 1)
        reach = np.clip(r_frac / np.maximum(1 - p_mean * r_frac, 1e-6), 0, 1 - s_frac)
        b = np.clip(1 - s_frac - reach, 0, 1)
        base = np.percentile(b, 20, axis=0)
        base = np.where(row_el >= FLOOR_ZONE_MIN_ELEVATION_DEG, 0.0, base)
        blocks.append(np.clip((b - base) / np.maximum(1 - base, 1e-6), 0, 1))
        selfs.append(s_frac)
    # Bins that only one recording samples well (the top of the elevation band) take
    # that recording's value.
    has = [g[0] >= 40 for g in grids]
    both = has[0] & has[1]
    block = np.where(both, np.minimum(*blocks), np.where(has[0], blocks[0], np.where(has[1], blocks[1], 0.0)))
    # In the floor zone the loss is fixed to the mount along the whole row, so bins
    # neither recording sampled take the row median of the sampled ones.
    sampled = has[0] | has[1]
    for e in range(EL_BINS):
        if row_el[e] >= FLOOR_ZONE_MIN_ELEVATION_DEG and sampled[:, e].any():
            block[~sampled[:, e], e] = np.median(block[sampled[:, e], e])
    # Returns within 0.35 m come from the robot in both recordings, so average them.
    self_p = np.where(both, 0.5 * (selfs[0] + selfs[1]), np.where(has[0], selfs[0], np.where(has[1], selfs[1], 0.0)))
    block[block < 0.05] = 0.0
    self_p[self_p < 0.005] = 0.0
    self_range = np.where(grids[0][3] > 0, grids[0][3], grids[1][3])
    cells = []
    for a in range(block.shape[0]):
        for e in range(EL_BINS):
            if block[a, e] > 0 or self_p[a, e] > 0:
                cells.append(
                    {
                        "azimuth_bin": a,
                        "elevation_bin": e,
                        "block_probability": round(float(block[a, e]), 3),
                        "self_return_probability": round(float(min(self_p[a, e], 1 - block[a, e])), 3),
                        "self_return_range_m": round(float(self_range[a, e]), 3),
                    }
                )
    rig = {
        "description": "Measured occlusion of the Unitree Go2 Mid-360 rig in the Livox sensor frame "
        "(x forward, y left, z up of the upside-down sensor). Derived from EIL_Box (400 frames) and "
        "EIL_Mask2 (280 frames) by tools/prepare_livox_mid360_model.py.",
        "min_azimuth_deg": -180.0,
        "azimuth_bin_deg": AZ_BIN_DEG,
        "azimuth_bins": int(360 / AZ_BIN_DEG),
        "min_elevation_deg": EL_MIN_DEG,
        "elevation_bin_deg": EL_BIN_DEG,
        "elevation_bins": EL_BINS,
        "cells": cells,
    }
    out_dir = ROOT / "assets/sensors/livox_mid360"
    out_dir.mkdir(parents=True, exist_ok=True)
    header = json.dumps({k: v for k, v in rig.items() if k != "cells"}, indent=1)[:-2]
    body = ",\n".join("  " + json.dumps(c, separators=(",", ":")) for c in cells)
    (out_dir / "go2_rig_occlusion.json").write_text(f'{header},\n "cells": [\n{body}\n ]\n}}\n')
    print(f"rig cells {len(cells)}")

    write_coefficients(model)
    write_fixture(mask2, model, m2)


def write_coefficients(model) -> None:
    lines = [
        "//! Livox Mid-360 scan-shape coefficients.",
        "//!",
        "//! Generated by `tools/prepare_livox_mid360_model.py build`; do not edit by hand.",
        "//! Fitted to 400 real frames of a Go2-mounted Mid-360 (EIL_Box recording).",
        "",
        "/// Rotor phase advance per firing, in radians.",
        f"pub(crate) const ROTOR_RAD_PER_FIRING: f64 = {model['wa']!r};",
        "/// Elevation-nod phase advance per firing, in radians.",
        f"pub(crate) const NOD_RAD_PER_FIRING: f64 = {model['wp']!r};",
        "/// `(rotor, nod)` harmonic of each basis pair; `(0, 0)` is the constant term.",
        f"pub(crate) const HARMONICS: [(i8, i8); {len(HARMONICS)}] = [",
    ]
    lines += [f"    ({i}, {j})," for i, j in HARMONICS]
    lines.append("];")
    ncol = 1 + 2 * (len(HARMONICS) - 1)
    lines += [
        "/// Per line, per rotor-frame component (x, y, z): basis weights in `HARMONICS`",
        "/// order, constant first, then a cosine/sine pair per harmonic.",
        f"pub(crate) const COEFFICIENTS: [[[f64; {ncol}]; 3]; 4] = [",
    ]
    for l in range(LINES):
        lines.append("    [")
        for c in range(3):
            vals = ", ".join(repr(float(v)) for v in model["coef"][l][c])
            lines.append(f"        [{vals}],")
        lines.append("    ],")
    lines.append("];")
    path = ROOT / "crates/rne_sensor/src/livox_mid360_coefficients.rs"
    path.write_text("\n".join(lines) + "\n")
    subprocess.run(["rustfmt", "--edition", "2021", str(path)], check=True)


def write_fixture(mask2: str, model, track_result) -> None:
    d = np.load(mask2)
    u, valid, _ = unit_returns(d)
    frame = d["frame"]
    rng = np.random.default_rng(360)
    held = np.nonzero(valid & (frame >= 150) & (frame < 300))[0]
    pick = np.sort(rng.choice(held, 1500, replace=False))
    a_off = track_result["a_off"][frame[pick]]
    p_off = track_result["p_off"][frame[pick]]
    pred = directions(model, pick // LINES, pick % LINES, a_off, p_off)
    err = angle_deg(pred, u[pick])
    print(f"fixture angle error deg p50/p90/p99 {np.percentile(err, [50, 90, 99]).round(3)}")
    samples = [
        {
            "firing": int(i // LINES),
            "line": int(i % LINES),
            "rotor_phase_offset_rad": round(float(a), 9),
            "nod_phase_offset_rad": round(float(p), 9),
            "direction": [round(float(x), 7) for x in u[i]],
        }
        for i, a, p in zip(pick, a_off, p_off)
    ]
    fixture = {
        "description": "Held-out real Mid-360 returns (EIL_Mask2 frames 150-299, a recording not used "
        "for the shape fit) with their tracked per-frame phase offsets.",
        "reference_median_error_deg": round(float(np.median(err)), 4),
        "samples": samples,
    }
    path = ROOT / "crates/rne_sensor/tests/fixtures/livox_mid360_eil_mask2_directions.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(fixture) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("extract")
    e.add_argument("bag")
    e.add_argument("out")
    e.add_argument("--frames", type=int, default=400)
    f = sub.add_parser("fit")
    f.add_argument("data")
    f.add_argument("out")
    f.add_argument("--train-frames", type=int, default=300)
    f.add_argument("--iterations", type=int, default=3)
    t = sub.add_parser("track")
    t.add_argument("data")
    t.add_argument("model")
    t.add_argument("out")
    b = sub.add_parser("build")
    b.add_argument("box")
    b.add_argument("mask2")
    b.add_argument("model")
    b.add_argument("mask2_track")
    args = ap.parse_args()
    if args.cmd == "extract":
        extract(args.bag, args.out, args.frames)
    elif args.cmd == "fit":
        fit(args.data, args.out, args.train_frames, args.iterations)
    elif args.cmd == "track":
        track(args.data, args.model, args.out)
    else:
        build(args.box, args.mask2, args.model, args.mask2_track)


if __name__ == "__main__":
    main()
