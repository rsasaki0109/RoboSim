//! The circuit: a closed centre line, its edges, a racing line inside them and
//! the speed a car can hold along it.
//!
//! Coordinates are the world's ground plane `(x, z)`.

/// Control points of the centre line, driven in this order.
const CONTROL: [(f64, f64); 14] = [
    (-150.0, 0.0),
    (0.0, 0.0),
    (150.0, 0.0),
    (198.0, 18.0),
    (206.0, 66.0),
    (170.0, 92.0),
    (126.0, 78.0),
    (92.0, 104.0),
    (18.0, 128.0),
    (-40.0, 118.0),
    (-122.0, 138.0),
    (-180.0, 122.0),
    (-208.0, 72.0),
    (-192.0, 20.0),
];
/// Track width in meters.
pub(crate) const WIDTH_M: f64 = 12.0;
/// Sample spacing along the centre line.
const STEP_M: f64 = 1.0;
const GRAVITY_M_S2: f64 = 9.81;

/// The sampled circuit.
pub(crate) struct Track {
    /// Centre line points, one per [`STEP_M`], closed (the last connects to the first).
    pub(crate) center: Vec<(f64, f64)>,
    /// Unit left normal at each centre point.
    pub(crate) normal: Vec<(f64, f64)>,
    /// Signed curvature of the centre line, positive turning left.
    pub(crate) curvature: Vec<f64>,
    /// Lap length in meters.
    pub(crate) length_m: f64,
}

impl Track {
    pub(crate) fn build() -> Self {
        let dense = catmull_rom_closed(&CONTROL, 400);
        let center = resample_closed(&dense, STEP_M);
        let n = center.len();
        let mut normal = Vec::with_capacity(n);
        for i in 0..n {
            let (x0, z0) = center[(i + n - 1) % n];
            let (x1, z1) = center[(i + 1) % n];
            let (dx, dz) = (x1 - x0, z1 - z0);
            let length = dx.hypot(dz);
            normal.push((-dz / length, dx / length));
        }
        let curvature = curvature_closed(&center);
        Self {
            length_m: n as f64 * STEP_M,
            center,
            normal,
            curvature,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.center.len()
    }

    /// The point `offset_m` to the left of the centre line at sample `index`.
    pub(crate) fn at(&self, index: usize, offset_m: f64) -> (f64, f64) {
        let index = index % self.len();
        let (x, z) = self.center[index];
        let (nx, nz) = self.normal[index];
        (x + nx * offset_m, z + nz * offset_m)
    }

    /// The nearest centre sample to `(x, z)`, searched within `window` samples
    /// of `hint`, and the signed lateral offset from the centre line there.
    pub(crate) fn project(&self, x: f64, z: f64, hint: usize, window: usize) -> (usize, f64) {
        let n = self.len();
        let mut best = (hint % n, f64::INFINITY);
        for delta in 0..=2 * window {
            let index = (hint + n + delta - window) % n;
            let (cx, cz) = self.center[index];
            let distance = (x - cx).hypot(z - cz);
            if distance < best.1 {
                best = (index, distance);
            }
        }
        let index = best.0;
        let (cx, cz) = self.center[index];
        let (nx, nz) = self.normal[index];
        (index, (x - cx) * nx + (z - cz) * nz)
    }
}

fn catmull_rom_closed(points: &[(f64, f64)], per_segment: usize) -> Vec<(f64, f64)> {
    let n = points.len();
    let mut out = Vec::with_capacity(n * per_segment);
    for i in 0..n {
        let p0 = points[(i + n - 1) % n];
        let p1 = points[i];
        let p2 = points[(i + 1) % n];
        let p3 = points[(i + 2) % n];
        for step in 0..per_segment {
            let t = step as f64 / per_segment as f64;
            let (t2, t3) = (t * t, t * t * t);
            let blend = |a: f64, b: f64, c: f64, d: f64| {
                0.5 * (2.0 * b
                    + (-a + c) * t
                    + (2.0 * a - 5.0 * b + 4.0 * c - d) * t2
                    + (-a + 3.0 * b - 3.0 * c + d) * t3)
            };
            out.push((blend(p0.0, p1.0, p2.0, p3.0), blend(p0.1, p1.1, p2.1, p3.1)));
        }
    }
    out
}

/// Points spaced `step` apart along a closed polyline.
fn resample_closed(points: &[(f64, f64)], step: f64) -> Vec<(f64, f64)> {
    let n = points.len();
    let mut lengths = Vec::with_capacity(n + 1);
    lengths.push(0.0);
    for i in 0..n {
        let (a, b) = (points[i], points[(i + 1) % n]);
        lengths.push(lengths[i] + (b.0 - a.0).hypot(b.1 - a.1));
    }
    let total = lengths[n];
    let count = (total / step).round() as usize;
    let spacing = total / count as f64;
    let mut out = Vec::with_capacity(count);
    let mut segment = 0;
    for k in 0..count {
        let s = k as f64 * spacing;
        while lengths[segment + 1] < s {
            segment += 1;
        }
        let t = (s - lengths[segment]) / (lengths[segment + 1] - lengths[segment]);
        let (a, b) = (points[segment], points[(segment + 1) % n]);
        out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
    }
    out
}

/// Signed curvature of a closed polyline by the circle through each point and
/// its two neighbours; positive turns left.
pub(crate) fn curvature_closed(points: &[(f64, f64)]) -> Vec<f64> {
    let n = points.len();
    (0..n)
        .map(|i| {
            let a = points[(i + n - 1) % n];
            let b = points[i];
            let c = points[(i + 1) % n];
            let cross = (b.0 - a.0) * (c.1 - b.1) - (b.1 - a.1) * (c.0 - b.0);
            let ab = (b.0 - a.0).hypot(b.1 - a.1);
            let bc = (c.0 - b.0).hypot(c.1 - b.1);
            let ca = (a.0 - c.0).hypot(a.1 - c.1);
            2.0 * cross / (ab * bc * ca).max(1e-12)
        })
        .collect()
}

/// A line around the lap: its lateral offset from the centre line at every
/// sample, its points, curvature and the fastest speed held along it.
pub(crate) struct RacingLine {
    pub(crate) offset: Vec<f64>,
    pub(crate) points: Vec<(f64, f64)>,
    pub(crate) curvature: Vec<f64>,
}

impl RacingLine {
    /// The minimum-curvature line within `margin_m` of the edges: each point is
    /// pulled toward the midpoint of its neighbours along the track normal,
    /// then clamped to the track. Iterated to convergence, this straightens
    /// what the track allows and uses its width through the corners.
    pub(crate) fn minimum_curvature(track: &Track, margin_m: f64) -> Self {
        let n = track.len();
        let limit = 0.5 * WIDTH_M - margin_m;
        let mut offset = vec![0.0; n];
        for _ in 0..4_000 {
            let points: Vec<(f64, f64)> = (0..n).map(|i| track.at(i, offset[i])).collect();
            for i in 0..n {
                let (ax, az) = points[(i + n - 1) % n];
                let (cx, cz) = points[(i + 1) % n];
                let (px, pz) = points[i];
                let (mx, mz) = (0.5 * (ax + cx), 0.5 * (az + cz));
                let (nx, nz) = track.normal[i];
                let pull = (mx - px) * nx + (mz - pz) * nz;
                offset[i] = (offset[i] + 0.8 * pull).clamp(-limit, limit);
            }
        }
        Self::from_offsets(track, offset)
    }

    pub(crate) fn from_offsets(track: &Track, offset: Vec<f64>) -> Self {
        let points: Vec<(f64, f64)> = (0..track.len()).map(|i| track.at(i, offset[i])).collect();
        let curvature = curvature_closed(&points);
        Self {
            offset,
            points,
            curvature,
        }
    }

    /// The fastest speed at each sample for a car that can use `lateral_m_s2`
    /// of cornering grip, accelerate at `accel_m_s2` and brake at
    /// `brake_m_s2`, up to `top_m_s`. Longitudinal and lateral demands share
    /// one friction circle.
    pub(crate) fn speed_profile(
        &self,
        lateral_m_s2: f64,
        accel_m_s2: f64,
        brake_m_s2: f64,
        top_m_s: f64,
    ) -> Vec<f64> {
        let n = self.points.len();
        let mut speed: Vec<f64> = self
            .curvature
            .iter()
            .map(|kappa| (lateral_m_s2 / kappa.abs().max(1e-6)).sqrt().min(top_m_s))
            .collect();
        let spacing = |i: usize| {
            let (a, b) = (self.points[i], self.points[(i + 1) % n]);
            (b.0 - a.0).hypot(b.1 - a.1)
        };
        let remaining = |v: f64, i: usize| {
            let used = (v * v * self.curvature[i].abs() / lateral_m_s2).min(1.0);
            (1.0 - used * used).max(0.0).sqrt()
        };
        // Two sweeps round the closed lap settle both passes.
        for _ in 0..2 {
            for k in 0..n {
                let (i, j) = (k, (k + 1) % n);
                let reachable = (speed[i].powi(2)
                    + 2.0 * accel_m_s2 * remaining(speed[i], i) * spacing(i))
                .sqrt();
                speed[j] = speed[j].min(reachable);
            }
            for k in (0..n).rev() {
                let (i, j) = (k, (k + 1) % n);
                let reachable = (speed[j].powi(2)
                    + 2.0 * brake_m_s2 * remaining(speed[j], j) * spacing(i))
                .sqrt();
                speed[i] = speed[i].min(reachable);
            }
        }
        speed
    }
}

/// Peak lateral acceleration the friction model allows, in m/s².
pub(crate) fn grip_m_s2(friction_coefficient: f64) -> f64 {
    friction_coefficient * GRAVITY_M_S2
}
