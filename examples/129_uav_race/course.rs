//! The race course: gates in the air, a closed path through their centres,
//! and the fastest speed a drone can hold along it.

use rne_math::Vec3;

/// Gate centres, flown in this order. Heights change from gate to gate.
pub(crate) const GATES: [(f64, f64, f64); 8] = [
    (12.0, 2.2, 0.0),
    (30.0, 3.4, 8.0),
    (34.0, 6.0, 26.0),
    (20.0, 7.5, 36.0),
    (2.0, 3.0, 32.0),
    (-14.0, 2.2, 22.0),
    (-20.0, 4.5, 6.0),
    (-6.0, 2.4, -4.0),
];
/// Side of the square gate opening, in meters.
pub(crate) const GATE_OPENING_M: f64 = 2.4;
const STEP_M: f64 = 0.1;

/// The closed flight path through the gate centres.
pub(crate) struct Course {
    pub(crate) points: Vec<Vec3>,
    pub(crate) tangent: Vec<Vec3>,
    /// Horizontal unit vector to the left of the path.
    pub(crate) left: Vec<Vec3>,
    /// Unit vector perpendicular to both, pointing up-ish.
    pub(crate) up: Vec<Vec3>,
    pub(crate) curvature: Vec<f64>,
    /// Path sample nearest each gate.
    pub(crate) gate_index: Vec<usize>,
}

impl Course {
    pub(crate) fn build() -> Self {
        let control: Vec<Vec3> = GATES
            .iter()
            .map(|(x, y, z)| Vec3::new(*x, *y, *z))
            .collect();
        let dense = catmull_rom_closed(&control, 300);
        let points = resample_closed(&dense, STEP_M);
        let n = points.len();
        let tangent: Vec<Vec3> = (0..n)
            .map(|i| (points[(i + 1) % n] - points[(i + n - 1) % n]).normalize())
            .collect();
        let left: Vec<Vec3> = tangent
            .iter()
            .map(|t| Vec3::Y.cross(*t).normalize_or_zero())
            .collect();
        let up: Vec<Vec3> = tangent
            .iter()
            .zip(&left)
            .map(|(t, l)| t.cross(*l).normalize_or_zero())
            .collect();
        let curvature = (0..n)
            .map(|i| {
                let span = 20;
                let a = points[(i + n - span) % n];
                let b = points[i];
                let c = points[(i + span) % n];
                let (ab, bc, ca) = ((b - a).length(), (c - b).length(), (a - c).length());
                2.0 * (b - a).cross(c - b).length() / (ab * bc * ca).max(1e-12)
            })
            .collect();
        let gate_index = control
            .iter()
            .map(|gate| {
                (0..n)
                    .min_by(|a, b| {
                        (points[*a] - *gate)
                            .length()
                            .total_cmp(&(points[*b] - *gate).length())
                    })
                    .expect("points")
            })
            .collect();
        Self {
            points,
            tangent,
            left,
            up,
            curvature,
            gate_index,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.points.len()
    }

    pub(crate) fn length_m(&self) -> f64 {
        self.len() as f64 * STEP_M
    }

    /// The point `lane` (left, up) off the path at sample `index`.
    pub(crate) fn at(&self, index: usize, lane: (f64, f64)) -> Vec3 {
        let i = index % self.len();
        self.points[i] + self.left[i] * lane.0 + self.up[i] * lane.1
    }

    /// Nearest sample to `position` within `window` samples of `hint`.
    pub(crate) fn project(&self, position: Vec3, hint: usize, window: usize) -> usize {
        let n = self.len();
        (0..=2 * window)
            .map(|delta| (hint + n + delta - window) % n)
            .min_by(|a, b| {
                (self.points[*a] - position)
                    .length()
                    .total_cmp(&(self.points[*b] - position).length())
            })
            .expect("window")
    }

    /// The fastest speed at each sample for a drone that can turn at
    /// `lateral_m_s2` and speed up or slow down at `longitudinal_m_s2`.
    pub(crate) fn speed_profile(
        &self,
        lateral_m_s2: f64,
        longitudinal_m_s2: f64,
        top_m_s: f64,
    ) -> Vec<f64> {
        let n = self.len();
        let mut speed: Vec<f64> = self
            .curvature
            .iter()
            .map(|kappa| (lateral_m_s2 / kappa.max(1e-6)).sqrt().min(top_m_s))
            .collect();
        let remaining = |v: f64, i: usize| {
            let used = (v * v * self.curvature[i] / lateral_m_s2).min(1.0);
            (1.0 - used * used).max(0.0).sqrt()
        };
        for _ in 0..2 {
            for k in 0..n {
                let (i, j) = (k, (k + 1) % n);
                let reach = (speed[i].powi(2)
                    + 2.0 * longitudinal_m_s2 * remaining(speed[i], i) * STEP_M)
                    .sqrt();
                speed[j] = speed[j].min(reach);
            }
            for k in (0..n).rev() {
                let (i, j) = (k, (k + 1) % n);
                let reach = (speed[j].powi(2)
                    + 2.0 * longitudinal_m_s2 * remaining(speed[j], j) * STEP_M)
                    .sqrt();
                speed[i] = speed[i].min(reach);
            }
        }
        speed
    }
}

fn catmull_rom_closed(points: &[Vec3], per_segment: usize) -> Vec<Vec3> {
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
            out.push(
                (p1 * 2.0
                    + (p2 - p0) * t
                    + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2
                    + (p1 * 3.0 - p0 - p2 * 3.0 + p3) * t3)
                    * 0.5,
            );
        }
    }
    out
}

fn resample_closed(points: &[Vec3], step: f64) -> Vec<Vec3> {
    let n = points.len();
    let mut lengths = vec![0.0];
    for i in 0..n {
        let next = lengths[i] + (points[(i + 1) % n] - points[i]).length();
        lengths.push(next);
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
        out.push(points[segment] + (points[(segment + 1) % n] - points[segment]) * t);
    }
    out
}
