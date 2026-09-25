//! Support polygon and static stability margin from measured ground contacts.
//!
//! A legged robot is statically stable while the ground projection of its
//! center of mass lies inside the convex hull of its ground contacts. This
//! module turns a set of measured contact points into that hull and reports the
//! signed distance from the projected center of mass to its boundary, so a fall
//! can be explained by where support was lost rather than only observed.
//!
//! Nothing here depends on a physics backend: contacts arrive as plain values,
//! whatever produced them.

use crate::horizontal::Horizontal;

/// Convex support polygon in the horizontal walking plane.
///
/// Vertices are the convex hull of the supplied contact points in
/// counter-clockwise order when viewed from above (`+X` right, `+Z` up in the
/// plane), starting from the lowest `(x, z)` vertex so the representation is
/// canonical and replay-stable. Degenerate support is represented honestly:
/// a single contact yields one vertex and collinear contacts yield two.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportPolygon {
    vertices: Vec<Horizontal>,
}

/// Static stability of one center-of-mass projection against a support polygon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StabilityMargin {
    /// Signed distance to the polygon boundary in meters.
    ///
    /// Positive inside the polygon, negative outside, and the magnitude is the
    /// distance to the nearest edge either way. Degenerate support (fewer than
    /// three hull vertices) can never be strictly inside, so the value is zero
    /// or negative there.
    pub signed_distance_m: f64,
    /// Whether the projection lies inside or on the polygon boundary.
    pub inside: bool,
}

/// Smallest area a hull may enclose before it is treated as degenerate, in m^2.
///
/// Contact points from a solver are never exactly collinear, so an absolute
/// floor keeps a support polygon of numerical noise from being reported as real
/// support.
const MIN_SUPPORT_AREA_M2: f64 = 1.0e-9;

/// Ordering that gives contact points a canonical, replay-stable sequence.
fn compare_points(a: &Horizontal, b: &Horizontal) -> std::cmp::Ordering {
    a.x_m
        .partial_cmp(&b.x_m)
        .expect("finite x")
        .then_with(|| a.z_m.partial_cmp(&b.z_m).expect("finite z"))
}

/// Twice the signed area of the triangle `o, a, b`, positive when
/// counter-clockwise.
fn cross(o: Horizontal, a: Horizontal, b: Horizontal) -> f64 {
    (a.x_m - o.x_m) * (b.z_m - o.z_m) - (a.z_m - o.z_m) * (b.x_m - o.x_m)
}

impl SupportPolygon {
    /// Builds the support polygon from measured contact points.
    ///
    /// Non-finite points are discarded rather than poisoning the hull. Points
    /// closer together than `merge_tolerance_m` collapse to one vertex, which
    /// keeps the many contact samples a solver reports for one foot from being
    /// treated as a polygon of their own. A negative or non-finite tolerance is
    /// treated as zero.
    pub fn from_contacts(contacts_m: &[Horizontal], merge_tolerance_m: f64) -> Self {
        let tolerance_m = if merge_tolerance_m.is_finite() && merge_tolerance_m > 0.0 {
            merge_tolerance_m
        } else {
            0.0
        };

        let mut sorted: Vec<Horizontal> = contacts_m
            .iter()
            .copied()
            .filter(|point| point.is_finite())
            .collect();
        sorted.sort_by(compare_points);

        // Greedy clustering against every kept representative, not just the
        // previous one: two feet can share an x coordinate, so their samples
        // interleave under the canonical sort and adjacent-only merging would
        // leave both clusters split. The contact count per step is small, so
        // the quadratic scan is not worth avoiding.
        let mut points: Vec<Horizontal> = Vec::with_capacity(sorted.len());
        for point in sorted {
            let merged = points.iter().any(|kept| {
                let dx = kept.x_m - point.x_m;
                let dz = kept.z_m - point.z_m;
                dx * dx + dz * dz <= tolerance_m * tolerance_m
            });
            if !merged {
                points.push(point);
            }
        }

        if points.len() < 3 {
            return Self { vertices: points };
        }

        // Monotone chain: lower then upper hull, both excluding their last
        // point because the other chain starts there.
        let mut lower: Vec<Horizontal> = Vec::with_capacity(points.len());
        for &point in &points {
            while lower.len() >= 2
                && cross(lower[lower.len() - 2], lower[lower.len() - 1], point) <= 0.0
            {
                lower.pop();
            }
            lower.push(point);
        }
        let mut upper: Vec<Horizontal> = Vec::with_capacity(points.len());
        for &point in points.iter().rev() {
            while upper.len() >= 2
                && cross(upper[upper.len() - 2], upper[upper.len() - 1], point) <= 0.0
            {
                upper.pop();
            }
            upper.push(point);
        }
        lower.pop();
        upper.pop();
        lower.extend(upper);

        if lower.len() < 3 {
            // Every point was collinear; report the extreme pair honestly.
            let first = *points.first().expect("non-empty");
            let last = *points.last().expect("non-empty");
            return Self {
                vertices: vec![first, last],
            };
        }
        Self { vertices: lower }
    }

    /// Returns the hull vertices in canonical counter-clockwise order.
    pub fn vertices(&self) -> &[Horizontal] {
        &self.vertices
    }

    /// Returns the enclosed area in square meters; zero for degenerate support.
    pub fn area_m2(&self) -> f64 {
        if self.vertices.len() < 3 {
            return 0.0;
        }
        let mut twice_area = 0.0;
        for index in 0..self.vertices.len() {
            let current = self.vertices[index];
            let next = self.vertices[(index + 1) % self.vertices.len()];
            twice_area += current.x_m * next.z_m - next.x_m * current.z_m;
        }
        (twice_area / 2.0).abs()
    }

    /// Returns whether the polygon encloses a usable support area.
    pub fn is_degenerate(&self) -> bool {
        self.vertices.len() < 3 || self.area_m2() <= MIN_SUPPORT_AREA_M2
    }

    /// Returns the static stability of one ground-projected center of mass.
    ///
    /// The sign convention is documented by [`StabilityMargin`]. For degenerate
    /// support the result is the negated distance to the supporting point or
    /// segment, because a point or a line cannot statically support a mass.
    pub fn stability_margin(&self, com_ground_m: Horizontal) -> StabilityMargin {
        if !com_ground_m.is_finite() || self.vertices.is_empty() {
            return StabilityMargin {
                signed_distance_m: f64::NEG_INFINITY,
                inside: false,
            };
        }
        if self.is_degenerate() {
            let distance_m = if self.vertices.len() == 1 {
                let offset = Horizontal::new(
                    com_ground_m.x_m - self.vertices[0].x_m,
                    com_ground_m.z_m - self.vertices[0].z_m,
                );
                offset.norm()
            } else {
                self.min_edge_distance_m(com_ground_m)
            };
            return StabilityMargin {
                signed_distance_m: -distance_m,
                inside: false,
            };
        }

        let inside = (0..self.vertices.len()).all(|index| {
            let current = self.vertices[index];
            let next = self.vertices[(index + 1) % self.vertices.len()];
            cross(current, next, com_ground_m) >= 0.0
        });
        let distance_m = self.min_edge_distance_m(com_ground_m);
        StabilityMargin {
            signed_distance_m: if inside { distance_m } else { -distance_m },
            inside,
        }
    }

    /// Distance from a point to the nearest polygon edge, in meters.
    fn min_edge_distance_m(&self, point: Horizontal) -> f64 {
        let mut nearest_m = f64::INFINITY;
        let edge_count = if self.vertices.len() == 2 {
            1
        } else {
            self.vertices.len()
        };
        for index in 0..edge_count {
            let start = self.vertices[index];
            let end = self.vertices[(index + 1) % self.vertices.len()];
            nearest_m = nearest_m.min(segment_distance_m(point, start, end));
        }
        nearest_m
    }
}

/// Distance from `point` to the segment `start`-`end`, in meters.
fn segment_distance_m(point: Horizontal, start: Horizontal, end: Horizontal) -> f64 {
    let edge = Horizontal::new(end.x_m - start.x_m, end.z_m - start.z_m);
    let offset = Horizontal::new(point.x_m - start.x_m, point.z_m - start.z_m);
    let edge_length_squared = edge.dot(edge);
    if edge_length_squared <= 0.0 {
        return offset.norm();
    }
    let projection = (offset.dot(edge) / edge_length_squared).clamp(0.0, 1.0);
    Horizontal::new(
        offset.x_m - projection * edge.x_m,
        offset.z_m - projection * edge.z_m,
    )
    .norm()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> SupportPolygon {
        SupportPolygon::from_contacts(
            &[
                Horizontal::new(-1.0, -1.0),
                Horizontal::new(1.0, -1.0),
                Horizontal::new(1.0, 1.0),
                Horizontal::new(-1.0, 1.0),
            ],
            0.0,
        )
    }

    #[test]
    fn hull_is_counter_clockwise_canonical_and_drops_interior_points() {
        let polygon = SupportPolygon::from_contacts(
            &[
                // Deliberately unordered, with an interior point and a duplicate.
                Horizontal::new(1.0, 1.0),
                Horizontal::new(0.0, 0.0),
                Horizontal::new(-1.0, 1.0),
                Horizontal::new(-1.0, -1.0),
                Horizontal::new(1.0, -1.0),
                Horizontal::new(1.0, -1.0),
            ],
            0.0,
        );
        assert_eq!(
            polygon.vertices(),
            &[
                Horizontal::new(-1.0, -1.0),
                Horizontal::new(1.0, -1.0),
                Horizontal::new(1.0, 1.0),
                Horizontal::new(-1.0, 1.0),
            ]
        );
        assert_eq!(polygon.area_m2(), 4.0);
        assert!(!polygon.is_degenerate());

        // Input order must not change the hull.
        let reversed = SupportPolygon::from_contacts(
            &[
                Horizontal::new(1.0, -1.0),
                Horizontal::new(-1.0, -1.0),
                Horizontal::new(-1.0, 1.0),
                Horizontal::new(0.0, 0.0),
                Horizontal::new(1.0, 1.0),
            ],
            0.0,
        );
        assert_eq!(reversed.vertices(), polygon.vertices());
    }

    #[test]
    fn stability_margin_is_signed_distance_to_the_nearest_edge() {
        let polygon = square();

        let center = polygon.stability_margin(Horizontal::new(0.0, 0.0));
        assert!(center.inside);
        assert!((center.signed_distance_m - 1.0).abs() < 1e-12);

        let near_edge = polygon.stability_margin(Horizontal::new(0.75, 0.0));
        assert!(near_edge.inside);
        assert!((near_edge.signed_distance_m - 0.25).abs() < 1e-12);

        let on_edge = polygon.stability_margin(Horizontal::new(1.0, 0.0));
        assert!(on_edge.inside, "a boundary projection still has support");
        assert!(on_edge.signed_distance_m.abs() < 1e-12);

        let outside = polygon.stability_margin(Horizontal::new(1.5, 0.0));
        assert!(!outside.inside);
        assert!((outside.signed_distance_m + 0.5).abs() < 1e-12);

        // Diagonally outside: nearest feature is the corner, not an edge line.
        let corner = polygon.stability_margin(Horizontal::new(2.0, 2.0));
        assert!(!corner.inside);
        assert!((corner.signed_distance_m + std::f64::consts::SQRT_2).abs() < 1e-12);
    }

    #[test]
    fn degenerate_support_is_never_reported_as_stable() {
        let single = SupportPolygon::from_contacts(&[Horizontal::new(0.0, 0.0)], 0.0);
        assert!(single.is_degenerate());
        assert_eq!(single.area_m2(), 0.0);
        let margin = single.stability_margin(Horizontal::new(0.3, 0.4));
        assert!(!margin.inside);
        assert!((margin.signed_distance_m + 0.5).abs() < 1e-12);
        // Even standing exactly on the single contact is not static support.
        assert!(!single.stability_margin(Horizontal::new(0.0, 0.0)).inside);

        let collinear = SupportPolygon::from_contacts(
            &[
                Horizontal::new(-1.0, 0.0),
                Horizontal::new(0.0, 0.0),
                Horizontal::new(1.0, 0.0),
            ],
            0.0,
        );
        assert!(collinear.is_degenerate());
        assert_eq!(collinear.vertices().len(), 2);
        let margin = collinear.stability_margin(Horizontal::new(0.0, 0.25));
        assert!(!margin.inside);
        assert!((margin.signed_distance_m + 0.25).abs() < 1e-12);

        let empty = SupportPolygon::from_contacts(&[], 0.0);
        assert!(empty.is_degenerate());
        let margin = empty.stability_margin(Horizontal::new(0.0, 0.0));
        assert!(!margin.inside);
        assert_eq!(margin.signed_distance_m, f64::NEG_INFINITY);
    }

    #[test]
    fn merge_tolerance_collapses_one_feet_worth_of_contact_samples() {
        // Three samples per foot at four quadruped foot positions. Left and
        // right feet share an x coordinate, so their samples interleave under
        // the canonical sort: merging must cluster against every kept
        // representative rather than only the previous one.
        let mut contacts = Vec::new();
        for (x_m, z_m) in [(-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1)] {
            for (dx, dz) in [(0.0, 0.0), (0.004, 0.002), (0.002, 0.004)] {
                contacts.push(Horizontal::new(x_m + dx, z_m + dz));
            }
        }
        let merged = SupportPolygon::from_contacts(&contacts, 0.01);
        assert_eq!(
            merged.vertices().len(),
            4,
            "one vertex per foot: {:?}",
            merged.vertices()
        );
        assert!(!merged.is_degenerate());

        // Without merging, each foot's spread contributes its own hull corners.
        let unmerged = SupportPolygon::from_contacts(&contacts, 0.0);
        assert!(
            unmerged.vertices().len() > 4,
            "unmerged hull should keep per-sample corners: {:?}",
            unmerged.vertices()
        );
        // Merging only shaves the millimetre-scale spread off the support area.
        assert!(unmerged.area_m2() >= merged.area_m2());
        assert!((unmerged.area_m2() - merged.area_m2()) / merged.area_m2() < 0.10);

        // A non-finite tolerance is treated as no merging rather than panicking.
        let nonfinite = SupportPolygon::from_contacts(&contacts, f64::NAN);
        assert_eq!(nonfinite.vertices(), unmerged.vertices());
    }

    #[test]
    fn nonfinite_contacts_are_discarded_rather_than_poisoning_the_hull() {
        let polygon = SupportPolygon::from_contacts(
            &[
                Horizontal::new(-1.0, -1.0),
                Horizontal::new(f64::NAN, 0.0),
                Horizontal::new(1.0, -1.0),
                Horizontal::new(0.0, f64::INFINITY),
                Horizontal::new(1.0, 1.0),
                Horizontal::new(-1.0, 1.0),
            ],
            0.0,
        );
        assert_eq!(polygon.vertices(), square().vertices());
        assert!(
            !polygon
                .stability_margin(Horizontal::new(f64::NAN, 0.0))
                .inside
        );
    }
}
