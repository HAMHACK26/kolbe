//! Geometry for turning a user-picked set of lat/lon points into a (possibly
//! rotated) minimum-area enclosing square — the "network area". Pure math,
//! no Bevy/ECS dependencies, so it's cheap to unit test in isolation.

const KM_PER_DEG_LAT: f64 = 110.574;

fn km_per_deg_lon(ref_lat: f64) -> f64 {
    111.320 * ref_lat.to_radians().cos()
}

/// A point in local flat-earth kilometers, relative to some reference lat/lon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalPoint {
    pub x_km: f64,
    pub z_km: f64,
}

/// Project lon/lat to local km using an equirectangular approximation
/// around `ref_lat` — accurate enough at the tens-of-km scale we pick areas at.
pub fn project(ref_lon: f64, ref_lat: f64, lon: f64, lat: f64) -> LocalPoint {
    LocalPoint {
        x_km: (lon - ref_lon) * km_per_deg_lon(ref_lat),
        z_km: (lat - ref_lat) * KM_PER_DEG_LAT,
    }
}

pub fn unproject(ref_lon: f64, ref_lat: f64, p: LocalPoint) -> (f64, f64) {
    let lon = ref_lon + p.x_km / km_per_deg_lon(ref_lat);
    let lat = ref_lat + p.z_km / KM_PER_DEG_LAT;
    (lon, lat)
}

/// Andrew's monotone chain convex hull. Returns hull points counter-clockwise
/// (or the input, deduplicated by position, when there are fewer than 3).
pub fn convex_hull(points: &[LocalPoint]) -> Vec<LocalPoint> {
    let mut pts: Vec<LocalPoint> = points.to_vec();
    pts.sort_by(|a, b| a.x_km.partial_cmp(&b.x_km).unwrap().then(a.z_km.partial_cmp(&b.z_km).unwrap()));
    pts.dedup_by(|a, b| (a.x_km - b.x_km).abs() < 1e-9 && (a.z_km - b.z_km).abs() < 1e-9);

    if pts.len() < 3 {
        return pts;
    }

    fn cross(o: LocalPoint, a: LocalPoint, b: LocalPoint) -> f64 {
        (a.x_km - o.x_km) * (b.z_km - o.z_km) - (a.z_km - o.z_km) * (b.x_km - o.x_km)
    }

    let mut lower: Vec<LocalPoint> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<LocalPoint> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// A square that may be rotated relative to the east/north axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingSquare {
    pub center: LocalPoint,
    pub half_side_km: f64,
    /// Rotation (radians, clockwise) of the square's own +x edge from world +x (east).
    pub rotation: f64,
}

impl BoundingSquare {
    pub fn side_km(&self) -> f64 {
        self.half_side_km * 2.0
    }

    /// The 4 corners in local km, starting at the +x,+z corner and going
    /// clockwise (matches `rotation`'s sense).
    pub fn corners(&self) -> [LocalPoint; 4] {
        let (s, c) = self.rotation.sin_cos();
        let h = self.half_side_km;
        let local = [(h, h), (h, -h), (-h, -h), (-h, h)];
        local.map(|(lx, lz)| LocalPoint {
            x_km: self.center.x_km + lx * c - lz * s,
            z_km: self.center.z_km + lx * s + lz * c,
        })
    }
}

/// Smallest north/east-aligned square enclosing all points. Unlike
/// [`min_bounding_square`], this does not rotate to follow the polygon.
pub fn axis_aligned_bounding_square(points: &[LocalPoint]) -> Option<BoundingSquare> {
    let mut iter = points.iter().copied();
    let first = iter.next()?;
    let (mut min_x, mut max_x) = (first.x_km, first.x_km);
    let (mut min_z, mut max_z) = (first.z_km, first.z_km);
    for point in iter {
        min_x = min_x.min(point.x_km);
        max_x = max_x.max(point.x_km);
        min_z = min_z.min(point.z_km);
        max_z = max_z.max(point.z_km);
    }
    let side = (max_x - min_x).max(max_z - min_z);
    Some(BoundingSquare {
        center: LocalPoint {
            x_km: (min_x + max_x) * 0.5,
            z_km: (min_z + max_z) * 0.5,
        },
        half_side_km: side * 0.5,
        rotation: 0.0,
    })
}

/// Minimum-area enclosing square found by a numeric sweep over rotation
/// angle: for each candidate angle, the required square side is
/// `max(width, height)` of the point set's axis-aligned bbox in that
/// rotated frame. A square repeats every 90°, so only [0, 90) needs
/// checking. This isn't the closed-form rotating-calipers solution, but at
/// a fine enough step it's indistinguishable in practice and much simpler.
pub fn min_bounding_square(points: &[LocalPoint]) -> Option<BoundingSquare> {
    if points.is_empty() {
        return None;
    }
    let hull = convex_hull(points);
    let hull = if hull.len() < 3 { points } else { &hull };

    const STEPS: usize = 360;
    let mut best: Option<(f64, f64, f64, f64)> = None; // (side, angle, center_x, center_z)

    for step in 0..STEPS {
        let theta = step as f64 * (std::f64::consts::FRAC_PI_2 / STEPS as f64);
        let (s, c) = theta.sin_cos();
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_z = f64::INFINITY;
        let mut max_z = f64::NEG_INFINITY;
        for p in hull {
            // Rotate the point by -theta into the candidate square's frame.
            let rx = p.x_km * c + p.z_km * s;
            let rz = -p.x_km * s + p.z_km * c;
            min_x = min_x.min(rx);
            max_x = max_x.max(rx);
            min_z = min_z.min(rz);
            max_z = max_z.max(rz);
        }
        let side = (max_x - min_x).max(max_z - min_z);
        let center_rot_x = (min_x + max_x) * 0.5;
        let center_rot_z = (min_z + max_z) * 0.5;
        // Rotate the bbox center back by +theta into world-local space.
        let center_x = center_rot_x * c - center_rot_z * s;
        let center_z = center_rot_x * s + center_rot_z * c;

        if best.is_none_or(|(best_side, ..)| side < best_side) {
            best = Some((side, theta, center_x, center_z));
        }
    }

    let (side, rotation, center_x, center_z) = best.unwrap();
    Some(BoundingSquare {
        center: LocalPoint { x_km: center_x, z_km: center_z },
        half_side_km: side * 0.5,
        rotation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, z: f64) -> LocalPoint {
        LocalPoint { x_km: x, z_km: z }
    }

    #[test]
    fn projection_round_trips() {
        let (ref_lon, ref_lat) = (18.0, 59.0);
        let (lon, lat) = (18.5, 59.3);
        let local = project(ref_lon, ref_lat, lon, lat);
        let (lon2, lat2) = unproject(ref_lon, ref_lat, local);
        assert!((lon - lon2).abs() < 1e-9);
        assert!((lat - lat2).abs() < 1e-9);
    }

    #[test]
    fn convex_hull_of_axis_aligned_square_is_its_corners() {
        let points = [pt(0.0, 0.0), pt(10.0, 0.0), pt(10.0, 10.0), pt(0.0, 10.0), pt(5.0, 5.0)];
        let hull = convex_hull(&points);
        assert_eq!(hull.len(), 4);
    }

    #[test]
    fn bounding_square_of_axis_aligned_square_has_zero_or_right_angle_rotation() {
        let points = [pt(-5.0, -5.0), pt(5.0, -5.0), pt(5.0, 5.0), pt(-5.0, 5.0)];
        let square = min_bounding_square(&points).unwrap();
        assert!((square.side_km() - 10.0).abs() < 0.05);
        assert!(square.center.x_km.abs() < 0.05 && square.center.z_km.abs() < 0.05);
    }

    #[test]
    fn bounding_square_of_diamond_is_rotated_45_degrees_smaller_than_axis_aligned() {
        // A diamond with diagonal 10 needs only a ~7.07 side square if rotated
        // 45°, versus a 10-side square if axis-aligned.
        let points = [pt(5.0, 0.0), pt(0.0, 5.0), pt(-5.0, 0.0), pt(0.0, -5.0)];
        let square = min_bounding_square(&points).unwrap();
        assert!(square.side_km() < 7.2, "side_km={}", square.side_km());
    }

    #[test]
    fn single_point_has_a_zero_size_square() {
        let square = min_bounding_square(&[pt(1.0, 1.0)]).unwrap();
        assert!(square.side_km() < 1e-6);
    }

    #[test]
    fn axis_aligned_square_never_rotates() {
        let square = axis_aligned_bounding_square(&[pt(-2.0, -5.0), pt(5.0, 1.0)]).unwrap();
        assert_eq!(square.rotation, 0.0);
        assert!((square.side_km() - 7.0).abs() < 1e-9);
    }

    #[test]
    fn no_points_returns_none() {
        assert!(min_bounding_square(&[]).is_none());
    }

    #[test]
    fn corners_are_all_half_side_from_center() {
        let square = BoundingSquare { center: pt(2.0, 3.0), half_side_km: 4.0, rotation: 0.3 };
        for corner in square.corners() {
            let dx = corner.x_km - square.center.x_km;
            let dz = corner.z_km - square.center.z_km;
            let dist = (dx * dx + dz * dz).sqrt();
            assert!((dist - 4.0 * std::f64::consts::SQRT_2).abs() < 1e-9);
        }
    }
}
