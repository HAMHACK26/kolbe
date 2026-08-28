//! Global spherical vector: two angles + a length.
//!
//! Direction is expressed the same way antennas are aimed — azimuth clockwise
//! from +Z, elevation up — so it composes directly with `radar_direction`.

use bevy::prelude::*;

use crate::antenna::{angles_toward, radar_direction};

/// A vector given as (azimuth°, elevation°, length). Length carries whatever
/// unit the caller uses (km here).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphericalVec {
    pub azimuth_deg: f32,
    pub elevation_deg: f32,
    pub length: f32,
}

impl SphericalVec {
    pub fn new(azimuth_deg: f32, elevation_deg: f32, length: f32) -> Self {
        Self { azimuth_deg, elevation_deg, length }
    }

    /// From a cartesian offset: angles from its direction, length from its norm.
    pub fn from_cartesian(v: Vec3) -> Self {
        let (azimuth_deg, elevation_deg) = angles_toward(Vec3::ZERO, v);
        Self { azimuth_deg, elevation_deg, length: v.length() }
    }

    /// Angles pointing `from` → `to`, with a caller-supplied length (e.g. a
    /// distance measured by timing rather than by the geometric gap).
    pub fn toward(from: Vec3, to: Vec3, length: f32) -> Self {
        let (azimuth_deg, elevation_deg) = angles_toward(from, to);
        Self { azimuth_deg, elevation_deg, length }
    }

    /// Back to a cartesian vector of magnitude `length`.
    pub fn to_cartesian(self) -> Vec3 {
        radar_direction(self.azimuth_deg, self.elevation_deg) * self.length
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `from_cartesian` → `to_cartesian` reproduces the original vector.
    #[test]
    fn cartesian_round_trip() {
        for v in [
            Vec3::new(3.0, 0.0, 4.0),
            Vec3::new(-2.0, 5.0, 1.0),
            Vec3::new(0.0, 0.0, -7.0),
        ] {
            let back = SphericalVec::from_cartesian(v).to_cartesian();
            assert!((back - v).length() < 1e-3, "{back:?} vs {v:?}");
        }
    }

    /// `toward` takes the direction from the two points but the length from
    /// the caller (not the geometric gap).
    #[test]
    fn toward_uses_given_length_and_pointing() {
        let sv = SphericalVec::toward(Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 5.0), 10.0);
        assert!((sv.length - 10.0).abs() < 1e-6);
        let dir = sv.to_cartesian().normalize();
        assert!((dir - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-4, "dir {dir:?}");
    }
}
