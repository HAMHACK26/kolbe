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
    #[allow(dead_code)]
    pub fn new(azimuth_deg: f32, elevation_deg: f32, length: f32) -> Self {
        Self { azimuth_deg, elevation_deg, length }
    }

    /// From a cartesian offset: angles from its direction, length from its norm.
    #[cfg_attr(not(test), allow(dead_code))]
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
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn to_cartesian(self) -> Vec3 {
        radar_direction(self.azimuth_deg, self.elevation_deg) * self.length
    }
}
