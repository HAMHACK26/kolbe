use bevy::prelude::*;

/// Every antenna carried by one radio node — a drone or the base.
///
/// Antennas live in their own component, not on [`crate::drone::Drone`] or
/// [`crate::base::Base`], because the radio behaves the same either way: link
/// detection, ranging, the mesh table and the link rendering all run over
/// "things with antennas", so a base takes part in the mesh on exactly the
/// same code path a drone does. What differs is only *aiming policy* — a drone
/// tracks its two ring neighbors, a base covers the whole formation — and the
/// color the links are drawn in.
#[derive(Component, Clone, Default)]
pub struct Antennas(pub Vec<Antenna>);

/// Physical parameters for a single directional antenna.
///
/// `gain_db`, `path_loss_db`, and `rssi_dbm` are intentionally separate so
/// callers can compose them independently. Never pre-fuse angle and range.
#[derive(Clone)]
pub struct Antenna {
    /// Yaw relative to the owning drone's own heading (0 = drone's forward,
    /// clockwise) — a drone's antennas turn with it. Bases have no heading, so
    /// their antennas' azimuth is effectively already world-frame.
    pub azimuth_deg: f32,
    /// Pitch above the horizon, world-frame. Every antenna shares the same
    /// "up" (gravity), so unlike azimuth this needs no drone-relative frame.
    pub elevation_deg: f32,

    // Gain pattern — 3GPP TR 38.901 parabolic-in-dB
    pub g_peak_dbi: f32,
    pub theta_3db_deg: f32,
    pub floor_db: f32,

    // Link budget
    pub p_tx_dbm: f32,
    pub frequency_mhz: f32,
    pub alpha_db_per_km: f32,

    // Receive side (device proxy)
    pub g_rx_dbi: f32,
    pub sensitivity_dbm: f32,
}

impl Antenna {
    /// G(θ) — 3GPP parabolic approximation. Pure angle term, no range.
    pub fn gain_db(&self, theta_deg: f32) -> f32 {
        let reduction = -12.0 * (theta_deg / self.theta_3db_deg).powi(2);
        self.g_peak_dbi + reduction.max(self.floor_db)
    }

    /// L(d) — Friis free-space + linear atmospheric absorption. Pure range term.
    /// d in km, frequency in MHz.
    pub fn path_loss_db(&self, distance_km: f32) -> f32 {
        20.0 * distance_km.max(f32::EPSILON).log10()
            + 20.0 * self.frequency_mhz.log10()
            + 32.44
            + self.alpha_db_per_km * distance_km
    }

    /// RSSI = P_tx + G(θ_tx) + G(θ_rx) − L(d).
    /// theta_rx in signature for future conscan / peer-gain composition;
    /// receive gain is fixed (g_rx_dbi) until a real peer exists.
    pub fn rssi_dbm(&self, theta_tx_deg: f32, _theta_rx_deg: f32, distance_km: f32) -> f32 {
        self.p_tx_dbm
            + self.gain_db(theta_tx_deg)
            + self.g_rx_dbi
            - self.path_loss_db(distance_km)
    }

    /// Range at which rssi_dbm(0, 0, d) == sensitivity_dbm.
    /// Solved by bisection because α·d makes the equation transcendental.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn max_range_km(&self) -> f32 {
        let target =
            self.p_tx_dbm + self.g_peak_dbi + self.g_rx_dbi - self.sensitivity_dbm;
        let mut lo = 1e-3_f32;
        let mut hi = 1e4_f32;
        for _ in 0..64 {
            let mid = (lo + hi) / 2.0;
            if self.path_loss_db(mid) < target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (lo + hi) / 2.0
    }

    /// World-frame azimuth: `azimuth_deg` (drone-relative) plus the owning
    /// drone's heading. Pass `0.0` for a base, which has no heading.
    pub fn world_azimuth_deg(&self, heading_deg: f32) -> f32 {
        (self.azimuth_deg + heading_deg).rem_euclid(360.0)
    }

    /// θ = acos(boresight · normalize(peer − self)).
    /// `heading_deg` is the owning drone's heading (0.0 for a base), needed to
    /// turn this antenna's drone-relative azimuth into a world direction.
    /// One dot-product, one acos.
    pub fn off_boresight_deg(&self, heading_deg: f32, self_pos: Vec3, peer_pos: Vec3) -> f32 {
        let boresight = radar_direction(self.world_azimuth_deg(heading_deg), self.elevation_deg);
        let to_peer = (peer_pos - self_pos).normalize_or_zero();
        boresight.dot(to_peer).clamp(-1.0, 1.0).acos().to_degrees()
    }
}

/// Unit vector from azimuth (°, clockwise from +Z) and elevation (°, up).
pub fn radar_direction(azimuth_deg: f32, elevation_deg: f32) -> Vec3 {
    let az = azimuth_deg.to_radians();
    let el = elevation_deg.to_radians();
    Vec3::new(el.cos() * az.sin(), el.sin(), el.cos() * az.cos()).normalize()
}

/// Inverse of `radar_direction`: (azimuth_deg in [0,360), elevation_deg) that
/// points a boresight from `from` toward `to`. Azimuth clockwise from +Z.
pub fn angles_toward(from: Vec3, to: Vec3) -> (f32, f32) {
    let d = (to - from).normalize_or_zero();
    let azimuth_deg = d.x.atan2(d.z).to_degrees().rem_euclid(360.0);
    let elevation_deg = d.y.clamp(-1.0, 1.0).asin().to_degrees();
    (azimuth_deg, elevation_deg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drone::make_antenna;

    /// Gain peaks dead on boresight, falls off with angle, and never drops
    /// below the pattern floor.
    #[test]
    fn gain_peaks_at_boresight_and_floors() {
        let a = make_antenna(0.0, 0.0, 0);
        assert!((a.gain_db(0.0) - a.g_peak_dbi).abs() < 1e-6);
        assert!(a.gain_db(0.5) < a.gain_db(0.0));
        // Far off-axis clamps to peak + floor.
        assert!((a.gain_db(90.0) - (a.g_peak_dbi + a.floor_db)).abs() < 1e-6);
    }

    /// Path loss grows monotonically with distance.
    #[test]
    fn path_loss_increases_with_distance() {
        let a = make_antenna(0.0, 0.0, 0);
        assert!(a.path_loss_db(1.0) < a.path_loss_db(2.0));
        assert!(a.path_loss_db(2.0) < a.path_loss_db(10.0));
    }

    /// Off-boresight angle is 0 when aimed straight at the target, 180 when
    /// it's directly behind.
    #[test]
    fn off_boresight_zero_when_aimed_at_target() {
        let a = make_antenna(90.0, 0.0, 0); // points +X
        assert!(a.off_boresight_deg(0.0, Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0)) < 0.01);
        assert!((a.off_boresight_deg(0.0, Vec3::ZERO, Vec3::new(-5.0, 0.0, 0.0)) - 180.0).abs() < 0.01);
    }

    /// `max_range_km` is exactly where a boresight link's RSSI meets
    /// sensitivity.
    #[test]
    fn max_range_is_where_rssi_hits_sensitivity() {
        let a = make_antenna(0.0, 0.0, 0);
        let rssi = a.rssi_dbm(0.0, 0.0, a.max_range_km());
        assert!((rssi - a.sensitivity_dbm).abs() < 0.1, "rssi {rssi} vs sens {}", a.sensitivity_dbm);
    }

    /// `angles_toward` is the exact inverse of `radar_direction`.
    #[test]
    fn angles_toward_inverts_radar_direction() {
        let from = Vec3::new(1.0, 2.0, -3.0);
        for to in [
            Vec3::new(5.0, 2.0, -3.0),
            Vec3::new(1.0, 7.0, -3.0),
            Vec3::new(-4.0, -1.0, 2.0),
        ] {
            let (az, el) = angles_toward(from, to);
            let dir = radar_direction(az, el);
            let expect = (to - from).normalize();
            assert!((dir - expect).length() < 1e-4, "dir {dir:?} vs {expect:?}");
        }
    }
}
