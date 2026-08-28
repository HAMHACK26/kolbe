use bevy::prelude::*;

/// Physical parameters for a single directional antenna.
///
/// `gain_db`, `path_loss_db`, and `rssi_dbm` are intentionally separate so
/// callers can compose them independently. Never pre-fuse angle and range.
#[derive(Clone)]
pub struct Antenna {
    pub azimuth_deg: f32,
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

    /// θ = acos(boresight · normalize(peer − self)).
    /// One dot-product, one acos.
    pub fn off_boresight_deg(&self, self_pos: Vec3, peer_pos: Vec3) -> f32 {
        let boresight = radar_direction(self.azimuth_deg, self.elevation_deg);
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
