use bevy::prelude::*;

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

    /// Conical-scan squint: how far off its nominal boresight the receive
    /// pattern nutates. Half the 3dB beamwidth is the classic choice — enough
    /// slope in `gain_db` for the scan's amplitude modulation to be readable,
    /// without giving up so much on-axis sensitivity that a well-aimed link
    /// starts to suffer.
    pub fn squint_deg(&self) -> f32 {
        self.theta_3db_deg * 0.5
    }

    /// World-frame (azimuth, elevation) of the antenna's boresight nutated to
    /// conical-scan phase `phase_rad` (0 = start of the sweep). Traces a
    /// circle of radius `squint_deg()` around the antenna's nominal boresight
    /// once per `TAU` of phase — see [`ConicalScan`] for why.
    pub fn scanned_boresight_deg(&self, heading_deg: f32, phase_rad: f32) -> (f32, f32) {
        let squint = self.squint_deg();
        let az = self.world_azimuth_deg(heading_deg) + squint * phase_rad.cos();
        let el = self.elevation_deg + squint * phase_rad.sin();
        (az.rem_euclid(360.0), el)
    }

    /// Gain toward `peer_pos` with the boresight nutated to conical-scan
    /// phase `phase_rad`, i.e. what a conscan receiver actually samples each
    /// instant — as opposed to `gain_db(off_boresight_deg(..))`, which is the
    /// (unrealistic, currently used everywhere else) gain of an antenna held
    /// dead-still on its nominal boresight.
    pub fn scanned_gain_db(
        &self,
        heading_deg: f32,
        phase_rad: f32,
        self_pos: Vec3,
        peer_pos: Vec3,
    ) -> f32 {
        let (az, el) = self.scanned_boresight_deg(heading_deg, phase_rad);
        let boresight = radar_direction(az, el);
        let to_peer = (peer_pos - self_pos).normalize_or_zero();
        let theta_deg = boresight.dot(to_peer).clamp(-1.0, 1.0).acos().to_degrees();
        self.gain_db(theta_deg)
    }
}

/// Nutation rate for conical scan — the antenna sweeps one full
/// `squint_deg()` circle around its nominal boresight this many times a
/// second. 30Hz is fast enough to track a maneuvering drone's pointing error
/// without the sweep itself being confused for drone motion.
pub const CONSCAN_RATE_HZ: f32 = 30.0;

/// Conical-scan (conscan) pointing-error demodulator.
///
/// A conscan antenna doesn't hold still on its nominal boresight — it
/// continuously nutates in a small circle around it (`Antenna::scanned_boresight_deg`)
/// and reads pointing error off how the received signal's *amplitude* rises
/// and falls in sync with that circle. Dead on target, the squint samples
/// the (symmetric) gain pattern the same way at every phase, so the received
/// power doesn't vary with phase at all and both accumulators below settle
/// at zero. Off target, the squint spends more of each revolution on the
/// near side of the true direction than the far side, so power modulates
/// once per revolution — and *synchronous detection* (correlating the power
/// samples against cos/sin of the scan's own phase over one full
/// revolution, same trick a lock-in amplifier uses) recovers exactly that
/// modulation's phase and depth as two orthogonal error components in the
/// antenna's own (azimuth, elevation) tangent frame.
///
/// This is only the sensing primitive: nutate, sample, demodulate. Nothing
/// here injects a real pointing error to correct — every antenna in this
/// sim is currently held exactly on its predicted target by `tracking.rs` —
/// and the result isn't fed back into aiming yet. Both are follow-up work,
/// same as the PR that will give drones a reason to be off boresight in the
/// first place.
#[derive(Default)]
pub struct ConicalScan {
    phase_rad: f32,
    az_accum: f64,
    el_accum: f64,
    power_accum: f64,
    sample_count: u32,
}

impl ConicalScan {
    /// Fold in one frame's gain sample (linear-domain internally — conscan is
    /// an amplitude technique, and dB is the wrong domain to correlate in)
    /// and advance the nutation phase by `dt` seconds. Returns the
    /// demodulated (azimuth, elevation) error the instant a full revolution
    /// completes, normalized by the revolution's own mean power so the
    /// result doesn't depend on link range or transmit power — the same way
    /// a real conscan tracker runs its error signal through the receiver's
    /// AGC before reading it. Accumulators reset for the next revolution.
    pub fn sample(&mut self, gain_db: f32, dt: f32) -> Option<(f32, f32)> {
        let power = 10f64.powf(gain_db as f64 / 10.0);
        self.az_accum += power * (self.phase_rad as f64).cos();
        self.el_accum += power * (self.phase_rad as f64).sin();
        self.power_accum += power;
        self.sample_count += 1;

        self.phase_rad += std::f32::consts::TAU * CONSCAN_RATE_HZ * dt;
        if self.phase_rad < std::f32::consts::TAU {
            return None;
        }
        // Rare (huge `dt`, or a stalled frame): drop any extra whole
        // revolutions rather than let phase grow unbounded.
        self.phase_rad = self.phase_rad.rem_euclid(std::f32::consts::TAU);

        let n = self.sample_count.max(1) as f64;
        let mean_power = self.power_accum / n;
        let error = if mean_power > 0.0 {
            // The ×2 is the standard Fourier-series normalization for
            // recovering a fundamental's amplitude from a cos/sin
            // correlation averaged over one period.
            (
                (2.0 * self.az_accum / n / mean_power) as f32,
                (2.0 * self.el_accum / n / mean_power) as f32,
            )
        } else {
            (0.0, 0.0)
        };

        self.az_accum = 0.0;
        self.el_accum = 0.0;
        self.power_accum = 0.0;
        self.sample_count = 0;
        Some(error)
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

    /// Drive one full conical-scan revolution and return its demodulated
    /// error. `dt` is deliberately much finer than a real frame's — tests
    /// want a clean, high-resolution demodulation, not the same coarse
    /// sampling `ConicalScan` will actually see once per `Update`.
    fn scan_one_revolution(antenna: &Antenna, self_pos: Vec3, peer_pos: Vec3) -> (f32, f32) {
        let mut scan = ConicalScan::default();
        let dt = 1.0 / (CONSCAN_RATE_HZ * 200.0);
        loop {
            let phase = scan.phase_rad;
            let gain = antenna.scanned_gain_db(0.0, phase, self_pos, peer_pos);
            if let Some(error) = scan.sample(gain, dt) {
                return error;
            }
        }
    }

    /// Nutating the boresight traces a circle of `squint_deg()` radius
    /// around the antenna's nominal (azimuth, elevation), once per `TAU`.
    #[test]
    fn scanned_boresight_traces_a_circle_around_nominal() {
        let a = make_antenna(10.0, 0.0, 0);
        let squint = a.squint_deg();
        let tau = std::f32::consts::TAU;

        let (az0, el0) = a.scanned_boresight_deg(0.0, 0.0);
        assert!((az0 - (10.0 + squint)).abs() < 1e-4);
        assert!(el0.abs() < 1e-4);

        let (az90, el90) = a.scanned_boresight_deg(0.0, tau / 4.0);
        assert!((az90 - 10.0).abs() < 1e-3);
        assert!((el90 - squint).abs() < 1e-4);

        let (az180, el180) = a.scanned_boresight_deg(0.0, tau / 2.0);
        assert!((az180 - (10.0 - squint)).abs() < 1e-3);
        assert!(el180.abs() < 1e-4);
    }

    /// A peer dead on the antenna's nominal boresight modulates
    /// symmetrically around the scan circle — both error components settle
    /// at zero.
    #[test]
    fn on_boresight_produces_no_demodulated_error() {
        let a = make_antenna(0.0, 0.0, 0); // points +Z
        let (az_err, el_err) = scan_one_revolution(&a, Vec3::ZERO, Vec3::new(0.0, 0.0, 5.0));
        assert!(az_err.abs() < 1e-3, "az error {az_err} should be ~0");
        assert!(el_err.abs() < 1e-3, "el error {el_err} should be ~0");
    }

    /// A peer offset in azimuth from the nominal boresight recovers a
    /// same-signed azimuth error and ~zero elevation error.
    #[test]
    fn azimuth_offset_recovers_matching_error_sign() {
        let a = make_antenna(0.0, 0.0, 0); // points +Z
        let offset = a.squint_deg(); // small, well within the beam

        let (az_err_pos, el_err_pos) =
            scan_one_revolution(&a, Vec3::ZERO, radar_direction(offset, 0.0) * 5.0);
        assert!(az_err_pos > 0.0, "peer offset toward +azimuth should read positive: {az_err_pos}");
        assert!(el_err_pos.abs() < az_err_pos.abs());

        let (az_err_neg, _) = scan_one_revolution(&a, Vec3::ZERO, radar_direction(-offset, 0.0) * 5.0);
        assert!(az_err_neg < 0.0, "peer offset toward -azimuth should read negative: {az_err_neg}");
    }

    /// A peer offset in elevation from the nominal boresight recovers a
    /// same-signed elevation error and ~zero azimuth error.
    #[test]
    fn elevation_offset_recovers_matching_error_sign() {
        let a = make_antenna(0.0, 0.0, 0); // points +Z
        let offset = a.squint_deg();

        let (az_err, el_err) =
            scan_one_revolution(&a, Vec3::ZERO, radar_direction(0.0, offset) * 5.0);
        assert!(el_err > 0.0, "peer offset toward +elevation should read positive: {el_err}");
        assert!(az_err.abs() < el_err.abs());
    }

    /// A bigger pointing error demodulates to a bigger error magnitude —
    /// this is what lets a future correction loop treat it as a usable
    /// signal rather than just a sign.
    #[test]
    fn larger_offset_yields_larger_error_magnitude() {
        let a = make_antenna(0.0, 0.0, 0);
        let small = a.squint_deg() * 0.5;
        let large = a.squint_deg() * 1.5;

        let (err_small, _) = scan_one_revolution(&a, Vec3::ZERO, radar_direction(small, 0.0) * 5.0);
        let (err_large, _) = scan_one_revolution(&a, Vec3::ZERO, radar_direction(large, 0.0) * 5.0);
        assert!(err_large > err_small, "{err_large} should exceed {err_small}");
    }

    /// `ConicalScan::sample` only emits once per full revolution, not every
    /// frame — the demodulation needs a whole circle of samples to mean
    /// anything.
    #[test]
    fn sample_emits_once_per_revolution() {
        let mut scan = ConicalScan::default();
        let dt = 1.0 / (CONSCAN_RATE_HZ * 8.0); // 8 samples/revolution
        // 7 samples can never complete a revolution (7/8 of one, with margin
        // against float rounding either way at the exact boundary).
        for _ in 0..7 {
            assert!(scan.sample(-10.0, dt).is_none(), "revolution shouldn't complete this early");
        }
        // The 8th sample completes it; by the 15th, exactly one revolution
        // (15/8 of one) has completed and the next hasn't started closing.
        let mut emitted = 0;
        for _ in 0..8 {
            if scan.sample(-10.0, dt).is_some() {
                emitted += 1;
            }
        }
        assert_eq!(emitted, 1, "exactly one revolution should complete in the next 8 samples");
    }
}
