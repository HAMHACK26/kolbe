use bevy::prelude::*;

use crate::antenna::Antenna;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DroneType {
    Node,
    Attack,
}

#[derive(Component)]
pub struct Drone {
    pub id: String,
    pub drone_type: DroneType,
    pub antennas: Vec<Antenna>,
}

#[derive(Resource, Default)]
pub struct SelectedDrone(pub Option<Entity>);

/// Deterministic 8-char hex ID from an integer seed.
pub fn drone_id(seed: usize) -> String {
    let mut x = (seed as u32).wrapping_mul(2654435761).wrapping_add(0x9e3779b9);
    x ^= x >> 16;
    x = x.wrapping_mul(0x45d9f3b);
    x ^= x >> 16;
    format!("{:08X}", x)
}

/// Build an antenna pointed at the given angles. All units share the same
/// hardware — only azimuth/elevation vary. `seed` is unused (kept for callers).
pub fn make_antenna(azimuth_deg: f32, elevation_deg: f32, _seed: usize) -> Antenna {
    Antenna {
        azimuth_deg,
        elevation_deg,
        g_peak_dbi: 11.0,
        theta_3db_deg: 1.0,
        floor_db: -30.0,
        p_tx_dbm: 20.0,
        frequency_mhz: 2400.0,
        alpha_db_per_km: 0.005,
        g_rx_dbi: 0.0,
        sensitivity_dbm: -80.0,
    }
}

/// Furthest a link can close on perfect boresight with the hardware above:
/// 3.52 km, where the 111 dB budget (`p_tx` + `g_peak` − `sensitivity`) is
/// exactly eaten by path loss.
///
/// The formation geometry is sized from this — see
/// [`crate::navigation::MAX_LINK_SPACING_KM`]. Change any of the antenna
/// parameters above and this must be recomputed, or the mesh silently stops
/// forming.
pub const ANTENNA_RANGE_KM: f32 = 3.52;
