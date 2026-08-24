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

/// Build an antenna with per-drone variation derived from seed.
pub fn make_antenna(azimuth_deg: f32, elevation_deg: f32, seed: usize) -> Antenna {
    let v = (seed as f32 * 1.618) % 1.0;
    Antenna {
        azimuth_deg,
        elevation_deg,
        g_peak_dbi: 8.0 + v * 6.0,
        theta_3db_deg: 0.8 + v * 0.4,
        floor_db: -30.0,
        p_tx_dbm: 18.0 + v * 4.0,
        frequency_mhz: 2400.0,
        alpha_db_per_km: 0.005,
        g_rx_dbi: 0.0,
        sensitivity_dbm: -80.0,
    }
}
