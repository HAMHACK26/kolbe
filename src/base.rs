#![allow(dead_code)]
//! Ground Control Station (Base).
//!
//! The base is a fixed entity with its own antennas. It observes which
//! drones are within radio range, issues commands to reachable drones,
//! and receives telemetry from them.
//!
//! All communication still goes through the antenna physics — a drone
//! only receives a command if `rssi_dbm(θ_tx, θ_rx, d) >= sensitivity`.
//!
//! Pipeline per frame:
//!   update_base_comms   — compute RSSI to every drone, tag reachable ones
//!   dispatch_commands   — push BaseCommand into CommandQueue of reachable drones
//!   process_drone_commands — each drone pops its queue, updates SeekTarget / Track

use std::collections::VecDeque;

use bevy::{color::palettes::css, prelude::*};

use crate::{
    antenna::Antenna,
    drone::Drone,
    factories::{
        movement::DroneKinematics,
        seek::SeekTarget,
        track::{Track, TrackingActive},
    },
    world::{DRONE_RADIUS, WORLD_SIZE},
};

// ─── Base entity ──────────────────────────────────────────────────────────────

/// Marks the ground control station entity.
#[derive(Component)]
pub struct Base {
    pub id: String,
    /// Fixed world-space position (km). Y = elevation.
    pub position: Vec3,
    /// Antennas this base uses to communicate with drones.
    pub antennas: Vec<Antenna>,
}

/// Tracks which drones the base can currently communicate with.
#[derive(Component, Default)]
pub struct BaseNetworkState {
    /// Entities currently above sensitivity threshold on at least one antenna.
    pub reachable_drones: Vec<Entity>,
    pub best_rssi_dbm: f32,
}

// ─── Commands ─────────────────────────────────────────────────────────────────

/// Orders the base can issue to drones over the radio link.
#[derive(Clone, Debug)]
pub enum BaseCommand {
    /// Navigate to world-space position (km).
    GoTo(Vec3),
    /// Lock tracking on a specific entity.
    TrackTarget(Entity),
    /// Hold current position.
    Hold,
    /// Return to base position.
    ReturnToBase,
    /// Abort all tasks; transition to loiter.
    Abort,
}

/// Per-drone command inbox. Base pushes commands here when in range.
/// Drone's `process_drone_commands` system consumes them front-to-back.
#[derive(Component, Default)]
pub struct CommandQueue {
    pub commands: VecDeque<BaseCommand>,
}

// ─── Spawning ─────────────────────────────────────────────────────────────────

/// Spawn the base at a fixed position with a visual marker.
/// Call from `world::setup` or as a separate `Startup` system.
pub fn spawn_base(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let pos = Vec3::new(0.0, DRONE_RADIUS, -WORLD_SIZE / 2.0 + 1.0); // south edge

    let antenna = Antenna {
        azimuth_deg: 0.0,
        elevation_deg: 5.0,
        g_peak_dbi: 18.0,
        theta_3db_deg: 15.0,   // wider beam — covers whole area
        floor_db: -30.0,
        p_tx_dbm: 30.0,        // 1 W — stronger than drones
        frequency_mhz: 2400.0,
        alpha_db_per_km: 0.005,
        g_rx_dbi: 2.0,
        sensitivity_dbm: -90.0,
    };

    commands.spawn((
        // Visual: yellow box
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::from(css::YELLOW),
            emissive: LinearRgba::new(1.5, 1.5, 0.0, 1.0),
            ..default()
        })),
        Transform::from_translation(pos),
        Base {
            id: "GCS-ALPHA".into(),
            position: pos,
            antennas: vec![antenna],
        },
        BaseNetworkState::default(),
    ));
}

// ─── Systems (stubs) ──────────────────────────────────────────────────────────

/// Compute RSSI from base to every drone; populate `BaseNetworkState::reachable_drones`.
///
/// Use `Antenna::off_boresight_deg(base_pos, drone_pos)` for θ_tx,
/// then `antenna.rssi_dbm(θ_tx, 0.0, d)` (θ_rx = 0 until drones expose
/// their own antenna direction to the base).
pub fn update_base_comms(
    mut bases: Query<(&Base, &mut BaseNetworkState)>,
    drones: Query<(Entity, &GlobalTransform), With<Drone>>,
) {
    todo!(
        "For each (base, antenna) × drone: \
         θ = antenna.off_boresight_deg(base.position, drone_pos), \
         d = (drone_pos - base.position).length(), \
         rssi = antenna.rssi_dbm(θ, 0.0, d); \
         collect entities where rssi >= antenna.sensitivity_dbm \
         into BaseNetworkState::reachable_drones"
    );
}

/// Push commands onto the `CommandQueue` of every reachable drone.
/// Only drones in `BaseNetworkState::reachable_drones` receive commands.
pub fn dispatch_commands(
    bases: Query<&BaseNetworkState>,
    mut drones: Query<(Entity, &mut CommandQueue)>,
) {
    todo!(
        "Iterate bases; for each reachable drone entity \
         push whatever command the base operator has queued; \
         gate on radio link — unreachable drones get nothing"
    );
}

/// Each drone pops its `CommandQueue` and updates its own components.
///
/// Mapping:
///   GoTo(pos)         → insert/replace SeekTarget { position: pos, .. }
///   TrackTarget(e)    → insert TrackingActive; set Track::peer = Some(e)
///   Hold              → remove SeekTarget; zero DroneKinematics::velocity
///   ReturnToBase      → GoTo(base.position)
///   Abort             → remove SeekTarget + TrackingActive; zero velocity
pub fn process_drone_commands(
    mut commands: Commands,
    mut drones: Query<(
        Entity,
        &mut CommandQueue,
        &mut DroneKinematics,
        Option<&mut Track>,
    )>,
    bases: Query<&Base>,
) {
    todo!(
        "For each drone with a non-empty CommandQueue: \
         pop front command, match on variant, \
         insert/remove components as documented above"
    );
}
