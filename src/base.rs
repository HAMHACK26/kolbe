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

use bevy::prelude::*;

use crate::{
    antenna::{Antenna, Antennas},
    camera::OrbitCamera,
    drone::{Drone, SelectedDrone, make_antenna},
    factories::{movement::DroneKinematics, track::Track},
    radar::{RadarCone, cone_mesh_for, cone_transform_for},
    networking::RadioBundle,
    theme::ThemeRole,
    world::DRONE_RADIUS,
};

// ─── Base entity ──────────────────────────────────────────────────────────────

/// Edge length of the cube the ground station is drawn as, km. Doubles as its
/// physical footprint — [`crate::avoidance`] derives the base's bounding
/// radius from it so drones keep clear of the structure.
pub const BASE_BOX_SIZE_KM: f32 = 0.075;
/// Keep a wide launch and recovery corridor clear of tree canopies around the
/// station. This is the former 0.35 km pad widened by 2 km.
pub const BASE_FOLIAGE_CLEARANCE_KM: f32 = 2.35;

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
    terrain: Res<crate::terrain::TerrainHeightMap>,
    theme: Res<crate::theme::Theme>,
    base_position: Res<crate::area::BasePosition>,
    area: Res<crate::area::ScenarioArea>,
) {
    // Initial colors from the palette; `apply_theme` re-syncs on toggle
    // (these entities carry ThemeRole markers).
    let pal = theme.palette();
    // User-chosen base location, converted from lat/lon into the same local
    // x/z frame the terrain mesh uses (see `fetch_height_map`'s row-flip
    // comment: +Z is north, +X is east). Falls back to the south edge if
    // somehow unset (shouldn't happen — `generate_terrain` requires it).
    let (x, z) = match base_position.0 {
        Some((lat, lon)) => (
            ((lon - area.longitude) * 111.320 * area.latitude.to_radians().cos()) as f32,
            ((lat - area.latitude) * 110.574) as f32,
        ),
        None => (0.0, -terrain.size_km() / 2.0 + 1.0),
    };
    let pos = Vec3::new(x, terrain.height_at(x, z) + DRONE_RADIUS, z);

    // 5 connections — same hardware as the drones, one antenna per 72° sector.
    let antennas: Vec<Antenna> =
        (0..5).map(|k| make_antenna(k as f32 * 72.0, 5.0, 200 + k)).collect();

    let base_entity = commands
        .spawn((
        // Visual: yellow box
        Mesh3d(meshes.add(Cuboid::from_length(BASE_BOX_SIZE_KM))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: pal.base,
            emissive: LinearRgba::new(1.5, 1.5, 0.0, 1.0),
            ..default()
        })),
        Transform::from_translation(pos),
        Base {
            id: "GCS-ALPHA".into(),
            position: pos,
            antennas: antennas.clone(),
        },
        // The base is a first-class radio node. This makes its direct link
        // appear in each drone's lookup table immediately and lets its table
        // relay newly learned peers through the mesh.
        Antennas(antennas.clone()),
        RadioBundle::random(),
        BaseNetworkState::default(),
        ThemeRole::BaseMarker,
        crate::SimulationEntity,
    ))
    .observe(
        |mut t: On<Pointer<Click>>, orbit: Res<OrbitCamera>, mut sel: ResMut<SelectedDrone>| {
            t.propagate(false);
            if orbit.drag_total < 5.0 {
                sel.0 = Some(t.original_event_target());
            }
        },
    )
    .id();

    // Radar cones — hidden until the base is selected (keyed by base entity).
    let cone_mat = materials.add(StandardMaterial {
        base_color: pal.base.with_alpha(0.20),
        emissive: LinearRgba::new(0.6, 0.5, 0.0, 0.0),
        alpha_mode: AlphaMode::Blend,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    for (antenna_index, antenna) in antennas.iter().enumerate() {
        commands.spawn((
            Mesh3d(cone_mesh_for(antenna, &mut meshes)),
            MeshMaterial3d(cone_mat.clone()),
            // Bases have no heading — 0.0 leaves azimuth effectively world-frame.
            cone_transform_for(antenna, 0.0, pos),
            Visibility::Hidden,
            RadarCone { drone_entity: base_entity, antenna_index },
            ThemeRole::BaseCone,
            crate::SimulationEntity,
        ));
    }
}

// ─── Systems (stubs) ──────────────────────────────────────────────────────────

/// Compute RSSI from base to every drone; populate `BaseNetworkState::reachable_drones`.
///
/// Use `Antenna::off_boresight_deg(0.0, base_pos, drone_pos)` for θ_tx — 0.0
/// because a base has no heading, so its antennas' azimuth is already
/// world-frame — then `antenna.rssi_dbm(θ_tx, 0.0, d)` (θ_rx = 0 until drones
/// expose their own antenna direction to the base).
pub fn update_base_comms(
    _bases: Query<(&Base, &mut BaseNetworkState)>,
    _drones: Query<(Entity, &GlobalTransform), With<Drone>>,
) {
    todo!(
        "For each (base, antenna) × drone: \
         θ = antenna.off_boresight_deg(0.0, base.position, drone_pos), \
         d = (drone_pos - base.position).length(), \
         rssi = antenna.rssi_dbm(θ, 0.0, d); \
         collect entities where rssi >= antenna.sensitivity_dbm \
         into BaseNetworkState::reachable_drones"
    );
}

/// Push commands onto the `CommandQueue` of every reachable drone.
/// Only drones in `BaseNetworkState::reachable_drones` receive commands.
pub fn dispatch_commands(
    _bases: Query<&BaseNetworkState>,
    _drones: Query<(Entity, &mut CommandQueue)>,
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
    _commands: Commands,
    _drones: Query<(
        Entity,
        &mut CommandQueue,
        &mut DroneKinematics,
        Option<&mut Track>,
    )>,
    _bases: Query<&Base>,
) {
    todo!(
        "For each drone with a non-empty CommandQueue: \
         pop front command, match on variant, \
         insert/remove components as documented above"
    );
}
