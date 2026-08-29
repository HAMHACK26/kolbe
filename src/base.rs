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
    antenna::Antenna,
    camera::OrbitCamera,
    drone::{Drone, SelectedDrone, make_antenna},
    factories::{movement::DroneKinematics, track::Track},
    networking::{BootstrapBaseLinks, LinkSet, NetworkingBundle},
    radar::{RadarCone, cone_mesh_for, cone_transform_for},
    theme::ThemeRole,
    world::DRONE_RADIUS,
};

// ─── Base entity ──────────────────────────────────────────────────────────────

/// Edge length of the cube the ground station is drawn as, km. Doubles as its
/// physical footprint — [`crate::avoidance`] derives the base's bounding
/// radius from it so drones keep clear of the structure.
pub const BASE_BOX_SIZE_KM: f32 = 0.3;
/// Visual marker size, kept separate from the operational exclusion footprint.
pub const BASE_RENDER_SIZE_KM: f32 = BASE_BOX_SIZE_KM / 4.0;

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

/// Where the ground station sits — fixed, and derived only from constants and
/// the terrain.
///
/// Exposed separately from [`spawn_base`] because every mesh-table location is
/// base-relative, and `world::setup` needs that frame to seed the drones'
/// initial tables *before* `spawn_base` has run (the two are chained in that
/// order). Computing it rather than querying the entity keeps the two in sync
/// by construction.
pub fn base_position(terrain: &crate::terrain::TerrainHeightMap) -> Vec3 {
    let z = -terrain.size_km() / 2.0 + 1.0; // south edge
    Vec3::new(0.0, terrain.height_at(0.0, z) + DRONE_RADIUS, z)
}

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
    network_area: Res<crate::area::NetworkArea>,
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
    // The shape the operator drew is the mission boundary — `NetworkArea::hull`,
    // not the `corners` square derived from it. That square only ever sized the
    // terrain fetch and the airframe count, and it always covers ground nobody
    // selected.
    //
    // Convert once here, then send only base-relative vectors over the mesh:
    // drones navigate the boundary from their own offset to the station and
    // never need a global area coordinate.
    let target_area: Option<std::sync::Arc<[Vec3]>> = (network_area.valid
        && network_area.hull.len() >= 3)
        .then(|| {
            network_area
                .hull
                .iter()
                .map(|&(lon, lat)| {
                    Vec3::new(
                        ((lon - area.longitude) * 111.320 * area.latitude.to_radians().cos()) as f32,
                        pos.y,
                        ((lat - area.latitude) * 110.574) as f32,
                    ) - pos
                })
                .collect()
        });

    // 5 connections — same hardware as the drones, one antenna per sector.
    // `networking::detect_base_links_and_send_headers` re-aims each one within
    // its own sector every frame, so these are the resting bearings.
    let antennas: Vec<Antenna> = (0..5)
        .map(|k| {
            make_antenna(
                k as f32 * crate::networking::BASE_SECTOR_DEG,
                crate::networking::BASE_SECTOR_ELEVATION_DEG,
                200 + k,
            )
        })
        .collect();
    let mut networking = NetworkingBundle::random(usize::MAX);
    networking.target_area.corners_from_base = target_area;

    let base_entity = commands
        .spawn((
        // Visual: yellow box
        Mesh3d(meshes.add(Cuboid::from_length(BASE_RENDER_SIZE_KM))),
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
        BaseNetworkState::default(),
        // The station tracks and seeks like an airframe: per-antenna peer
        // assignment plus per-antenna spiral state.
        crate::tracking::BaseAntennaTargets::default(),
        crate::seeking::BaseSeekState::default(),
        // The station is a static mesh node: same header/table protocol as a
        // drone, but five antenna slots and no flight systems.
        networking,
        BootstrapBaseLinks,
        DroneKinematics::default(),
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

    // Radar cones — always drawn, keyed by base entity.
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
            Visibility::Visible,
            RadarCone { drone_entity: base_entity, antenna_index },
            ThemeRole::BaseCone,
            crate::SimulationEntity,
        ));
    }
}

// ─── Systems (stubs) ──────────────────────────────────────────────────────────

/// Mirror the base's live radio links into the UI-facing network state.
///
/// `detect_base_links_and_send_headers` is the authority for the physical
/// antenna link. Reading its `LinkSet` here keeps the displayed connection
/// count, mesh routing, and recovery logic on one shared definition of a link.
pub fn update_base_comms(
    mut bases: Query<(&LinkSet, &mut BaseNetworkState), With<Base>>,
    drones: Query<(), With<Drone>>,
) {
    for (links, mut state) in &mut bases {
        state.reachable_drones = links
            .connected
            .keys()
            .copied()
            .filter(|entity| drones.get(*entity).is_ok())
            .collect();
        // Detailed link-budget reporting is not currently surfaced, and a
        // stale RSSI is worse than an explicit unavailable value.
        state.best_rssi_dbm = f32::NEG_INFINITY;
    }
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
