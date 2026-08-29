//! Antenna tracking: deciding which way a drone's antennas physically point.
//!
//! This is deliberately separate from [`crate::networking`] — the comms
//! protocol (headers, packets, ranging, mesh-table gossip) is one concern,
//! and pointing a directional antenna at a target based on what that protocol
//! learns is a different one, the same way `radar.rs` (antenna cone geometry)
//! is its own module rather than living inside `networking.rs`. This module
//! *reads* data networking produces (`TrackedPeers`, populated by
//! `networking::route_packets` from a peer's self-reported `flight_direction`)
//! but the dependency only ever runs one way — networking has no idea this
//! module exists.
//!
//! Nothing here ever touches a drone's own position, velocity, or
//! navigation — this module only decides which way antennas point.

use std::collections::HashMap;

use bevy::prelude::*;

use crate::antenna::{Antennas, angles_toward};
use crate::base::Base;
use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::networking::{DroneUuid, MeshTable, Pairing, RingIndex};
use crate::world::RelayTopology;

/// Where this drone currently believes a tracked peer *will be*, based
/// purely on the last header that peer sent — never on omniscient ECS
/// state. Refreshed every ~`networking::HEADER_INTERVAL_SECS` when a new
/// header arrives (`networking::route_packets`); held fixed between
/// arrivals.
///
/// This exists for exactly one purpose: letting a directional antenna keep
/// its lock on a moving peer despite only hearing from it periodically, the
/// way a real tracking antenna leads a moving target using the target's own
/// reported heading/speed rather than needing a continuous, magical view of
/// where it actually is right now. It is consumed *only* by
/// [`maintain_mesh_antennas`] for aiming.
#[derive(Component, Default)]
pub struct TrackedPeers(pub HashMap<Entity, Vec3>);

/// Keep every drone locked onto its ring neighbors: antenna #1 → next
/// neighbor, antenna #2 → base, antenna #3 → previous neighbor. Non-adjacent
/// drones are never targeted by any antenna, so they can only learn about
/// each other by relay through the mesh table.
///
/// Antennas #1/#3 aim at the neighbor's **predicted** position from
/// [`TrackedPeers`] (led by that peer's last self-reported flight direction)
/// rather than this drone's live, omniscient ECS view of them — a real
/// tracking antenna only knows where its target *reported* it would be, not
/// where it actually is between updates. Before any header has ever arrived
/// directly from a given neighbor, fall back to that neighbor's last mesh
/// table row (`base_pos + row.location`) — a relayed, comms-derived estimate,
/// the same base-relative pattern `seeking::seek_one_slot` uses. Only if
/// there is truly *no* information about them yet (never predicted, never
/// relayed) is the antenna left at whatever angle it already had — a drone
/// never reads a neighbor's true position directly.
///
/// The base itself is different: it's a fixed, known anchor every drone's own
/// position is already expressed relative to (`self_pos - base_pos`), not
/// something sensed from a peer, so aiming antenna #2 at `base_pos` directly
/// is not an omniscience violation.
///
/// This overrides the fixed 120°-apart layout `world::setup` used to compute
/// the initial angles; those were only ever a starting point.
pub fn maintain_mesh_antennas(
    mut drones: Query<(
        &Transform,
        &mut Antennas,
        &DroneKinematics,
        &RingIndex,
        &TrackedPeers,
        &MeshTable,
        &Pairing,
    )>,
    positions: Query<(Entity, &RingIndex, &DroneUuid), With<Drone>>,
    bases: Query<&Base>,
) {
    let base_pos = bases.iter().next().map(|b| b.position);

    // Ring neighbors, not "the other drone" — with N > 2 most pairs are
    // never mutually visible on purpose, so relayed (multi-hop) rows in the
    // mesh table actually get exercised.
    let mut ring: Vec<(usize, Entity, String)> =
        positions.iter().map(|(e, ri, uuid)| (ri.0, e, uuid.0.clone())).collect();
    ring.sort_by_key(|(i, ..)| *i);
    let n = ring.len();

    for (self_transform, mut antennas, kin, self_ring, tracked, table, pairing) in &mut drones {
        if n == 0 {
            continue;
        }
        // "Stopped" for a reconnection handshake — hold antenna slew steady so
        // a lock can be acquired/kept. Don't re-aim this frame.
        if pairing.frozen {
            continue;
        }
        let self_pos = self_transform.translation;
        let (_, next_entity, next_uuid) = &ring[(self_ring.0 + 1) % n];
        let (_, prev_entity, prev_uuid) = &ring[(self_ring.0 + n - 1) % n];
        // Predicted (from the peer's own last header) beats relayed mesh-table
        // last-known-position, which beats "no info, don't move".
        let mesh_pos = |uuid: &str| {
            base_pos.zip(table.0.get(uuid)).map(|(base, row)| base + row.location)
        };
        let next_pos = tracked.0.get(next_entity).copied().or_else(|| mesh_pos(next_uuid));
        let prev_pos = tracked.0.get(prev_entity).copied().or_else(|| mesh_pos(prev_uuid));

        // TODO: error correction via conical scan.
        //
        // The predicted aim above is pure dead reckoning from the peer's
        // last self-report — it has no way to notice it's wrong between
        // updates (peer maneuvers, dropped/late header, bad velocity
        // estimate, etc). A real tracking antenna corrects this by nutating
        // its beam in a small cone around the current boresight (a few
        // squints/sec, offset by a fraction of theta_3db_deg) and comparing
        // received signal strength at each point around that cone: if RSSI
        // is even all the way around, boresight is dead-on target; if it's
        // stronger on one side, the target is off-axis in that direction,
        // and the aim gets nudged accordingly. Implement as a closed-loop
        // correction on top of (not a replacement for) the predictive aim
        // computed here — predict first, then trim with conical-scan error
        // feedback using `Antenna::rssi_dbm`/`off_boresight_deg` sampled at
        // a few points around the current boresight each tick.

        // `angles_toward` returns a world-frame bearing while
        // `Antenna::azimuth_deg` is drone-relative — the antennas turn with the
        // airframe — so the drone's own heading comes back out of every
        // azimuth. Elevation is already world-frame (everything shares one
        // "up") and needs no such correction.
        let relative_az = |az: f32| (az - kin.heading_deg).rem_euclid(360.0);
        if let (Some(next_pos), Some(first)) = (next_pos, antennas.0.get_mut(0)) {
            let (az, el) = angles_toward(self_pos, next_pos);
            first.azimuth_deg = relative_az(az);
            first.elevation_deg = el;
        }
        if let (Some(base_pos), Some(second)) = (base_pos, antennas.0.get_mut(1)) {
            let (az, el) = angles_toward(self_pos, base_pos);
            second.azimuth_deg = relative_az(az);
            second.elevation_deg = el;
        }
        if let (Some(prev_pos), Some(third)) = (prev_pos, antennas.0.get_mut(2)) {
            let (az, el) = angles_toward(self_pos, prev_pos);
            third.azimuth_deg = relative_az(az);
            third.elevation_deg = el;
        }
    }
}

/// Aim the antenna slots reserved by the relay topology. This runs after the
/// legacy ring tracker and therefore gives required active and pending relay
/// edges priority without changing how optional mesh links are maintained.
/// Peer positions come only from direct prediction or relayed mesh knowledge.
#[allow(clippy::type_complexity)]
pub fn maintain_relay_antennas(
    topology: Res<RelayTopology>,
    mut drones: Query<(
        Entity,
        &Transform,
        &mut Antennas,
        &DroneKinematics,
        &TrackedPeers,
        &MeshTable,
    ), With<Drone>>,
    uuids: Query<&DroneUuid>,
    bases: Query<(Entity, &Base)>,
) {
    let base = bases.iter().next();

    for (entity, transform, mut antennas, kinematics, tracked, table) in &mut drones {
        let self_pos = transform.translation;
        for (slot, target) in topology.antenna_targets(entity) {
            let target_pos = if base.is_some_and(|(base_entity, _)| target == base_entity) {
                base.map(|(_, base)| base.position)
            } else {
                let mesh_position = uuids
                    .get(target)
                    .ok()
                    .and_then(|uuid| table.0.get(&uuid.0))
                    .and_then(|row| base.map(|(_, base)| base.position + row.location));
                tracked.0.get(&target).copied().or(mesh_position)
            };
            let Some(target_pos) = target_pos else {
                continue;
            };
            let Some(antenna) = antennas.0.get_mut(slot) else {
                continue;
            };
            let (azimuth, elevation) = angles_toward(self_pos, target_pos);
            antenna.azimuth_deg = (azimuth - kinematics.heading_deg).rem_euclid(360.0);
            antenna.elevation_deg = elevation;
        }
    }
}

/// Keep the base's antennas — one per drone — locked onto the formation.
///
/// Same policy as [`maintain_mesh_antennas`], and the same information
/// discipline: a target is aimed at from that drone's **predicted** position
/// ([`TrackedPeers`], led by its own last self-reported flight direction),
/// falling back to its last relayed mesh-table row, and left untouched when
/// the base knows nothing about it yet. The base never reads a drone's true
/// `Transform`.
///
/// The difference from a drone is only coverage: a drone watches its two ring
/// neighbors, the base watches every ring slot it has an antenna for. Antenna
/// `k` tracks ring slot `k`, which is the pairing `base::spawn_base` aims at
/// on frame 0.
pub fn maintain_base_antennas(
    mut bases: Query<(&Base, &mut Antennas, &TrackedPeers, &MeshTable, &Pairing)>,
    drones: Query<(Entity, &RingIndex, &DroneUuid), With<Drone>>,
) {
    let mut ring: Vec<(usize, Entity, String)> =
        drones.iter().map(|(e, ri, uuid)| (ri.0, e, uuid.0.clone())).collect();
    ring.sort_by_key(|(i, ..)| *i);

    for (base, mut antennas, tracked, table, pairing) in &mut bases {
        // Same slew-freeze rule the drones follow during a handshake.
        if pairing.frozen {
            continue;
        }
        let base_pos = base.position;
        for (slot, drone_entity, drone_uuid) in &ring {
            let Some(antenna) = antennas.0.get_mut(*slot) else {
                break; // fewer antennas than drones — the rest go uncovered.
            };
            let target = tracked
                .0
                .get(drone_entity)
                .copied()
                .or_else(|| table.0.get(drone_uuid).map(|row| base_pos + row.location));
            let Some(target) = target else {
                continue; // nothing known about this drone yet — hold the aim.
            };
            // A base has no heading, so the world-frame bearing is already the
            // azimuth its antennas are expressed in.
            let (az, el) = angles_toward(base_pos, target);
            antenna.azimuth_deg = az;
            antenna.elevation_deg = el;
        }
    }
}
#[cfg(test)]
mod relay_tracking_tests {
    use super::*;
    use crate::drone::make_antenna;
    use crate::networking::MeshRow;

    #[test]
    fn pending_relay_target_overrides_its_reserved_antenna_with_vector_aim() {
        let mut app = App::new();
        let base = app
            .world_mut()
            .spawn(Base { id: "base".into(), position: Vec3::ZERO, antennas: Vec::new() })
            .id();
        let older = app.world_mut().spawn(DroneUuid("older".into())).id();
        let newer = app
            .world_mut()
            .spawn((
                Drone { id: "newer".into() },
                DroneUuid("newer".into()),
                Transform::from_translation(Vec3::ZERO),
                Antennas(vec![
                    make_antenna(0.0, 5.0, 1),
                    make_antenna(120.0, 5.0, 2),
                    make_antenna(240.0, 5.0, 3),
                ]),
                DroneKinematics::default(),
                TrackedPeers::default(),
                MeshTable(HashMap::from([(
                    "older".into(),
                    MeshRow {
                        id: "older".into(),
                        timestamp: 0.0,
                        location: Vec3::X,
                        neighbour_distance: 0,
                        connections: Vec::new(),
                    },
                )])),
            ))
            .id();
        let mut topology = RelayTopology::default();
        topology.register_wave(base, vec![older]);
        topology.register_wave(base, vec![newer]);
        let target_slot = topology
            .antenna_targets(newer)
            .into_iter()
            .find_map(|(slot, target)| (target == older).then_some(slot))
            .unwrap();
        app.insert_resource(topology);
        app.add_systems(Update, maintain_relay_antennas);

        app.update();

        let antennas = app.world().entity(newer).get::<Antennas>().unwrap();
        assert!((antennas.0[target_slot].azimuth_deg - 90.0).abs() < 1e-6);
        assert!(antennas.0[target_slot].elevation_deg.abs() < 1e-6);
    }
}
