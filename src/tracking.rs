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

use crate::antenna::angles_toward;
use crate::base::Base;
use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::networking::{DroneUuid, MeshTable, Pairing, RingIndex};

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
        &mut Drone,
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

    for (self_transform, mut drone, kin, self_ring, tracked, table, pairing) in &mut drones {
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
        if let (Some(next_pos), Some(first)) = (next_pos, drone.antennas.get_mut(0)) {
            let (az, el) = angles_toward(self_pos, next_pos);
            first.azimuth_deg = relative_az(az);
            first.elevation_deg = el;
        }
        if let (Some(base_pos), Some(second)) = (base_pos, drone.antennas.get_mut(1)) {
            let (az, el) = angles_toward(self_pos, base_pos);
            second.azimuth_deg = relative_az(az);
            second.elevation_deg = el;
        }
        if let (Some(prev_pos), Some(third)) = (prev_pos, drone.antennas.get_mut(2)) {
            let (az, el) = angles_toward(self_pos, prev_pos);
            third.azimuth_deg = relative_az(az);
            third.elevation_deg = el;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::system::RunSystemOnce;

    use crate::drone::{make_antenna, DroneType};
    use crate::networking::{DroneUuid, MeshRow, NetworkingBundle};

    /// Spawn a drone at `pos` in ring slot `ring`, with 3 zeroed antennas the
    /// aiming system will retarget. Uses the real `NetworkingBundle` (which
    /// carries `RingIndex`, `TrackedPeers`, and any other per-drone networking
    /// state) so the drone matches `maintain_mesh_antennas`'s query in full —
    /// hand-listing components would silently stop matching if the query ever
    /// gains a param.
    fn spawn_drone(world: &mut World, pos: Vec3, ring: usize) -> Entity {
        world
            .spawn((
                Transform::from_translation(pos),
                Drone { id: format!("d{ring}"), drone_type: DroneType::Node },
                Antennas(vec![
                    make_antenna(0.0, 0.0, 0),
                    make_antenna(0.0, 0.0, 1),
                    make_antenna(0.0, 0.0, 2),
                ]),
                NetworkingBundle::random(ring),
                // Heading 0 — the aiming math converts world bearings into
                // this drone's frame, so the tests' expected angles are the
                // world-frame ones.
                DroneKinematics::default(),
            ))
            .id()
    }

    fn spawn_base(world: &mut World, pos: Vec3) {
        world.spawn(Base { id: "base".into(), position: pos });
    }

    fn antenna0_az(world: &World, drone: Entity) -> f32 {
        world.get::<Drone>(drone).unwrap().antennas[0].azimuth_deg
    }

    /// Set drone `owner`'s tracked prediction for `peer`.
    fn set_tracked(world: &mut World, owner: Entity, peer: Entity, predicted: Vec3) {
        world.get_mut::<TrackedPeers>(owner).unwrap().0.insert(peer, predicted);
    }

    /// Give `owner`'s mesh table a relayed/last-known row for `peer`, as if
    /// gossip (not a direct header) taught it that base-relative location.
    fn set_mesh_row(world: &mut World, owner: Entity, peer: Entity, location: Vec3) {
        let peer_uuid = world.get::<DroneUuid>(peer).unwrap().0.clone();
        world.get_mut::<MeshTable>(owner).unwrap().0.insert(
            peer_uuid.clone(),
            MeshRow {
                id: peer_uuid,
                timestamp: 0.0,
                location,
                neighbour_distance: 1,
                connections: vec![],
            },
        );
    }

    /// With a tracked prediction present, antenna #1 aims at the *predicted*
    /// point — not the peer's true position.
    #[test]
    fn aims_at_predicted_not_true_position() {
        let mut world = World::new();
        spawn_base(&mut world, Vec3::new(0.0, 0.0, -5.0));
        let a = spawn_drone(&mut world, Vec3::ZERO, 0);
        let b = spawn_drone(&mut world, Vec3::new(1.0, 0.0, 0.0), 1);

        // B's true bearing from A is +X → azimuth 90°. Predicted point is
        // off in +Z, which bears 45°.
        set_tracked(&mut world, a, b, Vec3::new(1.0, 0.0, 1.0));
        world.run_system_once(maintain_mesh_antennas).unwrap();

        let az = antenna0_az(&world, a);
        assert!((az - 45.0).abs() < 0.5, "expected ~45° (predicted), got {az}");
        assert!((az - 90.0).abs() > 10.0, "must not aim at true position (90°)");
    }

    /// As the tracked prediction moves, the antenna follows it — the whole
    /// point of prediction-led tracking.
    #[test]
    fn antenna_follows_moving_prediction() {
        let mut world = World::new();
        spawn_base(&mut world, Vec3::new(0.0, 0.0, -5.0));
        let a = spawn_drone(&mut world, Vec3::ZERO, 0);
        let b = spawn_drone(&mut world, Vec3::new(1.0, 0.0, 0.0), 1);

        // No prediction and no mesh-table row yet → truly no info, so the
        // antenna is left exactly where it spawned (0.0), never at B's real
        // position.
        world.run_system_once(maintain_mesh_antennas).unwrap();
        let az_cold = antenna0_az(&world, a);
        assert_eq!(az_cold, 0.0, "no info yet must leave antenna untouched, got {az_cold}");

        // Prediction slides in +Z; azimuth should swing monotonically toward 45°.
        set_tracked(&mut world, a, b, Vec3::new(1.0, 0.0, 0.5));
        world.run_system_once(maintain_mesh_antennas).unwrap();
        let az_mid = antenna0_az(&world, a);

        set_tracked(&mut world, a, b, Vec3::new(1.0, 0.0, 1.0));
        world.run_system_once(maintain_mesh_antennas).unwrap();
        let az_far = antenna0_az(&world, a);

        // az_cold (0.0, untouched) isn't on this curve — only compare the two
        // tracked-prediction samples, which should converge monotonically.
        assert!(
            az_mid > az_far,
            "antenna should track the moving prediction: {az_mid} -> {az_far}"
        );
        assert!((az_far - 45.0).abs() < 0.5, "final aim should reach ~45°, got {az_far}");
    }

    /// With no direct prediction but a relayed mesh-table row, antenna #1
    /// aims at that row's base-relative location — comms-derived, never the
    /// peer's true ECS `Transform`.
    #[test]
    fn falls_back_to_mesh_table_when_no_prediction() {
        let mut world = World::new();
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        spawn_base(&mut world, base_pos);
        let a = spawn_drone(&mut world, Vec3::ZERO, 0);
        // B's true position (+X, 5 south) bears ~90°; deliberately different
        // from the mesh-table row below so the test can't pass by accident.
        let b = spawn_drone(&mut world, Vec3::new(1.0, 0.0, 0.0), 1);

        // Mesh-table row says B is at base_pos + (0, 0, 8) = (0, 0, 3), i.e.
        // due north of A (bearing 0°) — deliberately not 90°.
        set_mesh_row(&mut world, a, b, Vec3::new(0.0, 0.0, 8.0));
        world.run_system_once(maintain_mesh_antennas).unwrap();

        let az = antenna0_az(&world, a);
        assert!((az - 0.0).abs() < 0.5, "expected ~0° (mesh-table row), got {az}");
        assert!((az - 90.0).abs() > 10.0, "must not aim at B's true position (90°)");
    }
}
