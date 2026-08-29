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
use crate::networking::{
    BASE_SECTOR_DEG, BASE_SECTOR_ELEVATION_DEG, DroneUuid, MeshTable, Pairing, RingIndex,
    shortest_angle_deg,
};

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

/// Which peer each of the ground station's antennas is currently tracking,
/// one slot per antenna. Written by [`maintain_base_antennas`] and read by
/// [`crate::seeking::seek_lost_base_links`], which needs to know whose link
/// went quiet before it may start sweeping that slot.
#[derive(Component, Default)]
pub struct BaseAntennaTargets(pub Vec<Option<Entity>>);

/// Keep the ground station's five sector antennas locked onto the drones it
/// knows about — the station tracks exactly the way an airframe does.
///
/// It used to aim straight at each drone's live `Transform`, which is
/// omniscience the radio cannot supply: the station knows only what arrived
/// over the air. So the aim point per peer is, in order, that peer's
/// **predicted** position from [`TrackedPeers`] (its own last self-reported
/// flight direction, led forward), then its last-known
/// [`MeshTable`] row (`base_pos + row.location`), then nothing — a peer the
/// station has never heard from is not aimed at.
///
/// Assignment is unchanged: peers claim antennas nearest-first, each taking
/// the free antenna whose sector its bearing falls closest to, so beams stay
/// spread while there is spread to be had but no known drone is left untracked
/// while an antenna idles. Genuinely spare antennas rest on their sector.
pub fn maintain_base_antennas(
    mut bases: Query<(&mut Base, &TrackedPeers, &MeshTable, &mut BaseAntennaTargets)>,
    drones: Query<(Entity, &DroneUuid), With<Drone>>,
) {
    for (mut base, tracked, table, mut targets) in &mut bases {
        let base_pos = base.position;

        // Identity comes from the ECS, position never does.
        let mut peers: Vec<(Entity, Vec3)> = drones
            .iter()
            .filter_map(|(entity, uuid)| {
                let estimate = tracked
                    .0
                    .get(&entity)
                    .copied()
                    .or_else(|| table.0.get(&uuid.0).map(|row| base_pos + row.location))?;
                Some((entity, estimate))
            })
            .collect();
        peers.sort_by(|a, b| (a.1 - base_pos).length().total_cmp(&(b.1 - base_pos).length()));

        let mut assigned: Vec<Option<(Entity, Vec3)>> = vec![None; base.antennas.len()];
        for (entity, peer_pos) in &peers {
            // At zero range there is no bearing to aim at. Those launch links
            // are held open by the zero-distance rule in the detector instead.
            if (*peer_pos - base_pos).length() <= f32::EPSILON {
                continue;
            }
            let (azimuth_deg, _) = angles_toward(base_pos, *peer_pos);
            let free = assigned
                .iter()
                .enumerate()
                .filter(|(_, taken)| taken.is_none())
                .min_by(|(a, _), (b, _)| {
                    let offset = |index: usize| {
                        shortest_angle_deg(azimuth_deg, index as f32 * BASE_SECTOR_DEG).abs()
                    };
                    offset(*a).total_cmp(&offset(*b))
                })
                .map(|(index, _)| index);
            match free {
                Some(index) => assigned[index] = Some((*entity, *peer_pos)),
                // Every antenna is already tracking someone.
                None => break,
            }
        }

        for (index, antenna) in base.antennas.iter_mut().enumerate() {
            match assigned[index] {
                Some((_, peer_pos)) => {
                    let (azimuth_deg, elevation_deg) = angles_toward(base_pos, peer_pos);
                    antenna.azimuth_deg = azimuth_deg;
                    antenna.elevation_deg = elevation_deg;
                }
                None => {
                    antenna.azimuth_deg = index as f32 * BASE_SECTOR_DEG;
                    antenna.elevation_deg = BASE_SECTOR_ELEVATION_DEG;
                }
            }
        }

        targets.0 = assigned.iter().map(|slot| slot.map(|(entity, _)| entity)).collect();
    }
}

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
#[allow(clippy::type_complexity)] // Bevy queries describe the component access contract.
pub fn maintain_mesh_antennas(
    mut drones: Query<(
        &Transform,
        &mut Drone,
        &RingIndex,
        &TrackedPeers,
        &MeshTable,
        &DroneKinematics,
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

    for (self_transform, mut drone, self_ring, tracked, table, kin, pairing) in &mut drones {
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

        // `angles_toward` returns a world-frame bearing, but
        // `Antenna::azimuth_deg` is drone-relative, so the drone's own yaw has
        // to come back out of it before the angle is stored. Elevation needs no
        // such adjustment — the airframe stays level, so its pitch frame and the
        // world's agree.
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
                Drone {
                    id: format!("d{ring}"),
                    drone_type: DroneType::Node,
                    antennas: vec![
                        make_antenna(0.0, 0.0, 0),
                        make_antenna(0.0, 0.0, 1),
                        make_antenna(0.0, 0.0, 2),
                    ],
                },
                NetworkingBundle::random(ring),
                // Heading 0 keeps drone-relative and world azimuth identical,
                // so these expectations stay readable as plain bearings.
                DroneKinematics::default(),
            ))
            .id()
    }

    fn spawn_base(world: &mut World, pos: Vec3) {
        world.spawn(Base { id: "base".into(), position: pos, antennas: vec![] });
    }

    /// A station that can actually aim: five sector antennas plus the same
    /// networking state a real one carries, so it matches
    /// `maintain_base_antennas`'s query in full.
    fn spawn_tracking_base(world: &mut World, pos: Vec3) -> Entity {
        let antennas = (0..5)
            .map(|k| make_antenna(k as f32 * BASE_SECTOR_DEG, BASE_SECTOR_ELEVATION_DEG, 200 + k))
            .collect();
        world
            .spawn((
                Transform::from_translation(pos),
                Base { id: "base".into(), position: pos, antennas },
                NetworkingBundle::random(usize::MAX),
                BaseAntennaTargets::default(),
            ))
            .id()
    }

    /// The bearing of whichever base antenna ended up aimed off its resting
    /// sector, if any.
    fn base_tracking_azimuth(world: &World, base: Entity) -> Option<f32> {
        let base_component = world.get::<Base>(base).unwrap();
        base_component
            .antennas
            .iter()
            .enumerate()
            .find(|(index, antenna)| {
                antenna.azimuth_deg != *index as f32 * BASE_SECTOR_DEG
                    || antenna.elevation_deg != BASE_SECTOR_ELEVATION_DEG
            })
            .map(|(_, antenna)| antenna.azimuth_deg)
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

    /// The station aims from what the radio told it — a relayed mesh-table
    /// row — not from the drone's true `Transform`.
    #[test]
    fn base_aims_at_mesh_table_row_not_true_position() {
        let mut world = World::new();
        let base = spawn_tracking_base(&mut world, Vec3::ZERO);
        // True bearing from the station is +X → 90°.
        let drone = spawn_drone(&mut world, Vec3::new(4.0, 0.0, 0.0), 0);
        // The station was told it sits due north instead (bearing 0°).
        set_mesh_row(&mut world, base, drone, Vec3::new(0.0, 0.0, 4.0));

        world.run_system_once(maintain_base_antennas).unwrap();

        let az = base_tracking_azimuth(&world, base).expect("an antenna should be tracking");
        assert!((az - 0.0).abs() < 0.5, "expected ~0° (mesh-table row), got {az}");
        assert!((az - 90.0).abs() > 10.0, "must not aim at the true position (90°)");
        assert_eq!(
            world.get::<BaseAntennaTargets>(base).unwrap().0[0],
            Some(drone),
            "the tracking slot should record whose link it owns"
        );
    }

    /// A direct prediction from the peer's own header beats the relayed row,
    /// same precedence a drone uses.
    #[test]
    fn base_prefers_prediction_over_mesh_table() {
        let mut world = World::new();
        let base = spawn_tracking_base(&mut world, Vec3::ZERO);
        let drone = spawn_drone(&mut world, Vec3::new(4.0, 0.0, 0.0), 0);
        set_mesh_row(&mut world, base, drone, Vec3::new(0.0, 0.0, 4.0));
        // Predicted point bears 45°, between the row (0°) and the truth (90°).
        set_tracked(&mut world, base, drone, Vec3::new(4.0, 0.0, 4.0));

        world.run_system_once(maintain_base_antennas).unwrap();

        let az = base_tracking_azimuth(&world, base).expect("an antenna should be tracking");
        assert!((az - 45.0).abs() < 0.5, "expected ~45° (prediction), got {az}");
    }

    /// A drone the station has never heard of is not aimed at, and the spare
    /// antennas stay resting on their sectors.
    #[test]
    fn base_leaves_unknown_drones_untracked() {
        let mut world = World::new();
        let base = spawn_tracking_base(&mut world, Vec3::ZERO);
        spawn_drone(&mut world, Vec3::new(4.0, 0.0, 0.0), 0);

        world.run_system_once(maintain_base_antennas).unwrap();

        assert!(
            base_tracking_azimuth(&world, base).is_none(),
            "no comms knowledge means no aim"
        );
    }
}
