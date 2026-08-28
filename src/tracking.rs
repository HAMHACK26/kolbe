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
use crate::networking::{Pairing, RingIndex};

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
/// from a given neighbor (cold start), there's nothing to predict from, so
/// aiming falls back to their actual position just to bootstrap the very
/// first lock.
///
/// This overrides the fixed 120°-apart layout `world::setup` used to compute
/// the initial angles; those were only ever a starting point.
pub fn maintain_mesh_antennas(
    mut drones: Query<(&Transform, &mut Drone, &RingIndex, &TrackedPeers, &Pairing)>,
    positions: Query<(Entity, &Transform, &RingIndex), With<Drone>>,
    bases: Query<&Base>,
) {
    let base_pos = bases.iter().next().map(|b| b.position);

    // Ring neighbors, not "the other drone" — with N > 2 most pairs are
    // never mutually visible on purpose, so relayed (multi-hop) rows in the
    // mesh table actually get exercised.
    let mut ring: Vec<(usize, Entity, Vec3)> =
        positions.iter().map(|(e, t, ri)| (ri.0, e, t.translation)).collect();
    ring.sort_by_key(|(i, ..)| *i);
    let n = ring.len();

    for (self_transform, mut drone, self_ring, tracked, pairing) in &mut drones {
        if n == 0 {
            continue;
        }
        // "Stopped" for a reconnection handshake — hold antenna slew steady so
        // a lock can be acquired/kept. Don't re-aim this frame.
        if pairing.frozen {
            continue;
        }
        let self_pos = self_transform.translation;
        let (_, next_entity, next_true_pos) = ring[(self_ring.0 + 1) % n];
        let (_, prev_entity, prev_true_pos) = ring[(self_ring.0 + n - 1) % n];
        // Lead the target using its self-reported prediction when we have
        // one; otherwise fall back to ground truth to bootstrap the lock.
        let next_pos = tracked.0.get(&next_entity).copied().unwrap_or(next_true_pos);
        let prev_pos = tracked.0.get(&prev_entity).copied().unwrap_or(prev_true_pos);

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

        if let Some(first) = drone.antennas.get_mut(0) {
            let (az, el) = angles_toward(self_pos, next_pos);
            first.azimuth_deg = az;
            first.elevation_deg = el;
        }
        if let (Some(base_pos), Some(second)) = (base_pos, drone.antennas.get_mut(1)) {
            let (az, el) = angles_toward(self_pos, base_pos);
            second.azimuth_deg = az;
            second.elevation_deg = el;
        }
        if let Some(third) = drone.antennas.get_mut(2) {
            let (az, el) = angles_toward(self_pos, prev_pos);
            third.azimuth_deg = az;
            third.elevation_deg = el;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::system::RunSystemOnce;

    use crate::drone::{make_antenna, DroneType};
    use crate::networking::NetworkingBundle;

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
            ))
            .id()
    }

    fn spawn_base(world: &mut World, pos: Vec3) {
        world.spawn(Base { id: "base".into(), position: pos, antennas: vec![] });
    }

    fn antenna0_az(world: &World, drone: Entity) -> f32 {
        world.get::<Drone>(drone).unwrap().antennas[0].azimuth_deg
    }

    /// Set drone `owner`'s tracked prediction for `peer`.
    fn set_tracked(world: &mut World, owner: Entity, peer: Entity, predicted: Vec3) {
        world.get_mut::<TrackedPeers>(owner).unwrap().0.insert(peer, predicted);
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

        // No prediction yet → aims at true B (+X, 90°).
        world.run_system_once(maintain_mesh_antennas).unwrap();
        let az_true = antenna0_az(&world, a);
        assert!((az_true - 90.0).abs() < 0.5, "cold start aims at true pos, got {az_true}");

        // Prediction slides in +Z; azimuth should swing monotonically toward 45°.
        set_tracked(&mut world, a, b, Vec3::new(1.0, 0.0, 0.5));
        world.run_system_once(maintain_mesh_antennas).unwrap();
        let az_mid = antenna0_az(&world, a);

        set_tracked(&mut world, a, b, Vec3::new(1.0, 0.0, 1.0));
        world.run_system_once(maintain_mesh_antennas).unwrap();
        let az_far = antenna0_az(&world, a);

        assert!(
            az_true > az_mid && az_mid > az_far,
            "antenna should track the moving prediction: {az_true} > {az_mid} > {az_far}"
        );
        assert!((az_far - 45.0).abs() < 0.5, "final aim should reach ~45°, got {az_far}");
    }
}
