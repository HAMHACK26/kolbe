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
use crate::networking::RingIndex;

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
    mut drones: Query<(&Transform, &mut Drone, &RingIndex, &TrackedPeers)>,
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

    for (self_transform, mut drone, self_ring, tracked) in &mut drones {
        if n == 0 {
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
