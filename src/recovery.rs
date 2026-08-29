//! Recovery mode: flying back to re-establish a link the mesh can't heal on
//! its own.
//!
//! The reconnection flood in [`crate::networking`] can only reconnect to a
//! drone that's still *somewhere* in the mesh — the request has to reach it
//! through live links. But a drone can drop off the mesh entirely: it drifts
//! out of range, or moves behind terrain, and the last drone still linked to
//! it loses that link. If that drone was the *only* thing connecting the lost
//! one to the rest of the mesh, no flood can reach it — it's partitioned off.
//!
//! These drones are dummies: the lost one won't come find you. So the drone
//! that noticed the loss has to act. Recovery mode is exactly that: "I was
//! this drone's only link and now it's gone — I need to fly back to where I
//! last had contact and try to re-acquire it."
//!
//! ## Deciding I was the sole link (from the lookup table)
//!
//! When a peer `P` drops out of my [`LinkSet`], I consult my mesh lookup
//! table ([`MeshTable`]): does any *other* drone's row still list `P` among
//! its direct `connections`? If someone else still reaches `P`, the mesh will
//! route around the loss and there's nothing for me to recover — the normal
//! reconnection flood can still find `P`. But if `P` appears in nobody
//! else's connections, then I was its last link and it's now partitioned:
//! I enter recovery.
//!
//! This is a best-effort *local* decision from possibly-stale gossip, which
//! is the point — a real drone only has its own table to reason from, not a
//! god's-eye view of the mesh.
//!
//! ## Going back
//!
//! While a link is up I continuously remember my own position, per linked
//! peer, as "where I was when I last had contact with them". On a sole-link
//! loss, that remembered position is the recovery waypoint. I fly back to it
//! under [`crate::navigation`] (recovery is a movement behavior — it *does*
//! move the airframe, unlike the comms/aiming systems). Once `P` re-appears
//! in my links, recovery is done and I hold station again.

use std::collections::{HashMap, HashSet, VecDeque};

use bevy::prelude::*;

use crate::base::Base;
use crate::factories::movement::DroneKinematics;
use crate::navigation::{navigate, DroneState, MovementSpeed};
use crate::networking::{DroneUuid, LinkSet, MeshTable};

/// Close enough to the recovery waypoint to consider "I'm back where I was",
/// km. Past this, the drone holds and lets `seeking` spiral-search from here.
pub const RECOVERY_ARRIVE_KM: f32 = 0.05;

/// Whether this drone is flying a recovery. `Nominal` = normal station-keeping.
#[derive(Component, Default, Clone, Debug, PartialEq)]
pub enum RecoveryState {
    #[default]
    Nominal,
    /// Holding and searching after `lost_peer` drops; once connected again,
    /// the drone returns to the remembered base-relative vector.
    Recovering { lost_peer: String, return_to: Vec3 },
}

/// Two one-second samples of the drone's own displacement from the base.
/// This deliberately stores vectors rather than world positions.
#[derive(Component, Default)]
pub struct PositionHistory {
    elapsed_secs: f32,
    samples: VecDeque<Vec3>,
}

impl PositionHistory {
    fn one_second_ago(&self) -> Option<Vec3> {
        self.samples.front().copied()
    }
}

pub fn record_position_history(
    time: Res<Time>,
    bases: Query<&Base>,
    mut drones: Query<(&Transform, &mut PositionHistory)>,
) {
    let Some(base_pos) = bases.iter().next().map(|base| base.position) else {
        return;
    };
    for (transform, mut history) in &mut drones {
        history.elapsed_secs += time.delta_secs();
        if history.samples.is_empty() || history.elapsed_secs >= 1.0 {
            history.elapsed_secs %= 1.0;
            history.samples.push_back(transform.translation - base_pos);
            while history.samples.len() > 2 {
                history.samples.pop_front();
            }
        }
    }
}

/// Per-peer memory of where this drone was when it last had contact, plus the
/// set of peer UUIDs it was linked to last frame (to spot drops). "Where I
/// was" is stored in world space so it can be flown back to directly.
#[derive(Component, Default)]
pub struct ContactMemory {
    /// peer UUID → this drone's world position when last linked to that peer.
    pub last_contact_pos: HashMap<String, Vec3>,
    /// Peer UUIDs linked last frame — diffed against current links to detect
    /// drops.
    pub prev_linked: HashSet<String>,
}

/// Was `peer_uuid` this drone's sole path to the mesh — i.e. does *no other*
/// drone's lookup-table row still claim a direct connection to it?
///
/// `self_uuid` and the peer's own row are excluded: `self` just lost the
/// link, and the peer listing *its* connections says nothing about who can
/// still reach the peer.
pub fn peer_is_orphaned(table: &MeshTable, peer_uuid: &str, self_uuid: &str) -> bool {
    for (owner_uuid, row) in &table.0 {
        if owner_uuid == peer_uuid || owner_uuid == self_uuid {
            continue;
        }
        if row.connections.iter().any(|c| c == peer_uuid) {
            return false; // someone else still reaches it — not orphaned.
        }
    }
    true
}

/// Update contact memory and, when a sole-link peer drops, enter recovery.
///
/// Runs after links are (re)detected each frame so `LinkSet` is current.
pub fn detect_partitions(
    mut drones: Query<(
        &Transform,
        &DroneUuid,
        &LinkSet,
        &MeshTable,
        &PositionHistory,
        &mut ContactMemory,
        &mut RecoveryState,
    )>,
    uuids: Query<&DroneUuid>,
    bases: Query<&Base>,
) {
    let base_pos = bases.iter().next().map(|base| base.position).unwrap_or(Vec3::ZERO);
    for (transform, self_uuid, links, table, history, mut memory, mut recovery) in &mut drones {
        let self_pos = transform.translation;

        // Current linked-peer UUIDs, and refresh last-contact positions.
        let mut linked_now: HashSet<String> = HashSet::new();
        for &peer_entity in links.connected.keys() {
            if let Ok(peer_uuid) = uuids.get(peer_entity) {
                linked_now.insert(peer_uuid.0.clone());
                memory.last_contact_pos.insert(peer_uuid.0.clone(), self_pos);
            }
        }

        // Any peer linked last frame but not now = a drop this frame.
        let dropped: Vec<String> = memory.prev_linked.difference(&linked_now).cloned().collect();
        memory.prev_linked = linked_now;

        // Only consider entering recovery if not already recovering.
        if matches!(*recovery, RecoveryState::Nominal) {
            for peer in dropped {
                if peer_is_orphaned(table, &peer, &self_uuid.0) {
                    let return_to = history.one_second_ago().unwrap_or(self_pos - base_pos);
                    *recovery = RecoveryState::Recovering { lost_peer: peer, return_to };
                    break; // handle one partition at a time.
                }
            }
        }
    }
}

/// Fly recovering drones back to their last-contact waypoint, and exit
/// recovery once the lost peer is re-acquired.
///
/// Only sets `DroneKinematics::velocity`; `factories::movement::apply_velocity`
/// integrates it — so there's no double integration and non-recovering drones
/// (velocity 0) stay put.
pub fn run_recovery(
    time: Res<Time>,
    mut drones: Query<(
        &Transform,
        &DroneUuid,
        &LinkSet,
        &mut DroneKinematics,
        &mut RecoveryState,
    )>,
    _uuids: Query<&DroneUuid>,
    bases: Query<&Base>,
    speed: Res<MovementSpeed>,
) {
    let dt = time.delta_secs();
    // Same envelope the drift flight uses, so a drone doesn't change pace the
    // moment it enters recovery.
    let limits = speed.limits_km();

    let Some(base_pos) = bases.iter().next().map(|base| base.position) else {
        return;
    };
    for (transform, _self_uuid, links, mut kin, mut recovery) in &mut drones {
        let RecoveryState::Recovering { lost_peer, return_to } = &*recovery else {
            continue;
        };

        // Never fly while disconnected. `seeking` keeps scanning for a peer;
        // the return leg only starts after that scan restores a live link.
        if links.connected.is_empty() {
            kin.velocity = Vec3::ZERO;
            continue;
        }

        let target = base_pos + *return_to;
        if (target - transform.translation).length() <= RECOVERY_ARRIVE_KM {
            kin.velocity = Vec3::ZERO;
            *recovery = RecoveryState::Nominal;
            continue;
        }

        // Rate-limited flight back. navigate() integrates position internally
        // using the clamped velocity; we keep only that velocity and let
        // apply_velocity do the actual integration (identical result, no
        // double step).
        let mut state = DroneState {
            position: transform.translation,
            velocity: kin.velocity,
            heading_deg: kin.heading_deg,
        };
        navigate(&mut state, target, &limits, dt);
        kin.velocity = state.velocity;
        kin.heading_deg = state.heading_deg;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::networking::{MeshRow, MeshTable};

    fn row(id: &str, connections: &[&str]) -> MeshRow {
        MeshRow {
            id: id.into(),
            timestamp: 0.0,
            location: Vec3::ZERO,
            neighbour_distance: 0,
            connections: connections.iter().map(|s| s.to_string()).collect(),
        }
    }

    fn table(rows: Vec<MeshRow>) -> MeshTable {
        MeshTable(rows.into_iter().map(|r| (r.id.clone(), r)).collect())
    }

    /// If nobody else lists a connection to the lost peer, I was its sole
    /// link — orphaned.
    #[test]
    fn sole_link_is_orphaned() {
        // "self" (S) lost peer P. Table has C, D — neither connects to P.
        let t = table(vec![row("C", &["D"]), row("D", &["C"]), row("P", &["S"])]);
        assert!(peer_is_orphaned(&t, "P", "S"));
    }

    /// If some other drone still connects to the lost peer, the mesh can
    /// route around it — not orphaned, no recovery.
    #[test]
    fn peer_still_reachable_is_not_orphaned() {
        // C still lists a connection to P.
        let t = table(vec![row("C", &["P"]), row("D", &["C"]), row("P", &["S", "C"])]);
        assert!(!peer_is_orphaned(&t, "P", "S"));
    }

    /// The peer's own row listing others must not count as "reachable" — a
    /// peer claiming it connects to X says nothing about who reaches the peer.
    #[test]
    fn peers_own_connections_do_not_count() {
        let t = table(vec![row("P", &["X", "Y"])]);
        assert!(peer_is_orphaned(&t, "P", "S"));
    }

    /// My own stale row still listing the peer must not keep me out of
    /// recovery — I just lost it.
    #[test]
    fn self_row_is_excluded() {
        let t = table(vec![row("S", &["P"]), row("P", &["S"])]);
        assert!(peer_is_orphaned(&t, "P", "S"));
    }
}
