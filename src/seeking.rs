//! Spiral search: reacquiring a peer whose direct link has dropped.
//!
//! [`crate::tracking`] keeps a *live* link aimed correctly by leading the
//! target with its self-reported flight direction. But if a link is already
//! lost — a header was missed, the peer maneuvered harder than predicted,
//! whatever — there's no fresh flight-direction to lead with anymore. All
//! that's left is the peer's last confirmed position, sitting in the mesh
//! lookup table ([`crate::networking::MeshTable`]), getting staler by the
//! second.
//!
//! This module looks up that last-known vector, works out where the drone
//! is *relative to itself* (`target_position − self_position`), and — since
//! the target could by now be anywhere within a cone of plausible positions
//! around that stale point — sweeps the antenna outward through that cone
//! in an expanding spiral until it hits the target again (RSSI back above
//! sensitivity, observed next frame by
//! [`crate::networking::detect_links_and_send_headers`]) or the sweep
//! completes and restarts.
//!
//! Like [`crate::tracking`], this only ever decides which way an antenna
//! points. It never touches a drone's own position, velocity, or
//! navigation.
//!
//! ## The search cone
//!
//! ```text
//! θ = 2 · arctan(Δr / R)
//! ```
//!
//! - `θ` — full angle of the cone that must be searched to be confident of
//!   covering the target (see [`search_cone_angle_deg`]).
//! - `Δr` — position-uncertainty radius: how far the target could plausibly
//!   have moved since its last confirmed position, worst case
//!   (see [`worst_case_uncertainty_radius_km`]).
//! - `R` — straight-line distance from this drone to the target's last
//!   known position.
//!
//! `Δr` grows with how long it's been since the mesh table was last updated
//! for that peer and with how fast both drones could plausibly be moving
//! apart (`Δr = v_rel · Δt`, `v_rel = 2 · v_max`, both at top speed, headed
//! straight away from each other — the worst case). The longer the silence,
//! the wider the cone that has to be searched; a peer heard from a moment
//! ago barely needs any search at all.
//!
//! ## Scan speed, spread by the golden ratio
//!
//! ```text
//! ω_i = ω_min + frac(ID_i · φ) · (ω_max − ω_min)
//! ```
//!
//! Every drone gets its own angular scan speed inside `[ω_min, ω_max]`,
//! chosen from its ID via the golden ratio `φ`. `ID_i` is the drone's real
//! identity — its UUID, folded to an integer by [`uuid_to_u64`] — not its
//! formation ring slot. If every drone searched at the same rate, two drones
//! both spiral-searching for *each other* could stay perfectly out of phase
//! indefinitely — always sweeping past each other's position at the same
//! moment, never overlapping (the same lockstep problem that makes naive
//! round-robin polling unreliable). `φ` is the "most irrational" number — the
//! golden-ratio spacing used for phyllotaxis/sunflower-seed spirals — so
//! `frac(ID_i · φ)` spreads drone IDs across `[0, 1)` about as evenly and
//! non-repeatingly as possible, which keeps any two drones' scan rates from
//! drifting into sync.

use std::collections::HashSet;

use bevy::prelude::*;

use crate::antenna::angles_toward;
use crate::base::Base;
use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::navigation::FlightLimits;
use crate::networking::{
    DroneClock, DroneUuid, LinkSet, MeshRow, MeshTable, Pairing, PairingState, ReconnectRequests,
    RingIndex,
};

/// Mechanical/electronic scan speed floor, rad/s (~4.8 rpm).
pub const OMEGA_MIN_RAD_S: f32 = 0.5;
/// Mechanical/electronic scan speed ceiling, rad/s (~57 rpm).
pub const OMEGA_MAX_RAD_S: f32 = 6.0;

/// How many full spiral turns it takes to sweep from the center out to the
/// edge of the search cone before the pattern resets and sweeps outward
/// again from scratch. Not part of the given search-cone/scan-speed specs —
/// a real search radar restarts its spiral once it's covered the whole
/// uncertainty cone without a hit, and this is how many turns that takes.
/// 4 is a reasonable, tunable middle ground: too few turns leaves gaps
/// between passes near the outer edge, too many makes each full sweep take
/// unnecessarily long to complete.
pub const SPIRAL_TURNS_PER_SWEEP: f32 = 4.0;

/// θ = 2·arctan(Δr / R) — the full angular width of the cone that must be
/// searched to be confident of covering a target that was last confirmed at
/// range `target_distance_km`, given it could have moved as far as
/// `uncertainty_radius_km` since then. Returns degrees.
///
/// This is the *full* cone angle, matching the θ in the spec — a spiral
/// search sweeping outward from the center wants the *half*-angle (θ/2) as
/// its outer radius.
pub fn search_cone_angle_deg(uncertainty_radius_km: f32, target_distance_km: f32) -> f32 {
    if target_distance_km <= f32::EPSILON {
        // Last-known position is (effectively) right on top of us — there's
        // no meaningful direction to narrow the search to, so search
        // everywhere.
        return 180.0;
    }
    2.0 * (uncertainty_radius_km / target_distance_km).atan().to_degrees()
}

/// Δr = v_rel · Δt, with the worst-case relative speed v_rel = 2·v_max (both
/// drones at top speed, separating head-on).
///
/// `max_speed_mps` is a real-world speed limit
/// ([`crate::navigation::FlightLimits::max_speed_mps`], meters/second); the
/// simulated world is scaled in kilometers, so this converts to km/s before
/// scaling by elapsed time. Returns kilometers.
pub fn worst_case_uncertainty_radius_km(max_speed_mps: f32, elapsed_secs: f32) -> f32 {
    let v_rel_km_s = 2.0 * (max_speed_mps / 1000.0);
    v_rel_km_s * elapsed_secs.max(0.0)
}

/// ω_i = ω_min + frac(ID_i · φ) · (ω_max − ω_min) — this drone's angular
/// scan speed, spread across `[omega_min, omega_max]` by the golden ratio.
/// See the module docs for why this avoids lockstep between searching
/// drones.
///
/// `drone_id` is the drone's real identity folded to a `u64` — see
/// [`uuid_to_u64`]. It is *not* the formation ring index (which isn't a real
/// identity and would collide across differently-sized formations).
///
/// `frac(ID · φ)` is computed by integer golden-ratio (Knuth multiplicative)
/// hashing, *not* by `ID as f64 * φ`: a real `u64` id is ~10^19, far past
/// f64's 2^53 exact-integer limit, so the float route loses all fractional
/// bits and collapses every drone to `omega_min`. Since `ID·φ = ID + ID·(φ−1)`
/// and `frac` drops the integer `ID`, `frac(ID·φ) = frac(ID·(φ−1))`, and
/// `round(2^64 · (φ−1)) = 0x9E3779B97F4A7C15` is exactly Knuth's constant —
/// so `(ID · that) >> ` gives `frac(ID·φ)` in `[0, 1)`, exact for all u64.
pub fn scan_angular_speed_rad_s(drone_id: u64, omega_min: f32, omega_max: f32) -> f32 {
    const GOLDEN_U64: u64 = 0x9E37_79B9_7F4A_7C15; // round(2^64 / φ)
    let frac = drone_id.wrapping_mul(GOLDEN_U64) as f64 / (u64::MAX as f64 + 1.0);
    omega_min + frac as f32 * (omega_max - omega_min)
}

/// Fold a drone's UUID string into a stable `u64` for formulas that need a
/// unique integer identity (here, the golden-ratio scan-speed spread). The
/// real unique id is the UUID; this is just a lossless-enough hash of it,
/// not a separate identity. FNV-1a over the bytes, diffused with one
/// SplitMix64 round.
pub fn uuid_to_u64(uuid: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in uuid.bytes() {
        h ^= byte as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    // SplitMix64 finalizer — diffuse so near-identical UUID tails don't map
    // to near-identical integers.
    let mut z = h.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// A point on an expanding, then-resetting (sawtooth) Archimedean spiral:
/// angle advances at `omega_rad_s`, radius grows linearly from 0 to
/// `max_radius_deg` over `turns_per_sweep` full turns, then the whole
/// pattern snaps back to the center and sweeps outward again. Returns
/// `(azimuth_offset_deg, elevation_offset_deg)` to add on top of a boresight
/// already aimed at the target's last-known direction.
pub fn spiral_offset_deg(
    elapsed_secs: f32,
    omega_rad_s: f32,
    max_radius_deg: f32,
    turns_per_sweep: f32,
) -> (f32, f32) {
    if max_radius_deg <= 0.0 {
        return (0.0, 0.0);
    }
    let turn_angle_rad = omega_rad_s * elapsed_secs;
    let sweep_period_rad = 2.0 * std::f32::consts::PI * turns_per_sweep;
    // Sawtooth 0→1 over one full sweep, then repeats.
    let progress = (turn_angle_rad / sweep_period_rad).rem_euclid(1.0);
    let radius_deg = max_radius_deg * progress;
    (radius_deg * turn_angle_rad.cos(), radius_deg * turn_angle_rad.sin())
}

/// Per-drone spiral-search progress for each ring slot (antenna #1 → next
/// neighbor, antenna #3 → previous neighbor). Reset to zero the instant
/// that slot's link is confirmed again, so the *next* loss always restarts
/// the spiral from the center instead of resuming at whatever radius it
/// last reached.
#[derive(Component, Default)]
pub struct SeekState {
    pub next_elapsed_secs: f32,
    pub prev_elapsed_secs: f32,
}

/// When a ring neighbor's direct link has dropped, spiral-search around its
/// last known position (from the mesh lookup table) instead of sitting
/// locked onto an increasingly stale point.
///
/// Only touches antenna slots that are currently *not* linked — a live link
/// is left entirely to `tracking::maintain_mesh_antennas`'s predictive aim.
/// Must run after that system each frame: it overrides whichever of
/// antennas #1/#3 have gone unlinked with a spiral offset on top of the
/// same "aim straight at the last-known point" baseline tracking would
/// otherwise leave them at.
#[allow(clippy::type_complexity)] // Bevy queries describe the component access contract.
pub fn seek_lost_links(
    time: Res<Time>,
    mut drones: Query<(
        &Transform,
        &mut Drone,
        &RingIndex,
        &DroneUuid,
        &LinkSet,
        &MeshTable,
        &DroneClock,
        &DroneKinematics,
        &mut SeekState,
        &Pairing,
    )>,
    positions: Query<(Entity, &Transform, &RingIndex, &DroneUuid), With<Drone>>,
    bases: Query<&Base>,
) {
    let dt = time.delta_secs();
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);
    // `navigation` isn't wired up as a live-updatable Bevy resource yet
    // (nothing in the running app currently calls `set_max_speed`), so this
    // reads its default. If/when it is promoted to a `Res<FlightLimits>`,
    // this should switch to reading that resource instead of constructing a
    // fresh default here.
    let max_speed_mps = FlightLimits::default().max_speed_mps;

    let mut ring: Vec<(usize, Entity, String)> =
        positions.iter().map(|(e, _, ri, uuid)| (ri.0, e, uuid.0.clone())).collect();
    ring.sort_by_key(|(i, ..)| *i);
    let n = ring.len();

    for (
        self_transform,
        mut drone,
        self_ring,
        self_uuid,
        links,
        table,
        clock,
        kin,
        mut seek,
        pairing,
    ) in &mut drones
    {
        if n == 0 {
            continue;
        }
        // "Stopped" for a reconnection handshake — hold antenna slew steady,
        // don't spiral-search this frame.
        if pairing.frozen {
            continue;
        }
        let self_pos = self_transform.translation;
        // Scan speed keys off the drone's real UUID identity, not its
        // formation ring slot.
        let omega = scan_angular_speed_rad_s(
            uuid_to_u64(&self_uuid.0),
            OMEGA_MIN_RAD_S,
            OMEGA_MAX_RAD_S,
        );

        let (_, next_entity, next_uuid) = ring[(self_ring.0 + 1) % n].clone();
        let (_, prev_entity, prev_uuid) = ring[(self_ring.0 + n - 1) % n].clone();

        seek_one_slot(SeekSlotArgs {
            drone: &mut drone,
            antenna_idx: 0,
            neighbor_entity: next_entity,
            neighbor_uuid: &next_uuid,
            links,
            table,
            self_pos,
            base_pos,
            self_heading_deg: kin.heading_deg,
            self_clock_now: clock.now,
            max_speed_mps,
            omega_rad_s: omega,
            dt,
            elapsed: &mut seek.next_elapsed_secs,
        });
        seek_one_slot(SeekSlotArgs {
            drone: &mut drone,
            antenna_idx: 2,
            neighbor_entity: prev_entity,
            neighbor_uuid: &prev_uuid,
            links,
            table,
            self_pos,
            base_pos,
            self_heading_deg: kin.heading_deg,
            self_clock_now: clock.now,
            max_speed_mps,
            omega_rad_s: omega,
            dt,
            elapsed: &mut seek.prev_elapsed_secs,
        });
    }
}

struct SeekSlotArgs<'a> {
    drone: &'a mut Drone,
    antenna_idx: usize,
    neighbor_entity: Entity,
    neighbor_uuid: &'a str,
    links: &'a LinkSet,
    table: &'a MeshTable,
    self_pos: Vec3,
    base_pos: Vec3,
    /// This drone's own yaw. `Antenna::azimuth_deg` is drone-relative, so the
    /// world-frame bearing computed here has to be brought into that frame.
    self_heading_deg: f32,
    self_clock_now: f64,
    max_speed_mps: f32,
    omega_rad_s: f32,
    dt: f32,
    elapsed: &'a mut f32,
}

fn seek_one_slot(args: SeekSlotArgs) {
    let SeekSlotArgs {
        drone,
        antenna_idx,
        neighbor_entity,
        neighbor_uuid,
        links,
        table,
        self_pos,
        base_pos,
        self_heading_deg,
        self_clock_now,
        max_speed_mps,
        omega_rad_s,
        dt,
        elapsed,
    } = args;

    if links.connected.contains_key(&neighbor_entity) {
        // Locked — nothing to search for. Reset so the *next* loss starts
        // the spiral over from the center, not wherever it last reached.
        *elapsed = 0.0;
        return;
    }
    let Some(row) = table.0.get(neighbor_uuid) else {
        // No lookup-table entry for them at all yet — nothing to seek from.
        return;
    };

    // The vector from this drone to the target, computed from the lookup
    // table's last-known position — not from any live/omniscient state.
    let target_pos = base_pos + row.location;
    let target_distance_km = (target_pos - self_pos).length();

    // Elapsed time since that lookup-table entry was last confirmed fresh.
    // `row.timestamp` may originate from a different drone's independent
    // clock if this entry was relayed rather than observed directly, so
    // this is an approximation — consistent with how the rest of the mesh
    // table already treats these timestamps as "seconds on whichever clock
    // last touched it".
    let time_since_update = (self_clock_now - row.timestamp).max(0.0) as f32;

    let uncertainty_km = worst_case_uncertainty_radius_km(max_speed_mps, time_since_update);
    let cone_deg = search_cone_angle_deg(uncertainty_km, target_distance_km);
    let half_cone_deg = cone_deg / 2.0;

    *elapsed += dt;
    let (delta_az, delta_el) =
        spiral_offset_deg(*elapsed, omega_rad_s, half_cone_deg, SPIRAL_TURNS_PER_SWEEP);

    // `angles_toward` is a world-frame bearing but `Antenna::azimuth_deg` is
    // drone-relative, so the drone's own yaw comes back out of it. Elevation
    // needs no such correction — the airframe stays level.
    let (center_az, center_el) = angles_toward(self_pos, target_pos);
    if let Some(antenna) = drone.antennas.get_mut(antenna_idx) {
        antenna.azimuth_deg = (center_az - self_heading_deg + delta_az).rem_euclid(360.0);
        antenna.elevation_deg = (center_el + delta_el).clamp(-90.0, 90.0);
    }
}

// ─── Reconnecting to the closest drone ────────────────────────────────────────

/// A peer with this many direct connections or fewer is treated as fragile and
/// will not be dropped to free an antenna.
///
/// The reasoning is about what the mesh loses, not what this drone gains. A
/// peer with three or more links has redundancy — cut one and it is still
/// reachable another way. A peer down to two or fewer is at or near the point
/// where *this* link is what keeps it attached, so trading it for a marginally
/// closer neighbour risks partitioning the drone off the mesh entirely. The
/// closer link is not worth that.
pub const FRAGILE_PEER_CONNECTIONS: usize = 2;

/// How much closer a candidate must be before it is worth displacing an
/// existing link, as a fraction of the current link's range.
///
/// Without this a pair of peers at nearly equal range would swap back and
/// forth every frame, each swap making the other look marginally better.
pub const RECONNECT_IMPROVEMENT: f32 = 0.9;

/// Reconnect each drone to the closest drone it knows about — unless doing so
/// would mean dropping a peer that can't spare the link.
///
/// Everything here is read out of the mesh lookup table
/// ([`crate::networking::MeshTable`]), never from a peer's live `Transform`.
/// Both facts this needs — where a peer is (`row.location`, base-relative) and
/// how many links it has (`row.connections`) — are things the drone was
/// *told*, directly or by relay. A drone has no other way to know them.
///
/// Per drone, per frame:
///
/// 1. Find the nearest known peer that isn't already linked.
/// 2. If an antenna is free, just take it — nothing has to be given up.
/// 3. Otherwise the nearest *existing* link has to be displaced, so pick the
///    farthest one as the candidate to drop, and only proceed if the new peer
///    is meaningfully closer ([`RECONNECT_IMPROVEMENT`]).
/// 4. Refuse the trade if that drop candidate has
///    [`FRAGILE_PEER_CONNECTIONS`] connections or fewer.
///
/// The actual link change is not made here — this only queues the attempt on
/// [`ReconnectRequests`], which `networking::process_reconnect` turns into the
/// priority handshake flood.
#[allow(clippy::type_complexity)] // Bevy queries describe the component access contract.
pub fn reconnect_to_closest(
    mut requests: ResMut<ReconnectRequests>,
    drones: Query<
        (Entity, &Transform, &Drone, &DroneUuid, &LinkSet, &MeshTable, &Pairing),
        With<Drone>,
    >,
    uuids: Query<&DroneUuid>,
    bases: Query<&Base>,
) {
    let Some(base_pos) = bases.iter().next().map(|b| b.position) else {
        // Every mesh-table location is base-relative, so without a base there
        // is no frame to resolve them in.
        return;
    };

    for (self_entity, transform, drone, self_uuid, links, table, pairing) in &drones {
        // One handshake at a time — don't pile a second request on a drone
        // that is already mid-flood or stopped for a lock.
        if pairing.state != PairingState::Idle || pairing.frozen {
            continue;
        }
        if requests.0.iter().any(|(entity, _)| *entity == self_entity) {
            continue;
        }

        let self_pos = transform.translation;
        let linked_uuids: HashSet<String> = links
            .connected
            .keys()
            .filter_map(|entity| uuids.get(*entity).ok().map(|u| u.0.clone()))
            .collect();

        // Range to a peer, as the lookup table describes it.
        let range_to = |row: &MeshRow| (base_pos + row.location - self_pos).length();

        // 1. Nearest known peer we are not already talking to.
        let closest = table
            .0
            .values()
            .filter(|row| row.id != self_uuid.0 && !linked_uuids.contains(&row.id))
            .min_by(|a, b| range_to(a).total_cmp(&range_to(b)));
        let Some(closest) = closest else {
            continue;
        };

        // 2. A free antenna means nothing has to be given up.
        if links.connected.len() < drone.antennas.len() {
            requests.0.push((self_entity, closest.id.clone()));
            continue;
        }

        // 3. Otherwise the farthest current link is what would be displaced.
        let drop_candidate = table
            .0
            .values()
            .filter(|row| linked_uuids.contains(&row.id))
            .max_by(|a, b| range_to(a).total_cmp(&range_to(b)));
        let Some(drop_candidate) = drop_candidate else {
            // Linked to peers we have no table rows for — nothing to reason
            // about, so don't trade blind.
            continue;
        };
        if range_to(closest) >= range_to(drop_candidate) * RECONNECT_IMPROVEMENT {
            continue;
        }

        // 4. Never strand a peer that is down to its last links.
        if drop_candidate.connections.len() <= FRAGILE_PEER_CONNECTIONS {
            continue;
        }

        requests.0.push((self_entity, closest.id.clone()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A target confirmed "just now" (zero elapsed time) needs essentially
    /// no search cone at all — Δr ≈ 0, so θ ≈ 0.
    #[test]
    fn fresh_sighting_needs_almost_no_search_cone() {
        let uncertainty = worst_case_uncertainty_radius_km(15.0, 0.0);
        let cone = search_cone_angle_deg(uncertainty, 1.0);
        assert!(cone < 0.01, "cone {cone} should be ~0 for zero elapsed time");
    }

    /// The longer it's been since a sighting, the wider the search cone
    /// needs to be.
    #[test]
    fn search_cone_grows_with_elapsed_time() {
        let target_distance_km = 1.0;
        let cone_at_1s = search_cone_angle_deg(
            worst_case_uncertainty_radius_km(15.0, 1.0),
            target_distance_km,
        );
        let cone_at_10s = search_cone_angle_deg(
            worst_case_uncertainty_radius_km(15.0, 10.0),
            target_distance_km,
        );
        assert!(cone_at_10s > cone_at_1s);
    }

    /// Two adjacent drone IDs should not land on (near-)identical scan
    /// speeds — that's the entire point of spreading them by the golden
    /// ratio.
    #[test]
    fn adjacent_ids_get_visibly_different_scan_speeds() {
        for id in 0..20u64 {
            let a = scan_angular_speed_rad_s(id, OMEGA_MIN_RAD_S, OMEGA_MAX_RAD_S);
            let b = scan_angular_speed_rad_s(id + 1, OMEGA_MIN_RAD_S, OMEGA_MAX_RAD_S);
            assert!(
                (a - b).abs() > 0.05,
                "ids {id} and {} got near-identical omega ({a} vs {b})",
                id + 1
            );
        }
    }

    /// Distinct UUIDs must fold to distinct integers and, in turn, distinct
    /// scan speeds — the whole reason we key on UUID rather than ring slot.
    #[test]
    fn distinct_uuids_get_distinct_scan_speeds() {
        let uuids = [
            "12b3a678-ed22-4d7d-9af1-2e97295f3c3b",
            "87bb4046-5d73-4732-b3d4-d67029beb770",
            "002da70d-7a81-4b3d-88b4-16db19f12fcc",
            "09e479b3-4b34-4466-ad35-ec41f5e0d996",
        ];
        let speeds: Vec<f32> = uuids
            .iter()
            .map(|u| scan_angular_speed_rad_s(uuid_to_u64(u), OMEGA_MIN_RAD_S, OMEGA_MAX_RAD_S))
            .collect();
        for i in 0..speeds.len() {
            for j in (i + 1)..speeds.len() {
                assert!(
                    (speeds[i] - speeds[j]).abs() > 0.05,
                    "uuids {} and {} got near-identical omega ({} vs {})",
                    uuids[i], uuids[j], speeds[i], speeds[j]
                );
            }
        }
    }

    /// Scan speed must always stay within the configured envelope.
    #[test]
    fn scan_speed_stays_within_bounds() {
        for id in 0..100u64 {
            let omega = scan_angular_speed_rad_s(id, OMEGA_MIN_RAD_S, OMEGA_MAX_RAD_S);
            assert!((OMEGA_MIN_RAD_S..=OMEGA_MAX_RAD_S).contains(&omega));
        }
    }

    /// The spiral must never sweep outside the requested cone radius.
    #[test]
    fn spiral_never_exceeds_max_radius() {
        let max_radius = 12.0;
        for i in 0..2000 {
            let t = i as f32 * 0.01;
            let (az, el) = spiral_offset_deg(t, 2.0, max_radius, SPIRAL_TURNS_PER_SWEEP);
            let radius = (az * az + el * el).sqrt();
            assert!(radius <= max_radius + 1e-3, "radius {radius} exceeded max {max_radius} at t={t}");
        }
    }

    /// The spiral starts at dead center (no offset) at t=0.
    #[test]
    fn spiral_starts_at_center() {
        let (az, el) = spiral_offset_deg(0.0, 2.0, 12.0, SPIRAL_TURNS_PER_SWEEP);
        assert_eq!((az, el), (0.0, 0.0));
    }
}
