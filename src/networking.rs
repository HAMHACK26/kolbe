//! Peer-to-peer mesh networking: the comms protocol only (headers, packet
//! routing, ranging, mesh-table gossip). Antenna *aiming* — including how
//! `flight_direction` below gets used to keep a lock on a moving peer — is a
//! separate concern and lives in [`crate::tracking`], which reads the data
//! this module produces but never the reverse.
//!
//! A drone "detects a radio link" from a peer when one of its antennas
//! receives that peer above sensitivity — which, with the 1°-beamwidth
//! antennas, can only happen when the two antennas are pointed at each
//! other.
//!
//! On the rising edge of a detected link, the drone emits a header:
//!
//! | id   | connected antenna | flight direction | time received |
//! | ---- | ----------------- | ---------------- | ------------- |
//! | UUID | vector            | vector           | datetime      |
//!
//! `connected antenna` = this drone's position vector relative to the base
//! (not the antenna's own pointing direction).
//!
//! ## Ranging (ping-pong)
//!
//! The peer echoes the header straight back. The originator times the round
//! trip on its own clock and recovers the one-way distance:
//!
//! ```text
//! distance = ((now − time_in_table) − responder_delay) · c / 2
//! ```
//!
//! `time_in_table` is the originator's send time carried in the header, so the
//! measurement stays on one clock even though every drone's clock is
//! independent. Time-of-flight is modeled (positions are known) — the point is
//! to demonstrate the timing method, expressed as a [`SphericalVec`].
//!
//! ## Mesh table (body)
//!
//! Every header send also carries the sender's full picture of the mesh — one
//! row per drone it knows about, itself excluded (the receiver already learns
//! the sender directly, at distance 0, from the header itself):
//!
//! | id   | timestamp | location | neighbour distance | connections |
//! | ---- | --------- | -------- | ------------------- | ----------- |
//! | UUID | datetime  | vector   | int                 | list[UUID]  |
//!
//! `neighbour distance` is hop count (0 = a direct connection). `connections`
//! is that row's drone's own *direct* peers.
//!
//! On receipt (`route_packets`, `PacketKind::Header` arm):
//! - The sender itself is upserted as a distance-0 row (direct connection, so
//!   the receiver stamps it with its **own** clock — direct knowledge is
//!   always fresh).
//! - Every other row in the body is a candidate at `row.neighbour_distance +
//!   1` (one hop further than it was from the sender). A brand-new id is
//!   added at that distance; a known id is only updated if the new path is
//!   strictly shorter (standard distance-vector relaxation).
//! - Relaxed/added rows keep the **incoming** timestamp, never the receiver's
//!   own clock — only a direct connection (distance 0) gets a fresh stamp.
//!   This is how staleness/provenance survives being relayed through the mesh.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use bevy::prelude::*;

use crate::base::Base;
use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::spherical::SphericalVec;
use crate::tracking::TrackedPeers;

/// How far ahead the flight-direction vector predicts (seconds).
pub const FLIGHT_LOOKAHEAD_SECS: f32 = 0.1;

/// How often a header resends while a link stays up (on the sender's own clock).
pub const HEADER_INTERVAL_SECS: f64 = 0.1;

/// Speed of light (km/s) — ranging is in km.
pub const SPEED_OF_LIGHT_KM_S: f64 = 299_792.458;

/// Responder turnaround: how long a drone takes to bounce the header back.
/// Subtracted from the measured round trip before converting to distance.
pub const TURNAROUND_DELAY_S: f64 = 1.0e-6;

// ─── Per-drone identity & clock ────────────────────────────────────────────────

/// Random per-drone UUID (v4-format string). Assigned once at spawn.
#[derive(Component, Clone)]
pub struct DroneUuid(pub String);

impl DroneUuid {
    /// Fresh random v4-format UUID, seeded from wall clock + a spawn counter so
    /// every drone gets a distinct value.
    pub fn random() -> Self {
        let seed = fresh_seed(0x9e3779b97f4a7c15);
        DroneUuid(format_uuid_v4(splitmix64(seed), splitmix64(seed ^ 0xD1B54A32D192ED03)))
    }
}

/// Each drone runs its own clock, started at a random offset from wall time and
/// advanced by frame delta. The clocks drift independently of one another.
#[derive(Component)]
pub struct DroneClock {
    /// Seconds on this drone's own clock (arbitrary epoch).
    pub now: f64,
}

impl DroneClock {
    /// Start each clock at a random point so no two drones agree on the time.
    pub fn random_start() -> Self {
        let seed = fresh_seed(0x2545f4914f6cdd1d);
        // Random offset within a ~day so timestamps look like independent clocks.
        let offset = (splitmix64(seed) % 86_400_000) as f64 / 1000.0;
        DroneClock { now: offset }
    }
}

/// Peers this drone currently has a detected link with, and when the header
/// was last sent to each (on this drone's own clock). A header resends every
/// `HEADER_INTERVAL_SECS` for as long as the link stays up.
#[derive(Component, Default)]
pub struct LinkSet {
    pub connected: std::collections::HashMap<Entity, f64>,
}

/// Distance/direction to each peer, recovered by ranging.
#[derive(Component, Default)]
pub struct RangingResults(pub Vec<(Entity, SphericalVec)>);

/// One row of the mesh body table: what this drone knows about a peer that
/// isn't itself. Keyed by UUID in `MeshTable`, not `Entity` — this is
/// gossiped knowledge, not necessarily anything the drone can see directly.
#[derive(Clone, Debug)]
pub struct MeshRow {
    pub id: String,
    /// The clock time (whichever drone's clock last had direct knowledge of
    /// this row) it was last confirmed fresh — see module docs.
    pub timestamp: f64,
    /// Position vector relative to base.
    pub location: Vec3,
    /// Hop count from this table's owner. 0 = owner has a direct connection.
    pub neighbour_distance: u32,
    /// UUIDs this row's drone is *directly* connected to.
    pub connections: Vec<String>,
}

/// This drone's picture of the mesh: every other drone it knows about,
/// directly or by relay. Never contains an entry for the drone itself.
#[derive(Component, Default)]
pub struct MeshTable(pub HashMap<String, MeshRow>);

/// This drone's fixed position in the ring formation (0..N). Used only to
/// pick its two direct mesh neighbors — see `crate::tracking::maintain_mesh_antennas`.
#[derive(Component)]
pub struct RingIndex(pub usize);

/// Everything a drone needs to take part in the mesh.
#[derive(Bundle)]
pub struct NetworkingBundle {
    pub uuid: DroneUuid,
    pub clock: DroneClock,
    pub links: LinkSet,
    pub sent: SentHeaders,
    pub ranging: RangingResults,
    pub mesh_table: MeshTable,
    pub ring_index: RingIndex,
    pub tracked_peers: TrackedPeers,
}

impl NetworkingBundle {
    pub fn random(ring_index: usize) -> Self {
        Self {
            uuid: DroneUuid::random(),
            clock: DroneClock::random_start(),
            links: LinkSet::default(),
            sent: SentHeaders::default(),
            ranging: RangingResults::default(),
            mesh_table: MeshTable::default(),
            ring_index: RingIndex(ring_index),
            tracked_peers: TrackedPeers::default(),
        }
    }
}

// ─── Header & packets ──────────────────────────────────────────────────────────

/// The header a drone broadcasts when it detects a peer link.
#[derive(Clone, Debug)]
pub struct NetworkHeader {
    /// Sending drone's UUID.
    pub id: String,
    /// Sender's position vector relative to the base.
    pub connected_antenna: Vec3,
    /// Where the drone will be, relative to now, in `FLIGHT_LOOKAHEAD_SECS`
    /// (= velocity · lookahead). Zero while hovering.
    pub flight_direction: Vec3,
    /// Timestamp on the sender's own clock when the link was received.
    pub time_received: f64,
}

/// Log of headers this drone has emitted (most recent last).
#[derive(Component, Default)]
pub struct SentHeaders(pub Vec<NetworkHeader>);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PacketKind {
    /// Originator → peer: the initial header.
    Header,
    /// Peer → originator: the header bounced straight back.
    Echo,
}

/// A packet in flight between two drones.
#[derive(Clone)]
pub struct Packet {
    pub kind: PacketKind,
    /// Drone that started the exchange.
    pub origin: Entity,
    /// Drone that echoes it back.
    pub responder: Entity,
    pub origin_pos: Vec3,
    /// The header itself — only meaningful for `PacketKind::Header`, but kept
    /// (cheaply cloned) on the echo too so ranging can read `time_received`
    /// without a separate field.
    pub header: NetworkHeader,
    /// Sender's mesh body table (its non-self rows) — see module docs.
    /// Only sent with `PacketKind::Header`.
    pub body: Vec<MeshRow>,
    /// UUIDs the sender is *directly* connected to right now — how the
    /// receiver fills in `connections` for the sender's own upserted row.
    pub origin_connections: Vec<String>,

    // Echo-only fields, filled by the responder.
    pub responder_pos: Vec3,
    pub responder_delay: f64,
    /// Modeled arrival time back at the originator, on the originator's clock.
    pub arrival_time: f64,
}

/// In-flight packets, keyed by target drone. Drained every frame.
#[derive(Resource, Default)]
pub struct Mailbox(pub Vec<(Entity, Packet)>);

// ─── Systems ───────────────────────────────────────────────────────────────────

/// Advance every drone's independent clock by the frame delta.
pub fn advance_clocks(time: Res<Time>, mut clocks: Query<&mut DroneClock>) {
    let dt = time.delta_secs_f64();
    for mut clock in &mut clocks {
        clock.now += dt;
    }
}

/// Detect peer radio links and, every `HEADER_INTERVAL_SECS` while a link
/// stays up, emit a header and send it to the peer.
///
/// For every ordered (self, peer) pair, take the best RSSI across self's
/// antennas. `rssi >= sensitivity` means self's antenna is receiving peer —
/// only possible when the antennas face each other.
pub fn detect_links_and_send_headers(
    mut mailbox: ResMut<Mailbox>,
    mut drones: Query<(
        Entity,
        &GlobalTransform,
        &Drone,
        &DroneKinematics,
        &DroneClock,
        &DroneUuid,
        &mut LinkSet,
        &mut SentHeaders,
        &MeshTable,
    )>,
    positions: Query<(Entity, &GlobalTransform), With<Drone>>,
    uuids: Query<&DroneUuid>,
    bases: Query<&Base>,
) {
    // `connected_antenna` is this drone's position vector relative to base —
    // not the antenna's own pointing direction.
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);

    for (self_entity, self_gt, drone, kin, clock, uuid, mut links, mut sent, table) in &mut drones
    {
        let self_pos = self_gt.translation();
        let vector_from_base = self_pos - base_pos;

        // All peers currently detected (regardless of resend cadence) — this
        // is what `connections` means for our own upserted row on the peer.
        let detected: Vec<Entity> = positions
            .iter()
            .filter(|(peer_entity, peer_gt)| {
                *peer_entity != self_entity && {
                    let peer_pos = peer_gt.translation();
                    let distance_km = (peer_pos - self_pos).length();
                    drone.antennas.iter().any(|antenna| {
                        let theta_tx = antenna.off_boresight_deg(self_pos, peer_pos);
                        antenna.rssi_dbm(theta_tx, 0.0, distance_km) >= antenna.sensitivity_dbm
                    })
                }
            })
            .map(|(peer_entity, _)| peer_entity)
            .collect();
        let origin_connections: Vec<String> =
            detected.iter().filter_map(|e| uuids.get(*e).ok().map(|u| u.0.clone())).collect();
        let body: Vec<MeshRow> = table.0.values().cloned().collect();

        let mut detected_now: std::collections::HashMap<Entity, f64> =
            std::collections::HashMap::new();

        for &peer_entity in &detected {
            // Keep this link's last-sent time unless it's due for a resend.
            let last_sent = links.connected.get(&peer_entity).copied();
            let due = match last_sent {
                Some(t) => clock.now - t >= HEADER_INTERVAL_SECS,
                None => true,
            };
            if !due {
                detected_now.insert(peer_entity, last_sent.unwrap());
                continue;
            }
            detected_now.insert(peer_entity, clock.now);

            let header = NetworkHeader {
                id: uuid.0.clone(),
                connected_antenna: vector_from_base,
                flight_direction: kin.velocity * FLIGHT_LOOKAHEAD_SECS,
                time_received: clock.now,
            };
            mailbox.0.push((
                peer_entity,
                Packet {
                    kind: PacketKind::Header,
                    origin: self_entity,
                    responder: peer_entity,
                    origin_pos: self_pos,
                    header: header.clone(),
                    body: body.clone(),
                    origin_connections: origin_connections.clone(),
                    responder_pos: Vec3::ZERO,
                    responder_delay: 0.0,
                    arrival_time: 0.0,
                },
            ));
            sent.0.push(header);
        }

        // Drop links no longer detected so they re-fire on reconnect.
        links.connected = detected_now;
    }
}

/// Route packets: peers echo headers back (ranging) and merge the sender's
/// mesh body table into their own (see module docs for the merge rule).
pub fn route_packets(
    mut mailbox: ResMut<Mailbox>,
    mut drones: Query<(
        &GlobalTransform,
        &mut RangingResults,
        &DroneUuid,
        &DroneClock,
        &mut MeshTable,
        &mut TrackedPeers,
    )>,
) {
    let packets = std::mem::take(&mut mailbox.0);
    let mut outgoing: Vec<(Entity, Packet)> = Vec::new();

    for (target, pkt) in packets {
        match pkt.kind {
            PacketKind::Header => {
                // `target` is the responder.
                let Ok((resp_gt, _, resp_uuid, resp_clock, mut resp_table, mut resp_tracked)) =
                    drones.get_mut(target)
                else {
                    continue;
                };
                let responder_pos = resp_gt.translation();

                // Antenna-tracking only: the sender told us, in this header,
                // where it believes it will be `FLIGHT_LOOKAHEAD_SECS` from
                // when it sent it. Record that predicted point so
                // `crate::tracking::maintain_mesh_antennas` can lead the
                // target instead of aiming at (comms-wise) unknowable live
                // ground truth. This never touches the responder's own
                // position/velocity.
                resp_tracked.0.insert(pkt.origin, pkt.origin_pos + pkt.header.flight_direction);

                // Relay-merge every third-party row: one hop further than it
                // was from the sender, and only if that's an improvement.
                // Timestamp is never touched here — only a direct connection
                // (handled below) is allowed to stamp our own clock.
                for row in &pkt.body {
                    if row.id == resp_uuid.0 || row.id == pkt.header.id {
                        continue;
                    }
                    let candidate_distance = row.neighbour_distance + 1;
                    match resp_table.0.get_mut(&row.id) {
                        Some(existing) if candidate_distance < existing.neighbour_distance => {
                            existing.location = row.location;
                            existing.connections = row.connections.clone();
                            existing.neighbour_distance = candidate_distance;
                            existing.timestamp = row.timestamp;
                        }
                        Some(_) => {}
                        None => {
                            resp_table.0.insert(row.id.clone(), MeshRow {
                                id: row.id.clone(),
                                timestamp: row.timestamp,
                                location: row.location,
                                neighbour_distance: candidate_distance,
                                connections: row.connections.clone(),
                            });
                        }
                    }
                }

                // The sender is a live direct connection right now — always
                // upsert at distance 0 with our own clock's current time.
                resp_table.0.insert(pkt.header.id.clone(), MeshRow {
                    id: pkt.header.id.clone(),
                    timestamp: resp_clock.now,
                    location: pkt.header.connected_antenna,
                    neighbour_distance: 0,
                    connections: pkt.origin_connections.clone(),
                });

                // Send the header straight back for ranging.
                let dist_km = (responder_pos - pkt.origin_pos).length() as f64;
                let prop = dist_km / SPEED_OF_LIGHT_KM_S;

                let mut echo = pkt.clone();
                echo.kind = PacketKind::Echo;
                echo.responder_pos = responder_pos;
                echo.responder_delay = TURNAROUND_DELAY_S;
                // Modeled receive time on the originator's clock:
                // send + round-trip propagation + responder turnaround.
                echo.arrival_time = pkt.header.time_received + 2.0 * prop + TURNAROUND_DELAY_S;
                outgoing.push((pkt.origin, echo));
            }
            PacketKind::Echo => {
                // `target` is the originator — recover distance from timing.
                let Ok((orig_gt, mut results, ..)) = drones.get_mut(target) else { continue };
                let round_trip = pkt.arrival_time - pkt.header.time_received;
                let distance_km =
                    ((round_trip - pkt.responder_delay) * SPEED_OF_LIGHT_KM_S / 2.0) as f32;
                let range =
                    SphericalVec::toward(orig_gt.translation(), pkt.responder_pos, distance_km);
                results.0.push((pkt.responder, range));
            }
        }
    }

    mailbox.0 = outgoing;
}

// ─── Helpers ───────────────────────────────────────────────────────────────────

/// A unique-ish seed from wall clock XOR a monotonic spawn counter.
fn fresh_seed(mix: u64) -> u64 {
    static COUNTER: AtomicU64 = AtomicU64::new(1);
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    nanos ^ COUNTER.fetch_add(1, Ordering::Relaxed).wrapping_mul(mix)
}

/// SplitMix64 — one round. Cheap, decent-quality scalar RNG.
fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9e3779b97f4a7c15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
    z ^ (z >> 31)
}

/// Format two u64 as a RFC-4122 v4-shaped UUID string.
fn format_uuid_v4(hi: u64, lo: u64) -> String {
    // Set version (4) and variant (10xx) bits.
    let hi = (hi & 0xffff_ffff_ffff_0fff) | 0x0000_0000_0000_4000;
    let lo = (lo & 0x3fff_ffff_ffff_ffff) | 0x8000_0000_0000_0000;
    let b = |v: u64, shift: u32| ((v >> shift) & 0xff) as u8;
    format!(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        b(hi, 56), b(hi, 48), b(hi, 40), b(hi, 32),
        b(hi, 24), b(hi, 16),
        b(hi, 8), b(hi, 0),
        b(lo, 56), b(lo, 48),
        b(lo, 40), b(lo, 32), b(lo, 24), b(lo, 16), b(lo, 8), b(lo, 0),
    )
}
