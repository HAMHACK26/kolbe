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
//!
//! ## Reconnection handshake (priority)
//!
//! Reconnecting to a specific drone that's currently linked to someone else
//! — so it isn't reachable directly, only by relay — is a three-message
//! handshake, flooded hop-by-hop through every drone's *current* live links
//! ([`LinkSet`]) rather than sent point-to-point:
//!
//! 1. **Request** — the requester floods `Request { id, requester, target }`.
//!    Every drone that isn't `target` just repeats it onward to its own
//!    links. When `target` receives it, `target` decides accept or not (see
//!    [`Pairing`]): if it accepts, it **stops** whatever antenna behavior it
//!    was doing for that slot and floods an `Accept` back; if not, it just
//!    continues — no reply.
//! 2. **Accept** — floods back the same way. When it reaches `requester`,
//!    *requester* makes the same stop-or-continue call: if this is still the
//!    request it's waiting on, it stops (commits) and floods a `Position`;
//!    if it already committed to a different accept (or isn't waiting on
//!    this id anymore), it just continues — the accept is dropped.
//! 3. **Position** — floods to `target`, carrying the requester's position.
//!    Receiving it is what "the pairing begins" means: both sides now know
//!    the other's identity and the requester's position.
//!
//! Two rules make this work as a flood instead of infinite re-broadcast:
//!
//! - **No throttle.** Unlike headers (`HEADER_INTERVAL_SECS`), these are
//!   forwarded the instant they're received — priority traffic, not gossip.
//! - **Dedup by `(id, phase)`.** Every drone remembers which
//!   `(request_id, phase)` pairs it has already processed ([`Pairing::seen`]).
//!   Seeing the same pair again — whether as the addressed recipient or just a
//!   repeater — is a no-op: don't reprocess, don't re-forward. This is what
//!   stops the flood from looping forever around the mesh. (Keyed on
//!   `(id, phase)` rather than bare id so a repeater that already forwarded
//!   the `Request` still forwards the later `Accept`/`Position` for the same
//!   id — split-horizon on `from` handles the rest.)

use std::collections::HashMap;
use std::collections::HashSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use bevy::prelude::*;

use crate::antenna::Antennas;
use crate::base::Base;
use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::spherical::SphericalVec;
use crate::world::MAX_NEIGHBOR_SPACING_KM;
use crate::tracking::TrackedPeers;

/// How far ahead the flight-direction vector predicts (seconds).
pub const FLIGHT_LOOKAHEAD_SECS: f32 = 0.1;

/// How often a header resends while a link stays up (on the sender's own clock).
pub const HEADER_INTERVAL_SECS: f64 = 0.1;

/// Desired number of simultaneous direct radio peers per drone.
pub const TARGET_DIRECT_CONNECTIONS: usize = 3;

/// Speed of light (km/s) — ranging is in km.
pub const SPEED_OF_LIGHT_KM_S: f64 = 299_792.458;

/// Responder turnaround: how long a drone takes to bounce the header back.
/// Subtracted from the measured round trip before converting to distance.
pub const TURNAROUND_DELAY_S: f64 = 1.0e-6;

/// How long a drone waits on an unanswered handshake before giving up, on its
/// own clock.
///
/// Without this a single unanswered `Request` wedges the requester in
/// `AwaitingAccept` forever — it can never originate another — and an
/// unanswered `Accept` leaves the target frozen with its antennas held. Both
/// are dead ends the protocol has no other way out of.
pub const RECONNECT_TIMEOUT_SECS: f64 = 5.0;

/// How long a drone holds off after a failed handshake before asking again,
/// on its own clock. Stops a drone that keeps failing from re-flooding the
/// mesh every frame.
pub const RECONNECT_RETRY_SECS: f64 = 10.0;

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
    /// The local antenna reserved for each live peer session. A peer can have
    /// only one active antenna on this node, even if beams overlap.
    pub antenna_for_peer: std::collections::HashMap<Entity, usize>,
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
pub struct RingIndex(#[allow(dead_code)] pub usize);

/// Where this drone currently is in (at most) one in-flight reconnection
/// handshake. See the module docs, "Reconnection handshake (priority)".
#[derive(Clone, Debug, Default, PartialEq)]
pub enum PairingState {
    #[default]
    Idle,
    /// I flooded a `Request` for `target` and I'm waiting on its `Accept`.
    AwaitingAccept { request_id: String, target: String },
    /// I accepted `requester`'s `Request` and stopped for them; waiting on
    /// their `Position` to complete the pairing.
    AcceptedAwaitingPosition { request_id: String, requester: String },
    /// Handshake complete — paired with `peer`.
    Paired { request_id: String, peer: String },
}

/// This drone's reconnection-handshake state.
///
/// - `state` — where this drone is in (at most) one in-flight handshake.
/// - `seen` — every `(request_id, phase)` it has already processed, as either
///   the addressed party or a mid-flood repeater. A duplicate sighting of the
///   same pair is dropped: not reprocessed, not re-forwarded. Keyed on
///   *`(id, phase)`*, not bare id, so forwarding the `Request` phase doesn't
///   make a repeater wrongly swallow the later `Accept`/`Position` phases that
///   share the same id.
/// - `frozen` — when true, this drone has "stopped": antenna slew is held so a
///   lock can be acquired/kept (`tracking`/`seeking` skip re-aiming). Never
///   touches the airframe — flight stays under `navigation`.
/// - `paired_peer_pos` — the requester's base-relative position, learned by
///   the target when the `Position` message lands. `Some` once paired.
///
/// See the module docs, "Reconnection handshake (priority)".
#[derive(Component, Default)]
pub struct Pairing {
    pub state: PairingState,
    pub seen: HashSet<(String, u8)>,
    pub frozen: bool,
    pub paired_peer_pos: Option<Vec3>,
    /// When the current state was entered, on this drone's own clock. Only
    /// meaningful while waiting on a reply — see [`expire_stale_handshakes`].
    pub state_since: f64,
    /// Earliest own-clock time this drone may originate another request. Set
    /// when a handshake times out, so a repeatedly failing drone backs off
    /// instead of re-flooding every frame.
    pub retry_after: f64,
}

/// Everything a *radio node* needs to take part in the mesh — carried by the
/// base as well as by every drone, because both run the same protocol on the
/// same systems (see [`crate::antenna::Antennas`]).
#[derive(Bundle)]
pub struct RadioBundle {
    pub uuid: DroneUuid,
    pub clock: DroneClock,
    pub links: LinkSet,
    pub sent: SentHeaders,
    pub ranging: RangingResults,
    pub mesh_table: MeshTable,
    pub tracked_peers: TrackedPeers,
    pub pairing: Pairing,
}

impl RadioBundle {
    pub fn random() -> Self {
        Self {
            uuid: DroneUuid::random(),
            clock: DroneClock::random_start(),
            links: LinkSet::default(),
            sent: SentHeaders::default(),
            ranging: RangingResults::default(),
            mesh_table: MeshTable::default(),
            tracked_peers: TrackedPeers::default(),
            pairing: Pairing::default(),
        }
    }
}

/// A drone's radio, plus the ring slot that decides which two peers it aims
/// at. The base has no ring slot — it covers the whole formation — which is
/// exactly why [`RingIndex`] is here and not in [`RadioBundle`].
#[derive(Bundle)]
pub struct NetworkingBundle {
    pub radio: RadioBundle,
    pub ring_index: RingIndex,
}

impl NetworkingBundle {
    pub fn random(ring_index: usize) -> Self {
        Self { radio: RadioBundle::random(), ring_index: RingIndex(ring_index) }
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

// ─── Reconnection handshake ─────────────────────────────────────────────────────

/// The three message kinds (phases) of the reconnection handshake, each
/// carrying the same `request_id`. See the module docs.
#[derive(Clone, Debug)]
pub enum ReconnectKind {
    /// Requester → target: "reconnect to me". Floods until it reaches
    /// `target`, which accepts (stops + floods `Accept`) or ignores it.
    Request,
    /// Target → requester: "accepted, I've stopped". Floods back.
    Accept,
    /// Requester → target: requester's base-relative position; receiving it
    /// starts the pairing. Floods forward. `payload` carries the position.
    Position { payload: Vec3 },
}

/// Phase discriminant for dedup keys — `(request_id, phase)`. Distinct from
/// [`ReconnectKind`] because the dedup key must ignore the `Position`
/// payload.
pub const PHASE_REQUEST: u8 = 0;
pub const PHASE_ACCEPT: u8 = 1;
pub const PHASE_POSITION: u8 = 2;

impl ReconnectKind {
    pub fn phase(&self) -> u8 {
        match self {
            ReconnectKind::Request => PHASE_REQUEST,
            ReconnectKind::Accept => PHASE_ACCEPT,
            ReconnectKind::Position { .. } => PHASE_POSITION,
        }
    }
}

/// One reconnection message in flight across the mesh flood. Addressed by
/// UUID (`requester`/`target`), because it hops through arbitrary repeater
/// drones that only know peers by UUID. Each hop is one RF transmission along
/// one live antenna link.
#[derive(Clone, Debug)]
pub struct ReconnectMsg {
    /// Unique per originating request. Dedup keys off `(request_id, phase)` —
    /// see [`Pairing::seen`].
    pub request_id: String,
    pub kind: ReconnectKind,
    /// UUID of the drone that started the handshake.
    pub requester: String,
    /// UUID of the drone being reconnected to.
    pub target: String,
    /// Entity this copy is delivered to this hop. The flood re-addresses a
    /// fresh copy to each of a repeater's live-link peers.
    pub to: Entity,
    /// Entity that transmitted this copy — used for split-horizon, so a
    /// repeater never bounces a message straight back the way it came.
    pub from: Entity,
}

/// Priority bus for reconnection messages. Separate from [`Mailbox`] and
/// drained-then-refilled every frame with zero throttle: a drone forwards the
/// instant it receives, it does not wait `HEADER_INTERVAL_SECS`. Because a
/// forward this frame lands in next frame's bus, propagation is naturally one
/// hop per frame — no whole-mesh teleport in a single tick.
#[derive(Resource, Default)]
pub struct ReconnectBus(pub Vec<ReconnectMsg>);

/// Queue of reconnection attempts to kick off (requester_entity, target_uuid).
/// A UI action, a `seeking` timeout, etc. pushes here; `process_reconnect`
/// turns each into an initial `Request` flood. Kept as its own resource so
/// *starting* a handshake doesn't need mutable access to the whole flood.
#[derive(Resource, Default)]
pub struct ReconnectRequests(pub Vec<(Entity, String)>);

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
///
/// This runs over every radio node — the base included, since it carries
/// [`Antennas`] just like a drone does. A base has no airframe, hence no
/// [`DroneKinematics`]: its heading is 0 (its antenna azimuths are already
/// world-frame) and it reports a zero flight direction, which is exactly true
/// of something bolted to the ground.
#[allow(clippy::type_complexity)] // Bevy queries describe the component access contract.
pub fn detect_links_and_send_headers(
    mut mailbox: ResMut<Mailbox>,
    mut nodes: Query<(
        Entity,
        &GlobalTransform,
        &Antennas,
        Option<&DroneKinematics>,
        &DroneClock,
        &DroneUuid,
        &mut LinkSet,
        &mut SentHeaders,
        &MeshTable,
    )>,
    positions: Query<
        (Entity, &GlobalTransform, &Antennas, Option<&DroneKinematics>),
        With<Antennas>,
    >,
    uuids: Query<&DroneUuid>,
    bases: Query<&Base>,
) {
    // `connected_antenna` is this drone's position vector relative to base —
    // not the antenna's own pointing direction.
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);

    for (self_entity, self_gt, antennas, kin, clock, uuid, mut links, mut sent, table) in
        &mut nodes
    {
        let (heading_deg, velocity) =
            kin.map(|k| (k.heading_deg, k.velocity)).unwrap_or((0.0, Vec3::ZERO));
        let self_pos = self_gt.translation();
        let vector_from_base = self_pos - base_pos;

        // All peers currently detected (regardless of resend cadence) — this
        // is what `connections` means for our own upserted row on the peer.
        let detected: Vec<(Entity, usize)> = positions
            .iter()
            .filter_map(|(peer_entity, peer_gt, peer_antennas, peer_kin)| {
                if peer_entity == self_entity {
                    return None;
                }
                let peer_pos = peer_gt.translation();
                let distance_km = (peer_pos - self_pos).length();
                let local_antenna = antennas.0.iter().enumerate().find_map(|(index, antenna)| {
                    let theta = antenna.off_boresight_deg(heading_deg, self_pos, peer_pos);
                    (antenna.rssi_dbm(theta, 0.0, distance_km) >= antenna.sensitivity_dbm)
                        .then_some(index)
                });
                let peer_heading = peer_kin.map(|kin| kin.heading_deg).unwrap_or(0.0);
                let peer_detects_us = peer_antennas.0.iter().any(|antenna| {
                    let theta = antenna.off_boresight_deg(peer_heading, peer_pos, self_pos);
                    antenna.rssi_dbm(theta, 0.0, distance_km) >= antenna.sensitivity_dbm
                });
                local_antenna
                    .filter(|_| peer_detects_us)
                    .map(|antenna_index| (peer_entity, antenna_index))
            })
            .collect();
        let origin_connections: Vec<String> =
            detected
                .iter()
                .filter_map(|(e, _)| uuids.get(*e).ok().map(|u| u.0.clone()))
                .collect();
        let body: Vec<MeshRow> = table.0.values().cloned().collect();

        let mut detected_now: std::collections::HashMap<Entity, f64> =
            std::collections::HashMap::new();
        let mut antenna_for_peer = std::collections::HashMap::new();

        for &(peer_entity, antenna_index) in &detected {
            antenna_for_peer.insert(peer_entity, antenna_index);
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
                flight_direction: velocity * FLIGHT_LOOKAHEAD_SECS,
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
        links.antenna_for_peer = antenna_for_peer;
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

/// Drive the priority reconnection flood one hop per frame.
///
/// Runs every frame, ungated by `HEADER_INTERVAL_SECS` (priority traffic).
/// Drains [`ReconnectBus`], processes each delivered message at its addressed
/// drone, and refills the bus with next-hop forwards — so a message advances
/// exactly one live-link hop per frame. Also drains [`ReconnectRequests`] to
/// originate new `Request` floods. See the module docs for the full state
/// machine; this is a direct transcription of it.
pub fn process_reconnect(
    mut bus: ResMut<ReconnectBus>,
    mut requests: ResMut<ReconnectRequests>,
    bases: Query<&Base>,
    mut drones: Query<(&DroneUuid, &GlobalTransform, &LinkSet, &DroneClock, &mut Pairing)>,
) {
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);
    let incoming = std::mem::take(&mut bus.0);
    let mut outgoing: Vec<ReconnectMsg> = Vec::new();

    // ── Originate new requests ──────────────────────────────────────────────
    for (requester_entity, target_uuid) in std::mem::take(&mut requests.0) {
        let Ok((uuid, _gt, links, clock, mut pairing)) = drones.get_mut(requester_entity) else {
            continue;
        };
        // One handshake at a time — ignore new attempts while mid-flight.
        if !matches!(pairing.state, PairingState::Idle) {
            continue;
        }
        let request_id = new_request_id();
        pairing.seen.insert((request_id.clone(), PHASE_REQUEST));
        pairing.state = PairingState::AwaitingAccept {
            request_id: request_id.clone(),
            target: target_uuid.clone(),
        };
        pairing.state_since = clock.now;
        // Flood the Request out every live link.
        for &peer in links.connected.keys() {
            outgoing.push(ReconnectMsg {
                request_id: request_id.clone(),
                kind: ReconnectKind::Request,
                requester: uuid.0.clone(),
                target: target_uuid.clone(),
                to: peer,
                from: requester_entity,
            });
        }
    }

    // ── Process one hop of in-flight messages ───────────────────────────────
    for msg in incoming {
        let Ok((uuid, gt, links, clock, mut pairing)) = drones.get_mut(msg.to) else { continue };

        // Dedup on (id, phase): already handled → drop entirely.
        let key = (msg.request_id.clone(), msg.kind.phase());
        if pairing.seen.contains(&key) {
            continue;
        }
        pairing.seen.insert(key);

        let me = &uuid.0;
        let addressed = match msg.kind {
            ReconnectKind::Request | ReconnectKind::Position { .. } => *me == msg.target,
            ReconnectKind::Accept => *me == msg.requester,
        };

        if addressed {
            match &msg.kind {
                ReconnectKind::Request => {
                    // Accept only if not already busy with another handshake.
                    if matches!(pairing.state, PairingState::Idle) {
                        pairing.frozen = true; // "stop": hold antenna slew.
                        pairing.state = PairingState::AcceptedAwaitingPosition {
                            request_id: msg.request_id.clone(),
                            requester: msg.requester.clone(),
                        };
                        pairing.state_since = clock.now;
                        // Originate the Accept flood back out every live link.
                        for &peer in links.connected.keys() {
                            outgoing.push(ReconnectMsg {
                                request_id: msg.request_id.clone(),
                                kind: ReconnectKind::Accept,
                                requester: msg.requester.clone(),
                                target: msg.target.clone(),
                                to: peer,
                                from: msg.to,
                            });
                        }
                    }
                    // else: just continue — no reply.
                }
                ReconnectKind::Accept => {
                    // Requester: stop & send position only if still waiting on
                    // *this* request; otherwise it already committed elsewhere
                    // — just continue (drop).
                    let waiting_on_this = matches!(
                        &pairing.state,
                        PairingState::AwaitingAccept { request_id, .. }
                            if *request_id == msg.request_id
                    );
                    if waiting_on_this {
                        pairing.frozen = true; // "stop".
                        pairing.state = PairingState::Paired {
                            request_id: msg.request_id.clone(),
                            peer: msg.target.clone(),
                        };
                        pairing.state_since = clock.now;
                        let pos_rel_base = gt.translation() - base_pos;
                        for &peer in links.connected.keys() {
                            outgoing.push(ReconnectMsg {
                                request_id: msg.request_id.clone(),
                                kind: ReconnectKind::Position { payload: pos_rel_base },
                                requester: msg.requester.clone(),
                                target: msg.target.clone(),
                                to: peer,
                                from: msg.to,
                            });
                        }
                    }
                }
                ReconnectKind::Position { payload } => {
                    // Target: the pairing begins — record requester's position.
                    pairing.paired_peer_pos = Some(*payload);
                    pairing.state = PairingState::Paired {
                        request_id: msg.request_id.clone(),
                        peer: msg.requester.clone(),
                    };
                    pairing.state_since = clock.now;
                    // Terminal for this message — don't forward past the target.
                }
            }
        } else {
            // Repeater: forward to every live link except where it came from.
            for &peer in links.connected.keys() {
                if peer == msg.from {
                    continue;
                }
                outgoing.push(ReconnectMsg {
                    request_id: msg.request_id.clone(),
                    kind: msg.kind.clone(),
                    requester: msg.requester.clone(),
                    target: msg.target.clone(),
                    to: peer,
                    from: msg.to,
                });
            }
        }
    }

    bus.0 = outgoing;
}

/// Stop any drone that is missing a direct link to one of its two ring
/// neighbors.
///
/// A directional 1°-beam link is lost by *drifting off boresight*, so the
/// worst thing a drone can do when a neighbor goes quiet is keep flying: it
/// widens the geometry the seeking spiral has to search and drags its own
/// remaining links toward their range limit. Holding station freezes the
/// problem in place while `crate::seeking` sweeps the lost slot and
/// `crate::tracking` holds the slots that are still up.
///
/// This zeroes `DroneKinematics::velocity` only — `apply_velocity` still does
/// the integration, and any navigator that runs afterwards (notably
/// `crate::recovery::run_recovery`, which flies back to the last-contact
/// waypoint when a link loss actually partitions the mesh) is free to
/// override the halt.
///
/// Runs after `detect_links_and_send_headers`, so `LinkSet` is this frame's.
pub fn halt_on_link_loss(
    mut drones: Query<(Entity, &RingIndex, &LinkSet, &mut DroneKinematics)>,
    ring_slots: Query<(Entity, &RingIndex), With<Drone>>,
    bases: Query<Entity, With<Base>>,
) {
    let Some(base_entity) = bases.iter().next() else {
        return;
    };
    let mut ring: Vec<(usize, Entity)> = ring_slots.iter().map(|(e, ri)| (ri.0, e)).collect();
    ring.sort_by_key(|(i, _)| *i);
    let n = ring.len();
    for (self_entity, self_ring, links, mut kin) in &mut drones {
        // Flight is permitted only while the drone is directly connected to
        // the ground station. A lost base link takes priority over mesh
        // movement or recovery commands.
        if !links.connected.contains_key(&base_entity) {
            kin.velocity = Vec3::ZERO;
            continue;
        }
        // Below 3 drones "next" and "previous" are the same peer (or
        // nonexistent), so only the base-link rule applies.
        if n < 3 {
            continue;
        }
        let next = ring[(self_ring.0 + 1) % n].1;
        let prev = ring[(self_ring.0 + n - 1) % n].1;
        let linked = |peer: Entity| peer == self_entity || links.connected.contains_key(&peer);
        if !linked(next) || !linked(prev) {
            kin.velocity = Vec3::ZERO;
        }
    }
}

/// Ask a nearby drone for a connection.
///
/// A drone that has a free moment — no handshake in flight, not backed off
/// from a failed one — looks through its mesh table for a drone it is *not*
/// already linked to whose last known position is within
/// [`MAX_NEIGHBOR_SPACING_KM`], and requests the closest one. That queues a
/// `Request` flood in [`ReconnectRequests`] for [`process_reconnect`] to
/// originate on the next hop.
///
/// The distance is measured against the mesh table's `base_pos + row.location`
/// — comms-derived, possibly relayed, possibly stale — not the peer's true
/// transform. That is the same estimate `seeking` aims at, and it is all a
/// real drone would have.
///
/// "Available" is not something the requester can see: a peer's handshake
/// state is its own. The requester only screens for *plausible* — known about,
/// in range, not already connected — and the target arbitrates for real, since
/// [`process_reconnect`] accepts a `Request` only when the target is
/// [`PairingState::Idle`].
///
/// A drone with no live links is skipped: the flood travels over live links,
/// so it would reach nobody and only burn a timeout. Those drones are the ones
/// `seeking` lets search on their own initiative instead.
pub fn request_nearby_connections(
    mut requests: ResMut<ReconnectRequests>,
    bases: Query<&Base>,
    drones: Query<
        (Entity, &GlobalTransform, &DroneClock, &LinkSet, &MeshTable, &Pairing),
        With<Drone>,
    >,
    uuids: Query<&DroneUuid>,
) {
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);

    for (entity, gt, clock, links, table, pairing) in &drones {
        // One handshake at a time, and honor the back-off after a failure.
        if !matches!(pairing.state, PairingState::Idle) || clock.now < pairing.retry_after {
            continue;
        }
        if links.connected.is_empty() {
            continue;
        }
        if links.connected.len() >= TARGET_DIRECT_CONNECTIONS {
            continue;
        }

        let linked: HashSet<&str> = links
            .connected
            .keys()
            .filter_map(|&peer| uuids.get(peer).ok())
            .map(|uuid| uuid.0.as_str())
            .collect();

        let self_pos = gt.translation();
        let mut nearest: Option<(f32, &str)> = None;
        for (peer_uuid, row) in &table.0 {
            if linked.contains(peer_uuid.as_str()) {
                continue; // already connected — nothing to ask for.
            }
            let distance_km = (base_pos + row.location - self_pos).length();
            if distance_km > MAX_NEIGHBOR_SPACING_KM {
                continue;
            }
            if nearest.is_none_or(|(best, _)| distance_km < best) {
                nearest = Some((distance_km, peer_uuid.as_str()));
            }
        }

        if let Some((_, target_uuid)) = nearest {
            requests.0.push((entity, target_uuid.to_string()));
        }
    }
}

/// Give up on a handshake nobody answered.
///
/// `AwaitingAccept` and `AcceptedAwaitingPosition` both wait on a reply that
/// may never come — the peer moved out of range, the flood never reached it,
/// it committed to someone else. Left alone, the requester can never ask again
/// and the target stays frozen with its antennas held, so both fall back to
/// `Idle` after [`RECONNECT_TIMEOUT_SECS`] on the waiting drone's own clock,
/// and back off for [`RECONNECT_RETRY_SECS`] before trying again.
///
/// `Paired` is deliberately not expired: it is the handshake's success state,
/// not a wait.
pub fn expire_stale_handshakes(mut nodes: Query<(&DroneClock, &mut Pairing)>) {
    for (clock, mut pairing) in &mut nodes {
        let waiting = matches!(
            pairing.state,
            PairingState::AwaitingAccept { .. } | PairingState::AcceptedAwaitingPosition { .. }
        );
        if !waiting || clock.now - pairing.state_since < RECONNECT_TIMEOUT_SECS {
            continue;
        }
        pairing.state = PairingState::Idle;
        pairing.frozen = false; // release the slew freeze taken for the handshake.
        pairing.state_since = clock.now;
        pairing.retry_after = clock.now + RECONNECT_RETRY_SECS;
    }
}

// ─── Helpers ───────────────────────────────────────────────────────────────────

/// A fresh random request id (v4-format UUID string) for a reconnection flood.
pub fn new_request_id() -> String {
    let seed = fresh_seed(0x1234_5678_9abc_def1);
    format_uuid_v4(splitmix64(seed), splitmix64(seed ^ 0xa5a5_5a5a_c3c3_3c3c))
}

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
