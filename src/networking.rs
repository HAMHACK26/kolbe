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
use std::sync::Arc;
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

/// Angular width of one ground-station antenna sector. Five antennas cover
/// the full circle, so this is what `base::spawn_base` lays them out on.
pub const BASE_SECTOR_DEG: f32 = 72.0;
/// Resting elevation of a sector antenna with nobody in it — matches the
/// initial aim `base::spawn_base` builds them with.
pub const BASE_SECTOR_ELEVATION_DEG: f32 = 5.0;

/// Signed difference `a - b`, folded into `[-180, 180)`.
pub fn shortest_angle_deg(a: f32, b: f32) -> f32 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

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

/// Marks a freshly spawned base that needs its initial reciprocal radio links
/// seeded before the first scheduled header exchange.
#[derive(Component)]
pub struct BootstrapBaseLinks;

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

/// Target-area geometry as vectors from the ground station, one per corner of
/// the mission boundary. The base is the source of truth; every mesh node
/// relays the most recent copy in its regular 0.1 s header traffic.
///
/// Held behind an `Arc` because it rides in every header packet and those are
/// cloned per link per tick — the geometry itself never changes during a run,
/// so a refcount bump is the whole cost of passing it on.
#[derive(Component, Clone, Debug, Default)]
pub struct TargetAreaVectors {
    pub corners_from_base: Option<Arc<[Vec3]>>,
    pub received_at: f64,
}

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
}

/// Everything a drone needs to take part in the mesh.
#[derive(Bundle)]
pub struct NetworkingBundle {
    pub uuid: DroneUuid,
    pub clock: DroneClock,
    pub links: LinkSet,
    pub sent: SentHeaders,
    pub ranging: RangingResults,
    pub mesh_table: MeshTable,
    pub target_area: TargetAreaVectors,
    pub ring_index: RingIndex,
    pub tracked_peers: TrackedPeers,
    pub pairing: Pairing,
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
            target_area: TargetAreaVectors::default(),
            ring_index: RingIndex(ring_index),
            tracked_peers: TrackedPeers::default(),
            pairing: Pairing::default(),
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
    /// The selected area, expressed only as vectors from the base. Present on
    /// base headers and relayed verbatim by every drone that has received it.
    /// The mission boundary, as vectors from the base. See
    /// [`TargetAreaVectors`].
    pub target_area: Option<Arc<[Vec3]>>,

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
#[allow(clippy::type_complexity)] // Bevy queries describe the component access contract.
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
        &TargetAreaVectors,
    )>,
    positions: Query<(Entity, &GlobalTransform), With<Drone>>,
    uuids: Query<&DroneUuid>,
    bases: Query<(Entity, &Base, &DroneUuid)>,
) {
    // `connected_antenna` is this drone's position vector relative to base —
    // not the antenna's own pointing direction.
    let base_pos = bases.iter().next().map(|(_, b, _)| b.position).unwrap_or(Vec3::ZERO);

    // The ground station is a peer like any other: every drone's antenna #2 is
    // aimed at it by `tracking::maintain_mesh_antennas`, and it carries a
    // `DroneUuid` so it can be named in mesh rows. Including it here is what
    // actually attaches it to the mesh — without it the base is drawn, aimed
    // at, and completely unreachable.
    let base_peers: Vec<(Entity, Vec3)> =
        bases.iter().map(|(entity, base, _)| (entity, base.position)).collect();

    for (self_entity, self_gt, drone, kin, clock, uuid, mut links, mut sent, table, target_area) in &mut drones
    {
        let self_pos = self_gt.translation();
        let vector_from_base = self_pos - base_pos;

        // All peers currently detected (regardless of resend cadence) — this
        // is what `connections` means for our own upserted row on the peer.
        let mut detected: Vec<Entity> = positions
            .iter()
            .filter(|(peer_entity, peer_gt)| {
                *peer_entity != self_entity && {
                    let peer_pos = peer_gt.translation();
                    let distance_km = (peer_pos - self_pos).length();
                    drone.antennas.iter().any(|antenna| {
                        let theta_tx = antenna.off_boresight_deg(kin.heading_deg, self_pos, peer_pos);
                        antenna.rssi_dbm(theta_tx, 0.0, distance_km) >= antenna.sensitivity_dbm
                    })
                }
            })
            .map(|(peer_entity, _)| peer_entity)
            .collect();
        // Antenna slot #2 is reserved for the base. A base is a real peer,
        // not a UI-only reachability marker, so its UUID enters LinkSet and
        // participates in normal header/table routing.
        detected.extend(base_peers.iter().filter_map(|(entity, peer_pos)| {
            let distance_km = (*peer_pos - self_pos).length();
            (distance_km <= f32::EPSILON || drone.antennas.iter().any(|antenna| {
                let theta = antenna.off_boresight_deg(kin.heading_deg, self_pos, *peer_pos);
                antenna.rssi_dbm(theta, 0.0, distance_km) >= antenna.sensitivity_dbm
            })).then_some(*entity)
        }));
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
                    target_area: target_area.corners_from_base.clone(),
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

/// The fixed base runs the same link/header protocol as a drone, with five
/// independently aimed sectors. It is intentionally separate from the drone
/// detector because it has no `Drone` component and must never enter flight.
///
/// Detection only. Where those five antennas *point* is decided upstream by
/// [`crate::tracking::maintain_base_antennas`] and
/// [`crate::seeking::seek_lost_base_links`], exactly as a drone's aim is —
/// this system reads the resulting boresight and evaluates link budget
/// against it.
pub fn detect_base_links_and_send_headers(
    mut mailbox: ResMut<Mailbox>,
    mut bases: Query<(
        Entity, &Base, &DroneClock, &DroneUuid, &mut LinkSet, &mut SentHeaders, &MeshTable,
        &TargetAreaVectors,
    )>,
    drones: Query<(Entity, &GlobalTransform, &DroneUuid), With<Drone>>,
) {
    for (base_entity, base, clock, uuid, mut links, mut sent, table, target_area) in &mut bases {
        let base_pos = base.position;
        let peers: Vec<(Entity, Vec3, String)> = drones
            .iter()
            .map(|(entity, transform, peer_uuid)| (entity, transform.translation(), peer_uuid.0.clone()))
            .collect();

        let mut detected_now = HashMap::new();
        for (peer_entity, peer_pos, _peer_uuid) in &peers {
            let distance_km = (*peer_pos - base_pos).length();
            // Every airframe begins exactly on the ground station. At zero
            // range there is no meaningful bearing for a directional antenna,
            // but the physical link is unambiguously present; preserve that
            // reciprocal launch connection until the normal antenna check can
            // take over once the drone has moved away.
            let linked = distance_km <= f32::EPSILON || base.antennas.iter().any(|antenna| {
                let theta = antenna.off_boresight_deg(0.0, base_pos, *peer_pos);
                antenna.rssi_dbm(theta, 0.0, distance_km) >= antenna.sensitivity_dbm
            });
            if !linked {
                continue;
            }
            let last_sent = links.connected.get(peer_entity).copied();
            let due = last_sent.is_none_or(|t| clock.now - t >= HEADER_INTERVAL_SECS);
            detected_now.insert(*peer_entity, last_sent.unwrap_or(clock.now));
            if !due { continue; }
            detected_now.insert(*peer_entity, clock.now);
            let header = NetworkHeader {
                id: uuid.0.clone(),
                connected_antenna: Vec3::ZERO,
                flight_direction: Vec3::ZERO,
                time_received: clock.now,
            };
            mailbox.0.push((*peer_entity, Packet {
                kind: PacketKind::Header,
                origin: base_entity,
                responder: *peer_entity,
                origin_pos: base_pos,
                header: header.clone(),
                body: table.0.values().cloned().collect(),
                origin_connections: peers.iter().map(|(_, _, id)| id.clone()).collect(),
                target_area: target_area.corners_from_base.clone(),
                responder_pos: Vec3::ZERO,
                responder_delay: 0.0,
                arrival_time: 0.0,
            }));
            sent.0.push(header);
        }
        links.connected = detected_now;
    }
}

/// Keep only antenna links that both endpoints independently detected this
/// frame. The drone and base detectors each evaluate their own antenna
/// boresight/RSSI, so their intersection is the physical two-antenna link.
///
/// This runs after both detectors and before packet routing: a one-way beam
/// can never be displayed, used for navigation, or carry a header.
pub fn retain_mutual_links(
    mut sets: ParamSet<(Query<(Entity, &LinkSet)>, Query<(Entity, &mut LinkSet)>)>,
) {
    let directed: std::collections::HashSet<(Entity, Entity)> = sets
        .p0()
        .iter()
        .flat_map(|(entity, links)| links.connected.keys().map(move |peer| (entity, *peer)))
        .collect();

    for (entity, mut links) in &mut sets.p1() {
        links.connected.retain(|peer, _| directed.contains(&(*peer, entity)));
    }
}

/// Give the base and every launch drone reciprocal live links before their
/// first flight tick. Its five antennas are directional sectors, not a
/// five-drone capacity cap; co-located launch drones share a sector.
pub fn bootstrap_base_links(
    mut commands: Commands,
    mut bases: Query<
        (Entity, &DroneUuid, &mut LinkSet, &mut MeshTable, &Base),
        (With<BootstrapBaseLinks>, Without<Drone>),
    >,
    mut drones: Query<
        (Entity, &DroneUuid, &mut LinkSet, &mut MeshTable),
        (With<Drone>, Without<BootstrapBaseLinks>),
    >,
) {
    for (base_entity, base_uuid, mut base_links, mut base_table, base) in &mut bases {
        for (drone_entity, drone_uuid, mut drone_links, mut drone_table) in &mut drones {
            base_links.connected.insert(drone_entity, 0.0);
            drone_links.connected.insert(base_entity, 0.0);
            base_table.0.insert(drone_uuid.0.clone(), MeshRow {
                id: drone_uuid.0.clone(),
                timestamp: 0.0,
                location: Vec3::ZERO,
                neighbour_distance: 0,
                connections: vec![base_uuid.0.clone()],
            });
            drone_table.0.insert(base_uuid.0.clone(), MeshRow {
                id: base_uuid.0.clone(),
                timestamp: 0.0,
                location: Vec3::ZERO,
                neighbour_distance: 0,
                connections: vec![drone_uuid.0.clone()],
            });
        }
        commands.entity(base_entity).remove::<BootstrapBaseLinks>();
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
        &mut TargetAreaVectors,
    )>,
) {
    let packets = std::mem::take(&mut mailbox.0);
    let mut outgoing: Vec<(Entity, Packet)> = Vec::new();

    for (target, pkt) in packets {
        match pkt.kind {
            PacketKind::Header => {
                // `target` is the responder.
                let Ok((resp_gt, _, resp_uuid, resp_clock, mut resp_table, mut resp_tracked, mut target_area)) =
                    drones.get_mut(target)
                else {
                    continue;
                };
                let responder_pos = resp_gt.translation();

                if let Some(corners) = pkt.target_area.clone() {
                    target_area.corners_from_base = Some(corners);
                    target_area.received_at = resp_clock.now;
                }

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
    mut drones: Query<(&DroneUuid, &GlobalTransform, &LinkSet, &mut Pairing)>,
) {
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);
    let incoming = std::mem::take(&mut bus.0);
    let mut outgoing: Vec<ReconnectMsg> = Vec::new();

    // ── Originate new requests ──────────────────────────────────────────────
    for (requester_entity, target_uuid) in std::mem::take(&mut requests.0) {
        let Ok((uuid, _gt, links, mut pairing)) = drones.get_mut(requester_entity) else {
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
        let Ok((uuid, gt, links, mut pairing)) = drones.get_mut(msg.to) else { continue };

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

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::system::RunSystemOnce;

    use crate::drone::{make_antenna, DroneType};
    use crate::tracking::maintain_mesh_antennas;

    /// A full drone with everything the networking + tracking systems need.
    fn spawn_drone(world: &mut World, pos: Vec3, ring: usize) -> Entity {
        world
            .spawn((
                Transform::from_translation(pos),
                GlobalTransform::from(Transform::from_translation(pos)),
                Drone {
                    id: format!("d{ring}"),
                    drone_type: DroneType::Node,
                    antennas: vec![
                        make_antenna(0.0, 0.0, 0),
                        make_antenna(0.0, 0.0, 1),
                        make_antenna(0.0, 0.0, 2),
                    ],
                },
                DroneKinematics::default(),
                NetworkingBundle::random(ring),
            ))
            .id()
    }

    fn uuid_of(world: &World, drone: Entity) -> String {
        world.get::<DroneUuid>(drone).unwrap().0.clone()
    }

    /// Seed `owner`'s mesh table with a base-relative row for `peer`, as if
    /// from a prior mission briefing rather than live telemetry — the same
    /// comms-derived channel `tracking::maintain_mesh_antennas` reads to aim
    /// antennas, so it can achieve an initial lock without ever reading
    /// `peer`'s live `Transform` directly (see `tracking.rs` docs). Tests
    /// call this only for the pairs that are meant to already "know" each
    /// other going in — never for a pair whose relay-learning is itself
    /// under test.
    fn seed_mesh_row(world: &mut World, base_pos: Vec3, owner: Entity, peer: Entity, peer_pos: Vec3) {
        let peer_uuid = uuid_of(world, peer);
        world.get_mut::<MeshTable>(owner).unwrap().0.insert(
            peer_uuid.clone(),
            MeshRow {
                id: peer_uuid,
                timestamp: 0.0,
                location: peer_pos - base_pos,
                neighbour_distance: 0,
                connections: vec![],
            },
        );
    }

    /// maintain (aim antennas) → detect (form links). Two drones a km apart,
    /// facing, should both end up with the other in their `LinkSet`.
    fn aim_and_detect(world: &mut World) {
        world.run_system_once(maintain_mesh_antennas).unwrap();
        world.run_system_once(detect_links_and_send_headers).unwrap();
    }

    #[test]
    fn link_forms_between_facing_in_range_drones() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base { id: "base".into(), position: base_pos, antennas: vec![] });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(1.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let b = spawn_drone(&mut world, b_pos, 1);
        seed_mesh_row(&mut world, base_pos, a, b, b_pos);
        seed_mesh_row(&mut world, base_pos, b, a, a_pos);

        aim_and_detect(&mut world);

        assert!(
            world.get::<LinkSet>(a).unwrap().connected.contains_key(&b),
            "A should have detected B"
        );
        assert!(
            world.get::<LinkSet>(b).unwrap().connected.contains_key(&a),
            "B should have detected A"
        );
        // A header was queued for the peer.
        assert!(!world.resource::<Mailbox>().0.is_empty(), "headers should be queued");
    }

    #[test]
    fn out_of_range_drones_do_not_link() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        world.spawn(Base { id: "base".into(), position: Vec3::new(0.0, 0.0, -5.0), antennas: vec![] });
        let a = spawn_drone(&mut world, Vec3::ZERO, 0);
        // 100 km apart — far past the ~3.5 km link budget.
        let b = spawn_drone(&mut world, Vec3::new(100.0, 0.0, 0.0), 1);

        aim_and_detect(&mut world);

        assert!(world.get::<LinkSet>(a).unwrap().connected.is_empty());
        assert!(world.get::<LinkSet>(b).unwrap().connected.is_empty());
    }

    #[test]
    fn mesh_table_learns_direct_peer_at_distance_zero() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base { id: "base".into(), position: base_pos, antennas: vec![] });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(1.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let b = spawn_drone(&mut world, b_pos, 1);
        let a_uuid = uuid_of(&world, a);
        seed_mesh_row(&mut world, base_pos, a, b, b_pos);
        seed_mesh_row(&mut world, base_pos, b, a, a_pos);

        aim_and_detect(&mut world);
        world.run_system_once(route_packets).unwrap();

        let b_table = world.get::<MeshTable>(b).unwrap();
        let row = b_table.0.get(&a_uuid).expect("B should have learned A");
        assert_eq!(row.neighbour_distance, 0, "a direct peer is distance 0");
    }

    #[test]
    fn ranging_recovers_the_true_distance() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base { id: "base".into(), position: base_pos, antennas: vec![] });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(2.0, 0.0, 0.0); // 2 km apart
        let a = spawn_drone(&mut world, a_pos, 0);
        let _b = spawn_drone(&mut world, b_pos, 1);
        seed_mesh_row(&mut world, base_pos, a, _b, b_pos);
        seed_mesh_row(&mut world, base_pos, _b, a, a_pos);

        aim_and_detect(&mut world);
        // First pass: headers → echoes. Second pass: echoes → ranging.
        world.run_system_once(route_packets).unwrap();
        world.run_system_once(route_packets).unwrap();

        let ranging = world.get::<RangingResults>(a).unwrap();
        let (_, sv) = ranging.0.last().expect("A should have a ranging result");
        assert!((sv.length - 2.0).abs() < 0.01, "measured distance ~2 km, got {}", sv.length);
    }

    /// One full round: advance every drone's clock past the resend interval,
    /// re-aim, detect links, route one hop of gossip.
    fn step(world: &mut World, drones: &[Entity]) {
        for &e in drones {
            world.get_mut::<DroneClock>(e).unwrap().now += 0.2;
        }
        world.run_system_once(maintain_mesh_antennas).unwrap();
        world.run_system_once(detect_links_and_send_headers).unwrap();
        world.run_system_once(route_packets).unwrap();
    }

    /// A—B—C in a line: A↔B and B↔C are in range, but A↔C (5 km) is not. After
    /// enough gossip rounds, C learns about A *by relay through B*, recorded at
    /// neighbour_distance 1 (one hop past its direct peer B).
    #[test]
    fn distant_peer_learned_by_relay_at_distance_one() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -8.0);
        world.spawn(Base { id: "base".into(), position: base_pos, antennas: vec![] });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(2.5, 0.0, 0.0);
        let c_pos = Vec3::new(5.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let b = spawn_drone(&mut world, b_pos, 1);
        let c = spawn_drone(&mut world, c_pos, 2);
        let a_uuid = uuid_of(&world, a);

        // Seed only the pairs meant to link directly (A-B, B-C) so their
        // antennas achieve an initial lock — deliberately NOT A-C, since C
        // learning of A is exactly what this test verifies happens by relay,
        // not by already knowing.
        seed_mesh_row(&mut world, base_pos, a, b, b_pos);
        seed_mesh_row(&mut world, base_pos, b, a, a_pos);
        seed_mesh_row(&mut world, base_pos, b, c, c_pos);
        seed_mesh_row(&mut world, base_pos, c, b, b_pos);

        // A few rounds so the gossip has time to walk A's row out to C via B.
        for _ in 0..4 {
            step(&mut world, &[a, b, c]);
        }

        // C never links A directly (out of range)…
        assert!(
            !world.get::<LinkSet>(c).unwrap().connected.contains_key(&a),
            "A↔C should be out of range — no direct link"
        );
        // …but still learns of A, relayed, at one hop past its direct peer.
        let c_table = world.get::<MeshTable>(c).unwrap();
        let row = c_table.0.get(&a_uuid).expect("C should have learned A by relay");
        assert_eq!(row.neighbour_distance, 1, "relayed peer is one hop past the direct link");
    }

    /// A header is not resent until `HEADER_INTERVAL_SECS` has passed on the
    /// sending drone's *own* clock — resend cadence is per-drone, not shared.
    #[test]
    fn header_not_resent_within_interval_on_own_clock() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base { id: "base".into(), position: base_pos, antennas: vec![] });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(1.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let _b = spawn_drone(&mut world, b_pos, 1);
        seed_mesh_row(&mut world, base_pos, a, _b, b_pos);
        seed_mesh_row(&mut world, base_pos, _b, a, a_pos);

        aim_and_detect(&mut world);
        assert!(!world.resource::<Mailbox>().0.is_empty(), "first detect sends headers");

        // Drain, then detect again with the clock unchanged — nothing due yet.
        world.resource_mut::<Mailbox>().0.clear();
        world.run_system_once(detect_links_and_send_headers).unwrap();
        assert!(
            world.resource::<Mailbox>().0.is_empty(),
            "no resend before the interval elapses on the own clock"
        );

        // Advance every clock past the interval → headers resend.
        world.get_mut::<DroneClock>(a).unwrap().now += HEADER_INTERVAL_SECS + 0.01;
        world.get_mut::<DroneClock>(_b).unwrap().now += HEADER_INTERVAL_SECS + 0.01;
        world.run_system_once(detect_links_and_send_headers).unwrap();
        assert!(!world.resource::<Mailbox>().0.is_empty(), "resend after interval elapses");
    }

    /// A link is dropped once the peer moves out of range — `LinkSet` reflects
    /// only currently-reachable peers, not stale ones.
    #[test]
    fn link_drops_when_peer_leaves_range() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base { id: "base".into(), position: base_pos, antennas: vec![] });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(1.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let b = spawn_drone(&mut world, b_pos, 1);
        seed_mesh_row(&mut world, base_pos, a, b, b_pos);
        seed_mesh_row(&mut world, base_pos, b, a, a_pos);

        aim_and_detect(&mut world);
        assert!(world.get::<LinkSet>(a).unwrap().connected.contains_key(&b), "linked in range");

        // Teleport B far out of range (update both transforms).
        let far = Vec3::new(100.0, 0.0, 0.0);
        *world.get_mut::<Transform>(b).unwrap() = Transform::from_translation(far);
        *world.get_mut::<GlobalTransform>(b).unwrap() =
            GlobalTransform::from(Transform::from_translation(far));

        aim_and_detect(&mut world);
        assert!(
            world.get::<LinkSet>(a).unwrap().connected.is_empty(),
            "link should drop once B is out of range"
        );
    }
}
    #[test]
    fn base_bootstrap_queries_are_disjoint() {
        // `App::update` validates system parameter access even without any
        // entities. This prevents a B0001 startup panic from regressing.
        let mut app = App::new();
        app.add_systems(Update, bootstrap_base_links);
        app.update();
    }

    #[test]
    fn one_way_antenna_detection_is_not_a_connection() {
        use bevy::ecs::system::RunSystemOnce;

        let mut world = World::new();
        let a = world.spawn(LinkSet::default()).id();
        let b = world.spawn(LinkSet::default()).id();
        world.get_mut::<LinkSet>(a).unwrap().connected.insert(b, 0.0);

        world.run_system_once(retain_mutual_links).unwrap();
        assert!(world.get::<LinkSet>(a).unwrap().connected.is_empty());

        world.get_mut::<LinkSet>(a).unwrap().connected.insert(b, 0.0);
        world.get_mut::<LinkSet>(b).unwrap().connected.insert(a, 0.0);
        world.run_system_once(retain_mutual_links).unwrap();
        assert!(world.get::<LinkSet>(a).unwrap().connected.contains_key(&b));
        assert!(world.get::<LinkSet>(b).unwrap().connected.contains_key(&a));
    }
