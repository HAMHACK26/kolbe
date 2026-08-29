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
//! `connected antenna` = the exchange originator's local direction toward its
//! direct neighbour. When the header returns as an echo, that originator is
//! the receiver and combines the direction with gravity and its base anchor.
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
//! row per drone it knows about, itself excluded (each exchange originator
//! learns its responder directly after the ranging echo returns):
//!
//! | id   | timestamp | location | neighbour distance | connections |
//! | ---- | --------- | -------- | ------------------- | ----------- |
//! | UUID | datetime  | vector   | int                 | list[UUID]  |
//!
//! `neighbour distance` is hop count (0 = a direct connection). `connections`
//! is that row's drone's own *direct* peers.
//!
//! On receipt (`route_packets`):
//! - A completed echo upserts its responder as a distance-0 row, stamped on
//!   the originator's own clock and projected through its base-frame anchor.
//! - Every other row in the body is a candidate at `row.neighbour_distance +
//!   1`. A shorter path wins immediately; another valid path may replace it
//!   after `HEADER_INTERVAL_SECS * (n + 1)` without an update.
//! - Relayed row age is measured on the sender's clock and rebased onto the
//!   receiver's clock. No comparison crosses independent clock domains.
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

use crate::base::Base;
use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::spherical::SphericalVec;
use crate::tracking::TrackedPeers;

/// Numeric tolerance used when validating a closed yaw loop.
pub const LOOP_CLOSURE_TOLERANCE_RAD: f32 = 1.0e-5;

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
        DroneUuid(format_uuid_v4(
            splitmix64(seed),
            splitmix64(seed ^ 0xD1B54A32D192ED03),
        ))
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

/// The receiver's continuously maintained relationship to the shared base
/// frame. Protocol projection reads this component instead of ECS world truth.
#[derive(Component, Clone, Debug)]
pub struct BaseFrameReference {
    pub own_location_base: Vec3,
    pub local_direction_to_base: Vec3,
    pub gravity_up_local: Vec3,
}

/// One measured, directed orientation edge used by yaw-only loop closure.
/// All timestamps are in the owning drone's local clock domain.
#[derive(Clone, Debug, PartialEq)]
pub struct YawObservation {
    pub from: String,
    pub to: String,
    pub measured_yaw_rad: f32,
    pub corrected_yaw_rad: f32,
    pub vector_length: f32,
    pub neighbour_distance: u32,
    pub timestamp: f64,
    pub generation: u64,
}

/// Network representation of a directed yaw observation. Corrections are
/// deliberately absent: every receiver computes those from its own graph.
#[derive(Clone, Debug)]
pub struct YawObservationMessage {
    pub from: String,
    pub to: String,
    pub measured_yaw_rad: f32,
    pub vector_length: f32,
    pub neighbour_distance: u32,
    pub timestamp: f64,
    pub generation: u64,
}

impl From<&YawObservation> for YawObservationMessage {
    fn from(edge: &YawObservation) -> Self {
        Self {
            from: edge.from.clone(),
            to: edge.to.clone(),
            measured_yaw_rad: edge.measured_yaw_rad,
            vector_length: edge.vector_length,
            neighbour_distance: edge.neighbour_distance,
            timestamp: edge.timestamp,
            generation: edge.generation,
        }
    }
}

/// The correction applied to an edge by [`close_yaw_loop`].
#[derive(Clone, Debug, PartialEq)]
pub struct YawCorrection {
    pub from: String,
    pub to: String,
    pub delta_rad: f32,
}

/// Directed yaw observations known to this drone.
#[derive(Component, Default)]
pub struct YawObservationTable {
    pub edges: HashMap<(String, String), YawObservation>,
    next_generation: u64,
}

impl YawObservationTable {
    pub fn corrected_yaw(&self, from: &str, to: &str) -> Option<f32> {
        self.edges
            .get(&(from.to_owned(), to.to_owned()))
            .map(|edge| edge.corrected_yaw_rad)
    }

    fn next_generation(&mut self) -> u64 {
        self.next_generation = self.next_generation.wrapping_add(1);
        self.next_generation
    }
}

/// Project a receiver-local antenna observation into the shared base frame.
///
/// Gravity fixes pitch. The horizontal direction to the base fixes the one
/// remaining degree of freedom (yaw). `own_location_base` is the receiver's
/// base-frame position and `local_base_direction` is its locally observed
/// direction toward the base.
pub fn project_direct_observation_to_base(
    local_antenna_direction: Vec3,
    gravity_up_local: Vec3,
    local_base_direction: Vec3,
    own_location_base: Vec3,
    distance: f32,
) -> Option<Vec3> {
    if !local_antenna_direction.is_finite()
        || !gravity_up_local.is_finite()
        || !local_base_direction.is_finite()
        || !own_location_base.is_finite()
        || !distance.is_finite()
        || distance < 0.0
    {
        return None;
    }
    let up = gravity_up_local.try_normalize()?;
    let local_forward =
        (local_base_direction - up * local_base_direction.dot(up)).try_normalize()?;
    let local_right = up.cross(local_forward).try_normalize()?;

    let base_up = Vec3::Y;
    let base_forward =
        (-own_location_base + base_up * own_location_base.dot(base_up)).try_normalize()?;
    let base_right = base_up.cross(base_forward).try_normalize()?;
    let local = local_antenna_direction.try_normalize()?;
    let direction_base = base_forward * local.dot(local_forward)
        + base_right * local.dot(local_right)
        + base_up * local.dot(up);
    Some(own_location_base + direction_base.normalize_or_zero() * distance)
}

fn row_is_valid(row: &MeshRow) -> bool {
    !row.id.is_empty()
        && row.timestamp.is_finite()
        && row.location.is_finite()
        && row.connections.iter().all(|id| !id.is_empty())
}

fn rebase_timestamp(sender_now: f64, incoming_timestamp: f64, receiver_now: f64) -> Option<f64> {
    if !sender_now.is_finite() || !incoming_timestamp.is_finite() || !receiver_now.is_finite() {
        return None;
    }
    let age = sender_now - incoming_timestamp;
    if age < 0.0 || !age.is_finite() {
        return None;
    }
    Some(receiver_now - age)
}

fn relayed_neighbour_distance(distance: u32) -> Option<u32> {
    distance.checked_add(1)
}

/// Whether an incoming row may replace the last valid observation.
/// Shorter paths win immediately. Otherwise the current path gets its
/// documented `u_t * (n + 1)` update window before a valid candidate wins.
pub fn should_replace_row(existing: &MeshRow, candidate: &MeshRow, now: f64) -> bool {
    if !row_is_valid(candidate) || !now.is_finite() {
        return false;
    }
    candidate.neighbour_distance < existing.neighbour_distance
        || now - existing.timestamp
            >= HEADER_INTERVAL_SECS * (existing.neighbour_distance as f64 + 1.0)
}

fn yaw_observation_is_valid(edge: &YawObservation) -> bool {
    !edge.from.is_empty()
        && !edge.to.is_empty()
        && edge.from != edge.to
        && edge.measured_yaw_rad.is_finite()
        && edge.corrected_yaw_rad.is_finite()
        && edge.vector_length.is_finite()
        && edge.vector_length >= 0.0
        && edge.timestamp.is_finite()
}

fn should_replace_yaw(existing: &YawObservation, candidate: &YawObservation, now: f64) -> bool {
    yaw_observation_is_valid(candidate)
        && (candidate.neighbour_distance < existing.neighbour_distance
            || now - existing.timestamp
                >= HEADER_INTERVAL_SECS * (existing.neighbour_distance as f64 + 1.0))
}

pub fn wrap_yaw_rad(yaw: f32) -> f32 {
    (yaw + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

/// Distribute a loop's signed yaw residual by confidence weight. Corrections
/// are calculated completely before being committed, so invalid input cannot
/// partially mutate the edge table. Vector lengths are never modified.
pub fn close_yaw_loop(edges: &mut [YawObservation]) -> Option<Vec<YawCorrection>> {
    if edges.len() < 3 {
        return None;
    }
    for pair in edges.windows(2) {
        if pair[0].to != pair[1].from {
            return None;
        }
    }
    if edges.last()?.to != edges.first()?.from {
        return None;
    }
    if edges.iter().any(|edge| {
        !edge.measured_yaw_rad.is_finite()
            || !edge.vector_length.is_finite()
            || edge.vector_length < 0.0
    }) {
        return None;
    }
    let residual = wrap_yaw_rad(edges.iter().map(|edge| edge.measured_yaw_rad).sum());
    let weight_sum: f32 = edges
        .iter()
        .map(|edge| edge.neighbour_distance as f32 + 1.0)
        .sum();
    let updates: Vec<(f32, f32)> = edges
        .iter()
        .map(|edge| {
            let delta = -residual * (edge.neighbour_distance as f32 + 1.0) / weight_sum;
            (delta, wrap_yaw_rad(edge.measured_yaw_rad + delta))
        })
        .collect();
    let mut corrections = Vec::with_capacity(edges.len());
    for (edge, (delta, corrected)) in edges.iter_mut().zip(updates) {
        edge.corrected_yaw_rad = corrected;
        corrections.push(YawCorrection {
            from: edge.from.clone(),
            to: edge.to.clone(),
            delta_rad: delta,
        });
    }
    debug_assert!(
        wrap_yaw_rad(edges.iter().map(|edge| edge.corrected_yaw_rad).sum()).abs()
            <= LOOP_CLOSURE_TOLERANCE_RAD
    );
    Some(corrections)
}

/// Discover deterministic directed loops from lookup-table `connections`.
/// References to missing rows are ignored, so incomplete gossip cannot form
/// a loop or mutate the table.
pub fn discover_loops(table: &MeshTable) -> Vec<Vec<String>> {
    fn canonical_rotation(path: &[String]) -> Vec<String> {
        let mut candidates = Vec::with_capacity(path.len());
        for offset in 0..path.len() {
            let mut candidate = path.to_vec();
            candidate.rotate_left(offset);
            candidates.push(candidate);
        }
        candidates.into_iter().min().unwrap()
    }

    fn undirected_key(path: &[String]) -> Vec<String> {
        canonical_rotation(path).min(canonical_rotation(
            &path.iter().rev().cloned().collect::<Vec<_>>(),
        ))
    }

    fn visit(
        table: &MeshTable,
        start: &str,
        current: &str,
        path: &mut Vec<String>,
        visited: &mut HashSet<String>,
        cycles: &mut HashMap<Vec<String>, Vec<String>>,
    ) {
        let Some(row) = table.0.get(current).filter(|row| row_is_valid(row)) else {
            return;
        };
        let mut neighbours = row.connections.clone();
        neighbours.sort();
        neighbours.dedup();
        for next in neighbours {
            if !table.0.get(&next).is_some_and(row_is_valid) {
                continue;
            }
            if next == start {
                if path.len() >= 3 {
                    let directed = canonical_rotation(path);
                    cycles.entry(undirected_key(path)).or_insert(directed);
                }
            } else if !visited.contains(&next) {
                visited.insert(next.clone());
                path.push(next.clone());
                visit(table, start, &next, path, visited, cycles);
                path.pop();
                visited.remove(&next);
            }
        }
    }

    let mut starts: Vec<String> = table.0.keys().cloned().collect();
    starts.sort();
    let mut cycles = HashMap::new();
    for start in starts {
        let mut path = vec![start.clone()];
        let mut visited = HashSet::from([start.clone()]);
        visit(table, &start, &start, &mut path, &mut visited, &mut cycles);
    }
    let mut loops: Vec<Vec<String>> = cycles.into_values().collect();
    loops.sort();
    loops
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
    AcceptedAwaitingPosition {
        request_id: String,
        requester: String,
    },
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
    pub yaw_observations: YawObservationTable,
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
            yaw_observations: YawObservationTable::default(),
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
    /// Sender-local unit direction from the sender toward this direct peer.
    pub connected_antenna: Vec3,
    /// Directly observed orientation offset from sender to receiver.
    pub relative_yaw_rad: f32,
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
    pub responder_id: String,
    pub origin_pos: Vec3,
    /// The header itself — only meaningful for `PacketKind::Header`, but kept
    /// (cheaply cloned) on the echo too so ranging can read `time_received`
    /// without a separate field.
    pub header: NetworkHeader,
    /// Sender's mesh body table (its non-self rows) — see module docs.
    /// Only sent with `PacketKind::Header`.
    pub body: Vec<MeshRow>,
    /// Sender's directed yaw observations, rebased by the receiver on merge.
    pub yaw_body: Vec<YawObservationMessage>,
    // Echo-only fields, filled by the responder.
    pub responder_pos: Vec3,
    pub responder_connections: Vec<String>,
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

/// Seed the base-frame anchor once, then dead-reckon it from the drone's own
/// kinematics. Only this sensor-boundary system reads world transforms.
#[allow(clippy::type_complexity)]
pub fn maintain_base_frame_references(
    mut commands: Commands,
    time: Res<Time>,
    bases: Query<&Base>,
    missing: Query<
        (Entity, &GlobalTransform, &DroneKinematics),
        (With<Drone>, Without<BaseFrameReference>),
    >,
    mut existing: Query<(&DroneKinematics, &mut BaseFrameReference), With<Drone>>,
) {
    let Some(base_pos) = bases.iter().next().map(|base| base.position) else {
        return;
    };
    for (entity, transform, kinematics) in &missing {
        let own_location_base = transform.translation() - base_pos;
        let local_direction_to_base = Quat::from_rotation_y(-kinematics.heading_deg.to_radians())
            * (-own_location_base).normalize_or_zero();
        commands.entity(entity).insert(BaseFrameReference {
            own_location_base,
            local_direction_to_base,
            gravity_up_local: Vec3::Y,
        });
    }
    let dt = time.delta_secs();
    for (kinematics, mut reference) in &mut existing {
        reference.own_location_base += kinematics.velocity * dt;
        reference.local_direction_to_base =
            Quat::from_rotation_y(-kinematics.heading_deg.to_radians())
                * (-reference.own_location_base).normalize_or_zero();
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
        &YawObservationTable,
    )>,
    positions: Query<(Entity, &GlobalTransform, &DroneKinematics), With<Drone>>,
    uuids: Query<&DroneUuid>,
) {
    for (self_entity, self_gt, drone, kin, clock, uuid, mut links, mut sent, table, yaw_table) in
        &mut drones
    {
        let self_pos = self_gt.translation();
        // All peers currently detected (regardless of resend cadence) — this
        // is what `connections` means for our own upserted row on the peer.
        let detected: Vec<Entity> = positions
            .iter()
            .filter(|(peer_entity, peer_gt, _)| {
                *peer_entity != self_entity && {
                    let peer_pos = peer_gt.translation();
                    let distance_km = (peer_pos - self_pos).length();
                    drone.antennas.iter().any(|antenna| {
                        let theta_tx =
                            antenna.off_boresight_deg(kin.heading_deg, self_pos, peer_pos);
                        antenna.rssi_dbm(theta_tx, 0.0, distance_km) >= antenna.sensitivity_dbm
                    })
                }
            })
            .map(|(peer_entity, _, _)| peer_entity)
            .collect();
        let body: Vec<MeshRow> = table.0.values().cloned().collect();
        let yaw_body: Vec<YawObservationMessage> = yaw_table
            .edges
            .values()
            .map(YawObservationMessage::from)
            .collect();

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

            let Ok((_, peer_gt, peer_kin)) = positions.get(peer_entity) else {
                continue;
            };
            let local_direction = Quat::from_rotation_y(-kin.heading_deg.to_radians())
                * (peer_gt.translation() - self_pos).normalize_or_zero();
            let header = NetworkHeader {
                id: uuid.0.clone(),
                connected_antenna: local_direction,
                relative_yaw_rad: wrap_yaw_rad(
                    peer_kin.heading_deg.to_radians() - kin.heading_deg.to_radians(),
                ),
                flight_direction: kin.velocity * FLIGHT_LOOKAHEAD_SECS,
                time_received: clock.now,
            };
            mailbox.0.push((
                peer_entity,
                Packet {
                    kind: PacketKind::Header,
                    origin: self_entity,
                    responder: peer_entity,
                    responder_id: uuids.get(peer_entity).unwrap().0.clone(),
                    origin_pos: self_pos,
                    header: header.clone(),
                    body: body.clone(),
                    yaw_body: yaw_body.clone(),
                    responder_pos: Vec3::ZERO,
                    responder_connections: Vec::new(),
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
#[allow(clippy::type_complexity)]
pub fn route_packets(
    mut mailbox: ResMut<Mailbox>,
    mut drones: Query<(
        Entity,
        &GlobalTransform,
        &mut RangingResults,
        &DroneUuid,
        &DroneClock,
        &LinkSet,
        &mut MeshTable,
        &mut YawObservationTable,
        &mut TrackedPeers,
        Option<&BaseFrameReference>,
    )>,
) {
    let uuid_by_entity: HashMap<Entity, String> = drones
        .iter()
        .map(|(entity, _, _, uuid, ..)| (entity, uuid.0.clone()))
        .collect();
    let packets = std::mem::take(&mut mailbox.0);
    let mut outgoing: Vec<(Entity, Packet)> = Vec::new();

    for (target, pkt) in packets {
        match pkt.kind {
            PacketKind::Header => {
                // `target` is the responder.
                let Ok((
                    _,
                    resp_gt,
                    _,
                    resp_uuid,
                    resp_clock,
                    resp_links,
                    mut resp_table,
                    mut resp_yaw,
                    mut resp_tracked,
                    _,
                )) = drones.get_mut(target)
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
                resp_tracked
                    .0
                    .insert(pkt.origin, pkt.origin_pos + pkt.header.flight_direction);

                // Relay-merge every third-party row: one hop further than it
                // was from the sender, and only if that's an improvement.
                // Preserve the sender-relative age while rebasing the stored
                // timestamp into this receiver's independent clock domain.
                for row in &pkt.body {
                    if row.id == resp_uuid.0 || row.id == pkt.header.id {
                        continue;
                    }
                    let Some(candidate_distance) =
                        relayed_neighbour_distance(row.neighbour_distance)
                    else {
                        continue;
                    };
                    let Some(timestamp) =
                        rebase_timestamp(pkt.header.time_received, row.timestamp, resp_clock.now)
                    else {
                        continue;
                    };
                    let candidate = MeshRow {
                        id: row.id.clone(),
                        timestamp,
                        location: row.location,
                        neighbour_distance: candidate_distance,
                        connections: row.connections.clone(),
                    };
                    match resp_table.0.get_mut(&row.id) {
                        Some(existing)
                            if should_replace_row(existing, &candidate, resp_clock.now) =>
                        {
                            *existing = candidate;
                        }
                        Some(_) => {}
                        None => {
                            if row_is_valid(&candidate) {
                                resp_table.0.insert(row.id.clone(), candidate);
                            }
                        }
                    }
                }

                for incoming in &pkt.yaw_body {
                    let Some(neighbour_distance) =
                        relayed_neighbour_distance(incoming.neighbour_distance)
                    else {
                        continue;
                    };
                    let Some(timestamp) = rebase_timestamp(
                        pkt.header.time_received,
                        incoming.timestamp,
                        resp_clock.now,
                    ) else {
                        continue;
                    };
                    let candidate = YawObservation {
                        from: incoming.from.clone(),
                        to: incoming.to.clone(),
                        measured_yaw_rad: incoming.measured_yaw_rad,
                        vector_length: incoming.vector_length,
                        neighbour_distance,
                        timestamp,
                        corrected_yaw_rad: incoming.measured_yaw_rad,
                        generation: incoming.generation,
                    };
                    let key = (candidate.from.clone(), candidate.to.clone());
                    match resp_yaw.edges.get_mut(&key) {
                        Some(existing)
                            if should_replace_yaw(existing, &candidate, resp_clock.now) =>
                        {
                            *existing = candidate;
                        }
                        Some(_) => {}
                        None if yaw_observation_is_valid(&candidate) => {
                            resp_yaw.edges.insert(key, candidate);
                        }
                        None => {}
                    }
                }

                // Send the header straight back for ranging.
                let dist_km = (responder_pos - pkt.origin_pos).length() as f64;
                let prop = dist_km / SPEED_OF_LIGHT_KM_S;

                let mut echo = pkt.clone();
                echo.kind = PacketKind::Echo;
                echo.responder_pos = responder_pos;
                echo.responder_connections = resp_links
                    .connected
                    .keys()
                    .filter_map(|entity| uuid_by_entity.get(entity).cloned())
                    .collect();
                echo.responder_connections.sort();
                echo.responder_delay = TURNAROUND_DELAY_S;
                // Modeled receive time on the originator's clock:
                // send + round-trip propagation + responder turnaround.
                echo.arrival_time = pkt.header.time_received + 2.0 * prop + TURNAROUND_DELAY_S;
                outgoing.push((pkt.origin, echo));
            }
            PacketKind::Echo => {
                // `target` is the originator — recover distance from timing.
                let Ok((
                    _,
                    orig_gt,
                    mut results,
                    origin_uuid,
                    origin_clock,
                    _,
                    mut table,
                    mut yaw,
                    _,
                    base_reference,
                )) = drones.get_mut(target)
                else {
                    continue;
                };
                let round_trip = pkt.arrival_time - pkt.header.time_received;
                let distance_km =
                    ((round_trip - pkt.responder_delay) * SPEED_OF_LIGHT_KM_S / 2.0) as f32;
                let range =
                    SphericalVec::toward(orig_gt.translation(), pkt.responder_pos, distance_km);
                results.0.push((pkt.responder, range));
                let Some(reference) = base_reference else {
                    continue;
                };
                let Some(location) = project_direct_observation_to_base(
                    pkt.header.connected_antenna,
                    reference.gravity_up_local,
                    reference.local_direction_to_base,
                    reference.own_location_base,
                    distance_km,
                ) else {
                    continue;
                };
                let responder_uuid = pkt.responder_id.clone();
                table.0.insert(
                    responder_uuid.clone(),
                    MeshRow {
                        id: responder_uuid.clone(),
                        timestamp: origin_clock.now,
                        location,
                        neighbour_distance: 0,
                        connections: pkt.responder_connections.clone(),
                    },
                );
                let generation = yaw.next_generation();
                yaw.edges.insert(
                    (origin_uuid.0.clone(), responder_uuid.clone()),
                    YawObservation {
                        from: origin_uuid.0.clone(),
                        to: responder_uuid,
                        measured_yaw_rad: pkt.header.relative_yaw_rad,
                        corrected_yaw_rad: pkt.header.relative_yaw_rad,
                        vector_length: distance_km,
                        neighbour_distance: 0,
                        timestamp: origin_clock.now,
                        generation,
                    },
                );
            }
        }
    }

    mailbox.0 = outgoing;
}

/// Recompute yaw-only loop closure from complete, fresh directed observations.
/// Every pass resets obsolete corrections to the measured value; each valid
/// loop is then committed atomically.
pub fn apply_loop_closure(
    mut drones: Query<(
        &DroneClock,
        Option<&DroneUuid>,
        &MeshTable,
        &mut YawObservationTable,
    )>,
) {
    for (clock, owner_uuid, table, mut observations) in &mut drones {
        let mut fresh = MeshTable(
            table
                .0
                .iter()
                .filter(|(_, row)| {
                    let age = clock.now - row.timestamp;
                    row_is_valid(row)
                        && age >= 0.0
                        && age <= HEADER_INTERVAL_SECS * (row.neighbour_distance as f64 + 1.0)
                })
                .map(|(id, row)| (id.clone(), row.clone()))
                .collect(),
        );
        if let Some(owner_uuid) = owner_uuid {
            let mut connections: Vec<String> = observations
                .edges
                .values()
                .filter(|edge| edge.from == owner_uuid.0)
                .map(|edge| edge.to.clone())
                .collect();
            connections.sort();
            connections.dedup();
            fresh.0.insert(
                owner_uuid.0.clone(),
                MeshRow {
                    id: owner_uuid.0.clone(),
                    timestamp: clock.now,
                    location: Vec3::ZERO,
                    neighbour_distance: 0,
                    connections,
                },
            );
        }
        for edge in observations.edges.values_mut() {
            edge.corrected_yaw_rad = edge.measured_yaw_rad;
        }
        for ids in discover_loops(&fresh) {
            let mut edges = Vec::with_capacity(ids.len());
            for index in 0..ids.len() {
                let from = &ids[index];
                let to = &ids[(index + 1) % ids.len()];
                let Some(edge) = observations.edges.get(&(from.clone(), to.clone())) else {
                    edges.clear();
                    break;
                };
                let age = clock.now - edge.timestamp;
                if !yaw_observation_is_valid(edge)
                    || age < 0.0
                    || age > HEADER_INTERVAL_SECS * (edge.neighbour_distance as f64 + 1.0)
                {
                    edges.clear();
                    break;
                }
                edges.push(edge.clone());
            }
            if close_yaw_loop(&mut edges).is_some() {
                for edge in edges {
                    if let Some(stored) = observations
                        .edges
                        .get_mut(&(edge.from.clone(), edge.to.clone()))
                    {
                        stored.corrected_yaw_rad = edge.corrected_yaw_rad;
                    }
                }
            }
        }
    }
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
    let base_pos = bases
        .iter()
        .next()
        .map(|b| b.position)
        .unwrap_or(Vec3::ZERO);
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
        let Ok((uuid, gt, links, mut pairing)) = drones.get_mut(msg.to) else {
            continue;
        };

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
                                kind: ReconnectKind::Position {
                                    payload: pos_rel_base,
                                },
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
        b(hi, 56),
        b(hi, 48),
        b(hi, 40),
        b(hi, 32),
        b(hi, 24),
        b(hi, 16),
        b(hi, 8),
        b(hi, 0),
        b(lo, 56),
        b(lo, 48),
        b(lo, 40),
        b(lo, 32),
        b(lo, 24),
        b(lo, 16),
        b(lo, 8),
        b(lo, 0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::system::RunSystemOnce;

    use crate::drone::{DroneType, make_antenna};
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
    fn seed_mesh_row(
        world: &mut World,
        base_pos: Vec3,
        owner: Entity,
        peer: Entity,
        peer_pos: Vec3,
    ) {
        if world.get::<BaseFrameReference>(owner).is_none() {
            let owner_pos = world.get::<GlobalTransform>(owner).unwrap().translation();
            world.entity_mut(owner).insert(BaseFrameReference {
                own_location_base: owner_pos - base_pos,
                local_direction_to_base: (base_pos - owner_pos).normalize_or_zero(),
                gravity_up_local: Vec3::Y,
            });
        }
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
        world
            .run_system_once(detect_links_and_send_headers)
            .unwrap();
    }

    #[test]
    fn link_forms_between_facing_in_range_drones() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base {
            id: "base".into(),
            position: base_pos,
            antennas: vec![],
        });
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
        assert!(
            !world.resource::<Mailbox>().0.is_empty(),
            "headers should be queued"
        );
    }

    #[test]
    fn out_of_range_drones_do_not_link() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        world.spawn(Base {
            id: "base".into(),
            position: Vec3::new(0.0, 0.0, -5.0),
            antennas: vec![],
        });
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
        world.spawn(Base {
            id: "base".into(),
            position: base_pos,
            antennas: vec![],
        });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(1.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let b = spawn_drone(&mut world, b_pos, 1);
        let a_uuid = uuid_of(&world, a);
        seed_mesh_row(&mut world, base_pos, a, b, b_pos);
        seed_mesh_row(&mut world, base_pos, b, a, a_pos);

        aim_and_detect(&mut world);
        world.get_mut::<MeshTable>(a).unwrap().0.clear();
        world.get_mut::<MeshTable>(b).unwrap().0.clear();
        // Header handling only creates the echo. Direct lookup knowledge is
        // committed after the echo-derived range is available.
        world.run_system_once(route_packets).unwrap();
        assert!(!world.get::<MeshTable>(b).unwrap().0.contains_key(&a_uuid));
        world.run_system_once(route_packets).unwrap();

        let b_table = world.get::<MeshTable>(b).unwrap();
        let row = b_table.0.get(&a_uuid).expect("B should have learned A");
        assert_eq!(row.neighbour_distance, 0, "a direct peer is distance 0");
        assert!((row.location - (a_pos - base_pos)).length() < 1.0e-4);
    }

    #[test]
    fn ranging_recovers_the_true_distance() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base {
            id: "base".into(),
            position: base_pos,
            antennas: vec![],
        });
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
        assert!(
            (sv.length - 2.0).abs() < 0.01,
            "measured distance ~2 km, got {}",
            sv.length
        );
    }

    /// One full round: advance every drone's clock past the resend interval,
    /// re-aim, detect links, route one hop of gossip.
    fn step(world: &mut World, drones: &[Entity]) {
        for &e in drones {
            world.get_mut::<DroneClock>(e).unwrap().now += 0.2;
        }
        world.run_system_once(maintain_mesh_antennas).unwrap();
        world
            .run_system_once(detect_links_and_send_headers)
            .unwrap();
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
        world.spawn(Base {
            id: "base".into(),
            position: base_pos,
            antennas: vec![],
        });
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
        let row = c_table
            .0
            .get(&a_uuid)
            .expect("C should have learned A by relay");
        assert_eq!(
            row.neighbour_distance, 1,
            "relayed peer is one hop past the direct link"
        );
    }

    fn test_row(
        id: &str,
        timestamp: f64,
        location: Vec3,
        distance: u32,
        connections: &[&str],
    ) -> MeshRow {
        MeshRow {
            id: id.into(),
            timestamp,
            location,
            neighbour_distance: distance,
            connections: connections.iter().map(|id| (*id).into()).collect(),
        }
    }

    #[test]
    fn direct_observation_is_projected_into_base_frame() {
        // Receiver is east of base. In its local frame the base is forward
        // and the observed peer is right; in the base frame that is +Z.
        let projected = project_direct_observation_to_base(
            Vec3::X,
            Vec3::Y,
            Vec3::Z,
            Vec3::new(10.0, 2.0, 0.0),
            3.0,
        )
        .unwrap();
        assert!((projected - Vec3::new(10.0, 2.0, 3.0)).length() < 1.0e-5);
    }

    #[test]
    fn direct_projection_is_independent_of_receiver_heading() {
        let heading = 73.0_f32.to_radians();
        let inverse_heading = Quat::from_rotation_y(-heading);
        let own = Vec3::new(4.0, 1.0, -2.0);
        let neighbour = Vec3::new(7.0, 3.0, 2.0);
        let local_neighbour = inverse_heading * (neighbour - own).normalize();
        let local_base = inverse_heading * (-own).normalize();
        let projected = project_direct_observation_to_base(
            local_neighbour,
            Vec3::Y,
            local_base,
            own,
            (neighbour - own).length(),
        )
        .unwrap();
        assert!((projected - neighbour).length() < 1.0e-5);
    }

    #[test]
    fn base_frame_reference_is_seeded_at_the_sensor_boundary() {
        let mut world = World::new();
        world.insert_resource(Time::<()>::default());
        let base_pos = Vec3::new(-2.0, 0.0, 4.0);
        world.spawn(Base {
            id: "base".into(),
            position: base_pos,
            antennas: vec![],
        });
        let drone = spawn_drone(&mut world, Vec3::new(3.0, 1.0, 6.0), 0);
        world
            .run_system_once(maintain_base_frame_references)
            .unwrap();
        let reference = world.get::<BaseFrameReference>(drone).unwrap();
        assert_eq!(reference.own_location_base, Vec3::new(5.0, 1.0, 2.0));
        assert!(
            (reference.local_direction_to_base - (-reference.own_location_base).normalize())
                .length()
                < 1.0e-6
        );
    }

    #[test]
    fn relayed_base_frame_location_is_not_rotated_again() {
        let relayed = test_row("remote", 2.0, Vec3::new(4.0, 5.0, 6.0), 1, &[]);
        let candidate = MeshRow {
            neighbour_distance: relayed.neighbour_distance + 1,
            ..relayed.clone()
        };
        assert_eq!(candidate.location, relayed.location);
    }

    #[test]
    fn lookup_replacement_prefers_distance_then_timeout_and_rejects_invalid() {
        let existing = test_row("peer", 10.0, Vec3::X, 2, &[]);
        let closer = test_row("peer", 10.01, Vec3::Z, 1, &[]);
        let farther = test_row("peer", 10.01, Vec3::Y, 3, &[]);
        assert!(should_replace_row(&existing, &closer, 10.01));
        assert!(!should_replace_row(&existing, &farther, 10.29));
        assert!(should_replace_row(&existing, &farther, 10.30));

        let invalid = test_row("peer", 10.01, Vec3::splat(f32::NAN), 0, &[]);
        assert!(!should_replace_row(&existing, &invalid, 100.0));
    }

    #[test]
    fn relayed_timestamp_is_rebased_between_independent_clocks() {
        // Sender saw the row 0.25 s ago on a clock near 10,000. Receiver's
        // unrelated clock is near 3; only the age crosses the link.
        let rebased = rebase_timestamp(10_000.0, 9_999.75, 3.0).unwrap();
        assert!((rebased - 2.75).abs() < f64::EPSILON);
        assert!(rebase_timestamp(10.0, 10.1, 3.0).is_none());
    }

    #[test]
    fn maximum_neighbour_distance_is_rejected_instead_of_wrapping() {
        assert_eq!(relayed_neighbour_distance(u32::MAX), None);
    }

    fn yaw_edge(from: &str, to: &str, yaw: f32, distance: u32) -> YawObservation {
        YawObservation {
            from: from.into(),
            to: to.into(),
            measured_yaw_rad: yaw,
            corrected_yaw_rad: yaw,
            vector_length: 3.5,
            neighbour_distance: distance,
            timestamp: 5.0,
            generation: 1,
        }
    }

    #[test]
    fn weighted_loop_closure_zeroes_residual_and_preserves_lengths() {
        let mut edges = vec![
            yaw_edge("a", "b", 0.0, 0),
            yaw_edge("b", "c", 0.3, 1),
            yaw_edge("c", "a", -0.1, 3),
        ];
        let lengths: Vec<f32> = edges.iter().map(|edge| edge.vector_length).collect();
        let corrections = close_yaw_loop(&mut edges).unwrap();
        let corrected_residual =
            wrap_yaw_rad(edges.iter().map(|edge| edge.corrected_yaw_rad).sum());
        assert!(corrected_residual.abs() <= LOOP_CLOSURE_TOLERANCE_RAD);
        for (edge, length) in edges.iter().zip(lengths) {
            assert_eq!(edge.vector_length, length);
        }
        assert!((corrections.iter().map(|c| c.delta_rad).sum::<f32>() + 0.2).abs() < 1.0e-5);
        assert!((corrections[2].delta_rad / corrections[0].delta_rad - 4.0).abs() < 1.0e-5);
    }

    #[test]
    fn perfect_and_full_turn_loops_receive_no_false_correction() {
        for yaws in [[0.2, -0.7, 0.5], [std::f32::consts::TAU, 0.0, 0.0]] {
            let mut edges = vec![
                yaw_edge("a", "b", yaws[0], 0),
                yaw_edge("b", "c", yaws[1], 1),
                yaw_edge("c", "a", yaws[2], 2),
            ];
            let corrections = close_yaw_loop(&mut edges).unwrap();
            assert!(
                corrections
                    .iter()
                    .all(|correction| correction.delta_rad.abs() < 1.0e-5)
            );
            assert!(
                wrap_yaw_rad(edges.iter().map(|edge| edge.corrected_yaw_rad).sum()).abs() < 1.0e-5
            );
        }
    }

    #[test]
    fn wire_yaw_observation_carries_measurement_not_local_correction() {
        let mut local = yaw_edge("a", "b", 0.25, 0);
        local.corrected_yaw_rad = -0.5;
        let message = YawObservationMessage::from(&local);
        assert_eq!(message.measured_yaw_rad, 0.25);
        let received = YawObservation {
            from: message.from,
            to: message.to,
            measured_yaw_rad: message.measured_yaw_rad,
            corrected_yaw_rad: message.measured_yaw_rad,
            vector_length: message.vector_length,
            neighbour_distance: message.neighbour_distance,
            timestamp: message.timestamp,
            generation: message.generation,
        };
        assert_eq!(received.corrected_yaw_rad, received.measured_yaw_rad);
        assert_ne!(received.corrected_yaw_rad, local.corrected_yaw_rad);
    }

    #[test]
    fn incomplete_loop_is_rejected_without_modification() {
        let mut edges = vec![yaw_edge("a", "b", 0.0, 0), yaw_edge("b", "c", 0.2, 0)];
        let before = edges.clone();
        assert!(close_yaw_loop(&mut edges).is_none());
        assert_eq!(edges, before);
    }

    #[test]
    fn loops_are_discovered_once_and_incomplete_references_are_ignored() {
        let table = MeshTable(HashMap::from([
            ("a".into(), test_row("a", 1.0, Vec3::ZERO, 0, &["b", "c"])),
            ("b".into(), test_row("b", 1.0, Vec3::X, 1, &["a", "c"])),
            (
                "c".into(),
                test_row("c", 1.0, Vec3::Z, 2, &["a", "b", "d", "e"]),
            ),
            ("d".into(), test_row("d", 1.0, Vec3::X, 0, &["c", "e"])),
            ("e".into(), test_row("e", 1.0, Vec3::Z, 0, &["c", "d"])),
            (
                "orphan".into(),
                test_row("orphan", 1.0, Vec3::Y, 0, &["missing"]),
            ),
        ]));
        assert_eq!(
            discover_loops(&table),
            vec![vec!["a", "b", "c"], vec!["c", "d", "e"]]
        );
    }

    #[test]
    fn loop_closure_system_resets_stale_corrections() {
        let mut world = World::new();
        let table = MeshTable(HashMap::from([
            ("a".into(), test_row("a", 5.0, Vec3::ZERO, 0, &["b"])),
            (
                "b".into(),
                test_row("b", 5.0, Vec3::new(1.0, 0.0, 1.0), 1, &["c"]),
            ),
            (
                "c".into(),
                test_row("c", 5.0, Vec3::new(-1.0, 0.0, 2.0), 2, &["a"]),
            ),
        ]));
        let observations = YawObservationTable {
            edges: HashMap::from([
                (("a".into(), "b".into()), yaw_edge("a", "b", 0.2, 0)),
                (("b".into(), "c".into()), yaw_edge("b", "c", 0.2, 0)),
                (("c".into(), "a".into()), yaw_edge("c", "a", 0.2, 0)),
            ]),
            next_generation: 1,
        };
        let entity = world
            .spawn((DroneClock { now: 5.0 }, table, observations))
            .id();
        world.run_system_once(apply_loop_closure).unwrap();
        let corrected = world.get::<YawObservationTable>(entity).unwrap();
        assert!(
            corrected
                .edges
                .values()
                .all(|edge| edge.corrected_yaw_rad.abs() < 1.0e-5)
        );

        // Stale topology must not keep applying an obsolete correction.
        world.get_mut::<DroneClock>(entity).unwrap().now = 100.0;
        world.run_system_once(apply_loop_closure).unwrap();
        let reset = world.get::<YawObservationTable>(entity).unwrap();
        assert!(
            reset
                .edges
                .values()
                .all(|edge| edge.corrected_yaw_rad == edge.measured_yaw_rad)
        );
    }

    /// A header is not resent until `HEADER_INTERVAL_SECS` has passed on the
    /// sending drone's *own* clock — resend cadence is per-drone, not shared.
    #[test]
    fn header_not_resent_within_interval_on_own_clock() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base {
            id: "base".into(),
            position: base_pos,
            antennas: vec![],
        });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(1.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let _b = spawn_drone(&mut world, b_pos, 1);
        seed_mesh_row(&mut world, base_pos, a, _b, b_pos);
        seed_mesh_row(&mut world, base_pos, _b, a, a_pos);

        aim_and_detect(&mut world);
        assert!(
            !world.resource::<Mailbox>().0.is_empty(),
            "first detect sends headers"
        );

        // Drain, then detect again with the clock unchanged — nothing due yet.
        world.resource_mut::<Mailbox>().0.clear();
        world
            .run_system_once(detect_links_and_send_headers)
            .unwrap();
        assert!(
            world.resource::<Mailbox>().0.is_empty(),
            "no resend before the interval elapses on the own clock"
        );

        // Advance every clock past the interval → headers resend.
        world.get_mut::<DroneClock>(a).unwrap().now += HEADER_INTERVAL_SECS + 0.01;
        world.get_mut::<DroneClock>(_b).unwrap().now += HEADER_INTERVAL_SECS + 0.01;
        world
            .run_system_once(detect_links_and_send_headers)
            .unwrap();
        assert!(
            !world.resource::<Mailbox>().0.is_empty(),
            "resend after interval elapses"
        );
    }

    /// A link is dropped once the peer moves out of range — `LinkSet` reflects
    /// only currently-reachable peers, not stale ones.
    #[test]
    fn link_drops_when_peer_leaves_range() {
        let mut world = World::new();
        world.insert_resource(Mailbox::default());
        let base_pos = Vec3::new(0.0, 0.0, -5.0);
        world.spawn(Base {
            id: "base".into(),
            position: base_pos,
            antennas: vec![],
        });
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3::new(1.0, 0.0, 0.0);
        let a = spawn_drone(&mut world, a_pos, 0);
        let b = spawn_drone(&mut world, b_pos, 1);
        seed_mesh_row(&mut world, base_pos, a, b, b_pos);
        seed_mesh_row(&mut world, base_pos, b, a, a_pos);

        aim_and_detect(&mut world);
        assert!(
            world.get::<LinkSet>(a).unwrap().connected.contains_key(&b),
            "linked in range"
        );

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
