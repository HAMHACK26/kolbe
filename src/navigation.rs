//! Drone navigation.
//!
//! This module is the single authority for how a drone physically moves.
//! Nothing else is allowed to write a drone's position or velocity directly
//! — everything goes through [`navigate`], so the speed/acceleration/climb
//! limits in [`FlightLimits`] can never be silently bypassed by some other
//! piece of code just setting `position += anything`.
//!
//! ## How a real quadcopter actually gets from A to B
//!
//! A multirotor has no wheels, wings, or rudder. The only thing it can do is
//! spin four (or more) rotors, all pointed the same direction, and vary each
//! one's thrust. To hover, all rotors push straight up, exactly cancelling
//! gravity. To move *horizontally*, the flight controller tilts the whole
//! rotor plane — pitching the nose down to go forward, rolling to go
//! sideways — which redirects a slice of that same total thrust from "pure
//! lift" into "sideways push". There is no separate horizontal-thrust
//! source; going faster horizontally always steals from the thrust holding
//! the drone up, which is exactly why real flight controllers rate-limit how
//! fast that tilt can change (both for stability/gimbal-comfort, and because
//! tilting too hard too fast risks a real altitude sag).
//!
//! The practical upshot, modeled here:
//!   - Velocity cannot jump to a new value in one tick. It can only be
//!     *steered* toward a desired velocity, at a bounded acceleration
//!     ([`FlightLimits::max_accel_mps2`]) — this is the tilt-and-wait
//!     behavior above.
//!   - Horizontal top speed is capped ([`FlightLimits::max_speed_mps`]).
//!   - Climbing and descending are capped *separately*, and asymmetrically —
//!     see the field docs on [`FlightLimits`] for why going down is more
//!     restricted than going up.
//!   - The drone visibly yaws to face its direction of travel, at a bounded
//!     turn rate, the way a camera/inspection quad does (so its gimbal keeps
//!     pointing forward) — it doesn't crab sideways or snap-rotate.
//!   - On final approach to a target it brakes early rather than flying
//!     straight at full speed and overshooting, because a real autopilot
//!     always plans a stopping distance.

use bevy::prelude::*;

use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::networking::{DroneClock, LinkSet};
use crate::recovery::RecoveryState;

/// The flight envelope a drone is governed to. All limits are deliberately
/// modest: real consumer/enterprise quads are electronically governed well
/// below their raw physical ceiling for stability, camera stability, and
/// safety margin — this models that governance, not a machine's theoretical
/// maximum. These are "normal" camera/inspection-quad numbers, not a racing
/// build's.
#[derive(Clone, Copy, Debug)]
pub struct FlightLimits {
    /// Maximum horizontal ground speed, in meters/second.
    ///
    /// Default 15.0 m/s (~54 km/h) — a typical "sport mode" ceiling for a
    /// mid-size camera quad (e.g. DJI Mavic-class). A racing/FPV quad can
    /// exceed 30 m/s, but that is *not* what this model represents.
    ///
    /// This is the one field [`FlightLimits::set_max_speed`] is meant to
    /// change at runtime — e.g. a mission planner dialing in a slower cap
    /// for a payload-heavy flight, a geofence controller throttling speed
    /// near a boundary, or a "return to home" profile that flies slower than
    /// normal cruise.
    pub max_speed_mps: f32,

    /// Maximum horizontal acceleration, in meters/second².
    ///
    /// Physically this is a limit on how fast the flight controller is
    /// willing to change the rotor-plane tilt angle. Default 4.0 m/s² is a
    /// gentle, stable-cinematic-mode value — a drone starting from a stop
    /// takes ~3.75 s to reach the default 15 m/s max speed under this limit,
    /// which reads as smooth, deliberate flight rather than a snap to full
    /// speed.
    pub max_accel_mps2: f32,

    /// Maximum climb rate, in meters/second.
    ///
    /// Climbing costs "spare" thrust beyond what's already being used to
    /// hover — the motors don't have unlimited headroom, so vertical speed
    /// is deliberately kept lower than horizontal cruise speed. Default
    /// 5.0 m/s.
    pub max_climb_mps: f32,

    /// Maximum descent rate, in meters/second.
    ///
    /// Descending fast means flying down through your own rotor downwash —
    /// "vortex ring state" — which causes a real, sudden loss of lift and is
    /// one of the more common causes of multirotor crashes. Real flight
    /// controllers cap descent noticeably below climb rate for exactly this
    /// reason, which is why this default (3.0 m/s) is lower than
    /// `max_climb_mps`, not just mirrored.
    pub max_descend_mps: f32,

    /// Maximum yaw (turn-to-face) rate, in degrees/second.
    ///
    /// A camera/inspection quad rotates to face its direction of travel so
    /// its forward-facing gimbal/camera stays pointed the right way — this
    /// is a flight-controller choice, not an aerodynamic requirement (the
    /// airframe itself can move in any horizontal direction regardless of
    /// which way it's facing), but it's what makes the motion read as a real
    /// piloted drone instead of a physics-sim puck. Default 90 deg/s: a full
    /// about-face takes 2 seconds.
    pub max_yaw_rate_deg_s: f32,
}

impl Default for FlightLimits {
    fn default() -> Self {
        Self {
            max_speed_mps: 15.0,
            max_accel_mps2: 4.0,
            max_climb_mps: 5.0,
            max_descend_mps: 3.0,
            max_yaw_rate_deg_s: 90.0,
        }
    }
}

impl FlightLimits {
    /// The sanctioned way to change a drone's speed cap at runtime.
    ///
    /// This exists so speed changes are a deliberate, auditable act (a
    /// mission planner, a geofence, an operator override) rather than
    /// something any code anywhere can do by poking a field directly.
    /// Negative input is clamped to 0 — a drone can always be told to
    /// (effectively) hold position, never to fly at a negative speed.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn set_max_speed(&mut self, max_speed_mps: f32) {
        self.max_speed_mps = max_speed_mps.max(0.0);
    }

    /// The whole envelope scaled by `factor` — a "fly everything N times
    /// faster" knob (see [`MovementSpeed`]).
    ///
    /// Every limit scales together, including yaw, which is what keeps the
    /// motion coherent rather than just faster. Braking distance is `v²/2a`,
    /// so scaling speed and acceleration by the same factor grows the stopping
    /// runway linearly — a 4x drone needs 4x the room to stop, and
    /// [`drift_navigate`] looks that far ahead automatically because it derives
    /// the lookahead from these same limits.
    pub fn scaled(&self, factor: f32) -> Self {
        let factor = factor.max(0.0);
        Self {
            max_speed_mps: self.max_speed_mps * factor,
            max_accel_mps2: self.max_accel_mps2 * factor,
            max_climb_mps: self.max_climb_mps * factor,
            max_descend_mps: self.max_descend_mps * factor,
            max_yaw_rate_deg_s: self.max_yaw_rate_deg_s * factor,
        }
    }

    /// The same envelope expressed in the kilometer units the simulation world
    /// is scaled in, so [`navigate`] integrates consistently with km-space
    /// positions.
    ///
    /// Only the linear limits convert — `max_yaw_rate_deg_s` is an angular
    /// rate and is scale-independent.
    pub fn in_km(&self) -> Self {
        Self {
            max_speed_mps: self.max_speed_mps / 1000.0,
            max_accel_mps2: self.max_accel_mps2 / 1000.0,
            max_climb_mps: self.max_climb_mps / 1000.0,
            max_descend_mps: self.max_descend_mps / 1000.0,
            max_yaw_rate_deg_s: self.max_yaw_rate_deg_s,
        }
    }
}

/// A drone's current physical state: where it is, how fast it's currently
/// moving, and which way it's currently facing.
#[derive(Clone, Copy, Debug, Default)]
pub struct DroneState {
    /// World-space position, meters. Y is altitude (up).
    pub position: Vec3,
    /// Current velocity, meters/second. This is *not* the same as "desired
    /// direction to target" — see [`navigate`] for why they differ.
    pub velocity: Vec3,
    /// Current heading (yaw), degrees. 0° = +Z, increasing clockwise when
    /// viewed from above.
    pub heading_deg: f32,
}

/// Advance a drone one simulation tick toward `target`, honoring `limits`.
///
/// This is the *only* function that should ever move a drone. Call it every
/// tick with the drone's current state, its current navigation goal (a
/// waypoint, a tracked peer's position, a return-to-home point — `navigate`
/// doesn't care what the target represents, only where it is), the flight
/// envelope to respect, and the elapsed time since the last call.
///
/// Internally this does exactly what a real flight controller does, in
/// order:
///
/// 1. **Compute the desired velocity.** Straight at the target, at full
///    speed — this is the "ideal" instruction a pilot would give ("go
///    there, as fast as you're allowed to"). Everything after this step
///    exists because the airframe physically can't achieve that instantly.
///    On final approach, the desired speed is capped by the braking
///    distance at max deceleration (`v = sqrt(2 * a * d)`), so the drone
///    starts slowing down *before* it reaches the target instead of flying
///    past it at full speed and having to loop back — exactly what a real
///    autopilot's trajectory planner does.
///
/// 2. **Steer current velocity toward desired velocity, rate-limited.** A
///    quad can't retarget its velocity vector instantly — it has to tilt
///    there, and that tilt rate is bounded. This moves `state.velocity`
///    toward the desired velocity by at most `max_accel_mps2 * dt` this
///    tick.
///
/// 3. **Clamp to the flight envelope.** Horizontal speed is capped at
///    `max_speed_mps`; vertical speed is clamped to the (deliberately
///    asymmetric) climb/descend limits.
///
/// 4. **Integrate position.** `position += velocity * dt`.
///
/// 5. **Yaw toward the direction of travel, rate-limited**, the way a
///    camera/inspection quad visibly turns to face where it's going.
pub fn navigate(state: &mut DroneState, target: Vec3, limits: &FlightLimits, dt: f32) {
    if dt <= 0.0 {
        // Nothing to integrate over a zero or negative timestep.
        return;
    }

    // ---- 1. Desired velocity -------------------------------------------
    let to_target = target - state.position;
    let distance = to_target.length();
    let desired_velocity = if distance > f32::EPSILON {
        // Brake early: don't approach at full speed only to overshoot and
        // have to correct. `sqrt(2 * a * d)` is the speed at which, braking
        // at max_accel_mps2 starting right now, you come to rest exactly at
        // the target.
        let approach_speed_cap = (2.0 * limits.max_accel_mps2 * distance).sqrt();
        let speed = limits.max_speed_mps.min(approach_speed_cap);
        to_target.normalize() * speed
    } else {
        Vec3::ZERO
    };

    // ---- 2. Rate-limited steering toward desired velocity --------------
    let velocity_error = desired_velocity - state.velocity;
    let max_delta_v = limits.max_accel_mps2 * dt;
    let steered_velocity = if velocity_error.length() <= max_delta_v {
        // Within reach this tick — snap the remainder rather than
        // asymptotically crawling toward it forever.
        desired_velocity
    } else {
        state.velocity + velocity_error.normalize() * max_delta_v
    };

    // ---- 3. Clamp to the flight envelope --------------------------------
    // Horizontal (XZ-plane) speed and vertical (Y) climb/descend are
    // governed separately and asymmetrically — see FlightLimits docs.
    let horizontal = Vec3::new(steered_velocity.x, 0.0, steered_velocity.z);
    let horizontal_speed = horizontal.length();
    let clamped_horizontal = if horizontal_speed > limits.max_speed_mps && horizontal_speed > 0.0
    {
        horizontal * (limits.max_speed_mps / horizontal_speed)
    } else {
        horizontal
    };
    let clamped_vertical = steered_velocity.y.clamp(-limits.max_descend_mps, limits.max_climb_mps);

    state.velocity = Vec3::new(clamped_horizontal.x, clamped_vertical, clamped_horizontal.z);

    // ---- 4. Integrate position ------------------------------------------
    state.position += state.velocity * dt;

    // ---- 5. Yaw toward direction of travel, rate-limited ----------------
    // Only re-yaw when actually moving with meaningful horizontal speed —
    // otherwise a nearly-stationary drone would visibly jitter its heading
    // in response to velocity noise.
    let final_horizontal_speed =
        Vec3::new(state.velocity.x, 0.0, state.velocity.z).length();
    if final_horizontal_speed > 0.05 {
        let desired_heading_deg = state.velocity.x.atan2(state.velocity.z).to_degrees();
        // Shortest signed angular difference, in (-180, 180].
        let mut delta = (desired_heading_deg - state.heading_deg + 540.0) % 360.0 - 180.0;
        let max_delta_yaw = limits.max_yaw_rate_deg_s * dt;
        delta = delta.clamp(-max_delta_yaw, max_delta_yaw);
        state.heading_deg = (state.heading_deg + delta).rem_euclid(360.0);
    }
}

// ─── Movement speed ───────────────────────────────────────────────────────────

/// Slowest setting of the movement-speed control — real-time flight.
pub const MIN_MOVEMENT_SPEED: f32 = 1.0;
/// Fastest setting of the movement-speed control.
pub const MAX_MOVEMENT_SPEED: f32 = 4.0;
/// Slider granularity for [`MovementSpeed`].
pub const MOVEMENT_SPEED_STEP: f32 = 0.1;

/// How much faster than real time the drones fly, 1x–4x.
///
/// Applied by scaling the whole [`FlightLimits`] envelope
/// ([`FlightLimits::scaled`]) rather than by scaling `dt`. That distinction
/// matters: this speeds up *the drones*, not the simulation. Each drone's own
/// clock, the header cadence, and the mesh gossip all keep running in real
/// time, so turning this up genuinely stresses the tracking and reconnection
/// logic with faster-moving targets instead of just fast-forwarding everything
/// uniformly.
#[derive(Resource, Clone, Copy, Debug)]
pub struct MovementSpeed(pub f32);

impl Default for MovementSpeed {
    fn default() -> Self {
        Self(MIN_MOVEMENT_SPEED)
    }
}

impl MovementSpeed {
    /// The flight envelope at this setting, in the world's km units.
    pub fn limits_km(&self) -> FlightLimits {
        FlightLimits::default().scaled(self.0).in_km()
    }
}

// ─── Patrol volume ────────────────────────────────────────────────────────────

/// How far inside the world's outer wall a drone is allowed to fly, km.
///
/// This is what makes the patrol volume *smaller* than the world: the drones
/// roam a box inset by this much on every side, so there is always a margin
/// of empty world between the formation and the edge of the map.
pub const BOUNDARY_MARGIN_KM: f32 = 3.0;

/// Closest two drones are allowed to get to each other, km.
///
/// Used twice: the initial scatter guarantees it, and [`drift_navigate`]
/// maintains it in flight by rounding drones off one another.
pub const MIN_SEPARATION_KM: f32 = 1.5;

/// Furthest apart two drones may sit and still be able to link, km.
///
/// The radio reaches [`crate::drone::ANTENNA_RANGE_KM`] (3.52 km) on perfect
/// boresight, but a link right at that range has no margin at all — the 1°
/// beam would then demand aim accurate to ~0.07°, which no tracking loop
/// holds. Backing off to 3.0 km leaves ~1.4 dB, and with it ~0.34° of aim
/// tolerance, which the predictive aim does hold.
///
/// This is what sizes the formation. The patrol area is fixed by
/// [`BOUNDARY_MARGIN_KM`], so the drone *count* is whatever it takes to keep
/// every neighbour inside this spacing — see [`crate::world::DRONE_COUNT`].
pub const MAX_LINK_SPACING_KM: f32 = 3.0;

/// How often a drone re-rolls its drift direction — seconds on *that drone's
/// own clock*, not shared wall time. Each drone's clock starts at a random
/// offset and drifts independently, so the whole formation never re-rolls in
/// lockstep on the same frame.
pub const DRIFT_REROLL_SECS: f64 = 10.0;

/// Lowest a drone flies, km **above ground level**.
pub const MIN_ALTITUDE_AGL_KM: f32 = 0.02;

/// Highest a drone flies, km **above ground level** — 60 m.
///
/// Above ground, not above sea level: these are low-flying drones that follow
/// the terrain rather than holding a fixed altitude, so the ceiling has to be
/// measured from whatever is underneath them at the time. A fixed sea-level
/// band would put a drone 60 m up over a valley and underground over a ridge.
pub const MAX_ALTITUDE_AGL_KM: f32 = 0.06;

/// The region the drones are allowed to fly inside — the "target area".
///
/// Horizontally this is a box: the world inset by [`BOUNDARY_MARGIN_KM`] on
/// all four sides. Vertically it is *not* a box — the ceiling and floor ride
/// the terrain (see [`MAX_ALTITUDE_AGL_KM`]), so the volume is a slab draped
/// over the landscape rather than a cube floating above it. That is why the
/// vertical test takes a ground height instead of living in `min`/`max`.
#[derive(Resource, Clone, Copy, Debug)]
pub struct PatrolVolume {
    /// Horizontal south-west corner. `y` is unused.
    pub min: Vec3,
    /// Horizontal north-east corner. `y` is unused.
    pub max: Vec3,
}

impl Default for PatrolVolume {
    fn default() -> Self {
        Self::inset(crate::world::WORLD_SIZE, BOUNDARY_MARGIN_KM)
    }
}

impl PatrolVolume {
    /// The volume for a square world `world_size_km` across (centered on the
    /// origin), inset by `margin_km` horizontally.
    pub fn inset(world_size_km: f32, margin_km: f32) -> Self {
        let half = world_size_km / 2.0 - margin_km;
        Self { min: Vec3::new(-half, 0.0, -half), max: Vec3::new(half, 0.0, half) }
    }

    /// Horizontal edge lengths of the volume, km.
    pub fn span_km(&self) -> Vec2 {
        Vec2::new(self.max.x - self.min.x, self.max.z - self.min.z)
    }

    /// Is this point inside the horizontal footprint?
    pub fn contains_horizontally(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// The altitude band available over ground at height `ground_km`.
    pub fn altitude_band(ground_km: f32) -> (f32, f32) {
        (ground_km + MIN_ALTITUDE_AGL_KM, ground_km + MAX_ALTITUDE_AGL_KM)
    }

    /// Is this point inside the footprint *and* within the AGL band over the
    /// ground beneath it?
    pub fn contains(&self, point: Vec3, ground_km: f32) -> bool {
        let (floor, ceiling) = Self::altitude_band(ground_km);
        self.contains_horizontally(point) && point.y >= floor && point.y <= ceiling
    }
}

// ─── Random drift ─────────────────────────────────────────────────────────────

/// The direction this drone is currently flying, plus when it next picks a new
/// one.
///
/// The direction is the drone's whole navigation intent — there is no waypoint
/// and no plan. It flies this way until something interrupts it: a neighbour
/// to round off, the patrol wall, or the [`DRIFT_REROLL_SECS`] timer.
#[derive(Component, Debug)]
pub struct DriftVector {
    /// Unit vector, world axes. Y is up.
    pub direction: Vec3,
    /// Deadline on this drone's own clock for the next re-roll.
    pub next_reroll_at: f64,
    /// This drone's private RNG stream, so each one re-rolls independently and
    /// the run is still reproducible from the spawn seed.
    pub rng: u64,
}

impl DriftVector {
    /// A drone starting at `clock_now` on its own clock, with `seed` driving
    /// its private direction stream.
    pub fn seeded(seed: u64, clock_now: f64) -> Self {
        let mut rng = splitmix64(seed ^ 0x5851_f42d_4c95_7f2d);
        let direction = random_direction(&mut rng);
        Self { direction, next_reroll_at: clock_now + DRIFT_REROLL_SECS, rng }
    }
}

/// One SplitMix64 round. Navigation keeps its own copy rather than reaching
/// into `networking`'s: a drone's flight path must not depend on the comms
/// stack's internals, and this is four lines.
fn splitmix64(seed: u64) -> u64 {
    let mut z = seed.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// Next value from `state`, as a float in `[0, 1)`.
fn next_unit(state: &mut u64) -> f32 {
    *state = splitmix64(*state);
    // Top 24 bits over 2^24 — every value is exactly representable in f32, so
    // this can't round up to 1.0 the way a /u64::MAX division can.
    (*state >> 40) as f32 / (1u64 << 24) as f32
}

/// How steep a random heading may be, as a vertical-to-horizontal ratio.
///
/// Deliberately tiny. The drones fly a 40 m band hugging the terrain
/// ([`MIN_ALTITUDE_AGL_KM`]..[`MAX_ALTITUDE_AGL_KM`]) while covering kilometers
/// horizontally, so anything steeper just pins against the band's ceiling or
/// floor and gets clamped away. At 0.05 a drone crosses its whole altitude
/// band over ~800 m of ground track, which is a visible climb without being a
/// pointless one.
pub const MAX_DRIFT_PITCH: f32 = 0.05;

/// A random unit heading: uniform in azimuth, deliberately shallow in pitch
/// (see [`MAX_DRIFT_PITCH`]).
pub fn random_direction(rng: &mut u64) -> Vec3 {
    let azimuth = next_unit(rng) * std::f32::consts::TAU;
    let vertical = (next_unit(rng) * 2.0 - 1.0) * MAX_DRIFT_PITCH;
    Vec3::new(azimuth.sin(), vertical, azimuth.cos()).normalize()
}

/// Re-roll each drone's drift direction every [`DRIFT_REROLL_SECS`] **on that
/// drone's own clock**.
///
/// Gating on `DroneClock` rather than on shared `Time` is the point: the
/// clocks start at random offsets and drift apart, so ten drones on a 10 s
/// cadence re-roll at ten different moments. Keying off `Time` would have the
/// entire formation change direction on the same frame.
pub fn reroll_drift_vectors(mut drones: Query<(&DroneClock, &mut DriftVector)>) {
    for (clock, mut drift) in &mut drones {
        if clock.now < drift.next_reroll_at {
            continue;
        }
        drift.direction = random_direction(&mut drift.rng);
        // Advance from the deadline, not from `now`, so the cadence doesn't
        // creep later every time a frame lands just past it.
        drift.next_reroll_at += DRIFT_REROLL_SECS;
        // If a long stall left the new deadline still in the past, catch up in
        // one step instead of re-rolling every frame until it overtakes.
        if drift.next_reroll_at <= clock.now {
            drift.next_reroll_at = clock.now + DRIFT_REROLL_SECS;
        }
    }
}

/// Turn `heading` so it rounds off any peer inside [`MIN_SEPARATION_KM`],
/// instead of flying into it.
///
/// For each peer the drone is *closing on* (one behind or abeam is not in the
/// way), the "toward the peer" component is projected out of the heading. What
/// remains is the tangent to the circle around them — so the drone goes
/// *around*, rather than stopping dead or reversing. How much of that tangent
/// is taken scales with how far inside the separation ring the peer already
/// is, so a peer at the very edge barely perturbs the course.
pub fn deflect_around_peers(self_pos: Vec3, heading: Vec3, peers: &[Vec3]) -> Vec3 {
    let mut steer = heading;
    let mut deflected = false;

    for &peer in peers {
        let to_peer = peer - self_pos;
        let distance = to_peer.length();
        if distance >= MIN_SEPARATION_KM || distance <= f32::EPSILON {
            continue;
        }
        let bearing = to_peer / distance;
        let closing = steer.dot(bearing);
        if closing <= 0.0 {
            // Already pointed away from them — they are not in the way.
            continue;
        }

        // 0 at the separation ring, 1 at contact.
        let urgency = (1.0 - distance / MIN_SEPARATION_KM).clamp(0.0, 1.0);

        let mut tangent = steer - bearing * closing;
        if tangent.length_squared() <= 1e-12 {
            // Dead-on: the projection leaves nothing, so no tangent falls out
            // of it and one has to be chosen. Take a horizontal one, so the
            // drone sidesteps rather than trying to climb over and running
            // straight into the (much lower) climb-rate limit.
            tangent = bearing.cross(Vec3::Y);
            if tangent.length_squared() <= 1e-12 {
                // Peer is directly above or below: any horizontal direction
                // rounds it off equally well. Fixed, so runs stay reproducible.
                tangent = Vec3::X;
            }
        }
        steer = steer.lerp(tangent.normalize(), urgency);
        deflected = true;
    }

    if deflected && steer.length_squared() > 1e-12 {
        steer.normalize()
    } else {
        heading
    }
}

// ─── Lateral speed cap (antenna tracking limit) ───────────────────────────────

/// How often the antenna tracking loop updates, Hz.
pub const TRACKING_UPDATE_HZ: f32 = 30.0;

/// The `1.8 · 0.25 · 0.07` of the lateral-speed law below.
///
/// The 1.8 already has the factor of ½ folded into it — it must **not** be
/// halved again.
pub const LATERAL_SPEED_COEFF: f32 = 1.8 * 0.25 * 0.07;

/// The fastest a drone may move *sideways* relative to a peer at
/// `distance_km`, given a tracking loop running at `update_rate_hz`:
///
/// ```text
/// v = 1.8 · (0.25 · 0.07 · d) · f
/// ```
///
/// Sideways motion is what costs an antenna its lock. Radial motion — straight
/// toward or away from the peer — barely moves the bearing at all, but lateral
/// motion sweeps the target across the beam at an angular rate of `v / d`. The
/// tracking loop can only correct so much per update, so the tolerable lateral
/// speed scales with **both** the range (a distant target subtends less angle
/// for the same sideways travel) and the update rate (more corrections per
/// second, more angular movement absorbed).
///
/// Range is in kilometers and the result is meters/second — at the default
/// 30 Hz that puts a peer 5 km out at ~4.7 m/s of lateral budget, and one at
/// the 3 km separation floor at ~2.8 m/s, both comfortably inside the 15 m/s
/// cruise cap. Closing on a peer is what tightens the leash.
pub fn max_lateral_speed_mps(distance_km: f32, update_rate_hz: f32) -> f32 {
    LATERAL_SPEED_COEFF * distance_km.max(0.0) * update_rate_hz.max(0.0)
}

/// Hold `velocity`'s sideways component relative to `bearing` under
/// `max_lateral`, leaving the along-bearing component untouched.
///
/// `bearing` must be a unit vector pointing from the drone at the peer it is
/// tracking. The velocity is split into the part along that bearing (radial —
/// free, it barely moves the antenna) and the part across it (lateral —
/// capped). Only the lateral part is scaled down, so a drone told to slow
/// sideways still closes or opens range at full speed.
pub fn clamp_lateral_speed(velocity: Vec3, bearing: Vec3, max_lateral: f32) -> Vec3 {
    let radial = bearing * velocity.dot(bearing);
    let lateral = velocity - radial;
    let lateral_speed = lateral.length();
    if lateral_speed <= max_lateral || lateral_speed <= f32::EPSILON {
        return velocity;
    }
    radial + lateral * (max_lateral / lateral_speed)
}

/// Distance needed to brake from full cruise to a stop, in whatever length
/// unit `limits` is expressed in. `v² / 2a`.
pub fn braking_distance(limits: &FlightLimits) -> f32 {
    if limits.max_accel_mps2 <= 0.0 {
        return 0.0;
    }
    limits.max_speed_mps * limits.max_speed_mps / (2.0 * limits.max_accel_mps2)
}

/// Fly every non-recovering drone along its drift vector.
///
/// Two things interrupt the drift, and they are deliberately different:
///
///   - **Another drone in the way** → round it off. A peer is an obstacle to
///     be gone around, so the heading is deflected tangentially and the drone
///     keeps flying (see [`deflect_around_peers`]).
///   - **The patrol wall** → stop. There is nothing on the other side to go
///     around *to*, so the drone brakes to a hold at the boundary and waits
///     there. It is not trapped: the next [`DRIFT_REROLL_SECS`] re-roll will
///     eventually hand it a heading pointing back inside.
///
/// A drone with **no live link holds position.** Flight is conditional on
/// being connected: a drone that has lost every peer has no business wandering
/// further from the mesh, so it brakes to a stop and waits to be reacquired
/// (`tracking`/`seeking` keep working the antennas while it sits). The one
/// exception is a drone in [`RecoveryState::Recovering`] — it is *deliberately*
/// flying while disconnected, back toward where it last had contact, and
/// [`crate::recovery::run_recovery`] owns its velocity for exactly that reason.
///
/// Drones in recovery are therefore skipped entirely; two systems writing the
/// same velocity field would fight.
///
/// Like [`crate::recovery::run_recovery`], this only writes
/// `DroneKinematics::velocity`; `factories::movement::apply_velocity`
/// integrates it, so there is exactly one integration per frame.
#[allow(clippy::type_complexity)] // Bevy queries describe the component access contract.
pub fn drift_navigate(
    time: Res<Time>,
    volume: Res<PatrolVolume>,
    speed: Res<MovementSpeed>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
    mut drones: Query<
        (Entity, &Transform, &DriftVector, &LinkSet, &RecoveryState, &mut DroneKinematics),
        With<Drone>,
    >,
    positions: Query<(Entity, &Transform), With<Drone>>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let limits = speed.limits_km();
    // How far ahead to look for the wall: a full braking distance, so the
    // drone starts stopping in time to come to rest *at* the edge rather than
    // sailing past it and needing to be shoved back inside.
    let lookahead_km = braking_distance(&limits).max(limits.max_speed_mps * dt);

    // Snapshot first — the loop below needs mutable access to each drone in
    // turn, so it can't also hold an immutable borrow of the whole set.
    let others: Vec<(Entity, Vec3)> =
        positions.iter().map(|(entity, t)| (entity, t.translation)).collect();

    for (self_entity, transform, drift, links, recovery, mut kin) in &mut drones {
        if matches!(recovery, RecoveryState::Recovering { .. }) {
            continue;
        }
        let self_pos = transform.translation;

        // No link, no flight. Brake to a hold and wait to be reacquired rather
        // than drifting further out of reach of the mesh.
        if links.connected.is_empty() {
            let mut state = DroneState {
                position: self_pos,
                velocity: kin.velocity,
                heading_deg: kin.heading_deg,
            };
            navigate(&mut state, self_pos, &limits, dt);
            kin.velocity = state.velocity;
            kin.heading_deg = state.heading_deg;
            continue;
        }

        let peers: Vec<Vec3> = others
            .iter()
            .filter(|(entity, _)| *entity != self_entity)
            .map(|(_, pos)| *pos)
            .collect();

        let heading = deflect_around_peers(self_pos, drift.direction, &peers);

        // Aim a braking distance ahead along the (possibly deflected) heading.
        // Outside the patrol wall there is nothing to go around *to*, so the
        // drone targets its own position and `navigate` brakes it to a stop at
        // the acceleration limit — a real stop, not a teleport to rest.
        //
        // The altitude is a separate matter: it is clamped, never stopped for.
        // Holding station because the ground rose underneath would be absurd,
        // so the target is simply pulled back into the AGL band over whichever
        // point the drone is heading for. That clamp is also what makes the
        // formation terrain-following — as the ground climbs ahead, so does
        // the target.
        let ahead = self_pos + heading * lookahead_km;
        let level = if volume.contains_horizontally(ahead) { ahead } else { self_pos };
        let ground = terrain.height_at(level.x, level.z);
        let (floor, ceiling) = PatrolVolume::altitude_band(ground);
        let target = Vec3::new(level.x, level.y.clamp(floor, ceiling), level.z);

        let mut state = DroneState {
            position: self_pos,
            velocity: kin.velocity,
            heading_deg: kin.heading_deg,
        };
        navigate(&mut state, target, &limits, dt);

        // Don't outrun the antenna. The binding peer is the *closest* one:
        // the lateral budget grows with range, so the nearest tracked drone is
        // always the one whose lock breaks first.
        if let Some(nearest) = peers
            .iter()
            .map(|peer| *peer - self_pos)
            .filter(|offset| offset.length() > f32::EPSILON)
            .min_by(|a, b| a.length().total_cmp(&b.length()))
        {
            let distance_km = nearest.length();
            // The law is stated in m/s over km of range; the world is km, so
            // the cap converts before it meets a km/s velocity.
            let max_lateral_km_s =
                max_lateral_speed_mps(distance_km, TRACKING_UPDATE_HZ) / 1000.0;
            state.velocity =
                clamp_lateral_speed(state.velocity, nearest / distance_km, max_lateral_km_s);
        }

        kin.velocity = state.velocity;
        kin.heading_deg = state.heading_deg;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Repeatedly navigating toward a far-away target should never exceed
    /// the configured max speed, even after many ticks of acceleration.
    #[test]
    fn never_exceeds_max_speed() {
        let limits = FlightLimits::default();
        let mut state = DroneState::default();
        let target = Vec3::new(1000.0, 0.0, 0.0);

        for _ in 0..2000 {
            navigate(&mut state, target, &limits, 0.02);
            let speed = Vec3::new(state.velocity.x, 0.0, state.velocity.z).length();
            assert!(
                speed <= limits.max_speed_mps + 1e-3,
                "speed {speed} exceeded max_speed_mps {}",
                limits.max_speed_mps
            );
        }
    }

    /// set_max_speed should actually take effect on subsequent ticks.
    #[test]
    fn set_max_speed_is_respected() {
        let mut limits = FlightLimits::default();
        limits.set_max_speed(2.0);
        let mut state = DroneState::default();
        let target = Vec3::new(1000.0, 0.0, 0.0);

        for _ in 0..2000 {
            navigate(&mut state, target, &limits, 0.02);
            let speed = Vec3::new(state.velocity.x, 0.0, state.velocity.z).length();
            assert!(speed <= 2.0 + 1e-3, "speed {speed} exceeded overridden cap 2.0");
        }
    }

    /// A drone told to go straight to its own current position shouldn't
    /// accelerate off in some arbitrary direction.
    #[test]
    fn holds_position_when_already_at_target() {
        let limits = FlightLimits::default();
        let mut state = DroneState::default();
        navigate(&mut state, Vec3::ZERO, &limits, 0.02);
        assert_eq!(state.velocity, Vec3::ZERO);
        assert_eq!(state.position, Vec3::ZERO);
    }

    /// Approaching a nearby target should brake rather than fly past it at
    /// full speed — after enough ticks the drone should come to rest at (or
    /// very near) the target, not oscillate past it.
    #[test]
    fn settles_at_target_without_overshoot_oscillation() {
        let limits = FlightLimits::default();
        let mut state = DroneState::default();
        let target = Vec3::new(5.0, 0.0, 0.0);

        for _ in 0..500 {
            navigate(&mut state, target, &limits, 0.02);
        }

        assert!((state.position - target).length() < 0.1);
        assert!(state.velocity.length() < 0.1);
    }

    /// Climb and descent are governed separately and asymmetrically — a quad
    /// descends slower than it climbs (vortex-ring-state safety margin).
    #[test]
    fn climbs_faster_than_it_descends() {
        let limits = FlightLimits::default();

        // Cruise straight up, far target so it reaches the vertical limit.
        let mut up = DroneState::default();
        for _ in 0..200 {
            navigate(&mut up, Vec3::new(0.0, 1000.0, 0.0), &limits, 0.02);
        }
        // Cruise straight down from altitude.
        let mut down = DroneState { position: Vec3::new(0.0, 1000.0, 0.0), ..Default::default() };
        for _ in 0..200 {
            navigate(&mut down, Vec3::ZERO, &limits, 0.02);
        }

        assert!((up.velocity.y - limits.max_climb_mps).abs() < 0.2, "climb {}", up.velocity.y);
        assert!(
            (down.velocity.y + limits.max_descend_mps).abs() < 0.2,
            "descend {}",
            down.velocity.y
        );
        assert!(up.velocity.y > -down.velocity.y, "climb should be faster than descent");
    }

    // ─── Drift, separation, and the patrol wall ───────────────────────────

    /// A peer dead ahead is rounded off, not stopped at and not reversed away
    /// from — the drone keeps making progress past it.
    #[test]
    fn peer_dead_ahead_is_rounded_off() {
        let heading = Vec3::Z;
        let peer = Vec3::new(0.0, 0.0, MIN_SEPARATION_KM * 0.4);
        let steer = deflect_around_peers(Vec3::ZERO, heading, &[peer]);

        let bearing = peer.normalize();
        assert!(
            steer.dot(bearing) < heading.dot(bearing),
            "should turn away from the peer's bearing"
        );
        assert!(steer.dot(bearing) > -0.5, "should go around, not reverse: {steer:?}");
        assert!((steer.length() - 1.0).abs() < 1e-4, "must stay a unit heading");
    }

    /// A peer already behind the drone isn't in the way, so the heading is
    /// returned untouched.
    #[test]
    fn peer_behind_does_not_deflect() {
        let heading = Vec3::Z;
        let behind = Vec3::new(0.0, 0.0, -MIN_SEPARATION_KM * 0.5);
        assert_eq!(deflect_around_peers(Vec3::ZERO, heading, &[behind]), heading);
    }

    /// A peer beyond the separation ring is simply not close enough to matter.
    #[test]
    fn peer_outside_separation_ring_is_ignored() {
        let heading = Vec3::Z;
        let far = Vec3::new(0.0, 0.0, MIN_SEPARATION_KM * 2.0);
        assert_eq!(deflect_around_peers(Vec3::ZERO, heading, &[far]), heading);
    }

    /// Flying at the patrol wall brings the drone to a stop rather than
    /// letting it leave the volume — and it stops *inside*.
    #[test]
    fn boundary_stops_the_drone_inside_the_volume() {
        let volume = PatrolVolume::inset(20.0, BOUNDARY_MARGIN_KM);
        let limits = FlightLimits::default().in_km();
        let lookahead = braking_distance(&limits);

        // Start well inside, headed straight at the +X wall.
        let mut state = DroneState {
            position: Vec3::new(volume.max.x - lookahead * 2.0, 1.0, 0.0),
            ..Default::default()
        };
        let heading = Vec3::X;

        for _ in 0..4000 {
            let ahead = state.position + heading * lookahead;
            let target = if volume.contains_horizontally(ahead) {
                ahead
            } else {
                state.position
            };
            navigate(&mut state, target, &limits, 1.0 / 60.0);
            assert!(
                state.position.x <= volume.max.x,
                "left the patrol volume at x = {}",
                state.position.x
            );
        }
        assert!(state.velocity.length() < 1e-3, "should have come to rest at the wall");
    }

    /// The re-roll gates on the drone's *own* clock. Two drones whose clocks
    /// start a half-period apart must not change direction on the same tick —
    /// that's the whole reason this doesn't use shared `Time`.
    #[test]
    fn reroll_follows_each_drones_own_clock() {
        use bevy::ecs::system::RunSystemOnce;

        let mut world = World::new();
        // Early clock: deadline not reached. Late clock: past its deadline.
        let early = world
            .spawn((DroneClock { now: 5.0 }, DriftVector::seeded(1, 0.0)))
            .id();
        let late = world
            .spawn((DroneClock { now: 12.0 }, DriftVector::seeded(2, 0.0)))
            .id();

        let before_early = world.get::<DriftVector>(early).unwrap().direction;
        let before_late = world.get::<DriftVector>(late).unwrap().direction;

        world.run_system_once(reroll_drift_vectors).unwrap();

        assert_eq!(
            world.get::<DriftVector>(early).unwrap().direction,
            before_early,
            "a drone whose own clock hasn't reached the deadline must not re-roll"
        );
        assert_ne!(
            world.get::<DriftVector>(late).unwrap().direction,
            before_late,
            "a drone past its own deadline must re-roll"
        );
    }

    /// A re-roll that lands far past the deadline catches up in one step
    /// instead of firing again every frame until it overtakes.
    #[test]
    fn reroll_catches_up_after_a_long_stall() {
        use bevy::ecs::system::RunSystemOnce;

        let mut world = World::new();
        let stalled = world
            .spawn((DroneClock { now: 500.0 }, DriftVector::seeded(3, 0.0)))
            .id();

        world.run_system_once(reroll_drift_vectors).unwrap();

        let next = world.get::<DriftVector>(stalled).unwrap().next_reroll_at;
        assert!(next > 500.0, "deadline should be pushed ahead of now, got {next}");
        assert!(next <= 500.0 + DRIFT_REROLL_SECS, "should catch up in one step, got {next}");
    }

    /// Random headings are unit vectors with a deliberately shallow climb, so
    /// the airframe can actually fly them.
    #[test]
    fn random_directions_are_flyable() {
        let mut rng = 0xabcd_ef01;
        for _ in 0..500 {
            let direction = random_direction(&mut rng);
            assert!((direction.length() - 1.0).abs() < 1e-4);
            assert!(direction.y.abs() <= 0.26, "pitch too steep: {}", direction.y);
        }
    }

    // ─── Lateral speed cap ────────────────────────────────────────────────

    /// The cap is exactly `1.8 · (0.25 · 0.07 · d) · f` — the 1.8 already has
    /// the ½ folded in and must not be halved again.
    #[test]
    fn lateral_cap_matches_the_stated_law() {
        for (distance, hz) in [(1.0, 30.0), (5.0, 30.0), (3.0, 60.0)] {
            let expected = 1.8 * (0.25 * 0.07 * distance) * hz;
            assert!(
                (max_lateral_speed_mps(distance, hz) - expected).abs() < 1e-4,
                "d={distance} f={hz}"
            );
        }
        // A peer 5 km out at 30 Hz gets ~4.7 m/s of sideways budget.
        assert!((max_lateral_speed_mps(5.0, 30.0) - 4.725).abs() < 1e-3);
    }

    /// The cap grows with range: a closer peer is a tighter leash. This is why
    /// `drift_navigate` measures against the *nearest* peer.
    #[test]
    fn lateral_cap_tightens_as_a_peer_closes() {
        let near = max_lateral_speed_mps(1.0, TRACKING_UPDATE_HZ);
        let far = max_lateral_speed_mps(8.0, TRACKING_UPDATE_HZ);
        assert!(near < far, "near {near} should be tighter than far {far}");
    }

    /// Only the sideways component is capped — closing on a peer head-on is
    /// left at full speed, because radial motion barely moves the bearing.
    #[test]
    fn lateral_clamp_leaves_radial_speed_alone() {
        let bearing = Vec3::X;
        let velocity = Vec3::new(10.0, 0.0, 0.0); // purely radial
        assert_eq!(clamp_lateral_speed(velocity, bearing, 0.001), velocity);
    }

    /// Sideways motion above the cap is scaled back to exactly the cap, with
    /// the radial part carried through untouched.
    #[test]
    fn lateral_clamp_limits_only_the_sideways_part() {
        let bearing = Vec3::X;
        let velocity = Vec3::new(3.0, 0.0, 9.0); // 3 radial, 9 lateral
        let capped = clamp_lateral_speed(velocity, bearing, 2.0);

        assert!((capped.dot(bearing) - 3.0).abs() < 1e-4, "radial must survive");
        let lateral = capped - bearing * capped.dot(bearing);
        assert!((lateral.length() - 2.0).abs() < 1e-4, "lateral should sit at the cap");
    }

    /// A zero or negative timestep is a no-op — nothing moves.
    #[test]
    fn zero_dt_is_a_noop() {
        let limits = FlightLimits::default();
        let mut state = DroneState { velocity: Vec3::new(1.0, 0.0, 0.0), ..Default::default() };
        let before = state;
        navigate(&mut state, Vec3::new(100.0, 0.0, 0.0), &limits, 0.0);
        assert_eq!(state.position, before.position);
        assert_eq!(state.velocity, before.velocity);
    }
}
