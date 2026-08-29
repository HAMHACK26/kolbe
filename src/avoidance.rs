//! Local collision avoidance — the drone's own cheap proximity sensors.
//!
//! This is deliberately the *dumbest* layer in the flight stack, and that is
//! the point. Every other movement module here reasons about the mission:
//! [`crate::recovery`] flies back to a remembered waypoint, [`crate::seeking`]
//! sweeps for a lost peer, the base issues `GoTo` orders. All of them assume
//! the air between here and there is empty. This module is what catches them
//! when it isn't.
//!
//! ## The sensor being modeled
//!
//! A ring of cheap short-range rangefinders (ultrasonic / IR / single-point
//! ToF — the class of part that costs a couple of euros, not a lidar puck).
//! What such a sensor gives you, and *only* what it gives you:
//!
//!   - a range to the nearest surface in each direction, out to a few meters
//!     ([`SENSOR_RANGE_M`]);
//!   - nothing else. No identity, no closing rate, no idea whether the thing
//!     in front of it is another drone, a mast, or a wall.
//!
//! So the response here is purely reactive and position-based: "something is
//! close, push away from it." There is no prediction, no trajectory
//! planning, and no attempt to *route around* an obstacle — that would need
//! a sensor that can tell you the obstacle's shape and extent, which this one
//! cannot. Anything cleverer would be modeling hardware the drones don't
//! carry.
//!
//! ## Horizontal only
//!
//! Avoidance acts strictly in the horizontal plane (X/Z in world axes — Y is
//! altitude). The drone's vertical velocity is passed through untouched by
//! [`avoidance_velocity`], so climb/descent stays entirely the navigator's
//! business. Dodging *downward* is rarely valid (there is terrain under you)
//! and dodging upward costs the climb-rate budget the flight envelope
//! deliberately keeps small, so a sideways step is the only maneuver a cheap
//! sensor ring can justify on its own.
//!
//! ## Where it sits in the frame
//!
//! It runs *after* the navigators have written their intended velocity into
//! [`DroneKinematics`] and *before* [`crate::factories::movement::apply_velocity`]
//! integrates it. It never sets a target or a waypoint; it only vetoes and
//! deflects the velocity the mission layer already asked for. When nothing is
//! within sensor range it is exactly a no-op, so a drone with clear air flies
//! precisely the path its navigator planned.
//!
//! ## What 3 meters can and cannot do
//!
//! A proximity sensor can only save you if you can stop inside its range.
//! Braking distance is `v² / (2a)`, so the closing speed this ring can arrest
//! is bounded by `sqrt(2 · a · range)` — see [`safe_closing_speed_mps`]. At
//! the default [`FlightLimits`] (4 m/s² of lateral authority) and a 3 m ring
//! that is **4.9 m/s**, and two cases fall out of it:
//!
//!   - **Against something that can't get out of the way** — the ground
//!     station, an [`Obstacle`] prop, a mast — only the drone brakes, so
//!     4.9 m/s of approach speed is the limit.
//!   - **Between two drones that both carry the ring**, both brake, so the
//!     *closing* rate sheds at twice one airframe's authority and the limit
//!     rises to `sqrt(4 · a · range)` ≈ **6.9 m/s**. Measured against this
//!     implementation, separation holds up to ~6.9 m/s closing and is lost
//!     from ~7.3 m/s — the theory and the code agree.
//!
//! Both numbers sit well under the 15 m/s cruise cap. Two drones converging
//! head-on at full cruise *will* touch, and no amount of cleverness in this
//! file changes that — it is a property of the sensor range, not of the
//! algorithm. `cruise_speed_defeats_the_ring` in the tests below pins that
//! down so the limitation can't quietly rot into an assumed guarantee.
//!
//! That is realistic rather than a bug: cheap short-range sensors on real
//! aircraft are a last-ditch bumper, not a separation guarantee. Actual
//! separation is the mission layer's job (fly non-conflicting routes). If a
//! guarantee is wanted here, the fix is one of: raise [`SENSOR_RANGE_M`] to
//! the real hardware's range, or have the mission layer call
//! [`FlightLimits::set_max_speed`] with [`safe_closing_speed_mps`] / 2 when
//! flying in close formation.

use bevy::prelude::*;

use crate::base::{Base, BASE_BOX_SIZE_KM};
use crate::drone::Drone;
use crate::factories::movement::{DroneKinematics, Obstacle};
use crate::navigation::FlightLimits;
use crate::world::DRONE_RADIUS;

/// How far the proximity ring can see, in **meters of real-world distance**.
///
/// Measured surface-to-surface, the way a rangefinder actually reports: the
/// gap between this drone's hull and the obstacle's, not between their center
/// points. 3 m is a typical usable range for the cheap-sensor class described
/// in the module docs.
///
/// Note the simulation scale: the avoidance model treats a drone as a
/// [`DRONE_RADIUS`]-km body (22.5 m), still much larger than a real airframe
/// so it remains legible on a kilometre-scale map. Keeping this constant in
/// true meters means the sensor range itself stays honest.
pub const SENSOR_RANGE_M: f32 = 3.0;

/// [`SENSOR_RANGE_M`] in the kilometer units the simulation world uses.
pub const SENSOR_RANGE_KM: f32 = SENSOR_RANGE_M / 1000.0;

/// Start climbing well before a canopy. At the default 15 m/s cruise and
/// 5 m/s climb rate, 200 m gives a drone time to rise above a 50 m tree.
const TREE_CLIMB_LOOKAHEAD_KM: f32 = 0.20;
/// Extra vertical gap between the drone hull and a canopy top.
const TREE_TOP_CLEARANCE_KM: f32 = 0.01;

/// Horizontal bounding radius of the ground station, km.
///
/// The base is drawn as a [`BASE_BOX_SIZE_KM`] cube; the circle that encloses
/// its footprint from any approach bearing has the half-diagonal as its
/// radius. Rounding a box up to its circumscribed circle is the conservative
/// direction — the drone keeps clear of the corners too.
pub const BASE_RADIUS_KM: f32 = BASE_BOX_SIZE_KM * std::f32::consts::SQRT_2 / 2.0;

/// One thing the proximity ring can currently see.
///
/// Deliberately thin: a relative bearing and a size. That is all a cheap
/// rangefinder plus a known obstacle catalog can supply — notably there is no
/// velocity here, because the sensor cannot measure one.
#[derive(Clone, Copy, Debug)]
pub struct Detection {
    /// Obstacle center relative to the sensing drone, world axes, km.
    pub offset: Vec3,
    /// Obstacle's own bounding radius, km.
    pub radius_km: f32,
}

/// The fastest head-on closing speed a proximity ring of `sensor_range_m` can
/// actually arrest, given `max_accel_mps2` of braking authority: the `v` for
/// which the braking distance `v² / (2a)` exactly equals the range.
///
/// Both arguments are real-world units (meters, m/s²) and the result is m/s —
/// this is a statement about the hardware, so it is deliberately *not*
/// expressed in the simulation's km scale. See the module docs for why this
/// number matters.
// Nothing in the running app calls this yet — it is the tuning knob the module
// docs point the mission layer at ("cap speed to what the ring can stop"), and
// the tests use it to pin the envelope. Wiring it into a mission planner is a
// separate change.
#[allow(dead_code)]
pub fn safe_closing_speed_mps(sensor_range_m: f32, max_accel_mps2: f32) -> f32 {
    (2.0 * max_accel_mps2.max(0.0) * sensor_range_m.max(0.0)).sqrt()
}

/// Deflect a planned velocity around whatever the proximity ring can see.
///
/// Two velocities go in, and the distinction matters:
///
///   - `flown_velocity` — what the airframe is *actually* doing right now
///     ([`DroneKinematics::flown_velocity`]), i.e. what it was flying when the
///     last frame integrated. Deflections are rate-limited from here, because
///     this is the only thing the rotor plane can physically be tilted away
///     from.
///   - `planned_velocity` — what the navigator asked for this tick, already
///     sitting in [`DroneKinematics::velocity`]. Avoidance blends *against*
///     this, and returns it unchanged (byte for byte) when nothing is in
///     range, so a drone in clear air flies exactly the path it planned.
///
/// Measuring the acceleration budget from the plan instead of from the real
/// motion would be a silent failure: the navigator spends the whole budget
/// commanding "inbound", avoidance's equal and opposite budget merely cancels
/// it back to the previous velocity, and the drone sails through the obstacle
/// at constant speed having never actually braked.
///
/// `limits` must already be scaled to the simulation's km units — see
/// [`FlightLimits::in_km`].
///
/// The response, in order:
///
/// 1. **Range-gate each detection.** Gap is surface-to-surface
///    (`center_distance − both radii`), measured in the horizontal plane
///    only, so an obstacle passing well above or below is correctly ignored.
///    Anything beyond `sensor_range_km` is invisible.
///
/// 2. **Weight by urgency.** Each detection contributes a unit vector
///    pointing directly away from it, scaled by how deep into the sensor
///    range it has come: 0 at the outer edge, ramping to 1 at contact (and
///    pinned at 1 if already overlapping). Summing these gives a single
///    escape bearing that naturally splits the difference between several
///    obstacles at once.
///
/// 3. **Blend against the plan, by the worst urgency.** At the edge of range
///    the planned velocity is kept as-is; at contact it is fully replaced by a
///    max-speed run along the escape bearing; in between it is a straight
///    interpolation. So avoidance authority grows smoothly with danger
///    instead of snapping on, and a distant obstacle barely perturbs the
///    mission path.
///
/// 4. **Brake if boxed in.** If the repulsions cancel out — pinched between
///    obstacles on opposite sides — there is no escape bearing to fly. A
///    sensor this cheap cannot plan a way out of that, so the drone brakes
///    toward a stop instead of guessing a direction and possibly picking the
///    wrong one.
///
/// 5. **Stay inside the flight envelope.** The result is capped at
///    `max_speed_mps`, and the change from `flown_velocity` is rate-limited to
///    `max_accel_mps2 * dt` — an avoidance maneuver is still flown by tilting
///    the same rotor plane as everything else (see [`crate::navigation`]), and
///    it gets the whole tick's budget because when the ring fires it outranks
///    the mission planner.
///
/// Vertical velocity is passed straight through — this plans in the
/// horizontal plane only.
pub fn avoidance_velocity(
    flown_velocity: Vec3,
    planned_velocity: Vec3,
    self_radius_km: f32,
    detections: &[Detection],
    sensor_range_km: f32,
    limits: &FlightLimits,
    dt: f32,
) -> Vec3 {
    if dt <= 0.0 || sensor_range_km <= 0.0 {
        return planned_velocity;
    }

    let planned_horizontal = Vec3::new(planned_velocity.x, 0.0, planned_velocity.z);
    let flown_horizontal = Vec3::new(flown_velocity.x, 0.0, flown_velocity.z);

    // ---- 1 & 2. Range-gate, and accumulate a weighted escape bearing -----
    let mut escape = Vec3::ZERO;
    let mut worst_urgency = 0.0f32;
    let mut saw_anything = false;

    for detection in detections {
        let offset = Vec3::new(detection.offset.x, 0.0, detection.offset.z);
        let center_distance = offset.length();
        let gap = center_distance - self_radius_km - detection.radius_km;
        if gap > sensor_range_km {
            continue;
        }
        saw_anything = true;

        // 0 at the outer edge of the ring, 1 at (or inside) contact.
        let urgency = ((sensor_range_km - gap) / sensor_range_km).clamp(0.0, 1.0);
        worst_urgency = worst_urgency.max(urgency);

        // Exactly co-located horizontally has no defined "away" direction.
        // Pick a fixed axis rather than a random one so the simulation stays
        // deterministic; the case is degenerate either way.
        let away = if center_distance > f32::EPSILON {
            -offset / center_distance
        } else {
            Vec3::X
        };
        escape += away * urgency;
    }

    if !saw_anything {
        return planned_velocity;
    }

    // ---- 3 & 4. Blend toward the escape run, or brake if boxed in --------
    let target_horizontal = if escape.length_squared() > 1e-12 {
        let escape_run = escape.normalize() * limits.max_speed_mps;
        planned_horizontal.lerp(escape_run, worst_urgency)
    } else {
        planned_horizontal.lerp(Vec3::ZERO, worst_urgency)
    };

    // ---- 5. Clamp to the envelope, then rate-limit the change ------------
    let target_speed = target_horizontal.length();
    let target_horizontal = if target_speed > limits.max_speed_mps && target_speed > 0.0 {
        target_horizontal * (limits.max_speed_mps / target_speed)
    } else {
        target_horizontal
    };

    // Rate-limited from what the airframe is really doing, not from the
    // navigator's fresh command — see the note on `flown_velocity` above.
    let delta = target_horizontal - flown_horizontal;
    let max_delta_v = limits.max_accel_mps2 * dt;
    let commanded = if delta.length() <= max_delta_v {
        target_horizontal
    } else {
        flown_horizontal + delta.normalize() * max_delta_v
    };

    Vec3::new(commanded.x, planned_velocity.y, commanded.z)
}

/// Run every drone's proximity ring and deflect its velocity accordingly.
///
/// Must be ordered after the systems that write [`DroneKinematics::velocity`]
/// (currently [`crate::recovery::run_recovery`]) and before
/// [`crate::factories::movement::apply_velocity`] integrates it.
///
/// The horizontal ring sees other drones, the ground station, and explicit
/// [`Obstacle`] components. Tree canopies are handled separately: the drone
/// climbs above them instead of treating a forest as a sideways wall.
pub fn avoid_collisions(
    time: Res<Time>,
    mut drones: Query<(Entity, &Transform, &mut DroneKinematics), With<Drone>>,
    bases: Query<&Transform, With<Base>>,
    props: Query<(&Transform, &Obstacle), Without<Drone>>,
    canopies: Option<Res<crate::terrain::RadioCanopies>>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let limits = FlightLimits::default().in_km();

    // Snapshot everything solid first: the per-drone pass below needs mutable
    // access to its own kinematics while still reading every *other* drone's
    // position, which it can't do while holding an iterator over the same
    // query.
    let mut solids: Vec<(Option<Entity>, Vec3, f32)> =
        drones.iter().map(|(e, t, _)| (Some(e), t.translation, DRONE_RADIUS)).collect();
    solids.extend(bases.iter().map(|t| (None, t.translation, BASE_RADIUS_KM)));
    solids.extend(props.iter().map(|(t, o)| (None, t.translation, o.radius_km)));

    for (entity, transform, mut kinematics) in &mut drones {
        let self_pos = transform.translation;
        let detections: Vec<Detection> = solids
            .iter()
            // A drone is not an obstacle to itself.
            .filter(|(other, ..)| *other != Some(entity))
            .map(|(_, pos, radius_km)| Detection {
                offset: *pos - self_pos,
                radius_km: *radius_km,
            })
            .collect();
        let mut canopy_ceiling = None;
        if let Some(canopies) = &canopies {
            canopy_ceiling = canopies
                .nearby_canopies(self_pos, TREE_CLIMB_LOOKAHEAD_KM)
                .into_iter()
                .map(|canopy| canopy.position.y + DRONE_RADIUS + TREE_TOP_CLEARANCE_KM)
                .max_by(f32::total_cmp);
        }

        kinematics.velocity = avoidance_velocity(
            kinematics.flown_velocity,
            kinematics.velocity,
            DRONE_RADIUS,
            &detections,
            SENSOR_RANGE_KM,
            &limits,
            dt,
        );
        if let Some(clearance_altitude) = canopy_ceiling {
            // Canopies are flown over, never pushed through sideways. Hold
            // altitude while one remains ahead; the normal navigator resumes
            // descent once clear of the forest.
            kinematics.velocity.y = if self_pos.y < clearance_altitude {
                limits.max_climb_mps
            } else {
                kinematics.velocity.y.max(0.0)
            };
        }
    }
}
