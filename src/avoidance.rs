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
/// Note the render scale: the simulation draws a drone as a
/// [`DRONE_RADIUS`]-km sphere (180 m) so it is visible on a 20 km map, which
/// makes the modeled body enormously larger than a real airframe. Keeping this
/// constant in true meters means the number stays honest — and because the gap
/// is measured surface-to-surface, avoidance still triggers exactly when the
/// drawn spheres are about to touch, which is what reads correctly on screen.
pub const SENSOR_RANGE_M: f32 = 3.0;

/// [`SENSOR_RANGE_M`] in the kilometer units the simulation world uses.
pub const SENSOR_RANGE_KM: f32 = SENSOR_RANGE_M / 1000.0;

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
/// What the ring can see: other drones, the ground station, and anything
/// carrying an [`Obstacle`] component. Terrain is not included — that is a
/// vertical concern, and `apply_velocity` already floors altitude.
pub fn avoid_collisions(
    time: Res<Time>,
    mut drones: Query<(Entity, &Transform, &mut DroneKinematics), With<Drone>>,
    bases: Query<&Transform, With<Base>>,
    props: Query<(&Transform, &Obstacle), Without<Drone>>,
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

        kinematics.velocity = avoidance_velocity(
            kinematics.flown_velocity,
            kinematics.velocity,
            DRONE_RADIUS,
            &detections,
            SENSOR_RANGE_KM,
            &limits,
            dt,
        );
    }
}

/// Runnable scenarios that print a flight trace, for watching the ring work.
///
/// The simulation's visuals can't show this: the orbit camera bottoms out at
/// a 5 km radius and drones are drawn as 180 m spheres, so a 3 m standoff is
/// a fraction of a pixel. The numbers are the only honest instrument at this
/// scale, so these print them.
///
/// They are `#[ignore]`d because their value is the output, not an assertion
/// (the real coverage is in `tests` below). Run them explicitly:
///
/// ```text
/// cargo test avoidance::demo -- --ignored --nocapture
/// ```
#[cfg(test)]
mod demo {
    use super::*;
    use crate::navigation::{navigate, DroneState};

    /// One thing in a scenario: a drone flying toward `waypoint`, or — with
    /// `waypoint: None` — a fixed obstacle that just sits there.
    struct Body {
        state: DroneState,
        waypoint: Option<Vec3>,
        radius_km: f32,
    }

    impl Body {
        fn drone(x_km: f32, z_km: f32, waypoint: Vec3) -> Self {
            Self {
                state: DroneState {
                    position: Vec3::new(x_km, 0.0, z_km),
                    ..Default::default()
                },
                waypoint: Some(waypoint),
                radius_km: DRONE_RADIUS,
            }
        }

        fn obstacle(x_km: f32, z_km: f32) -> Self {
            Self {
                state: DroneState {
                    position: Vec3::new(x_km, 0.0, z_km),
                    ..Default::default()
                },
                waypoint: None,
                radius_km: DRONE_RADIUS,
            }
        }

        fn speed_mps(&self) -> f32 {
            self.state.velocity.length() * 1000.0
        }
    }

    /// Two drones facing each other with `runway_km` of *clear air* between
    /// their hulls, each aimed at a waypoint far beyond the other.
    ///
    /// The runway is hull-to-hull, not center-to-center — bodies are 180 m in
    /// radius here, so placing them by center coordinate is an easy way to
    /// start a scenario already overlapping. It also has to be long enough for
    /// `navigate` to wind up to the speed cap under test, or the trace only
    /// ever shows a crawl.
    fn head_on_pair(runway_km: f32) -> Vec<Body> {
        let x = DRONE_RADIUS + runway_km / 2.0;
        vec![
            Body::drone(-x, 0.0, Vec3::new(1000.0, 0.0, 0.0)),
            Body::drone(x, 0.0, Vec3::new(-1000.0, 0.0, 0.0)),
        ]
    }

    /// Advance every mobile body one tick through the real pipeline: navigate,
    /// then let the ring veto, then integrate the vetoed velocity — the same
    /// order `run_recovery` → `avoid_collisions` → `apply_velocity` runs in.
    fn step(bodies: &mut [Body], limits: &FlightLimits, dt: f32) {
        let snapshot: Vec<(Vec3, f32)> =
            bodies.iter().map(|b| (b.state.position, b.radius_km)).collect();

        // Iterating mutably is safe alongside `snapshot` because that is an
        // independent copy taken before the loop — every body reacts to where
        // the others were at the top of the tick, not to partially-updated
        // positions.
        for (index, body) in bodies.iter_mut().enumerate() {
            let Some(waypoint) = body.waypoint else {
                continue;
            };
            let position = body.state.position;
            let flown = body.state.velocity;

            navigate(&mut body.state, waypoint, limits, dt);

            let detections: Vec<Detection> = snapshot
                .iter()
                .enumerate()
                .filter(|(other, _)| *other != index)
                .map(|(_, (pos, radius_km))| Detection {
                    offset: *pos - position,
                    radius_km: *radius_km,
                })
                .collect();

            body.state.velocity = avoidance_velocity(
                flown,
                body.state.velocity,
                body.radius_km,
                &detections,
                SENSOR_RANGE_KM,
                limits,
                dt,
            );
            body.state.position = position + body.state.velocity * dt;
        }
    }

    /// Surface-to-surface gap between the first two bodies, meters.
    fn gap_m(bodies: &[Body]) -> f32 {
        ((bodies[1].state.position - bodies[0].state.position).length()
            - bodies[0].radius_km
            - bodies[1].radius_km)
            * 1000.0
    }

    /// Run a scenario to a standstill (or to contact) and print the trace.
    ///
    /// Rows are sampled coarsely on the long approach and every tick once the
    /// ring is within reach, so the interesting part is at full resolution
    /// without thousands of lines of cruise.
    fn run(title: &str, detail: &str, bodies: &mut [Body], limits: &FlightLimits) {
        let dt = 0.02;
        let ring_m = SENSOR_RANGE_M;

        println!("\n=== {title} ===");
        println!("{detail}");
        // Column widths match the row format below: {:>7} then four {:>9},
        // separated by two spaces.
        println!("   t(s)     gap(m)     A(m/s)     B(m/s)     A z(m)  ring");

        let mut last_printed = f32::NEG_INFINITY;
        let mut min_gap = f32::MAX;
        let mut contact = false;
        let mut was_engaged = false;
        let mut rows = 0usize;

        for tick in 0..8000 {
            let t = tick as f32 * dt;
            let gap = gap_m(bodies);
            min_gap = min_gap.min(gap);
            was_engaged |= gap <= ring_m;

            // Fine resolution only while the ring is actually acting, a
            // sample every half second on the run-in. Keying this on "engaged"
            // rather than on proximity matters: a slow lateral pass spends
            // many seconds just *near* the obstacle, and printing all of it
            // buries the part where something happens. Past a row budget,
            // back off to coarse everywhere — a long engagement is usually
            // chatter, and its shape is clear from the first few seconds.
            let fine = gap <= ring_m && rows < 60;
            let sample_every = if fine { 0.1 } else { 0.5 };
            if t - last_printed >= sample_every {
                last_printed = t;
                rows += 1;
                println!(
                    "{:>7.2}  {:>9.3}  {:>9.3}  {:>9.3}  {:>9.3}  {}",
                    t,
                    gap,
                    bodies[0].speed_mps(),
                    bodies[1].speed_mps(),
                    bodies[0].state.position.z * 1000.0,
                    if gap <= ring_m { "ENGAGED" } else { "-" }
                );
            }

            if gap <= 0.0 {
                contact = true;
                println!("  >> CONTACT at t={t:.2}s");
                break;
            }

            // Settled into a standoff: everything mobile has effectively
            // stopped with the ring still holding it off.
            let moving = bodies.iter().any(|b| b.waypoint.is_some() && b.speed_mps() > 0.01);
            if !moving && gap <= ring_m {
                println!("  >> settled at t={t:.2}s");
                break;
            }
            // Or flew past and left it behind — a deflection, not a standoff.
            // Without this a glancing pass would cruise on to the horizon.
            if was_engaged && gap > ring_m * 2.0 {
                println!("  >> cleared at t={t:.2}s, opening up");
                break;
            }

            step(bodies, limits, dt);
        }

        println!(
            "  result: min gap {:.3} m, final gap {:.3} m — {}",
            min_gap,
            gap_m(bodies),
            if contact { "CONTACT" } else { "no contact" }
        );
    }

    /// Closing at exactly what one airframe can brake from — the rated case.
    #[test]
    #[ignore]
    fn head_on_at_rated_speed() {
        let accel = FlightLimits::default().max_accel_mps2;
        let stoppable = safe_closing_speed_mps(SENSOR_RANGE_M, accel);
        let mut limits = FlightLimits::default().in_km();
        limits.set_max_speed(stoppable / 2.0 / 1000.0);

        // 10 m of runway: reaching 2.45 m/s at 4 m/s² takes under a meter.
        let mut bodies = head_on_pair(0.01);
        run(
            "Head-on at the rated closing speed",
            &format!(
                "Two drones flying through each other at {stoppable:.2} m/s closing \
                 (sqrt(2*a*range), what a single airframe can brake from).",
            ),
            &mut bodies,
            &limits,
        );
    }

    /// Past the single-airframe limit: both drones brake, so the pair still
    /// holds — this is the sqrt(4*a*range) regime.
    #[test]
    #[ignore]
    fn head_on_past_the_single_airframe_limit() {
        let mut limits = FlightLimits::default().in_km();
        limits.set_max_speed(6.8 / 2.0 / 1000.0);

        // 20 m of runway: reaching 3.4 m/s at 4 m/s² takes ~1.4 m.
        let mut bodies = head_on_pair(0.02);
        run(
            "Head-on at 6.8 m/s closing — past one airframe's limit",
            "Both drones brake, so the closing rate sheds at twice one \
             airframe's authority. Rated single-braker limit is 4.90 m/s.",
            &mut bodies,
            &limits,
        );
    }

    /// The documented failure: 3 m of warning cannot arrest a 30 m/s closing
    /// rate, and the drones touch.
    #[test]
    #[ignore]
    fn head_on_at_cruise_speed() {
        let limits = FlightLimits::default().in_km();
        let mut bodies = head_on_pair(0.4);
        run(
            "Head-on at full cruise — the ring loses",
            "30 m/s closing needs ~28 m of braking distance. The ring gives 3 m. \
             This is expected to make contact.",
            &mut bodies,
            &limits,
        );
    }

    /// The other half of the behavior: an obstacle off to one side produces a
    /// sideways step, not a stop. Watch the `A z(m)` column.
    #[test]
    #[ignore]
    fn glancing_pass_deflects_sideways() {
        let mut limits = FlightLimits::default().in_km();
        limits.set_max_speed(4.0 / 1000.0);

        // A dead drone parked 2 m inside our flight path — without a deflection
        // we clip it.
        let mut bodies = vec![
            Body::drone(-0.08, 0.0, Vec3::new(1000.0, 0.0, 0.0)),
            Body::obstacle(0.0, DRONE_RADIUS * 2.0 - 0.002),
        ];
        run(
            "Glancing pass around a parked drone",
            "Flying +X at 4 m/s past a static obstacle sitting 2 m inside our \
             path. The ring should step us sideways (A z) rather than stop us.",
            &mut bodies,
            &limits,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::{navigate, DroneState};

    /// Km-scaled limits, matching what the running system uses.
    fn limits() -> FlightLimits {
        FlightLimits::default().in_km()
    }

    /// Most cases don't care about the flown/planned split — the drone is
    /// already flying what it planned. Tests that *do* care pass both.
    fn avoid(planned: Vec3, detections: &[Detection], dt: f32) -> Vec3 {
        avoidance_velocity(
            planned,
            planned,
            DRONE_RADIUS,
            detections,
            SENSOR_RANGE_KM,
            &limits(),
            dt,
        )
    }

    /// A detection `gap_km` of clear air away, on the given horizontal
    /// bearing from the drone. Both bodies are drone-sized.
    fn detection_at(direction: Vec3, gap_km: f32) -> Detection {
        let center_distance = gap_km + DRONE_RADIUS * 2.0;
        Detection {
            offset: direction.normalize() * center_distance,
            radius_km: DRONE_RADIUS,
        }
    }

    /// Clear air is a strict no-op: a drone with nothing in sensor range must
    /// fly exactly the velocity its navigator planned.
    #[test]
    fn clear_air_leaves_the_plan_untouched() {
        let planned = Vec3::new(0.01, 0.002, -0.004);
        let far = detection_at(Vec3::X, SENSOR_RANGE_KM * 10.0);
        assert_eq!(avoid(planned, &[far], 0.02), planned);
    }

    /// An obstacle exactly at the edge of the ring is seen but not yet acted
    /// on — avoidance authority ramps up from zero, it doesn't snap on.
    #[test]
    fn edge_of_range_does_not_deflect_yet() {
        let planned = Vec3::new(0.01, 0.0, 0.0);
        let edge = detection_at(Vec3::X, SENSOR_RANGE_KM);
        let flown = avoid(planned, &[edge], 0.02);
        assert!((flown - planned).length() < 1e-6, "deflected at the edge: {flown:?}");
    }

    /// Flying straight at something close pushes back against the plan.
    #[test]
    fn obstacle_dead_ahead_pushes_back() {
        let planned = Vec3::new(0.01, 0.0, 0.0);
        let ahead = detection_at(Vec3::X, SENSOR_RANGE_KM * 0.1);
        let flown = avoid(planned, &[ahead], 0.02);
        assert!(flown.x < planned.x, "should have slowed or reversed: {flown:?}");
    }

    /// The drone's own vertical velocity is the navigator's business — this
    /// module plans in the horizontal plane only and must not touch it.
    #[test]
    fn vertical_velocity_passes_through() {
        let planned = Vec3::new(0.01, 0.003, 0.0);
        // Obstacle directly overhead: same X/Z, far above. Its *horizontal*
        // separation is zero, so it is very much detected — and the response
        // must still be purely horizontal.
        let overhead = Detection { offset: Vec3::new(0.0, 0.5, 0.0), radius_km: DRONE_RADIUS };
        assert_eq!(avoid(planned, &[overhead], 0.02).y, planned.y);
    }

    /// A drone holding station still gets shoved out of the way when
    /// something closes on it — the ring is not gated on the drone moving.
    #[test]
    fn stationary_drone_is_pushed_clear() {
        let close = detection_at(Vec3::X, SENSOR_RANGE_KM * 0.05);
        let flown = avoid(Vec3::ZERO, &[close], 0.02);
        assert!(flown.x < 0.0, "should back away from +X obstacle: {flown:?}");
    }

    /// Pinched between two obstacles on opposite sides, the repulsions cancel
    /// and there is no escape bearing to fly. The cheap-sensor answer is to
    /// stop, not to guess a direction.
    #[test]
    fn boxed_in_brakes_instead_of_guessing() {
        let planned = Vec3::new(0.01, 0.0, 0.0);
        let left = detection_at(Vec3::Z, SENSOR_RANGE_KM * 0.05);
        let right = detection_at(-Vec3::Z, SENSOR_RANGE_KM * 0.05);
        let flown = avoid(planned, &[left, right], 0.02);
        // Braking, i.e. moving toward zero rather than being deflected off to
        // one side.
        assert!(flown.x < planned.x, "should be braking: {flown:?}");
        assert!(flown.z.abs() < 1e-6, "should not pick a side: {flown:?}");
    }

    /// Avoidance never buys speed the airframe doesn't have.
    #[test]
    fn never_exceeds_the_speed_envelope() {
        let limits = limits();
        let mut planned = Vec3::new(0.01, 0.0, 0.005);
        for _ in 0..500 {
            let close = detection_at(Vec3::new(1.0, 0.0, 1.0), 0.0);
            planned = avoid(planned, &[close], 0.02);
            let speed = Vec3::new(planned.x, 0.0, planned.z).length();
            assert!(
                speed <= limits.max_speed_mps + 1e-6,
                "speed {speed} exceeded cap {}",
                limits.max_speed_mps
            );
        }
    }

    /// An avoidance maneuver is flown by tilting the same rotor plane as
    /// everything else, so it is bound by the same acceleration limit.
    #[test]
    fn deflection_is_acceleration_limited() {
        let limits = limits();
        let dt = 0.02;
        // Worst case: already flying hard at something that is touching us,
        // so the demanded change is a full reversal.
        let planned = Vec3::new(limits.max_speed_mps, 0.0, 0.0);
        let touching = detection_at(Vec3::X, 0.0);
        let flown = avoid(planned, &[touching], dt);
        let change = (flown - planned).length();
        assert!(
            change <= limits.max_accel_mps2 * dt + 1e-9,
            "changed velocity by {change}, budget was {}",
            limits.max_accel_mps2 * dt
        );
    }

    /// A zero timestep can't fly a maneuver — nothing changes.
    #[test]
    fn zero_dt_is_a_noop() {
        let planned = Vec3::new(0.01, 0.0, 0.0);
        let touching = detection_at(Vec3::X, 0.0);
        assert_eq!(avoid(planned, &[touching], 0.0), planned);
    }

    /// Fly two drones head-on into each other through the real pipeline and
    /// report `(smallest gap reached, final gap)`, both km, surface to
    /// surface.
    ///
    /// This mirrors the running frame exactly — `navigate` writes a planned
    /// velocity the way `recovery::run_recovery` does, avoidance deflects it,
    /// and the deflected value is what gets integrated and fed back as next
    /// tick's flown velocity, the way `apply_velocity` does.
    ///
    /// `runway_km` is the clear air each drone gets before the gap closes.
    /// It has to be long enough for `navigate` to actually reach the speed
    /// cap under test — starting them nose-to-nose from rest would only ever
    /// exercise a slow crawl, no matter what `limits` claims. Each navigator
    /// aims at a waypoint far *beyond* the other drone, so `navigate` never
    /// brakes for a target of its own and the ring is the only thing standing
    /// between them.
    fn head_on(limits: &FlightLimits, runway_km: f32) -> (f32, f32) {
        let dt = 0.02;
        let half_separation = DRONE_RADIUS + runway_km / 2.0;
        let mut a = DroneState { position: Vec3::new(-half_separation, 0.0, 0.0), ..default() };
        let mut b = DroneState { position: Vec3::new(half_separation, 0.0, 0.0), ..default() };
        let mut min_gap = f32::MAX;

        for _ in 0..8000 {
            let (a_pos, b_pos) = (a.position, b.position);
            let (a_flown, b_flown) = (a.velocity, b.velocity);

            // Navigators plan first, straight through each other.
            navigate(&mut a, Vec3::new(1000.0, 0.0, 0.0), limits, dt);
            navigate(&mut b, Vec3::new(-1000.0, 0.0, 0.0), limits, dt);

            // Then the ring gets its veto.
            a.velocity = avoidance_velocity(
                a_flown,
                a.velocity,
                DRONE_RADIUS,
                &[Detection { offset: b_pos - a_pos, radius_km: DRONE_RADIUS }],
                SENSOR_RANGE_KM,
                limits,
                dt,
            );
            b.velocity = avoidance_velocity(
                b_flown,
                b.velocity,
                DRONE_RADIUS,
                &[Detection { offset: a_pos - b_pos, radius_km: DRONE_RADIUS }],
                SENSOR_RANGE_KM,
                limits,
                dt,
            );

            // `navigate` integrated its own (pre-avoidance) step internally;
            // discard that and integrate the deflected velocity instead —
            // exactly what `run_recovery` + `apply_velocity` do between them.
            a.position = a_pos + a.velocity * dt;
            b.position = b_pos + b.velocity * dt;

            min_gap = min_gap.min((b.position - a.position).length() - DRONE_RADIUS * 2.0);
            if min_gap <= 0.0 {
                break;
            }
        }
        (min_gap, (b.position - a.position).length() - DRONE_RADIUS * 2.0)
    }

    /// The headline behavior: two drones whose navigators are actively flying
    /// them through each other must not actually touch — provided they close
    /// no faster than the ring can physically arrest.
    #[test]
    fn converging_drones_do_not_touch() {
        // Each drone flies at the full single-airframe stoppable speed, so
        // they close at `safe_closing_speed_mps` — the whole budget the ring
        // is claimed to cover, not a soft fraction of it.
        let mut limits = limits();
        let stoppable_kms =
            safe_closing_speed_mps(SENSOR_RANGE_M, FlightLimits::default().max_accel_mps2) / 1000.0;
        limits.set_max_speed(stoppable_kms / 2.0);

        // 50 m of runway: `navigate` needs ~0.75 m to reach this cap, so both
        // drones are unambiguously at speed well before the ring sees
        // anything.
        let (min_gap, final_gap) = head_on(&limits, 0.05);

        assert!(min_gap > 0.0, "drones touched: closest gap was {} m", min_gap * 1000.0);
        // And they should end in a standoff rather than still grinding
        // inward: the stable point is where the inward plan and the outward
        // escape run cancel, i.e. partway into the ring.
        assert!(
            final_gap > 0.0 && final_gap < SENSOR_RANGE_KM,
            "expected a standoff inside the ring, got gap {} m",
            final_gap * 1000.0
        );
    }

    /// Both drones braking means the *closing* rate sheds at twice one
    /// airframe's authority, so the pair actually holds separation a good way
    /// past the single-braker `safe_closing_speed_mps` — up to
    /// `sqrt(4·a·range)`, ~6.9 m/s at the defaults.
    #[test]
    fn mutual_braking_beats_the_single_airframe_limit() {
        let accel = FlightLimits::default().max_accel_mps2;
        let mut limits = limits();
        // Closing at 6.8 m/s — past sqrt(2·a·range) = 4.9, just under
        // sqrt(4·a·range) = 6.93.
        limits.set_max_speed(6.8 / 2.0 / 1000.0);
        assert!(6.8 > safe_closing_speed_mps(SENSOR_RANGE_M, accel));

        let (min_gap, _) = head_on(&limits, 0.05);
        assert!(min_gap > 0.0, "drones touched: closest gap was {} m", min_gap * 1000.0);
    }

    /// The documented limitation, pinned so it can't quietly rot into an
    /// assumed guarantee: at the default 15 m/s cruise cap the drones close
    /// far faster than 3 m of warning can arrest, and they *do* touch.
    ///
    /// If this test ever starts failing, the ring got better (bigger
    /// [`SENSOR_RANGE_M`], more acceleration authority, a smarter blend) —
    /// update the "what 3 meters can and cannot do" section of the module
    /// docs to match rather than just deleting the test.
    #[test]
    fn cruise_speed_defeats_the_ring() {
        let limits = limits(); // default 15 m/s cap — 30 m/s closing
        // 400 m of runway: `navigate` needs ~28 m to wind up to 15 m/s.
        let (min_gap, _) = head_on(&limits, 0.4);
        assert!(
            min_gap <= 0.0,
            "a {SENSOR_RANGE_M} m ring should NOT be able to stop a {} m/s \
             head-on closing rate, but it held {} m of separation — if the \
             ring really did get this good, update the module docs",
            limits.max_speed_mps * 2000.0,
            min_gap * 1000.0
        );
    }

    /// The pure math above is only useful if the Bevy system around it
    /// actually runs. Query access conflicts and component-signature mistakes
    /// are runtime panics in Bevy, invisible to `cargo build`, so this drives
    /// `avoid_collisions` through a real `App`: two drones overlapping, one
    /// commanded straight into the other.
    #[test]
    fn system_deflects_in_a_real_app() {
        use crate::drone::DroneType;
        use std::time::Duration;

        fn drone(id: &str) -> Drone {
            Drone { id: id.into(), drone_type: DroneType::Node }
        }

        let mut app = App::new();
        app.insert_resource(Time::<()>::default());
        app.add_systems(Update, avoid_collisions);

        let commanded = Vec3::new(0.01, 0.002, 0.0);
        let mover = app
            .world_mut()
            .spawn((
                drone("A"),
                Transform::from_xyz(0.0, 0.0, 0.0),
                DroneKinematics { velocity: commanded, ..default() },
            ))
            .id();
        // Sitting just inside the ring, straight ahead on +X.
        app.world_mut().spawn((
            drone("B"),
            Transform::from_xyz(DRONE_RADIUS * 2.0 + SENSOR_RANGE_KM * 0.1, 0.0, 0.0),
            DroneKinematics::default(),
        ));

        app.world_mut().resource_mut::<Time>().advance_by(Duration::from_millis(20));
        app.update();

        let flown = app.world().entity(mover).get::<DroneKinematics>().unwrap().velocity;
        assert!(flown.x < commanded.x, "system did not deflect: {flown:?}");
        assert_eq!(flown.y, commanded.y, "system touched vertical velocity");
    }

    /// Braking distance beyond the sensor range is unrecoverable — the
    /// function that says so must agree with `v² = 2·a·d`.
    #[test]
    fn safe_closing_speed_matches_braking_distance() {
        let v = safe_closing_speed_mps(3.0, 4.0);
        assert!((v - (24.0f32).sqrt()).abs() < 1e-4, "got {v}");
        // Sanity: the default 15 m/s cruise is well past what 3 m can stop,
        // which is exactly the caveat documented at the top of this module.
        assert!(v < FlightLimits::default().max_speed_mps);
    }

    /// Degenerate input (two bodies at the exact same spot) must still
    /// produce a finite, deterministic escape rather than a NaN.
    #[test]
    fn coincident_bodies_escape_deterministically() {
        let coincident = Detection { offset: Vec3::ZERO, radius_km: DRONE_RADIUS };
        let flown = avoid(Vec3::ZERO, &[coincident], 0.02);
        assert!(flown.is_finite());
        assert!(flown.length() > 0.0);
    }
}
