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

use std::collections::HashMap;

use bevy::prelude::*;

use crate::drone::Drone;
use crate::factories::movement::DroneKinematics;
use crate::world::{DeploymentTarget, RELAY_WORKING_HOP_KM, RelayTopology};

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

    /// The same envelope expressed in the kilometer units the simulation
    /// world is scaled in, so [`navigate`] integrates consistently with
    /// km-space positions.
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

/// Fly drones toward the selected area while preserving their protected
/// upstream relay hop. Once inside, fixed survey slots fill the area and a
/// local separation bias keeps peers from crowding closer than 2 km.
pub fn go_to_network_area(
    time: Res<Time>,
    network_area: Res<crate::area::NetworkArea>,
    scenario: Res<crate::area::ScenarioArea>,
    topology: Res<RelayTopology>,
    positions: Query<(Entity, &Transform), Or<(With<Drone>, With<crate::base::Base>)>>,
    mut drones: Query<
        (Entity, &Transform, &mut DeploymentTarget, &mut DroneKinematics),
        With<Drone>,
    >,
) {
    let dt = time.delta_secs();
    let limits = FlightLimits::default().in_km();
    let positions: HashMap<Entity, Vec3> = positions
        .iter()
        .map(|(entity, transform)| (entity, transform.translation))
        .collect();
    let peer_positions: Vec<(Entity, Vec3)> = positions
        .iter()
        .filter(|(entity, _)| topology.parent(**entity).is_some())
        .map(|(&entity, &position)| (entity, position))
        .collect();

    for (entity, transform, mut target, mut kin) in &mut drones {
        if !target.spreading && inside_target_area(transform.translation, &network_area, &scenario) {
            target.spreading = true;
        }
        let desired_waypoint = if target.spreading {
            let slot = repel_from_target_boundary(
                transform.translation,
                clamp_to_target_area(target.slot, &network_area, &scenario),
                &network_area,
                &scenario,
            );
            repel_from_nearby_drones(
                entity,
                transform.translation,
                slot,
                &peer_positions,
                &network_area,
                &scenario,
            )
        } else {
            target.ingress
        };
        let waypoint = topology
            .parent(entity)
            .and_then(|parent| positions.get(&parent).copied())
            .map(|parent| clamp_to_relay_parent(desired_waypoint, parent))
            .unwrap_or(transform.translation);
        let mut state = DroneState {
            position: transform.translation,
            velocity: kin.velocity,
            heading_deg: kin.heading_deg,
        };
        navigate(&mut state, waypoint, &limits, dt);
        kin.velocity = state.velocity;
        kin.heading_deg = state.heading_deg;
    }
}

fn clamp_to_relay_parent(waypoint: Vec3, parent: Vec3) -> Vec3 {
    let offset = waypoint - parent;
    let distance = offset.length();
    if distance <= RELAY_WORKING_HOP_KM || distance <= f32::EPSILON {
        waypoint
    } else {
        parent + offset / distance * RELAY_WORKING_HOP_KM
    }
}

const AREA_MIN_SPACING_KM: f32 = 2.0;
const AREA_PREFERRED_SPACING_KM: f32 = 2.5;

fn repel_from_nearby_drones(
    self_entity: Entity,
    position: Vec3,
    waypoint: Vec3,
    peers: &[(Entity, Vec3)],
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
) -> Vec3 {
    let mut push = Vec3::ZERO;
    for &(peer, peer_position) in peers {
        if peer == self_entity {
            continue;
        }
        let offset = Vec3::new(position.x - peer_position.x, 0.0, position.z - peer_position.z);
        let distance = offset.length();
        if distance >= AREA_MIN_SPACING_KM {
            continue;
        }
        let away = if distance > f32::EPSILON {
            offset / distance
        } else if self_entity.to_bits() & 1 == 0 {
            Vec3::X
        } else {
            Vec3::Z
        };
        push += away * (1.0 - distance / AREA_MIN_SPACING_KM);
    }
    if push.length_squared() <= f32::EPSILON {
        waypoint
    } else {
        clamp_to_target_area(
            waypoint + push.normalize() * AREA_PREFERRED_SPACING_KM,
            area,
            scenario,
        )
    }
}

/// The blue boundary behaves like a virtual survey neighbor at this distance.
/// It keeps the formation off edges and combines two edge forces at corners.
const TARGET_BOUNDARY_SPACING_KM: f32 = crate::world::FORMATION_RADIUS_KM;

fn repel_from_target_boundary(
    position: Vec3,
    slot: Vec3,
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
) -> Vec3 {
    let hull = target_hull_local(area, scenario);
    if hull.len() < 3 {
        return slot;
    }

    // The stored hull is counter-clockwise, so its left-hand normals point
    // inward. Using every nearby edge makes a corner produce a diagonal push.
    let point = position.xz();
    let mut push = Vec2::ZERO;
    let mut urgency = 0.0_f32;
    for (start, end) in hull.iter().zip(hull.iter().cycle().skip(1)).take(hull.len()) {
        let edge = *end - *start;
        let length = edge.length();
        if length <= f32::EPSILON {
            continue;
        }
        let inward = Vec2::new(-edge.y, edge.x) / length;
        let distance = ((point - *start).dot(inward)).max(0.0);
        if distance < TARGET_BOUNDARY_SPACING_KM {
            let weight = (1.0 - distance / TARGET_BOUNDARY_SPACING_KM).clamp(0.0, 1.0);
            push += inward * weight;
            urgency = urgency.max(weight);
        }
    }
    if push.length_squared() <= f32::EPSILON {
        return slot;
    }
    let escape = position + Vec3::new(push.x, 0.0, push.y).normalize() * TARGET_BOUNDARY_SPACING_KM;
    slot.lerp(clamp_to_target_area(escape, area, scenario), urgency)
}

fn target_hull_local(
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
) -> Vec<Vec2> {
    area.hull
        .iter()
        .map(|&(lon, lat)| Vec2::new(
            ((lon - scenario.longitude) * 111.320 * scenario.latitude.to_radians().cos()) as f32,
            ((lat - scenario.latitude) * 110.574) as f32,
        ))
        .collect()
}

fn inside_target_area(
    position: Vec3,
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
) -> bool {
    if !area.valid || area.hull.len() < 3 {
        return false;
    }
    let point = position.xz();
    let hull = target_hull_local(area, scenario);
    let mut sign = 0.0_f32;
    for (start, end) in hull.iter().zip(hull.iter().cycle().skip(1)).take(hull.len()) {
        let cross = (end.x - start.x) * (point.y - start.y)
            - (end.y - start.y) * (point.x - start.x);
        if cross.abs() > 1e-5 {
            if sign != 0.0 && cross.signum() != sign {
                return false;
            }
            sign = cross.signum();
        }
    }
    true
}

/// Keep an already-deployed drone inside the blue target polygon.
/// This is a hard geofence rather than a steering suggestion: collision
/// avoidance is allowed to change a course, but not to take a drone back out
/// of the target area after it has entered it.
pub fn clamp_to_target_area(
    mut position: Vec3,
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
) -> Vec3 {
    if !area.valid || area.hull.len() < 3 || inside_target_area(position, area, scenario) {
        return position;
    }
    let point = position.xz();
    let hull = target_hull_local(area, scenario);
    let closest = hull.iter().zip(hull.iter().cycle().skip(1)).take(hull.len())
        .map(|(start, end)| {
            let edge = *end - *start;
            let t = ((point - *start).dot(edge) / edge.length_squared()).clamp(0.0, 1.0);
            *start + edge * t
        })
        .min_by(|a, b| a.distance_squared(point).total_cmp(&b.distance_squared(point)))
        .expect("a target polygon has edges");
    position.x = closest.x;
    position.z = closest.y;
    position
}

#[cfg(test)]
mod relay_navigation_tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn relay_waypoint_never_exceeds_working_hop() {
        let parent = Vec3::new(1.0, 0.0, -2.0);
        let waypoint = clamp_to_relay_parent(Vec3::new(20.0, 0.0, 5.0), parent);
        assert!((waypoint.distance(parent) - RELAY_WORKING_HOP_KM).abs() < 1e-6);
    }

    #[test]
    fn connected_drone_accelerates_toward_the_area() {
        let mut app = App::new();
        app.insert_resource(Time::<()>::default());
        app.insert_resource(crate::area::NetworkArea::default());
        app.insert_resource(crate::area::ScenarioArea::default());
        let base = app
            .world_mut()
            .spawn((
                crate::base::Base {
                    id: "base".into(),
                    position: Vec3::ZERO,
                    antennas: Vec::new(),
                },
                Transform::default(),
            ))
            .id();
        let drone = app
            .world_mut()
            .spawn((
                Drone { id: "drone".into() },
                Transform::from_xyz(0.1, 0.0, 0.0),
                DeploymentTarget {
                    ingress: Vec3::X,
                    slot: Vec3::X,
                    spreading: false,
                },
                DroneKinematics::default(),
            ))
            .id();
        let mut topology = RelayTopology::default();
        topology.register_wave(base, vec![drone]);
        app.insert_resource(topology);
        app.add_systems(Update, go_to_network_area);
        app.world_mut()
            .resource_mut::<Time>()
            .advance_by(Duration::from_secs(1));

        app.update();

        let velocity = app
            .world()
            .entity(drone)
            .get::<DroneKinematics>()
            .unwrap()
            .velocity;
        assert!(velocity.x > 0.0);
    }
}
