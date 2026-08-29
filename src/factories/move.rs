#![allow(dead_code)]
//! Per-drone movement and obstacle avoidance.
//!
//! Two layers:
//!   1. `apply_velocity`      — real integration loop; runs every frame, moves drones.
//!   2. `MovementLogic::update` — per-drone velocity planner (Rust/Python); combines
//!      seek direction with avoidance forces before handing the final velocity to
//!      the integrator.
//!
//! Typical pipeline per frame (all per-drone, no shared state):
//!   SeekLogic  → desired_direction
//!   Avoidance  → repulsion_vector    (see `crate::avoidance`)
//!   MovementLogic::update(self, vel, desired, obstacles) → final_velocity
//!   apply_velocity writes final_velocity into DroneKinematics + Transform

use bevy::prelude::*;

// ─── Trait ───────────────────────────────────────────────────────────────────

/// Per-drone movement interface. Implement in Rust or Python.
///
/// # Python implementation
///
/// ```python
/// class DroneMovement:
///     def update(
///         self,
///         self_pos:        tuple[float, float, float],  # km
///         self_velocity:   tuple[float, float, float],  # km/s
///         desired_velocity: tuple[float, float, float], # from seek, km/s
///         obstacles: list[dict],  # [{"position": (x,y,z), "radius_km": float}, ...]
///     ) -> tuple[float, float, float]:  # final velocity to apply (km/s)
///         # Blend seek direction with avoidance repulsion here.
///         # Return zero-vector to hover in place.
///         ...
///
/// # Register with DroneAi::python("my_module").
/// # The module must expose a `DroneMovement` class at top level.
/// # Keep final speed within FlightConstraints.max_speed_km_s.
/// ```
pub trait MovementLogic: Send + Sync {
    /// Called each frame before `apply_velocity`.
    ///
    /// `self_pos`        — current world position (km)
    /// `self_velocity`   — current velocity (km/s)
    /// `desired_velocity`— velocity computed by SeekLogic (km/s)
    /// `obstacles`       — nearby objects that must be avoided
    ///
    /// Returns the final velocity to integrate this frame (km/s).
    fn update(
        &mut self,
        self_pos: Vec3,
        self_velocity: Vec3,
        desired_velocity: Vec3,
        obstacles: &[ObstacleInfo],
    ) -> Vec3;
}

// ─── Base components ─────────────────────────────────────────────────────────

/// Current kinematic state of a drone (world-space, km / km·s⁻¹).
/// Written by `apply_velocity`; read by seek, track, network.
#[derive(Component, Default)]
pub struct DroneKinematics {
    /// This frame's commanded velocity. Navigators overwrite it outright, so
    /// mid-frame it is an *intent*, not necessarily what the airframe is
    /// doing — see `flown_velocity`.
    pub velocity: Vec3,
    /// The velocity the drone was actually flying as of the last integration,
    /// recorded by `apply_velocity` just before it steps the transform.
    ///
    /// This exists because `velocity` is clobbered by whichever navigator runs
    /// first each frame, which destroys the only record of what the airframe
    /// was really doing. `crate::avoidance` needs that record: an avoidance
    /// maneuver is bound by the airframe's acceleration limit measured from
    /// the drone's *real* motion, not from a command the flight controller has
    /// not had a single tick to act on yet. Anchoring on `velocity` instead
    /// would let a navigator spend the whole tilt budget flying inbound and
    /// leave avoidance only enough to cancel it — the drone would coast
    /// straight through an obstacle at constant speed, unable to brake.
    pub flown_velocity: Vec3,
    /// Yaw in degrees, 0 = +Z, clockwise.
    pub heading_deg: f32,
}

/// Small, temporary wind disturbances around a drone's intended trajectory.
///
/// This is deliberately a lightweight hover model rather than atmospheric
/// simulation. A gust applies a short horizontal acceleration, while a damped
/// virtual position-hold controller pulls the drone back toward the path it
/// would have flown in still air. Consequently a hovering drone wanders and
/// corrects instead of accumulating an unbounded random walk.
#[derive(Component, Debug)]
pub struct HoverWind {
    /// Strength multiplier: 0 disables wind, 1 is the baseline, 2 doubles it.
    intensity: f32,
    /// Displacement from the still-air trajectory, km.
    offset: Vec3,
    /// Velocity contributed by the disturbance, km/s.
    velocity: Vec3,
    /// Smoothed acceleration of the current gust, km/s².
    gust_acceleration: Vec3,
    /// Acceleration the current gust is building toward, km/s².
    target_acceleration: Vec3,
    /// Seconds until the current gust or lull changes state.
    phase_remaining_secs: f32,
    gusting: bool,
    rng_state: u32,
}

// These excursions are intentionally a little larger than GPS position-hold
// error on a consumer quad: the map is 20 km wide, so centimetre-scale motion
// would be invisible. They remain tiny relative to the simulation scale.
const MAX_WIND_OFFSET_KM: f32 = 0.015; // 15 m
const MAX_WIND_SPEED_KM_S: f32 = 0.006; // 6 m/s
const MAX_GUST_ACCEL_KM_S2: f32 = 0.005; // 5 m/s² at the strongest instant
const POSITION_HOLD_OMEGA: f32 = 0.65;
const POSITION_HOLD_DAMPING: f32 = 0.85;
const GUST_RESPONSE_SECS: f32 = 0.30;

impl HoverWind {
    /// Create a deterministic but distinct wind response for one drone.
    ///
    /// `intensity` is a linear strength multiplier. Negative values are
    /// treated as zero.
    pub fn new(seed: usize, intensity: f32) -> Self {
        let mut wind = Self {
            intensity: intensity.max(0.0),
            offset: Vec3::ZERO,
            velocity: Vec3::ZERO,
            gust_acceleration: Vec3::ZERO,
            target_acceleration: Vec3::ZERO,
            phase_remaining_secs: 0.0,
            gusting: false,
            rng_state: Self::mixed_seed(seed),
        };

        // Do not put every drone at the start of a gust on the same frame.
        // Each independently seeded stream chooses its own initial phase.
        if wind.random_range(0.0, 1.0) < 0.5 {
            wind.begin_gust();
        } else {
            wind.begin_lull();
        }
        wind
    }

    /// Avalanche adjacent drone indices into unrelated PRNG states. Feeding
    /// sequential seeds directly into a linear generator leaves visible
    /// correlations between drones even though their state is technically
    /// separate.
    fn mixed_seed(seed: usize) -> u32 {
        let mut x = (seed as u32).wrapping_add(0x9e37_79b9);
        x = (x ^ (x >> 16)).wrapping_mul(0x21f0_aaad);
        x = (x ^ (x >> 15)).wrapping_mul(0x735a_2d97);
        (x ^ (x >> 15)).max(1)
    }

    /// Advance the disturbance and return its extra displacement this frame.
    /// Returning displacement, rather than mutating commanded velocity, keeps
    /// wind independent of navigation and recovery logic.
    fn advance(&mut self, frame_dt: f32) -> Vec3 {
        // A long debugger pause must not launch the drone across the map. The
        // normal render timestep is far below this cap.
        let dt = frame_dt.clamp(0.0, 0.1);
        if dt == 0.0 {
            return Vec3::ZERO;
        }

        self.phase_remaining_secs -= dt;
        if self.phase_remaining_secs <= 0.0 {
            if self.gusting {
                self.begin_lull();
            } else {
                self.begin_gust();
            }
        }

        // Rotor/controller response rounds off the beginning and end of a
        // gust instead of changing acceleration discontinuously.
        let response = 1.0 - (-dt / GUST_RESPONSE_SECS).exp();
        self.gust_acceleration =
            self.gust_acceleration.lerp(self.target_acceleration, response);

        // Damped spring about the still-air trajectory: the position-hold
        // flight controller correcting the accumulated wind error.
        let restoring_acceleration = -POSITION_HOLD_OMEGA.powi(2) * self.offset
            - 2.0 * POSITION_HOLD_DAMPING * POSITION_HOLD_OMEGA * self.velocity;
        self.velocity += (self.gust_acceleration + restoring_acceleration) * dt;
        let max_speed = MAX_WIND_SPEED_KM_S * self.intensity;
        self.velocity = self.velocity.clamp_length_max(max_speed);

        let previous_offset = self.offset;
        self.offset += self.velocity * dt;
        self.offset.y = 0.0;
        let max_offset = MAX_WIND_OFFSET_KM * self.intensity;
        self.offset = self.offset.clamp_length_max(max_offset);

        // Do not keep integrating outward velocity against the displacement
        // limiter; allow the next controller update to move inward at once.
        if self.offset.length_squared() >= max_offset.powi(2)
            && self.velocity.dot(self.offset) > 0.0
        {
            self.velocity -= self.velocity.project_onto(self.offset);
        }

        self.offset - previous_offset
    }

    fn begin_gust(&mut self) {
        self.gusting = true;
        self.phase_remaining_secs = self.random_range(0.7, 1.8);

        let mut direction = Vec3::new(
            self.random_range(-1.0, 1.0),
            0.0,
            self.random_range(-1.0, 1.0),
        );
        if direction.length_squared() < 1e-4 {
            direction = Vec3::X;
        }
        let magnitude = self.random_range(0.25, 1.0) * MAX_GUST_ACCEL_KM_S2 * self.intensity;
        self.target_acceleration = direction.normalize() * magnitude;
    }

    fn begin_lull(&mut self) {
        self.gusting = false;
        self.phase_remaining_secs = self.random_range(0.4, 1.2);
        self.target_acceleration = Vec3::ZERO;
    }

    fn random_range(&mut self, min: f32, max: f32) -> f32 {
        // Xorshift32: each HoverWind owns this state, so drones advance wholly
        // independent streams while scenarios remain deterministic.
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 17;
        self.rng_state ^= self.rng_state << 5;
        let unit = (self.rng_state >> 8) as f32 / 0x00ff_ffff as f32;
        min + (max - min) * unit
    }
}

impl Default for HoverWind {
    fn default() -> Self {
        Self::new(0, 1.0)
    }
}

/// A static or dynamic obstacle this drone must avoid.
#[derive(Component)]
pub struct Obstacle {
    pub radius_km: f32,
}

/// Snapshot passed into `MovementLogic::update` each frame.
pub struct ObstacleInfo {
    pub position: Vec3,
    pub radius_km: f32,
}

/// Waypoint queue for this drone (consumed front-to-back by SeekLogic).
#[derive(Component, Default)]
pub struct WaypointQueue {
    pub waypoints: std::collections::VecDeque<Vec3>,
}

// ─── Rust implementation (stub) ───────────────────────────────────────────────

pub struct RustMove;

impl MovementLogic for RustMove {
    /// Delegates to [`crate::avoidance`], which is the single authority on how
    /// the proximity ring deflects a planned velocity — a Python
    /// [`PythonMove`] swapped in here is overriding *that* behavior, so the
    /// Rust default has to be the same behavior the running simulation flies.
    ///
    /// The trait passes no timestep, so this reports the velocity avoidance
    /// would settle on given unlimited time to tilt into it (`dt` large enough
    /// that the acceleration limit never binds). The real per-frame,
    /// rate-limited version is `avoidance::avoid_collisions`.
    fn update(
        &mut self,
        self_pos: Vec3,
        self_velocity: Vec3,
        desired_velocity: Vec3,
        obstacles: &[ObstacleInfo],
    ) -> Vec3 {
        use crate::avoidance::{avoidance_velocity, Detection, SENSOR_RANGE_KM};
        use crate::navigation::FlightLimits;

        let detections: Vec<Detection> = obstacles
            .iter()
            .map(|o| Detection { offset: o.position - self_pos, radius_km: o.radius_km })
            .collect();

        avoidance_velocity(
            self_velocity,
            desired_velocity,
            crate::world::DRONE_RADIUS,
            &detections,
            SENSOR_RANGE_KM,
            &FlightLimits::default().in_km(),
            f32::MAX,
        )
    }
}

// ─── Real integration system ──────────────────────────────────────────────────
//
// This system is NOT a stub — it actually moves drones every frame.
// Wire it into App::add_systems(Update, ...) in main.rs.

/// Integrate each drone's velocity into its world Transform.
/// Keeps each drone's hull at least 50 m above the terrain directly below it.
pub fn apply_velocity(
    time: Res<Time>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
    network_area: Res<crate::area::NetworkArea>,
    scenario: Res<crate::area::ScenarioArea>,
    mut drones: Query<(
        &mut Transform,
        &mut DroneKinematics,
        Option<&crate::world::DeploymentTarget>,
        Option<&mut HoverWind>,
    )>,
) {
    let dt = time.delta_secs();
    for (mut transform, mut kin, deployment, wind) in &mut drones {
        let wind_displacement = wind.map_or(Vec3::ZERO, |mut wind| wind.advance(dt));
        let wind_velocity = if dt > 0.0 { wind_displacement / dt } else { Vec3::ZERO };
        // Everything that gets a say in this frame's velocity has now had it,
        // so this is what the airframe actually flies — record it before
        // integrating, for next frame's avoidance to measure against.
        kin.flown_velocity = kin.velocity + wind_velocity;
        transform.translation += kin.velocity * dt + wind_displacement;

        // Once a drone has crossed into the blue target polygon it is geofenced there.
        // This runs at integration time, after every navigator and avoidance
        // system, so a late collision deflection cannot carry it across the
        // orange boundary.
        if deployment.is_some_and(|target| target.spreading) {
            let before_clamp = transform.translation;
            transform.translation = crate::navigation::clamp_to_target_area(
                transform.translation,
                &network_area,
                &scenario,
            );
            if transform.translation.x != before_clamp.x {
                kin.velocity.x = 0.0;
                kin.flown_velocity.x = 0.0;
            }
            if transform.translation.z != before_clamp.z {
                kin.velocity.z = 0.0;
                kin.flown_velocity.z = 0.0;
            }
        }

        // Terrain following is enforced after every motion command, including
        // avoidance deflections and recovery. The radius converts the 50 m
        // hull clearance into the sphere center's required altitude.
        let ground = terrain.height_at(transform.translation.x, transform.translation.z);
        // Terrain is a floor, not a hard flight level: canopy avoidance may
        // have climbed above it and must keep that clearance until the forest
        // is behind the drone.
        transform.translation.y = transform.translation.y.max(
            ground + crate::world::DRONE_GROUND_CLEARANCE_KM + crate::world::DRONE_RADIUS,
        );

        // Update heading from velocity XZ projection
        if kin.velocity.xz().length_squared() > 1e-6 {
            kin.heading_deg = kin.velocity.x.atan2(kin.velocity.z).to_degrees();
        }
    }
}

#[cfg(test)]
mod hover_wind_tests {
    use super::*;

    #[test]
    fn hover_wind_moves_but_stays_bounded_and_horizontal() {
        let mut wind = HoverWind::new(7, 1.0);
        let mut greatest_excursion = 0.0_f32;

        // Four simulated minutes exercises many independent gust/lull cycles.
        for _ in 0..12_000 {
            wind.advance(0.02);
            greatest_excursion = greatest_excursion.max(wind.offset.length());
            assert!(wind.offset.is_finite());
            assert!(wind.velocity.is_finite());
            assert_eq!(wind.offset.y, 0.0);
            assert!(wind.offset.length() <= MAX_WIND_OFFSET_KM + 1e-6);
            assert!(wind.velocity.length() <= MAX_WIND_SPEED_KM_S + 1e-6);
        }

        assert!(greatest_excursion > 0.001, "gusts never produced visible movement");
    }

    #[test]
    fn drone_seeds_have_independent_phases_and_motion() {
        let mut winds: Vec<_> = (0..12).map(|seed| HoverWind::new(seed, 1.0)).collect();

        let gusting_count = winds.iter().filter(|wind| wind.gusting).count();
        assert!(
            gusting_count > 0 && gusting_count < winds.len(),
            "all drones started in the same wind phase"
        );

        for _ in 0..500 {
            for wind in &mut winds {
                wind.advance(0.02);
            }
        }

        for pair in winds.windows(2) {
            assert!(
                pair[0].offset.distance(pair[1].offset) > 1e-5,
                "adjacent drone seeds moved in lockstep"
            );
        }
    }

    #[test]
    fn zero_timestep_is_a_noop() {
        let mut wind = HoverWind::new(3, 1.0);
        let before = wind.offset;
        assert_eq!(wind.advance(0.0), Vec3::ZERO);
        assert_eq!(wind.offset, before);
    }

    #[test]
    fn zero_intensity_disables_wind() {
        let mut wind = HoverWind::new(4, 0.0);

        for _ in 0..1_000 {
            assert_eq!(wind.advance(0.02), Vec3::ZERO);
        }

        assert_eq!(wind.offset, Vec3::ZERO);
        assert_eq!(wind.velocity, Vec3::ZERO);
    }
}

// ─── Avoidance ────────────────────────────────────────────────────────────────
//
// The live avoidance system is `crate::avoidance::avoid_collisions` — it
// gathers detections from the world and deflects `DroneKinematics::velocity`
// in place, running after the navigators and before `apply_velocity` above.
// The sensor model and the deflection math live in `crate::avoidance`.

// ─── Python bridge ────────────────────────────────────────────────────────────

#[cfg(feature = "python")]
pub struct PythonMove {
    instance: pyo3::PyObject,
}

#[cfg(feature = "python")]
impl PythonMove {
    /// `module` must expose a `DroneMovement` class.
    pub fn new(module: &str) -> Self {
        use pyo3::prelude::*;
        let instance = Python::with_gil(|py| {
            py.import(module)
                .expect("python module not found")
                .getattr("DroneMovement")
                .expect("DroneMovement class not found")
                .call0()
                .expect("DroneMovement() constructor failed")
                .into()
        });
        Self { instance }
    }
}

#[cfg(feature = "python")]
impl MovementLogic for PythonMove {
    fn update(
        &mut self,
        self_pos: Vec3,
        self_velocity: Vec3,
        desired_velocity: Vec3,
        obstacles: &[ObstacleInfo],
    ) -> Vec3 {
        use pyo3::prelude::*;
        Python::with_gil(|py| {
            let obs: Vec<_> = obstacles
                .iter()
                .map(|o| {
                    let d = pyo3::types::PyDict::new(py);
                    d.set_item("position", (o.position.x, o.position.y, o.position.z)).unwrap();
                    d.set_item("radius_km", o.radius_km).unwrap();
                    d
                })
                .collect();

            let result = self
                .instance
                .call_method1(
                    py,
                    "update",
                    (
                        (self_pos.x, self_pos.y, self_pos.z),
                        (self_velocity.x, self_velocity.y, self_velocity.z),
                        (desired_velocity.x, desired_velocity.y, desired_velocity.z),
                        obs,
                    ),
                )
                .expect("DroneMovement.update() failed");

            let (vx, vy, vz): (f32, f32, f32) =
                result.extract(py).expect("DroneMovement.update() must return (float, float, float)");
            Vec3::new(vx, vy, vz)
        })
    }
}
