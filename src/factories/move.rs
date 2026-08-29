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
    mut drones: Query<(&mut Transform, &mut DroneKinematics)>,
) {
    let dt = time.delta_secs();
    for (mut transform, mut kin) in &mut drones {
        // Everything that gets a say in this frame's velocity has now had it,
        // so this is what the airframe actually flies — record it before
        // integrating, for next frame's avoidance to measure against.
        kin.flown_velocity = kin.velocity;
        transform.translation += kin.velocity * dt;

        // Terrain following is enforced after every motion command, including
        // avoidance deflections and recovery. The radius converts the 50 m
        // hull clearance into the sphere center's required altitude.
        let ground = terrain.height_at(transform.translation.x, transform.translation.z);
        transform.translation.y = transform.translation.y.max(
            ground
                + crate::world::DRONE_GROUND_CLEARANCE_KM
                + crate::world::DRONE_RADIUS,
        );

        // Update heading from velocity XZ projection
        if kin.velocity.xz().length_squared() > 1e-6 {
            kin.heading_deg = kin.velocity.x.atan2(kin.velocity.z).to_degrees();
        }
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
