#![allow(dead_code)]
//! Per-drone movement and obstacle avoidance.
//!
//! Two layers:
//!   1. `apply_velocity`      — real integration loop; runs every frame, moves drones.
//!   2. `MovementLogic::update` — per-drone velocity planner (stub/Python); combines
//!                              seek direction with avoidance forces before handing
//!                              the final velocity to the integrator.
//!
//! Typical pipeline per frame (all per-drone, no shared state):
//!   SeekLogic  → desired_direction
//!   Avoidance  → repulsion_vector
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
    pub velocity: Vec3,
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
    fn update(
        &mut self,
        _self_pos: Vec3,
        _self_velocity: Vec3,
        desired_velocity: Vec3,
        _obstacles: &[ObstacleInfo],
    ) -> Vec3 {
        todo!(
            "Compute repulsion vectors from each ObstacleInfo \
             (potential field: repulsion ∝ 1/dist²), \
             blend with desired_velocity, clamp magnitude to max_speed_km_s"
        );
        // Placeholder so the compiler knows the return type:
        #[allow(unreachable_code)]
        desired_velocity
    }
}

// ─── Real integration system ──────────────────────────────────────────────────
//
// This system is NOT a stub — it actually moves drones every frame.
// Wire it into App::add_systems(Update, ...) in main.rs.

/// Integrate each drone's velocity into its world Transform.
/// Clamps altitude to [DRONE_RADIUS, max_altitude_km].
pub fn apply_velocity(
    time: Res<Time>,
    mut drones: Query<(&mut Transform, &mut DroneKinematics)>,
) {
    let dt = time.delta_secs();
    for (mut transform, mut kin) in &mut drones {
        transform.translation += kin.velocity * dt;

        // Keep above ground
        transform.translation.y = transform.translation.y
            .max(crate::world::DRONE_RADIUS);

        // Update heading from velocity XZ projection
        if kin.velocity.xz().length_squared() > 1e-6 {
            kin.heading_deg = kin.velocity.x.atan2(kin.velocity.z).to_degrees();
        }
    }
}

// ─── Avoidance system (stub) ──────────────────────────────────────────────────

/// Gather nearby obstacles and push updated velocity into `DroneKinematics`.
/// Runs before `apply_velocity`.
pub fn run_avoidance(
    _time: Res<Time>,
    _drones: Query<
        (Entity, &GlobalTransform, &mut DroneKinematics, &mut crate::factories::DroneAi),
        Without<Obstacle>,
    >,
    _obstacles: Query<(&GlobalTransform, &Obstacle)>,
) {
    todo!(
        "For each drone: collect ObstacleInfo for all obstacles within sensor range, \
         call drone_ai.movement.update(self_pos, vel, desired_vel, &obstacles), \
         write result into DroneKinematics::velocity"
    );
}

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
