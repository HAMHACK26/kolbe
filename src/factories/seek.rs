#![allow(dead_code)]
//! Per-drone seek / navigation logic.
//!
//! Each drone steers itself independently. There is no global planner.
//! The drone only knows its own position, heading, and the target it
//! is trying to reach (usually supplied by its own tracker).

use bevy::prelude::*;

// ─── Trait ───────────────────────────────────────────────────────────────────

/// Per-drone seek interface. Implement this in Rust or Python.
///
/// # Python implementation
///
/// ```python
/// class DroneSeek:
///     def update(
///         self,
///         self_pos:    tuple[float, float, float],  # drone world position (km)
///         heading_deg: float,                        # current heading (°, 0 = +Z)
///         target_pos:  tuple[float, float, float],  # where to go (km)
///     ) -> tuple[
///         tuple[float, float, float],  # desired velocity direction (unit vec)
///         float,                        # desired speed (km/s)
///     ]:
///         ...
///
/// # Register with DroneAi::python("my_module").
/// # The module must expose a `DroneSeek` class at top level.
/// ```
pub trait SeekLogic: Send + Sync {
    /// Called each frame from this drone's perspective.
    ///
    /// `self_pos`   — drone's own world-space position (km)
    /// `heading_deg`— current yaw in degrees (0 = +Z, clockwise)
    /// `target_pos` — position to navigate toward (km)
    ///
    /// Returns `(desired_velocity_dir, speed_km_s)`.
    fn update(&mut self, self_pos: Vec3, heading_deg: f32, target_pos: Vec3) -> (Vec3, f32);
}

// ─── Base components ─────────────────────────────────────────────────────────

/// Where this drone is trying to navigate. Set by mission logic or tracker.
#[derive(Component)]
pub struct SeekTarget {
    pub position: Vec3,
    /// Stop steering when closer than this (km).
    pub arrival_radius_km: f32,
}

// DroneKinematics lives in factories::movement — import it for use in this module.
pub use crate::factories::movement::DroneKinematics;

/// Hard flight envelope for the seek planner.
#[derive(Component)]
pub struct FlightConstraints {
    pub max_speed_km_s: f32,
    pub max_turn_rate_deg_s: f32,
    pub min_altitude_km: f32,
    pub max_altitude_km: f32,
}

impl Default for FlightConstraints {
    fn default() -> Self {
        Self {
            max_speed_km_s: 0.05,
            max_turn_rate_deg_s: 30.0,
            min_altitude_km: 0.05,
            max_altitude_km: 3.0,
        }
    }
}

// ─── Rust implementation (stub) ───────────────────────────────────────────────

pub struct RustSeek;

impl SeekLogic for RustSeek {
    fn update(&mut self, _self_pos: Vec3, _heading_deg: f32, _target_pos: Vec3) -> (Vec3, f32) {
        todo!(
            "Proportional navigation: bearing = atan2(Δx, Δz), \
             clamp turn by max_turn_rate_deg_s * dt, \
             return unit direction + clamped speed"
        )
    }
}

// ─── System stub ─────────────────────────────────────────────────────────────

pub fn run_seek_logic(
    _time: Res<Time>,
    _drones: Query<(
        &mut Transform,
        &mut DroneKinematics,
        &SeekTarget,
        &FlightConstraints,
        &mut crate::factories::DroneAi,
    )>,
) {
    todo!(
        "For each drone: call drone_ai.seek.update(self_pos, heading, target), \
         apply velocity to Transform respecting FlightConstraints, \
         remove SeekTarget on arrival"
    );
}

// ─── Python bridge ────────────────────────────────────────────────────────────

#[cfg(feature = "python")]
pub struct PythonSeek {
    instance: pyo3::PyObject,
}

#[cfg(feature = "python")]
impl PythonSeek {
    /// `module` must expose a `DroneSeek` class.
    pub fn new(module: &str) -> Self {
        use pyo3::prelude::*;
        let instance = Python::with_gil(|py| {
            py.import(module)
                .expect("python module not found")
                .getattr("DroneSeek")
                .expect("DroneSeek class not found")
                .call0()
                .expect("DroneSeek() constructor failed")
                .into()
        });
        Self { instance }
    }
}

#[cfg(feature = "python")]
impl SeekLogic for PythonSeek {
    fn update(&mut self, self_pos: Vec3, heading_deg: f32, target_pos: Vec3) -> (Vec3, f32) {
        use pyo3::prelude::*;
        Python::with_gil(|py| {
            let result = self
                .instance
                .call_method1(
                    py,
                    "update",
                    (
                        (self_pos.x, self_pos.y, self_pos.z),
                        heading_deg,
                        (target_pos.x, target_pos.y, target_pos.z),
                    ),
                )
                .expect("DroneSeek.update() failed");
            let ((dx, dy, dz), speed): ((f32, f32, f32), f32) =
                result.extract(py).expect("DroneSeek.update() bad return type");
            (Vec3::new(dx, dy, dz), speed)
        })
    }
}
