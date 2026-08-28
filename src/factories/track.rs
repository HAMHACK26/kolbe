#![allow(dead_code)]
//! Per-drone tracking logic.
//!
//! Each drone runs its own tracker independently — no shared state.
//! All inputs are from this drone's own sensor perspective.
//!
//! Intended approach: conscan sweep modulates RSSI through the angle
//! term only (`Antenna::gain_db(θ)`). Geometry:
//!   θ = `Antenna::off_boresight_deg(self_pos, peer_pos)`
//!   d = `(peer_pos - self_pos).length()`

use bevy::prelude::*;

// ─── Trait ───────────────────────────────────────────────────────────────────

/// Per-drone tracking interface. Implement this in Rust or Python.
///
/// # Python implementation
///
/// Create a class in your module with this method:
///
/// ```python
/// class DroneTrack:
///     def update(
///         self,
///         self_pos: tuple[float, float, float],   # drone world position (km)
///         rssi_samples: list[tuple[float, float]], # (angle_deg, rssi_dbm) from conscan sweep
///     ) -> tuple[
///         tuple[float, float, float],  # estimated target world position (km)
///         float,                        # confidence 0.0..1.0
///     ]:
///         ...
///
/// # Register with DroneAi::python("my_module") — the module must expose
/// # a `DroneTrack` class at top level.
/// ```
pub trait TrackLogic: Send + Sync {
    /// Called each frame from this drone's perspective.
    ///
    /// `self_pos`     — drone's own world-space position (km units)
    /// `rssi_samples` — `(off_boresight_deg, rssi_dbm)` from latest antenna sweep
    ///
    /// Returns `(estimated_target_pos, confidence)`.
    fn update(&mut self, self_pos: Vec3, rssi_samples: &[(f32, f32)]) -> (Vec3, f32);
}

// ─── Base components ─────────────────────────────────────────────────────────

/// Active track held by a single drone.
#[derive(Component)]
pub struct Track {
    pub target_position: Vec3,
    pub confidence: f32,
    pub last_rssi_dbm: f32,
    pub peer: Option<Entity>,
}

/// Marks a drone as running its tracking loop this frame.
#[derive(Component)]
pub struct TrackingActive;

/// RSSI sample ring-buffer filled by the conscan sweep, fed into TrackLogic.
#[derive(Component, Default)]
pub struct RssiHistory {
    pub samples: Vec<(f32, f32)>, // (angle_deg, rssi_dbm)
}

// ─── Rust implementation (stub) ───────────────────────────────────────────────

pub struct RustTrack;

impl TrackLogic for RustTrack {
    fn update(&mut self, _self_pos: Vec3, _rssi_samples: &[(f32, f32)]) -> (Vec3, f32) {
        todo!(
            "Conscan: sweep boresight, collect gain_db(θ) samples, \
             fit Gaussian/parabolic peak → bearing estimate → target position"
        )
    }
}

// ─── System stubs ─────────────────────────────────────────────────────────────

pub fn run_track_logic(
    _time: Res<Time>,
    _drones: Query<(Entity, &GlobalTransform, &mut crate::factories::DroneAi, &mut Track, &RssiHistory), With<TrackingActive>>,
) {
    todo!(
        "For each drone: call drone_ai.track.update(self_pos, &history.samples), \
         write result into Track component"
    );
}

// ─── Python bridge ────────────────────────────────────────────────────────────

#[cfg(feature = "python")]
pub struct PythonTrack {
    instance: pyo3::PyObject,
}

#[cfg(feature = "python")]
impl PythonTrack {
    /// `module` must be importable by the Python runtime and expose a `DroneTrack` class.
    pub fn new(module: &str) -> Self {
        use pyo3::prelude::*;
        let instance = Python::with_gil(|py| {
            let m = py.import(module).expect("python module not found");
            m.getattr("DroneTrack")
                .expect("DroneTrack class not found in module")
                .call0()
                .expect("DroneTrack() constructor failed")
                .into()
        });
        Self { instance }
    }
}

#[cfg(feature = "python")]
impl TrackLogic for PythonTrack {
    fn update(&mut self, self_pos: Vec3, rssi_samples: &[(f32, f32)]) -> (Vec3, f32) {
        use pyo3::prelude::*;
        Python::with_gil(|py| {
            let pos = (self_pos.x, self_pos.y, self_pos.z);
            let samples: Vec<(f32, f32)> = rssi_samples.to_vec();
            let result = self
                .instance
                .call_method1(py, "update", (pos, samples))
                .expect("DroneTrack.update() failed");
            let (px, py_, pz, conf): (f32, f32, f32, f32) =
                result.extract(py).expect("DroneTrack.update() bad return type");
            (Vec3::new(px, py_, pz), conf)
        })
    }
}
