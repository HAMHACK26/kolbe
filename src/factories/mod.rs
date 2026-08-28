pub mod network;
pub mod seek;
pub mod track;

// `move` is a Rust keyword — expose as `movement`.
#[path = "move.rs"]
pub mod movement;

use bevy::prelude::*;

use crate::factories::{
    movement::MovementLogic,
    network::NetworkLogic,
    seek::SeekLogic,
    track::TrackLogic,
};

// ─── Per-drone AI component ───────────────────────────────────────────────────
//
// One DroneAi per drone entity. Every drone is independent — no shared state.
// All logic runs from the drone's own perspective (its sensors, its position).
//
// Usage:
//   DroneAi::rust()             — all Rust stubs (will panic until implemented)
//   DroneAi::python("module")   — all Python (requires --features python)
//   Mix manually:
//     DroneAi {
//         track:    Box::new(MyRustTracker),
//         seek:     Box::new(PythonSeek::new("seek_mod")),
//         network:  Box::new(RustNetwork),
//         movement: Box::new(PythonMove::new("move_mod")),
//     }

#[derive(Component)]
pub struct DroneAi {
    pub track:    Box<dyn TrackLogic>,
    pub seek:     Box<dyn SeekLogic>,
    pub network:  Box<dyn NetworkLogic>,
    pub movement: Box<dyn MovementLogic>,
}

impl DroneAi {
    pub fn rust() -> Self {
        Self {
            track:    Box::new(track::RustTrack),
            seek:     Box::new(seek::RustSeek),
            network:  Box::new(network::RustNetwork),
            movement: Box::new(movement::RustMove),
        }
    }

    #[cfg(feature = "python")]
    pub fn python(module: &str) -> Self {
        Self {
            track:    Box::new(track::PythonTrack::new(module)),
            seek:     Box::new(seek::PythonSeek::new(module)),
            network:  Box::new(network::PythonNetwork::new(module)),
            movement: Box::new(movement::PythonMove::new(module)),
        }
    }
}

impl Default for DroneAi {
    fn default() -> Self {
        Self::rust()
    }
}
