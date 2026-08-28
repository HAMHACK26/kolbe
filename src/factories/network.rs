#![allow(dead_code)]
//! Per-drone network / mesh communication logic.
//!
//! Each drone manages its own link state independently. A drone only knows
//! about peers it can locally sense (above sensitivity threshold).
//! Topology is a directed graph — links are asymmetric when antenna gains differ.
//!
//! Link geometry:
//!   θ_tx = `Antenna::off_boresight_deg(self_pos, peer_pos)`
//!   θ_rx = computed from peer's antenna (when peer info is available)
//!   d    = `(peer_pos - self_pos).length()`
//!   rssi = `Antenna::rssi_dbm(θ_tx, θ_rx, d)`

use bevy::prelude::*;

// ─── Shared types ─────────────────────────────────────────────────────────────

/// A peer visible to this drone's antenna.
pub struct PeerInfo {
    pub entity: Entity,
    pub position: Vec3,
    pub rssi_dbm: f32,
}

/// Decisions returned by NetworkLogic each frame.
pub struct NetworkDecision {
    /// Entities this drone wants to maintain active links with.
    pub connect_to: Vec<Entity>,
    pub role: NetworkRole,
}

/// Role this drone plays in the mesh.
#[derive(Component, Default, PartialEq, Eq, Clone, Copy)]
pub enum NetworkRole {
    #[default]
    Leaf,
    Relay,
    Master,
}

// ─── Trait ───────────────────────────────────────────────────────────────────

/// Per-drone network interface. Implement this in Rust or Python.
///
/// # Python implementation
///
/// ```python
/// class DroneNetwork:
///     def update(
///         self,
///         self_pos: tuple[float, float, float],  # drone world position (km)
///         peers: list[dict],  # each: {"entity": int, "position": (x,y,z), "rssi_dbm": float}
///     ) -> dict:
///         # Return:
///         # {
///         #   "connect_to": [entity_id, ...],   # ints
///         #   "role": "Leaf" | "Relay" | "Master"
///         # }
///         ...
///
/// # Register with DroneAi::python("my_module").
/// # The module must expose a `DroneNetwork` class at top level.
/// # Entity IDs are stable integers you can store and compare across frames.
/// ```
pub trait NetworkLogic: Send + Sync {
    /// Called each frame from this drone's perspective.
    ///
    /// `self_pos` — this drone's world-space position (km)
    /// `peers`    — locally visible peers with pre-computed RSSI
    ///
    /// Returns link and role decisions.
    fn update(&mut self, self_pos: Vec3, peers: &[PeerInfo]) -> NetworkDecision;
}

// ─── Base components ─────────────────────────────────────────────────────────

/// One directed radio link from this drone to `peer`.
/// Multiple links per drone (one per reachable peer per antenna).
#[derive(Component)]
pub struct NetworkLink {
    pub peer: Entity,
    pub rssi_dbm: f32,
    pub connected: bool,
    pub antenna_idx: usize,
}

/// Aggregate health across all of this drone's active links.
#[derive(Component, Default)]
pub struct NetworkState {
    pub connected_peers: usize,
    pub best_rssi_dbm: f32,
}

// ─── Rust implementation (stub) ───────────────────────────────────────────────

pub struct RustNetwork;

impl NetworkLogic for RustNetwork {
    fn update(&mut self, _self_pos: Vec3, _peers: &[PeerInfo]) -> NetworkDecision {
        todo!(
            "Compute rssi_dbm(θ_tx, θ_rx, d) for each peer; \
             connect to peers above sensitivity; \
             elect role by comparing connected_peers count vs neighbours"
        )
    }
}

// ─── System stubs ─────────────────────────────────────────────────────────────

pub fn run_network_logic(
    _commands: Commands,
    _drones: Query<(Entity, &GlobalTransform, &crate::drone::Drone, &mut crate::factories::DroneAi, &mut NetworkState)>,
    _all_positions: Query<(Entity, &GlobalTransform), With<crate::drone::Drone>>,
    _links: Query<&mut NetworkLink>,
) {
    todo!(
        "For each drone: gather PeerInfo for all reachable neighbours \
         (rssi above sensitivity), call drone_ai.network.update(self_pos, peers), \
         insert/remove NetworkLink components, update NetworkState"
    );
}

// ─── Python bridge ────────────────────────────────────────────────────────────

#[cfg(feature = "python")]
pub struct PythonNetwork {
    instance: pyo3::PyObject,
}

#[cfg(feature = "python")]
impl PythonNetwork {
    /// `module` must expose a `DroneNetwork` class.
    pub fn new(module: &str) -> Self {
        use pyo3::prelude::*;
        let instance = Python::with_gil(|py| {
            py.import(module)
                .expect("python module not found")
                .getattr("DroneNetwork")
                .expect("DroneNetwork class not found")
                .call0()
                .expect("DroneNetwork() constructor failed")
                .into()
        });
        Self { instance }
    }
}

#[cfg(feature = "python")]
impl NetworkLogic for PythonNetwork {
    fn update(&mut self, self_pos: Vec3, peers: &[PeerInfo]) -> NetworkDecision {
        use pyo3::prelude::*;
        Python::with_gil(|py| {
            let peer_list: Vec<_> = peers
                .iter()
                .map(|p| {
                    pyo3::types::PyDict::new(py).tap(|d| {
                        d.set_item("entity", p.entity.index()).unwrap();
                        d.set_item("position", (p.position.x, p.position.y, p.position.z)).unwrap();
                        d.set_item("rssi_dbm", p.rssi_dbm).unwrap();
                    })
                })
                .collect();

            let result = self
                .instance
                .call_method1(
                    py,
                    "update",
                    ((self_pos.x, self_pos.y, self_pos.z), peer_list),
                )
                .expect("DroneNetwork.update() failed");

            let dict = result.downcast::<pyo3::types::PyDict>(py)
                .expect("DroneNetwork.update() must return a dict");

            let role_str: &str = dict.get_item("role").unwrap().unwrap().extract().unwrap();
            let role = match role_str {
                "Master" => NetworkRole::Master,
                "Relay"  => NetworkRole::Relay,
                _        => NetworkRole::Leaf,
            };

            // connect_to: list of entity index ints — caller must resolve to Entity
            let connect_to: Vec<Entity> = dict
                .get_item("connect_to").unwrap().unwrap()
                .extract::<Vec<u32>>().unwrap()
                .into_iter()
                .map(Entity::from_raw)
                .collect();

            NetworkDecision { connect_to, role }
        })
    }
}

#[cfg(feature = "python")]
trait Tap: Sized {
    fn tap(self, f: impl FnOnce(&Self)) -> Self { f(&self); self }
}
#[cfg(feature = "python")]
impl<T> Tap for T {}
