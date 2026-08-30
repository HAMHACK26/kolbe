use std::collections::HashMap;

use bevy::{picking::prelude::*, prelude::*};

use crate::{
    antenna::{Antenna, Antennas, angles_toward},
    base::{Base, CommandQueue},
    camera::OrbitCamera,
    drone::{Drone, SelectedDrone, drone_id, make_antenna},
    factories::{
        DroneAi,
        movement::{DroneKinematics, HoverWind},
    },
    networking::{LinkSet, MeshRow, MeshTable, NetworkingBundle},
    radar::{RadarCone, cone_mesh_for, cone_transform_for},
    recovery::{ContactMemory, RecoveryState},
    seeking::SeekState,
    theme::{Theme, ThemeRole},
    ui::{
        InfoPopup, InfoPopupTable, InfoPopupTitle, NetworkTableButton, NetworkTablePanelText,
        NetworkTablePopup,
    },
};

pub const WORLD_SIZE: f32 = 20.0;
pub const DRONE_RADIUS: f32 = 0.0225;
/// Radius of the visible drone sphere. Kept separate from collision geometry
/// so the marker can be tuned without changing flight behavior.
pub const DRONE_VISUAL_RADIUS: f32 = DRONE_RADIUS;
pub const MIN_WIND_INTENSITY: f32 = 0.0;
pub const MAX_WIND_INTENSITY: f32 = 20.0;
pub const WIND_INTENSITY_STEP: f32 = 1.0;
pub const DEFAULT_WIND_INTENSITY: f32 = 3.0;

/// Wind strength chosen on the setup screen. 0 disables wind, 1 is the
/// baseline disturbance, and values up to 20 progressively amplify it.
#[derive(Resource, Clone, Copy, Debug)]
pub struct WindSettings {
    pub intensity: f32,
}

impl Default for WindSettings {
    fn default() -> Self {
        Self { intensity: DEFAULT_WIND_INTENSITY }
    }
}
/// Clearance from the terrain to the underside of each drone, in km (50 m).
pub const DRONE_GROUND_CLEARANCE_KM: f32 = 0.05;
pub const DEPLOYMENT_INTERVAL_SECS: f32 = 30.0;
const DEPLOYMENT_BATCH_SIZE: usize = 3;

fn next_deployment_batch_size(remaining: usize) -> usize {
    if remaining == DEPLOYMENT_BATCH_SIZE + 1 {
        2
    } else {
        remaining.min(DEPLOYMENT_BATCH_SIZE)
    }
}
/// Keeps launches clear of the base and of each other before navigation has
/// had a chance to separate the formation.
const LAUNCH_RING_RADIUS_KM: f32 = 0.10;
/// Absolute communication limit for every protected relay hop.
pub const MAX_RELAY_HOP_KM: f32 = 3.0;
/// Navigation and integration target, leaving margin below the hard limit.
pub const RELAY_WORKING_HOP_KM: f32 = 2.75;
const RELAY_ACQUIRE_FRAMES: u8 = 3;
const RELAY_LOSS_GRACE_FRAMES: u8 = 3;

/// The individual destination assigned to a drone during deployment.
#[derive(Component)]
pub struct DeploymentTarget {
    pub ingress: Vec3,
    /// Stable, per-drone destination once ingress is complete. Keeping this
    /// stateful prevents the reactive spacing rule from flipping direction on
    /// consecutive mesh-table updates.
    pub slot: Vec3,
    pub spreading: bool,
}

#[derive(Resource)]
pub(crate) struct DeploymentQueue {
    target_slots: Vec<Vec3>,
    total_count: usize,
    next_index: usize,
    timer: Timer,
    base_entity: Entity,
    base_pos: Vec3,
    ingress: Vec3,
    drone_mesh: Handle<Mesh>,
    drone_mat: Handle<StandardMaterial>,
    cone_mat: Handle<StandardMaterial>,
}

/// Protected upstream links created by wave deployment.
///
/// Every drone has exactly one active parent closer to the base. A new wave
/// starts a pending handoff. Each previous-wave drone keeps its base parent
/// until its own replacement edge has been physically acquired and held stable.
#[derive(Resource, Default)]
pub(crate) struct RelayTopology {
    base: Option<Entity>,
    parents: HashMap<Entity, Entity>,
    waves: Vec<Vec<Entity>>,
    links: HashMap<RelayEdge, RelayLinkPhase>,
    pending_handoff: Option<Vec<RelayEdge>>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub(crate) struct RelayEdge {
    child: Entity,
    parent: Entity,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RelayLinkPhase {
    Searching,
    Acquiring { consecutive_frames: u8 },
    Tracking,
    Degraded { missed_frames: u8 },
    Reacquiring,
}

impl RelayTopology {
    pub(crate) fn register_wave(&mut self, base: Entity, wave: Vec<Entity>) {
        if wave.is_empty() {
            return;
        }
        assert!(self.pending_handoff.is_none(), "finish the current handoff first");
        self.base = Some(base);

        let previous = self.waves.last().cloned();
        for &drone in &wave {
            self.parents.insert(drone, base);
        }
        self.waves.push(wave);

        if let Some(previous) = previous {
            let newest = self.waves.last().expect("wave was just inserted");
            let pending: Vec<RelayEdge> = previous
                .into_iter()
                .enumerate()
                .map(|(index, child)| RelayEdge {
                    child,
                    parent: newest[index % newest.len()],
                })
                .collect();
            for &edge in &pending {
                self.links.insert(edge, RelayLinkPhase::Searching);
            }
            self.pending_handoff = Some(pending);
        }
    }

    pub(crate) fn parent(&self, drone: Entity) -> Option<Entity> {
        self.parents.get(&drone).copied()
    }

    pub(crate) fn requires_link(&self, a: Entity, b: Entity) -> bool {
        self.parent(a) == Some(b) || self.parent(b) == Some(a)
    }

    pub(crate) fn same_wave(&self, a: Entity, b: Entity) -> bool {
        self.waves.iter().any(|wave| wave.contains(&a) && wave.contains(&b))
    }

    pub(crate) fn involves_base(&self, a: Entity, b: Entity) -> bool {
        self.base.is_some_and(|base| a == base || b == base)
    }

    pub(crate) fn handoff_pending(&self) -> bool {
        self.pending_handoff.is_some()
    }

    #[cfg(test)]
    pub(crate) fn link_phase(&self, a: Entity, b: Entity) -> Option<RelayLinkPhase> {
        self.links
            .iter()
            .find_map(|(edge, phase)| edge.connects(a, b).then_some(*phase))
    }

    /// Search responsibility belongs to the newer, base-side endpoint while
    /// the older endpoint holds vector aim. This prevents two narrow beams
    /// from sweeping past one another indefinitely.
    pub(crate) fn search_phase(
        &self,
        entity: Entity,
        target: Entity,
    ) -> Option<RelayLinkPhase> {
        self.links.iter().find_map(|(edge, phase)| {
            (edge.parent == entity && edge.child == target).then_some(*phase)
        })
    }

    #[cfg(test)]
    pub(crate) fn should_spiral_search(&self, entity: Entity, target: Entity) -> bool {
        matches!(
            self.search_phase(entity, target),
            Some(RelayLinkPhase::Searching | RelayLinkPhase::Reacquiring)
        )
    }

    /// Stable antenna ownership for active and pending relay edges. Antenna 2
    /// (index 1) is reserved for a direct base parent; the remaining slots are
    /// assigned deterministically to drone peers.
    pub(crate) fn antenna_targets(&self, entity: Entity) -> Vec<(usize, Entity)> {
        let mut peers = Vec::new();
        if let Some(parent) = self.parent(entity) {
            peers.push(parent);
        }
        for (&child, &parent) in &self.parents {
            if parent == entity {
                peers.push(child);
            }
        }
        if let Some(pending) = &self.pending_handoff {
            for edge in pending {
                if edge.child == entity {
                    peers.push(edge.parent);
                } else if edge.parent == entity {
                    peers.push(edge.child);
                }
            }
        }
        peers.sort_by_key(|peer| peer.to_bits());
        peers.dedup();

        let base_peer = peers.iter().copied().find(|peer| Some(*peer) == self.base);
        let mut targets = Vec::new();
        if let Some(base) = base_peer {
            targets.push((1, base));
        }
        let slots: &[usize] = if base_peer.is_some() { &[0, 2] } else { &[0, 2, 1] };
        for (slot, peer) in slots.iter().copied().zip(
            peers.into_iter().filter(|peer| Some(*peer) != self.base),
        ) {
            targets.push((slot, peer));
        }
        targets
    }

    fn observe_link(&mut self, edge: RelayEdge, detected: bool) {
        let Some(phase) = self.links.get_mut(&edge) else {
            return;
        };
        *phase = match (*phase, detected) {
            (RelayLinkPhase::Searching | RelayLinkPhase::Reacquiring, true) => {
                RelayLinkPhase::Acquiring { consecutive_frames: 1 }
            }
            (RelayLinkPhase::Acquiring { consecutive_frames }, true)
                if consecutive_frames + 1 >= RELAY_ACQUIRE_FRAMES => RelayLinkPhase::Tracking,
            (RelayLinkPhase::Acquiring { consecutive_frames }, true) => {
                RelayLinkPhase::Acquiring { consecutive_frames: consecutive_frames + 1 }
            }
            (RelayLinkPhase::Tracking, false) => RelayLinkPhase::Degraded { missed_frames: 1 },
            (RelayLinkPhase::Degraded { .. }, true) => RelayLinkPhase::Tracking,
            (RelayLinkPhase::Degraded { missed_frames }, false)
                if missed_frames + 1 >= RELAY_LOSS_GRACE_FRAMES => RelayLinkPhase::Reacquiring,
            (RelayLinkPhase::Degraded { missed_frames }, false) => {
                RelayLinkPhase::Degraded { missed_frames: missed_frames + 1 }
            }
            (RelayLinkPhase::Acquiring { .. }, false) => RelayLinkPhase::Searching,
            (phase, _) => phase,
        };
    }

    fn complete_handoff_if_ready(&mut self) -> bool {
        let Some(pending) = self.pending_handoff.as_ref() else {
            return false;
        };
        let ready: Vec<RelayEdge> = pending
            .iter()
            .copied()
            .filter(|edge| self.links.get(edge) == Some(&RelayLinkPhase::Tracking))
            .collect();
        if ready.is_empty() {
            return false;
        }
        let pending = self.pending_handoff.take().expect("pending was checked");
        for edge in ready.iter() {
            self.parents.insert(edge.child, edge.parent);
        }
        let remaining: Vec<RelayEdge> = pending
            .into_iter()
            .filter(|edge| !ready.contains(edge))
            .collect();
        if !remaining.is_empty() {
            self.pending_handoff = Some(remaining);
        }
        true
    }

    #[cfg(test)]
    fn all_paths_reach_base(&self) -> bool {
        let Some(base) = self.base else {
            return self.parents.is_empty();
        };
        self.parents.keys().all(|&drone| {
            let mut cursor = drone;
            for _ in 0..=self.parents.len() {
                if cursor == base {
                    return true;
                }
                let Some(parent) = self.parent(cursor) else {
                    return false;
                };
                cursor = parent;
            }
            false
        })
    }
}

impl RelayEdge {
    #[cfg(test)]
    fn connects(self, a: Entity, b: Entity) -> bool {
        (self.child == a && self.parent == b) || (self.child == b && self.parent == a)
    }
}

/// Advance every required drone-to-drone edge through the same acquisition,
/// tracking, degradation, and reacquisition lifecycle. Each pending edge is
/// promoted independently as soon as it is stably tracking.
pub fn update_relay_link_lifecycle(
    mut topology: ResMut<RelayTopology>,
    link_sets: Query<&LinkSet>,
) {
    let edges: Vec<RelayEdge> = topology.links.keys().copied().collect();
    for edge in edges {
        let detected = link_sets
            .get(edge.child)
            .is_ok_and(|links| links.connected.contains_key(&edge.parent))
            && link_sets
                .get(edge.parent)
                .is_ok_and(|links| links.connected.contains_key(&edge.child));
        topology.observe_link(edge, detected);
    }
    topology.complete_handoff_if_ready();
}

/// Radius each drone's coverage footprint must reach.
pub const FORMATION_RADIUS_KM: f32 = 3.0;
/// Fleet multiplier applied after computing the minimum gap-free coverage grid.
const COVERAGE_RESERVE: f32 = 1.75;

/// Area of the blue, operator-selected target polygon in km². The orange
/// square is deliberately excluded: it only exists to fetch terrain.
fn blue_target_area_km2(area: &crate::area::NetworkArea) -> f32 {
    if area.hull.len() < 3 {
        return 0.0;
    }
    let (center_lon, center_lat) = area.center;
    let points: Vec<Vec2> = area
        .hull
        .iter()
        .map(|&(lon, lat)| Vec2::new(
            ((lon - center_lon) * 111.320 * center_lat.to_radians().cos()) as f32,
            ((lat - center_lat) * 110.574) as f32,
        ))
        .collect();
    points.iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(a, b)| a.x * b.y - a.y * b.x)
        .sum::<f32>()
        .abs()
        * 0.5
}

/// Balanced grid dimensions for the fleet calculated from the blue target.
fn coverage_grid_dimensions(area: &crate::area::NetworkArea) -> (usize, usize) {
    let count = target_area_drone_count(area);
    let columns = (count as f32).sqrt().ceil() as usize;
    (columns, count.div_ceil(columns))
}

/// Number of drones for the blue target's 3 km coverage cells, including 75%
/// reserve. A radius-3 km circle's gap-free square cell is 3√2 km wide, or
/// 18 km², so this uses the selected polygon's area rather than its orange
/// bounding square.
pub fn target_area_drone_count(area: &crate::area::NetworkArea) -> usize {
    let safe_cell_area = 2.0 * FORMATION_RADIUS_KM.powi(2);
    ((blue_target_area_km2(area) / safe_cell_area) * COVERAGE_RESERVE)
        .ceil()
        .max(5.0) as usize
}

/// Extra rear waves needed so even the last area-assigned wave can reach its
/// furthest slot without stretching any protected hop past its working range.
fn relay_reserve_drone_count(base_pos: Vec3, target_slots: &[Vec3]) -> usize {
    let furthest_slot_km = target_slots
        .iter()
        .map(|slot| slot.distance(base_pos))
        .fold(0.0_f32, f32::max);
    let required_hops = (furthest_slot_km / RELAY_WORKING_HOP_KM).ceil() as usize;
    required_hops.saturating_sub(1) * DEPLOYMENT_BATCH_SIZE
}

fn launch_position(base_pos: Vec3, index: usize, count: usize) -> Vec3 {
    let count = count.max(1);
    let angle = index as f32 / count as f32 * std::f32::consts::TAU;
    // Expand the launch ring for larger calculated fleets so adjacent pads
    // still begin outside one another's collision hulls.
    let required_separation = DRONE_RADIUS * 2.0 + crate::avoidance::SENSOR_RANGE_KM + 0.001;
    let minimum_radius = required_separation
        / (2.0 * (std::f32::consts::PI / count as f32).sin()).max(f32::EPSILON);
    let radius = LAUNCH_RING_RADIUS_KM.max(minimum_radius);
    base_pos + Vec3::new(
        radius * angle.sin(),
        0.0,
        radius * angle.cos(),
    )
}

/// Evenly distribute area assignments across a gap-free 3 km coverage grid.
/// Relay constraints are layered onto these fixed survey cells during flight.
pub fn target_area_formation(
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
    terrain: &crate::terrain::TerrainHeightMap,
) -> Vec<Vec3> {
    let (lon, lat) = area.center;
    let center_x = ((lon - scenario.longitude) * 111.320 * scenario.latitude.to_radians().cos()) as f32;
    let center_z = ((lat - scenario.latitude) * 110.574) as f32;
    let (columns, rows) = coverage_grid_dimensions(area);
    let cell_width = area.side_km as f32 / columns as f32;
    let cell_height = area.side_km as f32 / rows as f32;
    let half_side = area.side_km as f32 * 0.5;
    (0..rows)
        .flat_map(|row| (0..columns).map(move |column| (column, row)))
        .take(target_area_drone_count(area))
        .map(|(column, row)| {
            let x = center_x - half_side + (column as f32 + 0.5) * cell_width;
            let z = center_z - half_side + (row as f32 + 0.5) * cell_height;
            Vec3::new(
                x,
                terrain.height_at(x, z) + DRONE_GROUND_CLEARANCE_KM + DRONE_RADIUS,
                z,
            )
        })
        .collect()
}

/// Center of the selected target area in the simulation's local kilometer
/// coordinate frame.
pub fn target_area_center(
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
    terrain: &crate::terrain::TerrainHeightMap,
) -> Vec3 {
    let (lon, lat) = area.center;
    let x = ((lon - scenario.longitude) * 111.320 * scenario.latitude.to_radians().cos()) as f32;
    let z = ((lat - scenario.latitude) * 110.574) as f32;
    Vec3::new(
        x,
        terrain.height_at(x, z) + DRONE_GROUND_CLEARANCE_KM + DRONE_RADIUS,
        z,
    )
}

/// The three antennas drone `i` spawns with, already aimed at its mesh
/// targets: #1 at the next ring neighbor, #2 at the base, #3 at the previous
/// neighbor.
///
/// Aiming them here is what makes the mesh live on frame 0 rather than after a
/// spiral search stumbles onto a 1°-wide beam. It is the same slot layout
/// [`crate::tracking::maintain_mesh_antennas`] maintains once the drones move.
///
/// [`angles_toward`] gives a world-frame bearing while `Antenna::azimuth_deg`
/// is heading-relative; a drone spawns with `DroneKinematics::default()`
/// (heading 0), so at spawn the two frames coincide.
pub fn formation_antennas(ring: &[Vec3], i: usize, base_pos: Vec3) -> Vec<Antenna> {
    let self_pos = ring[i];
    let next = ring[(i + 1) % ring.len()];
    let prev = ring[(i + ring.len() - 1) % ring.len()];
    let (az_next, el_next) = angles_toward(self_pos, next);
    let (az_base, el_base) = angles_toward(self_pos, base_pos);
    let (az_prev, el_prev) = angles_toward(self_pos, prev);
    vec![
        make_antenna(az_next, el_next, i),
        make_antenna(az_base, el_base, i + 100),
        make_antenna(az_prev, el_prev, i + 200),
    ]
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
    theme: Res<Theme>,
    wind: Res<WindSettings>,
    bases: Query<(Entity, &Base)>,
    network_area: Res<crate::area::NetworkArea>,
    scenario: Res<crate::area::ScenarioArea>,
) {
    let pal = theme.palette();
    // `AmbientLight` is a per-camera override of `GlobalAmbientLight` and so
    // `#[require(Camera)]`s one. Spawned on its own it lights nothing and leaves
    // a camera entity with no render graph, which Bevy warns about every run.
    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        AmbientLight { brightness: 300.0, ..default() },
        crate::SimulationEntity,
    ));

    commands.spawn((
        DirectionalLight { illuminance: 8000.0, shadow_maps_enabled: false, ..default() },
        Transform::from_xyz(8.0, 16.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        crate::SimulationEntity,
    ));

    let drone_mesh = meshes.add(Sphere::new(DRONE_VISUAL_RADIUS));
    // Initial colors come from the palette; `apply_theme` keeps them in sync on
    // theme toggles (these entities carry ThemeRole markers).
    let drone_mat = materials.add(StandardMaterial {
        base_color: pal.drone,
        emissive: LinearRgba::new(2.0, 0.0, 0.0, 1.0),
        ..default()
    });
    let cone_mat = materials.add(StandardMaterial {
        base_color: pal.base.with_alpha(0.30),
        emissive: LinearRgba::new(0.6, 0.5, 0.0, 0.0),
        alpha_mode: AlphaMode::Blend,
        double_sided: true,
        cull_mode: None,
        ..default()
    });

    // The base is the launch position. Drones deploy in batches of three,
    // then spread toward individual formation slots.
    let (base_entity, base) = bases.iter().next().expect("base is spawned before world setup");
    let base_pos = base.position;
    let target_slots = target_area_formation(&network_area, &scenario, &terrain);
    let total_count = target_slots.len() + relay_reserve_drone_count(base_pos, &target_slots);
    let ingress = target_area_center(&network_area, &scenario, &terrain);
    let initial_count = total_count.min(DEPLOYMENT_BATCH_SIZE);
    let mut initial_wave = Vec::with_capacity(initial_count);
    let initial_snapshot = HashMap::new();
    for index in 0..initial_count {
        initial_wave.push(spawn_deployment_drone(
            &mut commands,
            &mut meshes,
            &drone_mesh,
            &drone_mat,
            &cone_mat,
            launch_position(base_pos, index, DEPLOYMENT_BATCH_SIZE),
            base_pos,
            ingress,
            &target_slots,
            &initial_snapshot,
            index,
            wind.intensity,
        ));
    }
    let mut relay_topology = RelayTopology::default();
    relay_topology.register_wave(base_entity, initial_wave);
    commands.insert_resource(relay_topology);
    commands.insert_resource(DeploymentQueue {
        target_slots,
        total_count,
        next_index: initial_count,
        timer: Timer::from_seconds(DEPLOYMENT_INTERVAL_SECS, TimerMode::Repeating),
        base_entity,
        base_pos,
        ingress,
        drone_mesh,
        drone_mat,
        cone_mat,
    });

    // Info popup
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                padding: UiRect::axes(Val::Px(10.0), Val::Px(8.0)),
                border_radius: BorderRadius::all(Val::Px(4.0)),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(2.0),
                ..default()
            },
            BackgroundColor(pal.surface.with_alpha(0.88)),
            Visibility::Hidden,
            InfoPopup,
            crate::SimulationEntity,
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(""),
                TextFont { font_size: FontSize::Px(13.0), ..default() },
                TextColor(pal.text),
                InfoPopupTitle,
            ));
            p.spawn((
                Node {
                    display: Display::Grid,
                    grid_template_columns: vec![RepeatedGridTrack::auto(3)],
                    column_gap: Val::Px(10.0),
                    row_gap: Val::Px(2.0),
                    margin: UiRect::top(Val::Px(4.0)),
                    ..default()
                },
                InfoPopupTable,
            ));
            p.spawn((
                Button,
                Node {
                    margin: UiRect::top(Val::Px(6.0)),
                    padding: UiRect::axes(Val::Px(6.0), Val::Px(3.0)),
                    border_radius: BorderRadius::all(Val::Px(3.0)),
                    align_self: AlignSelf::FlexStart,
                    ..default()
                },
                BackgroundColor(pal.text.with_alpha(0.12)),
                NetworkTableButton,
            ))
            .observe(
                |mut t: On<Pointer<Click>>, mut open: ResMut<crate::ui::NetworkTablePanelOpen>| {
                    t.propagate(false);
                    open.0 = !open.0;
                },
            )
            .with_children(|b| {
                b.spawn((
                    Text::new("View Network Table"),
                    TextFont { font_size: FontSize::Px(11.0), ..default() },
                    TextColor(pal.text),
                ));
            });
        });

    // Network table window — a separate popup, same look as the info popup.
    // Its lifecycle is tied to the info popup (see `update_popup_position`):
    // closing the info popup closes this too.
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                padding: UiRect::axes(Val::Px(10.0), Val::Px(8.0)),
                border_radius: BorderRadius::all(Val::Px(4.0)),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(2.0),
                max_width: Val::Px(280.0),
                ..default()
            },
            BackgroundColor(pal.surface.with_alpha(0.88)),
            Visibility::Hidden,
            NetworkTablePopup,
            crate::SimulationEntity,
        ))
        .with_children(|p| {
            p.spawn((
                Text::new("Network Table"),
                TextFont { font_size: FontSize::Px(13.0), ..default() },
                TextColor(pal.text),
            ));
            p.spawn((
                Text::new(""),
                TextFont { font_size: FontSize::Px(11.0), ..default() },
                TextColor(pal.subtext),
                NetworkTablePanelText,
            ));
        });
}

/// Give a newly launched drone the current network picture maintained by the
/// base station. Rebase row ages onto the new drone's independent clock so
/// spiral uncertainty starts at launch instead of comparing unrelated epochs.
fn networking_with_launch_briefing(
    ring_index: usize,
    base_snapshot: &HashMap<String, MeshRow>,
) -> NetworkingBundle {
    let mut networking = NetworkingBundle::random(ring_index);
    let local_now = networking.radio.clock.now;
    networking.radio.mesh_table.0 = base_snapshot
        .iter()
        .map(|(id, row)| {
            let mut row = row.clone();
            row.timestamp = local_now;
            row.neighbour_distance = row.neighbour_distance.saturating_add(1);
            (id.clone(), row)
        })
        .collect();
    networking
}

/// Launch the next batch of up to three nodes every thirty seconds until the
/// initial deployment queue is exhausted.
pub fn spawn_next_drone(
    time: Res<Time>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut deployment: ResMut<DeploymentQueue>,
    mut relay_topology: ResMut<RelayTopology>,
    base_tables: Query<&MeshTable, With<Base>>,
    wind: Res<WindSettings>,
) {
    if deployment.next_index >= deployment.total_count
        || relay_topology.handoff_pending()
        || !deployment.timer.tick(time.delta()).just_finished()
    {
        return;
    }

    let base_snapshot = base_tables
        .get(deployment.base_entity)
        .map(|table| table.0.clone())
        .unwrap_or_default();
    let remaining = deployment.total_count - deployment.next_index;
    let end = deployment.next_index + next_deployment_batch_size(remaining);
    let mut wave = Vec::with_capacity(end - deployment.next_index);
    for index in deployment.next_index..end {
        wave.push(spawn_deployment_drone(
            &mut commands,
            &mut meshes,
            &deployment.drone_mesh.clone(),
            &deployment.drone_mat.clone(),
            &deployment.cone_mat.clone(),
            launch_position(deployment.base_pos, index % DEPLOYMENT_BATCH_SIZE, DEPLOYMENT_BATCH_SIZE),
            deployment.base_pos,
            deployment.ingress,
            &deployment.target_slots,
            &base_snapshot,
            index,
            wind.intensity,
        ));
    }
    deployment.next_index = end;
    relay_topology.register_wave(deployment.base_entity, wave);
}

fn spawn_deployment_drone(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    drone_mesh: &Handle<Mesh>,
    drone_mat: &Handle<StandardMaterial>,
    cone_mat: &Handle<StandardMaterial>,
    launch_pos: Vec3,
    base_pos: Vec3,
    ingress: Vec3,
    target_slots: &[Vec3],
    base_snapshot: &HashMap<String, MeshRow>,
    index: usize,
    wind_intensity: f32,
) -> Entity {
    let slot_index = index % target_slots.len();
    let antennas = formation_antennas(target_slots, slot_index, base_pos);
    let drone_entity = commands
        .spawn((
            Mesh3d(drone_mesh.clone()),
            MeshMaterial3d(drone_mat.clone()),
            Transform::from_translation(launch_pos),
            Drone { id: drone_id(index) },
            Antennas(antennas.clone()),
            DeploymentTarget {
                ingress,
                slot: target_slots[slot_index],
                spreading: false,
            },
            DroneKinematics::default(),
            DroneAi::default(),
            CommandQueue::default(),
            networking_with_launch_briefing(index, base_snapshot),
            SeekState::default(),
            RecoveryState::default(),
            ContactMemory::default(),
            ThemeRole::Drone,
            crate::SimulationEntity,
        ))
        .insert(HoverWind::new(index, wind_intensity))
        .observe(
            |mut t: On<Pointer<Click>>, orbit: Res<OrbitCamera>, mut sel: ResMut<SelectedDrone>| {
                t.propagate(false);
                if orbit.drag_total < 5.0 {
                    sel.0 = Some(t.original_event_target());
                }
            },
        )
        .id();

    for (antenna_index, antenna) in antennas.iter().enumerate() {
        commands.spawn((
            Mesh3d(cone_mesh_for(antenna, meshes)),
            MeshMaterial3d(cone_mat.clone()),
            cone_transform_for(antenna, 0.0, base_pos),
            Visibility::Hidden,
            RadarCone { drone_entity, antenna_index },
            ThemeRole::DroneCone,
            crate::SimulationEntity,
        ));
    }
    drone_entity
}

fn clamp_to_relay_parent(position: Vec3, parent_position: Vec3) -> Vec3 {
    let offset = position - parent_position;
    let distance = offset.length();
    if distance <= RELAY_WORKING_HOP_KM || distance <= f32::EPSILON {
        position
    } else {
        parent_position + offset / distance * RELAY_WORKING_HOP_KM
    }
}

/// Final hard guard for the protected relay tree. Connectivity outranks wind,
/// collision avoidance, and the target geofence.
pub fn enforce_relay_hops(
    topology: Res<RelayTopology>,
    mut queries: ParamSet<(
        Query<(Entity, &Transform), Or<(With<Drone>, With<Base>)>>,
        Query<(Entity, &mut Transform, &mut DroneKinematics), With<Drone>>,
    )>,
) {
    let positions: HashMap<Entity, Vec3> = queries
        .p0()
        .iter()
        .map(|(entity, transform)| (entity, transform.translation))
        .collect();
    let mut corrected = positions.clone();

    // Resolve from the base-connected rear wave outward. Each child therefore
    // clamps against its parent's corrected position, even when several hops
    // overshoot during the same frame.
    for wave in topology.waves.iter().rev() {
        for &entity in wave {
            let Some(parent_position) = topology
                .parent(entity)
                .and_then(|parent| corrected.get(&parent).copied())
            else {
                continue;
            };
            let Some(position) = corrected.get(&entity).copied() else {
                continue;
            };
            corrected.insert(entity, clamp_to_relay_parent(position, parent_position));
        }
    }

    for (entity, mut transform, mut kinematics) in &mut queries.p1() {
        let Some(parent_position) = topology
            .parent(entity)
            .and_then(|parent| corrected.get(&parent).copied())
        else {
            kinematics.velocity = Vec3::ZERO;
            continue;
        };
        let before = transform.translation;
        let Some(clamped) = corrected.get(&entity).copied() else {
            kinematics.velocity = Vec3::ZERO;
            continue;
        };
        if clamped == before {
            continue;
        }
        let outward = (before - parent_position).normalize_or_zero();
        transform.translation = clamped;
        let commanded_outward = kinematics.velocity.dot(outward).max(0.0);
        let flown_outward = kinematics.flown_velocity.dot(outward).max(0.0);
        kinematics.velocity -= outward * commanded_outward;
        kinematics.flown_velocity -= outward * flown_outward;
    }
}

pub fn draw_grid(
    mut gizmos: Gizmos,
    theme: Res<Theme>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
) {
    // Scaled to the *actual* fetched terrain, not the fixed `WORLD_SIZE` the
    // hand-placed drone ring is designed for — a hardcoded 5km step only
    // covered a 20km world; anything bigger left the outer terrain grid-less.
    let half = terrain.size_km() * 0.5;
    let step = terrain.size_km() / 4.0; // 5 lines (0..=4) spanning the full terrain
    let color = theme.palette().grid.with_alpha(0.25);
    const SEGMENTS: usize = 64;
    for i in 0..=4 {
        let offset = -half + i as f32 * step;
        for segment in 0..SEGMENTS {
            let a = -half + terrain.size_km() * segment as f32 / SEGMENTS as f32;
            let b = -half + terrain.size_km() * (segment + 1) as f32 / SEGMENTS as f32;
            gizmos.line(
                Vec3::new(a, terrain.height_at(a, offset) + 0.01, offset),
                Vec3::new(b, terrain.height_at(b, offset) + 0.01, offset),
                color,
            );
            gizmos.line(
                Vec3::new(offset, terrain.height_at(offset, a) + 0.01, a),
                Vec3::new(offset, terrain.height_at(offset, b) + 0.01, b),
                color,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn launch_pads_clear_each_other_and_the_base() {
        let base = Vec3::ZERO;
        let count = DEPLOYMENT_BATCH_SIZE;
        let pads: Vec<Vec3> = (0..count)
            .map(|index| launch_position(base, index, count))
            .collect();
        for pad in &pads {
            assert!(pad.xz().length() >= LAUNCH_RING_RADIUS_KM);
        }
        for (index, pad) in pads.iter().enumerate() {
            for other in pads.iter().skip(index + 1) {
                assert!(
                    pad.xz().distance(other.xz())
                        >= DRONE_RADIUS * 2.0 + crate::avoidance::SENSOR_RANGE_KM
                );
            }
        }
    }

    #[test]
    fn launch_briefing_seeds_previous_wave_at_the_new_drones_local_time() {
        let previous_id = String::default();
        let snapshot = HashMap::from([(
            previous_id.clone(),
            crate::networking::MeshRow {
                id: previous_id.clone(),
                timestamp: 123.0,
                location: Vec3::new(1.5, 0.0, 0.5),
                neighbour_distance: 0,
                connections: Vec::new(),
            },
        )]);

        let networking = networking_with_launch_briefing(3, &snapshot);
        let inherited = networking.radio.mesh_table.0.get(&previous_id).unwrap();

        assert_eq!(inherited.location, Vec3::new(1.5, 0.0, 0.5));
        assert_eq!(inherited.timestamp, networking.radio.clock.now);
        assert_eq!(inherited.neighbour_distance, 1);
    }

    #[test]
    fn coverage_count_uses_the_blue_polygon_not_its_orange_square() {
        let area = crate::area::NetworkArea {
            side_km: 6.0,
            valid: true,
            center: (0.0, 0.0),
            hull: vec![
                (-3.0 / 111.320, -3.0 / 110.574),
                (3.0 / 111.320, -3.0 / 110.574),
                (3.0 / 111.320, 3.0 / 110.574),
                (-3.0 / 111.320, 3.0 / 110.574),
            ],
            ..default()
        };
        let (columns, rows) = coverage_grid_dimensions(&area);
        // The computed four is raised to the five-drone fleet minimum.
        assert_eq!(target_area_drone_count(&area), 5);
        assert_eq!((columns, rows), (3, 2));
    }

    #[test]
    fn wave_handoff_promotes_each_stable_replacement_link() {
        let entity = |id| Entity::from_raw_u32(id).expect("valid test entity");
        let base = entity(1);
        let first = vec![entity(10), entity(11), entity(12)];
        let second = vec![entity(20), entity(21)];
        let mut topology = RelayTopology::default();

        topology.register_wave(base, first.clone());
        topology.register_wave(base, second.clone());
        let pending = topology.pending_handoff.clone().expect("handoff is pending");

        assert!(topology.handoff_pending());
        assert!(first.iter().all(|&drone| topology.requires_link(drone, base)));
        assert!(second.iter().all(|&drone| topology.requires_link(drone, base)));
        assert!(topology.should_spiral_search(second[0], first[0]));
        assert!(!topology.should_spiral_search(first[0], second[0]));

        for _ in 0..RELAY_ACQUIRE_FRAMES {
            topology.observe_link(pending[0], true);
        }
        assert!(topology.complete_handoff_if_ready());
        assert_eq!(topology.parent(first[0]), Some(second[0]));
        assert!(!topology.requires_link(first[0], base));
        assert!(
            first[1..]
                .iter()
                .all(|&drone| topology.requires_link(drone, base))
        );
        assert!(topology.handoff_pending());
        assert!(topology.all_paths_reach_base());

        for &edge in pending.iter().skip(1) {
            for _ in 0..RELAY_ACQUIRE_FRAMES {
                topology.observe_link(edge, true);
            }
        }
        assert!(topology.complete_handoff_if_ready());
        assert_eq!(topology.parent(first[0]), Some(second[0]));
        assert_eq!(topology.parent(first[1]), Some(second[1]));
        assert_eq!(topology.parent(first[2]), Some(second[0]));
        assert!(first.iter().all(|&drone| !topology.requires_link(drone, base)));
        assert!(topology.all_paths_reach_base());
    }

    #[test]
    fn lifecycle_system_requires_bidirectional_detection_before_handoff() {
        let mut app = App::new();
        let base = app.world_mut().spawn_empty().id();
        let older = app.world_mut().spawn(LinkSet::default()).id();
        let newer = app.world_mut().spawn(LinkSet::default()).id();
        let mut topology = RelayTopology::default();
        topology.register_wave(base, vec![older]);
        topology.register_wave(base, vec![newer]);
        app.insert_resource(topology);
        app.add_systems(Update, update_relay_link_lifecycle);

        app.world_mut()
            .entity_mut(older)
            .get_mut::<LinkSet>()
            .unwrap()
            .connected
            .insert(newer, 0.0);
        for _ in 0..RELAY_ACQUIRE_FRAMES {
            app.update();
        }
        assert_eq!(app.world().resource::<RelayTopology>().parent(older), Some(base));

        app.world_mut()
            .entity_mut(newer)
            .get_mut::<LinkSet>()
            .unwrap()
            .connected
            .insert(older, 0.0);
        for _ in 0..RELAY_ACQUIRE_FRAMES {
            app.update();
        }
        let topology = app.world().resource::<RelayTopology>();
        assert_eq!(topology.parent(older), Some(newer));
        assert!(!topology.handoff_pending());
    }

    #[test]
    fn established_relay_links_reenter_search_after_sustained_loss() {
        let entity = |id| Entity::from_raw_u32(id).expect("valid test entity");
        let base = entity(1);
        let older = entity(10);
        let newer = entity(20);
        let mut topology = RelayTopology::default();
        topology.register_wave(base, vec![older]);
        topology.register_wave(base, vec![newer]);
        let edge = topology.pending_handoff.as_ref().unwrap()[0];

        for _ in 0..RELAY_ACQUIRE_FRAMES {
            topology.observe_link(edge, true);
        }
        topology.complete_handoff_if_ready();
        assert_eq!(topology.link_phase(older, newer), Some(RelayLinkPhase::Tracking));

        for _ in 0..RELAY_LOSS_GRACE_FRAMES {
            topology.observe_link(edge, false);
        }
        assert_eq!(topology.link_phase(older, newer), Some(RelayLinkPhase::Reacquiring));
        assert!(topology.should_spiral_search(newer, older));

        for _ in 0..RELAY_ACQUIRE_FRAMES {
            topology.observe_link(edge, true);
        }
        assert_eq!(topology.link_phase(older, newer), Some(RelayLinkPhase::Tracking));
    }

    #[test]
    fn pending_partial_wave_uses_each_antenna_slot_at_most_once() {
        let entity = |id| Entity::from_raw_u32(id).expect("valid test entity");
        let base = entity(1);
        let first = vec![entity(10), entity(11), entity(12)];
        let second = vec![entity(20), entity(21)];
        let mut topology = RelayTopology::default();
        topology.register_wave(base, first);
        topology.register_wave(base, second.clone());

        let targets = topology.antenna_targets(second[0]);
        let mut slots: Vec<usize> = targets.iter().map(|(slot, _)| *slot).collect();
        slots.sort_unstable();
        slots.dedup();
        assert_eq!(targets.len(), 3);
        assert_eq!(slots, vec![0, 1, 2]);
    }

    #[test]
    fn deployment_timer_fires_once_every_thirty_seconds() {
        let mut timer = Timer::from_seconds(DEPLOYMENT_INTERVAL_SECS, TimerMode::Repeating);
        assert!(!timer.tick(Duration::from_secs(29)).just_finished());
        assert!(timer.tick(Duration::from_secs(1)).just_finished());
        assert!(!timer.tick(Duration::from_secs(29)).just_finished());
        assert!(timer.tick(Duration::from_secs(1)).just_finished());
    }

    #[test]
    fn four_remaining_drones_split_into_two_two_drone_waves() {
        assert_eq!(next_deployment_batch_size(5), 3);
        assert_eq!(next_deployment_batch_size(4), 2);
        assert_eq!(next_deployment_batch_size(2), 2);
        assert_eq!(next_deployment_batch_size(3), 3);
    }

    #[test]
    fn distant_targets_add_only_the_required_relay_waves() {
        let base = Vec3::ZERO;
        assert_eq!(relay_reserve_drone_count(base, &[Vec3::new(2.7, 0.0, 0.0)]), 0);
        assert_eq!(relay_reserve_drone_count(base, &[Vec3::new(5.0, 0.0, 0.0)]), 3);
        assert_eq!(relay_reserve_drone_count(base, &[Vec3::new(8.0, 0.0, 0.0)]), 6);
    }

    #[test]
    fn relay_clamp_preserves_margin_below_three_kilometres() {
        let parent = Vec3::ZERO;
        let clamped = clamp_to_relay_parent(Vec3::new(10.0, 0.0, 0.0), parent);
        assert!((clamped.distance(parent) - RELAY_WORKING_HOP_KM).abs() < 1e-6);
        assert!(clamped.distance(parent) < MAX_RELAY_HOP_KM);
    }

    #[test]
    fn relay_guard_clamps_multiple_overshooting_hops_base_outward() {
        let mut app = App::new();
        let base = app
            .world_mut()
            .spawn((
                Base { id: "base".into(), position: Vec3::ZERO, antennas: Vec::new() },
                Transform::default(),
            ))
            .id();
        let front = app
            .world_mut()
            .spawn((
                Drone { id: "front".into() },
                Transform::from_xyz(12.0, 0.0, 0.0),
                DroneKinematics::default(),
            ))
            .id();
        let rear = app
            .world_mut()
            .spawn((
                Drone { id: "rear".into() },
                Transform::from_xyz(10.0, 0.0, 0.0),
                DroneKinematics::default(),
            ))
            .id();
        let mut topology = RelayTopology::default();
        topology.register_wave(base, vec![front]);
        topology.register_wave(base, vec![rear]);
        let pending = topology.pending_handoff.clone().unwrap();
        for _ in 0..RELAY_ACQUIRE_FRAMES {
            topology.observe_link(pending[0], true);
        }
        assert!(topology.complete_handoff_if_ready());
        app.insert_resource(topology);
        app.add_systems(Update, enforce_relay_hops);

        app.update();

        let front_position = app.world().entity(front).get::<Transform>().unwrap().translation;
        let rear_position = app.world().entity(rear).get::<Transform>().unwrap().translation;
        assert!(rear_position.distance(Vec3::ZERO) <= RELAY_WORKING_HOP_KM);
        assert!(front_position.distance(rear_position) <= RELAY_WORKING_HOP_KM);
    }

    #[test]
    fn relay_guard_system_clamps_the_real_transform() {
        let mut app = App::new();
        let base = app
            .world_mut()
            .spawn((
                Base { id: "base".into(), position: Vec3::ZERO, antennas: Vec::new() },
                Transform::default(),
            ))
            .id();
        let drone = app
            .world_mut()
            .spawn((
                Drone { id: "drone".into() },
                Transform::from_xyz(10.0, 0.0, 0.0),
                DroneKinematics::default(),
            ))
            .id();
        let mut topology = RelayTopology::default();
        topology.register_wave(base, vec![drone]);
        app.insert_resource(topology);
        app.add_systems(Update, enforce_relay_hops);

        app.update();

        let position = app.world().entity(drone).get::<Transform>().unwrap().translation;
        assert!((position.length() - RELAY_WORKING_HOP_KM).abs() < 1e-6);
    }
}
