use bevy::{picking::prelude::*, prelude::*};

use crate::{
    antenna::{Antenna, Antennas, angles_toward},
    base::{Base, CommandQueue},
    camera::OrbitCamera,
    drone::{Drone, SelectedDrone, drone_id, make_antenna},
    factories::{DroneAi, movement::DroneKinematics},
    networking::NetworkingBundle,
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
/// Clearance from the terrain to the underside of each drone, in km (50 m).
pub const DRONE_GROUND_CLEARANCE_KM: f32 = 0.05;
const DEPLOYMENT_INTERVAL_SECS: f32 = 10.0;
const DEPLOYMENT_BATCH_SIZE: usize = 3;
/// Keeps launches clear of the base and of each other before navigation has
/// had a chance to separate the formation.
const LAUNCH_RING_RADIUS_KM: f32 = 0.10;

/// Per-drone deployment and in-target survey state.
#[derive(Component)]
pub struct DeploymentTarget {
    pub ingress: Vec3,
    pub spreading: bool,
    /// Per-drone survey heading used after the drone enters the blue target.
    pub bias_direction: Vec2,
    pub bias_seed: u32,
    pub bias_elapsed_secs: f32,
    pub next_bias_change_secs: f32,
    pub bias_window: u32,
}

/// Deterministic UUID-seeded survey heading. Keeping this pure makes every
/// simulation repeatable while ensuring drones do not share a direction.
fn bias_direction(seed: usize, window: u32) -> Vec2 {
    let mut value = (seed as u32).wrapping_mul(0x9e37_79b9) ^ window.wrapping_mul(0x85eb_ca6b);
    value ^= value >> 16;
    value = value.wrapping_mul(0x7feb_352d);
    value ^= value >> 15;
    let angle = value as f32 / u32::MAX as f32 * std::f32::consts::TAU;
    Vec2::new(angle.sin(), angle.cos())
}

fn initial_bias_delay_secs(seed: usize) -> f32 {
    // The UUID seed staggers the first change over the first two minutes;
    // following changes stay exactly two minutes apart for that drone.
    ((seed as u32).wrapping_mul(0x9e37_79b9) % 121) as f32
}

fn uuid_bias_seed(id: &str) -> usize {
    id.bytes().fold(0x811c_9dc5_u32, |hash, byte| {
        (hash ^ byte as u32).wrapping_mul(0x0100_0193)
    }) as usize
}

#[derive(Resource)]
pub(crate) struct DeploymentQueue {
    target_slots: Vec<Vec3>,
    next_index: usize,
    timer: Timer,
    base_pos: Vec3,
    ingress: Vec3,
    drone_mesh: Handle<Mesh>,
    drone_mat: Handle<StandardMaterial>,
    cone_mat: Handle<StandardMaterial>,
}

/// Radius each drone's coverage footprint must reach.
pub const FORMATION_RADIUS_KM: f32 = 3.0;
/// Fleet multiplier applied after computing the minimum gap-free coverage grid.
const COVERAGE_RESERVE: f32 = 2.0;

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

/// Number of drones for the blue target's 3 km coverage cells, doubled from
/// the required coverage count.
/// reserve. A radius-3 km circle's gap-free square cell is 3√2 km wide, or
/// 18 km², so this uses the selected polygon's area rather than its orange
/// bounding square.
pub fn target_area_drone_count(area: &crate::area::NetworkArea) -> usize {
    let safe_cell_area = 2.0 * FORMATION_RADIUS_KM.powi(2);
    ((blue_target_area_km2(area) / safe_cell_area) * COVERAGE_RESERVE)
        .ceil()
        .max(5.0) as usize
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

/// Evenly distribute the swarm across a gap-free 3 km coverage grid inside the
/// selected area. Communication range is intentionally not part of this
/// deployment mode; each drone has a fixed survey cell.
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
    // The blue target is a convex hull. Its vertex mean is guaranteed to sit
    // inside it, unlike the orange bounding-square centre for a triangle.
    let (x, z) = if area.hull.len() >= 3 {
        let sum = area.hull.iter().fold(Vec2::ZERO, |sum, &(lon, lat)| {
            sum + Vec2::new(
                ((lon - scenario.longitude) * 111.320 * scenario.latitude.to_radians().cos())
                    as f32,
                ((lat - scenario.latitude) * 110.574) as f32,
            )
        });
        let center = sum / area.hull.len() as f32;
        (center.x, center.y)
    } else {
        let (lon, lat) = area.center;
        (
            ((lon - scenario.longitude) * 111.320 * scenario.latitude.to_radians().cos()) as f32,
            ((lat - scenario.latitude) * 110.574) as f32,
        )
    };
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
    bases: Query<&Base>,
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

    let drone_mesh = meshes.add(Sphere::new(DRONE_RADIUS));
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
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);
    let target_slots = target_area_formation(&network_area, &scenario, &terrain);
    let ingress = target_area_center(&network_area, &scenario, &terrain);
    let initial_count = target_slots.len().min(DEPLOYMENT_BATCH_SIZE);
    for index in 0..initial_count {
        spawn_deployment_drone(
            &mut commands,
            &mut meshes,
            &drone_mesh,
            &drone_mat,
            &cone_mat,
            launch_position(base_pos, index, target_slots.len()),
            base_pos,
            ingress,
            &target_slots,
            index,
        );
    }
    commands.insert_resource(DeploymentQueue {
        target_slots,
        next_index: initial_count,
        timer: Timer::from_seconds(DEPLOYMENT_INTERVAL_SECS, TimerMode::Repeating),
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

/// Launch the next batch of up to three nodes every ten seconds until the
/// initial deployment queue is exhausted.
pub fn spawn_next_drone(
    time: Res<Time>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut deployment: ResMut<DeploymentQueue>,
) {
    if deployment.next_index >= deployment.target_slots.len()
        || !deployment.timer.tick(time.delta()).just_finished()
    {
        return;
    }

    let end = (deployment.next_index + DEPLOYMENT_BATCH_SIZE).min(deployment.target_slots.len());
    for index in deployment.next_index..end {
        spawn_deployment_drone(
            &mut commands,
            &mut meshes,
            &deployment.drone_mesh.clone(),
            &deployment.drone_mat.clone(),
            &deployment.cone_mat.clone(),
            launch_position(deployment.base_pos, index, deployment.target_slots.len()),
            deployment.base_pos,
            deployment.ingress,
            &deployment.target_slots,
            index,
        );
    }
    deployment.next_index = end;
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
    index: usize,
) {
    let antennas = formation_antennas(target_slots, index, base_pos);
    let id = drone_id(index);
    let bias_seed = uuid_bias_seed(&id);
    let drone_entity = commands
        .spawn((
            Mesh3d(drone_mesh.clone()),
            MeshMaterial3d(drone_mat.clone()),
            Transform::from_translation(launch_pos),
            Drone { id },
            Antennas(antennas.clone()),
            DeploymentTarget {
                ingress,
                spreading: false,
                bias_direction: bias_direction(bias_seed, 0),
                bias_seed: bias_seed as u32,
                bias_elapsed_secs: 0.0,
                next_bias_change_secs: initial_bias_delay_secs(bias_seed),
                bias_window: 0,
            },
            DroneKinematics::default(),
            DroneAi::default(),
            CommandQueue::default(),
            NetworkingBundle::random(index),
            SeekState::default(),
            RecoveryState::default(),
            ContactMemory::default(),
            ThemeRole::Drone,
            crate::SimulationEntity,
        ))
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
    use super::*;

    #[test]
    fn launch_pads_clear_each_other_and_the_base() {
        let base = Vec3::ZERO;
        let count = 17;
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
        // 36 km² / 18 km² per cell × 2 = 4, raised to the fleet minimum.
        assert_eq!(target_area_drone_count(&area), 5);
        assert_eq!((columns, rows), (3, 2));
    }
}
