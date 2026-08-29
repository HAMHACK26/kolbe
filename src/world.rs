use bevy::{picking::prelude::*, prelude::*};

use crate::{
    base::CommandQueue,
    camera::OrbitCamera,
    drone::{Drone, DroneType, SelectedDrone, drone_id, make_antenna},
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

<<<<<<< Updated upstream
pub const WORLD_SIZE: f32 = 20.0;
pub const DRONE_COUNT: usize = 12;
pub const DRONE_RADIUS: f32 = 0.18;
=======
pub const DRONE_COUNT: usize = 5;
pub const DRONE_RADIUS: f32 = 0.0225;
/// Clearance from the terrain to the underside of each drone, in km (50 m).
pub const DRONE_GROUND_CLEARANCE_KM: f32 = 0.05;
const DEPLOYMENT_INTERVAL_SECS: f32 = 10.0;

/// The individual destination assigned to a drone during deployment.
#[derive(Component)]
pub struct DeploymentTarget {
    pub ingress: Vec3,
    pub spreading: bool,
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

/// Hard cap on the distance between two ring neighbors, km.
///
/// The link budget in `make_antenna` closes at ~3.5 km on boresight
/// (`Antenna::max_range_km`), so holding neighbors at 3 km keeps every mesh
/// hop inside range with margin for terrain relief and drift.
pub const MAX_NEIGHBOR_SPACING_KM: f32 = 3.0;

/// Evenly distribute the swarm inside the selected target area.  The radius
/// stays inside the area's inscribed circle and within the mesh link budget.
pub fn target_area_formation(
    area: &crate::area::NetworkArea,
    scenario: &crate::area::ScenarioArea,
    terrain: &crate::terrain::TerrainHeightMap,
) -> Vec<Vec3> {
    let (lon, lat) = area.center;
    let center_x = ((lon - scenario.longitude) * 111.320 * scenario.latitude.to_radians().cos()) as f32;
    let center_z = ((lat - scenario.latitude) * 110.574) as f32;
    let max_mesh_radius = MAX_NEIGHBOR_SPACING_KM
        / (2.0 * (std::f32::consts::PI / DRONE_COUNT as f32).sin());
    let radius = ((area.side_km as f32) * 0.25).min(max_mesh_radius).max(0.05);
    (0..DRONE_COUNT)
        .map(|i| {
            let angle = i as f32 / DRONE_COUNT as f32 * std::f32::consts::TAU;
            let x = center_x + radius * angle.sin();
            let z = center_z + radius * angle.cos();
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
>>>>>>> Stashed changes

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
    theme: Res<Theme>,
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

    let positions: [(f32, f32); 12] = [
        (2.3, 4.1), (7.8, 1.5), (14.2, 6.3), (18.0, 2.0),
        (5.5, 11.0), (11.3, 9.7), (16.8, 13.2), (3.0, 16.5),
        (9.1, 17.8), (13.5, 14.0), (19.0, 18.5), (6.7, 7.3),
    ];

    let drone_mesh = meshes.add(Sphere::new(DRONE_RADIUS));
    // Initial colors come from the palette; `apply_theme` keeps them in sync on
    // theme toggles (these entities carry ThemeRole markers).
    let drone_mat = materials.add(StandardMaterial {
        base_color: pal.drone,
        emissive: LinearRgba::new(2.0, 0.0, 0.0, 1.0),
        ..default()
    });
    let cone_mat = materials.add(StandardMaterial {
        base_color: pal.drone_cone.with_alpha(0.30),
        emissive: LinearRgba::new(0.0, 0.4, 0.8, 0.0),
        alpha_mode: AlphaMode::Blend,
        double_sided: true,
        cull_mode: None,
        ..default()
    });

    // `positions` are hand-placed for a `WORLD_SIZE` (20km) world — scale
    // proportionally so the ring spans the *actual* fetched terrain, which
    // can be smaller or much larger (a network area can run up to
    // `area::MAX_SIDE_KM` per side) depending on what was picked on the
    // map. Without this, drones stayed clustered in a fixed center 20km
    // regardless of the real terrain extent.
    let half = terrain.size_km() * 0.5;
    let scale = terrain.size_km() / WORLD_SIZE;
    for i in 0..DRONE_COUNT {
        let (km_x, km_z) = positions[i];
        let x = km_x * scale - half;
        let z = km_z * scale - half;
        let drone_pos = Vec3::new(x, terrain.height_at(x, z) + DRONE_RADIUS, z);
        let drone_type = if i % 3 == 0 { DroneType::Attack } else { DroneType::Node };

        let az0 = (i as f32 * 137.5) % 360.0;
        let el0 = ((i as f32 * 23.0) % 30.0) - 15.0;

        // Every drone carries exactly 3 antennas, 120° apart. Initial layout
        // only — `maintain_mesh_antennas` retargets every frame based on
        // each drone's ring-index neighbors.
        let antennas = vec![
            make_antenna(az0, el0, i),
            make_antenna((az0 + 120.0) % 360.0, el0, i + 100),
            make_antenna((az0 + 240.0) % 360.0, el0, i + 200),
        ];

        let drone_entity = commands
            .spawn((
                Mesh3d(drone_mesh.clone()),
                MeshMaterial3d(drone_mat.clone()),
                Transform::from_translation(drone_pos),
                Drone { id: drone_id(i), drone_type, antennas: antennas.clone() },
                DroneKinematics::default(),
                DroneAi::default(),
                CommandQueue::default(),
                NetworkingBundle::random(i),
                SeekState::default(),
                RecoveryState::default(),
                ContactMemory::default(),
                ThemeRole::Drone,
                crate::SimulationEntity,
            ))
            .observe(
                |mut t: On<Pointer<Click>>,
                 orbit: Res<OrbitCamera>,
                 mut sel: ResMut<SelectedDrone>| {
                    t.propagate(false);
                    if orbit.drag_total < 5.0 {
                        sel.0 = Some(t.original_event_target());
                    }
                },
            )
            .id();

        for antenna in &antennas {
            commands.spawn((
                Mesh3d(cone_mesh_for(antenna, &mut meshes)),
                MeshMaterial3d(cone_mat.clone()),
                // heading 0.0 matches the DroneKinematics::default() this
                // drone spawns with; apply_velocity updates heading as it
                // moves, but nothing currently re-syncs the cone transform.
                cone_transform_for(antenna, 0.0, drone_pos),
                Visibility::Hidden,
                RadarCone { drone_entity },
                ThemeRole::DroneCone,
                crate::SimulationEntity,
            ));
        }
    }

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
