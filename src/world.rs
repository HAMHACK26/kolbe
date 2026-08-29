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

pub const WORLD_SIZE: f32 = 20.0;
pub const DRONE_COUNT: usize = 12;
pub const DRONE_RADIUS: f32 = 0.18;

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
    theme: Res<Theme>,
) {
    let pal = theme.palette();
    commands.spawn((Camera3d::default(), Transform::default(), crate::SimulationEntity));

    commands.spawn((AmbientLight { brightness: 300.0, ..default() }, crate::SimulationEntity));
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

    let half = WORLD_SIZE / 2.0;
    for i in 0..DRONE_COUNT {
        let (km_x, km_z) = positions[i];
        let x = km_x - half;
        let z = km_z - half;
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
    let half = WORLD_SIZE / 2.0;
    let color = theme.palette().grid.with_alpha(0.25);
    const SEGMENTS: usize = 64;
    for i in 0..=4 {
        let offset = -half + i as f32 * 5.0;
        for segment in 0..SEGMENTS {
            let a = -half + WORLD_SIZE * segment as f32 / SEGMENTS as f32;
            let b = -half + WORLD_SIZE * (segment + 1) as f32 / SEGMENTS as f32;
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
