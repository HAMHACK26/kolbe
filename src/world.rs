use bevy::{color::palettes::css, picking::prelude::*, prelude::*};

use crate::{
    base::CommandQueue,
    camera::OrbitCamera,
    drone::{Drone, DroneType, SelectedDrone, drone_id, make_antenna},
    factories::{DroneAi, movement::DroneKinematics},
    radar::{RadarCone, cone_mesh_for, cone_transform_for},
    ui::{InfoPopup, InfoPopupText},
};

pub const WORLD_SIZE: f32 = 20.0;
pub const DRONE_COUNT: usize = 12;
pub const DRONE_RADIUS: f32 = 0.18;

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((Camera3d::default(), Transform::default()));

    commands.spawn(AmbientLight { brightness: 300.0, ..default() });
    commands.spawn((
        DirectionalLight { illuminance: 8000.0, shadow_maps_enabled: false, ..default() },
        Transform::from_xyz(8.0, 16.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground
    commands
        .spawn((
            Mesh3d(meshes.add(Plane3d::default().mesh().size(WORLD_SIZE, WORLD_SIZE))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.13, 0.25, 0.13),
                perceptual_roughness: 1.0,
                ..default()
            })),
            Transform::IDENTITY,
        ))
        .observe(
            |_: On<Pointer<Click>>, orbit: Res<OrbitCamera>, mut sel: ResMut<SelectedDrone>| {
                if orbit.drag_total < 5.0 {
                    sel.0 = None;
                }
            },
        );

    let positions: [(f32, f32); 12] = [
        (2.3, 4.1), (7.8, 1.5), (14.2, 6.3), (18.0, 2.0),
        (5.5, 11.0), (11.3, 9.7), (16.8, 13.2), (3.0, 16.5),
        (9.1, 17.8), (13.5, 14.0), (19.0, 18.5), (6.7, 7.3),
    ];

    let drone_mesh = meshes.add(Sphere::new(DRONE_RADIUS));
    let drone_mat = materials.add(StandardMaterial {
        base_color: Color::from(css::RED),
        emissive: LinearRgba::new(2.0, 0.0, 0.0, 1.0),
        ..default()
    });
    let cone_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.0, 0.9, 1.0, 0.30),
        emissive: LinearRgba::new(0.0, 0.4, 0.8, 0.0),
        alpha_mode: AlphaMode::Blend,
        double_sided: true,
        cull_mode: None,
        ..default()
    });

    let half = WORLD_SIZE / 2.0;
    for i in 0..DRONE_COUNT {
        let (km_x, km_z) = positions[i];
        let drone_pos = Vec3::new(km_x - half, DRONE_RADIUS, km_z - half);
        let drone_type = if i % 3 == 0 { DroneType::Attack } else { DroneType::Node };

        let az0 = (i as f32 * 137.5) % 360.0;
        let el0 = ((i as f32 * 23.0) % 30.0) - 15.0;
        let az1 = (az0 + 150.0) % 360.0;

        let antennas = match drone_type {
            DroneType::Attack => vec![make_antenna(az0, el0, i)],
            DroneType::Node => {
                vec![make_antenna(az0, el0, i), make_antenna(az1, -el0, i + 100)]
            }
        };

        let drone_entity = commands
            .spawn((
                Mesh3d(drone_mesh.clone()),
                MeshMaterial3d(drone_mat.clone()),
                Transform::from_translation(drone_pos),
                Drone { id: drone_id(i), drone_type, antennas: antennas.clone() },
                DroneKinematics::default(),
                DroneAi::default(),
                CommandQueue::default(),
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
                cone_transform_for(antenna, drone_pos),
                Visibility::Hidden,
                RadarCone { drone_entity },
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
            BackgroundColor(Color::srgba(0.05, 0.05, 0.05, 0.88)),
            Visibility::Hidden,
            InfoPopup,
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(""),
                TextFont { font_size: FontSize::Px(12.0), ..default() },
                TextColor(Color::WHITE),
                InfoPopupText,
            ));
        });
}

pub fn draw_grid(mut gizmos: Gizmos) {
    let half = WORLD_SIZE / 2.0;
    let color = Color::srgba(0.8, 0.9, 0.8, 0.18);
    for i in 0..=4 {
        let offset = -half + i as f32 * 5.0;
        gizmos.line(Vec3::new(-half, 0.01, offset), Vec3::new(half, 0.01, offset), color);
        gizmos.line(Vec3::new(offset, 0.01, -half), Vec3::new(offset, 0.01, half), color);
    }
}
