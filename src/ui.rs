use bevy::{prelude::*, window::PrimaryWindow};

use crate::{
    antenna::Antenna,
    base::Base,
    drone::{Drone, DroneType, SelectedDrone},
    networking::MeshTable,
    theme::Theme,
};

#[derive(Component)]
pub struct UiCamera;

/// UI exists before the 3D world, so it needs its own camera from startup.
pub fn setup_camera(mut commands: Commands) {
    commands.spawn((Camera2d, IsDefaultUiCamera, UiCamera));
}

/// Once the 3D camera exists, render UI transparently above it.
pub fn make_camera_overlay(mut cameras: Query<&mut Camera, With<UiCamera>>) {
    if let Ok(mut camera) = cameras.single_mut() {
        camera.order = 1;
        camera.clear_color = ClearColorConfig::None;
    }
}

/// Drops the whole simulation and returns to the area-selection map.
#[derive(Component)]
pub struct ResetButton;

pub fn spawn_reset_button(mut commands: Commands) {
    commands
        .spawn((
            Button,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(16.0),
                top: Val::Px(16.0),
                padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
                border_radius: BorderRadius::all(Val::Px(6.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
            ResetButton,
            crate::SimulationEntity,
        ))
        .with_child((
            Text::new("Back to area selection"),
            TextFont { font_size: FontSize::Px(14.0), ..default() },
            TextColor(Color::WHITE),
        ));
}

pub fn reset_button_interactions(
    interactions: Query<&Interaction, (Changed<Interaction>, With<ResetButton>)>,
    mut next_state: ResMut<NextState<crate::AppState>>,
) {
    for interaction in &interactions {
        if *interaction == Interaction::Pressed {
            next_state.set(crate::AppState::AreaSelection);
        }
    }
}

#[derive(Component)]
pub struct InfoPopup;

/// Separate floating window showing the mesh body table. Lifecycle is tied
/// to `InfoPopup` — closing the info popup (deselecting) closes this too.
#[derive(Component)]
pub struct NetworkTablePopup;

/// Single-line heading above the table (id + type + count).
#[derive(Component)]
pub struct InfoPopupTitle;

/// Grid node whose children are the table cells.
#[derive(Component)]
pub struct InfoPopupTable;

/// "View Network Table" button in the popup.
#[derive(Component)]
pub struct NetworkTableButton;

/// Text node showing the selected drone's mesh body table.
#[derive(Component)]
pub struct NetworkTablePanelText;

/// Whether the network table panel is expanded. Toggled by `NetworkTableButton`.
#[derive(Resource, Default)]
pub struct NetworkTablePanelOpen(pub bool);

// All antennas share the same hardware — only the pointing angles differ,
// and those angles are all the drone can know.
const COLS: usize = 3;
const HEADER: [&str; COLS] = ["#", "Az", "El"];

pub fn update_popup_position(
    mut commands: Commands,
    selected: Res<SelectedDrone>,
    drones: Query<(&GlobalTransform, &Drone)>,
    bases: Query<(&GlobalTransform, &Base)>,
    camera_q: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    _windows: Query<&Window, With<PrimaryWindow>>,
    mut popup_q: Query<(&mut Node, &mut Visibility), With<InfoPopup>>,
    mut title_q: Query<&mut Text, With<InfoPopupTitle>>,
    table_q: Query<(Entity, Option<&Children>), With<InfoPopupTable>>,
    table_open: Res<NetworkTablePanelOpen>,
    mesh_tables_q: Query<&MeshTable>,
    mut net_popup_q: Query<
        (&mut Node, &mut Visibility),
        (With<NetworkTablePopup>, Without<InfoPopup>),
    >,
    mut table_text_q: Query<&mut Text, (With<NetworkTablePanelText>, Without<InfoPopupTitle>)>,
    theme: Res<Theme>,
    mut last_sig: Local<String>,
) {
    let pal = theme.palette();
    let Ok((mut node, mut vis)) = popup_q.single_mut() else {
        return;
    };
    let Ok(mut title) = title_q.single_mut() else {
        return;
    };
    let Ok((table_entity, table_children)) = table_q.single() else {
        return;
    };
    let mut net_popup = net_popup_q.single_mut().ok();
    let mut table_text = table_text_q.single_mut().ok();

    let Some(entity) = selected.0 else {
        *vis = Visibility::Hidden;
        last_sig.clear();
        if let Some((_, ref mut net_vis)) = net_popup {
            **net_vis = Visibility::Hidden;
        }
        if let Some(ref mut t) = table_text {
            **t = Text::new("");
        }
        return;
    };

    // Live network-table readout — refreshed every frame while the panel is
    // open, independent of the antenna-table rebuild below.
    if let Some(ref mut t) = table_text {
        **t = Text::new(if let Ok(mesh) = mesh_tables_q.get(entity) {
            if mesh.0.is_empty() {
                "no known peers yet".into()
            } else {
                let mut rows: Vec<_> = mesh.0.values().collect();
                rows.sort_by(|a, b| a.id.cmp(&b.id));
                rows.iter()
                    .map(|r| {
                        format!(
                            "id: {}\n  t: {:.3}  loc: {:.3?}\n  dist: {}  conn: [{}]",
                            r.id,
                            r.timestamp,
                            r.location,
                            r.neighbour_distance,
                            r.connections.join(", ")
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("\n")
            }
        } else {
            "base has no network table".into()
        });
    }

    // Resolve world position + table content from either a drone or the base.
    let (world_pos, title_str, rows) = if let Ok((gt, drone)) = drones.get(entity) {
        (
            gt.translation(),
            drone_title(drone),
            antenna_rows(&drone.antennas),
        )
    } else if let Ok((gt, base)) = bases.get(entity) {
        (
            gt.translation(),
            base_title(base),
            antenna_rows(&base.antennas),
        )
    } else {
        *vis = Visibility::Hidden;
        last_sig.clear();
        if let Some((_, ref mut net_vis)) = net_popup {
            **net_vis = Visibility::Hidden;
        }
        return;
    };

    let Ok((camera, cam_gt)) = camera_q.single() else {
        return;
    };
    let Ok(screen_pos) = camera.world_to_viewport(cam_gt, world_pos) else {
        *vis = Visibility::Hidden;
        if let Some((_, ref mut net_vis)) = net_popup {
            **net_vis = Visibility::Hidden;
        }
        return;
    };

    *vis = Visibility::Visible;
    node.left = Val::Px(screen_pos.x + 14.0);
    node.top = Val::Px(screen_pos.y - 10.0);

    // Network table window — a second window tied to this one's lifecycle:
    // open only while a drone/base is selected (above) *and* the button has
    // toggled it on; closing the info popup always closes this too.
    if let Some((ref mut net_node, ref mut net_vis)) = net_popup {
        if table_open.0 {
            **net_vis = Visibility::Visible;
            net_node.left = Val::Px(screen_pos.x + 14.0);
            net_node.top = Val::Px(screen_pos.y + 130.0);
        } else {
            **net_vis = Visibility::Hidden;
        }
    }

    // Rebuild the grid only when the selected content (or theme) changes.
    let sig = format!("{}|{title_str}|{rows:?}", theme.dark);
    if *last_sig == sig {
        return;
    }
    *last_sig = sig;
    **title = title_str;

    if let Some(children) = table_children {
        for &c in children {
            commands.entity(c).despawn();
        }
    }
    let accent = pal.accent;
    let text = pal.text;
    commands.entity(table_entity).with_children(|p| {
        for h in HEADER {
            p.spawn((
                Text::new(h),
                TextFont {
                    font_size: FontSize::Px(12.0),
                    ..default()
                },
                TextColor(accent),
            ));
        }
        for row in &rows {
            for cell in row {
                p.spawn((
                    Text::new(cell.clone()),
                    TextFont {
                        font_size: FontSize::Px(12.0),
                        ..default()
                    },
                    TextColor(text),
                ));
            }
        }
    });
}

fn drone_title(drone: &Drone) -> String {
    let type_str = match drone.drone_type {
        DroneType::Attack => "Attack",
        DroneType::Node => "Node",
    };
    format!("{}  [{}]  {} ant", drone.id, type_str, drone.antennas.len())
}

fn base_title(base: &Base) -> String {
    format!("{}  [Base]  {} connections", base.id, base.antennas.len())
}

fn antenna_rows(antennas: &[Antenna]) -> Vec<[String; COLS]> {
    antennas
        .iter()
        .enumerate()
        .map(|(i, ant)| {
            [
                format!("{}", i + 1),
                format!("{:.0}", ant.azimuth_deg),
                format!("{:.0}", ant.elevation_deg),
            ]
        })
        .collect()
}
