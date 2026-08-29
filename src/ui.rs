use bevy::{prelude::*, window::PrimaryWindow};

use crate::{
    antenna::Antenna,
    base::{Base, BaseNetworkState},
    drone::{Drone, DroneType, SelectedDrone},
    networking::MeshTable,
    navigation::{MAX_MOVEMENT_SPEED, MIN_MOVEMENT_SPEED, MOVEMENT_SPEED_STEP, MovementSpeed},
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

#[derive(Component)]
pub struct SpeedLabel;

#[derive(Component)]
pub struct SpeedFill;

const SPEED_SLIDER_WIDTH_PX: f32 = 120.0;

fn adjust_speed(speed: &mut MovementSpeed, delta: f32) {
    speed.0 = ((speed.0 + delta).clamp(MIN_MOVEMENT_SPEED, MAX_MOVEMENT_SPEED)
        / MOVEMENT_SPEED_STEP)
        .round()
        * MOVEMENT_SPEED_STEP;
}

/// Compact flight-speed control for the live simulation.
pub fn spawn_speed_control(mut commands: Commands, theme: Res<Theme>) {
    let pal = theme.palette();
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(16.0),
                bottom: Val::Px(16.0),
                padding: UiRect::axes(Val::Px(10.0), Val::Px(7.0)),
                column_gap: Val::Px(8.0),
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(pal.surface.with_alpha(0.88)),
            crate::SimulationEntity,
        ))
        .with_children(|p| {
            p.spawn((Text::new("Drone speed"), TextFont { font_size: FontSize::Px(12.0), ..default() }, TextColor(pal.subtext)));
            for delta in [-MOVEMENT_SPEED_STEP] {
                p.spawn((
                    Button,
                    Node { padding: UiRect::axes(Val::Px(7.0), Val::Px(2.0)), ..default() },
                    BackgroundColor(pal.text.with_alpha(0.12)),
                ))
                .observe(move |mut event: On<Pointer<Click>>, mut speed: ResMut<MovementSpeed>| {
                    event.propagate(false);
                    adjust_speed(&mut speed, delta);
                })
                .with_child((
                    Text::new(if delta.is_sign_negative() { "−" } else { "+" }),
                    TextFont { font_size: FontSize::Px(16.0), ..default() },
                    TextColor(pal.text),
                ));
            }
            // The draggable track covers the full 1×–4× flight envelope.
            p.spawn((
                Button,
                Node {
                    width: Val::Px(SPEED_SLIDER_WIDTH_PX),
                    height: Val::Px(8.0),
                    padding: UiRect::all(Val::Px(1.0)),
                    ..default()
                },
                BackgroundColor(pal.text.with_alpha(0.18)),
            ))
            .observe(|mut event: On<Pointer<Drag>>, mut speed: ResMut<MovementSpeed>| {
                event.propagate(false);
                let span = MAX_MOVEMENT_SPEED - MIN_MOVEMENT_SPEED;
                adjust_speed(&mut speed, event.delta.x / SPEED_SLIDER_WIDTH_PX * span);
            })
            .with_child((
                Node {
                    width: Val::Percent(0.0),
                    height: Val::Percent(100.0),
                    ..default()
                },
                BackgroundColor(pal.accent),
                Pickable::IGNORE,
                SpeedFill,
            ));
            for delta in [MOVEMENT_SPEED_STEP] {
                p.spawn((
                    Button,
                    Node { padding: UiRect::axes(Val::Px(7.0), Val::Px(2.0)), ..default() },
                    BackgroundColor(pal.text.with_alpha(0.12)),
                ))
                .observe(move |mut event: On<Pointer<Click>>, mut speed: ResMut<MovementSpeed>| {
                    event.propagate(false);
                    adjust_speed(&mut speed, delta);
                })
                .with_child((
                    Text::new("+"),
                    TextFont { font_size: FontSize::Px(16.0), ..default() },
                    TextColor(pal.text),
                ));
            }
            p.spawn((Text::new("1.0×"), TextFont { font_size: FontSize::Px(13.0), ..default() }, TextColor(pal.accent), SpeedLabel));
        });
}

pub fn update_speed_label(
    speed: Res<MovementSpeed>,
    mut labels: Query<&mut Text, With<SpeedLabel>>,
    mut fills: Query<&mut Node, With<SpeedFill>>,
) {
    if !speed.is_changed() { return; }
    for mut label in &mut labels {
        **label = format!("{:.1}×", speed.0);
    }
    let percent = (speed.0 - MIN_MOVEMENT_SPEED) / (MAX_MOVEMENT_SPEED - MIN_MOVEMENT_SPEED) * 100.0;
    for mut fill in &mut fills {
        fill.width = Val::Percent(percent);
    }
}

// All antennas share the same hardware — only the pointing angles differ,
// and those angles are all the drone can know.
const COLS: usize = 3;
const HEADER: [&str; COLS] = ["#", "Az", "El"];

#[allow(clippy::too_many_arguments, clippy::type_complexity)] // Queries are distinct Bevy system inputs.
pub fn update_popup_position(
    mut commands: Commands,
    selected: Res<SelectedDrone>,
    drones: Query<(&GlobalTransform, &Drone)>,
    bases: Query<(Entity, &GlobalTransform, &Base)>,
    base_networks: Query<&BaseNetworkState>,
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
    } else if let Ok((base_entity, gt, base)) = bases.get(entity) {
        (
            gt.translation(),
            base_title(base, base_networks.get(base_entity).map_or(0, |state| state.reachable_drones.len())),
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

fn base_title(base: &Base, connections: usize) -> String {
    let connection_label = if connections == 1 { "connection" } else { "connections" };
    format!("{}  [Base]  {} antennas  ·  {connections} {connection_label}", base.id, base.antennas.len())
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
