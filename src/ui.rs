use bevy::{prelude::*, window::PrimaryWindow};

use crate::{
    antenna::Antenna,
    base::{Base, BaseNetworkState},
    drone::{Drone, DroneType, SelectedDrone},
    networking::{DroneClock, DroneUuid, LinkSet, MeshTable, TargetAreaVectors},
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

/// Heading of the mesh debug panel.
#[derive(Component)]
pub struct NetworkTableTitle;

/// Grid holding the selected node's own state (label/value pairs).
#[derive(Component)]
pub struct NetworkTableSummary;

/// Grid holding one row per known mesh peer.
#[derive(Component)]
pub struct NetworkTableGrid;

/// Whether the network table panel is expanded. Toggled by `NetworkTableButton`.
#[derive(Resource, Default)]
pub struct NetworkTablePanelOpen(pub bool);

#[derive(Component)]
pub struct SpeedLabel;

#[derive(Component)]
pub struct SpeedFill;

/// True while a UI control owns a pointer gesture. World/map navigation must
/// not interpret the same drag as an orbit or pan.
#[derive(Resource, Default)]
pub struct UiPointerCapture(pub bool);

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
            .observe(|mut event: On<Pointer<Press>>, mut capture: ResMut<UiPointerCapture>| {
                event.propagate(false);
                capture.0 = true;
            })
            .observe(|mut event: On<Pointer<Release>>, mut capture: ResMut<UiPointerCapture>| {
                event.propagate(false);
                capture.0 = false;
            })
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

    let Some(entity) = selected.0 else {
        *vis = Visibility::Hidden;
        last_sig.clear();
        return;
    };

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
        return;
    };

    let Ok((camera, cam_gt)) = camera_q.single() else {
        return;
    };
    let Ok(screen_pos) = camera.world_to_viewport(cam_gt, world_pos) else {
        *vis = Visibility::Hidden;
        return;
    };

    *vis = Visibility::Visible;
    node.left = Val::Px(screen_pos.x + 14.0);
    node.top = Val::Px(screen_pos.y - 10.0);

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

// ─── Mesh debug panel ───────────────────────────────────────────────────────

/// Columns of the mesh body table, in display order.
const MESH_HEADER: [&str; 6] = ["ID", "HOP", "AGE", "X", "Z", "CONN"];

/// How many peer rows the panel will draw before summarising the remainder.
/// Enough to see a whole 74-airframe mesh's near neighbourhood without the
/// grid running off the bottom of the window.
const MAX_MESH_ROWS: usize = 26;

/// Fill the docked mesh debug panel for whatever node is selected.
///
/// Kept out of `update_popup_position` because it answers a different
/// question: that popup says what the antennas are doing, this says what the
/// node *knows* — the gossiped table, the target-area vectors it was briefed
/// with, and whether it currently believes itself inside that area. Those
/// three together are what a stuck or wandering drone has to be diagnosed
/// from, so they belong side by side and pinned in place.
#[allow(clippy::too_many_arguments, clippy::type_complexity)] // Queries are distinct Bevy system inputs.
pub fn update_network_table(
    mut commands: Commands,
    selected: Res<SelectedDrone>,
    table_open: Res<NetworkTablePanelOpen>,
    theme: Res<Theme>,
    fonts: Res<crate::UiFonts>,
    drones: Query<(
        &Drone,
        &GlobalTransform,
        &DroneUuid,
        &DroneClock,
        &LinkSet,
        &MeshTable,
        &TargetAreaVectors,
        Option<&crate::world::LaunchTarget>,
    )>,
    node_bases: Query<(
        &Base,
        &DroneUuid,
        &DroneClock,
        &LinkSet,
        &MeshTable,
        &TargetAreaVectors,
    )>,
    bases: Query<&Base>,
    mut panel_q: Query<&mut Visibility, With<NetworkTablePopup>>,
    mut title_q: Query<&mut Text, With<NetworkTableTitle>>,
    summary_q: Query<(Entity, Option<&Children>), With<NetworkTableSummary>>,
    grid_q: Query<(Entity, Option<&Children>), (With<NetworkTableGrid>, Without<NetworkTableSummary>)>,
    mut last_sig: Local<String>,
) {
    let Ok(mut vis) = panel_q.single_mut() else {
        return;
    };
    let open = table_open.0 && selected.0.is_some();
    *vis = if open { Visibility::Visible } else { Visibility::Hidden };
    if !open {
        last_sig.clear();
        return;
    }
    let entity = selected.0.expect("checked by `open`");
    let base_pos = bases.iter().next().map(|base| base.position);

    let (title, summary, mesh) = if let Ok(node) = drones.get(entity) {
        drone_debug(node, base_pos)
    } else if let Ok(node) = node_bases.get(entity) {
        base_debug(node)
    } else {
        last_sig.clear();
        return;
    };

    // The signature is built from the *rendered* strings, so the grid only
    // rebuilds when something a reader could actually see has changed. That
    // also throttles it naturally: a position formatted to 2 dp settles long
    // before the float does.
    let sig = format!("{}|{title}|{summary:?}|{mesh:?}", theme.dark);
    if *last_sig == sig {
        return;
    }
    *last_sig = sig;

    let pal = theme.palette();
    if let Ok(mut text) = title_q.single_mut() {
        **text = title;
    }

    let label_font = || TextFont { font_size: FontSize::Px(10.0), ..default() };
    let value_font = || TextFont {
        font: fonts.mono.clone().into(),
        font_size: FontSize::Px(11.0),
        ..default()
    };

    if let Ok((summary_entity, children)) = summary_q.single() {
        if let Some(children) = children {
            for &child in children {
                commands.entity(child).despawn();
            }
        }
        commands.entity(summary_entity).with_children(|grid| {
            for (label, value, tone) in &summary {
                grid.spawn((Text::new(label.clone()), label_font(), TextColor(pal.subtext)));
                grid.spawn((
                    Text::new(value.clone()),
                    value_font(),
                    TextColor(match tone {
                        Tone::Normal => pal.text,
                        Tone::Good => pal.accent,
                        Tone::Bad => pal.danger,
                    }),
                ));
            }
        });
    }

    if let Ok((grid_entity, children)) = grid_q.single() {
        if let Some(children) = children {
            for &child in children {
                commands.entity(child).despawn();
            }
        }
        commands.entity(grid_entity).with_children(|grid| {
            for header in MESH_HEADER {
                grid.spawn((Text::new(header), label_font(), TextColor(pal.accent)));
            }
            for row in &mesh {
                for cell in row {
                    grid.spawn((Text::new(cell.clone()), value_font(), TextColor(pal.text)));
                }
            }
        });
    }
}

/// How a summary value should read at a glance.
#[derive(Debug, PartialEq, Eq)]
enum Tone {
    Normal,
    /// The system is where it should be.
    Good,
    /// Something the operator has to act on.
    Bad,
}

type SummaryRow = (String, String, Tone);
type MeshRowCells = [String; 6];

/// Short form of a UUID — enough to tell rows apart, not so much that the
/// column eats the panel.
fn short_id(id: &str) -> String {
    id.chars().take(8).collect()
}

fn vec_xz(v: Vec3) -> String {
    format!("{:+.2} {:+.2}", v.x, v.z)
}

fn drone_debug(
    (drone, transform, uuid, clock, links, table, target_area, launch): (
        &Drone,
        &GlobalTransform,
        &DroneUuid,
        &DroneClock,
        &LinkSet,
        &MeshTable,
        &TargetAreaVectors,
        Option<&crate::world::LaunchTarget>,
    ),
    base_pos: Option<Vec3>,
) -> (String, Vec<SummaryRow>, Vec<MeshRowCells>) {
    let position = transform.translation();
    let from_base = base_pos.map(|base| position - base);

    let mut summary = vec![
        ("UUID".into(), short_id(&uuid.0), Tone::Normal),
        ("CLOCK".into(), format!("{:.3} s", clock.now), Tone::Normal),
        (
            "DIRECT LINKS".into(),
            format!("{}", links.connected.len()),
            if links.connected.is_empty() { Tone::Bad } else { Tone::Good },
        ),
        ("TABLE ROWS".into(), format!("{}", table.0.len()), Tone::Normal),
        ("POSITION".into(), vec_xz(position), Tone::Normal),
        (
            "FROM BASE".into(),
            from_base.map_or_else(|| "no base".into(), vec_xz),
            Tone::Normal,
        ),
    ];

    // The whole point of the briefing: did the corners arrive, and what does
    // this drone conclude from them about its own position?
    match target_area.corners_from_base.as_deref() {
        Some(corners) => {
            summary.push((
                "AREA VECTORS".into(),
                format!("{} pts @ {:.2} s", corners.len(), target_area.received_at),
                Tone::Good,
            ));
            for (i, corner) in corners.iter().enumerate() {
                summary.push((format!("  CORNER {}", i + 1), vec_xz(*corner), Tone::Normal));
            }
            let inside = from_base.map(|v| crate::navigation::target_contains(v, corners));
            summary.push(match inside {
                Some(true) => ("INSIDE AREA".into(), "YES".into(), Tone::Good),
                Some(false) => ("INSIDE AREA".into(), "NO — INGRESS".into(), Tone::Bad),
                None => ("INSIDE AREA".into(), "unknown".into(), Tone::Bad),
            });
        }
        None => summary.push(("AREA VECTORS".into(), "NOT RECEIVED".into(), Tone::Bad)),
    }

    summary.push(match launch {
        Some(target) => ("LAUNCH TARGET".into(), vec_xz(target.0), Tone::Normal),
        None => ("LAUNCH TARGET".into(), "on station".into(), Tone::Normal),
    });

    (
        format!("MESH DEBUG  ·  {}", drone.id),
        summary,
        mesh_rows(table, clock.now),
    )
}

fn base_debug(
    (base, uuid, clock, links, table, target_area): (
        &Base,
        &DroneUuid,
        &DroneClock,
        &LinkSet,
        &MeshTable,
        &TargetAreaVectors,
    ),
) -> (String, Vec<SummaryRow>, Vec<MeshRowCells>) {
    let mut summary = vec![
        ("UUID".into(), short_id(&uuid.0), Tone::Normal),
        ("CLOCK".into(), format!("{:.3} s", clock.now), Tone::Normal),
        (
            "DIRECT LINKS".into(),
            format!("{}", links.connected.len()),
            if links.connected.is_empty() { Tone::Bad } else { Tone::Good },
        ),
        ("TABLE ROWS".into(), format!("{}", table.0.len()), Tone::Normal),
        ("POSITION".into(), vec_xz(base.position), Tone::Normal),
    ];

    // Where each sector antenna is currently pointed. All five reading the
    // same bearing means they are all aimed at co-located drones — which is
    // exactly what launch looks like before anything has moved.
    for (i, antenna) in base.antennas.iter().enumerate() {
        summary.push((
            format!("  ANT {} AZ/EL", i + 1),
            format!("{:>6.1} {:>6.1}", antenna.azimuth_deg, antenna.elevation_deg),
            Tone::Normal,
        ));
    }

    match target_area.corners_from_base.as_deref() {
        Some(corners) => {
            summary.push((
                "AREA VECTORS".into(),
                format!("BROADCASTING {} pts", corners.len()),
                Tone::Good,
            ));
            for (i, corner) in corners.iter().enumerate() {
                summary.push((format!("  CORNER {}", i + 1), vec_xz(*corner), Tone::Normal));
            }
        }
        None => summary.push(("AREA VECTORS".into(), "NONE — NOT SENT".into(), Tone::Bad)),
    }

    (
        format!("MESH DEBUG  ·  {}", base.id),
        summary,
        mesh_rows(table, clock.now),
    )
}

/// Mesh body table as display rows, nearest hop first then by age.
fn mesh_rows(table: &MeshTable, now: f64) -> Vec<MeshRowCells> {
    let mut rows: Vec<_> = table.0.values().collect();
    rows.sort_by(|a, b| {
        a.neighbour_distance
            .cmp(&b.neighbour_distance)
            .then((now - a.timestamp).total_cmp(&(now - b.timestamp)))
    });

    let shown = rows.len().min(MAX_MESH_ROWS);
    let mut cells: Vec<MeshRowCells> = rows[..shown]
        .iter()
        .map(|row| {
            [
                short_id(&row.id),
                format!("{}", row.neighbour_distance),
                format!("{:.2}", now - row.timestamp),
                format!("{:+.2}", row.location.x),
                format!("{:+.2}", row.location.z),
                format!("{}", row.connections.len()),
            ]
        })
        .collect();
    if rows.len() > shown {
        cells.push([
            format!("+{}", rows.len() - shown),
            "more".into(),
            String::new(),
            String::new(),
            String::new(),
            String::new(),
        ]);
    }
    cells
}
