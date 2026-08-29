use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    ui::{RelativeCursorPosition, UiTransform, Val2},
};

use crate::{polygon, sweden_geo, theme::Theme, AppState};

/// Kept for existing callers (terrain fetch math) — the network-area picker
/// now drives `ScenarioArea.size_km` dynamically instead.
pub const AREA_SIZE_KM: f32 = 20.0;

const MIN_POINTS: usize = 3;
const MAX_SIDE_KM: f64 = 50.0;
const POINT_DOT_SIZE: f32 = 9.0;
const CITY_DOT_SIZE: f32 = 5.0;
const EDGE_THICKNESS: f32 = 2.0;
const SQUARE_THICKNESS: f32 = 2.0;
const ZOOM_MIN: f32 = 1.0;
const ZOOM_MAX: f32 = 5.0;

#[derive(Resource, Clone, Debug)]
pub struct ScenarioArea {
    pub name: &'static str,
    pub latitude: f64,
    pub longitude: f64,
    pub size_km: f32,
}

impl Default for ScenarioArea {
    fn default() -> Self {
        Self {
            name: "Stockholm",
            latitude: 59.3293,
            longitude: 18.0686,
            size_km: AREA_SIZE_KM,
        }
    }
}

impl ScenarioArea {
    /// STAC searches use WGS84 longitude/latitude bounds.
    pub fn wgs84_bbox(&self) -> [f64; 4] {
        let half_km = self.size_km as f64 * 0.5;
        let lat_delta = half_km / 110.574;
        let lon_delta = half_km / (111.320 * self.latitude.to_radians().cos());
        [
            self.longitude - lon_delta,
            self.latitude - lat_delta,
            self.longitude + lon_delta,
            self.latitude + lat_delta,
        ]
    }
}

/// The lat/lon points the user has clicked, in click order.
#[derive(Resource, Default, Clone)]
pub struct PendingPoints(pub Vec<(f64, f64)>);

#[derive(Resource, Default, Clone, Copy, PartialEq, Eq)]
pub enum PickMode {
    #[default]
    Adding,
    Reviewing,
    PlacingBase,
}

/// Where the user placed the base — must land inside the axis-aligned area
/// that actually gets fetched (`NetworkArea::fetch_corners`).
#[derive(Resource, Default, Clone, Copy)]
pub struct BasePosition(pub Option<(f64, f64)>);

/// The (possibly rotated) minimum-area square enclosing the picked points —
/// the "network area". Recomputed live as points are added/removed; the
/// version present when "Generate terrain" is pressed is what gets fetched
/// and, later, marked on the 3D terrain.
#[derive(Resource, Clone, Debug, Default)]
pub struct NetworkArea {
    pub points: Vec<(f64, f64)>,
    pub corners: [(f64, f64); 4],
    pub center: (f64, f64),
    pub side_km: f64,
    pub rotation_deg: f64,
    /// Axis-aligned size guaranteed to cover the (possibly rotated) square —
    /// what actually gets requested from the height server.
    pub fetch_size_km: f32,
    /// Corners of that axis-aligned fetch square — "the area it's going to
    /// pull." The base must land inside this box.
    pub fetch_corners: [(f64, f64); 4],
    pub valid: bool,
    pub over_limit: bool,
}

impl NetworkArea {
    /// Distance in km from a lat/lon point to the (possibly rotated)
    /// network-area square — 0.0 if the point is inside it.
    pub fn distance_to_square_km(&self, lat: f64, lon: f64) -> f64 {
        if !self.valid {
            return f64::INFINITY;
        }
        let (clat, clon) = self.center;
        let local = polygon::project(clon, clat, lon, lat);
        let (s, c) = self.rotation_deg.to_radians().sin_cos();
        // Rotate into the square's own axes.
        let rx = local.x_km * c + local.z_km * s;
        let rz = -local.x_km * s + local.z_km * c;
        let half = self.side_km * 0.5;
        let dx = (rx.abs() - half).max(0.0);
        let dz = (rz.abs() - half).max(0.0);
        (dx * dx + dz * dz).sqrt()
    }
}

fn recompute_network_area(points: &[(f64, f64)]) -> NetworkArea {
    if points.len() < MIN_POINTS {
        return NetworkArea { points: points.to_vec(), ..default() };
    }

    let ref_lat = points.iter().map(|p| p.0).sum::<f64>() / points.len() as f64;
    let ref_lon = points.iter().map(|p| p.1).sum::<f64>() / points.len() as f64;
    let locals: Vec<polygon::LocalPoint> = points
        .iter()
        .map(|&(lat, lon)| polygon::project(ref_lon, ref_lat, lon, lat))
        .collect();

    let Some(square) = polygon::min_bounding_square(&locals) else {
        return NetworkArea { points: points.to_vec(), ..default() };
    };

    let center = polygon::unproject(ref_lon, ref_lat, square.center);
    let corners = square.corners().map(|lp| polygon::unproject(ref_lon, ref_lat, lp));

    let (clat, clon) = center;
    let mut half_ns_km = 0.0_f64;
    let mut half_ew_km = 0.0_f64;
    for &(lat, lon) in &corners {
        half_ns_km = half_ns_km.max((lat - clat).abs() * 110.574);
        half_ew_km = half_ew_km.max((lon - clon).abs() * 111.320 * clat.to_radians().cos());
    }
    let fetch_size_km = (half_ns_km.max(half_ew_km) * 2.0) as f32;
    let side_km = square.side_km();

    let fetch_half_km = fetch_size_km as f64 * 0.5;
    let lat_delta = fetch_half_km / 110.574;
    let lon_delta = fetch_half_km / (111.320 * clat.to_radians().cos());
    let fetch_corners = [
        (clat + lat_delta, clon + lon_delta),
        (clat + lat_delta, clon - lon_delta),
        (clat - lat_delta, clon - lon_delta),
        (clat - lat_delta, clon + lon_delta),
    ];

    NetworkArea {
        points: points.to_vec(),
        corners,
        center,
        side_km,
        rotation_deg: square.rotation.to_degrees(),
        fetch_size_km,
        fetch_corners,
        valid: true,
        over_limit: side_km > MAX_SIDE_KM,
    }
}

/// Current pan/zoom of the map content, relative to its base 1:1 layout.
/// `zoom` must never be 0 — `apply_pan_zoom` applies it directly as the
/// content node's scale, and `#[derive(Default)]` would give `0.0` here,
/// collapsing the whole map (image, cities, points) to nothing on the very
/// first frame after `MapView::default()` is inserted.
#[derive(Resource)]
pub(crate) struct MapView {
    pub zoom: f32,
    pub pan: Vec2,
}

impl Default for MapView {
    fn default() -> Self {
        Self { zoom: ZOOM_MIN, pan: Vec2::ZERO }
    }
}

impl MapView {
    fn reset(&mut self) {
        self.zoom = ZOOM_MIN;
        self.pan = Vec2::ZERO;
    }
}

/// Handle to the rasterized outline texture, kept around so the theme
/// toggle can re-rasterize it with the other palette's colors.
#[derive(Resource)]
pub(crate) struct SwedenMapHandle(pub Handle<Image>);

#[derive(Component)]
pub(crate) struct AreaSelectionRoot;

#[derive(Component)]
pub(crate) struct AreaBg;

#[derive(Component)]
pub(crate) struct MapViewport;

#[derive(Component)]
pub(crate) struct MapContent;

#[derive(Component)]
pub(crate) struct PolygonVisual;

#[derive(Component)]
pub(crate) struct CityDot;

#[derive(Component)]
pub(crate) struct HeadingText;

#[derive(Component)]
pub(crate) struct BodyText;

#[derive(Component)]
pub(crate) struct SummaryText;

#[derive(Component)]
pub(crate) struct WarningText;

#[derive(Component)]
pub(crate) struct SourceText;

#[derive(Component)]
pub(crate) struct GenerateTerrain;

#[derive(Component)]
pub(crate) struct AddStopButton;

#[derive(Component)]
pub(crate) struct AddStopLabel;

#[derive(Component)]
pub(crate) struct ClearButton;

#[derive(Component)]
pub(crate) struct PointsTableRoot;

#[derive(Component)]
pub(crate) struct RemovePointButton(usize);

#[derive(Component)]
pub(crate) struct SetBaseButton;

#[derive(Component)]
pub(crate) struct SetBaseLabel;

#[derive(Component)]
pub(crate) struct ZoomInButton;

#[derive(Component)]
pub(crate) struct ZoomOutButton;

fn spawn_zoom_button(
    parent: &mut ChildSpawnerCommands,
    label: &str,
    marker: impl Component,
    bg: Color,
    fg: Color,
) {
    parent
        .spawn((
            Button,
            Node {
                width: Val::Px(28.0),
                height: Val::Px(28.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border_radius: BorderRadius::all(Val::Px(4.0)),
                ..default()
            },
            BackgroundColor(bg),
            marker,
        ))
        .with_child((
            Text::new(label),
            TextFont { font_size: FontSize::Px(18.0), ..default() },
            TextColor(fg),
            Pickable::IGNORE,
        ));
}

pub fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    load_error: Option<Res<crate::terrain::TerrainLoadError>>,
    theme: Res<Theme>,
) {
    commands.insert_resource(PendingPoints::default());
    commands.insert_resource(PickMode::default());
    commands.insert_resource(recompute_network_area(&[]));
    commands.insert_resource(MapView::default());
    commands.insert_resource(BasePosition::default());

    let p = theme.palette();

    let map_handle = images.add(Image::new(
        Extent3d {
            width: sweden_geo::IMG_W,
            height: sweden_geo::IMG_H,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        sweden_geo::rasterize(theme.dark),
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    ));
    commands.insert_resource(SwedenMapHandle(map_handle.clone()));

    commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                column_gap: Val::Px(40.0),
                ..default()
            },
            BackgroundColor(p.bg),
            AreaSelectionRoot,
            AreaBg,
        ))
        .with_children(|root| {
            // Clipped viewport: fixed size, click target for adding points.
            // Its child `MapContent` carries the pan/zoom transform used in
            // review mode, so points/edges/city dots zoom along with the map.
            root.spawn((
                Node {
                    width: Val::Px(sweden_geo::IMG_W as f32),
                    height: Val::Px(sweden_geo::IMG_H as f32),
                    overflow: Overflow::clip(),
                    position_type: PositionType::Relative,
                    ..default()
                },
                Interaction::None,
                RelativeCursorPosition::default(),
                MapViewport,
            ))
            .with_children(|viewport| {
                viewport
                    .spawn((
                        Node {
                            width: Val::Px(sweden_geo::IMG_W as f32),
                            height: Val::Px(sweden_geo::IMG_H as f32),
                            ..default()
                        },
                        UiTransform::IDENTITY,
                        MapContent,
                        Pickable::IGNORE,
                    ))
                    .with_children(|content| {
                        content.spawn((
                            Node {
                                width: Val::Px(sweden_geo::IMG_W as f32),
                                height: Val::Px(sweden_geo::IMG_H as f32),
                                ..default()
                            },
                            ImageNode::new(map_handle),
                            Pickable::IGNORE,
                        ));

                        for (name, lat, lon) in sweden_geo::CITIES {
                            let (x, y) = sweden_geo::lonlat_to_pixel(*lon, *lat);
                            content.spawn((
                                Node {
                                    position_type: PositionType::Absolute,
                                    left: Val::Px(x - CITY_DOT_SIZE * 0.5),
                                    top: Val::Px(y - CITY_DOT_SIZE * 0.5),
                                    width: Val::Px(CITY_DOT_SIZE),
                                    height: Val::Px(CITY_DOT_SIZE),
                                    border_radius: BorderRadius::MAX,
                                    ..default()
                                },
                                BackgroundColor(p.accent),
                                Pickable::IGNORE,
                                CityDot,
                            ));
                            content.spawn((
                                Node {
                                    position_type: PositionType::Absolute,
                                    left: Val::Px(x + CITY_DOT_SIZE),
                                    top: Val::Px(y - 7.0),
                                    ..default()
                                },
                                Text::new(*name),
                                TextFont { font_size: FontSize::Px(11.0), ..default() },
                                TextColor(p.text.with_alpha(0.85)),
                                Pickable::IGNORE,
                            ));
                        }
                    });

                // Zoom controls — scroll-wheel zoom is unreliable on macOS
                // trackpads, so these buttons (Google Maps-style) are the
                // primary way to zoom. Siblings of `MapContent`, not
                // children, so they stay fixed in the corner instead of
                // scaling/panning with the map.
                viewport
                    .spawn((
                        Node {
                            position_type: PositionType::Absolute,
                            right: Val::Px(10.0),
                            bottom: Val::Px(10.0),
                            flex_direction: FlexDirection::Column,
                            ..default()
                        },
                        Pickable::IGNORE,
                    ))
                    .with_children(|controls| {
                        spawn_zoom_button(controls, "+", ZoomInButton, p.surface, p.text);
                        controls.spawn(Node { height: Val::Px(2.0), ..default() });
                        spawn_zoom_button(controls, "\u{2212}", ZoomOutButton, p.surface, p.text);
                    });
            });

            root.spawn(Node {
                width: Val::Px(440.0),
                height: Val::Px(sweden_geo::IMG_H as f32),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(14.0),
                overflow: Overflow::clip_y(),
                ..default()
            })
            .with_children(|panel| {
                panel.spawn((
                    Text::new("Select a network area"),
                    TextFont { font_size: FontSize::Px(30.0), ..default() },
                    TextColor(p.text),
                    HeadingText,
                ));
                panel.spawn((
                    Text::new(
                        "Click points on the map to draw the area's outline (3+ points). \
                         We'll fit the smallest square around them and fetch that terrain.",
                    ),
                    TextFont { font_size: FontSize::Px(15.0), ..default() },
                    TextColor(p.text.with_alpha(0.8)),
                    BodyText,
                ));

                panel.spawn((
                    Text::new(""),
                    TextFont { font_size: FontSize::Px(16.0), ..default() },
                    TextColor(p.accent),
                    SummaryText,
                ));
                panel.spawn((
                    Text::new(""),
                    TextFont { font_size: FontSize::Px(13.0), ..default() },
                    TextColor(Color::srgb(1.0, 0.55, 0.4)),
                    WarningText,
                ));

                panel.spawn(Node {
                    display: Display::Grid,
                    grid_template_columns: vec![RepeatedGridTrack::auto(4)],
                    column_gap: Val::Px(14.0),
                    row_gap: Val::Px(3.0),
                    ..default()
                })
                .with_children(|grid| {
                    for h in ["#", "Lat", "Lon", ""] {
                        grid.spawn((
                            Text::new(h),
                            TextFont { font_size: FontSize::Px(12.0), ..default() },
                            TextColor(p.accent),
                        ));
                    }
                })
                .insert(PointsTableRoot);

                panel.spawn(Node {
                    flex_direction: FlexDirection::Row,
                    column_gap: Val::Px(12.0),
                    ..default()
                })
                .with_children(|row| {
                    row.spawn((
                        Button,
                        Node {
                            padding: UiRect::axes(Val::Px(14.0), Val::Px(9.0)),
                            border_radius: BorderRadius::all(Val::Px(6.0)),
                            ..default()
                        },
                        BackgroundColor(p.surface),
                        AddStopButton,
                    ))
                    .with_child((
                        Text::new("Stop adding points"),
                        TextFont { font_size: FontSize::Px(14.0), ..default() },
                        TextColor(p.text),
                        AddStopLabel,
                    ));

                    row.spawn((
                        Button,
                        Node {
                            padding: UiRect::axes(Val::Px(14.0), Val::Px(9.0)),
                            border_radius: BorderRadius::all(Val::Px(6.0)),
                            ..default()
                        },
                        BackgroundColor(p.surface),
                        ClearButton,
                    ))
                    .with_child((
                        Text::new("Clear"),
                        TextFont { font_size: FontSize::Px(14.0), ..default() },
                        TextColor(p.text),
                    ));
                });

                panel
                    .spawn((
                        Button,
                        Node {
                            padding: UiRect::axes(Val::Px(14.0), Val::Px(9.0)),
                            border_radius: BorderRadius::all(Val::Px(6.0)),
                            ..default()
                        },
                        BackgroundColor(p.surface),
                        SetBaseButton,
                    ))
                    .with_child((
                        Text::new("Set base location"),
                        TextFont { font_size: FontSize::Px(14.0), ..default() },
                        TextColor(p.text),
                        SetBaseLabel,
                    ));

                panel
                    .spawn((
                        Button,
                        Node {
                            width: Val::Px(230.0),
                            height: Val::Px(48.0),
                            justify_content: JustifyContent::Center,
                            align_items: AlignItems::Center,
                            border_radius: BorderRadius::all(Val::Px(7.0)),
                            ..default()
                        },
                        BackgroundColor(p.accent),
                        GenerateTerrain,
                    ))
                    .with_child((
                        Text::new("Generate terrain"),
                        TextFont { font_size: FontSize::Px(18.0), ..default() },
                        TextColor(p.bg),
                    ));
                panel.spawn((
                    Text::new("Terrain source: Lantmateriet (local)"),
                    TextFont { font_size: FontSize::Px(14.0), ..default() },
                    TextColor(p.subtext),
                    SourceText,
                ));
                if let Some(error) = load_error.as_ref() {
                    panel.spawn((
                        Text::new(format!("Last attempt failed: {}", error.0)),
                        TextFont { font_size: FontSize::Px(14.0), ..default() },
                        TextColor(p.danger),
                    ));
                }
            });
        });
}

/// Add a point when the map is clicked in `Adding` mode.
pub fn add_point_on_click(
    map_q: Query<(&Interaction, &RelativeCursorPosition), (With<MapViewport>, Changed<Interaction>)>,
    mode: Res<PickMode>,
    mut points: ResMut<PendingPoints>,
    mut base: ResMut<BasePosition>,
) {
    if *mode != PickMode::Adding {
        return;
    }
    for (interaction, cursor) in &map_q {
        if *interaction != Interaction::Pressed {
            continue;
        }
        let Some(normalized) = cursor.normalized else {
            continue;
        };
        let px = (normalized.x + 0.5) * sweden_geo::IMG_W as f32;
        let py = (normalized.y + 0.5) * sweden_geo::IMG_H as f32;
        let (lon, lat) = sweden_geo::pixel_to_lonlat(px, py);
        if sweden_geo::point_in_sweden(lon, lat) {
            points.0.push((lat, lon));
            // The shape changed — a previously chosen base may no longer sit
            // inside it, so make the user re-confirm it.
            base.0 = None;
        }
    }
}

/// Base placements farther than this from the network-area square are rejected.
const MAX_BASE_DISTANCE_KM: f64 = 3.0;

/// Place (or cancel placing) the base while in `PlacingBase` mode. Must land
/// within `MAX_BASE_DISTANCE_KM` of the network-area square.
pub fn place_base_on_click(
    map_q: Query<(&Interaction, &RelativeCursorPosition), (With<MapViewport>, Changed<Interaction>)>,
    mut mode: ResMut<PickMode>,
    net: Res<NetworkArea>,
    mut base: ResMut<BasePosition>,
) {
    if *mode != PickMode::PlacingBase {
        return;
    }
    for (interaction, cursor) in &map_q {
        if *interaction != Interaction::Pressed {
            continue;
        }
        let Some(normalized) = cursor.normalized else {
            continue;
        };
        let px = (normalized.x + 0.5) * sweden_geo::IMG_W as f32;
        let py = (normalized.y + 0.5) * sweden_geo::IMG_H as f32;
        let (lon, lat) = sweden_geo::pixel_to_lonlat(px, py);
        if net.distance_to_square_km(lat, lon) <= MAX_BASE_DISTANCE_KM {
            base.0 = Some((lat, lon));
            *mode = PickMode::Reviewing;
        }
    }
}

pub fn point_table_and_buttons(
    remove_buttons: Query<(&Interaction, &RemovePointButton), Changed<Interaction>>,
    add_stop: Query<&Interaction, (Changed<Interaction>, With<AddStopButton>)>,
    clear: Query<&Interaction, (Changed<Interaction>, With<ClearButton>)>,
    set_base: Query<&Interaction, (Changed<Interaction>, With<SetBaseButton>)>,
    mut points: ResMut<PendingPoints>,
    mut mode: ResMut<PickMode>,
    mut view: ResMut<MapView>,
    mut base: ResMut<BasePosition>,
    net: Res<NetworkArea>,
) {
    for (interaction, remove) in &remove_buttons {
        if *interaction == Interaction::Pressed && remove.0 < points.0.len() {
            points.0.remove(remove.0);
            base.0 = None;
        }
    }

    if add_stop.iter().any(|i| *i == Interaction::Pressed) {
        *mode = match *mode {
            PickMode::Adding => {
                view.reset();
                PickMode::Reviewing
            }
            PickMode::Reviewing | PickMode::PlacingBase => PickMode::Adding,
        };
    }

    if set_base.iter().any(|i| *i == Interaction::Pressed) {
        *mode = match *mode {
            PickMode::PlacingBase => PickMode::Reviewing,
            PickMode::Adding | PickMode::Reviewing if net.valid && !net.over_limit => {
                view.reset();
                PickMode::PlacingBase
            }
            other => other,
        };
    }

    if clear.iter().any(|i| *i == Interaction::Pressed) {
        points.0.clear();
        *mode = PickMode::Adding;
        view.reset();
        base.0 = None;
    }
}

/// Zoom (scroll) works in any mode; pan (drag) only while reviewing, so it
/// doesn't fight click-to-place-a-point. Both only act while the cursor is
/// over the viewport.
pub fn pan_zoom(
    mode: Res<PickMode>,
    viewport_q: Query<&RelativeCursorPosition, With<MapViewport>>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut view: ResMut<MapView>,
) {
    let Ok(cursor) = viewport_q.single() else {
        return;
    };
    if !cursor.cursor_over {
        return;
    }

    if scroll.delta.y != 0.0 {
        view.zoom = (view.zoom + scroll.delta.y * 0.15).clamp(ZOOM_MIN, ZOOM_MAX);
    }
    if *mode == PickMode::Reviewing
        && mouse_button.pressed(MouseButton::Left)
        && motion.delta != Vec2::ZERO
    {
        view.pan += motion.delta;
    }
}

const ZOOM_STEP: f32 = 0.5;

/// Google Maps-style +/- buttons — the reliable zoom path, since scroll-wheel
/// zoom is flaky on macOS trackpads (two-finger scroll doesn't consistently
/// reach `AccumulatedMouseScroll`).
pub fn zoom_buttons(
    zoom_in: Query<&Interaction, (Changed<Interaction>, With<ZoomInButton>)>,
    zoom_out: Query<&Interaction, (Changed<Interaction>, With<ZoomOutButton>)>,
    mut view: ResMut<MapView>,
) {
    if zoom_in.iter().any(|i| *i == Interaction::Pressed) {
        view.zoom = (view.zoom + ZOOM_STEP).clamp(ZOOM_MIN, ZOOM_MAX);
    }
    if zoom_out.iter().any(|i| *i == Interaction::Pressed) {
        view.zoom = (view.zoom - ZOOM_STEP).clamp(ZOOM_MIN, ZOOM_MAX);
    }
}

pub fn apply_pan_zoom(view: Res<MapView>, mut content_q: Query<&mut UiTransform, With<MapContent>>) {
    if !view.is_changed() {
        return;
    }
    if let Ok(mut transform) = content_q.single_mut() {
        transform.scale = Vec2::splat(view.zoom.max(0.01));
        transform.translation = Val2::px(view.pan.x, view.pan.y);
    }
}

/// Recompute the network-area preview whenever the point list changes.
pub fn recompute_area_on_change(points: Res<PendingPoints>, mut area: ResMut<NetworkArea>) {
    if !points.is_changed() {
        return;
    }
    *area = recompute_network_area(&points.0);
}

/// Rebuild the point dots, polygon edges, and bounding-square outline
/// whenever the point list (or the computed square) changes.
pub fn redraw_polygon(
    mut commands: Commands,
    points: Res<PendingPoints>,
    net: Res<NetworkArea>,
    base: Res<BasePosition>,
    theme: Res<Theme>,
    content_q: Query<Entity, With<MapContent>>,
    visuals: Query<Entity, With<PolygonVisual>>,
    mut last_len: Local<usize>,
) {
    if !points.is_changed() && !theme.is_changed() && !base.is_changed() {
        return;
    }
    *last_len = points.0.len();

    for entity in &visuals {
        commands.entity(entity).despawn();
    }
    let Ok(content) = content_q.single() else {
        return;
    };

    let pal = theme.palette();
    let pixels: Vec<Vec2> = points
        .0
        .iter()
        .map(|&(lat, lon)| sweden_geo::lonlat_to_pixel(lon, lat).into())
        .collect();

    commands.entity(content).with_children(|parent| {
        // Polygon edges (only meaningful once the shape is closed, 3+ points).
        if pixels.len() >= 2 {
            let n = pixels.len();
            let edge_count = if pixels.len() >= 3 { n } else { n - 1 };
            for i in 0..edge_count {
                spawn_edge(parent, pixels[i], pixels[(i + 1) % n], pal.accent, EDGE_THICKNESS);
            }
        }

        // The area that's actually going to be fetched (axis-aligned, always
        // a bit bigger than the rotated square) — drawn first, dimmer, so
        // the network-area square reads as the "real" selection on top.
        if net.valid {
            let fetch_px: Vec<Vec2> = net
                .fetch_corners
                .iter()
                .map(|&(lat, lon)| sweden_geo::lonlat_to_pixel(lon, lat).into())
                .collect();
            for i in 0..4 {
                spawn_edge(parent, fetch_px[i], fetch_px[(i + 1) % 4], pal.text.with_alpha(0.35), 1.0);
            }
        }

        // Bounding square outline — the "network area".
        if net.valid {
            let square_color = if net.over_limit { Color::srgb(1.0, 0.35, 0.3) } else { pal.base };
            let corners_px: Vec<Vec2> = net
                .corners
                .iter()
                .map(|&(lat, lon)| sweden_geo::lonlat_to_pixel(lon, lat).into())
                .collect();
            for i in 0..4 {
                spawn_edge(parent, corners_px[i], corners_px[(i + 1) % 4], square_color, SQUARE_THICKNESS);
            }
        }

        // Base marker.
        if let Some((lat, lon)) = base.0 {
            let p: Vec2 = sweden_geo::lonlat_to_pixel(lon, lat).into();
            const BASE_SIZE: f32 = 11.0;
            parent.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(p.x - BASE_SIZE * 0.5),
                    top: Val::Px(p.y - BASE_SIZE * 0.5),
                    width: Val::Px(BASE_SIZE),
                    height: Val::Px(BASE_SIZE),
                    ..default()
                },
                BackgroundColor(pal.base),
                Pickable::IGNORE,
                PolygonVisual,
            ));
            parent.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(p.x + BASE_SIZE),
                    top: Val::Px(p.y - 8.0),
                    ..default()
                },
                Text::new("BASE"),
                TextFont { font_size: FontSize::Px(11.0), ..default() },
                TextColor(pal.base),
                Pickable::IGNORE,
                PolygonVisual,
            ));
        }

        // Point dots + index labels, drawn last so they sit on top.
        for (i, &p) in pixels.iter().enumerate() {
            parent.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(p.x - POINT_DOT_SIZE * 0.5),
                    top: Val::Px(p.y - POINT_DOT_SIZE * 0.5),
                    width: Val::Px(POINT_DOT_SIZE),
                    height: Val::Px(POINT_DOT_SIZE),
                    border_radius: BorderRadius::MAX,
                    ..default()
                },
                BackgroundColor(pal.drone),
                Pickable::IGNORE,
                PolygonVisual,
            ));
            parent.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(p.x + POINT_DOT_SIZE),
                    top: Val::Px(p.y - 8.0),
                    ..default()
                },
                Text::new(format!("{}", i + 1)),
                TextFont { font_size: FontSize::Px(11.0), ..default() },
                TextColor(pal.text),
                Pickable::IGNORE,
                PolygonVisual,
            ));
        }
    });
}

/// A thin bar Node rotated to connect `from` to `to` (screen-space pixels).
/// `UiTransform` rotates a node about its own center, so we position the
/// unrotated bar centered on the segment's midpoint and rotate from there.
fn spawn_edge(parent: &mut ChildSpawnerCommands, from: Vec2, to: Vec2, color: Color, thickness: f32) {
    let delta = to - from;
    let length = delta.length().max(0.01);
    let angle = delta.y.atan2(delta.x);
    let mid = (from + to) * 0.5;
    parent.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(mid.x - length * 0.5),
            top: Val::Px(mid.y - thickness * 0.5),
            width: Val::Px(length),
            height: Val::Px(thickness),
            ..default()
        },
        UiTransform::from_rotation(Rot2::radians(angle)),
        BackgroundColor(color),
        Pickable::IGNORE,
        PolygonVisual,
    ));
}

/// Rebuild the points table whenever the point list changes.
pub fn redraw_table(
    mut commands: Commands,
    points: Res<PendingPoints>,
    theme: Res<Theme>,
    table_q: Query<(Entity, Option<&Children>), With<PointsTableRoot>>,
    mut last_len: Local<usize>,
) {
    if !points.is_changed() && !theme.is_changed() {
        return;
    }
    *last_len = points.0.len();

    let Ok((table_entity, children)) = table_q.single() else {
        return;
    };
    let pal = theme.palette();

    // Keep the 4 header cells (first 4 children); drop the rest.
    if let Some(children) = children {
        for child in children.iter().skip(4) {
            commands.entity(child).despawn();
        }
    }

    commands.entity(table_entity).with_children(|grid| {
        for (i, &(lat, lon)) in points.0.iter().enumerate() {
            grid.spawn((
                Text::new(format!("{}", i + 1)),
                TextFont { font_size: FontSize::Px(12.0), ..default() },
                TextColor(pal.text),
            ));
            grid.spawn((
                Text::new(format!("{lat:.4}")),
                TextFont { font_size: FontSize::Px(12.0), ..default() },
                TextColor(pal.text),
            ));
            grid.spawn((
                Text::new(format!("{lon:.4}")),
                TextFont { font_size: FontSize::Px(12.0), ..default() },
                TextColor(pal.text),
            ));
            grid.spawn((
                Button,
                Node { padding: UiRect::axes(Val::Px(5.0), Val::Px(1.0)), ..default() },
                BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.08)),
                RemovePointButton(i),
            ))
            .with_child((
                Text::new("x"),
                TextFont { font_size: FontSize::Px(11.0), ..default() },
                TextColor(Color::srgb(1.0, 0.55, 0.5)),
            ));
        }
    });
}

/// Keeps the summary/warning text, add-stop label, and generate-button
/// affordance in sync with the current picking state — cheap enough to run
/// every frame rather than tracking another change signature.
pub fn update_status_text(
    points: Res<PendingPoints>,
    net: Res<NetworkArea>,
    mode: Res<PickMode>,
    base: Res<BasePosition>,
    theme: Res<Theme>,
    mut summary_q: Query<&mut Text, (With<SummaryText>, Without<WarningText>, Without<AddStopLabel>, Without<SetBaseLabel>)>,
    mut warning_q: Query<&mut Text, (With<WarningText>, Without<SummaryText>, Without<AddStopLabel>, Without<SetBaseLabel>)>,
    mut label_q: Query<&mut Text, (With<AddStopLabel>, Without<SummaryText>, Without<WarningText>, Without<SetBaseLabel>)>,
    mut base_label_q: Query<&mut Text, (With<SetBaseLabel>, Without<SummaryText>, Without<WarningText>, Without<AddStopLabel>)>,
    mut generate_bg: Query<&mut BackgroundColor, With<GenerateTerrain>>,
) {
    let pal = theme.palette();

    if let Ok(mut text) = summary_q.single_mut() {
        **text = if net.valid {
            let base_str = match base.0 {
                Some((lat, lon)) => format!("base at {lat:.4}, {lon:.4}"),
                None => "base not set".into(),
            };
            format!(
                "{} points  |  {:.1} km square  |  rotated {:.0}°  |  {base_str}",
                points.0.len(),
                net.side_km,
                net.rotation_deg
            )
        } else {
            format!("{} point(s) — need at least {MIN_POINTS} to form an area", points.0.len())
        };
    }

    if let Ok(mut text) = warning_q.single_mut() {
        **text = if net.over_limit {
            format!("Area is {:.1} km — over the {MAX_SIDE_KM:.0} km limit. Remove or move points.", net.side_km)
        } else if *mode == PickMode::PlacingBase {
            format!("Click within {MAX_BASE_DISTANCE_KM:.0} km of the network area to place the base.")
        } else if net.valid && base.0.is_none() {
            "Set the base location before generating terrain.".into()
        } else {
            String::new()
        };
    }

    if let Ok(mut text) = label_q.single_mut() {
        **text = match *mode {
            PickMode::Adding => "Stop adding points".into(),
            PickMode::Reviewing | PickMode::PlacingBase => "Add points".into(),
        };
    }

    if let Ok(mut text) = base_label_q.single_mut() {
        **text = match *mode {
            PickMode::PlacingBase => "Cancel placing base".into(),
            _ if base.0.is_some() => "Change base location".into(),
            _ => "Set base location".into(),
        };
    }

    let ready = net.valid && !net.over_limit && base.0.is_some();
    if let Ok(mut bg) = generate_bg.single_mut() {
        bg.0 = if ready { pal.accent } else { pal.accent.with_alpha(0.35) };
    }
}

pub fn generate_terrain(
    generate: Query<&Interaction, (Changed<Interaction>, With<GenerateTerrain>)>,
    net: Res<NetworkArea>,
    base: Res<BasePosition>,
    mut area: ResMut<ScenarioArea>,
    mut commands: Commands,
    mut next_state: ResMut<NextState<AppState>>,
) {
    if !generate.iter().any(|i| *i == Interaction::Pressed) {
        return;
    }
    if !net.valid || net.over_limit || base.0.is_none() {
        return;
    }
    area.name = "Network area";
    area.latitude = net.center.0;
    area.longitude = net.center.1;
    area.size_km = net.fetch_size_km.max(1.0);
    commands.remove_resource::<crate::terrain::TerrainLoadError>();
    next_state.set(AppState::LoadingTerrain);
}

pub fn cleanup(mut commands: Commands, roots: Query<Entity, With<AreaSelectionRoot>>) {
    for entity in &roots {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<SwedenMapHandle>();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bbox_contains_selected_center() {
        let area = ScenarioArea::default();
        let [west, south, east, north] = area.wgs84_bbox();
        assert!(west < area.longitude && area.longitude < east);
        assert!(south < area.latitude && area.latitude < north);
    }

    #[test]
    fn bbox_represents_requested_square_size() {
        let area = ScenarioArea::default();
        let [west, south, east, north] = area.wgs84_bbox();
        let north_south_km = (north - south) * 110.574;
        let east_west_km =
            (east - west) * 111.320 * area.latitude.to_radians().cos();

        assert!((north_south_km - area.size_km as f64).abs() < 0.001);
        assert!((east_west_km - area.size_km as f64).abs() < 0.001);
    }

    #[test]
    fn fewer_than_3_points_is_invalid() {
        let net = recompute_network_area(&[(59.0, 18.0), (59.1, 18.1)]);
        assert!(!net.valid);
    }

    #[test]
    fn a_triangle_produces_a_valid_covering_square() {
        let net = recompute_network_area(&[(59.0, 18.0), (59.05, 18.05), (58.98, 18.06)]);
        assert!(net.valid);
        assert!(net.side_km > 0.0);
        assert!(net.fetch_size_km as f64 >= net.side_km);
        assert!(!net.over_limit);
    }

    #[test]
    fn a_huge_triangle_is_over_the_limit() {
        let net = recompute_network_area(&[(58.0, 17.0), (60.0, 19.0), (58.0, 19.0)]);
        assert!(net.valid);
        assert!(net.over_limit);
    }
}
