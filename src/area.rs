use std::collections::{HashMap, HashSet};

use bevy::{
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    prelude::*,
    ui::RelativeCursorPosition,
};

use crate::{polygon, sweden_geo, theme::Theme, tiles, AppState};

/// Kept for existing callers (terrain fetch math) — the network-area picker
/// now drives `ScenarioArea.size_km` dynamically instead.
pub const AREA_SIZE_KM: f32 = 20.0;

/// On-screen size of the map viewport. No longer tied to any fixed source
/// image (the map is live OSM tiles, effectively infinite) — this is just
/// how big a window into the world the panel gives you.
const MAP_VIEWPORT_W: f32 = 343.0;
const MAP_VIEWPORT_H: f32 = 760.0;

fn viewport_size() -> Vec2 {
    Vec2::new(MAP_VIEWPORT_W, MAP_VIEWPORT_H)
}

const MIN_POINTS: usize = 3;
const MAX_SIDE_KM: f64 = 50.0;
const POINT_DOT_SIZE: f32 = 9.0;
const EDGE_THICKNESS: f32 = 2.0;
const SQUARE_THICKNESS: f32 = 2.0;

/// Sweden's rough centroid — the default view on first opening the picker.
const DEFAULT_CENTER_LON: f64 = 17.65;
const DEFAULT_CENTER_LAT: f64 = 62.15;
/// Shows roughly the whole country in `MAP_VIEWPORT_W`/`H`.
const DEFAULT_ZOOM: u8 = 5;

/// Sweden's bounding box, with a little slack. Clamps `MapView::center` so
/// panning can't wander off into Norway/Finland/Denmark — this only pins the
/// *viewport center*, though: OSM tiles don't respect political borders, so
/// any tile straddling the border still shows both sides regardless of this
/// clamp, and zoomed out enough you'll still see neighboring coastline at
/// the edge of the viewport. There's no way around that with raster tiles;
/// this just stops you from being able to scroll away and look at Oslo.
const SWEDEN_LON_RANGE: (f64, f64) = (10.8, 24.5);
const SWEDEN_LAT_RANGE: (f64, f64) = (55.0, 69.4);

fn clamp_to_sweden(center: (f64, f64)) -> (f64, f64) {
    (center.0.clamp(SWEDEN_LON_RANGE.0, SWEDEN_LON_RANGE.1), center.1.clamp(SWEDEN_LAT_RANGE.0, SWEDEN_LAT_RANGE.1))
}

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

#[derive(Resource, Default, Clone, Copy, PartialEq, Eq, Debug)]
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
        // `center` is `(lon, lat)` — the order `recompute_network_area`
        // actually produces it in (via `polygon::unproject`, which returns
        // `(lon, lat)`). Destructuring it as `(lat, lon)` here used to feed
        // `project` a reference point with lon/lat swapped, which for a
        // point ~63°N ~17°E silently computed a "distance" around 7,000 km
        // instead of the real few-km distance — this is what made "click to
        // place the base" always land outside `MAX_BASE_DISTANCE_KM` and
        // appear to do nothing.
        let (clon, clat) = self.center;
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

/// What part of the world the map viewport currently shows: an OSM integer
/// zoom level and the lon/lat shown at the viewport's own center. Unlike the
/// old fixed-image map, there's no "whole map" bounds to clamp against — OSM
/// tiles cover the world, so panning/zooming is unbounded (bounded only by
/// `tiles::MIN_ZOOM`/`MAX_ZOOM`).
#[derive(Resource)]
pub(crate) struct MapView {
    pub zoom: u8,
    pub center: (f64, f64),
}

impl Default for MapView {
    fn default() -> Self {
        Self { zoom: DEFAULT_ZOOM, center: (DEFAULT_CENTER_LON, DEFAULT_CENTER_LAT) }
    }
}

impl MapView {
    fn reset(&mut self) {
        *self = Self::default();
    }
}

#[derive(Component)]
pub(crate) struct AreaSelectionRoot;

#[derive(Component)]
pub(crate) struct AreaBg;

#[derive(Component)]
pub(crate) struct MapViewport;

#[derive(Component)]
pub(crate) struct MapContent;

/// Marks a spawned OSM tile image entity.
#[derive(Component)]
struct MapTile;

/// One spawned tile entity, and whether it's showing the real tile yet.
struct SpawnedTile {
    entity: Entity,
    /// `false` while this is a temporary placeholder — a crop of an
    /// already-cached, coarser ancestor tile shown so zooming in doesn't
    /// leave a blank gap while the real tile loads (see `cached_ancestor`).
    /// Swapped for the real tile the moment it's ready.
    is_final: bool,
}

/// Currently-spawned tile entities, so `sync_map_tiles` can diff against
/// "what's actually needed this frame" instead of despawning/respawning
/// everything every frame.
#[derive(Resource, Default)]
pub(crate) struct SpawnedTiles {
    entities: HashMap<tiles::TileKey, SpawnedTile>,
    /// Zoom the spawned tiles were fetched at — every tile must be dropped
    /// and refetched on a zoom change (a different zoom is a different
    /// pixel grid entirely, not a resize of the same one).
    zoom: Option<u8>,
}

#[derive(Component)]
pub(crate) struct PolygonVisual;

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
    load_error: Option<Res<crate::terrain::TerrainLoadError>>,
    theme: Res<Theme>,
) {
    commands.insert_resource(PendingPoints::default());
    commands.insert_resource(PickMode::default());
    commands.insert_resource(recompute_network_area(&[]));
    commands.insert_resource(MapView::default());
    commands.insert_resource(BasePosition::default());
    commands.insert_resource(SpawnedTiles::default());

    let p = theme.palette();

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
            // `MapContent` holds every OSM tile plus the point/polygon/base
            // overlay — all positioned directly in screen space from the
            // current `MapView` (see `lonlat_to_screen_px`), rebuilt
            // whenever pan/zoom/points change rather than carrying any
            // scale/translate transform of its own.
            root.spawn((
                Node {
                    width: Val::Px(MAP_VIEWPORT_W),
                    height: Val::Px(MAP_VIEWPORT_H),
                    overflow: Overflow::clip(),
                    position_type: PositionType::Relative,
                    ..default()
                },
                BackgroundColor(p.surface),
                Interaction::None,
                RelativeCursorPosition::default(),
                MapViewport,
            ))
            .with_children(|viewport| {
                viewport.spawn((
                    Node {
                        width: Val::Px(MAP_VIEWPORT_W),
                        height: Val::Px(MAP_VIEWPORT_H),
                        ..default()
                    },
                    MapContent,
                    Pickable::IGNORE,
                ));

                // Zoom controls — scroll-wheel zoom is unreliable on macOS
                // trackpads, so these buttons (Google Maps-style) are the
                // primary way to zoom. Siblings of `MapContent`, not
                // children, so they stay fixed in the corner instead of
                // panning with the map.
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

                // OpenStreetMap's tile usage policy requires visible
                // attribution wherever the tiles are displayed:
                // https://operations.osmfoundation.org/policies/tiles/
                viewport.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(4.0),
                        bottom: Val::Px(2.0),
                        padding: UiRect::axes(Val::Px(4.0), Val::Px(1.0)),
                        ..default()
                    },
                    BackgroundColor(Color::BLACK.with_alpha(0.35)),
                    Pickable::IGNORE,
                    children![(
                        Text::new("\u{00A9} OpenStreetMap contributors"),
                        TextFont { font_size: FontSize::Px(9.0), ..default() },
                        TextColor(Color::WHITE.with_alpha(0.85)),
                    )],
                ));
            });

            root.spawn(Node {
                width: Val::Px(440.0),
                height: Val::Px(MAP_VIEWPORT_H),
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

/// Project lon/lat to a screen position (px, relative to the viewport's own
/// top-left — the same frame `RelativeCursorPosition::normalized` uses) at
/// the current pan/zoom. The inverse of `cursor_to_lonlat`.
fn lonlat_to_screen_px(lon: f64, lat: f64, view: &MapView) -> Vec2 {
    let world = tiles::lonlat_to_world_px(lon, lat, view.zoom);
    let center_world = tiles::lonlat_to_world_px(view.center.0, view.center.1, view.zoom);
    world - center_world + viewport_size() * 0.5
}

/// Turn a click's viewport-relative `normalized` position into lon/lat at
/// the current pan/zoom — the inverse of `lonlat_to_screen_px`.
fn cursor_to_lonlat(normalized: Vec2, view: &MapView) -> (f64, f64) {
    let screen = (normalized + Vec2::splat(0.5)) * viewport_size();
    let center_world = tiles::lonlat_to_world_px(view.center.0, view.center.1, view.zoom);
    let world = center_world + screen - viewport_size() * 0.5;
    tiles::world_px_to_lonlat(world, view.zoom)
}

/// Add a point when the map is clicked in `Adding` mode.
pub fn add_point_on_click(
    map_q: Query<(&Interaction, &RelativeCursorPosition), (With<MapViewport>, Changed<Interaction>)>,
    mode: Res<PickMode>,
    view: Res<MapView>,
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
        let (lon, lat) = cursor_to_lonlat(normalized, &view);
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
    view: Res<MapView>,
    mut base: ResMut<BasePosition>,
) {
    if *mode != PickMode::PlacingBase {
        return;
    }
    for (interaction, cursor) in &map_q {
        info!("[base] viewport interaction changed to {interaction:?} while placing base");
        if *interaction != Interaction::Pressed {
            continue;
        }
        let Some(normalized) = cursor.normalized else {
            warn!("[base] click registered but cursor.normalized was None — no position to place at");
            continue;
        };
        let (lon, lat) = cursor_to_lonlat(normalized, &view);
        let distance = net.distance_to_square_km(lat, lon);
        info!(
            "[base] click at normalized {normalized:?} -> lon={lon:.5} lat={lat:.5}, {distance:.3} km from area (limit {MAX_BASE_DISTANCE_KM})"
        );
        if distance <= MAX_BASE_DISTANCE_KM {
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

    // Mode toggles deliberately leave `view` (zoom/pan) alone — switching
    // modes shouldn't throw away where you were looking, especially right
    // before placing the base, which usually wants to stay zoomed in for
    // precision.
    if add_stop.iter().any(|i| *i == Interaction::Pressed) {
        *mode = match *mode {
            PickMode::Adding => PickMode::Reviewing,
            PickMode::Reviewing | PickMode::PlacingBase => PickMode::Adding,
        };
    }

    if set_base.iter().any(|i| *i == Interaction::Pressed) {
        let before = *mode;
        *mode = match *mode {
            PickMode::PlacingBase => PickMode::Reviewing,
            PickMode::Adding | PickMode::Reviewing if net.valid && !net.over_limit => {
                PickMode::PlacingBase
            }
            other => other,
        };
        info!(
            "[base] Set base location pressed: {before:?} -> {:?} (net.valid={}, net.over_limit={})",
            *mode, net.valid, net.over_limit
        );
    }

    if clear.iter().any(|i| *i == Interaction::Pressed) {
        points.0.clear();
        *mode = PickMode::Adding;
        view.reset();
        base.0 = None;
    }
}

/// Adjust `view` so the lon/lat currently under `anchor_px` (screen space,
/// relative to the viewport's own top-left) stays visually fixed while zoom
/// changes to `new_zoom`. This is what makes zoom feel anchored to the
/// cursor (or the viewport's center, for the +/- buttons) instead of always
/// re-centering on whatever `view.center` already was.
fn rezoom_around(view: &mut MapView, anchor_px: Vec2, new_zoom: u8) {
    let (lon, lat) = cursor_to_lonlat(anchor_px / viewport_size() - Vec2::splat(0.5), view);
    view.zoom = new_zoom;
    let anchor_world = tiles::lonlat_to_world_px(lon, lat, new_zoom);
    let new_center_world = anchor_world - anchor_px + viewport_size() * 0.5;
    view.center = clamp_to_sweden(tiles::world_px_to_lonlat(new_center_world, new_zoom));
}

/// Zoom (scroll) works in any mode. Pan works two ways: left-drag while
/// reviewing (kept for continuity with the old behavior), and right-drag in
/// *any* mode — including `Adding`, where left-drag is reserved for
/// click-to-place-a-point — so panning is always available regardless of
/// what you're doing. Both only act while the cursor is over the viewport.
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
        // One tile zoom level per notch — OSM tiles only exist at integer
        // zooms, so (unlike the old continuous multiplier) this steps by
        // exactly ±1 regardless of how big one frame's scroll delta is.
        let step: i16 = if scroll.delta.y > 0.0 { 1 } else { -1 };
        let new_zoom = (view.zoom as i16 + step).clamp(tiles::MIN_ZOOM as i16, tiles::MAX_ZOOM as i16) as u8;
        if new_zoom != view.zoom {
            // `normalized` is in [-0.5, 0.5] over the viewport's own box —
            // zoom around wherever the cursor actually is.
            if let Some(normalized) = cursor.normalized {
                let anchor = (normalized + Vec2::splat(0.5)) * viewport_size();
                rezoom_around(&mut view, anchor, new_zoom);
            } else {
                view.zoom = new_zoom;
            }
        }
    }
    let panning = (*mode == PickMode::Reviewing && mouse_button.pressed(MouseButton::Left))
        || mouse_button.pressed(MouseButton::Right);
    if panning && motion.delta != Vec2::ZERO {
        let center_world = tiles::lonlat_to_world_px(view.center.0, view.center.1, view.zoom);
        view.center = clamp_to_sweden(tiles::world_px_to_lonlat(center_world - motion.delta, view.zoom));
    }
}

/// Google Maps-style +/- buttons — the reliable zoom path, since scroll-wheel
/// zoom is flaky on macOS trackpads (two-finger scroll doesn't consistently
/// reach `AccumulatedMouseScroll`). Zooms around the viewport's center, since
/// there's no cursor-hover point to anchor to for a button press.
pub fn zoom_buttons(
    zoom_in: Query<&Interaction, (Changed<Interaction>, With<ZoomInButton>)>,
    zoom_out: Query<&Interaction, (Changed<Interaction>, With<ZoomOutButton>)>,
    mut view: ResMut<MapView>,
) {
    let center = viewport_size() * 0.5;
    if zoom_in.iter().any(|i| *i == Interaction::Pressed) && view.zoom < tiles::MAX_ZOOM {
        let new_zoom = view.zoom + 1;
        rezoom_around(&mut view, center, new_zoom);
    }
    if zoom_out.iter().any(|i| *i == Interaction::Pressed) && view.zoom > tiles::MIN_ZOOM {
        let new_zoom = view.zoom - 1;
        rezoom_around(&mut view, center, new_zoom);
    }
}

/// How many zoom levels up `cached_ancestor` is willing to search for a
/// placeholder. Ordinary ±1-step zooming only ever needs 1 (you were just
/// there), but this covers e.g. a `Clear` that jumps back to the default
/// zoom, or one zoom level's fetch failing outright.
const MAX_PLACEHOLDER_ANCESTOR_STEPS: u8 = 6;

/// Find the nearest already-cached ancestor of `key` (by successively
/// halving zoom) and the sub-rectangle of that ancestor's tile image
/// corresponding to `key`'s coverage — a coarser, already-loaded crop to
/// show in place of `key` while its own fetch is still in flight, the same
/// "blurry-then-sharp" placeholder every slippy map uses instead of leaving
/// a blank gap during a zoom-in. `None` if no ancestor within
/// `MAX_PLACEHOLDER_ANCESTOR_STEPS` levels is cached yet.
fn cached_ancestor(cache: &tiles::TileCache, key: tiles::TileKey) -> Option<(Handle<Image>, Rect)> {
    let (mut z, mut x, mut y) = (key.z, key.x, key.y);
    for steps in 1..=MAX_PLACEHOLDER_ANCESTOR_STEPS {
        if z == 0 {
            break;
        }
        z -= 1;
        x /= 2;
        y /= 2;
        let Some(handle) = cache.ready.get(&tiles::TileKey { z, x, y }) else { continue };
        // `key` is one `1 / 2^steps`-sized cell of the ancestor's tile,
        // positioned by the low `steps` bits of its original x/y.
        let n = 1u32 << steps;
        let cell = tiles::TILE_SIZE / n as f32;
        let (cx, cy) = ((key.x % n) as f32, (key.y % n) as f32);
        let rect = Rect::new(cx * cell, cy * cell, (cx + 1.0) * cell, (cy + 1.0) * cell);
        return Some((handle.clone(), rect));
    }
    None
}

/// Spawn/reposition/despawn OSM tile images to match the current `MapView`.
/// Runs every frame — cheap at this scale (usually a couple dozen tiles
/// visible at once): on a zoom change every tile is dropped and refetched at
/// the new pixel grid; on a pure pan, tiles already spawned are just
/// repositioned and only the ones newly scrolled into view get requested.
pub fn sync_map_tiles(
    mut commands: Commands,
    mut cache: ResMut<tiles::TileCache>,
    mut spawned: ResMut<SpawnedTiles>,
    view: Res<MapView>,
    content_q: Query<Entity, With<MapContent>>,
) {
    let Ok(content) = content_q.single() else {
        return;
    };

    if spawned.zoom != Some(view.zoom) {
        for (_, tile) in spawned.entities.drain() {
            commands.entity(tile.entity).despawn();
        }
        spawned.zoom = Some(view.zoom);
    }

    let center_world = tiles::lonlat_to_world_px(view.center.0, view.center.1, view.zoom);
    let top_left_world = center_world - viewport_size() * 0.5;
    let bottom_right_world = center_world + viewport_size() * 0.5;

    // One tile of margin on every side so tiles are already loaded by the
    // time a pan scrolls them into view, not popping in at the edge.
    let x0 = (top_left_world.x / tiles::TILE_SIZE).floor() as i64 - 1;
    let x1 = (bottom_right_world.x / tiles::TILE_SIZE).floor() as i64 + 1;
    let y0 = (top_left_world.y / tiles::TILE_SIZE).floor() as i64 - 1;
    let y1 = (bottom_right_world.y / tiles::TILE_SIZE).floor() as i64 + 1;

    let mut wanted: HashSet<tiles::TileKey> = HashSet::new();
    for ty in y0..=y1 {
        if ty < 0 {
            continue;
        }
        for tx in x0..=x1 {
            if tx < 0 {
                continue;
            }
            let key = tiles::TileKey { z: view.zoom, x: tx as u32, y: ty as u32 };
            if !key.in_range() {
                continue;
            }
            wanted.insert(key);

            let tile_screen = Vec2::new(tx as f32, ty as f32) * tiles::TILE_SIZE - top_left_world;
            let tile_node = || Node {
                position_type: PositionType::Absolute,
                left: Val::Px(tile_screen.x),
                top: Val::Px(tile_screen.y),
                width: Val::Px(tiles::TILE_SIZE),
                height: Val::Px(tiles::TILE_SIZE),
                ..default()
            };
            let spawn_tile = |commands: &mut Commands, image: ImageNode, is_final: bool| {
                let entity = commands
                    .spawn((tile_node(), image, Pickable::IGNORE, MapTile))
                    .id();
                commands.entity(content).add_child(entity);
                SpawnedTile { entity, is_final }
            };

            match spawned.entities.get(&key) {
                Some(existing) if existing.is_final => {
                    // Already showing the real tile — just reposition (a
                    // pan may have moved it).
                    commands.entity(existing.entity).insert(tile_node());
                }
                Some(existing) => {
                    // Still a placeholder — reposition it, and swap for the
                    // real tile the instant it's ready.
                    commands.entity(existing.entity).insert(tile_node());
                    if let Some(handle) = cache.ready.get(&key).cloned() {
                        commands.entity(existing.entity).despawn();
                        let image = ImageNode::new(handle).with_mode(NodeImageMode::Stretch);
                        spawned.entities.insert(key, spawn_tile(&mut commands, image, true));
                    }
                }
                None => {
                    if let Some(handle) = cache.ready.get(&key).cloned() {
                        let image = ImageNode::new(handle).with_mode(NodeImageMode::Stretch);
                        spawned.entities.insert(key, spawn_tile(&mut commands, image, true));
                    } else {
                        cache.request(key);
                        if let Some((handle, rect)) = cached_ancestor(&cache, key) {
                            let image =
                                ImageNode::new(handle).with_mode(NodeImageMode::Stretch).with_rect(rect);
                            spawned.entities.insert(key, spawn_tile(&mut commands, image, false));
                        }
                    }
                }
            }
        }
    }

    spawned.entities.retain(|key, tile| {
        if wanted.contains(key) {
            true
        } else {
            commands.entity(tile.entity).despawn();
            false
        }
    });
}

/// Recompute the network-area preview whenever the point list changes.
pub fn recompute_area_on_change(points: Res<PendingPoints>, mut area: ResMut<NetworkArea>) {
    if !points.is_changed() {
        return;
    }
    *area = recompute_network_area(&points.0);
}

/// Rebuild the point dots, polygon edges, and bounding-square outline
/// whenever the point list, view (pan/zoom), or theme changes.
///
/// Every entity spawned here carries `ZIndex(1)` so it draws above `MapTile`
/// images regardless of spawn order — `sync_map_tiles` runs every frame and
/// keeps appending newly-loaded tiles as later `MapContent` children, so
/// relying on "points were spawned after the tiles" (true only the instant
/// this system last ran) would let a tile that finishes loading a few
/// frames later cover the points back up.
pub fn redraw_polygon(
    mut commands: Commands,
    points: Res<PendingPoints>,
    net: Res<NetworkArea>,
    base: Res<BasePosition>,
    theme: Res<Theme>,
    view: Res<MapView>,
    content_q: Query<Entity, With<MapContent>>,
    visuals: Query<Entity, With<PolygonVisual>>,
) {
    if !points.is_changed() && !theme.is_changed() && !base.is_changed() && !view.is_changed() {
        return;
    }

    for entity in &visuals {
        commands.entity(entity).despawn();
    }
    let Ok(content) = content_q.single() else {
        return;
    };

    let pal = theme.palette();
    let pixels: Vec<Vec2> =
        points.0.iter().map(|&(lat, lon)| lonlat_to_screen_px(lon, lat, &view)).collect();

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
                .map(|&(lat, lon)| lonlat_to_screen_px(lon, lat, &view))
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
                .map(|&(lat, lon)| lonlat_to_screen_px(lon, lat, &view))
                .collect();
            for i in 0..4 {
                spawn_edge(parent, corners_px[i], corners_px[(i + 1) % 4], square_color, SQUARE_THICKNESS);
            }
        }

        // Base marker.
        if let Some((lat, lon)) = base.0 {
            let p: Vec2 = lonlat_to_screen_px(lon, lat, &view);
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
                ZIndex(1),
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
                ZIndex(1),
            ));
        }

        // Point dots + index labels.
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
                ZIndex(1),
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
                ZIndex(1),
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
        bevy::ui::UiTransform::from_rotation(Rot2::radians(angle)),
        BackgroundColor(color),
        Pickable::IGNORE,
        PolygonVisual,
        ZIndex(1),
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
    // `net.center` is `(lon, lat)` — see the comment on `distance_to_square_km`.
    area.latitude = net.center.1;
    area.longitude = net.center.0;
    area.size_km = net.fetch_size_km.max(1.0);
    commands.remove_resource::<crate::terrain::TerrainLoadError>();
    next_state.set(AppState::LoadingTerrain);
}

pub fn cleanup(mut commands: Commands, roots: Query<Entity, With<AreaSelectionRoot>>) {
    for entity in &roots {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<SpawnedTiles>();
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

    /// `lonlat_to_screen_px`/`cursor_to_lonlat` must invert each other at any
    /// pan/zoom, or clicks stop landing where they visually appear to (the
    /// bug that made point-adding and base-placement feel broken once zoom
    /// started working). Tolerance is ~11 m (1e-4°) at Sweden's latitude —
    /// not exact, since the round trip goes through `Vec2` (f32) pixel
    /// coordinates like the real UI does, but well inside what matters for
    /// clicking a point on a map (`MAX_BASE_DISTANCE_KM` is 3 km).
    #[test]
    fn screen_lonlat_roundtrip_is_close_at_any_zoom() {
        let cases = [(17.65, 62.15, 5u8), (18.0686, 59.3293, 12), (11.0, 68.0, 9)];
        for (lon, lat, zoom) in cases {
            let view = MapView { zoom, center: (lon + 0.4, lat - 0.3) };
            let screen = lonlat_to_screen_px(lon, lat, &view);
            let normalized = screen / viewport_size() - Vec2::splat(0.5);
            let (back_lon, back_lat) = cursor_to_lonlat(normalized, &view);
            assert!((back_lon - lon).abs() < 1e-4, "lon {back_lon} vs {lon} at zoom {zoom}");
            assert!((back_lat - lat).abs() < 1e-4, "lat {back_lat} vs {lat} at zoom {zoom}");
        }
    }

    /// Zooming in/out around an anchor point must leave the lon/lat under
    /// that anchor unchanged — the actual fix for "zoom doesn't follow the
    /// cursor" / "goes through the map".
    #[test]
    fn rezoom_keeps_the_anchor_point_fixed() {
        let mut view = MapView { zoom: 6, center: (17.65, 62.15) };
        let anchor = Vec2::new(100.0, 300.0);
        let (lon_before, lat_before) =
            cursor_to_lonlat(anchor / viewport_size() - Vec2::splat(0.5), &view);

        rezoom_around(&mut view, anchor, 10);

        let (lon_after, lat_after) =
            cursor_to_lonlat(anchor / viewport_size() - Vec2::splat(0.5), &view);
        assert!((lon_after - lon_before).abs() < 1e-4);
        assert!((lat_after - lat_before).abs() < 1e-4);
    }

    /// End-to-end simulation of `place_base_on_click`'s actual condition:
    /// pick 3 points, compute the network area exactly like
    /// `recompute_area_on_change` does, then click the square's own center
    /// at a deeply-zoomed-in view (placing the base usually happens zoomed
    /// in for precision, per `point_table_and_buttons`'s comment) — the
    /// resulting `distance_to_square_km` must clear
    /// `MAX_BASE_DISTANCE_KM`, or `place_base_on_click` would reject a click
    /// that's visually dead-center on the area.
    #[test]
    fn clicking_the_network_areas_own_center_places_the_base() {
        let points = [(59.0, 18.0), (59.05, 18.05), (58.98, 18.06)];
        let net = recompute_network_area(&points);
        assert!(net.valid, "test setup: 3 points should form a valid area");

        // Ground truth independent of `distance_to_square_km`'s own
        // (lon, lat)-vs-(lat, lon) convention: `center` must land near the
        // picked points' own coordinates, not off by a lat/lon swap. This
        // is what would have caught the real bug — that test previously
        // destructured `net.center` the same wrong way `distance_to_square_km`
        // did internally, so both sides shared the same mistake and it
        // passed anyway.
        let (center_lon, center_lat) = net.center;
        assert!((center_lon - 18.03).abs() < 0.5, "center_lon {center_lon} looks swapped with lat");
        assert!((center_lat - 59.0).abs() < 0.5, "center_lat {center_lat} looks swapped with lon");

        for zoom in [5u8, 10, 15, 19] {
            // Zoomed in tight on the area's own center, same as a user
            // would do to place the base precisely.
            let view = MapView { zoom, center: (center_lon, center_lat) };
            let (lon, lat) = cursor_to_lonlat(Vec2::ZERO, &view); // dead center of the viewport
            let distance = net.distance_to_square_km(lat, lon);
            assert!(
                distance <= MAX_BASE_DISTANCE_KM,
                "clicking the area's own center at zoom {zoom} landed {distance:.4} km away — should be ~0"
            );
        }
    }
}
