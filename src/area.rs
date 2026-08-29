use std::collections::{HashMap, HashSet};

use bevy::{
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    prelude::*,
    ui::RelativeCursorPosition,
    ui_widgets::{Slider, SliderPrecision, SliderRange, SliderStep, SliderValue, TrackClick, ValueChange},
};

use crate::{
    polygon, sweden_geo,
    terrain::{DENSITY_STEP, MAX_DENSITY, MIN_DENSITY, VegetationSettings},
    theme::{Palette, Slot, Theme, UiFill, UiInk, UiStroke},
    tiles, AppState,
};

/// Kept for existing callers (terrain fetch math) — the network-area picker
/// now drives `ScenarioArea.size_km` dynamically instead.
pub const AREA_SIZE_KM: f32 = 20.0;

/// Fallback viewport size, used only for the frame or two before Bevy's
/// layout has measured the real one. The map fills whatever space the window
/// leaves beside the instrument rail, so its true size lives in
/// [`MapView::size`] and is refreshed every frame by `track_viewport_size`.
const FALLBACK_VIEWPORT: Vec2 = Vec2::new(900.0, 800.0);

/// Width of the instrument rail down the right-hand side.
const RAIL_W: f32 = 392.0;
/// Height of the title strip across the top.
const TOP_RAIL_H: f32 = 44.0;
/// Every rule and control border in this screen is exactly one pixel. Nothing
/// on it is drawn with a fill where a line will do.
const HAIRLINE: f32 = 1.0;
/// Horizontal padding inside the rail and the title strip.
const RAIL_PAD: f32 = 18.0;

const MIN_POINTS: usize = 3;
/// Largest terrain square we will download for a temporary, self-healing
/// drone mesh. This limits the actual axis-aligned terrain request, not just
/// the selected rotated square, so a diagonal selection cannot evade it.
const MAX_SIDE_KM: f64 = 20.0;
const POINT_DOT_SIZE: f32 = 8.0;
const EDGE_THICKNESS: f32 = 1.0;
const SQUARE_THICKNESS: f32 = 2.0;

/// Sweden's rough centroid — the default view on first opening the picker.
const DEFAULT_CENTER_LON: f64 = 17.65;
const DEFAULT_CENTER_LAT: f64 = 62.15;
/// Shows roughly the whole country at a typical viewport size.
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
    #[cfg_attr(not(test), allow(dead_code))]
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
    /// The mission boundary the drones actually fly to: the convex hull of the
    /// points the operator drew, `(lon, lat)`, counter-clockwise.
    ///
    /// Not `corners`. That square exists to size the terrain fetch and the
    /// airframe count, and it is always bigger than what was asked for — a
    /// rotated selection can enclose a lot of ground nobody picked. The hull
    /// is the shape on screen, so it is the shape the mesh covers.
    ///
    /// Hull rather than the raw click order because `navigation`'s containment
    /// test is a convex one; a concave outline would otherwise report points
    /// inside it as out.
    pub hull: Vec<(f64, f64)>,
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
    let hull: Vec<(f64, f64)> = polygon::convex_hull(&locals)
        .into_iter()
        .map(|lp| polygon::unproject(ref_lon, ref_lat, lp))
        .collect();

    // `unproject` yields `(lon, lat)`, and so do `center` and `corners`.
    // Reading them as `(lat, lon)` here used to measure the north-south half
    // span from a longitude delta and vice versa, which at Swedish latitudes
    // inflated `fetch_size_km` by about 2.2x — every mission fetched twice
    // the terrain it needed and tripped `over_limit` at well under the real
    // side limit. Same class of bug as the one `distance_to_square_km`
    // documents; the convention is `(lon, lat)` everywhere this struct is
    // read.
    let (clon, clat) = center;
    let mut half_ns_km = 0.0_f64;
    let mut half_ew_km = 0.0_f64;
    for &(lon, lat) in &corners {
        half_ns_km = half_ns_km.max((lat - clat).abs() * 110.574);
        half_ew_km = half_ew_km.max((lon - clon).abs() * 111.320 * clat.to_radians().cos());
    }
    let fetch_size_km = (half_ns_km.max(half_ew_km) * 2.0) as f32;
    let side_km = square.side_km();

    let fetch_half_km = fetch_size_km as f64 * 0.5;
    let lat_delta = fetch_half_km / 110.574;
    let lon_delta = fetch_half_km / (111.320 * clat.to_radians().cos());
    let fetch_corners = [
        (clon + lon_delta, clat + lat_delta),
        (clon - lon_delta, clat + lat_delta),
        (clon - lon_delta, clat - lat_delta),
        (clon + lon_delta, clat - lat_delta),
    ];

    NetworkArea {
        points: points.to_vec(),
        corners,
        hull,
        center,
        side_km,
        rotation_deg: square.rotation.to_degrees(),
        fetch_size_km,
        fetch_corners,
        valid: true,
        // `fetch_size_km` is the axis-aligned square sent to the terrain
        // service. A 20 km square rotated by 45° needs a ~28 km fetch, so
        // checking `side_km` alone would still let an oversized pull through.
        over_limit: fetch_size_km as f64 > MAX_SIDE_KM,
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
    /// Logical (not physical) pixel size of the viewport node. Every
    /// projection on this screen is relative to the viewport's own box, so
    /// this is the one number that lets the map fill the window instead of
    /// being pinned to a hardcoded rectangle.
    pub size: Vec2,
}

impl Default for MapView {
    fn default() -> Self {
        Self {
            zoom: DEFAULT_ZOOM,
            center: (DEFAULT_CENTER_LON, DEFAULT_CENTER_LAT),
            size: FALLBACK_VIEWPORT,
        }
    }
}

impl MapView {
    /// Back to the default view. Keeps `size` — that is a property of the
    /// window, not of what the operator is looking at.
    fn reset(&mut self) {
        let size = self.size;
        *self = Self { size, ..Self::default() };
    }
}

/// Keep [`MapView::size`] in step with the laid-out viewport node.
///
/// `ComputedNode::size` is in physical pixels while `Node` positions are
/// logical, so this scales back down — otherwise every overlay would be
/// placed at double coordinates on a retina display.
pub fn track_viewport_size(
    mut view: ResMut<MapView>,
    viewport_q: Query<&ComputedNode, With<MapViewport>>,
) {
    let Ok(node) = viewport_q.single() else {
        return;
    };
    let size = node.size() * node.inverse_scale_factor();
    if size.x < 1.0 || size.y < 1.0 {
        return;
    }
    // Guarded so a stable window doesn't mark `MapView` changed every frame —
    // tile sync and the overlay redraw both key off that flag.
    if size.distance_squared(view.size) > 0.01 {
        view.size = size;
    }
}

#[derive(Component)]
pub(crate) struct AreaSelectionRoot;

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

/// Every piece of text on the rail that `update_status_text` writes to.
///
/// One component naming a field, rather than one marker component per field:
/// a `Query<&mut Text>` per marker would be a dozen mutable queries over the
/// same component, and Bevy cannot prove those disjoint without every pair
/// carrying a `Without` of the other. This collapses all of them into a
/// single query and a `match`.
#[derive(Component, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RailField {
    Vertices,
    Side,
    Rotation,
    Fetch,
    Airframes,
    Station,
    /// The state chip in the title strip — the one thing on screen that says
    /// whether the mission can be launched.
    StatusChip,
    Warning,
    AddStopLabel,
    SetBaseLabel,
    GenerateLabel,
}

/// The chip's border, which is recolored alongside its text.
#[derive(Component)]
pub(crate) struct StatusChip;

/// Live centre/zoom readout under the map. Kept separate from [`RailField`]
/// because it is driven by `MapView`, which changes on pan frames when no
/// mission state does.
#[derive(Component)]
pub(crate) struct MapCoordReadout;

#[derive(Component)]
pub(crate) struct GenerateTerrain;

#[derive(Component)]
pub(crate) struct AddStopButton;

#[derive(Component)]
pub(crate) struct ClearButton;

/// Present only so `spawn_button` always has a label marker to attach; the
/// Clear button's caption never changes.
#[derive(Component)]
pub(crate) struct ClearLabel;

#[derive(Component)]
pub(crate) struct PointsTableRoot;

#[derive(Component)]
pub(crate) struct RemovePointButton(usize);

#[derive(Component)]
pub(crate) struct SetBaseButton;

/// The KOLBE wordmark in the title strip, which doubles as "show me
/// everything" — the way a map app's own logo returns you to the whole map.
#[derive(Component)]
pub(crate) struct ResetViewButton;

#[derive(Component)]
pub(crate) struct ZoomInButton;

#[derive(Component)]
pub(crate) struct ZoomOutButton;

/// Button that turns the procedural forest on and off.
#[derive(Component)]
pub(crate) struct TreesToggle;

#[derive(Component)]
pub(crate) struct TreesToggleLabel;

#[derive(Component)]
pub(crate) struct DensityLabel;

/// Filled portion of the density slider track.
#[derive(Component)]
pub(crate) struct DensityFill;

// ─── Type ──────────────────────────────────────────────────────────────────

/// Widen a short label by inserting thin spaces between its characters.
///
/// `TextFont` has no letter-spacing, and all-caps at 10 px without any reads
/// as a solid bar rather than words. Only for section headers and the state
/// chip — it is a typographic workaround, so nothing should ever compare or
/// parse the string it returns.
fn tracked(label: &str) -> String {
    label.chars().map(String::from).collect::<Vec<_>>().join("\u{2009}")
}

/// Fira Mono at `size`, for anything numeric. See `crate::MONO_UI_FONT`.
fn mono(fonts: &crate::UiFonts, size: f32) -> TextFont {
    TextFont {
        font: fonts.mono.clone().into(),
        font_size: FontSize::Px(size),
        ..default()
    }
}

/// The default UI face at `size`, for prose and labels.
fn sans(size: f32) -> TextFont {
    TextFont { font_size: FontSize::Px(size), ..default() }
}

// ─── Chrome builders ───────────────────────────────────────────────────────

/// A section header: an amber tick, a tracked caps label, and a rule running
/// out to the edge of the rail. The rule is what separates sections — there
/// are no boxes around them, because a box costs two lines where one will do.
fn spawn_section(parent: &mut ChildSpawnerCommands, label: &str, p: &Palette) {
    parent
        .spawn(Node {
            flex_direction: FlexDirection::Row,
            align_items: AlignItems::Center,
            column_gap: Val::Px(8.0),
            margin: UiRect::top(Val::Px(6.0)),
            ..default()
        })
        .with_children(|row| {
            row.spawn((
                Node { width: Val::Px(2.0), height: Val::Px(9.0), ..default() },
                BackgroundColor(p.signal),
                UiFill::new(Slot::Signal),
            ));
            row.spawn((
                Text::new(tracked(label)),
                sans(10.0),
                TextColor(p.subtext),
                UiInk::new(Slot::Subtext),
            ));
            row.spawn((
                Node { flex_grow: 1.0, height: Val::Px(HAIRLINE), ..default() },
                BackgroundColor(p.line),
                UiFill::new(Slot::Line),
            ));
        });
}

/// One `LABEL ................ VALUE` line inside a readout block. The label
/// is sans and dim, the value is mono and bright, so a column of these scans
/// as a column of numbers rather than a paragraph.
fn spawn_readout(
    parent: &mut ChildSpawnerCommands,
    label: &str,
    initial: &str,
    field: RailField,
    fonts: &crate::UiFonts,
    p: &Palette,
) {
    parent
        .spawn(Node {
            flex_direction: FlexDirection::Row,
            justify_content: JustifyContent::SpaceBetween,
            align_items: AlignItems::Center,
            ..default()
        })
        .with_children(|row| {
            row.spawn((
                Text::new(label),
                sans(10.0),
                TextColor(p.subtext),
                UiInk::new(Slot::Subtext),
            ));
            row.spawn((
                Text::new(initial),
                mono(fonts, 12.0),
                TextColor(p.text),
                UiInk::new(Slot::Text),
                field,
            ));
        });
}

/// Which of the two button treatments this screen has. There are only two on
/// purpose: exactly one action per screen is the one you came here to take.
enum ButtonKind {
    /// Outline only. Everything that changes what you are editing.
    Ghost,
    /// Amber fill. The single action that leaves this screen.
    Primary,
}

/// A square-cornered, tracked-caps button. `label_marker` rides on the text
/// entity so callers can retitle it later without touching the button itself.
fn spawn_button(
    parent: &mut ChildSpawnerCommands,
    label: &str,
    kind: ButtonKind,
    marker: impl Component,
    label_marker: impl Component,
    fill: bool,
    p: &Palette,
) {
    let (bg, fg, border) = match kind {
        ButtonKind::Ghost => (Color::NONE, p.text, p.line),
        ButtonKind::Primary => (p.signal, p.bg, p.signal),
    };
    let mut button = parent.spawn((
        Button,
        Node {
            flex_grow: if fill { 1.0 } else { 0.0 },
            height: Val::Px(34.0),
            padding: UiRect::horizontal(Val::Px(14.0)),
            justify_content: JustifyContent::Center,
            align_items: AlignItems::Center,
            border: UiRect::all(Val::Px(HAIRLINE)),
            ..default()
        },
        BackgroundColor(bg),
        BorderColor::all(border),
        marker,
    ));
    if matches!(kind, ButtonKind::Ghost) {
        // The primary button's colors are driven by readiness every frame,
        // so only the ghost treatment can be left to the theme system.
        button.insert((UiFill(Slot::Line, 0.0), UiStroke::new(Slot::Line)));
    }
    button.with_child((
        Text::new(tracked(label)),
        sans(11.0),
        TextColor(fg),
        label_marker,
        Pickable::IGNORE,
    ));
}

/// A single corner bracket on the map — two hairlines meeting at `corner`,
/// inset from the edge. Four of these frame the viewport without drawing a
/// full box around it, which would fight the map's own linework.
fn spawn_bracket(parent: &mut ChildSpawnerCommands, right: bool, bottom: bool, color: Color) {
    const INSET: f32 = 12.0;
    const ARM: f32 = 20.0;

    let edge = |horizontal: bool| {
        let (w, h) = if horizontal { (ARM, HAIRLINE) } else { (HAIRLINE, ARM) };
        let mut node = Node {
            position_type: PositionType::Absolute,
            width: Val::Px(w),
            height: Val::Px(h),
            ..default()
        };
        if right {
            node.right = Val::Px(INSET);
        } else {
            node.left = Val::Px(INSET);
        }
        if bottom {
            node.bottom = Val::Px(INSET);
        } else {
            node.top = Val::Px(INSET);
        }
        node
    };

    for horizontal in [true, false] {
        parent.spawn((edge(horizontal), BackgroundColor(color), Pickable::IGNORE));
    }
}

fn spawn_zoom_button(
    parent: &mut ChildSpawnerCommands,
    label: &str,
    marker: impl Component,
    fonts: &crate::UiFonts,
    p: &Palette,
) {
    parent
        .spawn((
            Button,
            Node {
                width: Val::Px(26.0),
                height: Val::Px(26.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(HAIRLINE)),
                ..default()
            },
            BackgroundColor(p.panel.with_alpha(0.92)),
            BorderColor::all(p.line),
            UiFill(Slot::Panel, 0.92),
            UiStroke::new(Slot::Line),
            marker,
        ))
        .with_child((
            Text::new(label),
            mono(fonts, 14.0),
            TextColor(p.text),
            UiInk::new(Slot::Text),
            Pickable::IGNORE,
        ));
}

pub fn setup(
    mut commands: Commands,
    vegetation: Res<VegetationSettings>,
    load_error: Option<Res<crate::terrain::TerrainLoadError>>,
    theme: Res<Theme>,
    fonts: Res<crate::UiFonts>,
) {
    commands.insert_resource(PendingPoints::default());
    commands.insert_resource(PickMode::default());
    commands.insert_resource(recompute_network_area(&[]));
    commands.insert_resource(MapView::default());
    commands.insert_resource(BasePosition::default());
    commands.insert_resource(SpawnedTiles::default());

    let p = theme.palette();
    let fonts = fonts.clone();

    commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                ..default()
            },
            BackgroundColor(p.bg),
            UiFill::new(Slot::Bg),
            AreaSelectionRoot,
        ))
        .with_children(|root| {
            spawn_title_strip(root, &p);

            root.spawn(Node {
                flex_grow: 1.0,
                width: Val::Percent(100.0),
                flex_direction: FlexDirection::Row,
                min_height: Val::Px(0.0),
                ..default()
            })
            .with_children(|body| {
                spawn_map(body, &fonts, &p);
                spawn_rail(body, &vegetation, load_error.as_deref(), &fonts, &p);
            });
        });
}

/// Title strip: who we are, what this screen is for, and the one-word state.
/// Right padding leaves the day/night toggle its corner.
fn spawn_title_strip(root: &mut ChildSpawnerCommands, p: &Palette) {
    root.spawn((
        Node {
            width: Val::Percent(100.0),
            height: Val::Px(TOP_RAIL_H),
            flex_direction: FlexDirection::Row,
            align_items: AlignItems::Center,
            column_gap: Val::Px(12.0),
            padding: UiRect::new(Val::Px(RAIL_PAD), Val::Px(58.0), Val::ZERO, Val::ZERO),
            border: UiRect::bottom(Val::Px(HAIRLINE)),
            flex_shrink: 0.0,
            ..default()
        },
        BackgroundColor(p.panel),
        BorderColor::all(p.line),
        UiFill::new(Slot::Panel),
        UiStroke::new(Slot::Line),
    ))
    .with_children(|strip| {
        strip
            .spawn((
                Button,
                Node {
                    padding: UiRect::axes(Val::Px(4.0), Val::Px(3.0)),
                    margin: UiRect::left(Val::Px(-4.0)),
                    ..default()
                },
                BackgroundColor(Color::NONE),
                ResetViewButton,
            ))
            .with_child((
                Text::new(tracked("KOLBE")),
                sans(13.0),
                TextColor(p.signal),
                UiInk::new(Slot::Signal),
                Pickable::IGNORE,
            ));
        strip.spawn((
            Node { width: Val::Px(HAIRLINE), height: Val::Px(16.0), ..default() },
            BackgroundColor(p.line),
            UiFill::new(Slot::Line),
        ));
        strip.spawn((
            Text::new(tracked("MESH AREA DEFINITION")),
            sans(10.0),
            TextColor(p.subtext),
            UiInk::new(Slot::Subtext),
        ));
        strip.spawn(Node { flex_grow: 1.0, ..default() });

        // Colors here are rewritten every frame from mission state, so this
        // one carries no slot tags — see `update_status_text`.
        strip
            .spawn((
                Node {
                    padding: UiRect::axes(Val::Px(9.0), Val::Px(4.0)),
                    border: UiRect::all(Val::Px(HAIRLINE)),
                    ..default()
                },
                BorderColor::all(p.line),
                StatusChip,
            ))
            .with_child((
                Text::new(tracked("NO AREA")),
                sans(9.0),
                TextColor(p.subtext),
                RailField::StatusChip,
            ));
    });
}

/// The map: everything left of the rail. Fills the window, so its pixel size
/// is whatever is left over — see `track_viewport_size`.
fn spawn_map(body: &mut ChildSpawnerCommands, fonts: &crate::UiFonts, p: &Palette) {
    body.spawn((
        Node {
            flex_grow: 1.0,
            height: Val::Percent(100.0),
            min_width: Val::Px(0.0),
            overflow: Overflow::clip(),
            position_type: PositionType::Relative,
            ..default()
        },
        BackgroundColor(p.bg),
        UiFill::new(Slot::Bg),
        Interaction::None,
        RelativeCursorPosition::default(),
        MapViewport,
    ))
    .with_children(|viewport| {
        // Tiles plus the point/polygon/base overlay, all positioned directly
        // in screen space from the current `MapView` (see
        // `lonlat_to_screen_px`) and rebuilt on pan/zoom/point changes rather
        // than carrying a scale/translate transform of their own.
        viewport.spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                ..default()
            },
            MapContent,
            Pickable::IGNORE,
        ));

        for (right, bottom) in [(false, false), (true, false), (false, true), (true, true)] {
            spawn_bracket(viewport, right, bottom, p.line);
        }

        // Centre reticle. Not decoration: the +/- buttons zoom about the
        // viewport centre, so this marks where that zoom is anchored.
        viewport
            .spawn((
                Node {
                    position_type: PositionType::Absolute,
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                Pickable::IGNORE,
            ))
            .with_children(|centre| {
                centre.spawn((
                    Node {
                        width: Val::Px(26.0),
                        height: Val::Px(26.0),
                        border: UiRect::all(Val::Px(HAIRLINE)),
                        ..default()
                    },
                    BorderColor::all(p.line.with_alpha(0.75)),
                    UiStroke(Slot::Line, 0.75),
                    Pickable::IGNORE,
                ));
            });

        // Zoom controls. Siblings of `MapContent`, not children, so they hold
        // their corner instead of panning with the map.
        viewport
            .spawn((
                Node {
                    position_type: PositionType::Absolute,
                    right: Val::Px(12.0),
                    top: Val::Px(12.0),
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(4.0),
                    ..default()
                },
                Pickable::IGNORE,
            ))
            .with_children(|controls| {
                spawn_zoom_button(controls, "+", ZoomInButton, fonts, p);
                spawn_zoom_button(controls, "\u{2212}", ZoomOutButton, fonts, p);
            });

        // Footer strip: live centre/zoom on the left, and the attribution
        // OpenStreetMap's tile usage policy requires wherever tiles are shown
        // (https://operations.osmfoundation.org/policies/tiles/) on the right.
        viewport
            .spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::ZERO,
                    right: Val::ZERO,
                    bottom: Val::ZERO,
                    height: Val::Px(22.0),
                    flex_direction: FlexDirection::Row,
                    align_items: AlignItems::Center,
                    justify_content: JustifyContent::SpaceBetween,
                    padding: UiRect::horizontal(Val::Px(10.0)),
                    border: UiRect::top(Val::Px(HAIRLINE)),
                    ..default()
                },
                BackgroundColor(p.panel.with_alpha(0.9)),
                BorderColor::all(p.line),
                UiFill(Slot::Panel, 0.9),
                UiStroke::new(Slot::Line),
                Pickable::IGNORE,
            ))
            .with_children(|footer| {
                // Cyan: this is the system reporting where it is looking,
                // not anything the operator placed.
                footer.spawn((
                    Text::new(""),
                    mono(fonts, 10.0),
                    TextColor(p.accent),
                    UiInk::new(Slot::Accent),
                    MapCoordReadout,
                ));
                footer.spawn((
                    Text::new("\u{00A9} OpenStreetMap contributors"),
                    sans(9.0),
                    TextColor(p.subtext),
                    UiInk::new(Slot::Subtext),
                ));
            });
    });
}

/// The instrument rail: a scrolling body of sections, and a footer pinned to
/// the bottom holding the warning line and the one action that leaves here.
fn spawn_rail(
    body: &mut ChildSpawnerCommands,
    vegetation: &VegetationSettings,
    load_error: Option<&crate::terrain::TerrainLoadError>,
    fonts: &crate::UiFonts,
    p: &Palette,
) {
    body.spawn((
        Node {
            width: Val::Px(RAIL_W),
            flex_shrink: 0.0,
            height: Val::Percent(100.0),
            flex_direction: FlexDirection::Column,
            border: UiRect::left(Val::Px(HAIRLINE)),
            ..default()
        },
        BackgroundColor(p.panel),
        BorderColor::all(p.line),
        UiFill::new(Slot::Panel),
        UiStroke::new(Slot::Line),
    ))
    .with_children(|rail| {
        rail.spawn(Node {
            flex_grow: 1.0,
            min_height: Val::Px(0.0),
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(10.0),
            padding: UiRect::all(Val::Px(RAIL_PAD)),
            overflow: Overflow::clip_y(),
            ..default()
        })
        .with_children(|col| {
            spawn_section(col, "AREA DEFINITION", p);
            col.spawn((
                Text::new(
                    "Click the map to mark the outline of the area to cover. \
                     Three points or more; the smallest enclosing square becomes \
                     the mesh area.",
                ),
                sans(11.0),
                TextColor(p.subtext),
                UiInk::new(Slot::Subtext),
            ));

            // Derived mission figures, boxed together because they are read
            // as a set: change one vertex and all five move at once.
            col.spawn((
                Node {
                    flex_direction: FlexDirection::Column,
                    row_gap: Val::Px(5.0),
                    padding: UiRect::axes(Val::Px(12.0), Val::Px(10.0)),
                    border: UiRect::all(Val::Px(HAIRLINE)),
                    ..default()
                },
                BackgroundColor(p.raised),
                BorderColor::all(p.line),
                UiFill::new(Slot::Raised),
                UiStroke::new(Slot::Line),
            ))
            .with_children(|box_| {
                spawn_readout(box_, "VERTICES", "0", RailField::Vertices, fonts, p);
                spawn_readout(box_, "SIDE", "—", RailField::Side, fonts, p);
                spawn_readout(box_, "ROTATION", "—", RailField::Rotation, fonts, p);
                spawn_readout(box_, "FETCH SPAN", "—", RailField::Fetch, fonts, p);
                spawn_readout(box_, "AIRFRAMES", "—", RailField::Airframes, fonts, p);
            });

            col.spawn(Node {
                flex_direction: FlexDirection::Row,
                column_gap: Val::Px(8.0),
                ..default()
            })
            .with_children(|row| {
                spawn_button(row, "STOP ADDING", ButtonKind::Ghost, AddStopButton, RailField::AddStopLabel, true, p);
                spawn_button(row, "CLEAR", ButtonKind::Ghost, ClearButton, ClearLabel, false, p);
            });

            spawn_section(col, "VERTEX LIST", p);
            col.spawn(Node {
                display: Display::Grid,
                grid_template_columns: vec![
                    RepeatedGridTrack::px(1, 22.0),
                    RepeatedGridTrack::flex(2, 1.0),
                    RepeatedGridTrack::px(1, 20.0),
                ],
                column_gap: Val::Px(10.0),
                row_gap: Val::Px(4.0),
                ..default()
            })
            .with_children(|grid| {
                for h in ["#", "LATITUDE", "LONGITUDE", ""] {
                    grid.spawn((
                        Text::new(h),
                        sans(9.0),
                        TextColor(p.subtext),
                        UiInk::new(Slot::Subtext),
                    ));
                }
            })
            .insert(PointsTableRoot);

            spawn_section(col, "GROUND STATION", p);
            spawn_readout(col, "POSITION", "NOT SET", RailField::Station, fonts, p);
            spawn_button(col, "SET LOCATION", ButtonKind::Ghost, SetBaseButton, RailField::SetBaseLabel, true, p);

            spawn_section(col, "TERRAIN", p);
            spawn_vegetation_controls(col, vegetation, fonts, p);
            col.spawn((
                Text::new("Elevation source: Lantmateriet DTM"),
                sans(10.0),
                TextColor(p.subtext),
                UiInk::new(Slot::Subtext),
            ));
            if let Some(error) = load_error {
                col.spawn((
                    Text::new(format!("Last attempt failed: {}", error.0)),
                    sans(10.0),
                    TextColor(p.danger),
                    UiInk::new(Slot::Danger),
                ));
            }
        });

        // Footer. Pinned below the scrolling body so the launch action never
        // scrolls out of reach, however many vertices are in the list.
        rail.spawn((
            Node {
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(8.0),
                padding: UiRect::all(Val::Px(RAIL_PAD)),
                border: UiRect::top(Val::Px(HAIRLINE)),
                flex_shrink: 0.0,
                ..default()
            },
            BorderColor::all(p.line),
            UiStroke::new(Slot::Line),
        ))
        .with_children(|footer| {
            footer.spawn((
                Text::new(""),
                sans(10.0),
                TextColor(p.danger),
                RailField::Warning,
            ));
            spawn_button(
                footer,
                "GENERATE TERRAIN",
                ButtonKind::Primary,
                GenerateTerrain,
                RailField::GenerateLabel,
                true,
                p,
            );
        });
    });
}

/// Trees toggle plus the density slider it controls.
fn spawn_vegetation_controls(
    panel: &mut ChildSpawnerCommands,
    vegetation: &VegetationSettings,
    fonts: &crate::UiFonts,
    p: &Palette,
) {
    panel
        .spawn(Node {
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(8.0),
            ..default()
        })
        .with_children(|group| {
            group
                .spawn((
                    Button,
                    Node {
                        width: Val::Percent(100.0),
                        height: Val::Px(30.0),
                        justify_content: JustifyContent::SpaceBetween,
                        align_items: AlignItems::Center,
                        padding: UiRect::horizontal(Val::Px(10.0)),
                        border: UiRect::all(Val::Px(HAIRLINE)),
                        ..default()
                    },
                    BackgroundColor(Color::NONE),
                    BorderColor::all(p.line),
                    UiStroke::new(Slot::Line),
                    TreesToggle,
                ))
                .with_child((
                    Text::new(trees_text(vegetation)),
                    sans(11.0),
                    TextColor(toggle_ink(vegetation.enabled, p)),
                    TreesToggleLabel,
                    Pickable::IGNORE,
                ));

            group
                .spawn(Node {
                    flex_direction: FlexDirection::Row,
                    justify_content: JustifyContent::SpaceBetween,
                    align_items: AlignItems::Center,
                    ..default()
                })
                .with_children(|row| {
                    row.spawn((
                        Text::new("CANOPY DENSITY"),
                        sans(10.0),
                        TextColor(p.subtext),
                        UiInk::new(Slot::Subtext),
                    ));
                    row.spawn((
                        Text::new(density_text(vegetation)),
                        mono(fonts, 12.0),
                        TextColor(p.text),
                        UiInk::new(Slot::Text),
                        DensityLabel,
                    ));
                });

            // Headless slider: the widget reports a new value, we own the state.
            // With no `SliderThumb` in the subtree the usable travel is the full
            // track width, which is exactly what the percentage fill draws.
            group
                .spawn((
                    Node {
                        width: Val::Percent(100.0),
                        height: Val::Px(8.0),
                        border: UiRect::all(Val::Px(HAIRLINE)),
                        ..default()
                    },
                    BackgroundColor(Color::NONE),
                    BorderColor::all(p.line),
                    UiStroke::new(Slot::Line),
                    Slider { track_click: TrackClick::Snap, ..default() },
                    SliderValue(vegetation.density),
                    SliderRange::new(MIN_DENSITY, MAX_DENSITY),
                    SliderStep(DENSITY_STEP),
                    SliderPrecision(2),
                ))
                .observe(
                    |change: On<ValueChange<f32>>,
                     mut commands: Commands,
                     mut vegetation: ResMut<VegetationSettings>| {
                        let value = change.value.clamp(MIN_DENSITY, MAX_DENSITY);
                        commands.entity(change.source).insert(SliderValue(value));
                        vegetation.density = value;
                    },
                )
                .with_child((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::ZERO,
                        top: Val::ZERO,
                        width: Val::Percent(density_fraction(vegetation.density) * 100.0),
                        height: Val::Percent(100.0),
                        ..default()
                    },
                    BackgroundColor(p.signal),
                    // The fill covers the whole track, so it has to be
                    // invisible to the pointer or it eats every click.
                    Pickable::IGNORE,
                    DensityFill,
                ));
        });
}

/// Turns the trees toggle on/off in response to a button press. Split out of
/// (what was, on `main`) a combined `interactions` system, since this branch's
/// picker flow already owns click handling for everything else on this screen.
pub fn trees_toggle_interactions(
    trees_toggle: Query<&Interaction, (Changed<Interaction>, With<TreesToggle>)>,
    mut vegetation: ResMut<VegetationSettings>,
) {
    if trees_toggle.iter().any(|interaction| *interaction == Interaction::Pressed) {
        vegetation.enabled = !vegetation.enabled;
    }
}

/// Project lon/lat to a screen position (px, relative to the viewport's own
/// top-left — the same frame `RelativeCursorPosition::normalized` uses) at
/// the current pan/zoom. The inverse of `cursor_to_lonlat`.
fn lonlat_to_screen_px(lon: f64, lat: f64, view: &MapView) -> Vec2 {
    let world = tiles::lonlat_to_world_px(lon, lat, view.zoom);
    let center_world = tiles::lonlat_to_world_px(view.center.0, view.center.1, view.zoom);
    world - center_world + view.size * 0.5
}

/// Turn a click's viewport-relative `normalized` position into lon/lat at
/// the current pan/zoom — the inverse of `lonlat_to_screen_px`.
fn cursor_to_lonlat(normalized: Vec2, view: &MapView) -> (f64, f64) {
    let screen = (normalized + Vec2::splat(0.5)) * view.size;
    let center_world = tiles::lonlat_to_world_px(view.center.0, view.center.1, view.zoom);
    let world = center_world + screen - view.size * 0.5;
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
    reset_view: Query<&Interaction, (Changed<Interaction>, With<ResetViewButton>)>,
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

    // Clear discards the *selection*, not the view. Zooming back out to the
    // whole country on every clear meant that fixing one badly placed vertex
    // cost the operator their framing as well, and they had to fly back in to
    // where they were already working.
    if clear.iter().any(|i| *i == Interaction::Pressed) {
        points.0.clear();
        *mode = PickMode::Adding;
        base.0 = None;
    }

    // Resetting the view is its own action, on the wordmark.
    if reset_view.iter().any(|i| *i == Interaction::Pressed) {
        view.reset();
    }
}

/// Adjust `view` so the lon/lat currently under `anchor_px` (screen space,
/// relative to the viewport's own top-left) stays visually fixed while zoom
/// changes to `new_zoom`. This is what makes zoom feel anchored to the
/// cursor (or the viewport's center, for the +/- buttons) instead of always
/// re-centering on whatever `view.center` already was.
fn rezoom_around(view: &mut MapView, anchor_px: Vec2, new_zoom: u8) {
    let (lon, lat) = cursor_to_lonlat(anchor_px / view.size - Vec2::splat(0.5), view);
    view.zoom = new_zoom;
    let anchor_world = tiles::lonlat_to_world_px(lon, lat, new_zoom);
    let new_center_world = anchor_world - anchor_px + view.size * 0.5;
    // Do not clamp here: clamping after this cursor-anchor calculation moves
    // the location under the cursor, which is why zooming near Sweden's edge
    // felt like the map slid away. Panning still keeps the normal view bound.
    view.center = tiles::world_px_to_lonlat(new_center_world, new_zoom);
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
                let anchor = (normalized + Vec2::splat(0.5)) * view.size;
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
    let center = view.size * 0.5;
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
    // With a stationary map, every tile is already at the correct position.
    // Avoid issuing ECS layout updates for the full visible tile set every
    // frame; we only need to revisit it after a pan/zoom or a cache update
    // (for example, when a real tile replaces a placeholder).
    if !view.is_changed() && !cache.is_changed() {
        return;
    }
    let Ok(content) = content_q.single() else {
        return;
    };

    if spawned.zoom != Some(view.zoom) {
        for (_, tile) in spawned.entities.drain() {
            commands.entity(tile.entity).despawn();
        }
        cache.drop_other_zooms(view.zoom);
        spawned.zoom = Some(view.zoom);
    }

    let center_world = tiles::lonlat_to_world_px(view.center.0, view.center.1, view.zoom);
    let top_left_world = center_world - view.size * 0.5;
    let bottom_right_world = center_world + view.size * 0.5;

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
    fonts: Res<crate::UiFonts>,
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

    let p = theme.palette();
    let pixels: Vec<Vec2> =
        points.0.iter().map(|&(lat, lon)| lonlat_to_screen_px(lon, lat, &view)).collect();

    commands.entity(content).with_children(|parent| {
        // The outline the operator drew. Dimmer than the square derived from
        // it: it is the input, not the result.
        if pixels.len() >= 2 {
            let n = pixels.len();
            let edge_count = if pixels.len() >= 3 { n } else { n - 1 };
            for i in 0..edge_count {
                spawn_edge(
                    parent,
                    pixels[i],
                    pixels[(i + 1) % n],
                    p.accent.with_alpha(0.55),
                    EDGE_THICKNESS,
                );
            }
        }

        // The axis-aligned box that actually gets fetched — always a little
        // bigger than the rotated square. Faintest of the three, since the
        // operator never chose it directly.
        if net.valid {
            let fetch_px: Vec<Vec2> = net
                .fetch_corners
                .iter()
                .map(|&(lon, lat)| lonlat_to_screen_px(lon, lat, &view))
                .collect();
            for i in 0..4 {
                spawn_edge(parent, fetch_px[i], fetch_px[(i + 1) % 4], p.line, HAIRLINE);
            }
        }

        // The mesh area itself: the brightest mark on the map, and amber
        // because it is the operator's own selection.
        if net.valid {
            let square_color = if net.over_limit { p.danger } else { p.signal };
            let corners_px: Vec<Vec2> = net
                .corners
                .iter()
                .map(|&(lon, lat)| lonlat_to_screen_px(lon, lat, &view))
                .collect();
            for i in 0..4 {
                spawn_edge(parent, corners_px[i], corners_px[(i + 1) % 4], square_color, SQUARE_THICKNESS);
            }
            // Side length written on the map, so the number and the shape it
            // describes are read in one place.
            let label_at = (corners_px[0] + corners_px[1]) * 0.5;
            spawn_map_label(
                parent,
                label_at + Vec2::new(6.0, -14.0),
                &format!("{:.2} km", net.side_km),
                square_color,
                &fonts,
            );
        }

        // Ground station: a hollow square with a centre dot — a site marker,
        // not a dropped pin.
        if let Some((lat, lon)) = base.0 {
            let at: Vec2 = lonlat_to_screen_px(lon, lat, &view);
            const BASE_SIZE: f32 = 13.0;
            parent.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(at.x - BASE_SIZE * 0.5),
                    top: Val::Px(at.y - BASE_SIZE * 0.5),
                    width: Val::Px(BASE_SIZE),
                    height: Val::Px(BASE_SIZE),
                    border: UiRect::all(Val::Px(HAIRLINE)),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BorderColor::all(p.base),
                Pickable::IGNORE,
                PolygonVisual,
                ZIndex(1),
                children![(
                    Node { width: Val::Px(3.0), height: Val::Px(3.0), ..default() },
                    BackgroundColor(p.base),
                    Pickable::IGNORE,
                )],
            ));
            spawn_map_label(parent, at + Vec2::new(10.0, -6.0), "STATION", p.base, &fonts);
        }

        // Vertices: small squares, numbered to match the rail's vertex list.
        for (i, &at) in pixels.iter().enumerate() {
            parent.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(at.x - POINT_DOT_SIZE * 0.5),
                    top: Val::Px(at.y - POINT_DOT_SIZE * 0.5),
                    width: Val::Px(POINT_DOT_SIZE),
                    height: Val::Px(POINT_DOT_SIZE),
                    border: UiRect::all(Val::Px(HAIRLINE)),
                    ..default()
                },
                BackgroundColor(p.bg.with_alpha(0.85)),
                BorderColor::all(p.accent),
                Pickable::IGNORE,
                PolygonVisual,
                ZIndex(1),
            ));
            spawn_map_label(
                parent,
                at + Vec2::new(POINT_DOT_SIZE * 0.7, -6.0),
                &format!("{:02}", i + 1),
                p.accent,
                &fonts,
            );
        }
    });
}

/// A small mono caption pinned at `at`, over a dark plate so it stays legible
/// against whatever the map happens to be underneath it.
fn spawn_map_label(
    parent: &mut ChildSpawnerCommands,
    at: Vec2,
    label: &str,
    color: Color,
    fonts: &crate::UiFonts,
) {
    parent.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(at.x),
            top: Val::Px(at.y),
            padding: UiRect::axes(Val::Px(3.0), Val::Px(1.0)),
            ..default()
        },
        BackgroundColor(Color::BLACK.with_alpha(0.55)),
        Pickable::IGNORE,
        PolygonVisual,
        ZIndex(1),
        children![(
            Text::new(label.to_string()),
            mono(fonts, 10.0),
            TextColor(color),
            Pickable::IGNORE,
        )],
    ));
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

/// Rebuild the vertex list whenever the point list changes.
pub fn redraw_table(
    mut commands: Commands,
    points: Res<PendingPoints>,
    theme: Res<Theme>,
    fonts: Res<crate::UiFonts>,
    table_q: Query<(Entity, Option<&Children>), With<PointsTableRoot>>,
) {
    if !points.is_changed() && !theme.is_changed() {
        return;
    }

    let Ok((table_entity, children)) = table_q.single() else {
        return;
    };
    let p = theme.palette();

    // Keep the 4 header cells (first 4 children); drop the rest.
    if let Some(children) = children {
        for child in children.iter().skip(4) {
            commands.entity(child).despawn();
        }
    }

    commands.entity(table_entity).with_children(|grid| {
        for (i, &(lat, lon)) in points.0.iter().enumerate() {
            // The index is amber because it is also drawn on the map next to
            // the vertex it names — same colour, same number, one mark.
            grid.spawn((
                Text::new(format!("{:02}", i + 1)),
                mono(&fonts, 11.0),
                TextColor(p.signal),
                UiInk::new(Slot::Signal),
            ));
            for value in [lat, lon] {
                grid.spawn((
                    Text::new(format!("{value:.4}")),
                    mono(&fonts, 11.0),
                    TextColor(p.text),
                    UiInk::new(Slot::Text),
                ));
            }
            grid.spawn((
                Button,
                Node {
                    width: Val::Px(16.0),
                    height: Val::Px(16.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    border: UiRect::all(Val::Px(HAIRLINE)),
                    ..default()
                },
                BackgroundColor(Color::NONE),
                BorderColor::all(p.line),
                UiStroke::new(Slot::Line),
                RemovePointButton(i),
            ))
            .with_child((
                Text::new("\u{00D7}"),
                mono(&fonts, 10.0),
                TextColor(p.danger),
                UiInk::new(Slot::Danger),
                Pickable::IGNORE,
            ));
        }
    });
}

/// What the mission is currently waiting on. Exactly one of these is true at
/// any moment, and the title-strip chip shows which — so "why can't I press
/// Generate" is answerable without reading the rail.
#[derive(Clone, Copy, PartialEq, Eq)]
enum MissionState {
    NeedPoints,
    OverLimit,
    PlacingStation,
    NeedStation,
    Ready,
}

impl MissionState {
    fn current(net: &NetworkArea, mode: PickMode, base: &BasePosition) -> Self {
        if net.over_limit {
            Self::OverLimit
        } else if mode == PickMode::PlacingBase {
            Self::PlacingStation
        } else if !net.valid {
            Self::NeedPoints
        } else if base.0.is_none() {
            Self::NeedStation
        } else {
            Self::Ready
        }
    }

    fn chip(self) -> &'static str {
        match self {
            Self::NeedPoints => "NO AREA",
            Self::OverLimit => "OVER LIMIT",
            Self::PlacingStation => "PLACING STATION",
            Self::NeedStation => "NO STATION",
            Self::Ready => "READY",
        }
    }

    /// Cyan means the system is satisfied, amber means it wants something from
    /// the operator, red means a limit is broken.
    fn tone(self, p: &Palette) -> Color {
        match self {
            Self::OverLimit => p.danger,
            Self::Ready => p.accent,
            Self::NeedPoints => p.subtext,
            Self::PlacingStation | Self::NeedStation => p.signal,
        }
    }
}

/// Drives every live value on the rail plus the title-strip chip. Cheap enough
/// to run unconditionally rather than tracking a change signature across five
/// resources.
pub fn update_status_text(
    points: Res<PendingPoints>,
    net: Res<NetworkArea>,
    mode: Res<PickMode>,
    base: Res<BasePosition>,
    theme: Res<Theme>,
    mut fields: Query<(&RailField, &mut Text, &mut TextColor)>,
    mut chip_border: Query<&mut BorderColor, With<StatusChip>>,
    mut generate: Query<(&mut BackgroundColor, &mut BorderColor), (With<GenerateTerrain>, Without<StatusChip>)>,
) {
    let p = theme.palette();
    let state = MissionState::current(&net, *mode, &base);
    let dash = "\u{2014}";
    // The primary action is the only filled control on the screen, so it has
    // to go quiet the moment it can't be taken — otherwise it reads as the
    // obvious next step when it isn't.
    let ready = state == MissionState::Ready;

    for (field, mut text, mut color) in &mut fields {
        match field {
            RailField::Vertices => **text = format!("{}", points.0.len()),
            RailField::Side => {
                **text = if net.valid { format!("{:.2} km", net.side_km) } else { dash.into() }
            }
            RailField::Rotation => {
                **text =
                    if net.valid { format!("{:.0}\u{00B0}", net.rotation_deg) } else { dash.into() }
            }
            RailField::Fetch => {
                **text =
                    if net.valid { format!("{:.2} km", net.fetch_size_km) } else { dash.into() }
            }
            // The same count `world::setup` will actually spawn — the radio
            // spacing decides it, so the operator can see the cost of an extra
            // kilometre of area before committing to the fetch.
            RailField::Airframes => {
                **text = match airframes_for(&net) {
                    Some(count) => format!("{count}"),
                    None => dash.into(),
                }
            }
            RailField::Station => {
                **text = match base.0 {
                    Some((lat, lon)) => format!("{lat:.4} {lon:.4}"),
                    None => "NOT SET".into(),
                }
            }
            RailField::StatusChip => {
                **text = tracked(state.chip());
                color.0 = state.tone(&p);
            }
            RailField::Warning => {
                **text = match state {
                    MissionState::OverLimit => format!(
                        "Area is {:.1} km across \u{2014} over the {MAX_SIDE_KM:.0} km mesh limit. Move or remove vertices.",
                        net.side_km
                    ),
                    MissionState::PlacingStation => format!(
                        "Click within {MAX_BASE_DISTANCE_KM:.0} km of the area to place the ground station."
                    ),
                    MissionState::NeedStation => {
                        "Ground station required before terrain can be fetched.".into()
                    }
                    MissionState::NeedPoints | MissionState::Ready => String::new(),
                }
            }
            RailField::AddStopLabel => {
                **text = tracked(match *mode {
                    PickMode::Adding => "STOP ADDING",
                    PickMode::Reviewing | PickMode::PlacingBase => "ADD VERTICES",
                })
            }
            RailField::SetBaseLabel => {
                **text = tracked(match *mode {
                    PickMode::PlacingBase => "CANCEL",
                    _ if base.0.is_some() => "MOVE STATION",
                    _ => "SET LOCATION",
                })
            }
            RailField::GenerateLabel => color.0 = if ready { p.bg } else { p.subtext },
        }
    }

    if let Ok(mut border) = chip_border.single_mut() {
        border.set_all(state.tone(&p));
    }
    if let Ok((mut bg, mut border)) = generate.single_mut() {
        bg.0 = if ready { p.signal } else { Color::NONE };
        border.set_all(if ready { p.signal } else { p.line });
    }
}

/// How many airframes the current selection would need, or `None` if there is
/// no valid area yet. Mirrors `world::setup`: the patrol volume is the area
/// inset by the boundary margin, and the count falls out of the radio pitch.
fn airframes_for(net: &NetworkArea) -> Option<usize> {
    if !net.valid || net.over_limit {
        return None;
    }
    let volume = crate::navigation::PatrolVolume::inset(
        net.side_km as f32,
        crate::navigation::BOUNDARY_MARGIN_KM,
    );
    Some(crate::world::drones_required_for_coverage(&volume))
}

/// Live centre/zoom readout under the map. Separate from the rail's status
/// pass because it keys off `MapView`, which changes on every pan frame while
/// none of the mission state does.
pub fn update_map_readout(
    view: Res<MapView>,
    mut readout: Query<&mut Text, With<MapCoordReadout>>,
) {
    if !view.is_changed() {
        return;
    }
    let Ok(mut text) = readout.single_mut() else {
        return;
    };
    let (lon, lat) = view.center;
    let (ns, ew) = (if lat >= 0.0 { 'N' } else { 'S' }, if lon >= 0.0 { 'E' } else { 'W' });
    **text = format!(
        "CENTRE {:.4}{ns}  {:.4}{ew}   Z{:02}",
        lat.abs(),
        lon.abs(),
        view.zoom
    );
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

/// Redraw the vegetation controls whenever the settings change, whichever
/// widget did the changing.
pub fn refresh_vegetation_controls(
    vegetation: Res<VegetationSettings>,
    theme: Res<Theme>,
    mut toggle_labels: Query<(&mut Text, &mut TextColor), With<TreesToggleLabel>>,
    mut density_labels: Query<&mut Text, (With<DensityLabel>, Without<TreesToggleLabel>)>,
    mut toggles: Query<&mut BorderColor, With<TreesToggle>>,
    mut fills: Query<(&mut Node, &mut BackgroundColor), With<DensityFill>>,
) {
    if !vegetation.is_changed() && !theme.is_changed() {
        return;
    }
    let p = theme.palette();

    for (mut label, mut color) in &mut toggle_labels {
        **label = trees_text(&vegetation);
        color.0 = toggle_ink(vegetation.enabled, &p);
    }
    for mut label in &mut density_labels {
        **label = density_text(&vegetation);
    }
    // The toggle is an outline, not a fill: an amber border says "on" without
    // adding a second filled control to a screen that has exactly one.
    for mut border in &mut toggles {
        border.set_all(if vegetation.enabled { p.signal } else { p.line });
    }
    for (mut node, mut color) in &mut fills {
        node.width = Val::Percent(density_fraction(vegetation.density) * 100.0);
        // Dim the fill when the slider drives nothing.
        color.0 = if vegetation.enabled { p.signal } else { p.signal.with_alpha(0.22) };
    }
}

pub fn cleanup(mut commands: Commands, roots: Query<Entity, With<AreaSelectionRoot>>) {
    for entity in &roots {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<SpawnedTiles>();
}

fn trees_text(vegetation: &VegetationSettings) -> String {
    if vegetation.enabled {
        "CANOPY  \u{2022}  ON".into()
    } else {
        "CANOPY  \u{2022}  OFF \u{2014} CONTOURS".into()
    }
}

fn density_text(vegetation: &VegetationSettings) -> String {
    format!("{:.2}x", vegetation.density)
}

/// Slider value as a 0-1 position along its range.
fn density_fraction(density: f32) -> f32 {
    ((density - MIN_DENSITY) / (MAX_DENSITY - MIN_DENSITY)).clamp(0.0, 1.0)
}

fn toggle_ink(enabled: bool, p: &Palette) -> Color {
    if enabled { p.signal } else { p.subtext }
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

    #[test]
    fn rotated_selection_cannot_download_more_than_the_limit() {
        // A diamond whose minimum rotated square is approximately 20 km per
        // side. Its axis-aligned terrain request is about 28 km, and must be
        // rejected even though the rotated selection itself is within 20 km.
        let net = recompute_network_area(&[
            (59.115, 18.000),
            (59.000, 18.222),
            (58.885, 18.000),
            (59.000, 17.778),
        ]);
        assert!(net.valid);
        assert!(net.side_km <= MAX_SIDE_KM, "side={} fetch={}", net.side_km, net.fetch_size_km);
        assert!(net.fetch_size_km as f64 > MAX_SIDE_KM, "side={} fetch={}", net.side_km, net.fetch_size_km);
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
            let view = MapView { zoom, center: (lon + 0.4, lat - 0.3), ..default() };
            let screen = lonlat_to_screen_px(lon, lat, &view);
            let normalized = screen / view.size - Vec2::splat(0.5);
            let (back_lon, back_lat) = cursor_to_lonlat(normalized, &view);
            assert!((back_lon - lon).abs() < 1e-4, "lon {back_lon} vs {lon} at zoom {zoom}");
            assert!((back_lat - lat).abs() < 1e-4, "lat {back_lat} vs {lat} at zoom {zoom}");
        }
    }


    /// The mission boundary the base broadcasts is the hull of what was drawn,
    /// not the bounding square. It must enclose every clicked point, and be
    /// no bigger than it has to be — a square around a triangle is roughly
    /// twice the area nobody asked for.
    #[test]
    fn the_hull_is_the_drawn_shape_not_the_bounding_square() {
        // A right triangle: the bounding square covers twice its area.
        let (lat, lon): (f64, f64) = (63.1792, 14.6357);
        let d = 2.0 / 110.574;
        let points = vec![(lat, lon), (lat, lon + d), (lat + d, lon)];

        let net = recompute_network_area(&points);
        assert!(net.valid);
        assert_eq!(net.hull.len(), 3, "hull of a triangle is the triangle");

        // Every clicked point lies on the hull (in `(lon, lat)` order).
        for &(plat, plon) in &points {
            assert!(
                net.hull.iter().any(|&(hlon, hlat)| {
                    (hlon - plon).abs() < 1e-9 && (hlat - plat).abs() < 1e-9
                }),
                "clicked point ({plat}, {plon}) missing from the hull {:?}",
                net.hull
            );
        }
    }

    /// `polygon::unproject` returns `(lon, lat)`, so every consumer of
    /// `NetworkArea::corners` / `center` has to read it in that order. This
    /// pins the convention down: a small axis-aligned square in Sweden must
    /// come back with a fetch span close to its own side length, and a centre
    /// whose `.0` is the longitude.
    #[test]
    fn network_area_reports_lon_lat_and_a_sane_fetch_span() {
        // ~4 km square around Ostersund, given as (lat, lon) click points.
        let (lat, lon): (f64, f64) = (63.1792, 14.6357);
        let dlat = 2.0 / 110.574;
        let dlon = 2.0 / (111.320 * lat.to_radians().cos());
        let points = vec![
            (lat - dlat, lon - dlon),
            (lat - dlat, lon + dlon),
            (lat + dlat, lon + dlon),
            (lat + dlat, lon - dlon),
        ];

        let net = recompute_network_area(&points);
        assert!(net.valid);
        assert!((net.center.0 - lon).abs() < 1e-3, "center.0 should be lon, got {}", net.center.0);
        assert!((net.center.1 - lat).abs() < 1e-3, "center.1 should be lat, got {}", net.center.1);
        assert!(
            (net.side_km - 4.0).abs() < 0.2,
            "side_km {} should be about 4",
            net.side_km
        );
        // An axis-aligned square needs no bounding slack, so the fetch span
        // should match the side. A swapped lat/lon here inflates it by the
        // ratio of the two degree scales (~2.2x at this latitude).
        assert!(
            (net.fetch_size_km - 4.0).abs() < 0.3,
            "fetch_size_km {} should be about 4",
            net.fetch_size_km
        );
    }
    /// Zooming in/out around an anchor point must leave the lon/lat under
    /// that anchor unchanged — the actual fix for "zoom doesn't follow the
    /// cursor" / "goes through the map".
    ///
    /// The anchor is deliberately near the viewport centre. `rezoom_around`
    /// runs its new centre through `clamp_to_sweden`, so an anchor far enough
    /// out at a shallow zoom resolves to a lon/lat outside Sweden entirely and
    /// the clamp — correctly — moves it. That is the boundary being enforced,
    /// not the projection failing.
    #[test]
    fn rezoom_keeps_the_anchor_point_fixed() {
        let mut view = MapView { zoom: 6, center: (17.65, 62.15), ..default() };
        let anchor = view.size * 0.5 + Vec2::new(120.0, -90.0);
        let (lon_before, lat_before) =
            cursor_to_lonlat(anchor / view.size - Vec2::splat(0.5), &view);

        rezoom_around(&mut view, anchor, 10);

        let (lon_after, lat_after) =
            cursor_to_lonlat(anchor / view.size - Vec2::splat(0.5), &view);
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
            let view = MapView { zoom, center: (center_lon, center_lat), ..default() };
            let (lon, lat) = cursor_to_lonlat(Vec2::ZERO, &view); // dead center of the viewport
            let distance = net.distance_to_square_km(lat, lon);
            assert!(
                distance <= MAX_BASE_DISTANCE_KM,
                "clicking the area's own center at zoom {zoom} landed {distance:.4} km away — should be ~0"
            );
        }
    }
}
