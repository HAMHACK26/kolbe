mod antenna;
mod area;
mod base;
mod camera;
mod drone;
mod factories;
mod navigation;
mod networking;
mod polygon;
mod radar;
mod recovery;
mod seeking;
mod spherical;
mod sweden_geo;
mod terrain;
mod theme;
mod tiles;
mod tracking;
mod ui;
mod world;

use bevy::{asset::AssetId, picking::prelude::MeshPickingPlugin, prelude::*};

use camera::OrbitCamera;
use drone::SelectedDrone;
use theme::Theme;

/// Fira Sans (OFL-licensed, bundled from Bevy's own example assets) — has
/// full Latin/Scandinavian coverage. Bevy's *actual* built-in default font is
/// a stripped-down subset (`FiraMono-subset.ttf`, enabled by the
/// `default_font` feature) that's missing å/ä/ö, rendering them as a tofu
/// (□) box — every city label and the "Östersund"/"Åre"-style names in this
/// app hit that immediately. Overwriting the default font *asset*, rather
/// than touching every `TextFont { .. }` call site, fixes every one of them
/// at once — see `bevy_text::TextPlugin::build`, which does the exact same
/// `Assets<Font>::insert(AssetId::default(), ..)` trick to install its own.
const DEFAULT_UI_FONT: &[u8] = include_bytes!("../assets/fonts/FiraSans-Bold.ttf");

fn install_default_font(mut fonts: ResMut<Assets<Font>>) {
    let _ = fonts.insert(AssetId::<Font>::default(), Font::from_bytes(DEFAULT_UI_FONT.to_vec()));
}

#[derive(States, Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
enum AppState {
    #[default]
    AreaSelection,
    LoadingTerrain,
    Simulation,
}

/// Marks every top-level entity spawned while entering `Simulation`, so
/// leaving it (via the reset button) can tear the whole scene back down.
#[derive(Component)]
pub struct SimulationEntity;

/// Despawn the simulated world and reset the resources it accumulated, so
/// the next area selection starts from a clean slate.
fn teardown_simulation(
    mut commands: Commands,
    entities: Query<Entity, With<SimulationEntity>>,
    mut selected: ResMut<SelectedDrone>,
    mut table_open: ResMut<ui::NetworkTablePanelOpen>,
) {
    for entity in &entities {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<terrain::TerrainHeightMap>();
    selected.0 = None;
    table_open.0 = false;
    commands.insert_resource(OrbitCamera::default());
    commands.insert_resource(networking::Mailbox::default());
    commands.insert_resource(networking::ReconnectBus::default());
    commands.insert_resource(networking::ReconnectRequests::default());
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Drone Map — 20km × 20km".into(),
                resolution: (1100u32, 800u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(MeshPickingPlugin)
        .init_state::<AppState>()
        .init_resource::<area::ScenarioArea>()
        .init_resource::<terrain::VegetationSettings>()
        .insert_resource(SelectedDrone(None))
        .insert_resource(OrbitCamera::default())
        .insert_resource(Theme::default())
        // Seed the clear color from the palette so frame 0 matches; apply_theme
        // keeps it in sync afterwards.
        .insert_resource(ClearColor(Theme::default().palette().bg))
        .init_resource::<networking::Mailbox>()
        .init_resource::<networking::ReconnectBus>()
        .init_resource::<networking::ReconnectRequests>()
        .init_resource::<ui::NetworkTablePanelOpen>()
        .init_resource::<tiles::TileCache>()
        .add_systems(Startup, (install_default_font, ui::setup_camera, theme::setup_moon))
        .add_systems(OnEnter(AppState::AreaSelection), area::setup)
        .add_systems(
            Update,
            (
                area::add_point_on_click,
                area::place_base_on_click,
                area::point_table_and_buttons,
                area::pan_zoom,
                area::zoom_buttons,
                tiles::poll_tile_fetches,
                area::sync_map_tiles,
                area::recompute_area_on_change,
                area::redraw_polygon,
                area::redraw_table,
                area::update_status_text,
                area::trees_toggle_interactions,
                area::refresh_vegetation_controls,
                area::generate_terrain,
            )
                .chain()
                .run_if(in_state(AppState::AreaSelection)),
        )
        .add_systems(OnExit(AppState::AreaSelection), area::cleanup)
        .add_systems(OnEnter(AppState::LoadingTerrain), terrain::start_loading)
        .add_systems(
            Update,
            (terrain::poll_loading, terrain::update_progress)
                .run_if(in_state(AppState::LoadingTerrain)),
        )
        .add_systems(OnExit(AppState::LoadingTerrain), terrain::cleanup_loading)
        .add_systems(
            OnEnter(AppState::Simulation),
            (
                terrain::spawn_mesh,
                terrain::spawn_water,
                terrain::spawn_trees,
                world::setup,
                base::spawn_base,
                ui::make_camera_overlay,
                ui::spawn_reset_button,
            )
                .chain(),
        )
        .add_systems(OnExit(AppState::Simulation), teardown_simulation)
        .add_systems(Update, ui::reset_button_interactions.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, camera::orbit_camera.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, radar::sync_radar_visibility.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, ui::update_popup_position.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, world::draw_grid.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, terrain::draw_network_area.run_if(in_state(AppState::Simulation)))
        // Contours and trees are alternatives: a forest covers the ground the
        // contours describe, so only one of the two is drawn.
        .add_systems(
            Update,
            terrain::draw_contours.run_if(
                in_state(AppState::Simulation)
                    .and_then(|vegetation: Res<terrain::VegetationSettings>| !vegetation.enabled),
            ),
        )
        .add_systems(OnExit(AppState::Simulation), terrain::cleanup_trees)
        .add_systems(Update, theme::moon_toggle)
        .add_systems(Update, theme::apply_theme)
        .add_systems(Update, theme::apply_loading_theme)
        .add_systems(Update, factories::movement::apply_velocity.run_if(in_state(AppState::Simulation)))
        .add_systems(
            Update,
            networking::advance_clocks.run_if(in_state(AppState::Simulation)),
        )
        .add_systems(
            Update,
            (
                // Priority reconnection flood first, so a fresh slew-freeze is
                // visible to the aiming systems this same frame.
                networking::process_reconnect,
                // Antenna aiming (tracking::maintain_mesh_antennas,
                // seeking::seek_lost_links) is disabled for now — antennas and
                // radar cones stay at their spawn angles. Wiring live aiming
                // back in is a future PR.
                networking::detect_links_and_send_headers,
                networking::route_packets,
                // Partition detection + recovery run last — they need the
                // freshly (re)detected links and updated mesh table.
                recovery::detect_partitions,
                recovery::run_recovery,
            )
                .chain()
                .run_if(in_state(AppState::Simulation)),
        )
        .run();
}
